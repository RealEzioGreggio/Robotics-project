#!/usr/bin/env python3 
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler


#AVVIA LA CHIAMATA AL SERVER SOLO SE STA FERMO.

#Creo una classe Pose il suo costruttore vuole px e py.
#Questa Pose esprime la posizone del robot nella matirce della mappa quindi la posizione sara si una cella ma espressa con due indici x e y appunto
class Pose:
    def __init__(self, px = 0, py = 0):
        self.x = px 
        self.y = py

def poseOnMap(pose: Pose, map_info: MapMetaData):
    #Sto confrontando pose.x e pose.y che sono delle celle con map_info.width e map_info.height che se ti ricordi sono espresse come celle e non metri
    return pose.x < map_info.width and pose.x >= 0 and pose.y < map_info.height and pose.y >= 0

def poseToCell(pose: Pose, map_info: MapMetaData):
    return map_info.width * pose.y + pose.x

#Questa funzione deve prendere px e py e convertirli in coordinate sulla mappa
def coordinatesToPose(px, py, map_info: MapMetaData):
    pose = Pose()
    #Questa è la posizone del robot ricavata rispetto alla mappa quindi è lettralemnte la cella partendo dall'angolo in alto a destra della mappa
    pose.x = round((px - map_info.origin.position.x) / map_info.resolution)
    pose.y = round((py - map_info.origin.position.y) / map_info.resolution)
    return pose

#Dati px e py
def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def cluster_points(new_obstacles):

    clusters = []

    threshold = 0.5
    #Obstacles contiene (px, py)
    for p in new_obstacles:

        assigned = False

        for cluster in clusters:

            if distance(p, cluster[-1]) < threshold:
                cluster.append(p)
                assigned = True
                break

        if not assigned:
            clusters.append([p])

    return clusters

    


class ObjDetector(Node):
    def __init__(self, name):
        super().__init__(name)

        self.new_obstacles = []
        self.map_flag = False
        self.map = OccupancyGrid()
        self.last_goal = None

        qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE
        )

        #Inizializzo il ros pub, sub e un timer ad ogni sec.
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

         #Server client verso /navigate_to_pose
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for NavigateToPose server...")

        
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def scan_callback(self, scan: LaserScan):

        if not self.map_flag:
            return
        #Imagazzino la trasformata fra odom frame(map.header.frame_id) e il frame del laser in una variabile t.
        #Potrebbe dare errore nel caso non sia possibile trovare una trasformata
        try:
            t = self.tf_buffer.lookup_transform(
                    self.map.header.frame_id,
                    scan.header.frame_id,
                    rclpy.time.Time()   # latest available
                )
        except LookupException:
            self.get_logger().error("Unable to trasform between /odom e /base_footprint")
            return
        
        #Devo convertire t in coordinate della occupancy grid, quindi in coordinate espresse rispetto alla mappa
        #La chiamo robot_p ma è la posizione del sensore scanner
        robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map.info)
        
        
        
        #Controllo che il robot non sia fuori dalla mappa
        if not poseOnMap(robot_p, self.map.info):
            self.get_logger().info("Robot out of the map")
            return
        
        #Trasformo il quaternione in angoli di eulero mi serve solo yaw
        (roll, pitch, yaw) = euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
        
        #Itero il vettore contenente le distanze
        for i  in range(len(scan.ranges)):
            #Mi assicuro che non ci siano letture di tipo Inf. Ho deciso di scartare anche tutte le misure che superano i 2 metri(cerco di ridurre il rumore)
            if math.isinf(scan.ranges[i]) or scan.ranges[i] > 3.0:
                continue #Skippa

            #Vedi appunti: Ricavo l'orientamneto dell'ostacolo detectato rispetto al sensore
            angle = scan.angle_min + (i * scan.angle_increment) + yaw

            #Ora ho distanza dell'iesimo ostacolo rispetto al frame dello scanner e l'orientamento di tale ostacolo rispetto al frame della mappa. Gli riporto 
            # in coordinate cartesiane (sono espressi in coordinate polari)
            px = scan.ranges[i] * math.cos(angle)
            py = scan.ranges[i] * math.sin(angle)

            #Ora esprimo anche la posizione rispetto al frame della mappa.
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            px += self.robot_x #Queste è la posa dell'ostacolo rispetto al frame della mappa in coordiante cartesiane
            py += self.robot_y #Ora converto queste coordinate cartesiane in cella della mappa e dopidiche converto la posizione di tale cella 
                                            #sulla mappa nella posizone del vettore che conserva la mappa in memoria 
            
            #Converto le coordinate cartesiane della posa dell'ostacolo in una cella della matrice della mappa
            beam_p = coordinatesToPose(px, py, self.map.info)
            #Controllo che l'ostacolo sia all'interno della mappa
            if not poseOnMap(beam_p, self.map.info):
                continue #Skippa
            
            #L'idea è quella di prendere una zona attorno al punto dell'ostacolo e valutarla => elimina quasi tutti i falsi positivi 
            found = False
            #Ciclo attorno all'ostacolo(la mappa ha celle di 5cm x 5cm. In questo modo cerco in un area di circa 20cm^2 attorno al punto segnato dal laser)
            for dx in range(-4,5):
                for dy in range(-4,5):
                    #Mi prendo la posa dell'attuale ciclo attorno all'ostacolo
                    p = Pose(beam_p.x + dx, beam_p.y + dy)
                    #Se è fori dalla mappa scarta
                    if not poseOnMap(p, self.map.info):
                        continue
                    #Converto in una cella del VETTORE della mappa l'attuale posizione dell'intorno dell'ostacolo
                    cell = poseToCell(p, self.map.info)
                    #Se il valore è >50 significa che nei dintorni c'è un ostacolo segnato sulla mappa quindi scarto il punto trovato dal laser
                    if self.map.data[cell] > 50:
                        found = True
                        break
                if found:
                    break
            #Se non è stato trovato nessun ostacolo segnato sulla mappa nei dintorni del punto segnato dal laser allora tale punto è un punto valido 
            if not found:
                self.new_obstacles.append((px,py))

            
    def map_callback(self, map: OccupancyGrid):
        self.get_logger().info("Mappa ricevuta!")
        self.map = map
        self.map_flag = True

               

    def timer_callback(self):
        self.get_logger().info(
        f"Punti nuovi trovati: {len(self.new_obstacles)}"
)       
        #CLUSTERING dei punti = Se i punti sono vicini fra di loro di threshold allora sono un unico oggetto e allora restituisce una sola posa
        if len(self.new_obstacles) == 0:
            return
        #cluster
        clusters = cluster_points(self.new_obstacles)
        #Se vuoto pulisci il vettore
        if len(clusters) == 0:
            self.new_obstacles.clear()
            return
        #Di tutti i cluster creati mi prendo quello contenente piu punti (maggiore probabilita che sia l'oggetto nuovo nella mappa)
        best_cluster = max(clusters, key=len)
        #se il cluster con maggiori punti ne ha comunque pochi scarta
        if len(best_cluster) < 5:
            self.new_obstacles.clear()
            return

        rx = self.robot_x
        ry = self.robot_y

        cx = sum(p[0] for p in best_cluster) / len(best_cluster)
        cy = sum(p[1] for p in best_cluster) / len(best_cluster)
        self.get_logger().info(
            f"Nuovo ostacolo in ({cx:.2f}, {cy:.2f})"
        )

        #Dato che cx e cy rappresentano il centroide del cluster designato molto probabilmente si trova all'interno del'oggetto da segurie.
        #Quindi creo un offset: Trovo la distanza robot-centroide impongo un offset di distanza [m]e lo do in pasto al server
        dx = cx - rx
        dy = cy - ry

        dist = math.sqrt(dx*dx + dy*dy)

        offset = 0.25

        goal_x = cx - offset * dx / dist
        goal_y = cy - offset * dy / dist

        #Calcolo l'angolo per orientarlo verso l'oggetto da seguire
        yaw = math.atan2(cy - goal_y, cx - goal_x)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

        #è un normalissimo PoseStamped
        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        #ho impostato il bt_navigator per usare un bt di default
        goal.behavior_tree = ""
        
        if self.last_goal is not None:
            #Se l'oggetto si è spostato di poco < 30cm non fare nada
            if distance((goal_x, goal_y), self.last_goal) < 0.3:
                self.new_obstacles.clear()
                return
        self.nav_to_pose_client.send_goal_async(goal)
        self.last_goal = (goal_x, goal_y)
       
       

def main():
    rclpy.init()
    node = ObjDetector("obj_detector")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
