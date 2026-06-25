#!/usr/bin/env python3 
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException
from tf_transformations import euler_from_quaternion

#Probabilita di base assegnata ad ogni cella della mappa: 0.5 perfettamnte ignoto
PRIOR_PROB = 0.5
#Se il laser colpisce un ostacolo assegnera a tale cella questa probabilita (90% di prob di essere occupata)
OCCUPANCY_PROB = 0.9
#Prob assegnata se in una cella il sensore non detecta l'ostacolo: (35% di prob di trovare un ostacolo)
FREE_PROB = 0.35


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

#Prende in input due pose: i due vertici della linea che vuoi disegnare e ti disegna una linea dra questi due punti. 
def bresenham(start: Pose, end: Pose):
    line = []

    dx = end.x - start.x
    dy = end.y - start.y

    xsign= 1 if dx > 0 else -1
    ysign= 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx = xsign
        xy = 0
        yx = 0
        yy = ysign
    else:
        tmp = dx 
        dx = dy
        dy = tmp
        xx = 0
        xy = ysign
        yx = xsign
        yy = 0
    
    D = 2 * dy - dx
    y = 0

    for i in range(dx + 1):
        line.append(Pose(start.x + i * xx + y * yx, start.y + i * xy + y * yy))
        if D >= 0:
            y += 1
            D -= 2 * dx
        D += 2 * dy

    return line

#Questa funzione prende in ingresso la posa del robot e la posa dell'ostacolo detectato dal laser. La cella su cio si trova l'osacolo la volgio
#marcare come occupata e tutte le celle fra il robot e l'ostacolo le voglio marcare libere
def inverseSensorModel(p_robot: Pose, p_beam: Pose):
    
    occ_values = []
    #Traccio una linea(vettore di pose) che inizia dalla posa del robot e finisce nell'ostacolo. L'ultima cella di questo vettore è dove si trova
    #l'ostacolo quindi la devo marcare come occupata
    line = bresenham(p_robot, p_beam)
    #Ciclo il vettore delle pose: line
    for pose in line[:-1]:
        #Marco tutte le celle libere
        occ_values.append((pose, FREE_PROB))
        #Marco l'ultima cella come occupata
    occ_values.append((line[-1], OCCUPANCY_PROB))
    return occ_values #Ritorno il vettore che contiene la singola scansione laser. Fino all'oggetto hai pose libere l'ultima(l'ostacolo) sarà ovviamente occupata


#Funzione che prende in ingresso una probabilita (valore da 0 a 1) e lo converte in log con logodds
def prob2logodds(p):
    return math.log(p / (1 - p))

def logodds2prob(l):
    try:
        return 1 - (1 / (1 + math.exp(l)))
    #Dato che puoi risultare in un risultato <0 e >1 :
    except OverflowError:
        return 1.0 if l > 0 else 0.0

class MappingWPose(Node):
    def __init__(self, name):
        super().__init__(name)

        #Voglio una occupancy grid di 50x50 m^2 con una risoluzione di 10cm
        self.declare_parameter("width", 50.0)
        self.declare_parameter("height", 50.0)
        self.declare_parameter("resolution", 0.1)

        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        resolution = self.get_parameter("resolution").value

        #Occupancy grid contiene sia ul vettore delle celle dia dei metadata per interpretarlo(risoluzione e dimensioni)
        self.map = OccupancyGrid()
        #In .info hai un messaggio di tipo mapMetadata
        self.map.info.resolution = resolution
        #Nella definizione del messaggio dice chiaramente che questi valori devono essere salvati non in metri ma in celle:
        #Devi esprimere l'altezza e lunghezza in cella quindi dividi con resolution
        self.map.info.width = round(width / resolution) #Approssimo ad un intero
        self.map.info.height = round(height / resolution)
        #Ora inserisco il sistema di riverimento globale della mappa
        #Tieni a mente che (0, 0) corrisponde all'angolo superiore destro. Lo volgio posizionare al centro della mappa che è di 50x50 m^2
        #Il - vedi gli appunti per capire perche
        self.map.info.origin.position.x = float(-round(width / 2.0))
        self.map.info.origin.position.y = float(-round(height / 2.0))
        #Aggiungo anche il reference frame rispetto alla quale vai a pubblicare le coordinate della mappa
        #Se metto odom è come se dicessi di utilizzare l'odometria per localizzare il robot e creare la mappa
        self.map.header.frame_id = "odom"
        #Qui ottieni i dati della mappa è un vettore molto lungo. Setto tutte le cello in stato sconosciuto
        self.map.data = [-1] * (self.map.info.width * self.map.info.height) #Prendo width e heught espressi  come celle 

        #Inizializzo la mappa delle probabilita: Prendo il matrix della mappa(tutte le celle della mappa) e le moltiplico per la prior prob. convertita in log
        self.probability_map = [prob2logodds(PRIOR_PROB)] * (self.map.info.width * self.map.info.height)

        #Inizializzo il ros pub, sub e un timer ad ogni sec.
        self.map_pub = self.create_publisher(OccupancyGrid, "map", 1)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def scan_callback(self, scan: LaserScan):
        #Imagazzino la trasformata fra odom frame(map.header.frame_id) e il frame del laser in una variabile t.
        #Potrebbe dare errore nel caso non sia possibile trovare una trasformata
        try:
            t = self.tf_buffer.lookup_transform(self.map.header.frame_id, scan.header.frame_id, rclpy.time.Time())
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
            #Mi assicuro che non ci siano letture di tipo Inf
            if math.isinf(scan.ranges[i]):
                continue #Skippa
            
            

            #Vedi appunti: Ricavo l'orientamneto dell'ostacolo detectato rispetto al frame della mappa
            angle = scan.angle_min + (i * scan.angle_increment) + yaw

            #Ora ho distanza dell'iesimo ostacolo rispetto al frame dello scanner e l'orientamento di tale ostacolo rispetto al frame della mappa. Gli riporto 
            # in coordinate cartesiane (sono espressi in coordinate polari)
            px = scan.ranges[i] * math.cos(angle)
            py = scan.ranges[i] * math.sin(angle)

            #Ora esprimo anche la posizione rispetto al frame della mappa.
            px += t.transform.translation.x #Queste è la posa dell'ostacolo rispetto al frame della mappa in coordiante cartesiane
            py += t.transform.translation.y #Ora converto queste coordinate cartesiane in cella della mappa e dopidiche converto la posizione di tale cella 
                                            #sulla mappa nella posizone del vettore che conserva la mappa in memoria 
            
            #Converto le coordinate cartesiane della posa dell'ostacolo in una cella della matrice della mappa
            beam_p = coordinatesToPose(px, py, self.map.info)
            #Controllo che l'ostacolo sia all'interno della mappa
            if not poseOnMap(beam_p, self.map.info):
                continue #Skippa
            
            #Quindi con la posa del sensore e dell'ostacolo (rispetto alla mappa e nelle matrice della mappa) ricavo il vettore delle 
            # posizioni libere fino all'ostacolo(sono tante pose fino all'ostacolo e ad ogni posa ho attribuito un valore per indicare se è occupata o libera) 
            poses = inverseSensorModel(robot_p, beam_p)

            #Per ogni posa e valore in poses
            for pose, value in poses:
                #Converto la cella della matrice della mappa in un indce del vettore che rappresenta la mappa in memoria
                cell = poseToCell(pose, self.map.info)
                #Implemento l'eq dagli appunti. RIcorda che prob._map è solo una fake simil della mappa. devi aggiornare la map.data 
                #Per questo motivo la converto direttamente in un Occupancy Grid (vedi timer callback)
                self.probability_map[cell] += prob2logodds(value) - prob2logodds(PRIOR_PROB)
                #self.map.data[cell] = value

            
            #Segno finalemte l'ostacolo sulla mappa andando a segnalarlo nel vettore della mappa 
            


    def timer_callback(self):
        self.map.header.stamp = self.get_clock().now().to_msg()
        #Quindi aggiorno data della mappa con la probabilita della probab.map la devo solo riconvertire in probabilita e moltiplicare con 100
        #Converti in int che senno si mette a piangere
        self.map.data = [int(logodds2prob(value) * 100) for value in self.probability_map]
        self.map_pub.publish(self.map)


def main():
    rclpy.init()
    node = MappingWPose("mapping_w_pose")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
