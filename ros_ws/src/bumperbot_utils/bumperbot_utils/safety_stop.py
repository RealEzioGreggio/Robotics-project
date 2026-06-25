#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from enum import Enum
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from twist_mux_msgs.action import JoyTurbo
from visualization_msgs.msg import Marker, MarkerArray
import math

#Un enum per indicare diversi stati del robot
class State(Enum):
    FREE = 0 #No detection
    WARNING = 1
    DANGER = 2 #L'ostacolo si trova oltra danger_distance


class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop_node")

        self.declare_parameter("danger_distance", 0.2) #Distanza alla quale attivare il safety stop 20cm
        self.declare_parameter("warning_distance", 0.6)#Distanza alla quale riduco la distanza 60cm
        self.declare_parameter("scan_topic", "scan")#Topic del messaggio Scan
        self.declare_parameter("safety_stop_topic", "safety_stop") #Topic del safety stop(Quello che agisce sul mux)

        self.danger_distance = self.get_parameter("danger_distance").get_parameter_value().double_value
        self.warning_distance = self.get_parameter("warning_distance").get_parameter_value().double_value
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value
        self.is_first_msg = True

        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.laser_callback, 10)
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)
        self.zones_pub = self.create_publisher(MarkerArray, "zones", 10)

        #Creo dei aaction Client che richiederanno 2 action messe a disposizioni da twist_relay(Action Server)
        #Servono a regolare la velocita di riferimento del joystick(SOLO DAL JOYSTICK SE USI ALTRE FONTI RIMANE INVARIATA)
        self.decrease_speed_client = ActionClient(self, JoyTurbo, "joy_turbo_decrease")
        self.increase_speed_client = ActionClient(self, JoyTurbo, "joy_turbo_increase")

        #Se l'action non diventa disponibile E il nodo non crasha non continuo
        while not self.decrease_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().warn("Action /joy_turbo_decrease not available. Keep waiting...")

        while not self.increase_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().warn("Action /joy_turbo_increase not available. Keep waiting...")

        #Creo array di Marker
        self.zones = MarkerArray()
        #Creo la warning Zone marker
        warning_zone = Marker()
        warning_zone.id = 0
        warning_zone.action = Marker.ADD 
        warning_zone.type = Marker.CYLINDER
        warning_zone.scale.z = 0.001 #Lo rendo piatto
        warning_zone.scale.x = self.warning_distance *2 #Gli impongo lo stesso diametro della mia warning zone
        warning_zone.scale.y = self.warning_distance *2 #Stessa cosa per y senno ottieni una linea
        warning_zone.color.r = 1.0
        warning_zone.color.g = 0.984
        warning_zone.color.b = 0.0
        warning_zone.color.a = 0.5 #Trasparenza senno non lo vedi
        #creo la danger_zone marker
        danger_zone = Marker()
        danger_zone.id = 1
        danger_zone.action = Marker.ADD 
        danger_zone.type = Marker.CYLINDER
        danger_zone.scale.z = 0.001 #Lo rendo piatto
        danger_zone.scale.x = self.danger_distance *2 #Gli impongo lo stesso diametro della mia warning zone
        danger_zone.scale.y = self.danger_distance *2 #Stessa cosa per y senno ottieni una linea
        danger_zone.color.r = 1.0
        danger_zone.color.g = 0.0
        danger_zone.color.b = 0.0
        danger_zone.color.a = 0.5
        #Sollevo leggermente uno dall'altro per evitare che si sovrappongano
        danger_zone.pose.position.z = 0.01 
        #Popolo L'array di marker
        self.zones.markers = [warning_zone, danger_zone]

        #All'inizio pongo lo stato del robot in free
        self.state = State.FREE
        self.prev_state = State.FREE


    def laser_callback(self, msg: LaserScan):

        self.state = State.FREE #resetto lo stato del robot
        #Vai a vedere laser Scan, contiene un elemetno range_value, che contiene un vettore di distanze in m
        for range_value in msg.ranges:
            #Volgio scartare tutte le letture che non vengono riflesse da nessun oggetto: danno infinito
            #Se il range value è un valore valido(non inf) e la distanza è <= warning_distance vai in WARNING 
            if not math.isinf(range_value) and range_value <= self.warning_distance:
                self.state = State.WARNING
                #Se oltre ad essere <= di warning_distance è anche <= danger_distance vai in WARNING
                if range_value <= self.danger_distance:
                    self.state = State.DANGER
                    break

        #La logica parte solo se il robot varia stato. In questo modo è molto comodo perchè come una macchina a stati finiti
        #devo gestire il comportamento in base alla sola variabile state.
        if self.state != self.prev_state:
            is_safety_stop = Bool()
            #Se sei in warning
            if self.state == State.WARNING:
                
                is_safety_stop.data = False #Lascialo false in modo da non fermarti
                self.decrease_speed_client.send_goal_async(JoyTurbo.Goal())#Questa action vogliono argomenti vuoti
                self.zones.markers[0].color.a = 1.0 #Cambio la trasparenza in modo da visualizzare solo la warning zone
                self.zones.markers[1].color.a = 0.5
            #In questo caso sono in Danger e devo fermare il robot
            elif self.state == State.DANGER:
                is_safety_stop.data = True
                self.zones.markers[0].color.a = 1.0 
                self.zones.markers[1].color.a = 1.0
            #In questo caso sono tornato in FREE e quindi posso aumentare da capo la velocità
            elif self.state == State.FREE:
                is_safety_stop.data = False
                self.increase_speed_client.send_goal_async(JoyTurbo.Goal())
                self.zones.markers[0].color.a = 0.5 
                self.zones.markers[1].color.a = 0.5

            self.prev_state = self.state
            self.safety_stop_pub.publish(is_safety_stop)

        #Ora volgio inserire lo stesso frame_id ai marker di quello contenuto in LaserScan
        #Lo faccio solo alla prima lettura poi non è piu necessario
        if self.is_first_msg:
            #Vai sempre a vedere come sono definiti i messaggi
            for zone in self.zones.markers:
                zone.header.frame_id = msg.header.frame_id
            
            self.is_first_msg = False
        
        self.zones_pub.publish(self.zones)


def main():
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy.node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


