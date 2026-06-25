#!/usr/bin/env python3 
# #Specifico il build type che ho scelto senno il Cmake non lo riconosce Questa cosa scritta sopra si chiama shebang
#non puoi scriverci niente neanche i commenti. Il pacchetto è stato cotruito con ament_cmake ma il nodo è in python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, TransformStamped
import numpy as np # Usata per fare roba con matrici
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS #Questa è una macro che ti permette di convertire i secondi in Nannosecodni
import math #Per la trigonometria(mi serve sin e cos)
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler #Ti permette di passare a quaternione da eulero
from tf2_ros import TransformBroadcaster


#Con questo nodo voglio dare dei riferimenti di velocita al robot rispetto al sistema di riferimento mobile, attraverso un 
#controller della xbox o PS

#Questo nodo si sottoscrive ad un topi che creo. In Questo topic saro io a pubblicare (da terminale o con un controller xbox)
#e ogni volta che ricevo un messaggio su questo topic prendo i valori gli converto in velocita ai giunit e pubblico in
# un formato complensibile per il controllore di ros_control

#Ho aggiunto anche l'odometria: mi sottoscrivo a /joint_states ros_control pubblica la posizone dei giunti(Nel caso di simulazione)
# Leggo il messaggio e applico le formula degli appunti

#Pubblico le trasformate fra i 2 frame 

class SimpleController(Node):
    
    def __init__(self):
        super().__init__("simple_controller")

        #Inserisco i parametri del robot. Quindi quando lo vuoi lanciare devi inserire anche questi parametri se non lo fai
        #valgono il valore di default specificato qui. In realta questo nodo sara lanciato dallo stesso launch file che lancia
        #la simulaizone di gz con ros_control e i suoi nodi. 
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius: %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation: %f" % self.wheel_separation_)

        #Roba che mi serve per ricavare la velocita
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()
        #Roba che mi serve per l'odometria
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        #Qui creo il publisher che pubblica nel topic giusto(vedi appunti)
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        #Creo il subsriber al topic che usero per comunicare a questo nodo le velocita linearie angolari desiderate.
        self.vel_sub_ = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", self.velCallback, 10)
        #Ok per l'odometri mi servono i dati dal sensore che non esiste quindi mi prendo la posizione da joint_states
        #Quello instanziato da ros_control. I messaggi sono pubblicati con la stessa interfaccia dei sensori, per utilizzarla
        #importa la libreria
        self.joint_sub = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        #Voglio pubblicare l'odometria. Uso il msg standarn del navigation pack
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom", 10)


        #Implemento la cinematica inversa(dalla velocita del robot assegnata, voglio ricavare la velocita dei giunti che 
        # mi consente di ottenerla)
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                            [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])

        #I nomi dei frame non devono essere per forza quelli del robot(URDF) non sono proprio collegati.
        #Quello che fai è definire i nomi di questi frame, poi la posa di una con l'altro sara data dalla lettura di 
        #joint_states. Vale sia per l'odometria che per le trasformate
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom" #Sistema di riferimento fisso (punto di spawn del robot)
        self.odom_msg_.child_frame_id = "base_footprint" #Sistema di riferimento mobile(robot)
        #Normalizzo il quaternione. Per l'orientamento accetta il quaternione in realta puoi inserire anche i rad
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0
        #Instanzio l'ogetto per pubblicare una trasformata
        self.br_= TransformBroadcaster(self)
        #Creo l'ogetto trasformata da pubblicare. Trasformata dinamica
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom" #Sistema di riferimento fisso (punto spawn)
        self.transform_stamped_.child_frame_id = "base_footprint" #Sistema di riferimento mobile(robot)


        self.get_logger().info("The conversion matrix is: %s" %self.speed_conversion_)



    def velCallback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x],
                               [msg.twist.angular.z]])
        
        #Qui risolvo il problema inverso(Vedi dagli appunti inverti la formula e questo è come la si scrive in py con math)
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)

        #Creo il msg da pubblicare
        wheel_speed_msg = Float64MultiArray()
        #Scrivi il messaggio esattamente come si aspetta di riceverlo (Esattamente come quando li lanciavo da terminale con 
        # ros2 pub simple_velocity_controller/commands ...)
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)


    #Qui faccio la magia dell'odometria voglio ricavare la variazione della velocita della ruota destra e sinistra [VEDI APPUNTI]
    # Nel messaggio hai una lista di posizioni. Ho aggiunto anche la pubblicazione della trasformata
    #Questo viene eseguito ogni volta che ricevi un messaggio in /joint_states
    def jointCallback(self, msg):
        dp_left = msg.position[1] - self.left_wheel_prev_pos_ #Variaz posizione ruota sx
        dp_right = msg.position[0] - self.right_wheel_prev_pos_ #Variaz posizione ruota dx
        #Importa la libreria per indicare un tempo?? e converti il messaggio che contiene il tempo in tempo :)
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        #dt è un oggetto di Time quindi puoi usare nanoseconds e te lo fai restituire in secodni poi con la macro dividendo
        #per 1ns lo converti in secondi. Roba molto strana sta accadendo
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        #Segui la formula dagli appunti=> Vel. lin e ang. nel sistema di riferimento mobile
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_
        #Posizione e orientamento del sistema di riferimento mobile rispetto al sistema di riferimento fisso
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
        #Aggiorno i parametri 
        self.theta_ += d_theta
        #Per la posizione ho d_s che è un vettore quindi elaboro separatamente x e y con la trigonometri(importa la libreria)
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        #Qui dati i risultati di posizione e velocita ricavati prima crei le trasformate (i TF) come vedi le trasformate del
        #robot(urdf) non centrano niente (quindi i nomi dei frame in questo file non devono corrispondere a quelli dell'URDF)
        #LA POSA RISPETTO AL FRAME PADRE LA CREI ATTRAVERSO POSIZONE E ORIENTAMENTO RICAVATI DALLE FORMULE
        #In pose metti la posizione e oreintamento del sistema di riferimento mobile rispetto a quello fisso
        #In twist metti la vel lineare e angolare del robot rispetto al sistema di riferimento mobile.
        #Convertol'orientamento in quaternione per poterlo inserire nel messaggio odom_msg_
        #In pose metti la posizione e oreintamento del sistema di riferimento mobile rispetto a quello fisso
        #In twist metti la vel lineare e angolare del robot rispetto al sistema di riferimento mobile.
        #Convertol'orientamento in quaternione per poterlo inserire nel messaggio odom_msg_
        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        #Aggiorno l'orario in cui questa informaizone viene aggiunta
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        #Aggiorno la posizione
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        #Aggiungo anche la vel. lineare angolare su z(eul)
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        #Aggiungo nel messaggio della trasformata la posa del sistema di riferimento mobile rispetto a quello fisso
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]

        #Nota come il msg trasformata e odometria contengono le stesse informazioni ma siano messaggi diversi. Questo
        # è necessario perchè transform_stamped puo essere interpretato da Rviz2 che ti da una interfaccia grafica.
        
        
        #Aggiorno l'ora in cui è stata generata
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        
        #Pubblico l'odometria
        self.odom_pub_.publish(self.odom_msg_)
        
        #Pubblico la trasformata in /tf quindi se avvii Rviz2 le puoi gia visualizzare
        self.br_.sendTransform(self.transform_stamped_)



        #
        # self.get_logger().info("Liner Vel: %f, Angular Vel: %f" % (linear, angular))
        # self.get_logger().info("x: %f, y: %f, theta: %f" % (self.x_, self.y_, self.theta_))
        #

def main():
    rclpy.init()
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#Ricorda di modificare il package aggingendo le dipendenze dai pacchetti ma anche dal build type: <buildtool_depend>ament_cmake_python</buildtool_depend>
#Modifica il Cmake con:find_package(ament_cmake_python REQUIRED)
# find_package(rclpy REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# ament_python_install_package($(PROJECT_NAME))
#install(PROGRAMS
# ${PROJECT_NAME}/simple_controller.py
# DESTINATION lib/${PROJECT_NAME}
