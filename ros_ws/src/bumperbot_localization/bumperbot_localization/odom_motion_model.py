#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, atan2, sqrt, fabs, pi
import random
import time

#Questa funziona ha il compito di calcolare la "distanza" minima fra i due angoli forniti
def angle_diff(a, b):
    a = atan2(sin(a), cos(a))
    b = atan2(sin(b), cos(b))
    d1 = a - b
    d2 = 2 * pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2


#Dall'odometria posso ricavare una posa precedente ed una attuale del robot. Posso ricavare anche il moto(decomposto non quello reale)
#che ha compito il robot per passare dalla possa precedente a quella attuale attraverso le misura date dall'odometria.
#Quindi ricavo il moto decomposto come un traslazione e 2 rotazione. Quindi se volessi ricavare la posa attuale ottengo esattamente
# quella data dall'odometria. 
# Tuttavia queste misure(odometria) sono affette da errore quindi, caratterizzo l'errore e lo sottraggo dal moto decomposto. 
# A questo punto ricavo la posa attuale (ovviamente col moto decomposto privo di errore) e ricavo una nuova stima della posa
# attuale (se hai caratterizzato bene il rumore che dipende dai sensori e dalla meccanica del robot sarà migliore)

#Lo stesso calcolo lo faccio 300 volte per ogni spostamento dove su ognuno di essi agisce un rumore diverso in modo da poter
# visualizzare su rviz l'incertezza del robot nel determinare la sua posizione.

#Ogni singloa particella sarà affetta da un errore diverso dalle altre perchè il rumore è, appunto, randomico, questo mi 
#permette di visualizzare l'incertezza del robot nel calcolare la sua posa e come vedrai tale incertezza cresce nel tempo

#In sostanza mi sottoscrivo ai messagi di odometria e calcolo l'dometry motion model affetto da un rumore gaussiano da questo,
# modello ho estratto un dato numero di particelle che rappresentano tutte le possibili posizioni del robot dopo un dato 
#movimento 

class OdomMotionModel(Node):
    def __init__(self):
        super().__init__("Odome_motion_model")
        #mi serve per discriminare la prima lettura dall'odom topic
        self.is_first_odom = True
        # Mi servono per imagazzinare valori precedenti di x y e theta.
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0

        #Ricorda che queste costanti variano da robot a robot e dipendono dalla meccanica e dai sensori utilizzati
        self.declare_parameter("alpha1", 0.1)
        self.declare_parameter("alpha2", 0.1)
        self.declare_parameter("alpha3", 0.1)
        self.declare_parameter("alpha4", 0.1)
        #Numero di particelle che vuoi generare per indicare le diverse stime della posa del robot
        self.declare_parameter("n_samples", 300)

        #Creo variabili che contengono il valore dei parametri settati dall'utente
        self.alpha1 = self.get_parameter("alpha1").get_parameter_value().double_value
        self.alpha2 = self.get_parameter("alpha2").get_parameter_value().double_value
        self.alpha3 = self.get_parameter("alpha3").get_parameter_value().double_value
        self.alpha4 = self.get_parameter("alpha4").get_parameter_value().double_value
        self.n_samples = self.get_parameter("n_samples").get_parameter_value().integer_value

        #Solo se il numero di campioni è valido inizializzo il messaggio che sara poi pubblicato
        if self.n_samples >= 0:
            #PoseArray contiene un header e Poses. Poses è dichiarato come un Array di Pose. Quindi importalo. All'interno di pose hai POint e Quaternion
            self.samples = PoseArray()
            self.samples.poses = [Pose()for _ in range(self.n_samples)] #Dichiaro un vettore di pose nullo per ogni elemento fino al range di numero campioni
        else:
            self.get_logger().fatal("Invalid number of samples: %d", self.n_samples)
            return
         
        #Mi iscrivo al topic dove il simple_controller pubblica odometria con rumore(Solo encoder niente IMU)
        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom", self.odomCallback, 10)
        #Pubblico in odom_kalman
        self.pose_pub_ = self.create_publisher(PoseArray, "odom_motion_model/samples", 10)
   
   
    #Qui applico l'algoritmo del FK: ogni volta che ricevo una trasformata con rumore
    def odomCallback(self, odom):
        #Odom pubblica l'orientamento con un quaternione(vai a vedere la def del messaggio) x y z w. Gli converto in angoli di eulero e quindi importa euler from quaternian
        q = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q) #Che poi tanto uso solo yaw

        #Qusta la esegui solo la prima volta quando ricevi un odom message
        if self.is_first_odom:
            self.last_odom_x = odom.pose.pose.position.x
            self.last_odom_y = odom.pose.pose.position.y
            self.last_odom_theta = yaw #Avrei dovuto fare odom.pose.pose.orientation pero contiene un quaternione che ho convertito in eulero
            
            self.samples.header.frame_id = odom.header.frame_id
            self.is_first_odom = False
            return
        #Calcolo x-x'
        odom_x_increment = odom.pose.pose.position.x - self.last_odom_x
        odom_y_increment = odom.pose.pose.position.y - self.last_odom_y
        odom_theta_increment = angle_diff(yaw, self.last_odom_theta)
        
        #Qui implemento le formule per il calcolo del movimento(2rotazioni e traslazione) affetto da errore (vedi appunti)
        #Mi assicuro che che ci sia una rotazione non piccola
        if sqrt(pow(odom_y_increment, 2) + pow(odom_x_increment, 2)) < 0.01:
            delta_rot1 = 0
        else:
            delta_rot1 = angle_diff(atan2(odom_y_increment, odom_x_increment), self.last_odom_theta)

        delta_trasl = sqrt(pow(odom_y_increment, 2) + pow(odom_x_increment, 2))

        delta_rot2 = angle_diff(odom_theta_increment, delta_rot1)

        #Caratterizzio il rumore come una variabile randomica gaussiana. esso agisce sui 3 componetni del moto "decomposto" (non reale)
        rot1_variance = self.alpha1 * delta_rot1 + self.alpha2 * delta_trasl
        trasl_variance = self.alpha3 * delta_trasl + self.alpha4 * (delta_rot1 + delta_rot2)
        rot2_variance = self.alpha1 * delta_rot2 + self.alpha2 * delta_trasl

        #Mi serve un seed per generare una variabile random. Uso il l'ora Attuale
        random.seed(int(time.time()))
        for sample in self.samples.poses:
            #Qui creo il rumore che affligge rot1 rot2 e trasl. Media nulla e varianza calcolata prima
            rot1_noise = random.gauss(0.0, rot1_variance)
            trasl_noise = random.gauss(0.0, trasl_variance)
            rot2_noise = random.gauss(0.0, rot2_variance)

            #Ora ricavo le vere misure di rotazione e traslazione non affette da rumore(Vdei formula appunti)
            delta_rot1_draw = angle_diff(delta_rot1, rot1_noise)
            delta_trasl_draw = delta_trasl - trasl_noise
            delta_rot2_draw = angle_diff(delta_rot2, rot2_noise)

            #Ora da questo spsotamento decomposto privo di errore ricavo il nuovo orientamneto (che si spera sia migliore di quello ricavato con semplice odometria)
            sample_q = [sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w]

            #Da ogni sample estraggo theta dal quaternione 
            sample_roll, sample_pitch, sample_yaw = euler_from_quaternion(sample_q)

            #Vedi gli appunti per le espressioni
            #Posizione attuale senza errore (Senza errore: dipende se lo hai caratterizzato bene)
            sample.position.x += delta_trasl_draw * cos(sample_yaw + delta_rot1_draw)
            sample.position.y += delta_trasl_draw * sin(sample_yaw + delta_rot1_draw)
            #Oreientamento attuale senza errore
            q = quaternion_from_euler(0.0 , 0.0, sample_yaw + delta_rot1_draw + delta_rot2_draw)
            sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w = q

        self.last_odom_x = odom.pose.pose.position.x
        self.last_odom_y = odom.pose.pose.position.y
        self.last_odom_theta = yaw
        self.pose_pub_.publish(self.samples)

            


def main():
    rclpy.init()
    node = OdomMotionModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

