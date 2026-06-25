#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

#Questo codice implementa il teorema di bayes. Quando ricevo la prima trasformata ne approfitto solo per ricavare la 
# velocita dall'encoder. l'uscita dell'encoder sarà la media della gaussiana a cui associo una varianza elevata questa sarà
# la prior belief.
# dal momento in cui ricevo la seconda trasformata si susseguono 2 fasi: measurementUpdate e statePredicion

# La state prediction ricavo la nuova velocita del robot dall'encoder e faccio la differenza con quella precedente. Con questo
# nuovo valore calcolo la gaussiana: il valore ricavato è la media e ci attribuisco una varianza. Dopodiche sommo le
# 2 gaussiane e ricavo il nuovo stato di velocita. Letteralmente l'idea è semplicemente quella di aggiornare lo stato vecchio
# con quello nuovo quindi fai la somma ma ti porti dietro anche gli errori di misura. Questa è la nuova prior belief#
# 
# La measurement update moltiplica la prior belief con la measurament update, essa è un'altra guassiana costruita
# imponendo l'uscita dell'Imu come media e associando una varianza bassa. Moltiplicando queste 2 gaussiane moltiplico
# la probabilità di un evento A (vel del robot data dall'encoder) con la probabilita di un evento B(vel del robot data
# dall'Imu) given A(una volta che l'evento A si è gia manifestato), questo mi consente di ricavare la probabilita di A aggiornataa
# ossia ricavo la probabilita di A given B => stando a bayes devi dividere con la marginal probability che considero 1
#In poche parole ricavo una stima della veolcita attuale migliore con il teorema di bayes. Questa è la nuova prior belief.
# Il ciclo si ripete all'infinito#

#Puoi pensare alla varianza anche come dei pesi


class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")

        #Mi iscrivo al topic dove il simple_controller pubblica odometria con rumore
        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom_noisy", self.odomCallback, 10)
        #Mi inscrivo al topic dove l'imu simulato(ignition) pubblica i dati in ros (tramite bridge) 
        self.imu_sub_ = self.create_subscription(Imu, "/imu/ros", self.imuCallback, 10)
        #Pubblico in odom_kalman
        self.odometry_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom_kalman", 10)

        #Variabili per il kf. SOno riferiti alla vel angolare. Sono delle condizioni iniziali e dato che all'inizio io non 
        #so niente della sua velocita angolare quindi ho un'elevata incertezza e la distribuzione gaussiana ha media bassa e 
        #varianza elevata. Questa è la prior belief Come vedi nella stima iniziale vario solo la media con la lettura dell
        #encoder la varianza è elevatissima perchè non mi fido molto dell'encoder. Cosi creo la stima iniziale. Su di questa
        # faccio la measurement update in pratica leggo dall'imu e creo un'altra gaussiana: la media sarà la sua lettura e
        #la varianza la imposto io con measurement_varaince_
        self.mean_ = 0.0
        self.variance_ = 1000.0

        #Media e varianza associata alla lettura dell'Imu
        self.imu_angular_z_ = 0.0 #Salvo il valore letto nel topic
        self.measurement_variance_ = 0.5

        self.is_first_odom_ = True #Mi serve per eseguire roba solo se è la prima lettura
        self.last_angular_z_ = 0.0 # conservo l'ultimo dato prima di recevere quello nuovo

        #Media e varianza associata alla lettura dell'encoder
        self.motion_ = 0.0 #Tiene conto della differenza fra 2 vel. angolari consecutive nel tempo. 
        self.motion_variance_ = 4.0 #I valori gli metti un po a caso ovviamente l'encoder ha un'incertezza piu elevata quindi lo metti piu elevato

        self.kalman_odom_ = Odometry()

        

    def measurement_update(self):
        #Questa rappresenta la nuova stima della velocita angolare del robot. all'inizio è un prodotto dato dalla stima dovuta
        #dall'encoder e dalla lettura dell'Imu ma dopo si agigorna 
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z_) / (self.variance_ + self.measurement_variance_)
        self.variance_ = (self.variance_ * self.measurement_variance_ ) / (self.variance_ + self.measurement_variance_)
   
    def state_prediction(self):
        self.mean_ = self.mean_ + self.motion_
        self.variance_ = self.variance_ + self.motion_variance_
   
    #Ogni volta che ricevo un messaggio da parte dell'imu
    def imuCallback(self, imu_msg):
        #salva solo l'informazione della velocita su z. è usata come la media della gaussiana associata all'imu
        self.imu_angular_z_ = imu_msg.angular_velocity.z

   
   
    #Qui applico l'algoritmo del FK: ogni volta che ricevo una trasformata con rumore
    def odomCallback(self, odom):
        self.kalman_odom_ = odom

        #Se è il primo odom mesg che ricevi:
        if self.is_first_odom_:
            #Aggiorna la stima iniziale della velocita angolare.
            #Leggo una prima stima dall'encoder e creo la mia distribuzione di probabilita (a forma di gaussiana) 
            #Come veedi prendo la misura di velocita è dico che quella è la media e la varianza l'ho descritta prima
            #ed è elevata perche non mi fido molto dell'encoder. Quindi ho creato una gaussiana ed è la mia inital guess
            self.mean_ = odom.twist.twist.angular.z #
            self.last_angular_z_ = odom.twist.twist.angular.z
            #Cosi qui dentro non ci torni piu
            self.is_first_odom_ = False
            return
        #Queste vengono eseguite dalla seconda callback in poi
        #Calcolo di quanto mi sono spostato dal punto precedente(Viene usata solo in state prediction)
        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_
        #Non invertire l'ordine
        self.state_prediction() 
        self.measurement_update()
        #Aggiorno il messaggio che voglio pubblicare(solo con l'info su z)
        self.kalman_odom_.twist.twist.angular.z = self.mean_
        #Pubblico
        self.odometry_pub_.publish(self.kalman_odom_) 


def main():
    rclpy.init()
    node = KalmanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

