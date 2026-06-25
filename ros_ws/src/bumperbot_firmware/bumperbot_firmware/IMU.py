#!/usr/bin/env python3
import rclpy.time
import smbus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

#I soliti indirizzi di memoria per la periferica imu
DEVICE_ADDRESS = 0x68 #Inidirizzo  cui risponde l'imu
#Tutte le sue aree di memoria
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


#Dopodichè uso i metodi della classe Nodo per definire il publisher(create_publisher è ereditato)
class ImuNode(Node):
    def __init__(self):
        super().__init__("Imu_node")

        self.pub_ = self.create_publisher(Imu, "imu_ekf", 10) #tipo messaggio, topic name, lunghezza coda

        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "imu_link_ekf" #Se vuoi visualizzarlo su rviz installa il plugin imu e inserisci qeusto come FixedFrame
        
        self.is_connected = False
        self.init_i2c()

        self.frequnecy = 0.01 # 100Hz Frequenza di pubblicazione dei messaggi nel topic
        self.timer = self.create_timer(self.frequnecy, self.timerCallback)#crea un timer ad ogni self.frequency chiama la callback

    def timerCallback(self):

        try:
            if not self.is_connected:
                self.init_i2c() #Riprova
        
            #Sta roba viene eseguita comunque ma vabbe..
            # Read Accelerometer raw value
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)
                
            # Read Gyroscope raw value
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)
                
            # Full scale range +/- 250 degree/C as per sensitivity scale factor
            # #Trovi sti valori sul datasheet e servono per la conversione in sto caso li voglio in m/s^2     
            self.imu_msg_.linear_acceleration.x = -acc_x / 1670.13 #Per come ho montato il sensore restituisce valori negativi quando il ronbot si sposta lungo x positivo
            self.imu_msg_.linear_acceleration.y = acc_y / 1670.13
            self.imu_msg_.linear_acceleration.z = acc_z / 1670.13
            # E queste in rad/s
            self.imu_msg_.angular_velocity.x = gyro_x / 7509.55
            self.imu_msg_.angular_velocity.y = gyro_y / 7509.55
            self.imu_msg_.angular_velocity.z = -gyro_z / 7509.55

            #Aggiorno l'orario del messaggio
            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()

            self.pub_.publish(self.imu_msg_) #uso il publisher creato prima per pubblicare il messaggio
        
        except OSError:
            #Se hai problemi riprova la connessione
            self.is_connected = False


    def read_raw_data(self, addr):
        #Questo legge du byte adiacenti
        high = self.bus.read_byte_data(DEVICE_ADDRESS, addr)
        low = self.bus.read_byte_data(DEVICE_ADDRESS, addr+1)

        #Creo una unica varaibile composta dai 2 byte high e low: sposta a sinistra high di 1byte e nei nuovi 8 bit creati inserisci low
        value = ((high << 8 ) | low)
        #In pratica 2^16=65536 la sua meta vale 32768, quindi se supera la meta allora il valore in realta
        #è negativo quindi lo riporti nel range corretto. Ti serve per attribuire un segno a variabili a 16bit unsigned(uint_16)
        if value > 32768:
            value = value - 65536 
        return value

    def init_i2c(self):
        #creo un nuovo oggetto della claasse SMBus della libreria smbus e mi serve per scrivere su i2c e inizializzare la comunicazione
        self.bus = smbus.SMBus(1)
        try:
            
            self.bus.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)
            self.bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
            self.bus.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)
            self.bus.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 24)
            self.bus.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)

            self.is_connected = True

        except OSError:
            self.is_connected = False


#Qui definisco il main
def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# Qui definisco la funzione che viene eseguita quando esegui questo file. Lancio il main
if __name__ == '__main__':
    main()


#Ogni volta che crei un nodo devi modificare il file setup.py che dice al compilatore? come convertire questo file in un eseguibile