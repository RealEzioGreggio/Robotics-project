#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SimpleReceiver(Node):
    def __init__(self):
        super().__init__('simple_serial_receiver')

        #Parametri da dover inserire quando avvii il nodo. Ho messo valori di default nel caso no gli dichiari
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        #Prendo i valori inseriti dall'utente
        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value

        self.pub_ = self.create_publisher(String, "serial_receiver", 10) #tipo messaggio, topic name, lunghezza coda
        self.frequnecy = 0.01 #Frequenza di pubblicazione dei messaggi nel topic

        self.get_logger().info("Publishing at %d Hz " % self.frequnecy)

        self.microcontroller = serial.Serial(port=self.port, baudrate=self.baudrate)

        self.timer = self.create_timer(self.frequnecy, self.timerCallback)#crea un timer ad ogni self.frequency chiama la callback

    def timerCallback(self):
        if rclpy.ok() and self.microcontroller.is_open:
            data = self.microcontroller.readline()
            try:
                data.decode("utf-8")
            except:
                return
            
            msg = String()
            msg.data = str(data)
            self.pub_.publish(msg)

#Qui definisco il main
def main(args=None):
    rclpy.init(args=args)
    node = SimpleReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# Qui definisco la funzione che viene eseguita quando esegui questo file. Lancio il main
if __name__== '__main__':
    main()


#Ogni volta che crei un nodo devi modificare il file setup.py che dice al compilatore? come convertire questo file in un eseguibile