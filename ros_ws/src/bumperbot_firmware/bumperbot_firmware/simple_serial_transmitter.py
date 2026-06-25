#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")

        #Parametri da dover inserire quando avvii il nodo. Ho messo valori di default nel caso no gli dichiari
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        #Prendo i valori inseriti dall'utente
        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value

        try:
            self.microcontroller = serial.Serial(self.port, self.baudrate, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
        self.sub = self.create_subscription(String, "serial_transmitter", self.msgCallback, 10) #Quando crei un subscriber devi definire anche la callback


    def msgCallback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)
        self.microcontroller.write((msg.data + "\n").encode("utf-8")) #Aggiungo al messaggio un terminator così è comprensibile dal HAL


def main():
    rclpy.init()
    node = SimpleSerialTransmitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
