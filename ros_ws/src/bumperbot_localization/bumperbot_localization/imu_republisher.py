#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu

#Questo ha il compito di cambiare il nome del frame_id di ogni messaggi pubblicato dall'imu e lo metto uguale frame del robot.
#Questo è reso 

imu_pub = None

def imuCallback(imu):
    global imu_pub
    imu.header.frame_id = "imu_link_ekf" 
    imu_pub.publish(imu)


def main(args=None):
    global imu_pub
    rclpy.init(args=args)
    node = Node('imu_republisher_node')
    time.sleep(1)
    imu_pub = node.create_publisher(Imu, "imu_ekf", 10)
    imu_sub = node.create_subscription(Imu, "imu/ros", imuCallback, 10)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()