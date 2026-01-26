#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


import warnings
# Ignore les avertissements liés à la fréquence I2C
warnings.filterwarnings("ignore", message="I2C frequency is not settable in python")

from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno055 import BNO055_I2C
import time


class IMU(Node):

    def __init__(self):
        super().__init__('imu')
        self.is_valid = True

        # Récupération des paramètres
        callback_delay_second = self.declare_parameter('callback_delay_second', -1.0).value
        
        if callback_delay_second == -1:
            self.get_logger().error("Parameter 'callback_delay_second' must be set to a positive float."
                                    + f" Current value : {callback_delay_second}")
            self.get_logger().warn("IMU node is shutting down...")
            self.is_valid = False

        # Initialisation du node si aucune erreur de paramètre
        else:
            # Initialise l'I2C
            i2c = I2C(1)
            self.sensor = BNO055_I2C(i2c, address=0x28) # sudo i2cdetect -y 1 # permet de connaitre les ports i2c detectés

            self.data_pub = self.create_publisher(Vector3, '/imu/orientation', 1)
            self.create_timer(callback_delay_second, self.send_imu_data)
            
            self.get_logger().info('IMU node has been started.')


    def read_imu_data(self):
        self.yaw, self.pitch, self.roll = self.sensor.euler
        
        if self.yaw is None or self.pitch is None or self.roll is None:
            self.get_logger().warn("Failed to read IMU data.")
            return False
        if not(0 < self.yaw and self.yaw < 360):
            self.get_logger().warn(f"IMU yaw data out of range : {self.yaw}")
            return False
        if not(-90 < self.pitch and self.pitch < 90):
            self.get_logger().warn(f"IMU pitch data out of range : {self.pitch}")
            return False
        if not(-180 < self.roll and self.roll < 180):
            self.get_logger().warn(f"IMU roll data out of range : {self.roll}")
            return False
        return True


    def send_imu_data(self):
        self.read_imu_data()
        
        if self.read_imu_data():
            msg = Vector3()
            msg.x = self.yaw
            msg.y = self.pitch
            msg.z = self.roll

            self.data_pub.publish(msg) 
            self.get_logger().info('IMU data sended : %f, %f, %f' % (msg.x, msg.y, msg.z))

       



def main(args=None):
    rclpy.init(args=args)

    imu_node = IMU()

    # let the node "alive" until interrupted
    try :
        if imu_node.is_valid:
            rclpy.spin(imu_node)

    except KeyboardInterrupt:
        imu_node.get_logger().warn('IMU node interrupted and is shutting down...')

    finally:
        if rclpy.ok():  # if the node is still running
            time.sleep(1)  # wait for logs to be sent
            rclpy.shutdown()