import cv2


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


class camera(Node):

    def __init__(self):
        super().__init__('camera')
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
            self.cap = cv2.VideoCapture(0)  # Ouvre la caméra par défaut

            self.create_timer(callback_delay_second, self.take_picture)

            self.get_logger().info('Camera node has been started.')


    def take_picture(self, path = "/home/cubesat/ros2_ws/pictures/"):
        ret, frame = self.cap.read()

        if ret:
            cv2.imwrite(path + "test.jpg", frame)
            self.get_logger().info("Picture taken and saved as 'test.jpg'")
        else:
            self.get_logger().error("Failed to capture image")

        self.cap.release()



       



def main(args=None):
    rclpy.init(args=args)

    camera_node = camera()

    # let the node "alive" until interrupted
    try :
        if camera_node.is_valid:
            rclpy.spin(camera_node)

    except KeyboardInterrupt:
        camera_node.get_logger().warn('Camera node interrupted and is shutting down...')

    finally:
        if rclpy.ok():  # if the node is still running
            time.sleep(1)  # wait for logs to be sent
            rclpy.shutdown()



