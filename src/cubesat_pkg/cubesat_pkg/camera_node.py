


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


import time
import cv2
import os
# Suppress OpenCV warnings
#os.environ["OPENCV_LOG_LEVEL"] = "ERROR"



class camera(Node):

    def __init__(self):
        super().__init__('camera')
        self.is_valid = True

        # Récupération des paramètres
        callback_delay_second = self.declare_parameter('callback_delay_second', -1.0).value
        
        if callback_delay_second == -1:
            self.get_logger().error("Parameter 'callback_delay_second' must be set to a positive float."
                                    + f" Current value : {callback_delay_second}")
            self.get_logger().warn("Camera node is shutting down...")
            self.is_valid = False

        # Initialisation du node si aucune erreur de paramètre
        else:
            self.cap = cv2.VideoCapture(0)  # Ouvre la caméra par défaut

            self.create_timer(callback_delay_second, self.take_picture)

            # create directory to save pictures if it doesn't exist
            self.path = "/home/cubesat/ros2_ws/pictures/"
            os.makedirs(self.path, exist_ok=True)

            self.get_logger().info('Camera node has been started.')


    def take_picture(self, compression_factor = 50):
        ret, frame = self.cap.read()

        file_name = f"test_{time.time()}.jpg"

        if ret:
            # Convertit l'image en niveaux de gris
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # enregistre l'image avec une qualité de 50%
            cv2.imwrite(self.path + file_name, gray, [cv2.IMWRITE_JPEG_QUALITY, compression_factor])

            self.get_logger().info(f"Picture taken and saved as '{file_name}'")

        else:
            self.get_logger().warn("Failed to capture image, attempting to re-initialize camera...")
            
            # release the camera and wait before re-initializing
            self.cap.release()
            time.sleep(0.5)

            # try reconnecting to the camera
            self.cap = cv2.VideoCapture(0)
            time.sleep(0.5)

            # check if the camera has been successfully reconnected
            if self.cap.isOpened():
                self.get_logger().warn("Camera re-initialized successfully.")
            else: 
                self.get_logger().error(f"Error re-initializing camera. Camera may be disconnected.")
            
        



       



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
            camera_node.cap.release()
            rclpy.shutdown()



