#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Int8

import time
import os

# Suppress OpenCV warnings
os.environ["OPENCV_LOG_LEVEL"] = "ERROR"
import cv2



class camera(Node):

    def __init__(self):
        super().__init__('camera')
        self.is_valid = True

        # Récupération des paramètres
        callback_delay_second = self.declare_parameter('callback_delay_second', -1.0).value
        
        if callback_delay_second == -1:
            self.get_logger().fatal("Parameter 'callback_delay_second' must be set to a positive float."
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

            # create pub sub
            self.create_subscription(Int8, '/camera/ask_picture', self.send_picture_when_ask, 1)
            self.picture_pub = self.create_publisher(UInt8MultiArray, '/camera/picture', 1)


            self.get_logger().info('Camera node has been started.')

    def send_picture_when_ask(self, msg):
        compression_factor = msg.data
        self.get_logger().info(f"Asking picture with a compression factor of {compression_factor}%")

        # take picture
        compressed_picture_bytes = self.take_picture(compression_factor, save_file=False)
        if compressed_picture_bytes:
            self.picture_pub.publish(UInt8MultiArray(data=list(compressed_picture_bytes)))

        else: 
            self.get_logger().warn("Picture transfert cancelled")
            self.picture_pub.publish(UInt8MultiArray(data=[]))




    def take_picture(self, compression_factor = 50, save_file = True):
        """ Prend une phot et l'enregistre 2 fois:
        1 fois avec son timestamp
        1 fois sous le nom 'last_picture' (remplacé a chaque photo)
        """
        if self.cap is not None:
            if self.cap.isOpened():

                ret, frame = self.cap.read()

                file_name = f"test_{time.time()}.jpg"

                if ret:
                    # enregistre l'image avec une qualité de 50%
                    if save_file:
                        cv2.imwrite(self.path + file_name, frame, [cv2.IMWRITE_JPEG_QUALITY, compression_factor])

                        self.get_logger().info(f"Picture taken and saved as '{file_name}'")
                        return
                    else:

                        success, jpeg_binary = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), compression_factor])
                        if not success:
                            self.get_logger().error(f"Error in picture compression. Returning None.")
                            return 
                        
                        self.get_logger().info(f"Picture taken and returned compressed successfully.")
                        return bytearray(jpeg_binary)


                else:
                    self.get_logger().warn("Failed to capture image from camera. Attempting to reconnect...")
                    self.cap.release()
                    self.cap = None

            else:
                self.get_logger().warn("Connection to camera lost. Attempting to reconnect...")
                self.cap.release()
                self.cap = None
            
        
        # if the image was not captured successfully
        self.try_reconnect()


    def try_reconnect(self):

        # try reconnecting to the camera
        self.cap = cv2.VideoCapture(0)
        time.sleep(0.5)

        # check if the camera has been successfully reconnected
        if self.cap.isOpened():
            self.get_logger().warn("Camera re-initialized successfully.")
        else: 
            self.get_logger().error(f"Error re-initializing camera. Camera may be disconnected.")
            self.cap.release()
            self.cap = None

    def destroy_node(self):
        if self.is_valid:
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
            camera.destroy_node()
            rclpy.shutdown()



