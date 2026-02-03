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
        """
        If a message is received on topic '/camera/ask_picture', 
        this function will take a picture and publish it on the topic '/camera/picture'.
        """
        compression_factor = msg.data
        self.get_logger().info(f"Asking picture with a compression factor of {compression_factor}%")

        # take picture
        compressed_picture_bytes = self.take_picture(compression_factor, save_file=False)
        if compressed_picture_bytes:
            self.get_logger().info(f"Published picture on topic '/camera/picture'.")
            self.picture_pub.publish(UInt8MultiArray(data=list(compressed_picture_bytes)))

        else: 
            self.get_logger().warn("Picture transfert cancelled")
            self.picture_pub.publish(UInt8MultiArray(data=[]))




    def take_picture(self, compression_factor = 50, save_file = True, try_number=0, max_try=5):
        """ 
        Take a picture with a certain compression factor.
        If save_file == True, save the picture in the folder 'pictures' with the name 'test_{timestamp}.jpg'.
        Else, return the picture as a bytearray object.
        """
        if not self.try_connect():
            self.get_logger().error("Cannot take any picture.")
            return
        time.sleep(0.5)

        if self.cap is not None:
            if self.cap.isOpened():

                ret, frame = self.cap.read()

                file_name = f"test_{time.time()}.jpg"

                if ret:
                    # enregistre l'image avec une qualité de 50%
                    if save_file:
                        cv2.imwrite(self.path + file_name, frame, [cv2.IMWRITE_JPEG_QUALITY, compression_factor])

                        self.get_logger().info(f"Picture taken and saved as '{file_name}'")
                        self.close_connection()
                        return
                    else:

                        success, jpeg_binary = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), compression_factor])
                        if not success:
                            self.get_logger().error(f"Error in picture compression. Returning None.")
                            self.close_connection()
                            return 
                        
                        self.get_logger().info(f"Picture taken and compressed successfully.")
                        self.close_connection()
                        return bytearray(jpeg_binary)

                else:
                    # if the camera is detected but the image was not captured successfully
                    # the connection is reinitialized
                    self.get_logger().warn(f"Failed to capture image from camera. Try number {try_number+1}/{max_try}")
                    self.close_connection()

                    # try tkaing a picture again 
                    if try_number < max_try:
                        time.sleep(0.5)
                        self.take_picture(compression_factor, save_file, try_number + 1, max_try)
                    else:
                        self.get_logger().error(f"Failed to capture image from camera after {max_try} tries. Returning None.")
                        return


    def close_connection(self):
        """
        Close connection with camera.
        """
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            self.get_logger().info("Camera connection closed.")

    def try_connect(self, try_number=0, max_try=5):
        """
        This function try reconnect to the camera if it is disconnected.
        Check if the connection is successful and if the camera is detected.
        Else it reinitialize the camera connection and try again (max number of try).
        """
        # try reconnecting to the camera
        self.cap = cv2.VideoCapture(0)
        time.sleep(0.5)

        # check if the camera has been successfully reconnected
        if self.cap.isOpened():
            self.get_logger().info("Camera initialized successfully.")
            return True
        else: 
            self.get_logger().warn(f"Error initializing camera. Try number {try_number+1}/{max_try}")
            self.close_connection()
            
            if try_number < max_try:
                time.sleep(0.5)
                self.try_connect(try_number+1, max_try) 
            else:
                self.get_logger().error(f"Failed to initialize camera after {max_try} attempts.")
                return False




    def destroy_node(self):
        """
        Close the connection with the camera.
        """
        if self.is_valid:
            self.close_connection()

            
        



       



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



