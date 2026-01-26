#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Int8

import RPi.GPIO as GPIO
import time

from cubesat_pkg.LoRa.utils.LoRa_class import LoRa



class lora(Node):

    def __init__(self):
        super().__init__('lora_node')
        self.is_valid = True

        # Récupération des paramètres
        self.loop_delay_milisecond = self.declare_parameter('loop_delay_milisecond', 10).value
        self.M0 = self.declare_parameter('M0_pin', -1).value
        self.M1 = self.declare_parameter('M1_pin', -1).value
        self.AUX = self.declare_parameter('AUX_pin', -1).value
        self.AUX_timeout = 5  # seconds
        self.serial_timeout = 5  # seconds

        if self.loop_delay_milisecond == -1:
            self.get_logger().fatal("Parameter 'loop_delay_milisecond' must be set to a positive int."
                                    + f" Current value : {self.loop_delay_milisecond}")
            self.is_valid = False

        if self.M0 == -1 or self.M1 == -1 or self.AUX == -1:
            self.get_logger().fatal("LoRa GPIO pins must be set to valid pin numbers."
                                    + f" Current values : M0={self.M0}, M1={self.M1}, AUX={self.AUX}")
            self.is_valid = False

        if not self.is_valid:
            self.get_logger().warn("LoRa node is shutting down...")
        else:
            # initialize LoRa module 
            self.lora = LoRa(self.M0, self.M1, self.AUX,
                             self.AUX_timeout, self.serial_timeout,
                             logger=self.get_logger()) # pass the ROS2 node logger to LoRa class
            self.current_file_paquets = []

            # create loop timer
            self.create_timer(self.loop_delay_milisecond/1000, self.loop)

            # create publisher and listener
            self.ask_camera_for_picture_pub = self.create_publisher(Int8, '/camera/ask_picture', 1)
            self.create_subscription(UInt8MultiArray, '/camera/picture', self.picture_received_from_camera, 1)

            self.get_logger().info('lora node has been started.')
        

    def loop(self):
        # recover any incoming messages
        self.lora.listen_radio()
        
        # read buffer for complete messages
        msg = self.lora.extract_message()

        # on quitte la boucle si rien n'est reçu
        if msg is None:
            return
        
        msg_type, message, checksum = msg

        # ACK may delay file transfert
        """
        # Acknowledge received messages
        self.get_logger().info(f"Complete message received: (type : {msg_type}) {message}.")
        self.lora.send_message(checksum, "ACK")
        """

        if "file" in msg_type or "picture" in msg_type:
            self.get_logger().info(f"Handling file transfert (received message type : {msg_type}) : {message}")
            self.handle_file_transfert(msg_type, message)

    
    def handle_file_transfert(self, message_type, message):
        if message_type == "ask_for_file_transmission":
            file_path = message

            try:
                with open(file_path, 'rb') as file:
                    data = file.read()
            except Exception as e:
                self.get_logger().error(f"Erreur lors de l'ouverture du fichier {file_path}. Demande de transfert annulée.")
                return

            # if the file has been read properly
            nb_of_paquets = self.save_packets_list(data)
            self.lora.send_message(nb_of_paquets, "file_info")
        

        elif message_type == "ask_for_picture":
            quality = Int8()
            quality.data = message
            self.ask_camera_for_picture_pub.publish(quality)

            # create a timeout for the LoRa : send a message to the user if the camera doesn't work anymore

            
        elif message_type == "ask_for_file_paquet":
            paquet_index = message
            if paquet_index<0 or paquet_index > len(self.current_file_paquets)-1:
                self.get_logger().error(f"Paquet demandé inexistant : {paquet_index}. Demande de transfert annulée.")
            
            else:
                self.lora.send_message((paquet_index, self.current_file_paquets[paquet_index]), "file_paquet")


    def picture_received_from_camera(self, msg):
        # when a picture (list of octets) is received from camera (after asking for it)
        self.get_logger().info("Picture received from camera.")
        data = bytearray(msg.data)

        if data == b'':
            self.get_logger().warn("Issue with camera. Picture transfert cancelled.")
            
        nb_of_paquets = self.save_packets_list(data)
        self.get_logger().info(f"Number of packets : {nb_of_paquets}")

        self.lora.send_message(nb_of_paquets, "file_info")
                

    def save_packets_list(self, data):
        # cut data and save the packets list in self.current_file_paquets
        max_paquet_size = self.lora.paquet_size - self.lora.wrapper_size - 2 # on ajoute 2 octets pour le numéro du paquet
        self.current_file_paquets = [data[i:i+max_paquet_size] for i in range(0, len(data), max_paquet_size)]
        return len(self.current_file_paquets)

    
    def destroy_node(self):
        if self.is_valid:

            # close serial connection
            self.ser.close()

            # clean GPIO
            GPIO.cleanup([self.M0, self.M1, self.AUX])
            self.get_logger().info('LoRa GPIO cleaned up.')




def main(args=None):
    rclpy.init(args=args)

    lora_node = lora()

    # let the node "alive" until interrupted
    try :
        if lora_node.is_valid:
            rclpy.spin(lora_node)

    except KeyboardInterrupt:
        lora_node.get_logger().warn('LoRa node interrupted and is shutting down...')

    finally:
        if rclpy.ok():  # if the node is still running
            time.sleep(1)  # wait for logs to be sent
            rclpy.shutdown()