#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO
import time
from LoRa_class import LoRa



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
                             logger=self.get_logger())

            # create loop timer
            self.create_timer(self.loop_delay_milisecond/1000, self.loop)

            self.get_logger().info('lora node has been started.')
        

    def loop(self):
        # recover any incoming messages
        self.lora.listen_radio()
        
        # read buffer for complete messages
        message = self.lora.extract_message()
        if message is not None:
            self.get_logger().info(f"Complete message received: {message}")
            self.lora.send_radio(f"ACK: {message}")

    
    def destroy_node(self):
        if self.is_valid:

            # close serial connection
            self.ser.close()

            # clean GPIO
            GPIO.cleanup([self.M0, self.M1, self.AUX])
            self.get_logger().info('LoRa GPIO cleaned up.')



def main(args=None):
    rclpy.init(args=args)

    imu_node = lora()

    # let the node "alive" until interrupted
    try :
        if imu_node.is_valid:
            rclpy.spin(imu_node)

    except KeyboardInterrupt:
        imu_node.get_logger().warn('LoRa node interrupted and is shutting down...')

    finally:
        if rclpy.ok():  # if the node is still running
            time.sleep(1)  # wait for logs to be sent
            rclpy.shutdown()