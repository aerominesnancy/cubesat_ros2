#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import serial
import RPi.GPIO as GPIO
import time


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

        if self.M0 == -1 or self.M1 == -1 or self.AUX == -1:
            self.get_logger().error("LoRa GPIO pins must be set to valid pin numbers."
                                    + f" Current values : M0={self.M0}, M1={self.M1}, AUX={self.AUX}")
            self.get_logger().warn("LoRa node is shutting down...")
            self.is_valid = False

        else:
            # setup GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.M0, GPIO.OUT)
            GPIO.setup(self.M1, GPIO.OUT)
            GPIO.setup(self.AUX, GPIO.IN)

            # Normal mode (transmission and reception)
            GPIO.output(self.M0, GPIO.LOW)
            GPIO.output(self.M1, GPIO.LOW)

            # setup serial connection
            self.ser = serial.Serial(port='/dev/serial0', baudrate=9600, timeout=self.serial_timeout)

            # create loop timer
            self.create_timer(self.loop_delay_milisecond, self.loop)

            self.get_logger().info('lora node has been started.')


    
    def wait_aux(self):
        """Waits until the AUX pin goes HIGH, indicating that the LoRa module is ready."""

        start_time = time.time()

        while (GPIO.input(self.AUX) == GPIO.LOW) and (time.time() - start_time < self.AUX_timeout):
            time.sleep(0.1)
        
        if GPIO.input(self.AUX) == GPIO.LOW:
            self.get_logger().warn("Timeout waiting for LoRa module to be ready (AUX pin HIGH).")
            return False
        
        return True
        


    def send_string(self, message: str):
        
        if not self.wait_aux():
            self.get_logger().error("Cannot send message because LoRa module is not ready.")
            return  # cannot send if AUX is not HIGH

        try:
            self.ser.write(message.encode('utf-8'))
            #self.ser.flush()   # ensure data is sent but blocks the program and bypass timeout handling
        except serial.SerialTimeoutException:
            self.get_logger().error("Error sending message: Serial timeout.")


        if not self.wait_aux():
            self.get_logger().error("Try sending message for too long, message may not have been sent.")
            return  
        
        print(time.time(), "Message envoyé :", message ,"\n\n\n")

    
    def receive_message(self):
    
        if self.ser.in_waiting > 0:
            message = self.ser.read(self.ser.in_waiting).decode('utf-8')
            self.get_logger().info(f"Received message: {message}")
            return message
        else:
            self.get_logger().info(f"No message received.")
            return None  # no message received
        

    def loop(self):
        # recover any incoming messages
        received = self.receive_message()
        if received:
            self.send_string(f"Echo: {received}")



    
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