#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import serial
import RPi.GPIO as GPIO
import time
from cubesat_pkg.LoRa.LoRa_data_encapsulation import encapsulate, Buffer



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
            self.get_logger().fatal("Parameter 'loop_delay_milisecond' must be set to a positive float."
                                    + f" Current value : {self.loop_delay_milisecond}")
            self.is_valid = False

        if self.M0 == -1 or self.M1 == -1 or self.AUX == -1:
            self.get_logger().fatal("LoRa GPIO pins must be set to valid pin numbers."
                                    + f" Current values : M0={self.M0}, M1={self.M1}, AUX={self.AUX}")
            self.is_valid = False

        if not self.is_valid:
            self.get_logger().warn("LoRa node is shutting down...")
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
            self.create_timer(self.loop_delay_milisecond/1000, self.loop)

            # buffer for incoming messages
            self.buffer = Buffer()

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
        


    def send_message(self, message: str):
        
        if not self.wait_aux():
            self.get_logger().error("Cannot send message because LoRa module is not ready.")
            return  # cannot send if AUX is not HIGH

        try:
            self.ser.write(encapsulate(message))
            #self.ser.flush()   # this line ensure data is sent but blocks the program and bypass timeout handling 
        except serial.SerialTimeoutException:
            self.get_logger().error("Error sending message: Serial timeout.")


        if not self.wait_aux():
            self.get_logger().error("Try sending message for too long, message may not have been sent.")
            return  

        self.get_logger().info(f"Message envoyé : {message}")

    
    def read_data(self):
    
        if self.ser.in_waiting > 0:
            bytes_msg = self.ser.read(self.ser.in_waiting)
            self.buffer.append(bytes_msg)
            self.get_logger().info(f"Received some data. Buffer size: {self.buffer.size} bytes.")
        else:
            #self.get_logger().info(f"No message received.")
            pass
        

    def loop(self):


        # recover any incoming messages
        self.read_data()
        
        # read buffer for complete messages
        message = self.buffer.extract_message()
        if message is not None:
            self.get_logger().info(f"Complete message received: {message}")

    
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