#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature

import RPi.GPIO as GPIO
import time

# ros2 run cubesat_pkg heater_node --ros-args -p heater_id:=1 -p pwm_pin:=18

class Heater(Node):

    def __init__(self):
        super().__init__('heater')
        self.is_valid = True

        # Récupération des paramètres
        self.heater_id = self.declare_parameter('heater_id', -1).value
        self.pin_pwm = self.declare_parameter('pwm_pin', -1).value

        if self.pin_pwm == -1:
            self.get_logger().fatal(f"Heater pwm pin must be set to valid pin numbers. Current value is pwm_pin={self.pin_pwm}")
            self.is_valid = False
        if self.heater_id == -1:
            self.get_logger().fatal(f"Heater id must be set to valid id. Current value is heater_id={self.heater_id}")
            self.is_valid = False
        
        if not self.is_valid:
            self.get_logger().warn("Heater node is shutting down...")
        else:
            # subscription to temperature data
            self.create_subscription(Temperature, f"/temp_hum_sensor_{self.heater_id}/temperature", self.temp_sensor_callback, 1)

            # init GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.pin_pwm, GPIO.OUT)
            self.pwm = GPIO.PWM(self.pin_pwm, 100)
            self.pwm.start(0)

            # log
            self.get_logger().info('Heater node has been started.')


    def temp_sensor_callback(self, msg:Temperature):
        """
        This callback is called when a message is recieved on topic '/temp_hum_sensor_#/temperature'
        It use the temperature information to chnage the pwm value of the heater.
        """
        self.get_logger().info(f'Received form sensor n°{self.heater_id} : Temp = {msg.temperature}')

        # THERE IS NO SCIENTIFIC REASON FOR THIS FORMULA, IT'S JUST A TEST
        pwm = min(100, 2*max(0, 50-msg.temperature))

        self.get_logger().info(f"Setting heater pwm to {pwm}%" )
        self.pwm.ChangeDutyCycle(pwm)


    def destroy_node(self):
        """
        Cleanup the GPIO pins when the node is destroyed.
        """
        if self.is_valid:
            self.pwm.stop()
            GPIO.cleanup([self.pin_pwm])
            self.get_logger().info(f'Heater n°{self.heater_id} GPIO cleaned up.')



def main(args=None):
    rclpy.init(args=args)
    heater_node = Heater()

    # let the node "alive" until interrupted
    try :
        if heater_node.is_valid:
            rclpy.spin(heater_node)

    except KeyboardInterrupt:
        heater_node.get_logger().warn('Heater node interrupted and is shutting down...')

    finally:
        heater_node.destroy_node()
        if rclpy.ok():  # if the node is still running
            time.sleep(1)  # wait for logs to be sent
            rclpy.shutdown()