#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import RPi.GPIO as GPIO
import time

class GPIOWrapper:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)

    def high(self):
        GPIO.output(self.pin, GPIO.HIGH)

    def low(self):
        GPIO.output(self.pin, GPIO.LOW)


class Motor(Node):

    def __init__(self, callback_delay_second=1.0):
        super().__init__('motor')
        self.is_valid = True

        # Récupération des paramètres
        self.pin_R = self.declare_parameter('pin_input_1', -1).value
        self.pin_L = self.declare_parameter('pin_input_2', -1).value
        self.pin_pwm = self.declare_parameter('pwm_pin', -1).value

        if self.pin_R == -1 or self.pin_L == -1 or self.pin_pwm == -1:
            self.get_logger().fatal("Motor GPIO pins must be set to valid pin numbers."
                                    + f" Current values : pin_input_1={self.pin_R}, pin_input_2={self.pin_L}, pwm_pin={self.pin_pwm}")
            self.get_logger().warn("Motor node is shutting down...")
            self.is_valid = False

        else:
            # subscription to IMU data
            self.imu_subscriber = self.create_subscription(Vector3, '/imu/orientation', self.imu_callback, 10)

            # alimentation du moteur (choix du sens du rotation)
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            self.input_R = GPIOWrapper(self.pin_R)  # fil orange
            self.input_L = GPIOWrapper(self.pin_L)  # fil vert
            self.input_R.high()
            self.input_L.low()

            # pwm moteur (choix de la vitesse de rotation)
            GPIO.setup(self.pin_pwm, GPIO.OUT)
            self.pwm = GPIO.PWM(self.pin_pwm, 100) # fil jaune
            self.pwm.start(0)

            # log
            self.get_logger().info('Motor node has been started.')

    def imu_callback(self, msg:Vector3):
        self.get_logger().info('Received IMU data: %f, %f, %f' % (msg.x, msg.y, msg.z))

        pwm = int(100*msg.x / 360)
        self.get_logger().info(f"Setting motor speed to {pwm}%" )
        self.pwm.ChangeDutyCycle(pwm)

    def destroy_node(self):
        if self.is_valid:
            self.pwm.stop()
            GPIO.cleanup([self.pin_R, self.pin_L, self.pin_pwm])
            self.get_logger().info('Motor GPIO cleaned up.')



def main(args=None):
    rclpy.init(args=args)
    motor_node = Motor()

    # let the node "alive" until interrupted
    try :
        if motor_node.is_valid:
            rclpy.spin(motor_node)

    except KeyboardInterrupt:
        motor_node.get_logger().info('Motor node interrupted and is shutting down...')

    finally:
        motor_node.destroy_node()
        if rclpy.ok():  # if the node is still running
            time.sleep(1)  # wait for logs to be sent
            rclpy.shutdown()