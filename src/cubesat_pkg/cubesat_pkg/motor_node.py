#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import RPi.GPIO as GPIO
import time

class motor_GPIOWrapper:
    def __init__(self, pin_R, pin_L, pin_pwm):
        # initialisation des pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(pin_R, GPIO.OUT)
        GPIO.setup(pin_L, GPIO.OUT)
        GPIO.setup(pin_pwm, GPIO.OUT)

        # alimentation du moteur (choix du sens du rotation)
        self.pin_R = pin_R  # fil orange
        self.pin_L = pin_L  # fil vert
        self.clockwise()

        # pwm moteur (choix de la vitesse de rotation)
        self.pin_pwm = pin_pwm
        self.pwm = GPIO.PWM(pin_pwm, 100) # fil jaune
        self.pwm.start(0)
        GPIO.setup(self.pin, GPIO.OUT)

    def clockwise(self):
        GPIO.output(self.pin_R, GPIO.HIGH)
        GPIO.output(self.pin_L, GPIO.LOW)

    def counterClockwise(self):
        GPIO.output(self.pin, GPIO.LOW)
    
    def setpwm(self, pwm):
        self.pwm.ChangeDutyCycle(pwm)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup([self.pin_R, self.pin_L, self.pin_pwm])



class Motor(Node):

    def __init__(self):
        super().__init__('motor')
        self.is_valid = True

        # Récupération des paramètres
        pin_R = self.declare_parameter('pin_input_1', -1).value
        pin_L = self.declare_parameter('pin_input_2', -1).value
        pin_pwm = self.declare_parameter('pwm_pin', -1).value

        if pin_R == -1 or pin_L == -1 or pin_pwm == -1:
            self.get_logger().fatal("Motor GPIO pins must be set to valid pin numbers."
                                    + f" Current values : pin_input_1={pin_R}, pin_input_2={pin_L}, pwm_pin={pin_pwm}")
            self.get_logger().warn("Motor node is shutting down...")
            self.is_valid = False

        else:
            # subscription to Temperature data
            self.create_subscription(Vector3, '/imu/orientation', self.imu_callback, 1)

            self.motor = motor_GPIOWrapper(pin_R, pin_L, pin_pwm)

            # log
            self.get_logger().info('Motor node has been started.')

    def imu_callback(self, msg:Vector3):
        self.get_logger().info('Received IMU data: %f, %f, %f' % (msg.x, msg.y, msg.z))

        pwm = int(100*msg.x / 360)
        self.get_logger().info(f"Setting motor speed to {pwm}%" )
        self.motor.setpwm(pwm)

    def destroy_node(self):
        if self.is_valid:
            self.motor.cleanup()
            self.get_logger().info('Motor GPIO cleaned up.')



def main(args=None):
    rclpy.init(args=args)
    motor_node = Motor()

    # let the node "alive" until interrupted
    try :
        if motor_node.is_valid:
            rclpy.spin(motor_node)

    except KeyboardInterrupt:
        motor_node.get_logger().warn('Motor node interrupted and is shutting down...')

    finally:
        motor_node.destroy_node()
        if rclpy.ok():  # if the node is still running
            time.sleep(1)  # wait for logs to be sent
            rclpy.shutdown()