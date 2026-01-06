#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import RPi.GPIO as GPIO

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
        self.imu_subscriber = self.create_subscription(Vector3, '/imu/data', self.imu_callback, 10)

        # alimentation du moteur (choix du sens du rotation)
        GPIO.setmode(GPIO.BCM)
        self.input_R = GPIOWrapper(20)  # fil orange
        self.input_L = GPIOWrapper(21)  # fil vert
        self.input_R.high()
        self.input_L.low()

        # pwm moteur (choix de la vitesse de rotation)
        GPIO.setup(16, GPIO.OUT)
        self.pwm = GPIO.PWM(16, 100) # fil jaune
        self.pwm.start(0)

        # log
        self.get_logger().info('Motor Node has been started.')

    def imu_callback(self, msg:Vector3):
        self.get_logger().info('Received IMU data: %f, %f, %f' % (msg.x, msg.y, msg.z))

        pwm = 100*msg.x // 360
        self.get_logger().info(f'Setting motor speed to {pwm}' )
        self.pwm.ChangeDutyCycle(pwm)

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    motor_node = Motor()

    # let the node "alive" until interrupted
    try :
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        pass
    
    motor_node.destroy_node()
    rclpy.shutdown()