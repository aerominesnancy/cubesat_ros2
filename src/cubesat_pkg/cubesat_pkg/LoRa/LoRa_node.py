#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


import time


class lora(Node):

    def __init__(self):
        super().__init__('lora_node')
        self.is_valid = True

        self.get_logger().info('lora node has been started.')



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