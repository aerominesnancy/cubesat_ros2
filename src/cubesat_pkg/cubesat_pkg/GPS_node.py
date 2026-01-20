import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import time
import serial

class GPS(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.is_valid = True

        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)

        # uart2 are GPIO (for raspberry pi 4) 12 (TX) and 13 (RX)
        # ttyAMA* index can change depending on the number of serial port on the raspberry pi
        self.ser = serial.Serial('/dev/ttyAMA1', baudrate=9600, timeout=1)
        self.timer = self.create_timer(1.0, self.read_gps_data)

        self.read_gps_data()
        self.get_logger().info('GPS node has been started.')

    def read_gps_data(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.read(self.ser.in_waiting)
                self.get_logger().info(f'Received GPS data: {line}')
            else:
                self.get_logger().info("Rien a lire...")


        except Exception as e:
            self.get_logger().error(f'Error reading GPS data: {e}')



def main(args=None):
    rclpy.init(args=args)

    gps_node = GPS()

    # let the node "alive" until interrupted
    try :
        if gps_node.is_valid:
            rclpy.spin(gps_node)

    except KeyboardInterrupt:
        gps_node.get_logger().warn('GPS node interrupted and is shutting down...')

    finally:
        if rclpy.ok():  # if the node is still running
            time.sleep(1)  # wait for logs to be sent
            rclpy.shutdown()