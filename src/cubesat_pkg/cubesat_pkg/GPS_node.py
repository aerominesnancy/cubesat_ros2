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
        # uart2 are GPIO (for raspberry pi 4) 4 (TX) and 5 (RX)
        self.serial_port = serial.Serial('/dev/serial2', baudrate=9600, timeout=1)
        self.timer = self.create_timer(1.0, self.read_gps_data)

    def read_gps_data(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line.startswith('$GPGGA'):
                parts = line.split(',')
                if parts[6] == '1':  # Fix quality
                    latitude = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
                    longitude = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
                    altitude = float(parts[9])

                    msg = NavSatFix()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'gps_frame'
                    msg.latitude = latitude
                    msg.longitude = longitude
                    msg.altitude = altitude
                    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published GPS data: Latitude={latitude}, Longitude={longitude}, Altitude={altitude}')
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
        gps_node.get_logger().warn('LoRa node interrupted and is shutting down...')

    finally:
        if rclpy.ok():  # if the node is still running
            time.sleep(1)  # wait for logs to be sent
            rclpy.shutdown()