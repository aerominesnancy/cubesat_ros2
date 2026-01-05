import rclpy
from rclpy.node import Node


class IMU(Node):

    def __init__(self, callback_delay=1.0):
        super().__init__('imu')

        self.create_timer(callback_delay, self.read_imu_data)
        
        self.get_logger().info('IMU Node has been started.')

    def read_imu_data(self):
        self.get_logger().info('Reading IMU data...')



def main(args=None):
    rclpy.init(args=args)
    imu_node = IMU()

    # let the node "alive" until interrupted
    rclpy.spin(imu_node)

    imu_node.destroy_node()
    rclpy.shutdown()