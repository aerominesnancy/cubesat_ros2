import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class Motor(Node):

    def __init__(self, callback_delay_second=1.0):
        super().__init__('motor')
        self.imu_subscriber = self.create_subscription(Vector3, '/imu/data', self.imu_callback, 10)
        self.get_logger().info('Motor Node has been started.')

    def imu_callback(self, msg:Vector3):
        self.get_logger().info('Received IMU data: %f, %f, %f' % (msg.x, msg.y, msg.z))



def main(args=None):
    rclpy.init(args=args)
    motor_node = Motor()

    # let the node "alive" until interrupted
    rclpy.spin(motor_node)

    motor_node.destroy_node()
    rclpy.shutdown()