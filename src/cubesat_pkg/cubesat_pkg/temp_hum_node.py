import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import time
import adafruit_dht
import board


class TemperatureHumidityNode(Node):

    def __init__(self):
        # this name is defined in the launch file (with a different index for each sensor)
        super().__init__("temp_hum_node")

        # Récupération des paramètres
        self.sensor_number = self.declare_parameter('sensor_number', -1).value
        self.pin = self.declare_parameter('gpio_pin', -1).value
        callback_delay_second = self.declare_parameter('callback_delay_second', -1.0).value

        if self.sensor_number == -1:
            raise ValueError("Parameter 'sensor_number' must be set to a valid sensor number.")
        if self.pin == -1:
            raise ValueError("Parameter 'gpio_pin' must be set to a valid GPIO pin number.")
        if callback_delay_second == -1:
            raise ValueError("Parameter 'callback_delay_second' must be set to a valid delay in seconds.")

        # create sensor instance
        self.sensor = adafruit_dht.DHT11(board.D4)  # pin 4 BCM numbering

        # publishers for temperature and humidity
        self.temp_pub = self.create_publisher(Float32, f"/temp_hum_sensor_{self.sensor_number}/temperature", 10)
        self.hum_pub = self.create_publisher(Float32, f"/temp_hum_sensor_{self.sensor_number}/humidity", 10)

        # timer for publishing sensor values
        self.create_timer(callback_delay_second, self.send_sensor_values)

        # log
        self.get_logger().info(f'Temperature and Humidity node n°{self.sensor_number} has been started.')

    def send_sensor_values(self):
        self.sensor.measure()
        temp, hum = self.sensor.temperature, self.sensor.humidity

        if hum is not None and temp is not None:
            msg_temp = Float32()
            msg_temp.data = float(temp)
            self.temp_pub.publish(msg_temp)

            msg_hum = Float32()
            msg_hum.data = float(hum)
            self.hum_pub.publish(msg_hum)
        
            self.get_logger().info(f"Measure sensor {temp:.2f} °C and humidity {hum:.2f} %")

        else:
            self.get_logger().warning(f"Failed to read sensor n°{self.sensor_number}.")


def main(args=None):
    rclpy.init(args=args)
    temp_hum_node = TemperatureHumidityNode()

    # let the node "alive" until interrupted
    try :
        rclpy.spin(temp_hum_node)
    except KeyboardInterrupt:
        temp_hum_node.get_logger().info(f'Temperature and Humidity node n°{temp_hum_node.sensor_number} interrupted and is shutting down...')
    
    except ValueError as e:
        temp_hum_node.get_logger().error(f'[ValueError] {e}')

    finally:
        if rclpy.ok():  # if the node is still running
            time.sleep(1)  # wait for logs to be sent
            rclpy.shutdown()

