import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity

import time
import adafruit_dht
import board

# ros2 run cubesat_pkg temp_hum_node --ros-args -p sensor_id:=1 -p gpio_pin:=4 -p callback_delay_second:=2.0

class TemperatureHumidityNode(Node):

    def __init__(self):
        # this name is defined in the launch file (with a different index for each sensor)
        super().__init__("temp_hum_node")
        self.is_valid = True
        
        # Récupération des paramètres
        self.sensor_id = self.declare_parameter('sensor_id', -1).value
        self.pin = self.declare_parameter('gpio_pin', -1).value
        callback_delay_second = self.declare_parameter('callback_delay_second', -1.0).value

        error = False
        if self.sensor_id == -1:
            self.get_logger().fatal("Parameter 'sensor_id' must be set to a valid sensor number."
                                    + f" Current value : {self.sensor_id}")
            error = True

        if self.pin == -1:
            self.get_logger().fatal("Parameter 'gpio_pin' must be set to a valid GPIO pin number."
                                    + f" Current value : {self.pin}")
            error = True

        if callback_delay_second == -1:
            self.get_logger().fatal("Parameter 'callback_delay_second' must be set to a positive float."
                                    + f" Current value : {callback_delay_second}")
            error = True

        if error:
            self.get_logger().warn("Temperature and Humidity node is shutting down...")
            self.is_valid = False

        else:
            # create sensor instance
            self.sensor = adafruit_dht.DHT11(getattr(board, f"D{self.pin}"))  # test rpi

            # publishers for temperature and humidity
            self.temp_pub = self.create_publisher(Temperature, f"/temp_hum_sensor_{self.sensor_id}/temperature", 1)
            self.hum_pub = self.create_publisher(RelativeHumidity, f"/temp_hum_sensor_{self.sensor_id}/humidity", 1)

            # timer for publishing sensor values
            self.create_timer(callback_delay_second, self.send_sensor_values)

            # log
            self.get_logger().info(f'Temperature and Humidity node n°{self.sensor_id} has been started.')

    def send_sensor_values(self):
        try: 
            self.sensor.measure()
        except:
            self.get_logger().warn(f"Failed to read sensor n°{self.sensor_id}.")
            return
        
        temp, hum = self.sensor.temperature, self.sensor.humidity

        if hum is not None and temp is not None:
            msg_temp = Temperature()
            msg_temp.temperature = float(temp)
            msg_temp.variance = 0.0
            self.temp_pub.publish(msg_temp)

            msg_hum = RelativeHumidity()
            msg_hum.relative_humidity= float(hum)/100
            msg_hum.variance = 0.0
            self.hum_pub.publish(msg_hum)
        
            self.get_logger().info(f"Measure sensor {temp:.2f} °C and humidity {hum:.2f} %")

        else:
            self.get_logger().warn(f"Failed to read sensor n°{self.sensor_id}.")


def main(args=None):
    rclpy.init(args=args)
    temp_hum_node = TemperatureHumidityNode()
    
    # let the node "alive" until interrupted
    try :
        if temp_hum_node.is_valid:
            rclpy.spin(temp_hum_node)

    except KeyboardInterrupt:
        temp_hum_node.get_logger().warn(f'Temperature and Humidity node n°{temp_hum_node.sensor_id} interrupted and is shutting down...')

    finally:
        if rclpy.ok():  # if the node is still running
            time.sleep(1)  # wait for logs to be sent
            rclpy.shutdown()

