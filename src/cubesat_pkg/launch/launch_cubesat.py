# launch module includes elements to launch all types of processes and actions
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable

# launch_ros module includes elements to launch ROS 2 processes and actions
from launch_ros.actions import Node




# This function is always needed
def generate_launch_description():
  SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
  SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '{time}\t{severity} [{name}] \t{message}'),
  
  # Declare a variable Node for each node
  imu_node = Node(
    package="cubesat_pkg",
    executable="imu_node",
    parameters=[{'callback_delay_second': .1}]
  )

  motor_node = Node(
    package="cubesat_pkg",
    executable="motor_node",
    parameters=[{'pin_input_1': 20, 'pin_input_2': 21, 'pwm_pin': 16}]
  )

  temp_hum_node_1 = Node(
    package="cubesat_pkg",
    executable="temp_hum_node",
    name="temp_hum_sensor_1",
    parameters=[{'sensor_id': 1, 'gpio_pin': 4, 'callback_delay_second': 2.0}]
  )

  camera_node = Node(
    package="cubesat_pkg",
    executable="camera_node",
    parameters=[{'callback_delay_second': 5.0}]
  )

  # Add the nodes and the process to the LaunchDescription list
  ld = [imu_node, temp_hum_node_1, camera_node]
  return LaunchDescription(ld)