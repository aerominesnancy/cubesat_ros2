# launch module includes elements to launch all types of processes and actions
from launch import LaunchDescription

# launch_ros module includes elements to launch ROS 2 processes and actions
from launch_ros.actions import Node

import os

# change log format
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}\t[{name}] {severity} \t{message}'

# This function is always needed
def generate_launch_description():

  # Declare a variable Node for each node
  imu_node = Node(
    package="cubesat_pkg",
    executable="imu_node",
    parameters=[{'callback_delay_second': 1.0}]
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
    parameters=[{'sensor_number': 1, 'gpio_pin': 4, 'callback_delay_second': 2.0}]
  )

  # Add the nodes and the process to the LaunchDescription list
  ld = [imu_node,   motor_node, temp_hum_node_1]
  return LaunchDescription(ld)