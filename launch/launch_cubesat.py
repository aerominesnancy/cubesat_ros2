# launch module includes elements to launch all types of processes and actions
from launch import LaunchDescription

# launch_ros module includes elements to launch ROS 2 processes and actions
from launch_ros.actions import Node


# This function is always needed
def generate_launch_description():

  # Declare a variable Node for each node
  imu_node = Node(
    package="cubesat_pkg",
    executable="imu_node"
  )
  motor_node = Node(
    package="cubesat_pkg",
    executable="motor_node",
    parameters=[{'pin_input_1': 20, 'pin_input_2': 21, 'pwm_pin': 16}]
  )

  # Add the nodes and the process to the LaunchDescription list
  ld = [imu_node,   motor_node]
  return LaunchDescription(ld)