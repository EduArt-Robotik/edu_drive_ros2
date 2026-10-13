import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
   
    package_path = FindPackageShare('edu_drive_ros2')
    parameter_file = PathJoinSubstitution([
      '.',
      'edu_bot.yaml'
    ])

    edu_drive = Node(
      package='edu_drive_ros2',
      executable='edu_drive_ros2_node',
      name='edu_drive_ros2_node',
      parameters=[parameter_file],
      namespace=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
      output='screen'
    )  

    joy_node = Node(
      package='joy_linux',
      executable='joy_linux_node',
      name='joy_linux_node',
      namespace=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
      output='screen',
      parameters=[{
          'autorepeat_rate': 25.0,
          #'dev_name': '/dev/input/js0'
      }],
      remappings=[
          ('joy', 'joy')
      ]
    )
    
    return LaunchDescription([
        edu_drive,
        joy_node
    ])