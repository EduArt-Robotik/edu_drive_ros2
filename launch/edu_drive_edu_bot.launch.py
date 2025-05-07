import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
   
    package_path = FindPackageShare('edu_drive_ros2')
    parameter_file = PathJoinSubstitution([
      package_path,
      'parameter',
      'edu_drive_edu_bot.yaml'
    ])

    edu_drive = Node(
      package='edu_drive_ros2',
      executable='edu_drive_ros2_node',
      name='edu_drive_ros2_node',
      parameters=[parameter_file],
      #namespace=os.environ.get('EDU_ROBOT_NAMESPACE', "eduard"),
      output='screen'
    )  
    
    return LaunchDescription([
        edu_drive
    ])
