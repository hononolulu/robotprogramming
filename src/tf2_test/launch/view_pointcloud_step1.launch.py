import os
import random

from jinja2 import Environment, FileSystemLoader
from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from decimal import Decimal

import launch
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart, OnProcessExit

POS_TIME = 0.3
DELAYED_NUM = 1
RL_VERSION = 0

def launch_setup(context, *args, **kwargs):

    rosbag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            os.path.join(get_package_share_directory('tf2_test'), 'config', '0904_1_mod_0.db3'),
            '--topics',
            '/camera/camera/depth/color/points',
            '/drone_0_visual_slam/odom',
        ],
        output='screen'
    )

    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('tf2_test'), 'config', 'view_pointcloud.rviz')],
        )

    nodes_to_start = [
        rviz,
        rosbag,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
