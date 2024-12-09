from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription

import os

def generate_launch_description():
    urdf_package_path = DeclareLaunchArgument(
        'urdf_package_path',
        description='The path to the robot description relative to the package root',
        default_value=os.path.join(get_package_share_directory('rover_line_follower'), 'urdf', 'simple_rover.urdf.xacro')
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('urdf_sim_tutorial'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={
            'urdf_package_path': LaunchConfiguration('urdf_package_path'),
            'world': 'empty'}.items()
    )
    
    xrce_agent_process = ExecuteProcess(
        cmd=[FindExecutable(name='MicroXRCEAgent'), 'udp4', '-p', '8888'],
        output='screen',
    )
    
    return LaunchDescription([
        urdf_package_path,
        xrce_agent_process,
        gazebo_node
    ])