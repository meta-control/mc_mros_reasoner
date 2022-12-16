import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_mros2_mock_path = get_package_share_directory(
        'mros2_mock')

    mock_node = Node(
        package='mros2_mock',
        executable='mock_objective.py',
        parameters=[{
            'fake_param': 1.0,
            'mock_param': 1.0,
        }]
    )

    return LaunchDescription([
        mock_node
    ])
