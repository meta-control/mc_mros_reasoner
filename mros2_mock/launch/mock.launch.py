import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_mros2_reasoner_path = get_package_share_directory(
        'mros2_reasoner')

    mros2_launch_path = os.path.join(
        pkg_mros2_reasoner_path, 'launch_reasoner.launch.py')

    pkg_mros_ontology_path = get_package_share_directory(
        'mros_ontology')

    mock_ontology_path = os.path.join(
        pkg_mros_ontology_path, 'owl', 'mock.owl')

    mros2_reasoner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mros2_launch_path),
        launch_arguments={'model_file': mock_ontology_path}.items())


    # mission_node = Node(
    #     package='pipeline_inspection',
    #     executable='mission.py',
    #     output='screen',
    # )

    return LaunchDescription([
        mros2_reasoner_node
    ])
