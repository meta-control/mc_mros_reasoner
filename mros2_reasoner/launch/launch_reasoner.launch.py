# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    tomasys_ontology_bringup_dir = get_package_share_directory(
        'mc_mdl_tomasys')
    mros_ontology_bringup_dir = get_package_share_directory('mros_ontology')

    # Create the launch configuration variables
    working_ontology_file = LaunchConfiguration('model_file')

    tomasys_files_array = [
        os.path.join(
            tomasys_ontology_bringup_dir, 'owl', 'tomasys.owl'), os.path.join(
            mros_ontology_bringup_dir, 'owl', 'mros.owl')]

    declare_working_ontology_cmd = DeclareLaunchArgument(
        'model_file',
        default_value=os.path.join(
            mros_ontology_bringup_dir,
            'owl',
            'urjc_pilot.owl'),
        description='File name for the Working ontology file')

    declare_desired_configuration_cmd = DeclareLaunchArgument(
        'desired_configuration',
        default_value='',
        description='Desired inital configuration (system mode)')

    bringup_reasoner_cmd = Node(
        package='mros2_reasoner',
        executable='mros2_reasoner_node',
        name='mros2_reasoner_node',
        output='screen',
        parameters=[{
            'tomasys_file': tomasys_files_array,
            'model_file': working_ontology_file,
        }],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_working_ontology_cmd)
    ld.add_action(declare_desired_configuration_cmd)

    # Add the actions to launch the reasoner node
    ld.add_action(bringup_reasoner_cmd)

    return ld
