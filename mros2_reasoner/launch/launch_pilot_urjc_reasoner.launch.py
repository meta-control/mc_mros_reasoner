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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_mc_mdl_tomasys_path = get_package_share_directory('mc_mdl_tomasys')
    pkg_mros_ontology_path = get_package_share_directory('mros_ontology')
    pkg_mros2_reasoner_path = get_package_share_directory('mros2_reasoner')
    pkg_mros_ontology_path = get_package_share_directory('mros_ontology')

    tomasys_files_array = [
        os.path.join(pkg_mc_mdl_tomasys_path, 'owl', 'tomasys.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mros.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'navigation_domain.owl')]

    urjc_pilot_ontology = os.path.join(
        pkg_mros_ontology_path, 'owl', 'urjc_pilot.owl')

    mros2_launch_path = os.path.join(
        pkg_mros2_reasoner_path, 'launch_reasoner.launch.py')

    mros2_reasoner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mros2_launch_path),
        launch_arguments={
            'tomasys_file': str(tomasys_files_array),
            'model_file': urjc_pilot_ontology,
            'desired_configuration': 'f_normal_mode'}.items())

    wrapper_cmd = Node(
        package='mros2_wrapper',
        executable='mros2_wrapper',
        name='mros2_wrapper',
        output='screen'
    )

    return LaunchDescription([
     mros2_reasoner_node,
     wrapper_cmd
    ])
