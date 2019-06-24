# Copyright (c) 2019 - for information on the respective copyright owner
# see the NOTICE file and/or the repository https://github.com/boschresearch/fmi_adapter_ros2.
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

import ament_index_python.packages

import launch
import launch.actions
import launch.launch_description_sources
import launch.substitutions


def generate_launch_description():
    fmu_path = (ament_index_python.packages.get_package_share_directory('fmi_adapter_examples') +
                '/share/DampedPendulum.fmu')

    fmi_adapter_description = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ament_index_python.packages.get_package_share_directory(
                'fmi_adapter') + '/launch/fmi_adapter_node.launch.py'),
        launch_arguments={'fmu_path': fmu_path}.items())

    description = launch.LaunchDescription()
    description.add_action(fmi_adapter_description)

    return description
