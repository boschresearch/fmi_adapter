# Copyright (c) 2019 - for information on the respective copyright owner
# see the NOTICE file and/or the repository https://github.com/boschresearch/fmi_adapter.
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
import launch.substitutions

import launch_ros.actions
import launch_ros.descriptions
import launch_ros.events
import launch_ros.events.lifecycle
import lifecycle_msgs.msg


def generate_launch_description():
    pendulum_fmu_path = (
        ament_index_python.packages.get_package_share_directory('fmi_adapter_examples') +
        '/share/DampedPendulum.fmu')

    delay_fmu_path = (
        ament_index_python.packages.get_package_share_directory('fmi_adapter_examples') +
        '/share/TransportDelay.fmu')

    node = launch_ros.actions.ComposableNodeContainer(
        node_name='fmi_adapter_nodes',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='fmi_adapter',
                node_plugin='fmi_adapter::FMIAdapterNode',
                node_namespace='/damped_pendulum',
                node_name='damped_pendulum',
                parameters=[{
                    'fmu_path': pendulum_fmu_path,
                    'l': 25.0,  # Set pendulum length to 25m.
                    'd': 0.01  # Reduce damping ratio (default is 0.1).
                }]),
            launch_ros.descriptions.ComposableNode(
                package='fmi_adapter',
                node_plugin='fmi_adapter::FMIAdapterNode',
                node_namespace='/transport_delay',
                node_name='transport_delay',
                parameters=[{
                    'fmu_path': delay_fmu_path,
                    'd': 2.33  # Set transport delay to 2.33s.
                }])
        ],
        remappings=[('/transport_delay/x', '/damped_pendulum/a')],
        output='screen'
    )

    return launch.LaunchDescription([node])
