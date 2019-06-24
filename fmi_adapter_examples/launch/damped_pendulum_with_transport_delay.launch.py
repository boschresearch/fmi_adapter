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
import launch.substitutions

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def generate_launch_description():
    delay_fmu_path = (
        ament_index_python.packages.get_package_share_directory('fmi_adapter_examples') +
        '/share/TransportDelay.fmu')

    delay_node = launch_ros.actions.LifecycleNode(
        package='fmi_adapter',
        node_executable='fmi_adapter_node',
        node_name='transport_delay',
        node_namespace='example',
        parameters=[{
            'fmu_path': delay_fmu_path,
            'd': 2.33}  # Set transport delay to 2.33s.
        ],
        output='screen',
        remappings=[('x', 'a')])  # Map input of transport delay node to output of pendulum node.

    configure_delay_node = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(delay_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_delay_node = launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
        lifecycle_node_matcher=launch.events.matchers.matches_action(delay_node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))

    pendulum_fmu_path = (
        ament_index_python.packages.get_package_share_directory('fmi_adapter_examples') +
        '/share/DampedPendulum.fmu')

    pendulum_node = launch_ros.actions.LifecycleNode(
        package='fmi_adapter',
        node_executable='fmi_adapter_node',
        node_name='damped_pendulum',
        node_namespace='example',
        parameters=[{
            'fmu_path': pendulum_fmu_path,
            'l': 25.0,  # Set pendulum length to 25m.
            'd': 0.01  # Reduce damping ratio (default is 0.1).
        }],
        output='screen')

    configure_pendulum_node = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(pendulum_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))

    activate_pendulum_node = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(pendulum_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))

    on_inactive_delay_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=delay_node, goal_state='inactive',
            entities=[configure_pendulum_node]))

    on_inactive_pendulum_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pendulum_node, goal_state='inactive',
            entities=[activate_delay_node]))

    on_active_delay_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=delay_node, goal_state='active',
            entities=[activate_pendulum_node]))

    description = launch.LaunchDescription()
    description.add_action(on_inactive_delay_handler)
    description.add_action(on_inactive_pendulum_handler)
    description.add_action(on_active_delay_handler)
    description.add_action(delay_node)
    description.add_action(pendulum_node)
    description.add_action(configure_delay_node)

    return description
