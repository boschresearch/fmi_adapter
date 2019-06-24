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

import launch
import launch.actions
import launch.substitutions

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def generate_launch_description():
    launch.actions.DeclareLaunchArgument('fmu_path', description='Path to fmu')

    node = launch_ros.actions.LifecycleNode(
        package='fmi_adapter',
        node_executable='fmi_adapter_node',
        node_name='fmi_adapter_node',
        parameters=[{'fmu_path': launch.substitutions.LaunchConfiguration('fmu_path')}],
        output='screen')

    configure_node = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))

    activate_node = launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
        lifecycle_node_matcher=launch.events.matchers.matches_action(node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))

    on_inactive_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=node, goal_state='inactive',
            entities=[activate_node]))

    description = launch.LaunchDescription()
    description.add_action(on_inactive_handler)
    description.add_action(node)
    description.add_action(configure_node)

    return description
