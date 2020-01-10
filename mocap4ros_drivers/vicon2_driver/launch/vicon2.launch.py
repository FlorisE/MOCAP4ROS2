# Copyright 2019 Intelligent Robotics Lab
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
#
# Author: David Vargas Frutos <david.vargas@urjc.es>

import os
import sys

import launch

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

import lifecycle_msgs.msg

def generate_launch_description():
    project_dir = get_package_share_directory('vicon2_driver')
    params_file = LaunchConfiguration('params_file')

    params_path = os.path.join(get_package_share_directory('vicon2_driver'), 'config', 'vicon2_driver_params.yaml')

    # print('')
    # print('params_path: ', params_path)
    # print('')

    driver_node = LifecycleNode(
        node_name='vicon2_driver_node',
        package='vicon2_driver',
        node_executable='vicon2_driver_main',
        output='screen',
        parameters=[params_path],
    )

    # Make the driver node take the 'configure' transition
    driver_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher = launch.events.matchers.matches_action(driver_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the driver node take the 'activate' transition
    driver_activate_trans_event = EmitEvent(
        event = ChangeState(
            lifecycle_node_matcher = launch.events.matchers.matches_action(driver_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
         )
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(driver_node)
    ld.add_action(driver_configure_trans_event)
    # ld.add_action(driver_activate_trans_event)

    return ld
