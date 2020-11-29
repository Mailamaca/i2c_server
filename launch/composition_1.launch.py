# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Launch a talker and a listener in a component container."""

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

def generate_launch_description():
    
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='maila',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='i2c_server',
                    plugin='i2c_server::I2CServer',
                    name='custom_i2c_server'),
                ComposableNode(
                    package='joystick_command',
                    plugin='motor::JoystickCommand',
                    name='custom_joystick_command',
                    parameters = [os.path.join(get_package_share_directory('joystick_command'),"config","defaults.yaml")]),
                ComposableNode(
                    package='motors_interface',
                    plugin='motor::MotorComponent',
                    name='custom_motors_interface',
                    parameters = [os.path.join(get_package_share_directory('motors_interface'),"config","defaults.yaml")])
                    #parameters=[{'topics.in_mode':"cucu"}])
            ],
            output='screen',
            emulate_tty=True,
    )

    return launch.LaunchDescription([container])
