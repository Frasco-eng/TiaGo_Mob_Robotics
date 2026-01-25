# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node




def generate_launch_description():
    declare_x_arg = DeclareLaunchArgument(
        'x', default_value='0.0',
        description='X coordinate of the robot spawn'
    )
    declare_y_arg = DeclareLaunchArgument(
        'y', default_value='-1.3',
        description='Y coordinate of the robot spawn'
    )
    declare_z_arg = DeclareLaunchArgument(
        'z', default_value='0.0',
        description='Z coordinate of the robot spawn'
    )
    declare_yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='Yaw orientation of the robot spawn'
    )

    # old coordinates: x=2.0, y=-1.3, z=0.0, yaw=0.0

    model_name = DeclareLaunchArgument(
        'model_name', default_value='tiago',
        description='Gazebo model name'
    )

    tiago_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', LaunchConfiguration(
                                       'model_name'),
                                   '-x', LaunchConfiguration('x'),
                                   '-y', LaunchConfiguration('y'),
                                   '-z', LaunchConfiguration('z'),
                                   '-Y', LaunchConfiguration('yaw'),
                                   ],
                        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_x_arg)
    ld.add_action(declare_y_arg)
    ld.add_action(declare_z_arg)
    ld.add_action(declare_yaw_arg)

    # ld.add_action(gz_pose)
    ld.add_action(model_name)
    ld.add_action(tiago_entity)

    return ld
