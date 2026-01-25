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

import os
from os import environ, pathsep

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_pal.include_utils import include_launch_py_description

from launch_ros.actions import Node


from launch.actions import  TimerAction

def get_model_paths(packages_names):
    model_paths = ''
    for package_name in packages_names:
        if model_paths != '':
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, 'share')

        model_paths += model_path

    return model_paths


def get_resource_paths(packages_names):
    resource_paths = ''
    for package_name in packages_names:
        if resource_paths != '':
            resource_paths += pathsep

        package_path = get_package_prefix(package_name)
        resource_paths += package_path

    return resource_paths


def generate_launch_description():

    # Declare the launch arguments
    world_name_arg = DeclareLaunchArgument("world_name", default_value="1")


    map_name_arg = DeclareLaunchArgument(
        'map_name', 
        default_value='env_map',
        description='Name of the map file to be saved'
    )

    #Include the launch descriptions

    # Gazebo launch (launch the gazebo simulator with the world_name argument for the world selection)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tiago_exam_worlds'),
            'launch'), '/pal_gazebo_exam.launch.py']),
            launch_arguments={'world_name': LaunchConfiguration("world_name")}.items()
    )

    # Tiago spawn launch (spawn the robot in gazebo)
    tiago_spawn = include_launch_py_description(
        'tiago_exam', ['launch', 'tiago_spawn.launch.py'],
        launch_arguments={'use_sim_time': 'True'}.items())

    # Tiago bringup launch (load the robot state publisher and controllers)
    tiago_bringup = include_launch_py_description(
        'tiago_bringup', ['launch', 'tiago_bringup.launch.py'],
        launch_arguments={'use_sim_time': 'True'}.items())

    # Moveit2 bringup launch (load the MoveIt2 stack)
    move_group = include_launch_py_description(
        'tiago_moveit_config', ['launch', 'move_group.launch.py'],
        launch_arguments={
            'use_sim_time': 'True',
            'arm_type': 'tiago-arm',        # Selects 'tiago' (vs 'no-arm')
            'end_effector': 'pal-gripper',  # Selects 'pal-gripper'
            'ft_sensor': 'schunk-ft',       # Selects 'schunk-ft'
        }.items(),
        condition=IfCondition('True'))

    # Tiago nav bringup launch (load the navigation2 stack)
    tiago_nav_bringup = include_launch_py_description(
        'tiago_2dnav', ['launch', 'tiago_nav_bringup.launch.py'],
        launch_arguments={'use_sim_time': 'True', 'is_public_sim': 'True', 'rviz' : 'True', 'slam': 'True', 'world_name': LaunchConfiguration("world_name")}.items())

    # Explore lite launch (load the explore_lite package for autonomous exploration)
    explore_lite_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('explore_lite'), 'launch'),
        '/explore.launch.py']),
        launch_arguments={
        'use_sim_time': 'True'}.items(),
    )


    # Tuck arm launch (load the tuck arm node to avoid collisions with the environment)
    tuck_arm = Node(package='tiago_exam',
                    executable='tuck_arm',
                    emulate_tty=True,
                    output='both',
                    parameters=[{'use_sim_time': True}])

    delayed_tuck_arm = TimerAction(
        period=8.0,
        actions=[tuck_arm]
    )

    delayed_explore_lite = TimerAction(
        period=12.0,
        actions=[explore_lite_cmd]
    )

    full_map_path = PathJoinSubstitution([
        '/home/giulio/tiago_ws/src/tiago_exam/maps',
        [LaunchConfiguration('world_name'), '_', LaunchConfiguration('map_name')]
    ])

    # Sostituiamo il save_map_on_exit con il nostro Watcher Node
    map_watcher_node = Node(
        package='tiago_exam',
        executable='map_saver_watcher',
        parameters=[{
            'map_full_path': full_map_path,
            'use_sim_time': True
        }],
        output='screen'
    )
    

    # Set the GAZEBO_MODEL_PATH and GAZEBO_RESOURCE_PATH environment variables
    packages = ['tiago_description', 'pmb2_description',
                'pal_hey5_description', 'pal_gripper_description',
                'pal_robotiq_description']
    
    # Get paths
    model_path = get_model_paths(packages)
    resource_path = get_resource_paths(packages)

    if 'GAZEBO_MODEL_PATH' in environ:
        model_path += pathsep + environ['GAZEBO_MODEL_PATH']

    if 'GAZEBO_RESOURCE_PATH' in environ:
        resource_path += pathsep + environ['GAZEBO_RESOURCE_PATH']

    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    ld.add_action(SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'))
    ld.add_action(SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'))

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))

    ld.add_action(map_name_arg)
    ld.add_action(world_name_arg)

    ld.add_action(gazebo)
    ld.add_action(tiago_spawn)
    ld.add_action(tiago_bringup)
    
    ld.add_action(tiago_nav_bringup)

    ld.add_action(move_group)

    ld.add_action(delayed_tuck_arm)

    ld.add_action(delayed_explore_lite)
    ld.add_action(map_watcher_node)

    return ld
