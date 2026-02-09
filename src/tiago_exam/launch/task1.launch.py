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

##################################
# Imports
##################################
# ROS2 Libraries
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
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import OpaqueFunction


# Python Libraries
import os
from os import environ, pathsep
from tiago_exam import utils

##################################
# Functions
##################################
def get_model_paths(packages_names: list[str]) -> str:
    """
    Constructs the GAZEBO_MODEL_PATH by aggregating 'share' directories from the specified packages.

    Parameters
    ----------
    packages_names : list of str
        List of package names to include in the model path.

    Returns
    -------
    model_paths : str
        Concatenated GAZEBO_MODEL_PATH string.
    """
    model_paths = ''
    for package_name in packages_names:
        if model_paths != '':
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, 'share')

        model_paths += model_path

    return model_paths

def get_resource_paths(packages_names: list[str]) -> str:
    """
    Constructs the GAZEBO_RESOURCE_PATH by aggregating package prefixes from the specified packages.

    Parameters
    ----------
    packages_names : list of str
        List of package names to include in the resource path.

    Returns
    -------
    resource_paths : str
        Concatenated GAZEBO_RESOURCE_PATH string.
    """
    resource_paths = ''
    for package_name in packages_names:
        if resource_paths != '':
            resource_paths += pathsep

        package_path = get_package_prefix(package_name)
        resource_paths += package_path

    return resource_paths



##################################
# Launch Setup
##################################
def launch_setup(context, *args, **kwargs):
    '''
    Launch setup function to configure and launch all necessary nodes and launch files.
    '''
    ##############
    # Paths and Configurations
    ##############

    # Declare the launch arguments
    world_name = LaunchConfiguration("world_name").perform(context)
    if not world_name:    
        world_name = 'group29'

    # Set the map default as env_map
    map_name = LaunchConfiguration("map_name").perform(context)
    if not map_name:    
        map_name = 'env_map'

    # Directories
    pkg_dir = utils.get_package_src_directory('tiago_exam')
    
    # Map path
    map_path = os.path.join(pkg_dir, f"maps/{world_name}_{map_name}")
    
    ##############
    # Launch Descriptions
    ##############
    
    # Gazebo launch 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tiago_exam_worlds'),
            'launch'), '/pal_gazebo_exam.launch.py']),
            launch_arguments={'world_name': LaunchConfiguration("world_name")}.items()
    )

    # Tiago spawn launch 
    tiago_spawn = include_launch_py_description(
        'tiago_exam', ['launch', 'tiago_spawn.launch.py'],
        launch_arguments={'use_sim_time': 'True'}.items())

    # Tiago bringup launch 
    tiago_bringup = include_launch_py_description(
        'tiago_bringup', ['launch', 'tiago_bringup.launch.py'],
        launch_arguments={'use_sim_time': 'True'}.items())

    # Moveit2 bringup launch 
    move_group = include_launch_py_description(
        'tiago_moveit_config', ['launch', 'move_group.launch.py'],
        launch_arguments={
            'use_sim_time': 'True',
            'arm_type': 'tiago-arm',        # Selects 'tiago' (vs 'no-arm')
            'end_effector': 'pal-gripper',  # Selects 'pal-gripper'
            'ft_sensor': 'schunk-ft',       # Selects 'schunk-ft'
        }.items(),
        condition=IfCondition('True'))

    # Tiago nav bringup launch 
    tiago_nav_bringup = include_launch_py_description(
        'tiago_2dnav', ['launch', 'tiago_nav_bringup.launch.py'],
        launch_arguments={'use_sim_time': 'True', 'is_public_sim': 'True', 'rviz' : 'True', 'slam': 'True', 'world_name': LaunchConfiguration("world_name")}.items())

    # Explore lite launch 
    explore_lite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('explore_lite'), 'launch'),
        '/explore.launch.py']),
        launch_arguments={
        'use_sim_time': 'True'}.items(),
    )

    # Tuck arm launch 
    tuck_arm = Node(package='tiago_exam',
                    executable='tuck_arm',
                    emulate_tty=True,
                    output='both',
                    parameters=[{'use_sim_time': True}])
    
   

    delayed_tuck_arm = TimerAction(
        period=8.0,
        actions=[tuck_arm]
    )
    
    def on_tuck_exit(event, context):
        if event.returncode == 0:
            print("Tuck arm SUCCESS. Starting exploration...")
            return [explore_lite]
        else:
            print("Tuck arm FAILED. Exploration aborted.")
            return [] # Return empty list to do nothing

        
    event_handler_tuck_to_explore = RegisterEventHandler(  
            event_handler=OnProcessExit(
                target_action=tuck_arm,
                on_exit=on_tuck_exit,
            )
        )

    # Map watcher node to monitor exploration and save the map
    map_watcher_node = Node(
        package='tiago_exam',
        executable='map_saver_watcher',
        parameters=[{
            'map_full_path': map_path,
            'use_sim_time': True
        }],
        output='screen'
    )
    

    return [gazebo, tiago_spawn, tiago_bringup, tiago_nav_bringup,
            move_group, delayed_tuck_arm, event_handler_tuck_to_explore, map_watcher_node]

##################################
# Generate Launch Description
##################################
def generate_launch_description():
    '''
    Generate the launch description for the Tiago exam task 1.
    '''

    # Aggregating paths from relevant packages
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
    
    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument( # Select the world to load in Gazebo 
        'world_name',
        default_value='group29',
        description='Name of the Gazebo world to load'))
    
    ld.add_action(DeclareLaunchArgument( # Select the map to load for navigation2 
        'map_name',
        default_value='env_map',
        description='Name of the map to load without extension. If empty, use default map for the world.'))
   
    ld.add_action(SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')) # Enable colored output to see errors/warnings in different colors
    ld.add_action(SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3')) 
    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))
    ld.add_action(OpaqueFunction(function=launch_setup)) # Call the launch setup function to add all nodes and launch files
                                                         # The OpaqueFunction is used to pass the launch context to the launch_setup function

    return ld

    #############################
    # OVERALL STRUCTURE FOR TASK1
    #############################
    # 1. Launch Gazebo with the specified world
    # 2. Spawn Tiago robot in Gazebo
    # 3. Launch Tiago bringup (robot state publisher and controllers)
    # 4. Launch MoveIt2 for Tiago
    # 5. Launch Navigation2 for Tiago with SLAM enabled
    # 6. Tuck Tiago's arm to avoid collisions
    # 7. Start autonomous exploration using explore_lite after the arm is tucked
    # 8. Monitor exploration and save the map when done

    
