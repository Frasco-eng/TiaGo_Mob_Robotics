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
from launch.substitutions import PythonExpression
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.actions import  TimerAction
from launch.actions import OpaqueFunction

from launch.actions import ExecuteProcess

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
    Launch setup function to configure and launch all necessary nodes and processes for task2.
    This function sets up the Gazebo simulator, spawns the Tiago robot, initializes various
    nodes for localization, navigation, aruco marker detection, and coordinates the overall task execution.
    '''
    ##########################
    # Paths and Configurations
    ##########################

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
    map_path = os.path.join(pkg_dir, f"maps/{world_name}_{map_name}.yaml")

    # Data path
    marker_file_path = os.path.join(pkg_dir, 'data/marker_locations.csv')
    
    # Coordinates generation
    coordinates = utils.map_pose_random_generator(map_path, task_name='task2')  

    ######################
    # Launch Descriptions
    ######################

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
        launch_arguments={'use_sim_time': 'True', 'x': str(coordinates[0]), 'y': str(coordinates[1]), 'z': str(coordinates[2]), 'yaw': str(coordinates[3])}.items())

    # Tiago bringup launch 
    tiago_bringup = include_launch_py_description(
        'tiago_bringup', ['launch', 'tiago_bringup.launch.py'],
        launch_arguments={'use_sim_time': 'True'}.items())
    
    # Tiago nav bringup launch 
    tiago_nav_bringup = include_launch_py_description(
        'tiago_2dnav', ['launch', 'tiago_nav_bringup.launch.py'],
        launch_arguments={'use_sim_time' : 'True', 'is_public_sim' : 'True', 'rviz' : 'True', 'map_path' : map_path ,'world_name': LaunchConfiguration("world_name")}.items())

    # Moveit2 bringup launch 
    move_group = include_launch_py_description(
        'tiago_moveit_config', ['launch', 'move_group.launch.py'],
        launch_arguments={
            'use_sim_time': 'True',
            'arm_type': 'tiago-arm',        # Selects 'tiago' 
            'end_effector': 'pal-gripper',  # Selects 'pal-gripper'
            'ft_sensor': 'schunk-ft',       # Selects 'schunk-ft'
        }.items(),
        condition=IfCondition('True'))

    # Tuck arm node 
    tuck_arm = Node(package='tiago_exam',
                    executable='tuck_arm',
                    emulate_tty=True,
                    output='both',
                    parameters=[{'use_sim_time': True}])
    
    # Safe delay before tucking the arm
    delayed_tuck_arm = TimerAction(
        period=8.0,
        actions=[tuck_arm]
    )

    # Localization node 
    localization = Node(
        package='tiago_exam',
        executable='localization',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    # Pick aruco_marker detector 26
    aruco_pick = Node(
        package="aruco_ros",
        executable="single",
        name="aruco_pick",
        parameters=[{
            "image_is_rectified": True,
            "marker_size": 0.25,
            "marker_id": 26,
            "camera_frame": "head_front_camera_rgb_optical_frame", # Frame of the camera
            "marker_frame": "marker_pick_frame", # Frame of the marker
        }],
        remappings=[ # Remap topics 
            ("/camera_info", "/head_front_camera/rgb/camera_info"),
            ("/image", "/head_front_camera/rgb/image_raw"),
            ("/aruco_single/pose", "/aruco_pick/pose")
        ],
    )

    # Place aruco_marker detector 238
    aruco_place = Node(
        package="aruco_ros",
        executable="single",
        name="aruco_place",
        parameters=[{
            "image_is_rectified": True,
            "marker_size": 0.25,
            "marker_id": 238,
            "camera_frame": "head_front_camera_rgb_optical_frame",
            "marker_frame": "marker_place_frame",
        }],
        remappings=[
            ("/camera_info", "/head_front_camera/rgb/camera_info"),
            ("/image", "/head_front_camera/rgb/image_raw"),
            ("/aruco_single/pose", "/aruco_place/pose")
        ],
    )

    # Coordinator node 
    coordinator = Node(
        package='tiago_exam',
        executable='coordinator_task2',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )
    
    # Map navigator node 
    map_navigator = Node(
        package='tiago_exam',
        executable='map_navigator',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    # Naive explorer node 
    naive_explorer = Node(
        package='tiago_exam',
        executable='naive_explorer',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True,
                    'map_path': map_path}]
    )

    # Aruco noter node 
    aruco_noter = Node(
        package='tiago_exam',
        executable='aruco_noter',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True,
                     'marker_file_path': marker_file_path}]
    )

    # Aruco tracker node 
    aruco_tracker = Node(
        package='tiago_exam',
        executable='aruco_tracker',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True,
                     'marker_file_path': marker_file_path}]
    )

    # Head motion node 
    move_head = Node(
        package='tiago_exam',
        executable='head_motion',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )
    
    
    def on_tuck_exit(event, context):
        if event.returncode == 0:
            print("Tuck arm SUCCESS. Starting exploration...")
            return [localization]
        else:
            print("Tuck arm FAILED. Exploration aborted.")
            return [] # Return empty list to do nothing

    
    # Start localization after tuck arm ends
    event_handler_tuck_to_localization = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=tuck_arm,
            on_exit=on_tuck_exit,
        )
    )
    
    # Start task2 framework after localization ends
    event_handler_localization_to_exploration = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=localization,
            on_exit=[naive_explorer, aruco_noter, aruco_tracker, coordinator, map_navigator, move_head],
        )
    )
    
    return [gazebo, tiago_spawn, tiago_bringup, move_group, tiago_nav_bringup, delayed_tuck_arm,
            event_handler_tuck_to_localization, aruco_place, aruco_pick, event_handler_localization_to_exploration]

##################################
# Generate Launch Description
##################################
def generate_launch_description():
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
    
    ld.add_action(DeclareLaunchArgument(
        'world_name',
        default_value='group29',
        description='Name of the Gazebo world to load'))
    
    ld.add_action(DeclareLaunchArgument(
        'map_name',
        default_value='env_map',
        description='Name of the map to load without extension. If empty, use default map for the world.'))
   
   

    ld.add_action(SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'))
    ld.add_action(SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'))

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))

    ld.add_action(OpaqueFunction(function=launch_setup))
    

    return ld

    #############################
    # OVERALL STRUCTURE FOR TASK2
    #############################
    # 1. Launch Gazebo with the specified world.
    # 2. Spawn the Tiago robot at a random location within the environment.
    # 3. Bring up the necessary nodes for the robot's operation, including localization, navigation, and manipulation.
    # 4. Tuck the robot's arm to ensure it is in a safe configuration for navigation.
    # 5. Start the localization node to determine the robot's initial position within the environment.
    # 6. Once localization is successful, start the exploration and mapping process, allowing the robot to navigate the environment and build a map.
    # 7. Simultaneously, start the aruco marker detection nodes to identify the locations of the pick and place markers.
    # 8. The coordinator node will manage the overall task execution, coordinating between the exploration, navigation, and manipulation tasks to ensure 
    # the robot successfully completes the pick and place operation.

