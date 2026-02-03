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
from launch.actions import TimerAction
from launch.actions import OpaqueFunction

from tiago_exam import utils

# Python Libraries
import os
from os import environ, pathsep



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
    ##############
    # Paths and Configurations
    ##############

    # Declare the launch arguments
    world_name = LaunchConfiguration("world_name").perform(context)

    # Set map default as env_map
    map_name = LaunchConfiguration("map_name").perform(context)
    if not map_name:    
        map_name = 'env_map'

    # Directories
    pkg_dir = utils.get_package_src_directory('tiago_exam')
    
    # Map path
    map_path = os.path.join(pkg_dir, f"maps/{world_name}_{map_name}.yaml")

    # Data path
    marker_file_path = os.path.join(pkg_dir, 'data/marker_locations.csv')

    # Coordinates
    safe_coords = [0.0, -1.3, 0.0, 0.0]  
    coordinates = utils.map_pose_random_generator(map_path, safe_coords=safe_coords, clearance=1.0)

    ##############
    # Launch Descriptions
    ##############

    # Gazebo launch (launch the gazebo simulator with the world_name argument for the world selection)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tiago_exam_worlds'),
            'launch'), '/pal_gazebo_exam.launch.py']),
            launch_arguments={'world_name': LaunchConfiguration("world_name")}.items()
    )

    # Tiago spawn launch (spawn the robot in gazebo at pick location)
    tiago_spawn = include_launch_py_description(
        'tiago_exam', ['launch', 'tiago_spawn.launch.py'],
        launch_arguments={'use_sim_time': 'True', 'x': str(coordinates[0]), 'y': str(coordinates[1]), 'z': f'{coordinates[2]}', 'yaw': f'{coordinates[3]}'}.items())

    # Tiago bringup launch (load the robot state publisher and controllers)
    tiago_bringup = include_launch_py_description(
        'tiago_bringup', ['launch', 'tiago_bringup.launch.py'],
        launch_arguments={'use_sim_time': 'True'}.items())
    
    # Tiago nav bringup launch (load the navigation2 stack)
    tiago_nav_bringup = include_launch_py_description(
        'tiago_2dnav', ['launch', 'tiago_nav_bringup.launch.py'],
        launch_arguments={'use_sim_time' : 'True', 'is_public_sim' : 'True', 'rviz' : 'True', 'map_path' : map_path ,'world_name': LaunchConfiguration("world_name")}.items())

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

    # Tuck arm launch (load the tuck arm node to avoid collisions with the environment)
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

    localization = Node(
        package='tiago_exam',
        executable='localization',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    # Aruco detector
    # NOTE: Il marker_id deve poter essere cambiato a seconda del marker da rilevare (63, 582)
    # NOTE: Questi numeri vengono comunicati dal coordinator
    aruco_detector63 = Node(
        package="aruco_ros",
        executable="single",
        name="aruco_detector63",
        parameters=[{
            "image_is_rectified": True,
            "marker_size": 0.07,
            "marker_id": 63,  
            "camera_frame": "head_front_camera_rgb_optical_frame",
            "marker_frame": "aruco_marker_frame_63",
        }],
        remappings=[
            ("/camera_info", "/head_front_camera/rgb/camera_info"),
            ("/image", "/head_front_camera/rgb/image_raw"),
            ("/aruco_single/pose", "/aruco_detector63/pose")
        ],
    )
    
    aruco_detector582 = Node(
        package="aruco_ros",
        executable="single",
        name="aruco_detector582",
        parameters=[{
            "image_is_rectified": True,
            "marker_size": 0.07,
            "marker_id": 582,  
            "camera_frame": "head_front_camera_rgb_optical_frame",
            "marker_frame": "aruco_marker_frame_582",
        }],
        remappings=[
            ("/camera_info", "/head_front_camera/rgb/camera_info"),
            ("/image", "/head_front_camera/rgb/image_raw"),
            ("/aruco_single/pose", "/aruco_detector582/pose")
        ],
    )

    # Coordinator node (main state machine for task3)
    coordinator = Node(
        package='tiago_exam',
        executable='coordinator_task3',
        emulate_tty=True,
        output='both',
        parameters=[{
            'use_sim_time': True,
            'marker_file_path': marker_file_path,
            'map_path': map_path
        }]
    )

    # Map Navigator node (handles navigation to pick and place locations)
    map_navigator = Node(
        package='tiago_exam',
        executable='map_navigator',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    # Manipulator node (state machine for grasp and release)
    manipulator = Node(
        package='tiago_exam',
        executable='manipulator',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    # Head Motion node (handles head movements)
    head_motion = Node(
        package='tiago_exam',
        executable='head_motion',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    event_handler_localization_pick_place = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=localization,
            on_exit=[coordinator, map_navigator, manipulator, head_motion],
        )
    )
    
    return [gazebo, tiago_spawn, tiago_bringup, tiago_nav_bringup, localization,
            move_group, delayed_tuck_arm, aruco_detector63, aruco_detector582,
            event_handler_localization_pick_place]

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
    
    # Declare launch arguments
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