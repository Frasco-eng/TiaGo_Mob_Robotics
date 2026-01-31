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

# Python Libraries
import os
from os import environ, pathsep
import numpy as np
import cv2
import yaml
import math
import random

##################################
# Functions
##################################
# TODO: trovare un modo per evitare di farci spawnare attaccati agli ostacoli, la clearance non funziona bene
def map_pose_random_generator(map_path, safe_coords=[0.0, -1.3, 0.0, 0.0], clearance=1.7) -> list[float]:
    """
    Generates a random pose (x, y, theta) on the map defined by map_path,
    ensuring the pose is in free space with a specified clearance from obstacles.
    
    Parameters
    ----------
    map_path : str
        Path to the YAML map file.
    safe_coords : list of float
        Default safe coordinates to return in case of failure [x, y, z, theta].
    clearance : float
        Minimum distance from obstacles in meters.
        
    Returns
    -------
    list of float
        Random pose [x, y, z, theta] in world coordinates.
    """
    print(f"Generating random pose from map: {map_path} with clearance {clearance}m")
    try: 
        # Load map YAML
        with open(map_path, 'r') as f:
            map_data = yaml.safe_load(f)
        
        # Extract map parameters
        resolution = map_data['resolution']
        origin = map_data['origin']

        # Load the map image
        pgm_path = os.path.join(os.path.dirname(map_path), map_data['image'])
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Map Image not found: {pgm_path}")

        # Create binary map: obstacles (0), free space (255)
        _, binary_map = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)

        # Compute erosion to ensure clearance
        kernel = np.ones((7, 7), np.uint8)
        binary_map = cv2.erode(binary_map, kernel, iterations=3) 

        # Compute Distance Transform
        dist_map = cv2.distanceTransform(binary_map, cv2.DIST_L2, 5)

        # Convert clearance from meters to pixels
        clearance_px = clearance / resolution

        # Find all pixels where distance to obstacle is > clearance
        safe_indices = np.where(dist_map >= clearance_px)
        
        # Zip indices into (row, col) pairs
        whitelist_pixels = list(zip(safe_indices[0], safe_indices[1]))

        if not whitelist_pixels:
            print("No safe pixels found. Using default.")
            return safe_coords
            
        # Pick a random pixel
        pixel_r, pixel_c = random.choice(whitelist_pixels)
        
        # Convert to world coordinates
        # Note: Coordinate systems vary. Standard ROS is:
        # world_x = origin_x + (column * resolution)
        # world_y = origin_y + ( (height - 1 - row) * resolution )
        height, width = img.shape
        world_x = origin[0] + (pixel_c * resolution)
        world_y = origin[1] + ((height - 1 - pixel_r) * resolution)
        world_theta = random.uniform(-math.pi, math.pi)

        return [world_x, world_y, 0.0, world_theta]
    
    except Exception as e:
        print(f"Error generating random pose: {e}")
        return safe_coords

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

    # Declare user home
    # TODO: LOOK FOR GET_PACKAG_DIRECTORY
    user_home = os.path.expanduser('~')
    workspace_src_path = os.path.join(user_home, 'tiago_mobile_robotics/src/tiago_exam')

    # Map path
    map_dir = os.path.join(workspace_src_path, 'maps')
    map_path = os.path.join(map_dir, f"{world_name}_env_map.yaml")

    # Data path
    data_dir = os.path.join(workspace_src_path, 'data')
    marker_file_path = os.path.join(data_dir, 'marker_locations.csv')
    
    # Coordinates generation
    # coordinates = [0.0, -1.3, 0.0, 0.0]  # Default coordinates if random generation fails
    coordinates = map_pose_random_generator(map_path)  

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

    # Tiago spawn launch (spawn the robot in gazebo)
    tiago_spawn = include_launch_py_description(
        'tiago_exam', ['launch', 'tiago_spawn.launch.py'],
        launch_arguments={'use_sim_time': 'True', 'x': str(coordinates[0]), 'y': str(coordinates[1]), 'z': str(coordinates[2]), 'yaw': str(coordinates[3])}.items())

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

    # Localization launch (load the localization node to perform AMCL localization)
    localization = Node(
        package='tiago_exam',
        executable='localization',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    # Pick aruco_marker detector
    aruco_pick = Node(
        package="aruco_ros",
        executable="single",
        name="aruco_pick",
        parameters=[{
            "image_is_rectified": True,
            "marker_size": 0.25,
            "marker_id": 26,
            "camera_frame": "head_front_camera_rgb_optical_frame",
            "marker_frame": "marker_pick_frame",
        }],
        remappings=[
            ("/camera_info", "/head_front_camera/rgb/camera_info"),
            ("/image", "/head_front_camera/rgb/image_raw"),
            ("/aruco_single/pose", "/aruco_pick/pose")
        ],
    )

    # Place aruco_marker detector
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

    # Naive explorer node (load the explorer node to spot the markers after localization)
    naive_explorer = Node(
        package='tiago_exam',
        executable='naive_explorer',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True,
                    'map_path': map_path,
                    'marker_file_path': marker_file_path}]
    )

    # Aruco noter node (load the aruco noter node to log found markers)
    aruco_noter = Node(
        package='tiago_exam',
        executable='aruco_noter',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )
    
    # Event handler to start exploration after localization ends
    event_handler_localization_exploration = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=localization,
            on_exit=[naive_explorer, aruco_noter],
        )
    )

    return [gazebo, tiago_spawn, tiago_bringup, tiago_nav_bringup,
            localization, move_group, delayed_tuck_arm, aruco_place, aruco_pick, event_handler_localization_exploration]

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
    
    ld.add_action(SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'))
    ld.add_action(SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'))

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))

    ld.add_action(OpaqueFunction(function=launch_setup))
    

    return ld

