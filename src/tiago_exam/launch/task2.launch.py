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
from launch.substitutions import PythonExpression

from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

from launch.actions import  TimerAction

from launch.actions import OpaqueFunction

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



def launch_setup(context, *args, **kwargs):
    # Declare the launch arguments
    world_name = LaunchConfiguration("world_name").perform(context)

    map_dir = os.path.join(get_package_share_directory('tiago_exam'), 'maps')
    map_path = os.path.join(map_dir, f"{world_name}_env_map.yaml")
    
    #coordinates = map_pose_random_generator(map_path)
    coordinates = [0.0, -1.3, 0.0, 0.0]  # Default coordinates if random generation fails
    
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
                    executable='tuck_arm.py',
                    emulate_tty=True,
                    output='both',
                    parameters=[{'use_sim_time': True}])
    
    delayed_tuck_arm = TimerAction(
        period=8.0,
        actions=[tuck_arm]
    )

    # Localization launch (load the localization node to perform AMCL localization)
    localization = Node(
        package='tiago_exam',
        executable='localization.py',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )


    # Detector per Marker PICK (es. ID 26)
    aruco_pick = Node(
        package="aruco_ros",
        executable="single",
        name="aruco_pick",
        parameters=[{
            "image_is_rectified": True,
            "marker_size": 0.25,
            "marker_id": 26,
            #"reference_frame": "map",
            "camera_frame": "head_front_camera_rgb_optical_frame",
            "marker_frame": "marker_pick_frame",
        }],
        remappings=[
            ("/camera_info", "/head_front_camera/rgb/camera_info"),
            ("/image", "/head_front_camera/rgb/image_raw"),
            ("/aruco_single/pose", "/aruco_pick/pose")
        ],
    )

    # Detector per Marker PLACE
    aruco_place = Node(
        package="aruco_ros",
        executable="single",
        name="aruco_place",
        parameters=[{
            "image_is_rectified": True,
            "marker_size": 0.25,
            "marker_id": 238,
            #"reference_frame": "map",
            "camera_frame": "head_front_camera_rgb_optical_frame",
            "marker_frame": "marker_place_frame",
        }],
        remappings=[
            ("/camera_info", "/head_front_camera/rgb/camera_info"),
            ("/image", "/head_front_camera/rgb/image_raw"),
            ("/aruco_single/pose", "/aruco_place/pose")
        ],
    )

    


    # # Il tuo nodo di ricerca custom
    # search_coordinator = Node(
    #     package='tiago_exam',
    #     executable='marker_searcher.py',
    #     emulate_tty=True,
    #     output='both',
    #     parameters=[{'use_sim_time': True}]
    # )

    naive_explorer = Node(
        package='tiago_exam',
        executable='naive_explorer.py',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True, 'map_path': map_path}]
    )

    aruco_noter = Node(
        package='tiago_exam',
        executable='aruco_noter.py',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    # aruco_place = Node(
    #     package="aruco_ros",
    #     executable="marker_publisher",
    #     name="aruco_place",
    #     parameters=[{
    #         "image_is_rectified": True,
    #         "marker_size": 0.25,
            
    #         "reference_frame": "map",
    #         "camera_frame": "head_front_camera_rgb_optical_frame",
    #         "marker_frame": "marker_place_frame",
    #     }],
    #     remappings=[
    #         ("/camera_info", "/head_front_camera/rgb/camera_info"),
    #         ("/image", "/head_front_camera/rgb/image_raw"),
    #         #("/aruco_single/pose", "/aruco_place/pose")
    #         ("/markers", "/aruco/markers")
    #     ],
    # )
    
    event_handler_localization_exploration = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=localization,
            on_exit=[naive_explorer, aruco_noter],
        )
    )


    return [gazebo, tiago_spawn, tiago_bringup, tiago_nav_bringup,
            localization, move_group, delayed_tuck_arm, aruco_place, aruco_pick, event_handler_localization_exploration]


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


#############################################################################


# From Pixel to World coordinates
    
# Image coordinates:
# px = pixel_c
# py = (self.height - 1 - pixel_r) 

# World coordinates:
# x = origin_x + (col * res)
# y = origin_y + ((height - 1 - row) * res)
        


import numpy as np
import cv2 # Using OpenCV for high-performance image processing
import yaml
import os
import math
import random

def map_pose_random_generator(map_path, safe_coords=[0.0, -1.3, 0.0, 0.0]):
    print("Generating random pose from map:", map_path)
    try: 
        with open(map_path, 'r') as f:
            map_data = yaml.safe_load(f)
            
        resolution = map_data['resolution']
        origin = map_data['origin']
        pgm_path = os.path.join(os.path.dirname(map_path), map_data['image'])
        
        # 1. Load image and threshold it
        # Map pixels: 255 (white) is free, 0 (black) is occupied
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Map Image not found: {pgm_path}")

        # 2. Create a binary mask where free space is 1 and obstacles are 0
        # In ROS maps, usually > 250 is free.
        _, binary_map = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)

        # 3. Compute Distance Transform
        # This calculates the distance from every white pixel to the nearest zero pixel
        dist_map = cv2.distanceTransform(binary_map, cv2.DIST_L2, 5)

        # 4. Convert clearance from meters to pixels
        clearance_px = 1.0 / resolution

        # 5. Find all pixels where distance to obstacle is > clearance
        safe_indices = np.where(dist_map >= clearance_px)
        
        # zip indices into (row, col) pairs
        whitelist_pixels = list(zip(safe_indices[0], safe_indices[1]))

        if not whitelist_pixels:
            print("No safe pixels found. Using default.")
            return safe_coords
            
        # 6. Pick a random pixel
        pixel_r, pixel_c = random.choice(whitelist_pixels)
        
        # 7. Convert to world coordinates
        # Note: Coordinate systems vary. Standard ROS is:
        # world_x = origin_x + (column * resolution)
        # world_y = origin_y + ( (height - 1 - row) * resolution )
        height, width = img.shape
        world_x = origin[0] + (pixel_c * resolution)
        world_y = origin[1] + ((height - 1 - pixel_r) * resolution)
        
        world_theta = random.uniform(-math.pi, math.pi)
        return [world_x, world_y, 0.0, world_theta]
    
    except Exception as e:
        print(f"Error: {e}")
        return safe_coords