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
import numpy as np
import cv2
import yaml
import math
import time
import random

##################################
# Functions
##################################
def map_pose_random_generator(map_path, safe_coords=[0.0, -1.3, 0.0, 0.0], clearance=1.0) -> list[float]:
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
    random.seed(time.time())
    
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

                
        
        # 1. Threshold strictly: only 255 (white) is truly free
        _, binary_map = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)

        # 2. Compute distance from obstacles (0s)
        dist_map = cv2.distanceTransform(binary_map, cv2.DIST_L2, 5)

        # 3. Convert clearance meters to pixels
        # Add a small buffer (e.g., 0.1m) to account for robot footprint
        effective_clearance_px = (clearance + 0.1) / resolution

        # 4. Filter pixels that satisfy the clearance
        safe_indices = np.where(dist_map >= effective_clearance_px)
        whitelist_pixels = list(zip(safe_indices[0], safe_indices[1]))

        # DEBUG: Save the distance map image
        debug_img = np.zeros_like(img)
        for r, c in whitelist_pixels:
            debug_img[r, c] = 255
        temp_dir_path = os.path.join(get_package_src_directory('tiago_exam'), 'temp')
        if not os.path.exists(temp_dir_path):
            os.makedirs(temp_dir_path)
        debug_image_path = os.path.join(temp_dir_path, 'debug_map.png')
        cv2.imwrite(debug_image_path, debug_img)
        print(f"Safe spawn pixels image saved to /temp/debug_safe_spawn.png")

        if not whitelist_pixels:
            print("No safe pixels found! Reducing clearance requirements...")
            # Fallback: find the max distance available if requested clearance is too high
            max_dist = np.max(dist_map)
            safe_indices = np.where(dist_map >= max_dist * 0.9)
            whitelist_pixels = list(zip(safe_indices[0], safe_indices[1]))

        # 5. Pick and Convert
        pixel_r, pixel_c = random.choice(whitelist_pixels)
        height, width = img.shape
        
        # Standard ROS Map Transform
        # world_x = origin[0] + (pixel_c * resolution) 
        # world_y = origin[1] + ((height - pixel_r) * resolution) - safe_coords[1] # Using height-pixel_r

        world_x = origin[0] + (pixel_c * resolution) 
        world_y = origin[1] + ((height - pixel_r) * resolution) + safe_coords[1] # Using height-pixel_r
        
        world_theta = random.uniform(-math.pi, math.pi) 
        
        # --- DEBUG SECTION START ---
        # Create a 3-channel BGR image for colorful markers
        debug_img_bgr = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)

        # A. Highlight Safe Zone (Light Green)
        for r, c in whitelist_pixels:
            debug_img_bgr[r, c] = [200, 255, 200] 

        # B. Draw OpenCV Origin & Axes (Top-Left)
        # CV Origin is (0,0). X is Col (Right), Y is Row (Down)
        cv2.circle(debug_img_bgr, (5, 5), 5, (0, 0, 255), -1) 
        cv2.arrowedLine(debug_img_bgr, (5, 5), (55, 5), (0, 0, 255), 2, tipLength=0.3)  # Red X
        cv2.arrowedLine(debug_img_bgr, (5, 5), (5, 55), (0, 255, 0), 2, tipLength=0.3)  # Green Y
        cv2.putText(debug_img_bgr, "CV Origin", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # C. Draw ROS World Origin & Axes
    
        ros_origin_px_x = int((0.0 - origin[0]) / resolution)
        ros_origin_px_y = int(height - ((0.0 - origin[1] - safe_coords[1]) / resolution))

        
        cv2.drawMarker(debug_img_bgr, (ros_origin_px_x, ros_origin_px_y), 
                    (255, 0, 0), cv2.MARKER_CROSS, 20, 1)
        
        # Length of axis in pixels (e.g., 1 meter)
        axis_len_px = int(1.0 / resolution)

        # Draw ROS X Axis (Blue) -> Points Right
        cv2.arrowedLine(debug_img_bgr, 
                        (ros_origin_px_x, ros_origin_px_y), 
                        (ros_origin_px_x + axis_len_px, ros_origin_px_y), 
                        (0, 0, 255), 2, tipLength=0.3)
        
        # Draw ROS Y Axis (Magenta) -> Points UP (which is decreasing row index in CV)
        cv2.arrowedLine(debug_img_bgr, 
                        (ros_origin_px_x, ros_origin_px_y), 
                        (ros_origin_px_x, ros_origin_px_y - axis_len_px), 
                        (0, 255, 0), 2, tipLength=0.3)

        cv2.putText(debug_img_bgr, "World", 
                    (ros_origin_px_x + 10, ros_origin_px_y - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # D. Mark Selected Spawn Point (Yellow)
        
        cv2.drawMarker(debug_img_bgr, (pixel_c, pixel_r), 
                    (0, 0, 0), cv2.MARKER_STAR, 20, 1)
        cv2.putText(debug_img_bgr, "Spawn", 
                    (pixel_c + 10, pixel_r - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                
    
        # Save Debug Image
        debug_image_path = os.path.join(temp_dir_path, 'debug_map_with_frames.png')
        cv2.imwrite(debug_image_path, debug_img_bgr)
        
        
        print(f"Debug map saved to: {debug_image_path}")
        # --- DEBUG SECTION END ---

        return [world_x, world_y, 0.0, world_theta]
    
    except Exception as e:
        print(f"Error generating random pose: {e}")
        return safe_coords
    
def get_package_src_directory(package_name):
    install_path = get_package_share_directory(package_name)
    
    # install_path: .../workspace/install/nome_pkg/share/nome_pkg
    workspace_path = install_path
    for _ in range(4):
        workspace_path = os.path.dirname(workspace_path)
    
    # Build the src path
    src_path = os.path.join(workspace_path, 'src', package_name)
    
    # Safety check
    if not os.path.exists(src_path):
        print(f"[WARN] Cannot find {src_path}. Are you using a standard workspace?")
        return None
        
    return src_path

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

    # Set the map default as env_map
    map_name = LaunchConfiguration("map_name").perform(context)
    if not map_name:    
        map_name = 'env_map'

    # Directories
    pkg_dir = get_package_src_directory('tiago_exam')
    
    # Map path
    map_path = os.path.join(pkg_dir, f"maps/{world_name}_{map_name}.yaml")

    # Data path
    marker_file_path = os.path.join(pkg_dir, 'data/marker_locations.csv')
    
    # Coordinates generation
    #coordinates = [0.0, -1.3, 0.0, 0.0]  # Default coordinates if random generation fails
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
        period=5.0,
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

    coordinator = Node(
        package='tiago_exam',
        executable='coordinator_task2',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )
    
    map_navigator = Node(
        package='tiago_exam',
        executable='map_navigator',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )

    # Naive explorer node (load the explorer node to generate navigation goals)
    naive_explorer = Node(
        package='tiago_exam',
        executable='naive_explorer',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True,
                    'map_path': map_path}]
    )

    # Aruco noter node (write marker detections to CSV file)
    aruco_noter = Node(
        package='tiago_exam',
        executable='aruco_noter',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True,
                     'marker_file_path': marker_file_path}]
    )

    # Aruco tracker node (track aruco markers and provide their last known pose)
    aruco_tracker = Node(
        package='tiago_exam',
        executable='aruco_tracker',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True,
                     'marker_file_path': marker_file_path}]
    )

    move_head = Node(
        package='tiago_exam',
        executable='head_motion',
        emulate_tty=True,
        output='both',
        parameters=[{'use_sim_time': True}]
    )
    
    # Event handler to start exploration after localization ends
    event_handler_localization_exploration = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=localization,
            on_exit=[naive_explorer, aruco_noter, aruco_tracker, coordinator, map_navigator, move_head],
        )
    )
    
    


    return [gazebo, tiago_spawn, tiago_bringup, tiago_nav_bringup,
            localization, move_group, aruco_place, aruco_pick, event_handler_localization_exploration, delayed_tuck_arm]

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

