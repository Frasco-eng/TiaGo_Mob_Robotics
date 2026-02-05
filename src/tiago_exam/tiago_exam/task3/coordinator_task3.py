#!usr/bin/env python3

##################################
# Imports
##################################
import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist

import PyKDL as kdl

from functools import partial
# Custom Services
from tiago_exam_interfaces.srv import StartNavigation, MoveHead, ManipulatorCommand, SetCollisionObject

# Python Libraries
from enum import Enum
from dataclasses import dataclass
from typing import Optional, List
import os
import csv
import time
from threading import Thread
from tiago_exam import utils
import numpy as np


##################################
# Data Classes
##################################
@dataclass
class CubeLocation:
    """
    Data class to hold pick and place locations for a cube.

    Attributes
    ----------
    marker_id : int
        The ArUco marker ID associated with the cube.
    grasp_pose : PoseStamped
        The pose where the cube should be picked up.
    release_pose : PoseStamped
        The pose where the cube should be placed.
    """
    marker_id: int  
    grasp_pose: Optional[PoseStamped] = None
    release_pose: Optional[PoseStamped] = None

##################################
# State Class
##################################
class State(Enum):
    """
    Enumeration of the different states in the coordinator state machine.
    Each state represents a specific phase in the pick-and-place task workflow.

    Attributes
    ----------
    INIT : int
        Initial state to read pick/place locations from CSV.
    NAVIGATE_PICK : int
        State to navigate to the pick location.
    PICK : int
        State to perform the pick operation.
    NAVIGATE_PLACE : int
        State to navigate to the place location.
    PLACE : int
        State to perform the place operation.
    NEXT_CUBE : int
        State to check if there are more cubes to process.
    HOME : int
        State to return to the home position.
    DONE : int
        Final state indicating task completion.
    """
    INIT = 0                # Read pick/place locations from CSV
    NAVIGATE_PICK = 1       # Navigate to pick location
    PICK = 2                # Perform pick operation
    NAVIGATE_PLACE = 3      # Navigate to place location
    PLACE = 4               # Perform place operation
    NEXT_CUBE = 5           # Check if more cubes to process
    HOME = 6                # Return to home position
    DONE = 7                # Task completed

##################################
# Coordinator Node
##################################
class CoordinatorTask3(Node):
    def __init__(self):
        super().__init__('coordinator_task3')

        ##################################
        # Callback Group
        ##################################
        self.callback_group = ReentrantCallbackGroup()

        ##################################
        # Class Parameters
        ##################################
        ws_src_path = utils.get_package_src_directory('tiago_exam')
        map_path_dir = os.path.join(ws_src_path, 'maps')
        data_path_dir = os.path.join(ws_src_path, 'data')
        
        # Declare parameters fallback
        self.map_path = self.declare_parameter('map_path', os.path.join(map_path_dir, 'group29_env_map.yaml')).get_parameter_value().string_value
        self.marker_file_path = self.declare_parameter('marker_file_path', os.path.join(data_path_dir, 'marker_locations.csv')).get_parameter_value().string_value

        ##################################
        # State Variables
        ##################################
        self.current_state = State.INIT
        self.sub_step = 0
        self.waiting_for_service = False
        
        self.future = None
        
        self.nav_in_progress = False
        self.nav_outcome = False
        self.move_head_in_progress = False
        self.move_head_outcome = False
        self.move_manip_in_progress = False
        self.move_manip_outcome = False
        
        self.target_cube_pose = None

        # Distance from table logic variables
        self.ref_distance_to_table = None
        self.approach_threshold = 0.01
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pick_marker_frame = "target_aruco_pick_marker_frame"
        self.place_marker_frame = "target_aruco_place_marker_frame"
        self.world_pick_surface = {
            'x': -3.0, 'y': -2.7, 'z': 0.15,
            'qx': 0.0, 'qy': 0.0, 'qz': -0.7071, 'qw': 0.7071,
            'size': [1.0, 0.5, 0.3] # x, y, z dal box
        }

        self.world_place_surface = {
            'x': 0.0, 'y': -5.7, 'z': 0.15,
            'qx': 0.0, 'qy': 0.0, 'qz': -0.7071, 'qw': 0.7071,
            'size': [1.0, 0.5, 0.3]
        }
        
        
        self.latest_camera_T_marker: Optional[kdl.Frame] = None
        self.latest_aruco_frame: Optional[TransformStamped] = None

        self.base_T_marker: Optional[kdl.Frame] = None

        self.home_pose = None
        
        self.debug_period = 1.0
        
        self.timer_debug = self.create_timer(self.debug_period, self.timer_callback)

        self.detection_start_time = None
        
        ##################################
        # Cubes Variables
        ##################################
        self.cubes_id = [63, 582]                       # List of cube marker IDs to pick
        self.current_cube_idx = 0                       # The cube index currently being processed
        self.cubes_to_pick: List[CubeLocation] = []     # List of cubes to pick and place
        
        ##################################
        # Station Locations
        ##################################
        self.pick_loc = (0.0, 0.0)      # Pick location (x, y)
        self.pick_orientation = (0.0, 0.0, 0.0, 1.0)
        self.place_loc = (0.0, 0.0)     # Place location (x, y)
        self.place_orientation = (0.0, 0.0, 0.0, 1.0)
        
        ###################################
        # Frame Names
        ##################################
        self.map_frame_name = "map"
        self.base_link_frame_name = "base_link"
        self.camera_frame_name = "head_front_camera_rgb_optical_frame"
        self.target_frame_name = "aruco_marker_target_frame"

        ##################################
        # TF Setup
        ##################################
        self.tf_buffer = Buffer()                                       # TF Buffer
        self.tf_listener = TransformListener(self.tf_buffer, self)      # TF Listener
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)   # Static TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)                # Dynamic TF Broadcaster
        
        ##################################
        # Service Clients
        ##################################
        self.nav_client = self.create_client(StartNavigation, 'start_nav_srv', callback_group=self.callback_group)
        self.head_client = self.create_client(MoveHead, 'move_head_srv', callback_group=self.callback_group)
        self.manip_client = self.create_client(ManipulatorCommand, 'manipulator_command_srv', callback_group=self.callback_group)
        self.collision_client = self.create_client(SetCollisionObject, 'set_collision_object_srv', callback_group=self.callback_group)

        ##################################
        # Subscriber
        ##################################
        self.sub_63 = self.create_subscription(
            TransformStamped,
            '/aruco_detector63/transform',
            partial(self.aruco_callback, marker_id=63),
            10
        )
        self.sub_582 = self.create_subscription(
            TransformStamped,
            '/aruco_detector582/transform',
            partial(self.aruco_callback, marker_id=582),
            10
        )
        
        ##################################
        # Publisher
        ##################################
        self.manipulator_pub = self.create_publisher(String, '/manipulator/command', 10)
        
        self.get_logger().info(f'{self.get_name()} node has been initialized.')

    ##################################
    # Callbacks
    ##################################

    def aruco_callback(self, msg: TransformStamped, marker_id: int):
        '''
        Callback function for ArUco marker detection.
        Updates the latest detected marker frame if it matches the current cube being processed.
        Args:
            msg (TransformStamped): The detected marker transform message.
            marker_id (int): The ID of the detected marker.
        Returns:
            None    
        '''
        
        if self.cubes_id[self.current_cube_idx] == marker_id:
            self.latest_aruco_frame = msg 
            
            
    def nav_response_callback(self, future):
        '''
        Callback function for navigation service response.
        Updates the navigation outcome based on the service response.
        Args:
            future: The future object containing the service response.
        Returns:
            None
        '''
        try:
            response = future.result()
            self.nav_outcome = response.success
            if self.nav_outcome:
                self.get_logger().info("Callback: Navigation Succeeded.")
            else:
                self.get_logger().warn("Callback: Navigation Failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.nav_outcome = False
        finally:
            # IMPORTANT: Lower the flag so the main loop knows we are done
            self.nav_in_progress = False
            
            
    def move_head_response_callback(self, future):
        '''
        Callback function for move head service response.
        Updates the move head outcome based on the service response.
        Args:
            future: The future object containing the service response.
        Returns:
            None
        '''
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Head moved successfully.")
                self.move_head_outcome = True
            else:
                self.get_logger().warn("Failed to move head.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def move_manip_response_callback(self, future):
        '''
        Callback function for manipulator command service response.
        Updates the manipulator command outcome based on the service response.
        Args:
            future: The future object containing the service response.
        Returns:
            None
        '''
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Manipulator command executed successfully.")
                self.move_manip_outcome = True
            else:
                self.get_logger().warn("Manipulator command failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def collision_response_callback(self, future) -> None:
        '''
        Callback function for collision object service response.

        Parameters
        ----------
        future : Future
            The future object containing the service response.
        '''
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Collision object operation executed successfully.")
            else:
                self.get_logger().warn("Collision object operation failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
             
    ##################################
    # Call
    ##################################
    def call_send_nav_goal_srv(self, goal: PoseStamped):
        '''
        Call the navigation service to send a navigation goal.
        Args:
            goal (PoseStamped): The target pose for navigation.
        Returns:
            None
        '''
        nav_req = StartNavigation.Request()
        nav_req.pose = goal
        
        self.get_logger().info("Sending navigation goal (Async)...")

        self.nav_in_progress = True
        self.nav_outcome = False
    
        nav_future = self.nav_client.call_async(nav_req)
        
        nav_future.add_done_callback(self.nav_response_callback)

    def call_move_head_srv(self, pan: float, tilt: float):
        '''
        Call the move head service to position the robot's head.
        Args:
            pan (float): The pan angle for the head.
            tilt (float): The tilt angle for the head.
        Returns:
            None
        '''
        move_head_req = MoveHead.Request()
        move_head_req.pan = pan
        move_head_req.tilt = tilt
        head_future = self.head_client.call_async(move_head_req)
        
        self.move_head_in_progress = True
        self.move_head_outcome = False

        head_future.add_done_callback(self.move_head_response_callback)
    
    def call_send_manip_goal_srv(self, type_cmd: str, frame: kdl.Frame):
        '''
        Call the manipulator command service to send a manipulator command.
        Args:
            type_cmd (str): The type of manipulator command (e.g., 'PICK', 'PLACE').
            frame (kdl.Frame): The target frame for the manipulator command.
        Returns:
            None
        '''
        
        # Create a request
        req = ManipulatorCommand.Request()

        # Set command type and target pose
        req.type = type_cmd
        req.frame = self.kdl_to_transform_stamped(frame, self.base_link_frame_name, self.target_frame_name)
        
        # Send the request
        manip_future = self.manip_client.call_async(req)

        self.move_manip_in_progress = True
        self.move_manip_outcome = False

        manip_future.add_done_callback(self.move_manip_response_callback)
    
    def call_send_collision_object_srv(self, operation: str, object_id: str, position: List[float] = None, quat_xyzw: List[float] = None, size: List[float] = None) -> None:
        '''
        Call the set collision object service to add or remove a collision object.

        Parameters
        ----------
        operation : str
            The operation to perform ('ADD' or 'REMOVE').
        object_id : str
            The ID of the collision object.
        position : List[float]
            The position of the collision object [x, y, z].
        quat_xyzw : List[float]
            The orientation of the collision object as a quaternion [x, y, z, w].
        size : List[float]
            The size of the collision object [x, y, z].
        '''
        req = SetCollisionObject.Request()
        req.command = operation
        req.object_id = object_id
        req.position = position if position is not None else [0.0, 0.0, 0.0]
        req.quat_xyzw = quat_xyzw if quat_xyzw is not None else [0.0, 0.0, 0.0, 1.0]
        req.size = size if size is not None else [0.0, 0.0, 0.0]

        collision_future = self.collision_client.call_async(req)

        collision_future.add_done_callback(self.collision_response_callback)

    def timer_callback(self):
        '''
        Timer callback for periodic debug logging.
        '''
        self.get_logger().info(f"--- COORDINATOR STATE ---: {self.current_state.name}")
        
    
    ##################################
    # Method
    ##################################
    def load_station_locations(self):
        """
        Load station locations from CSV file
        """
        # Read CSV file
        try:
            with open(self.marker_file_path, 'r') as f:
                reader = csv.DictReader(f)

                for row in reader:
                    # Position
                    x = float(row['x'])
                    y = float(row['y'])
                    z = float(row['z'])

                    q = Pose().orientation

                    # Orientation
                    q.x = float(row['qx'])
                    q.y = float(row['qy'])
                    q.z = float(row['qz'])
                    q.w = float(row['qw'])

                    if 'target_aruco_pick_marker_frame' in row['name']:
                        self.pick_loc = (x, y)
                        self.pick_orientation = q

                    elif 'target_aruco_place_marker_frame' in row['name']:
                        self.place_loc = (x, y)
                        self.place_orientation = q
                    
                    # Broadcast static TF
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'map'
                    t.child_frame_id = row['name']
                    
                    t.transform.translation.x = x
                    t.transform.translation.y = y
                    t.transform.translation.z = z
                    t.transform.rotation = q
                    
                    self.tf_static_broadcaster.sendTransform(t)
            
            self.get_logger().info('Station locations loaded successfully from CSV.')
    
        except Exception as e:
            self.get_logger().error(f'Failed to load station locations: {e}')
    
    def save_home_pose(self):
        self.home_pose = PoseStamped()
        self.home_pose.header.stamp = self.get_clock().now().to_msg()
        self.home_pose.header.frame_id = 'map'
        self.home_pose.pose.orientation.w = 1.0
    
    def get_transformed_aruco_pose_stamped(self):
        '''
        Transform the latest detected ArUco marker pose to the base_link frame.

        Returns
        -------
            Aruco pose : PoseStamped  
                The transformed pose in the base_link frame, or None if transformation fails.
        '''
        if self.latest_aruco_frame is None:
            return None
            
        try:
            target_pose = PoseStamped()
            target_pose.header = self.latest_aruco_frame.header
            target_pose.pose.position.x = self.latest_aruco_frame.transform.translation.x
            target_pose.pose.position.y = self.latest_aruco_frame.transform.translation.y
            target_pose.pose.position.z = self.latest_aruco_frame.transform.translation.z
            target_pose.pose.orientation = self.latest_aruco_frame.transform.rotation

            target_pose.header.stamp = rclpy.time.Time().to_msg() 
            
            # transform = self.tf_buffer.lookup_transform(
            #     'base_link', 
            #     target_pose.header.frame_id, 
            #     rclpy.time.Time())
                
            # pose_base = do_transform_pose(target_pose.pose, transform)
            return target_pose
        except Exception as e:
            self.get_logger().warn(f"TF Transform failed: {e}")
            return None
        
    def publish_tf(self, pose_to_publish: PoseStamped) -> None:
        '''
        Publish a TF frame for the target cube based on the given pose.
        
        Parameters
        ----------
            pose_to_publish : PoseStamped
                The pose of the target cube to publish as a TF frame.
        '''

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'target_cube_frame'
        t.transform.translation.x = pose_to_publish.pose.position.x
        t.transform.translation.y = pose_to_publish.pose.position.y
        t.transform.translation.z = pose_to_publish.pose.position.z
        t.transform.rotation = pose_to_publish.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def get_base_T_marker_frame(self):
        '''
        Compute the transformation from base_link to the detected ArUco marker.
        Returns:
            kdl.Frame: The transformation from base_link to the marker, or None if transformation fails.
        '''

        try:
            self.camera_T_marker = self.transform_stamped_to_kdl(self.latest_aruco_frame)
        except Exception as e:
            self.get_logger().error(f"Failed to convert base_T_camera to KDL: {e}")
            return
    
        try:
            # lookup transform: base_link <- camera
            base_T_camera_ts = self.tf_buffer.lookup_transform(self.base_link_frame_name, self.camera_frame_name, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warning(f"Could not lookup base_link<-camera transform: {e}")
            return

        # convert base_T_camera to KDL
        try:
            base_T_camera = self.transform_stamped_to_kdl(base_T_camera_ts)
        except Exception as e:
            self.get_logger().error(f"Failed to convert base_T_camera to KDL: {e}")
            return
        
        base_T_marker = base_T_camera * self.camera_T_marker

        return base_T_marker
    
    def get_map_goal_pose(self, location: str) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = 'map'

        if location == 'pick':
            goal.pose.position.x = self.pick_loc[0]
            goal.pose.position.y = self.pick_loc[1]
            goal.pose.orientation = self.pick_orientation
        elif location == 'place':
            goal.pose.position.x = self.place_loc[0]
            goal.pose.position.y = self.place_loc[1]
            goal.pose.orientation = self.place_orientation

        return goal
    
    def get_distance_from_base(self, target_frame_id: str) -> Optional[float]:
        """
        Calculate the Euclidean distance (only XY) between base_link and the target frame.
        """
        try:
            # Look up the transform: base_link -> target_frame
            t = self.tf_buffer.lookup_transform(
                self.base_link_frame_name,
                target_frame_id,
                rclpy.time.Time()
            )
            
            dx = t.transform.translation.x
            dy = t.transform.translation.y
            
            # Euclidean distance in the ground plane
            dist = math.sqrt(dx**2 + dy**2)
            return dist
        except Exception as e:
            self.get_logger().warn(f"Impossible to compute distance from base to {target_frame_id}: {e}")
            return None
        
    def generate_collision_object(self, command: str, object_id: str) -> None:
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.header.stamp = self.get_clock().now().to_msg()

        # table_width = 1.0
        # table_depth = 0.5
        # table_height = 0.3


        # position = [self.ref_distance_to_table + (table_depth),
        #             0.0,
        #             table_height / 2]

        # quat_xyzw = [0.0,
        #              0.0,
        #              0.0,
        #              1.0]

        # table_size = [table_depth,
        #               table_width,
        #               table_height]

        # 1. Seleziona i dati corretti dal dizionario
        if 'pick' in object_id:
            data = self.world_pick_surface
            obj_id = "pick_table"
        elif 'place' in object_id:
            data = self.world_place_surface
            obj_id = "place_table"
        else:
            self.get_logger().error(f"Unknown table type: {object_id}")
            return

        # 2. Crea la PoseStamped nel frame MAP (World)
        map_pose = PoseStamped()
        map_pose.header.frame_id = "map"
        map_pose.header.stamp = self.get_clock().now().to_msg()
        
        map_pose.pose.position.x = data['x']
        map_pose.pose.position.y = data['y']
        map_pose.pose.position.z = data['z']
        map_pose.pose.orientation.x = data['qx']
        map_pose.pose.orientation.y = data['qy']
        map_pose.pose.orientation.z = data['qz']
        map_pose.pose.orientation.w = data['qw']

        # 3. TRASFORMAZIONE: Da Map -> Base_Link
        try:
            # Cerchiamo la trasformazione inversa: dove è il punto della mappa rispetto a me?
            # Target: base_link, Source: map
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                "map",
                rclpy.time.Time()) # Ultima disponibile
            
            # Applichiamo la matematica della trasformazione
            base_link_pose = do_transform_pose(map_pose.pose, transform)
            
        except Exception as e:
            self.get_logger().error(f"Failed to transform table pose from map to base_link: {e}")
            return
        
        # 4. Prepara i dati per il servizio
        # Nota: ora usiamo le coordinate TRASFORMATE
        position = [base_link_pose.position.x,
                    base_link_pose.position.y,
                    base_link_pose.position.z]

        quat_xyzw = [base_link_pose.orientation.x,
                     base_link_pose.orientation.y,
                     base_link_pose.orientation.z,
                     base_link_pose.orientation.w]
        
        table_size = data['size']

        self.call_send_collision_object_srv(command, object_id, position, quat_xyzw, table_size)
    
    def remove_collision_object(self, object_id: str) -> None:
        self.call_send_collision_object_srv("REMOVE", object_id)
        
    ##################################
    # STATE MACHINE
    ##################################   
    def run_state_machine(self) -> None:
        """Main state machine callback executed periodically."""
        
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok():
            match self.current_state:
                case State.INIT:
                    self.get_logger().debug("--- State: INIT ---")

                    # 1. Check if all required services are available
                    if not self.nav_client.service_is_ready() or \
                    not self.head_client.service_is_ready() or \
                    not self.manip_client.service_is_ready():
                        
                        self.get_logger().info("Waiting for services to be available...", throttle_duration_sec=2.0)
                        continue

                    self.get_logger().info("All services are ready.")
                    
                    # Save the home pose
                    self.save_home_pose()

                    # Load station locations from CSV
                    self.load_station_locations()   

                    self.get_logger().info(f'Pick Location: {self.pick_loc}, Place Location: {self.place_loc}')

                    # Load cubes to pick
                    for cube_id in self.cubes_id:
                        self.cubes_to_pick.append(CubeLocation(marker_id=cube_id))
                        self.get_logger().info(f'Added cube with marker ID {cube_id} to the pick list.')

                    # Look for cubes to pick
                    if self.cubes_to_pick:
                        self.current_state = State.NAVIGATE_PICK
                    else:
                        self.current_state = State.DONE
                        self.get_logger().error('No cubes to pick?')
            
                case State.NAVIGATE_PICK:
                    self.get_logger().debug("--- State: NAVIGATE_PICK ---")

                    if self.current_cube_idx < len(self.cubes_to_pick):
        
                        cube = self.cubes_to_pick[self.current_cube_idx]
                        
                        if not self.nav_in_progress and not self.nav_outcome:
                            
                            goal = self.get_map_goal_pose('pick')
                            
                            self.call_send_nav_goal_srv(goal)
                            self.get_logger().info(f'Navigating to pick location for cube {cube.marker_id}')
                            
                        
                        elif self.nav_in_progress and not self.nav_outcome:
                            self.get_logger().info("Navigation goal in progress...")
                            time.sleep(self.debug_period)
                                
                        else: 
                            self.get_logger().info("Arrived at Pick Station.")
                            self.current_state = State.PICK
                            self.sub_step = 0
                            self.nav_outcome = False
                            self.nav_in_progress = False
            
                    else:
                        self.current_state = State.HOME
            
                case State.PICK:
                    
                    # --- MODIFICA 2: Lettura e Salvataggio Distanza ---
                    # Lo facciamo solo una volta all'inizio dello stato PICK
                    if self.ref_distance_to_table is None:
                        # Leggiamo la distanza reale tra robot e tavolo di pick
                        dist = self.get_distance_from_base(self.pick_marker_frame)
                        
                        if dist is not None:
                            self.ref_distance_to_table = dist
                            self.get_logger().info(f"REFERENCE DISTANCE SAVED: {self.ref_distance_to_table:.3f} meters")
                            
                            # OPZIONALE: Manda comando al manipulator per aggiungere collision object
                            
                            
                            # Invia comando per AGGIUNGERE il collision object
                            # self.call_send_manip_goal_srv(type_cmd="ADD_COLLISION_TABLE", frame=collision_frame)
                        else:
                            self.get_logger().warn("Could not read distance! Using default.")
                            self.ref_distance_to_table = 0.14 # Default di sicurezza



                        self.generate_collision_object("ADD", "pick_table")






                    self.get_logger().debug("--- State: PICK ---")
                    
                    if not self.move_head_in_progress and not self.move_head_outcome:
                        self.get_logger().info("Positioning head for pick...")
                        self.call_move_head_srv(pan=0.0, tilt=-0.9)

                    elif self.move_head_in_progress and not self.move_head_outcome:
                        self.get_logger().info("Head movement in progress...")
                        time.sleep(self.debug_period)
                        
                    else:
                        if self.detection_start_time is None:
                            self.detection_start_time = time.time() 
                        
                        if not self.move_manip_in_progress and not self.move_manip_outcome:
                            self.get_logger().info("Head positioned. Preparing for pick...")
                            
                            if self.latest_aruco_frame is not None:
                                grasp_pose = self.get_transformed_aruco_pose_stamped()

                                self.cubes_to_pick[self.current_cube_idx].grasp_pose = grasp_pose

                                if grasp_pose:
                                    self.target_cube_pose = grasp_pose # PoseStamped
                                    self.get_logger().info(f"Cube found at (base_link): {grasp_pose.pose.position}")
                                    
                                    # Publish TF 
                                    self.publish_tf(grasp_pose)
                                    
                                    self.get_logger().info("Sending pick command to manipulator...")

                                    # Get base_T_marker frame
                                    self.base_T_marker = self.get_base_T_marker_frame()

                                    type_cmd = 'PICK_' + str(self.cubes_to_pick[self.current_cube_idx].marker_id)

                                    self.call_send_manip_goal_srv(type_cmd=type_cmd, frame=self.base_T_marker)
                                    self.latest_aruco_frame = None
                                else:
                                    self.get_logger().warn("TF Transform Failed, retrying...")
                                    time.sleep(self.debug_period)
                            else:
                                # Timeout: se non trova nulla dopo 5 secondi
                                if time.time() - self.detection_start_time > 5.0:
                                    self.get_logger().error("Marker NOT found. Skipping cube.")
                                    self.current_state = State.NAVIGATE_PLACE
                            
                            
                        elif self.move_manip_in_progress and not self.move_manip_outcome:
                            self.get_logger().info("Manipulator command in progress...")
                            time.sleep(self.debug_period)
                            
                        else:
                            self.get_logger().info("Pick operation completed.")
                            self.remove_collision_object("pick_table")
                            
                            self.move_head_outcome = False
                            self.move_head_in_progress = False
                            self.latest_aruco_frame = None
                            self.move_manip_in_progress = False
                            self.move_manip_outcome = False
                            self.detection_start_time = None
                        
                            self.current_state = State.NAVIGATE_PLACE
                                    
                case State.NAVIGATE_PLACE:
                    self.get_logger().debug("--- State: NAVIGATE_PLACE ---")
                    
                    if not self.move_head_in_progress and not self.move_head_outcome:
                        self.call_move_head_srv(pan=0.0, tilt=0.0)
                    elif self.move_head_in_progress and not self.move_head_outcome:
                        self.get_logger().info("Head movement in progress...")
                        time.sleep(self.debug_period)
                    else:
                        
                        if self.current_cube_idx < len(self.cubes_to_pick):            
                            cube = self.cubes_to_pick[self.current_cube_idx]
                            
                            if not self.nav_in_progress and not self.nav_outcome:
                                self.get_logger().info("Head centered.")
                                
                                goal = self.get_map_goal_pose('place')
                                
                                self.call_send_nav_goal_srv(goal)
                                self.get_logger().info(f'Navigating to place location for cube {cube.marker_id}')
                            
                            elif self.nav_in_progress and not self.nav_outcome:
                                self.get_logger().info("Navigation goal in progress...")
                                time.sleep(self.debug_period)
                                    
                            else: 
                                self.get_logger().info("Arrived at Place Station, checking distance...")
                                current_dist = self.get_distance_from_base(self.place_marker_frame)
                                if current_dist is not None and self.ref_distance_to_table is not None:
                                    diff = abs(current_dist - self.ref_distance_to_table)
                                    if diff > self.approach_threshold:
                                        self.get_logger().info(f"Too far! Diff: {diff:.3f}m. Approaching...")
                                        
                                        # Comando di velocità lineare X (avanti piano)
                                        cmd = Twist()
                                        cmd.linear.x = 0.01 # 1 cm/s
                                        self.cmd_vel_pub.publish(cmd)
                                    else:
                                        # Siamo alla distanza giusta (o troppo vicini, nel dubbio ci fermiamo)
                                        self.get_logger().info(f"Target distance reached (Error: {diff:.3f}m). Stopping.")
                                        
                                        # Stop robot
                                        stop_cmd = Twist()
                                        self.cmd_vel_pub.publish(stop_cmd)
                                        
                                        # Passa allo stato successivo
                                        self.current_state = State.PLACE
                                        # Resetta flag
                                        self.nav_outcome = False
                                        self.nav_in_progress = False
                                else:
                                    self.get_logger().warn("Could not read distance! Going to PLACE state anyway.")
                                    self.current_state = State.PLACE

                                    self.nav_outcome = False
                                    self.nav_in_progress = False
                                    self.move_head_outcome = False
                                    self.move_head_in_progress = False
                            
                        else:
                            self.current_state = State.HOME
            
                case State.PLACE:
                    self.get_logger().debug("--- State: PLACE ---")

                    if not self.move_head_in_progress and not self.move_head_outcome:
                        self.get_logger().info("Positioning head for place...")
                        self.call_move_head_srv(pan=0.0, tilt=-0.7)

                    elif self.move_head_in_progress and not self.move_head_outcome:
                        self.get_logger().info("Head movement in progress...")
                        time.sleep(self.debug_period)
                    else:
                        
                        if not self.move_manip_in_progress and not self.move_manip_outcome:
                            self.get_logger().info("Head positioned. Preparing for place...")

                            self.generate_collision_object("ADD", "place_table")

                            # get x position of base_T_marker
                            # TODO: questa è hardcode
                            #self.base_T_marker.p[0] = np.clip(self.base_T_marker.p[0] + 0.4, a_min=0.0, a_max=0.7)
                            
                            # Define release pose based on grasp transform
                            release_frame = kdl.Frame(self.base_T_marker)

                            # Raise Z by 10 cm for placing
                            release_frame.p[2] += 0.1 

                            # Save release pose
                            self.cubes_to_pick[self.current_cube_idx].release_pose = self.kdl_to_pose_msg(release_frame, self.base_link_frame_name)

                            # Publish TF
                            self.publish_tf(self.cubes_to_pick[self.current_cube_idx].release_pose)
                            
                            # Write the place command
                            type_cmd = 'PLACE_' + str(self.cubes_to_pick[self.current_cube_idx].marker_id)

                            self.get_logger().info("Sending PLACE command...")

                            # Send place command
                            self.call_send_manip_goal_srv(type_cmd=type_cmd, frame=release_frame)
                    
                        elif self.move_manip_in_progress and not self.move_manip_outcome:
                            self.get_logger().info("Manipulator command in progress...")
                            time.sleep(self.debug_period)
                            
                        else:
                            self.get_logger().info("Place operation completed.")
                            
                            self.move_head_outcome = False
                            self.move_head_in_progress = False
                            self.latest_aruco_frame = None
                            self.move_manip_in_progress = False
                            self.move_manip_outcome = False

                            self.remove_collision_object("place_table")
                        
                            self.current_state = State.NEXT_CUBE     
                            
                    
                case State.NEXT_CUBE:
                    self.get_logger().debug("--- State: NEXT_CUBE ---")

                    if not self.move_head_in_progress and not self.move_head_outcome:
                        self.get_logger().info("Centering head for next cube...")
                        self.call_move_head_srv(pan=0.0, tilt=0.0)
                    elif self.move_head_in_progress and not self.move_head_outcome:
                        self.get_logger().info("Head movement in progress...")
                        time.sleep(self.debug_period)
                    else:
                        self.get_logger().info("Head centered.")

                        self.move_head_outcome = False
                        self.move_head_in_progress = False
                    
                        # Increment cube index
                        self.current_cube_idx += 1
                        self.ref_distance_to_table = None
                        if self.current_cube_idx < len(self.cubes_to_pick):
                            next_id = self.cubes_to_pick[self.current_cube_idx].marker_id
                            self.get_logger().info(f'Switching to next cube ID: {next_id}')
                            
                            self.current_state = State.NAVIGATE_PICK
                            self.get_logger().info('Moving to next cube')
                        else:
                            self.current_state = State.HOME
                            
                case State.HOME:
                    self.get_logger().debug("--- State: HOME ---")
                    
                    if not self.nav_in_progress and not self.nav_outcome:
                        self.get_logger().info("Returning home...")
                        self.call_send_nav_goal_srv(self.home_pose)
                    elif self.nav_in_progress and not self.nav_outcome:
                        self.get_logger().info("Navigation goal in progress...")
                        time.sleep(self.debug_period)
                    else:
                        self.get_logger().info("Arrived at Home.")
                        self.current_state = State.DONE
                        self.nav_outcome = False
                        self.nav_in_progress = False
                        self.move_head_outcome = False
                        self.move_head_in_progress = False

            
                case State.DONE:
                    self.get_logger().info('Task 3 completed!')
                    return 
                
            rate.sleep()       
            
                    
    #################################
    # Helper Functions
    ##################################

    def transform_stamped_to_kdl(self, ts: TransformStamped) -> kdl.Frame:
        # Translation
        t = ts.transform.translation
        pos = kdl.Vector(t.x, t.y, t.z)

        # Rotation
        q = ts.transform.rotation
        rot = kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w)

        return kdl.Frame(rot, pos)
    
    def kdl_to_transform_stamped(self, frame: kdl.Frame, parent: str, child: str) -> TransformStamped:
        t = TransformStamped()

        # Basic information
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        # Position
        x = frame.p[0]
        y = frame.p[1]
        z = frame.p[2]

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        # Rotation
        qx, qy, qz, qw = frame.M.GetQuaternion()

        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        return t
    
    def kdl_to_pose_msg(self, frame: kdl.Frame, frame_id: str) -> PoseStamped:
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = frame_id
        p.pose.position.x = frame.p.x()
        p.pose.position.y = frame.p.y()
        p.pose.position.z = frame.p.z()
        qx, qy, qz, qw = frame.M.GetQuaternion()
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p


#################################
# Main Function
##################################
def main(args=None):
    rclpy.init(args=args)
    coordinator = CoordinatorTask3()

    # Run the state machine loop in a separate thread
    state_thread = Thread(target=coordinator.run_state_machine, daemon=True)
    state_thread.start()

    # ROS 2 executor in main thread (multi-threaded for callbacks)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(coordinator)

    try:
        executor.spin() 
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()

#################################
# Entry Point
##################################
if __name__ == '__main__':
    main()