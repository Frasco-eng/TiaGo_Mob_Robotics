#!usr/bin/env python3

##################################
# Imports
##################################
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose

# Custom Services
from exam_interfaces.srv import StartNavigation, MoveHead

# ROS2 Parameter Services (Per cambiare ID ArUco)
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType

# Python Libraries
from enum import Enum
from dataclasses import dataclass
from typing import Optional, List
import os
import csv
import time

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
    grasp_pose: PoseStamped
    release_pose: PoseStamped

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
    INIT = 0        # Read pick/place locations from CSV
    NAVIGATE_PICK = 1    # Navigate to pick location
    PICK = 2        # Perform pick operation
    NAVIGATE_PLACE = 3   # Navigate to place location
    PLACE = 4       # Perform place operation
    NEXT_CUBE = 5   # Check if more cubes to process
    HOME = 6        # Return to home position
    DONE = 7        # Task completed

##################################
# Coordinator Node
##################################
class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')

        ##################################
        # Class Parameters
        ##################################
        ws_src_path = os.path.join(os.path.expanduser('~'), 'tiago_gh_ws/src/tiago_exam')
        map_path_dir = os.path.join(ws_src_path, 'maps')
        data_path_dir = os.path.join(ws_src_path, 'data')
        
        # Declare parameters fallback
        self.map_path = self.declare_parameter('map_path', os.path.join(map_path_dir, 'group29_env_map.yaml')).get_parameter_value().string_value
        self.marker_file_path = self.declare_parameter('marker_file_path', os.path.join(data_path_dir, 'marker_locations.csv')).get_parameter_value().string_value

        ##################################
        # State Variables
        ##################################
        self.current_state = State.INIT
        self.timer = self.create_timer(0.1, self.state_machine_callback) # 10Hz
        self.sub_step = 0
        self.waiting_for_service = False
        self.waiting_for_manip = False
        self.future = None

        ##################################
        # Cubes Variables
        ##################################
        self.cubes_id = [63, 582]                       # List of cube marker IDs to pick
        self.current_cube_idx = 0                       # The cube index currently being processed
        self.cubes_to_pick: List[CubeLocation] = []     # List of cubes to pick and place
        self.latest_aruco_pose: Optional[PoseStamped] = None  # Latest detected ArUco pose

        ##################################
        # Station Locations
        ##################################
        self.pick_loc = (0.0, 0.0)      # Pick location (x, y)
        self.place_loc = (0.0, 0.0)     # Place location (x, y)
        self.height_place = 0.0         # Place height (z)

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
        self.nav_client = self.create_client(StartNavigation, 'start_nav_srv')
        self.head_client = self.create_client(MoveHead, 'move_head_srv')
        self.aruco_param_client = self.create_client(SetParameters, '/aruco_detector/set_parameters')

        ##################################
        # Subscriber
        ##################################
        # NOTE: aruco listen to a transform
        # TODO: pensa se mettere il rentalgroup callback 
        self.create_subscription(TransformStamped, '/aruco_detector/transform', self.aruco_callback, 10)
        self.manipulator_sub = self.create_subscription(Bool, '/manipulator/done', self.manip_done_cb, 10)
        
        ##################################
        # Publisher
        ##################################
        self.manipulator_pub = self.create_publisher(String, '/manipulator/command', 10)
        
        self.get_logger().info(f'{self.get_name()} node has been initialized.')

    ##################################
    # Callbacks
    ##################################
    def manip_done_cb(self, msg):
        if msg.data and self.waiting_for_manip:
            self.waiting_for_manip = False
            self.get_logger().info("Manipulator finished.")

    def aruco_callback(self, msg: TransformStamped):
        self.latest_aruco_frame = msg
        # Debug solo se stiamo cercando attivamente (per non intasare il log)
        if self.current_state == State.PICK and self.sub_step == 2:
            self.get_logger().debug(f"ArUco detected: {msg.transform.translation.x}\
                                    {msg.transform.translation.y}\
                                    {msg.transform.translation.z}")
    
    ##################################
    # Call
    ##################################
    def send_nav_goal(self, x, y):
        # Create a request
        req = StartNavigation.Request()
        req.pose.header.frame_id = 'map'
        req.pose.header.stamp = self.get_clock().now().to_msg()

        # Set position
        req.pose.pose.position.x = x
        req.pose.pose.position.y = y
        req.pose.pose.position.z = 0.0
        req.pose.pose.orientation.w = 1.0

        # Send the request
        self.future = self.nav_client.call_async(req)

        # Raise waiting for service flag
        self.waiting_for_service = True

    def send_head_goal(self, pan, tilt):
        # Create a request
        req = MoveHead.Request()

        # Set pan and tilt
        req.pan = float(pan)
        req.tilt = float(tilt)

        # Send the request
        self.future = self.head_client.call_async(req)

        # Raise waiting for service flag
        self.waiting_for_service = True

    def send_manip_cmd(self, cmd_str):
        # Create a message String
        msg = String()

        # Set command
        msg.data = cmd_str

        # Publish the command
        self.manipulator_pub.publish(msg)

        # Raise waiting for manipulator flag
        self.waiting_for_manip = True

    def set_aruco_id(self, new_id):
        """Change the ArUco marker ID parameter in the aruco_detector node."""
        req = SetParameters.Request()
        param = Parameter()
        param.name = "marker_id"
        param.value.type = ParameterType.PARAMETER_INTEGER
        param.value.integer_value = new_id
        req.parameters = [param]
        
        self.future = self.aruco_param_client.call_async(req)
        self.waiting_for_service = True

    ##################################
    # Method
    ##################################
    def load_station_locations(self) -> tuple[tuple, tuple, float]:
        """
        Load station locations from CSV file
        
        Returns
        -------
        pick_loc : tuple
            (x, y) coordinates of pick location
        place_loc : tuple
            (x, y) coordinates of place location
        height_place : float
            z coordinate (height) of place location
        """
        try:
            with open(self.marker_file_path, 'r') as f:
                reader = csv.DictReader(f)
                
                for row in reader:
                    if 'aruco_pick_marker_frame' in row.get('id', ''):
                        pick_loc = (float(row['x']), float(row['y']))
                    elif 'aruco_place_marker_frame' in row.get('id', ''):
                        place_loc = (float(row['x']), float(row['y']))
                        height_place = float(row['z'])
            
            station_marker_size = 0.25
        
            # Assuming marker size almost covering the vertical dimension of the station
            self.height_place += station_marker_size / 2.0 + 0.10 # Add 10cm safety clearance
            
            self.get_logger().info('Station locations loaded successfully from CSV.')
                    
        except Exception as e:
            self.get_logger().error(f'Failed to load station locations: {e}')
            return (0.0, 0.0), (0.0, 0.0), 0.0
        
        return pick_loc, place_loc, height_place
    
    def get_transformed_aruco_pose(self):
        if self.latest_aruco_pose is None:
            return None
            
        try:
            target_pose = self.latest_aruco_pose
            target_pose.header.stamp = rclpy.time.Time().to_msg() 
            
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                target_pose.header.frame_id, 
                rclpy.time.Time())
                
            pose_base = do_transform_pose(target_pose.pose, transform)
            return pose_base
        except Exception as e:
            self.get_logger().warn(f"TF Transform failed: {e}")
            return None
    
    ##################################
    # STATE MACHINE
    ##################################   
    def state_machine_callback(self) -> None:
        """Main state machine callback executed periodically."""

        # Waiting for service response
        if self.waiting_for_service:
            if self.future.done():
                self.waiting_for_service = False
                self.sub_step += 1
            return

        # Waiting for manipulator operation to complete
        if self.waiting_for_manip:
            if not self.waiting_for_manip:
                self.sub_step += 1
            else:
                return

        match self.current_state:
            case State.INIT:
                self.get_logger().info("--- State: INIT ---")

                # Save the home pose
                self.home_pose = PoseStamped()
                self.home_pose.header.frame_id = 'map'
                self.home_pose.pose.orientation.w = 1.0

                # Load station locations from CSV
                self.pick_loc, self.place_loc, self.height_place = self.load_station_locations()

                self.get_logger().info(f'Pick Location: {self.pick_loc}\
                                        Place Location: {self.place_loc}\
                                        Place Height: {self.height_place}')

                # Load cubes to pick
                for cube_id in self.cubes_id:
                    self.cubes_to_pick.append(CubeLocation(marker_id=cube_id))

                # Look for cubes to pick
                if self.cubes_to_pick:
                    self.current_state = State.NAVIGATE_PICK
                    self.sub_step = 0
                else:
                    self.current_state = State.DONE
                    self.get_logger().error('No cubes to pick?')
        
            case State.NAVIGATE_PICK:
                self.get_logger().info("--- State: NAVIGATE_PICK ---")

                if self.current_cube_idx < len(self.cubes_to_pick):

                    match self.sub_step:
                        # Navigate to pick location
                        case 0: 
                            cube = self.cubes_to_pick[self.current_cube_idx]
                            self.send_nav_goal(self.pick_loc[0], self.pick_loc[1])
                            self.get_logger().info(f'Navigating to pick location for cube {cube.marker_id}')

                        # Handle arrival
                        case 1: 
                            self.get_logger().info("Arrived at Pick Station.")
                            self.current_state = State.PICK
                            self.sub_step = 0
                    
                else:
                    self.current_state = State.HOME
        
            case State.PICK:
                # TODO: migliorare la logica di pick
                self.get_logger().info("--- State: PICK ---")

                # match self.sub_step:

                #     # Move head to search for ArUco
                #     case 0:
                #         self.get_logger().info("Lowering head...")
                #         self.send_head_goal(0.0, -0.8)

                #     # Start ArUco detection
                #     case 1: 
                #         self.latest_aruco_pose = None
                #         self.detection_start_time = time.time()
                #         self.sub_step += 1

                #     # Process ArUco detection
                #     case 2: 
                #         if self.latest_aruco_frame is not None:
                #             pose_base = self.get_transformed_aruco_pose()
                #             if pose_base:
                #                 self.target_cube_pose = pose_base
                #                 self.get_logger().info(f"Cube found at (base_link): {pose_base.position}")
                #                 self.sub_step += 1
                #             else:
                #                 self.get_logger().warn("TF Failed, retrying...")
                #         else:
                #             if time.time() - self.detection_start_time > 5.0:
                #                 self.get_logger().error("Marker NOT found. Skipping cube.")
                #                 self.current_state = State.NAVIGATE_PLACE
                #                 self.sub_step = 0
                    
                #     case 3:
                #         pos = self.target_cube_pose.position
                #         cmd = f"PICK:{pos.x:.3f}:{pos.y:.3f}:{pos.z:.3f}"
                #         self.get_logger().info(f"Sending grasp cmd: {cmd}")
                #         self.send_manip_cmd(cmd)

                
            case State.NAVIGATE_PLACE:
                self.get_logger().info("--- State: NAVIGATE_PLACE ---")
                if self.current_cube_idx < len(self.cubes_to_pick):

                    match self.sub_step:
                        # Navigate to place location
                        case 0:
                            cube = self.cubes_to_pick[self.current_cube_idx]
                            self.send_nav_goal(self.place_loc[0], self.place_loc[1])
                            self.get_logger().info(f'Navigating to place location for cube {cube.marker_id}')

                        # Handle arrival
                        case 1: 
                            self.get_logger().info("Arrived at Place Station.")
                            self.current_state = State.PLACE
                            self.sub_step = 0
                    
                else:
                    self.current_state = State.HOME
        
            case State.PLACE:
                self.get_logger().info("--- State: PLACE ---")
                # computa la trasformata in base a una posa ben specifica -> pubblica la transformata sul broadcaster
                # viene passato il comando di place al maniuplator (pos + orientation) -> il maniuplator implementa la state machine del lab4
        
            case State.NEXT_CUBE:
                self.get_logger().info("--- State: NEXT_CUBE ---")


                match self.sub_step:
                    case 0:
                        # Increment cube index
                        self.current_cube_idx += 1
                        if self.current_cube_idx < len(self.cubes_to_pick):
                            next_id = self.cubes_to_pick[self.current_cube_idx].marker_id
                            self.get_logger().info(f'Switching to next cube ID: {next_id}')

                            # Change aruco param
                            self.set_aruco_id(next_id)
                            self.current_state = State.NAVIGATE_PICK
                            self.get_logger().info('Moving to next cube')
                        else:
                            self.current_state = State.HOME
                            sub_step = 0
                    
                    # Param changed go ahead
                    case 1:
                        self.current_state = State.NAVIGATE_PICK
                        self.sub_step = 0

        
            case State.HOME:
                self.get_logger().info("--- State: HOME ---")

                match self.sub_step:
                    case 0:
                        self.get_logger().info("Returning home...")
                        self.send_nav_goal(self.home_pose.pose.position.x, self.home_pose.pose.position.y)
                    
                    case 1:
                        self.get_logger().info("Arrived at Home.")
                        self.current_state = State.DONE

        
            case State.DONE:
                self.get_logger().info('Task 3 completed!')
                # Optionally shutdown
                # rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    coordinator = Coordinator()
    rclpy.spin(coordinator)
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()