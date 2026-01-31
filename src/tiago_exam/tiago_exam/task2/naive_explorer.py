#!/usr/bin/env python3

##################################
# Imports
##################################
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectoryPoint
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_geometry_msgs import do_transform_pose

# Python Standard Libraries
import yaml
import os
import cv2
import numpy as np
import random
import math
import time
import datetime
from threading import Thread
from enum import Enum


##################################
# State Class
##################################
class State(Enum):
    INIT = 0
    EXPLORING = 1
    TRACKING = 2
    INSPECTING = 3
    HOMING = 4
    DONE = 5


##################################
# Helper Function
##################################
def generate_random_goal(map_path: str, num_goals: int, clearance: float = 0.4) -> tuple[list, list]:
    """
    Generates random goal coordinates within the map that are safe based on the specified clearance.

    Parameters
    ----------
    map_path : str
        The file path to the map YAML file.
    num_goals : int
        The number of random goals to generate.
    clearance : float, optional
        The minimum clearance distance from obstacles in meters, by default 0.4.

    Returns
    -------
    tuple[list, list]
        Two lists containing the x and y coordinates of the generated goals.
    """
    try:
        # Open and parse the map YAML file
        with open(map_path, 'r') as f:
            map_data = yaml.safe_load(f)

        # Extract map parameters
        resolution = map_data['resolution']
        origin = map_data['origin'] # world origin (bottom-left corner) ||| pixel origin (top-left corner)

        # Load the map image
        pgm_path = os.path.join(os.path.dirname(map_path), map_data['image'])
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None: return [], []

        # Create binary map
        _, binary_map = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)

        # Compute distance transform
        dist_map = cv2.distanceTransform(binary_map, cv2.DIST_L2, 5)

        # Identify safe pixels based on clearance
        safe_indices = np.where(dist_map >= (clearance / resolution))

        # Generate whitelist of safe pixels
        whitelist_pixels = list(zip(safe_indices[0], safe_indices[1]))
        if not whitelist_pixels: return [], []
        wx, wy = [], []
        h, w = img.shape

        # Randomly select goal pixels and convert to world coordinates
        for _ in range(num_goals):
            r, c = random.choice(whitelist_pixels) # row, col (pixel coordinates)
            wx.append(origin[0] + c * resolution) # x, y (world coordinates)
            wy.append(origin[1] + (h - 1 - r) * resolution)
        return wx, wy
    except Exception as e:
        print(f"Map Error: {e}")
        return [], []


##################################
# Naive Explorer Node
##################################
class NaiveExplorer(Node):
    def __init__(self):
        super().__init__('naive_explorer')
        
        ##################################
        # Reentrant Callback Group
        ##################################
        self.cb_group = ReentrantCallbackGroup()
        
        ##################################
        # Action Clients
        ##################################
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)
        self.head_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory', callback_group=self.cb_group)
        
        ##################################
        # Subscribers
        ##################################
        self.create_subscription(PoseStamped, 'move_to_marker', self.marker_approach_callback, 10, callback_group=self.cb_group)
        self.create_subscription(PoseStamped, 'detected_marker_real_pose', self.real_marker_callback, 10, callback_group=self.cb_group)
        
        ##################################
        # TF Setup
        ##################################
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ##################################
        # Class Parameters
        ##################################
        # Retrieve paths
        ws_src_path = os.path.join(os.path.expanduser('~'), 'tiago_gh_ws/src/tiago_exam')
        map_path_dir = os.path.join(ws_src_path, 'maps')
        data_path_dir = os.path.join(ws_src_path, 'data')
        
        # Declare parameters fallback
        self.map_path = self.declare_parameter('map_path', os.path.join(map_path_dir, 'group29_env_map.yaml')).get_parameter_value().string_value
        self.marker_file_path = self.declare_parameter('marker_file_path', os.path.join(data_path_dir, 'marker_locations.csv')).get_parameter_value().string_value
        self._init_marker_file()
        
        ##################################
        # State Variables
        ##################################
        self.state = State.INIT
        self.home_pose = None
        self.exploration_goals = []
        self.init_goals = 5
        self.clearance = 0.5
        self.current_expl_goal_idx = 0
        
        ##################################
        # Marker Tracking Variables
        ##################################
        self.found_markers = set()
        self.target_marker_pose = None
        self.target_marker_id = None
        self.is_new_marker_detected = False
        self.last_valid_real_pose = None

        self.get_logger().info("Explorer Initialized.")

    ##################################
    # Helper Methods
    ##################################
    def _init_marker_file(self):
        """
        Initializes the marker CSV file with headers.
        """
        # Ensure the directory exists
        output_folder = os.path.dirname(os.path.abspath(self.marker_file_path))
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
            self.get_logger().info(f"Folder created: {output_folder}")

        # Create or overwrite the CSV file with headers
        try:
            with open(self.marker_file_path, 'w') as f:
                f.write("timestamp,id,x,y,z,qx,qy,qz,qw\n")
            self.get_logger().info(f"File CSV initialized at: {self.marker_file_path}")
        except Exception as e:
            self.get_logger().error(f"Error initializing file: {e}")

    ##################################
    # Callbacks
    ##################################
    def marker_approach_callback(self, msg: PoseStamped) -> None:
        """
        This callback is triggered when a new marker approach goal is published. It checks if the marker has already been found.
        If it's a new marker, it updates the target marker pose and ID, and sets a flag to indicate a new marker has been detected.

        Parameters
        ----------
        msg : PoseStamped
            The pose of the detected marker to approach.
        """
        # Extract marker ID from frame_id
        marker_id = msg.header.frame_id
        
        # Ignore if already found
        if marker_id in self.found_markers:
            return

        # Spot new marker pose found
        self.target_marker_pose = msg
        self.target_marker_id = marker_id
        
        # Fix z position to 0 and frame_id to 'map' for Nav2 stack
        self.target_marker_pose.header.frame_id = 'map'
        self.target_marker_pose.pose.position.z = 0.0

        # Set new marker detected flag
        if self.state == State.EXPLORING:
            # Raise flag for the first time only to avoid multiple interrupts in the same exploration goal
            if not self.is_new_marker_detected:
                self.is_new_marker_detected = True
                self.get_logger().info(f"MARKER DETECTED: {marker_id}")

    def real_marker_callback(self, msg: PoseStamped) -> None:
        """
        This callback is triggered when a new real pose of the detected marker is published. It updates the last valid real pose.

        Parameters
        ----------
        msg : PoseStamped
            The real pose of the detected marker.
        """
        self.last_valid_real_pose = msg

    ##################################
    # Motion Methods
    ##################################
    def send_nav_goal(self, pose: PoseStamped):
        """
        Sends a navigation goal to the NAV2 stack.
        
        Parameters
        ----------
        pose : PoseStamped
            The target pose for navigation.

        Returns
        -------
        goal_handle : ActionClient.GoalHandle
            The goal handle returned by the NAV2 action server.
        """
        # Wait for the Nav2 action server to be available
        if not self.nav_client.wait_for_server(2.0):
            self.get_logger().error("Nav2 Action Server not available")
            return None
        
        # Initialize goal
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.z = 0.0
        
        # Send goal
        future = self.nav_client.send_goal_async(goal)

        # Brief wait to ensure the goal has been sent
        # TODO : It uses a timeout while which is not ideal, but avoids blocking the state machine indefinitely
        # TODO : It would be better to implement a proper async wait with a callback, that for instance sets a flag when done
        # TODO : Can be addressed by spinning the executor in a separate thread
        start = time.time()
        while not future.done():
            if time.time() - start > 2.0: return None
            time.sleep(0.05)
        
        # Get goal handle
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("Nav Goal Rejected")
            return None
        return gh

    def move_head(self, pan: float, tilt: float) -> None:
        """
        Moves the robot's head to the specified pan and tilt angles.
        
        Parameters
        ----------
        pan : float
            The pan angle in radians.
        tilt : float
            The tilt angle in radians.
        """
        # Wait for the head action server to be available
        if not self.head_client.wait_for_server(2.0): 
            self.get_logger().error("Head Action Server not available")
            return
        
        # Initialize goal
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [float(pan), float(tilt)]
        pt.time_from_start = Duration(seconds=0.5).to_msg()
        g.trajectory.points = [pt]
        
        # Send goal
        f = self.head_client.send_goal_async(g)

        # Wait for the action to complete
        while not f.done(): time.sleep(0.1)

        # Get result
        res = f.result().get_result_async()

        # Wait for result
        while not res.done(): time.sleep(0.1)

        self.get_logger().info(f"Head motion completed.")

    def move_head2(self, pan: float, tilt: float) -> None:
        # 1. Verifica rapida senza attendere troppo
        if not self.head_client.server_is_ready():
            self.get_logger().warn("Head Action Server non pronto, salto movimento")
            return
        
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        
        pt = JointTrajectoryPoint()
        pt.positions = [float(pan), float(tilt)]
        # Riduciamo il tempo per rendere la testa reattiva (es. 0.5s invece di 2s)
        pt.time_from_start = Duration(seconds=0.5).to_msg()
        g.trajectory.points = [pt]
        
        # 2. Invia e dimentica (Asincrono vero)
        # Non mettiamo i "while not f.done()" qui!
        self.head_client.send_goal_async(g)

    def generate_goals(self):
        """
        Generates random exploration goals within the map.
        """
        wx, wy = generate_random_goal(self.map_path, self.init_goals, self.clearance)
        self.exploration_goals = []
        if wx:
            for x, y in zip(wx, wy):
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.pose.position.x, p.pose.position.y = float(x), float(y)
                p.pose.orientation.w = 1.0
                self.exploration_goals.append(p)
            self.current_expl_goal_idx = 0
    
    ##################################
    # Save Marker Method
    ##################################
    def save_current_marker(self) -> None:
        """
        Saves the current target marker's real pose to the CSV file and broadcasts its static TF.
        """
        # Retrieve last valid pose
        pose = self.last_valid_real_pose
        if pose and self.target_marker_id:
            # Extract Marker Pose
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            qx = pose.pose.orientation.x
            qy = pose.pose.orientation.y
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w

            # Extract TargetMarker Pose
            x_target = self.target_marker_pose.pose.position.x
            y_target = self.target_marker_pose.pose.position.y
            z_target = self.target_marker_pose.pose.position.z
            qx_target = self.target_marker_pose.pose.orientation.x
            qy_target = self.target_marker_pose.pose.orientation.y
            qz_target = self.target_marker_pose.pose.orientation.z
            qw_target = self.target_marker_pose.pose.orientation.w

            # Extract timestamp
            ts = datetime.datetime.now().strftime("%H:%M:%S")

            # Append to CSV file
            with open(self.marker_file_path, 'a') as f:
                f.write(f"{ts},{self.target_marker_id},{x:.3f},{y:.3f},{z:.3f},{qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f}\n")
                f.write(f"{ts},target_{self.target_marker_id},{x_target:.3f},{y_target:.3f},{z_target:.3f},{qx_target:.3f},{qy_target:.3f},{qz_target:.3f},{qw_target:.3f}\n")
            
            # Broadcast static TF
            t_marker = TransformStamped()
            t_marker.header.stamp = self.get_clock().now().to_msg()
            t_marker.header.frame_id = 'map'
            t_marker.child_frame_id = str(self.target_marker_id)
            t_marker.transform.translation.x = x
            t_marker.transform.translation.y = y
            t_marker.transform.translation.z = z
            t_marker.transform.rotation = pose.pose.orientation

            t_target = TransformStamped()
            t_target.header.stamp = self.get_clock().now().to_msg()
            t_target.header.frame_id = 'map'
            t_target.child_frame_id = "target_" + str(self.target_marker_id)
            t_target.transform.translation.x = x_target
            t_target.transform.translation.y = y_target
            t_target.transform.translation.z = z_target
            t_target.transform.rotation = self.target_marker_pose.pose.orientation
            self.tf_static_broadcaster.sendTransform([t_marker, t_target])
            
            self.found_markers.add(self.target_marker_id)
            self.get_logger().info(f"SAVED: {self.target_marker_id}")

    ##################################
    # STATE MACHINE
    ##################################
    def run_state_machine(self):
        """
        Runs the main state machine for the naive explorer.
        """
        time.sleep(1.0)
        
        while rclpy.ok():
            
            match self.state:
                
                # --- INIT ---
                case State.INIT:
                    self.get_logger().info("--- State: INIT ---")
                    # Save the home pose
                    self.home_pose = PoseStamped()
                    self.home_pose.header.frame_id = 'map'
                    self.home_pose.pose.orientation.w = 1.0

                    # Move head to home
                    self.move_head(0.0, 0.0)

                    # Generate initial exploration goals
                    self.generate_goals()

                    # Switch to EXPLORING state
                    self.state = State.EXPLORING

                # --- EXPLORING ---
                case State.EXPLORING:
                    self.get_logger().info(f"--- State: EXPLORING ---")
                    # Add exploration goals if finished
                    if self.current_expl_goal_idx >= len(self.exploration_goals):
                        # If less than 2 markers found, generate new goals
                        if len(self.found_markers) < 2:
                            self.generate_goals()
                        else:
                            self.state = State.HOMING
                            continue

                    # Retrieve current exploration goal
                    goal = self.exploration_goals[self.current_expl_goal_idx]
                    self.get_logger().info(f"Exploration Goal {self.current_expl_goal_idx+1}")

                    # Send navigation goal
                    gh = self.send_nav_goal(goal)
                    
                    # Wait for result or interrupt
                    if not gh:
                        self.current_expl_goal_idx += 1
                        continue
                    
                    # Monitor goal execution
                    res_future = gh.get_result_async()

                    # Reset new marker detected flag
                    self.is_new_marker_detected = False
                    
                    
                    
                    # Wait for goal completion or interrupt
                    while not res_future.done():
                        # Check for new marker detection interrupt
                        if self.is_new_marker_detected:
                            self.get_logger().info("INTERRUPT: Canceling goal...")

                            # Cancel current goal
                            gh.cancel_goal_async()
                            
                            # Wait for nav2 to confirm cancellation
                            while not res_future.done():
                                time.sleep(0.1)

                            self.get_logger().info("Exploration canceled. Switching to TRACKING.")

                            # Switch to TRACKING state
                            self.state = State.TRACKING
                            break
                        time.sleep(0.1)
                    
                    if self.state == State.TRACKING:
                        pass 
                    else:
                        self.current_expl_goal_idx += 1

                
               
                
                
                
             
                # --- TRACKING  ---
                case State.TRACKING:
                    self.get_logger().info("--- State: TRACKING ---")
                    self.get_logger().info(f"Tracking: {self.target_marker_id}")

                    # Check if we have a valid target marker pose, otherwise return to EXPLORING
                    if not self.target_marker_pose:
                        self.state = State.EXPLORING
                        continue
                    
                    # Send Initial Goal
                    current_active_goal = self.target_marker_pose
                    gh = self.send_nav_goal(current_active_goal)
                    
                    if not gh:
                        self.state = State.EXPLORING
                        continue

                    # Monitoring loop with updates
                    nav_result_future = gh.get_result_async()
                    
                    while not nav_result_future.done():
                        # Check for updates every 0.2 seconds
                        time.sleep(0.2)
                        
                        if self.target_marker_pose:
                            # Compute distance between current goal and new marker estimate
                            dx = self.target_marker_pose.pose.position.x - current_active_goal.pose.position.x
                            dy = self.target_marker_pose.pose.position.y - current_active_goal.pose.position.y
                            
                            # Obtain camera position relative to map
                            try:
                                t = self.tf_buffer.lookup_transform('map', 'head_1_link', rclpy.time.Time())
                            except Exception as e:
                                self.get_logger().warn(f"TF Lookup Error: {e}")
                                continue
                            dx_cam = self.target_marker_pose.pose.position.x - t.transform.translation.x
                            dy_cam = self.target_marker_pose.pose.position.y - t.transform.translation.y
                            dz_cam = self.target_marker_pose.pose.position.z - t.transform.translation.z
                            dxy_cam = math.sqrt(dx_cam*dx_cam + dy_cam*dy_cam)
                            
                            
                            # # Obtain yaw of the base_link relative to map
                            # try:
                            #     t_base = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                            # except Exception as e:
                            #     self.get_logger().warn(f"TF Lookup Error: {e}")
                            #     continue
                            # q = t_base.transform.rotation
                            # siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                            # cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                            # yaw_cam = math.atan2(siny_cosp, cosy_cosp)
                            
                            
                            # self.get_logger().info(f"Marker Correction: dx={dx_cam:.2f}m, dy={dy_cam:.2f}m, dz={dz_cam:.2f}m, yaw={math.degrees(yaw_cam):.1f}deg")
                            # #Log atan values for debugging in degrees
                            # self.get_logger().info(f"atan2(dy, dx)={math.degrees(math.atan2(dy_cam, dx_cam)):.1f}deg, atan(dy, dx)={math.degrees(math.atan(dy_cam/dx_cam)):.1f}deg")

                            # #dy_cam_rot = math.cos(-yaw_cam) * dy_cam - math.sin(-yaw_cam) * dx_cam
                            # #dx_cam_rot = math.sin(-yaw_cam) * dy_cam + math.cos(-yaw_cam) * dx_cam

                            
                            # # # compute pan using dx, dy and yaw
                            # #pan_to_marker = math.atan2(dy_cam_rot, dx_cam_rot) 
                            
                            
                            # # We want to know where the map is relative to the robot's base
                            transform = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time())
                            
                            # # 2. Transform the marker pose into the robot's local frame (base_link)
                            # # This handles the robot's X, Y, Z, and Yaw/Pitch/Roll automatically
                            local_pose = do_transform_pose(self.target_marker_pose.pose, transform)
                            
                            target_x = local_pose.position.x
                            target_y = local_pose.position.y
                            # target_z = local_pose.position.z
                            
                            
                            # 3. Calculate Pan (Head Rotation)
                            # Looking at the ground plane: how much left/right from the center?
                            #pan_to_marker = math.atan2(target_y, target_x)
                            
                            # 4. Calculate Tilt (Head Pitch)
                            # We need the vertical angle relative to the head's height.
                            # Tiago's head is roughly 1.0m to 1.4m high (depending on torso).
                            # To be precise, let's find the distance in the XY plane:
                            dist_horizontal = math.sqrt(target_x**2 + target_y**2)
                            
                            # We need the target's height relative to the head joints. 
                            # For simplicity, we can use the base_link Z, but the camera is higher.
                            # Let's assume the head is ~1.1m above base_link:
                            
                            tilt_to_marker = math.atan2(dz_cam, dist_horizontal)

                            # 5. Limit the angles (Safety)
                            # Tiago Pan: ~ -1.57 to 1.57 | Tilt: ~ -0.7 to 0.7
                            #pan_to_marker = max(-1.5, min(1.5, pan_to_marker))
                            tilt_to_marker = max(-0.7, min(0.7, tilt_to_marker))

                            # self.get_logger().info(f"Target in Base: x={target_x:.2f}, y={target_y:.2f}")
                            # self.get_logger().info(f"Calculated: Pan={math.degrees(pan_to_marker):.1f}°, Tilt={math.degrees(tilt_to_marker):.1f}°")

                            # Use the non-blocking move_head2 you created to keep the loop fluid
                            self.move_head2(0.0, tilt_to_marker)
                                                    

                            
                            # # Compute distance difference
                            dist_diff = math.sqrt(dx*dx + dy*dy)

                            # # Tilt head towards marker
                            
                            # tilt_to_marker = math.atan2(dz_cam, dxy_cam)
                            # print(f"Pan to marker: {math.degrees(pan_to_marker):.1f}deg, tilt to marker: {math.degrees(tilt_to_marker):.1f}deg")
                            
                            # self.move_head(pan_to_marker, tilt_to_marker)
                            
                            # If the estimated position has changed by > 5cm, UPDATE THE GOAL!
                            if dist_diff > 0.10: 
                                self.get_logger().info(f"Refining Target (Correction: {dist_diff:.2f}m)...")
                                current_active_goal = self.target_marker_pose
                                
                                # Send a new goal which preempts (cancels) the old one automatically in Nav2
                                gh = self.send_nav_goal(current_active_goal)
                                if gh:
                                    nav_result_future = gh.get_result_async()
                                else:
                                    # If sending fails, exit
                                    break

                    # Fine tracking (success or failure of the last goal)
                    if gh:
                        # Final status check
                        if not nav_result_future.done(): 
                            # Rare case if we exit the loop due to break
                            pass
                        elif nav_result_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                            self.get_logger().info("Arrived at target.")
                            self.state = State.INSPECTING
                        else:
                            self.get_logger().warn("Tracking failed. Retrying exploration.")
                            self.state = State.EXPLORING
                    else:
                        self.state = State.EXPLORING

                # --- INSPECTING ---
                case State.INSPECTING:
                    self.get_logger().info("--- State: INSPECTING ---")
                    
                    # Save marker info
                    self.save_current_marker()

                    # Return head to home
                    self.move_head(0.0, 0.0)
                    
                    # Return to EXPLORING or HOMING based on found markers
                    if len(self.found_markers) >= 2:
                        self.state = State.HOMING
                    else:
                        self.state = State.EXPLORING

                # --- HOMING ---
                case State.HOMING:
                    self.get_logger().info("--- State: HOMING ---")
                    gh = self.send_nav_goal(self.home_pose)

                    if gh:
                        res_future = gh.get_result_async()
                        while not res_future.done():
                            time.sleep(0.1)
                        
                        if res_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                            self.get_logger().info("Returned Home Successfully.")
                        else:
                            self.get_logger().error("Failed to Return Home.")
                    
                    self.state = State.DONE

                # --- DONE ---
                case State.DONE:
                    self.get_logger().info("--- State: DONE ---")
                    # End state machine
                    break


##################################
# Main Method
##################################
def main(args=None):
    rclpy.init(args=args)
    node = NaiveExplorer()

    th = Thread(target=node.run_state_machine, daemon=True)
    th.start()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

##################################
# Entry Point
##################################
if __name__ == '__main__':
    main()
