#!usr/bin/env python3

##################################
# Imports
##################################
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped

# Custom Services and Messages
from tiago_exam_interfaces.srv import StartNavigation, MoveHead, InitGoals, GetNextGoal, SaveMarker
from tiago_exam_interfaces.msg import HeadPose

from std_srvs.srv import Trigger

# Python Libraries
import time
from threading import Thread
from enum import Enum
import math

##################################
# State Class
##################################
class State(Enum):
    """
    Enumeration for the different states of the coordinator state machine.

    Attributes
    ----------
    INIT : int
        Initial State
    EXPLORING : int
        Exploring the environment
    ROTATING : int
        Rotating to scan for markers
    TRACKING : int
        Tracking a detected marker
    INSPECTING : int
        Inspecting the marker
    HOMING : int
        Returning to home position
    DONE : int
        Finished task 2
    """
    INIT = 0            # Initial State
    EXPLORING = 1       # Exploring the environment
    ROTATING = 2        # Rotating to scan for markers
    TRACKING = 3        # Tracking a detected marker
    INSPECTING = 4      # Inspecting the marker
    HOMING = 5          # Returning to home position
    DONE = 6            # Finished task 2

##################################
# Coordinator Node
##################################
class CoordinatorTask2(Node):
    def __init__(self):
        super().__init__('coordinator_task2')

        ##################################
        # Callback Group
        ##################################
        self.callback_group = ReentrantCallbackGroup()

        ##################################
        # State Variables
        ##################################
        self.state = State.INIT

        ##################################
        # Head Variables
        ##################################
        self.pan = 0.0
        self.tilt = 0.0
        self.move_head_in_progress = False      # Flag to indicate if head movement is in progress

        ##################################
        # Navigation Variables
        ##################################
        self.init_goals = 20                    # Number of initial exploration goals
        self.clearance = 0.7                    # Clearance around obstacles for exploration
        self.nav_in_progress = False            # Flag to indicate if navigation is in progress
        self.nav_outcome = False                # Outcome of the last navigation attempt

        self.rotate_in_progress = False         # Flag to indicate if rotation is in progress
        self.rotate_outcome = False             # Outcome of the last rotation attempt

        ##################################
        # Coordinator Variables
        ##################################
        self.home_pose = PoseStamped()          # Home pose to return to
        self.home_pose.header.frame_id = 'map'  # Frame ID for home pose
        self.home_pose.pose.orientation.w = 1.0 # Neutral orientation
        

        ##################################
        # Marker Tracking Variables
        ##################################
        self.found_markers = set()              # Set of found marker IDs
        self.target_marker_pose = None          # Current target pose achievable for robot to navigate in front of marker
        self.target_marker_id = None            # Current target marker ID (same of marker)
        self.is_new_marker_detected = False     # Flag for new marker detection interrupt
        self.last_valid_real_pose = None        # Last valid real pose received from detection node

        ##################################
        # Service Clients
        ##################################
        self.nav_client = self.create_client(StartNavigation, 'start_nav_srv', callback_group=self.callback_group)          
        self.head_client = self.create_client(MoveHead, 'move_head_srv', callback_group=self.callback_group)
        self.init_goals_client = self.create_client(InitGoals, 'init_goals', callback_group=self.callback_group)
        self.get_next_goal_client = self.create_client(GetNextGoal, 'get_next_goal', callback_group=self.callback_group)
        self.save_marker_client = self.create_client(SaveMarker, 'save_marker_srv', callback_group=self.callback_group)
        self.rotate_client = self.create_client(Trigger, 'rotate_360', callback_group=self.callback_group)
        
        ##################################
        # Subscribers
        ##################################
        self.create_subscription(PoseStamped, 'move_to_marker', self.marker_approach_callback, 10, callback_group=self.callback_group)
        self.create_subscription(PoseStamped, 'detected_marker_real_pose', self.real_marker_callback, 10, callback_group=self.callback_group)
        self.create_subscription(HeadPose, 'head_pose', self.head_pose_callback, 10, callback_group=self.callback_group)

        ##################################
        # Debug Timer (avoid info log spam)
        ##################################
        self.debug_period = 1.0   
        self.timer = self.create_timer(self.debug_period, self.timer_callback, callback_group=self.callback_group)
    
    ##################################
    # Subscription Callbacks
    ##################################
    def marker_approach_callback(self, msg: PoseStamped) -> None:
        """
        Callback for detected marker approach poses.

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
        if self.state == State.EXPLORING or self.state == State.ROTATING:
            # Raise flag for the first time only to avoid multiple interrupts in the same exploration goal
            if not self.is_new_marker_detected:
                self.is_new_marker_detected = True
                self.get_logger().info(f"MARKER DETECTED: {marker_id}")
    
    def real_marker_callback(self, msg: PoseStamped) -> None:
        """
        Callback for receiving the real pose of the detected marker.

        Parameters
        ----------
        msg : PoseStamped
            The real pose of the detected marker.
        """
        self.last_valid_real_pose = msg

    def head_pose_callback(self, msg: HeadPose) -> None:
        """
        Callback for head pose updates.

        Parameters
        ----------
        msg : HeadPose
            The current head pose.
        """
        self.pan = msg.pan
        self.tilt = msg.tilt
    
    ##################################
    # Client Callbacks
    ##################################
    def nav_response_callback(self, future) -> None:
        """
        Callback for navigation service response.
        
        Parameters
        ----------
        future : Future
            The future object containing the service response.
        """
        try:
            response = future.result()
            self.nav_outcome = response.success
            if self.nav_outcome:
                self.get_logger().info("Callback: Navigation Succeeded.")
            else:
                if not self.nav_in_progress:
                    self.get_logger().warn("Callback: Navigation Failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.nav_outcome = False
        finally:
            self.nav_in_progress = False
        
    def save_marker_callback(self, future) -> None:
        """
        Callback for save marker service response.

        Parameters
        ----------
        future : Future
            The future object containing the service response.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Marker saved successfully.")
            else:
                self.get_logger().warn("Failed to save marker.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def move_head_response_callback(self, future) -> None:
        """
        Callback for move head service response.

        Parameters
        ----------
        future : Future
            The future object containing the service response.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Head moved successfully.")
            else:
                if not self.move_head_in_progress:
                    self.get_logger().warn("Failed to move head.")
            self.move_head_in_progress = False
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        
    def rotate_response_callback(self, future) -> None:
        """
        Callback for rotate 360 service response.

        Parameters
        ----------
        future : Future
            The future object containing the service response.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Rotation completed successfully.")
                self.rotate_outcome = True
            else:
                self.get_logger().warn("Rotation failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        finally:
            self.rotate_in_progress = False
            
    def timer_callback(self):
        """
        Timer callback to periodically log the current state.
        """
        self.get_logger().info(f"--- COORDINATOR STATE ---: {self.state.name}")

    ##################################
    # Methods
    ##################################
    def call_move_head_srv(self, pan: float, tilt: float):
        '''
        Calls the move head service asynchronously.
        Parameters
        ----------
        pan : float
            The desired pan angle.
        tilt : float
            The desired tilt angle.
        '''
        move_head_req = MoveHead.Request()
        move_head_req.pan = pan
        move_head_req.tilt = tilt
        head_future = self.head_client.call_async(move_head_req)
        
        self.move_head_in_progress = True

        head_future.add_done_callback(self.move_head_response_callback)
    
    def call_init_goals_srv(self):
        """
        Modified to be synchronous/blocking for the state machine thread.
        Returns True if successful, False otherwise.
        """
        init_goal_req = InitGoals.Request()
        init_goal_req.init_goals = self.init_goals
        init_goal_req.clearance = self.clearance
        
        # 1. Wait for the service to be available
        self.get_logger().info("Waiting for init_goals service...")
        if not self.init_goals_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("InitGoals service not available.")
            return False

        # 2. Call the service asynchronously
        goals_future = self.init_goals_client.call_async(init_goal_req)
        
        # 3. Wait for the result (Blocking the State Machine Thread, NOT the Node/Executor)
        while rclpy.ok() and not goals_future.done():
            time.sleep(0.1)
            
        # 4. Process Result
        try:
            response = goals_future.result()
            if response.success:
                self.get_logger().info("Exploration goals initialized successfully.")
                return True
            else:
                self.get_logger().warn("Service returned failure for InitGoals.")
                return False
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return False
        
    def call_send_nav_goal_srv(self, goal: PoseStamped):
        nav_req = StartNavigation.Request()
        nav_req.pose = goal
        
        self.get_logger().debug("Sending navigation goal (Async)...")

        self.nav_in_progress = True
        self.nav_outcome = False
    
        nav_future = self.nav_client.call_async(nav_req)
        
        nav_future.add_done_callback(self.nav_response_callback)

    def call_get_next_goal_srv(self):
        '''
        Calls the get next goal service synchronously.
        Returns the next goal PoseStamped and a boolean indicating if it's the last goal.
        '''

        get_goal_req = GetNextGoal.Request()
        
        # Wait for service to be available
        if not self.get_next_goal_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("GetNextGoal service not available!")
            return None, None

        # Using synchronous-like call in a separate thread is safer here
        # or keep call_async but handle the future correctly
        future = self.get_next_goal_client.call_async(get_goal_req)
        
        # In a MultiThreadedExecutor, we can wait for the future
        # while the executor continues to spin in the main thread.
        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        if future.result() is not None:
            res = future.result()
            if res.success:
                return res.next_goal, res.is_last_goal
        return None, None
    
    def call_save_marker(self, name_str, pose_obj):
        """
        Calls the save marker service.
        """
        if not self.save_marker_client.service_is_ready():
            self.get_logger().warn("SaveMarker service not ready!")
            return

        save_req = SaveMarker.Request()
        save_req.name = name_str
        save_req.pose = pose_obj

        save_future = self.save_marker_client.call_async(save_req)
        
        save_future.add_done_callback(self.save_marker_callback)
        
    def call_rotate_360_srv(self):
        """
        Calls the rotate 360 service. It is used as during naive exploration.
        """
        if not self.rotate_client.service_is_ready():
            self.get_logger().warn("Rotate360 service not ready!")
            return

        rotate_req = Trigger.Request()
        
        self.rotate_in_progress = True
        self.rotate_outcome = False

        rotate_future = self.rotate_client.call_async(rotate_req)
        
        rotate_future.add_done_callback(self.rotate_response_callback)
        
    ##################################
    # STATE MACHINE
    ##################################
    def run_state_machine(self):
        """
        Runs the main state machine for the naive explorer.
        """    
        while rclpy.ok():
            
            match self.state:
                
                # --- INIT ---
                case State.INIT:
                    self.get_logger().debug("--- State: INIT ---")

                    # Save the home pose
                    self.home_pose = PoseStamped()
                    self.home_pose.header.frame_id = 'map'
                    self.home_pose.pose.orientation.w = 1.0
                  
                    # Move head to home
                    self.call_move_head_srv(0.0, 0.0)

                    # Generate initial exploration goals
                    self.call_init_goals_srv()

                    # Switch to EXPLORING state
                    self.state = State.EXPLORING

                # --- EXPLORING ---
                case State.EXPLORING:
                    self.get_logger().debug(f"--- State: EXPLORING ---")

                    
                    if self.is_new_marker_detected:
                        self.get_logger().info("New marker detected! Switching to TRACKING.")
                        self.state = State.TRACKING
                        self.is_new_marker_detected = False
                        self.nav_outcome = False
                        self.nav_in_progress = False
                        continue       
                    
                    if not self.nav_in_progress and not self.nav_outcome:
                     
                        # Retrieve current exploration goal
                        goal, is_last_goal = self.call_get_next_goal_srv()
                        
                        if not goal:
                            self.get_logger().error("No exploration goal retrieved. Switching to HOMING.")
                            self.state = State.HOMING
                            continue
                            
                        
                        if is_last_goal:
                            self.get_logger().info("Last exploration goal reached. Switching to HOMING after this goal.")
                            self.state = State.HOMING
                            continue
                        
                        self.call_send_nav_goal_srv(goal)
                        
                    elif self.nav_in_progress and not self.nav_outcome:
                        self.get_logger().info("Exploration goal in progress...")
                        time.sleep(self.debug_period)
                        continue
                    
                    else:                         
                        self.get_logger().info("Exploration goal reached successfully, continuing exploration...")
                        # Reset flag for next goal
                        self.nav_outcome = False
                        self.nav_in_progress = False
                        self.rotate_in_progress = False
                        self.rotate_outcome = False
                        
                        self.state = State.ROTATING  # Continue exploring
                        self.rotation_steps = 0
                
                # --- ROTATING ---
                case State.ROTATING:
                    self.get_logger().debug("--- State: ROTATING ---")
                    
                    if self.is_new_marker_detected:
                        self.get_logger().info("New marker detected! Switching to TRACKING.")
                        self.state = State.TRACKING
                        self.is_new_marker_detected = False
                        self.rotate_outcome = False
                        self.rotate_in_progress = False
                        continue       
                    
                    # 1. Start Rotation
                    if not self.rotate_in_progress and not self.rotate_outcome:
                        self.get_logger().info("Starting 360-degree rotation...")
                        self.call_rotate_360_srv()
                    
                    # 2. Wait for Rotation
                    elif self.rotate_in_progress and not self.rotate_outcome:
                        time.sleep(self.debug_period)
                        continue
                    
                    # 3. Rotation Finished
                    else:
                        self.get_logger().info("Rotation completed.")
                        self.rotate_outcome = False
                        self.rotate_in_progress = False
                        
                        self.state = State.EXPLORING
                                                    
                # --- TRACKING  ---
                case State.TRACKING:
                    self.get_logger().debug("--- State: TRACKING ---")
                    
                    # Check if we have a valid target marker pose, otherwise return to EXPLORING
                    if not self.target_marker_pose:
                        self.get_logger().info("No valid target marker pose. Returning to EXPLORING.")
                        self.state = State.EXPLORING
                        continue
                    
                    if not self.nav_in_progress and not self.nav_outcome and not self.rotate_in_progress and not self.rotate_outcome:
                        self.get_logger().info(f"Tracking: {self.target_marker_id}")
                        current_active_goal = self.target_marker_pose
                        self.call_send_nav_goal_srv(current_active_goal)
                      
                    # Use the non-blocking move_head you created to keep the loop fluid
                    self.call_move_head_srv(self.pan, self.tilt)  # Look down while approaching marker
                                
                    
                    dx = self.target_marker_pose.pose.position.x - current_active_goal.pose.position.x
                    dy = self.target_marker_pose.pose.position.y - current_active_goal.pose.position.y
                            
                    dist_diff = math.sqrt(dx**2 + dy**2)
                    
                    
                    if dist_diff > 0.10: 
                        self.get_logger().info(f"Refining Target (Correction: {dist_diff:.2f}m)...")
                        current_active_goal = self.target_marker_pose
                    
                        self.call_send_nav_goal_srv(current_active_goal)
                    else:
                        self.get_logger().info("Target within acceptable range.")
                    
                        if self.nav_in_progress and not self.nav_outcome:
                            self.get_logger().info("Tracking goal in progress...")
                            time.sleep(self.debug_period)
                            continue
                        else:
                            self.get_logger().info("Tracking goal reached successfully.")
                            # Reset flags
                            self.nav_outcome = False
                            self.nav_in_progress = False
                            self.rotate_outcome = False
                            self.rotate_in_progress = False
                            
                            # Switch to INSPECTING state
                            self.state = State.INSPECTING
                
                # --- INSPECTING ---
                case State.INSPECTING:
                    self.get_logger().debug("--- State: INSPECTING ---")
                    
                    if self.last_valid_real_pose is None:
                        self.get_logger().warn("Waiting for real pose...")
                        time.sleep(self.debug_period)
                        continue # Try again next loop

                    time.sleep(1) # Wait a moment to stabilize
                                    
                    # Save marker pose (real pose from detection)
                    self.call_save_marker(
                        name_str = f"marker_{self.target_marker_id}",
                        pose_obj = self.last_valid_real_pose.pose
                    )

                    # Save target marker pose (achievable by robot)
                    self.call_save_marker(
                        name_str = f"target_{self.target_marker_id}",
                        pose_obj = self.target_marker_pose.pose
                    )
                    
                    self.found_markers.add(self.target_marker_id)

                    # Return head to home
                    self.call_move_head_srv(0.0, 0.0)
                    
                    # Return to EXPLORING or HOMING based on found markers
                    if len(self.found_markers) >= 2:
                        self.state = State.HOMING
                    else:
                        self.state = State.EXPLORING

                # --- HOMING ---
                case State.HOMING:
                    self.get_logger().debug("--- State: HOMING ---")
                    if not self.home_pose:
                        self.get_logger().error("No home pose defined. Switching to DONE.")
                        self.state = State.DONE
                        continue
                    
                    if not self.nav_in_progress and not self.nav_outcome:
                        self.get_logger().info("Sending home navigation goal...")
                        self.call_send_nav_goal_srv(self.home_pose)
                    elif self.nav_in_progress and not self.nav_outcome:
                        self.get_logger().info("Homing goal in progress...")
                        time.sleep(self.debug_period)
                        continue
                    else:
                        self.get_logger().info("Homing goal reached successfully.")
                        # Reset flags
                        self.nav_outcome = False
                        self.nav_in_progress = False
                        
                        # Switch to DONE state
                        self.state = State.DONE

                # --- DONE ---
                case State.DONE:
                    self.get_logger().debug("--- State: DONE ---")
                    self.get_logger().info("Task 2 completed successfully")
                    
                    return # Exit the state machine

##################################
# Coordinator Node
##################################
def main(args=None):
    rclpy.init(args=args)
    coordinator = CoordinatorTask2()
    
    # Start state machine in a separate thread
    state_machine_thread = Thread(target=coordinator.run_state_machine)
    state_machine_thread.start()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(coordinator)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()

##################################
# Coordinator Node
##################################
if __name__ == '__main__':
    main()