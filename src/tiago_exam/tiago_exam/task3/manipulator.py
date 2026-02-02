#!usr/bin/env python3

##################################
# Imports
##################################
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2
from pymoveit2 import GripperInterface
from geometry_msgs.msg import PoseStamped, WrenchStamped

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
# Custom Service
from exam_interfaces.srv import ManipulatorCommand


# Python Libraries
from enum import Enum
from threading import Thread, Event
import time
import copy
import numpy as np
import PyKDL as kdl

##################################
# State Class
##################################
class State(Enum):
    IDLE = 0                    # Manipulator is idle
    MOVE_TO_PRE_GRASP = 1       # Moving arm to pre-grasp position
    MOVE_TO_GRASP = 2           # Moving arm to grasp position
    GRASP = 3                   # Performing grasp
    POST_GRASP = 4              # Arm to post-grasp position
    MOVE_TO_SAFE = 5            # Moving arm to safe position after grasp
    WAITING_FOR_PLACE = 6       # Waiting for PLACE command
    MOVE_TO_PRE_PLACE = 7       # Moving arm to pre-place position
    MOVE_TO_PLACE = 8           # Moving arm to place position
    POST_PLACE = 9              # Arm to post-place position
    RELEASE = 10                # Releasing the object
    MOVE_TO_HOME = 11           # Moving arm to home position

##################################
# Joint Names
##################################
JOINT_ARM_NAMES = [
    "torso_lift_joint",
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
    "arm_tool_joint",
]

JOINT_GRIPPER_NAMES = [
"gripper_left_finger_joint",
"gripper_right_finger_joint",
]

##################################
# Gripper Constants
##################################
OPEN_GRIPPER_JOINT_POSITIONS = [0.044, 0.044]
CLOSED_GRIPPER_JOINT_POSITIONS = [0.025, 0.025]
GRIPPER_GROUP_NAME = "gripper"
GRIPPER_COMMAND_ACTION_NAME = "gripper_controller/joint_trajectory"

##################################
# Manipulator Node
##################################
class Manipulator(Node):
    def __init__(self):
        super().__init__('manipulator')

        ##################################
        # Callback Groups
        ##################################
        self.callback_group_arm = ReentrantCallbackGroup()
        self.callback_group_gripper = ReentrantCallbackGroup()
        self.callback_group_service = ReentrantCallbackGroup()

        ##################################
        # Manipulator Interfaces
        ##################################
        self.robot_base_frame = "base_link"
        self.target_frame_name = "aruco_marker_target_frame"
        
        # Arm Interface
        self.arm = MoveIt2(
            node=self,
            joint_names=JOINT_ARM_NAMES,
            base_link_name=self.robot_base_frame,
            end_effector_name="gripper_grasping_frame",
            group_name="arm_torso",
            callback_group=self.callback_group_arm,
        )

        # Bidirectional RRT
        self.arm.planner_id = "RRTConnectkConfigDefault"

        # Add collision objects
        # self.arm.add_collision_box(id='env_exam_pick_surface',
        #                                 position=[-3.0, -4.0, 0.15],
        #                                 quat_xyzw=[0.0, 0.0, -0.7071, 0.7071],
        #                                 size=[0.5, 0.5, 0.3])
        
        # self.arm.add_collision_box(id='env_exam_place_surface',
        #                                 position=[0.0, -7, 0.15],
        #                                 quat_xyzw=[0.0, 0.0, -0.7071434, 0.7070701],
        #                                 size=[0.5, 0.5, 0.3])    

        # Gripper Interface
        self.gripper = GripperInterface(
        node=self,
        gripper_joint_names=JOINT_GRIPPER_NAMES,
        open_gripper_joint_positions=OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=CLOSED_GRIPPER_JOINT_POSITIONS,
        gripper_group_name=GRIPPER_GROUP_NAME,
        callback_group=self.callback_group_gripper,
        gripper_command_action_name=GRIPPER_COMMAND_ACTION_NAME,
        )

        ##################################
        # Velocity and Acceleration Limits
        ##################################
        self.arm.max_velocity = 1.0
        self.arm.max_acceleration = 0.5
        self.gripper.max_velocity = 1.0
        self.gripper.max_acceleration = 1.0
        self.safe_joints_position = [0.0, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.00, 0.04]

        ##################################
        # FT Sensor Variables
        ##################################
        self.current_force = 0.0
        self.force_threshold = 1.0  # Newtons
   
        ##################################
        # State Variables
        ##################################
        self.current_state = State.IDLE     # State Machine Current State
        self.target_pose = None             # Target Pose for current command
        self.current_command_type = None    # "PICK" or "PLACE"
        self.task_finished_event = Event()  # To sync with service
        self.task_success = False           # To indicate task success
        self.base_T_marker = kdl.Frame()    # Transformation from base to marker frame
        
        ##################################
        # Service Server
        ##################################
        self.srv = self.create_service(
            ManipulatorCommand, 
            'manipulator_command_srv', 
            self.handle_manipulator_command, 
            callback_group=self.callback_group_service
        )
        
        ##################################
        # Subscibers
        ##################################
        qos_profile = rclpy.qos.QoSProfile(depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        
        self.ft_sub = self.create_subscription(WrenchStamped, '/ft_sensor_controller/wrench', self.ft_callback, qos_profile, callback_group=self.callback_group_service)

        self.get_logger().info(f"Manipulator Node Ready.")

    ##################################
    # Callbacks
    ##################################
    def ft_callback(self, msg: WrenchStamped):
        force = msg.wrench.force
        self.current_force = np.sqrt(force.x**2 + force.y**2)
        self.get_logger().info(f"Current Force: x{force.x:.2f} N, y{force.y:.2f} N, z{force.z:.2f} N | Magnitude (y,z): {self.current_force:.2f} N")

    def handle_manipulator_command(self, request, response):
        self.get_logger().info(f"Received Manipulator Command: {request.type}")
        self.current_command_type = request.type  # "PICK" or "PLACE"
        self.base_T_marker =  self.transform_stamped_to_kdl(request.frame)
       
        self.task_finished_event.clear()
        self.task_success = False

        match request.type:
            case "PICK":
                self.get_logger().info("Starting PICK operation.")
                self.current_state = State.MOVE_TO_PRE_GRASP
                self.task_success = True
            case "PLACE":
                self.get_logger().info("Starting PLACE operation.")
                self.current_state = State.MOVE_TO_PRE_PLACE
            case _:
                self.get_logger().error(f"Unknown command type: {request.type}")
                response.success = False
                return response

    
        event_set = self.task_finished_event.wait(timeout=60.0) 

        if not event_set:
            self.get_logger().error("Operation timed out!")
            self.task_success = False
            self.current_state = State.IDLE # Force reset

        response.success = self.task_success
        self.get_logger().info(f"Command {request.type} completed with success: {self.task_success}")
        return response
    
    def move_to_joints(self, joint_positions: list) -> bool:
        self.arm.move_to_configuration(joint_positions)
        return self.arm.wait_until_executed()
    
    ##################################
    # Helper Functions
    ##################################    
    def call_move_arm_srv(self, pose: PoseStamped, cartesian:bool=False) -> bool:
        pose = pose.pose
        position = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        self.arm.move_to_pose(position=position, quat_xyzw=quat, cartesian=cartesian)
        success = self.arm.wait_until_executed()
        return success
    
    def grasp_with_force(self) -> bool:
        """
        Closes the gripper incrementally while monitoring the force sensor.
        Returns True if an object was grasped (force threshold met), 
        False if the gripper closed completely without feeling force.
        """
        self.get_logger().info("Starting Force Grasp...")
        
        # Define range and step
        # Assuming open is 0.044 and closed is 0.032 (from your constants)
        start_pos = OPEN_GRIPPER_JOINT_POSITIONS[0] 
        end_pos = CLOSED_GRIPPER_JOINT_POSITIONS[0]
        
        # Define the step size (e.g., 1mm or 0.5mm per step)
        step_size = 0.001 
        
        current_target = start_pos

        # Determine direction (Closing means going from 0.044 -> 0.032)
        # We assume both fingers move symmetrically, so we drive based on the first joint value
        
        grasped_object = False

        while current_target > end_pos:
            # 1. Check Force
            #self.get_logger().info(f"force at grasp: {self.current_force:.2f} N")
            if self.current_force >= self.force_threshold:
                self.get_logger().info(f"Grasp detected! Force: {self.current_force:.2f} N")
                grasped_object = True
                break

            # 2. Decrement position (close slightly)
            current_target -= step_size
            
            # Clamp to ensure we don't go past closed limit
            if current_target < end_pos:
                current_target = end_pos

            # 3. Send command
            # move_to_position sends the command to both fingers
            self.gripper.move_to_position(current_target)
            
            # 4. Wait briefly for motion to occur and sensor to update
            # Adjust this sleep based on robot response time
            time.sleep(0.3) 

        if grasped_object:
            # Optional: Command the gripper to stay exactly here (or squeeze slightly more)
            # to ensure the object doesn't slip.
            self.gripper.move_to_position(current_target)
            return True
        else:
            self.get_logger().warn("Gripper fully closed but no object detected.")
            return False
    
    ##################################
    # STATE MACHINE
    ##################################
    def run_state_machine(self):
        while rclpy.ok():
            match self.current_state:
                case State.IDLE:
                    self.get_logger().debug("--- State: IDLE ---")
                
                case State.MOVE_TO_PRE_GRASP:
                    self.get_logger().info("--- State: MOVE_TO_PRE_GRASP ---")

                    
                    # Compute pre-grasp pose                    
                    self.base_T_marker.M = self.base_T_marker.M * kdl.Rotation.RotY(np.pi/2)
                    approach_twist = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(-0.4, 0.0, 0.0))

                    base_T_approach = self.base_T_marker * approach_twist
                    
                    pre_grasp_pose = self.kdl_to_pose_msg(base_T_approach, self.robot_base_frame)
                         
                    if self.call_move_arm_srv(pre_grasp_pose):
                        # Open gripper
                        self.gripper.open()
                        self.gripper.wait_until_executed()
                                      
                        self.current_state = State.MOVE_TO_GRASP
                        

                    else:
                        self.get_logger().error("Failed to reach Pre-Grasp")
                        self.task_success = False
                        self.current_state = State.IDLE
                        self.task_finished_event.set()

                case State.MOVE_TO_GRASP:
                    self.get_logger().info("--- State: MOVE_TO_GRASP ---")
                    
                    approach_twist = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(-0.03, 0.0, 0.0))
                    base_T_approach = self.base_T_marker * approach_twist

                    grasp_pose = self.kdl_to_pose_msg(base_T_approach, self.robot_base_frame)

                    
                    if self.call_move_arm_srv(grasp_pose, cartesian=True):
                        self.current_state = State.GRASP
                    else:
                        self.get_logger().error("Failed to reach Grasp Pose")
                        self.task_success = False
                        self.current_state = State.IDLE
                        self.task_finished_event.set()
                
                case State.GRASP:
                    self.get_logger().info("--- State: GRASP ---")

                    # # Close gripper
                    success = self.grasp_with_force()
                    #self.gripper.wait_until_executed()
                    
                    if success:            
                        self.current_state = State.POST_GRASP
                    else:
                        self.get_logger().error("Grasp failed (no object detected).")
                        # Decide logic here: Retry? Go Home? Or proceed anyway?
                        self.task_success = False
                        self.current_state = State.IDLE
                        self.task_finished_event.set()
                    
                case State.POST_GRASP:
                    approach_twist = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(-0.2, 0.0, 0.0))
                    base_T_approach = self.base_T_marker * approach_twist

                    grasp_pose = self.kdl_to_pose_msg(base_T_approach, self.robot_base_frame)

                    if self.call_move_arm_srv(grasp_pose, cartesian=True):
                        self.get_logger().info("Reached Grasp Approach Pose")
                        self.current_state = State.MOVE_TO_SAFE
                    else:
                        self.get_logger().error("Failed to reach Grasp Approach Pose")
                        self.task_success = False
                        self.current_state = State.IDLE
                        self.task_finished_event.set()
                        continue

                case State.MOVE_TO_SAFE:
                    self.get_logger().info("--- State: MOVE_TO_SAFE ---")

                    if self.move_to_joints(self.safe_joints_position):
                        self.get_logger().info("Reached Safe joints")
                        self.task_success = True
                        self.current_state = State.IDLE
                        self.task_finished_event.set()
                    else:
                        self.get_logger().error("Failed to reach Safe joints")
                        self.task_success = False
                        self.current_state = State.IDLE
                        self.task_finished_event.set()

               
                case State.MOVE_TO_PRE_PLACE:
                    
                    self.get_logger().info("--- State: MOVE_TO_PRE_PLACE ---")

                    # Compute pre-place pose
                    pre_place_pose = copy.deepcopy(self.target_pose) # questa dovrà essere la posizione del marker station (x,y uguali, z diversa)
                    pre_place_pose.position.z += 0.20

                    if self.move_arm_to(pre_place_pose):
                        self.current_state = State.MOVE_TO_PLACE
                    else:
                        self.get_logger().error("Failed to reach Pre-Place")
                        self.task_success = False
                        self.current_state = State.IDLE
                        self.task_finished_event.set()

                case State.MOVE_TO_PLACE:
                    self.get_logger().info("--- State: MOVE_TO_PLACE ---")

                    place_pose = copy.deepcopy(self.target_pose)
                    if self.move_arm_to(place_pose):
                        self.current_state = State.RELEASE
                    else:
                        self.get_logger().error("Failed to reach Place Pose")
                        self.task_success = False
                        self.current_state = State.IDLE
                        self.task_finished_event.set()

                case State.RELEASE:
                    self.get_logger().info("--- State: RELEASE ---")

                    # Open gripper
                    self.gripper.open()
                    self.gripper.wait_until_executed()

                    self.task_success = True

                    self.current_state = State.MOVE_TO_SAFE
                    
        

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
    
    def transform_stamped_to_kdl(self, ts: TransformStamped) -> kdl.Frame:
        # Translation
        t = ts.transform.translation
        pos = kdl.Vector(t.x, t.y, t.z)

        # Rotation
        q = ts.transform.rotation
        rot = kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w)

        return kdl.Frame(rot, pos)
    
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
        


# [ PUOI USARE QUESTO PER MUOVERTI - ARM]
# self.moveit2.move_to_pose(position=self.position, quat_xyzw=self.orientation)
# self.moveit2.wait_until_executed()
# self.get_logger().info("Reached target pose!")

# [ PUOI USARE QUESTO PER MUOVERTI - GRIPPER]
# self.get_logger().info("Opening gripper")
# self.gripper.open()
# self.gripper.wait_until_executed()
# self.get_logger().info("Gripper opened!")



##################################
# Main Function
##################################
def main(args=None):
    rclpy.init(args=args)
    manipulator = Manipulator()

    # Run the state machine loop in a separate thread
    state_thread = Thread(target=manipulator.run_state_machine, daemon=True)
    state_thread.start()

    # ROS 2 executor in main thread (multi-threaded for callbacks)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(manipulator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        manipulator.destroy_node()
        rclpy.shutdown()


##################################
# Entry Point
##################################
if __name__ == '__main__':
    main()