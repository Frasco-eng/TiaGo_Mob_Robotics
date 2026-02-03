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
from linkattacher_msgs.srv import AttachLink, DetachLink
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Custom Service
from tiago_exam_interfaces.srv import ManipulatorCommand


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
    ATTACH = 4                  # Attaching object to gripper
    POST_GRASP = 5              # Arm to post-grasp position
    MOVE_TO_SAFE = 6            # Moving arm to safe position after grasp
    WAITING_FOR_PLACE = 7       # Waiting for PLACE command
    MOVE_TO_PRE_PLACE = 8       # Moving arm to pre-place position
    MOVE_TO_PLACE = 9           # Moving arm to place position
    POST_PLACE = 10             # Arm to post-place position
    RELEASE = 11                # Releasing the object
    DETACH = 12                 # Detaching object from gripper
    MOVE_TO_HOME = 13           # Moving arm to home position

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
CLOSED_GRIPPER_JOINT_POSITIONS = [0.034, 0.034]
FULLY_CLOSED_GRIPPER_JOINT_POSITIONS = [0.0, 0.0]
GRIPPER_GROUP_NAME = "gripper"
GRIPPER_COMMAND_ACTION_NAME = "gripper_controller/joint_trajectory"

##################################
# Manipulator Node
##################################
class Manipulator(Node):
    '''
    Manipulator Node for Tiago Robot
    '''
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
        self.arm.add_collision_box(id="10",
                                        position=[-3.0, -4.0, 0.15],
                                        quat_xyzw=[0.0, 0.0, -0.7071, 0.7071],
                                        size=[0.5, 0.5, 0.3])
        
        self.arm.add_collision_box(id="11",
                                        position=[0.0, -7, 0.15],
                                        quat_xyzw=[0.0, 0.0, -0.7071434, 0.7070701],
                                        size=[0.5, 0.5, 0.3])    

        # Gripper Interface
        self.gripper = GripperInterface(
        node=self,
        gripper_joint_names=JOINT_GRIPPER_NAMES,
        open_gripper_joint_positions=OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=CLOSED_GRIPPER_JOINT_POSITIONS,
        gripper_group_name=GRIPPER_GROUP_NAME,
        callback_group=self.callback_group_gripper,
        gripper_command_action_name=GRIPPER_COMMAND_ACTION_NAME,
        max_effort=100.0
        )

        # Add collision objects
        self.gripper.add_collision_box(id='12',
                                            position=[-3.0, -4.0, 0.15],
                                            quat_xyzw=[0.0, 0.0, -0.7071, 0.7071],
                                            size=[0.5, 0.5, 0.3])
        
        self.gripper.add_collision_box(id='13',
                                            position=[0.0, -7, 0.15],
                                            quat_xyzw=[0.0, 0.0, -0.7071434, 0.7070701],
                                            size=[0.5, 0.5, 0.3]) 

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
        self.task_finished_event = Event()  # To sync with service
        self.task_success = False           # To indicate task success
        self.base_T_marker = kdl.Frame()    # Transformation from base to marker frame
        self.marker_id = None               # ID of the marker to manipulate
        self.attach_future = None           # Future for attach service
        self.detach_future = None           # Future for detach service
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
        # Services Clients
        ##################################
        self.attach_client = self.create_client(AttachLink, 'ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, 'DETACHLINK')
        
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
        #self.get_logger().info(f"Current Force: x{force.x:.2f} N, y{force.y:.2f} N, z{force.z:.2f} N | Magnitude (y,z): {self.current_force:.2f} N")

    def handle_manipulator_command(self, request, response):
        self.get_logger().info(f"Received Manipulator Command: {request.type}")
        self.base_T_marker =  self.transform_stamped_to_kdl(request.frame)
       
        self.task_finished_event.clear()
        self.task_success = False

        # Extract operation type
        if 'PICK' in request.type:
            operation_type = 'PICK'
            self.marker_id = request.type.split('_')[-1]
        elif 'PLACE' in request.type:
            operation_type = 'PLACE'
            self.marker_id = request.type.split('_')[-1]
        else:
            operation_type = 'UNKNOWN'
            self.marker_id = None

        match operation_type:
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
        for i in range(10):
            self.arm.move_to_configuration(joint_positions)
            success = self.arm.wait_until_executed()
            if success:
                break
            else:
                self.get_logger().warning(f"Attempt {i+1}/10: Failed to reach joint positions, retrying...")
                
        if i == 9 and not success:
            self.get_logger().error("Exceeded maximum attempts to reach joint positions.")     
        return success
    
    ##################################
    # Helper Functions
    ##################################    
    def call_move_arm_srv(self, pose: PoseStamped, cartesian:bool=False) -> bool:
        pose = pose.pose
        position = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        
        i = 0
        for i in range (10):
            self.arm.move_to_pose(position=position, quat_xyzw=quat, cartesian=cartesian)
            success = self.arm.wait_until_executed()
            if success:
                break
            else:
                self.get_logger().warning(f"Attempt {i+1}/10: Failed to reach pose, retrying...")
        if i == 9 and not success:
            self.get_logger().error("Exceeded maximum attempts to reach pose.")     
        return success
     
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
                    
                    approach_twist = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(-0.05, 0.0, 0.0))
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
                    self.gripper.closed_gripper_joint_positions = FULLY_CLOSED_GRIPPER_JOINT_POSITIONS
                    # Close gripper
                    self.gripper.close()
                    self.gripper.wait_until_executed()
                    self.gripper.closed_gripper_joint_positions = CLOSED_GRIPPER_JOINT_POSITIONS
                    time.sleep(1.0)  # Wait for gripper to close
                    

                    self.current_state = State.ATTACH
                
                case State.ATTACH:
                    self.get_logger().info("--- State: ATTACH ---")
                    
                    if self.attach_future is None:
                        
                        # wait for the attach service to be available
                        while not self.attach_client.wait_for_service(timeout_sec=1.0):
                            self.get_logger().info('attach_link service not available, waiting...')
                    
                        # Attach the object to the gripper
                        attach_request = AttachLink.Request()
                        
                        attach_request.model1_name = "tiago"
                        attach_request.link1_name = "gripper_left_finger_link"
                        attach_request.model2_name = "aruco_cube_exam_id" + str(self.marker_id)
                        attach_request.link2_name = "link"
                        
                        self.attach_future = self.attach_client.call_async(attach_request)      
                    
                    if self.attach_future is not None and self.attach_future.done():
                        attach_response = self.attach_future.result()
                        if attach_response.success:
                            self.get_logger().info("Object successfully attached to gripper.")
                            self.current_state = State.POST_GRASP
                            self.attach_future = None  # Reset future for next use
                        else:
                            self.get_logger().error("Failed to attach object to gripper.")
                            self.task_success = False
                            self.current_state = State.IDLE
                            self.attach_future = None  # Reset future for next use
                            self.task_finished_event.set()                
                    
                case State.POST_GRASP:
                    self.get_logger().info("--- State: POST_GRASP ---")
                    approach_twist = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(-0.4, 0.0, 0.0))
                    base_T_approach = self.base_T_marker * approach_twist

                    post_grasp_pose = self.kdl_to_pose_msg(base_T_approach, self.robot_base_frame)

                    if self.call_move_arm_srv(post_grasp_pose, cartesian=True):
                        self.get_logger().info("Reached Post-Grasp Pose")
                        self.current_state = State.MOVE_TO_SAFE
                    else:
                        self.get_logger().error("Failed to reach Post-Grasp Pose")
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

                    # Compute pre-place pose: from base_T_marker which is on the side of the table to the top of the table
                    self.base_T_marker.M = self.base_T_marker.M * kdl.Rotation.RotY(np.pi/2)
                    pre_place_twist = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(-0.4, 0.0, 0.0))
                    base_T_pre_place = self.base_T_marker * pre_place_twist
                    pre_place_pose = self.kdl_to_pose_msg(base_T_pre_place, self.robot_base_frame)
                    
                    
                    if self.call_move_arm_srv(pre_place_pose):
                        self.current_state = State.MOVE_TO_PLACE
                    else:
                        self.get_logger().error("Failed to reach Pre-Place")
                        self.task_success = False
                        self.current_state = State.IDLE
                        self.task_finished_event.set()   
                

                case State.MOVE_TO_PLACE:
                    self.get_logger().info("--- State: MOVE_TO_PLACE ---")

                    place_twist = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(-0.1, 0.0, 0.0))

                    base_T_place = self.base_T_marker * place_twist
                    place_pose = self.kdl_to_pose_msg(base_T_place, self.robot_base_frame)
                    
                    if self.call_move_arm_srv(place_pose, cartesian=True):
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

                    self.current_state = State.DETACH
                    
                case State.DETACH:
                    
                    if self.detach_future is None:
                        self.get_logger().info("Preparing to detach object from gripper...")
                        self.detach_future = self.detach_client.call_async(DetachLink.Request())

                    
                        # Detach the object from the gripper
                        detach_request = DetachLink.Request()
                        detach_request.model1_name = "tiago"
                        detach_request.link1_name = "gripper_left_finger_link"
                        detach_request.model2_name = "aruco_cube_exam_id" + str(self.marker_id)
                        detach_request.link2_name = "link"
                        self.get_logger().info("aruco_cube_exam_id_" + str(self.marker_id))
                        self.detach_future = self.detach_client.call_async(detach_request)
                    
                    self.get_logger().info("--- State: DETACH ---")
                    if self.detach_future is not None and self.detach_future.done():
                        detach_response = self.detach_future.result()
                        if detach_response.success:
                            self.get_logger().info("Object successfully detached from gripper.")
                            # Open gripper
                            self.gripper.close()
                            self.gripper.wait_until_executed()
                            self.current_state = State.POST_PLACE
                            self.detach_future = None  # Reset future for next use
                        else:
                            self.get_logger().error("Failed to detach object from gripper.")
                            self.task_success = False
                            self.current_state = State.IDLE
                            self.task_finished_event.set()
                            self.detach_future = None  # Reset future for next use
                            
                case State.POST_PLACE:
                    self.get_logger().info("--- State: POST_PLACE ---")
                    approach_twist = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(-0.4, 0.0, 0.0))
                    base_T_approach = self.base_T_marker * approach_twist

                    post_place_pose = self.kdl_to_pose_msg(base_T_approach, self.robot_base_frame)

                    if self.call_move_arm_srv(post_place_pose, cartesian=True):
                        self.get_logger().info("Reached Post-Place Pose")
                        self.current_state = State.MOVE_TO_SAFE
                    else:
                        self.get_logger().error("Failed to reach Post-Place Pose")
                        self.task_success = False
                        self.current_state = State.IDLE
                        self.task_finished_event.set()
                        continue

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

