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

# Custom Service
from exam_interfaces.srv import ManipulatorCommand

# Python Libraries
from enum import Enum
from threading import Thread, Event
import time
import copy

##################################
# State Class
##################################
class State(Enum):
    IDLE = 0                # Manipulator is idle
    MOVE_TO_PRE_GRASP = 1   # Moving arm to pre-grasp position
    MOVE_TO_GRASP = 2       # Moving arm to grasp position
    GRASP = 3               # Performing grasp
    MOVE_TO_SAFE = 4        # Moving arm to safe position after grasp
    MOVE_TO_PRE_PLACE = 5   # Moving arm to pre-place position
    MOVE_TO_PLACE = 6       # Moving arm to place position
    RELEASE = 7             # Releasing the object
    MOVE_TO_HOME = 8        # Moving arm to home position

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

OPEN_GRIPPER_JOINT_POSITIONS = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS = [0.0, 0.0]
GRIPPER_GROUP_NAME = "gripper"
GRIPPER_COMMAND_ACTION_NAME = "gripper_controller/joint_trajectory"

##################################
# Commands
##################################


##################################
# Manipulator Node
##################################
class Manipulator(Node):
    def __init__(self):
        super().__init__('manipulator')
        self.robot_base_frame = "base_link"

        # Create callback groups
        self.callback_group_arm = ReentrantCallbackGroup()
        self.callback_group_gripper = ReentrantCallbackGroup()
        self.callback_group_service = ReentrantCallbackGroup()

        # Create arm interface
        self.arm = MoveIt2(
            node=self,
            joint_names=JOINT_ARM_NAMES,
            base_link_name=self.robot_base_frame,
            end_effector_name="gripper_grasping_frame",
            group_name="arm_torso",
            callback_group=self.callback_group_arm,
        )
        self.arm.planner_id = "RRTConnectkConfigDefault"

        # Create gripper interface
        self.gripper = GripperInterface(
        node=self,
        gripper_joint_names=JOINT_GRIPPER_NAMES,
        open_gripper_joint_positions=OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=CLOSED_GRIPPER_JOINT_POSITIONS,
        gripper_group_name=GRIPPER_GROUP_NAME,
        callback_group=self.callback_group_gripper,
        gripper_command_action_name=GRIPPER_COMMAND_ACTION_NAME,
        )

        # Scale down velocity and acceleration of joints (percentage of maximum)
        self.arm.max_velocity = 1.0
        self.arm.max_acceleration = 1.0
        self.gripper.max_velocity = 1.0
        self.gripper.max_acceleration = 1.0

        # State Machine variables
        self.current_state = State.IDLE
        self.target_pose = None
        self.current_command_type = None    # "PICK" or "PLACE"
        self.task_finished_event = Event()  # To sync with service
        self.task_success = False

        # Service Server
        self.srv = self.create_service(
            ManipulatorCommand, 
            'manipulator_command_srv', 
            self.handle_manipulator_command, 
            callback_group=self.callback_group_service
        )

        # Start executor in separate thread
        executor = rclpy.executors.MultiThreadedExecutor(4) # Increased threads for safety
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        # Start State Machine Loop in separate thread
        self.state_machine_thread = Thread(target=self.run_state_machine, daemon=True)
        self.state_machine_thread.start()

        self.get_logger().info(f"Manipulator Node Ready.")


    def handle_manipulator_command(self, request, response):
        self.get_logger().info(f"Received Manipulator Command: {request.type}")
        self.current_command_type = request.type  # "PICK" or "PLACE"
        self.target_pose = request.pose
        self.task_finished_event.clear()
        self.task_success = False

        match request.type:
            case "PICK":
                self.get_logger().info("Starting PICK operation.")
                self.current_state = State.MOVE_TO_PRE_GRASP
            case "PLACE":
                self.get_logger().info("Starting PLACE operation.")
                self.current_state = State.MOVE_TO_PRE_PLACE
            case _:
                self.get_logger().error(f"Unknown command type: {request.type}")
                response.success = False
                return response

        # Wait for task to finish
        self.task_finished_event.wait()

        response.success = self.task_success
        self.get_logger().info(f"Command {request.type} completed with success: {self.task_success}")

    def move_arm_to(self, pose):
        position = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        self.arm.move_to_pose(position=position, quat_xyzw=quat)
        success = self.arm.wait_until_executed()
        return success
    
    def run_state_machine(self):
        while rclpy.ok():
            match self.current_state:
                case State.IDLE:
                    self.get_logger().debug("--- State: IDLE ---")
                    time.sleep(0.1)
                
                case State.MOVE_TO_PRE_GRASP:
                    self.get_logger().info("--- State: MOVE_TO_PRE_GRASP ---")

                    # Open gripper
                    self.gripper.open()
                    self.gripper.wait_until_executed()

                    # Compute pre-grasp pose
                    pre_grasp_pose = copy.deepcopy(self.target_pose)
                    pre_grasp_pose.position.z += 0.20

                    if self.move_arm_to(pre_grasp_pose):
                        self.current_state = State.MOVE_TO_GRASP
                    else:
                        self.get_logger().error("Failed to reach Pre-Grasp")
                        self.task_success = False
                        self.current_state = State.IDLE
                        self.task_finished_event.set()

                case State.MOVE_TO_GRASP:
                    self.get_logger().info("--- State: MOVE_TO_GRASP ---")

                    grasp_pose = copy.deepcopy(self.target_pose)

                    if self.move_arm_to(grasp_pose):
                        self.current_state = State.GRASP
                    else:
                        self.get_logger().error("Failed to reach Grasp Pose")
                        self.task_success = False
                        self.current_state = State.IDLE
                        self.task_finished_event.set()
                
                case State.GRASP:
                    self.get_logger().info("--- State: GRASP ---")

                    # Close gripper
                    self.gripper.close()
                    self.gripper.wait_until_executed()

                    self.current_state = State.MOVE_TO_SAFE

                case State.MOVE_TO_SAFE:
                    self.get_logger().info("--- State: MOVE_TO_SAFE ---")

                    safe_pose = Pose()

                    safe_pose.position = [0.20, -1.34, -0.20]
                    safe_pose.orientation = [1.94, -1.57, 1.37, 0.00]

                    if self.move_arm_to(safe_pose):
                        self.task_success = True
                        self.current_state = State.IDLE
                        self.task_finished_event.set()
                    else:
                        self.get_logger().error("Failed to Safe")
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


        


# [ PUOI USARE QUESTO PER MUOVERTI - ARM]
# self.moveit2.move_to_pose(position=self.position, quat_xyzw=self.orientation)
# self.moveit2.wait_until_executed()
# self.get_logger().info("Reached target pose!")

# [ PUOI USARE QUESTO PER MUOVERTI - GRIPPER]
# self.get_logger().info("Opening gripper")
# self.gripper.open()
# self.gripper.wait_until_executed()
# self.get_logger().info("Gripper opened!")




def main(args=None):
    rclpy.init(args=args)
    manipulator = Manipulator()
    # Lo spin è gestito nel thread executor creato nell'init
    try:
        # Teniamo il main thread vivo
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        manipulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()