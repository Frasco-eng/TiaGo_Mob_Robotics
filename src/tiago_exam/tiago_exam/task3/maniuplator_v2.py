#!usr/bin/env python3

##################################
# Imports
##################################
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2


# Python Libraries
from enum import Enum
from threading import Thread

##################################
# State Class
##################################
class ManipulatorState(Enum):
    IDLE = 0          # Manipulator is idle
    MOVE_TO_PRE_GRASP = 1  # Moving arm to pre-grasp position
    MOVE_TO_GRASP = 2      # Moving arm to grasp position
    GRASP = 3              # Performing grasp
    MOVE_TO_SAFE = 4       # Moving arm to safe position after grasp
    MOVE_TO_PRE_PLACE = 5  # Moving arm to pre-place position
    MOVE_TO_PLACE = 6      # Moving arm to place position
    RELEASE = 7            # Releasing the object
    MOVE_TO_HOME = 8       # Moving arm to home position

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
# Manipulator Node
##################################
class Manipulator(Node):
    def __init__(self):
        super().__init__('manipulator')

        self.robot_base_frame = "base_link"

        callback_group_arm = ReentrantCallbackGroup()
        callback_group_gripper = ReentrantCallbackGroup()

        self.arm = MoveIt2(
            node=self,
            joint_names=JOINT_ARM_NAMES,
            base_link_name=self.robot_base_frame,
            end_effector_name="gripper_grasping_frame",
            group_name="arm_torso",
            callback_group=callback_group_arm,
        )
        self.arm.planner_id = "RRTConnectkConfigDefault"

        # [ PUOI USARE QUESTO PER MUOVERTI ]
        # executor = rclpy.executors.MultiThreadedExecutor(2)
        # executor.add_node(self)
        # executor_thread = Thread(target=executor.spin, daemon=True, args=())
        # executor_thread.start()
        # self.create_rate(1.0).sleep()
        # self.get_logger().info(f"Ready!")
        # self.get_logger().info(f"Moving to {{position: {list(self.position)}, orientation: {list(self.orientation)}}}")
        # self.moveit2.move_to_pose(position=self.position, quat_xyzw=self.orientation)
        # self.moveit2.wait_until_executed()
        # self.get_logger().info("Position reached!")