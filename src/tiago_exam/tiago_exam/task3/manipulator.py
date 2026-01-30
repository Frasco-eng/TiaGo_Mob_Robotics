#!usr/bin/env python3

##################################
# Imports
##################################
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import tf2_ros
import tf2_geometry_msgs
import time
import math

# Python Libraries
from enum import Enum
from typing import Optional

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


##################################
# Manipulator Node
##################################
class Manipulator(Node):
    def __init__(self):
        super().__init__('manipulator')
        
        # State management
        self.current_state = ManipulatorState.IDLE
        self.previous_state = ManipulatorState.IDLE
        self.state_start_time = time.time()
        self.state_timeout = 10.0  # 10 seconds per state
        
        # Target poses
        self.target_pick_pose: Optional[PoseStamped] = None
        self.target_place_pose: Optional[PoseStamped] = None
        
        # Gripper state
        self.gripper_closed = False
        
        # TF2 for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.operation_done_pub = self.create_publisher(Bool, '/manipulator/operation_done', 10)
        self.arm_trajectory_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_command_pub = self.create_publisher(String, '/gripper/command', 10)
        
        # Subscribers
        self.create_subscription(String, '/manipulator/command', self.on_command_received, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state_received, 10)
        
        # Command info
        self.current_command = None
        self.current_marker_id = None
        
        # Joint state tracking
        self.current_joint_positions = {}
        self.arm_ready = False
        
        # Predefined positions
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # All zeros for home
        self.pre_grasp_height_offset = 0.15  # 15cm above target
        self.safe_height = 0.5  # Safe height for transporting
        
        # Timer for state machine
        self.create_timer(0.1, self.state_machine_callback)
        
        self.get_logger().info(f'{self.get_name()} node has been initialized.')
    
    def on_command_received(self, msg: String) -> None:
        """Callback when coordinator sends a manipulation command
        
        Command format:
        - PICK:x:y:z
        - PLACE:x:y:z
        """
        try:
            parts = msg.data.split(':')
            command = parts[0]
            
            if command == 'PICK' and len(parts) == 4:
                self.target_pick_pose = PoseStamped()
                self.target_pick_pose.header.frame_id = 'base_footprint'
                self.target_pick_pose.pose.position.x = float(parts[1])
                self.target_pick_pose.pose.position.y = float(parts[2])
                self.target_pick_pose.pose.position.z = float(parts[3])
                self.target_pick_pose.pose.orientation.w = 1.0
                
                self.current_command = 'PICK'
                self.current_state = ManipulatorState.MOVE_TO_PRE_GRASP
                self.state_start_time = time.time()
                
                self.get_logger().info(
                    f'Pick command received: ({self.target_pick_pose.pose.position.x:.2f}, '
                    f'{self.target_pick_pose.pose.position.y:.2f}, '
                    f'{self.target_pick_pose.pose.position.z:.2f})')
                
            elif command == 'PLACE' and len(parts) == 4:
                self.target_place_pose = PoseStamped()
                self.target_place_pose.header.frame_id = 'base_footprint'
                self.target_place_pose.pose.position.x = float(parts[1])
                self.target_place_pose.pose.position.y = float(parts[2])
                self.target_place_pose.pose.position.z = float(parts[3])
                self.target_place_pose.pose.orientation.w = 1.0
                
                self.current_command = 'PLACE'
                self.current_state = ManipulatorState.MOVE_TO_PRE_PLACE
                self.state_start_time = time.time()
                
                self.get_logger().info(
                    f'Place command received: ({self.target_place_pose.pose.position.x:.2f}, '
                    f'{self.target_place_pose.pose.position.y:.2f}, '
                    f'{self.target_place_pose.pose.position.z:.2f})')
            else:
                self.get_logger().warn(f'Invalid command format: {msg.data}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to parse command: {e}')
    
    def on_joint_state_received(self, msg: JointState) -> None:
        """Callback for joint state updates"""
        for i, name in enumerate(msg.name):
            self.current_joint_positions[name] = msg.position[i]
        
        self.arm_ready = len(self.current_joint_positions) > 0
    
    def close_gripper(self) -> None:
        """Close the gripper"""
        cmd = String()
        cmd.data = 'CLOSE'
        self.gripper_command_pub.publish(cmd)
        self.gripper_closed = True
        self.get_logger().info('Gripper close command sent')
    
    def open_gripper(self) -> None:
        """Open the gripper"""
        cmd = String()
        cmd.data = 'OPEN'
        self.gripper_command_pub.publish(cmd)
        self.gripper_closed = False
        self.get_logger().info('Gripper open command sent')
    
    def send_arm_trajectory(self, target_pose: PoseStamped, duration: float = 3.0) -> None:
        """Send arm trajectory to reach target pose (IK-based)"""
        try:
            # In a real scenario, you would use MoveIt2 or similar to compute IK
            # For now, we're sending a simplified trajectory
            trajectory = JointTrajectory()
            trajectory.header.frame_id = 'base_link'
            trajectory.joint_names = [
                'arm_1_joint',
                'arm_2_joint', 
                'arm_3_joint',
                'arm_4_joint',
                'arm_5_joint',
                'arm_6_joint',
                'arm_7_joint'
            ]
            
            point = JointTrajectoryPoint()
            point.positions = [0.0, 0.8, -1.57, 1.57, -1.57, -1.57, 0.0]
            point.time_from_start = Duration(sec=int(duration), nanosec=0)
            
            trajectory.points = [point]
            self.arm_trajectory_pub.publish(trajectory)
            
            self.get_logger().debug(f'Arm trajectory sent for {duration}s')
            
        except Exception as e:
            self.get_logger().error(f'Failed to send arm trajectory: {e}')
    
    def move_to_target(self, target_pose: PoseStamped, duration: float = 3.0) -> None:
        """Move arm to target pose"""
        # This would normally use MoveIt2, for now we simulate
        self.send_arm_trajectory(target_pose, duration)
    
    def state_machine_callback(self) -> None:
        """Main manipulator state machine"""
        
        # Check for state timeout
        if time.time() - self.state_start_time > self.state_timeout:
            self.get_logger().warn(f'State {self.current_state.name} timeout')
            if self.current_state != ManipulatorState.IDLE:
                self.current_state = ManipulatorState.MOVE_TO_HOME
        
        if self.current_state == ManipulatorState.IDLE:
            # Wait for commands
            pass
        
        elif self.current_state == ManipulatorState.MOVE_TO_PRE_GRASP:
            if self.target_pick_pose and self.arm_ready:
                # Move to pre-grasp position (above target)
                pre_grasp = PoseStamped()
                pre_grasp.header.frame_id = self.target_pick_pose.header.frame_id
                pre_grasp.pose.position.x = self.target_pick_pose.pose.position.x
                pre_grasp.pose.position.y = self.target_pick_pose.pose.position.y
                pre_grasp.pose.position.z = self.target_pick_pose.pose.position.z + self.pre_grasp_height_offset
                pre_grasp.pose.orientation.w = 1.0
                
                self.move_to_target(pre_grasp, duration=3.0)
                self.current_state = ManipulatorState.MOVE_TO_GRASP
                self.state_start_time = time.time()
                self.get_logger().info('Moving to pre-grasp position')
        
        elif self.current_state == ManipulatorState.MOVE_TO_GRASP:
            if self.target_pick_pose and self.arm_ready:
                # Move to actual grasp position
                self.move_to_target(self.target_pick_pose, duration=2.0)
                self.current_state = ManipulatorState.GRASP
                self.state_start_time = time.time()
                self.get_logger().info('Moving to grasp position')
        
        elif self.current_state == ManipulatorState.GRASP:
            # Close gripper
            self.close_gripper()
            self.current_state = ManipulatorState.MOVE_TO_SAFE
            self.state_start_time = time.time()
            self.get_logger().info('Grasping object')
        
        elif self.current_state == ManipulatorState.MOVE_TO_SAFE:
            if self.arm_ready:
                # Move to safe position
                safe_pose = PoseStamped()
                safe_pose.header.frame_id = 'base_footprint'
                safe_pose.pose.position.x = 0.3
                safe_pose.pose.position.y = 0.0
                safe_pose.pose.position.z = self.safe_height
                safe_pose.pose.orientation.w = 1.0
                
                self.move_to_target(safe_pose, duration=3.0)
                self.current_state = ManipulatorState.IDLE
                self.state_start_time = time.time()
                
                if self.current_command == 'PICK':
                    # Notify coordinator that pick is complete
                    msg = Bool()
                    msg.data = True
                    self.operation_done_pub.publish(msg)
                    self.get_logger().info('Pick operation completed')
        
        elif self.current_state == ManipulatorState.MOVE_TO_PRE_PLACE:
            if self.target_place_pose and self.arm_ready:
                # Move to pre-place position
                pre_place = PoseStamped()
                pre_place.header.frame_id = self.target_place_pose.header.frame_id
                pre_place.pose.position.x = self.target_place_pose.pose.position.x
                pre_place.pose.position.y = self.target_place_pose.pose.position.y
                pre_place.pose.position.z = self.target_place_pose.pose.position.z + self.pre_grasp_height_offset
                pre_place.pose.orientation.w = 1.0
                
                self.move_to_target(pre_place, duration=3.0)
                self.current_state = ManipulatorState.MOVE_TO_PLACE
                self.state_start_time = time.time()
                self.get_logger().info('Moving to pre-place position')
        
        elif self.current_state == ManipulatorState.MOVE_TO_PLACE:
            if self.target_place_pose and self.arm_ready:
                # Move to actual place position
                self.move_to_target(self.target_place_pose, duration=2.0)
                self.current_state = ManipulatorState.RELEASE
                self.state_start_time = time.time()
                self.get_logger().info('Moving to place position')
        
        elif self.current_state == ManipulatorState.RELEASE:
            # Open gripper
            self.open_gripper()
            self.current_state = ManipulatorState.MOVE_TO_HOME
            self.state_start_time = time.time()
            self.get_logger().info('Releasing object')
        
        elif self.current_state == ManipulatorState.MOVE_TO_HOME:
            # Move back to home position
            home = PoseStamped()
            home.header.frame_id = 'base_footprint'
            home.pose.position.x = 0.3
            home.pose.position.y = 0.0
            home.pose.position.z = 0.5
            home.pose.orientation.w = 1.0
            
            self.move_to_target(home, duration=3.0)
            self.current_state = ManipulatorState.IDLE
            self.state_start_time = time.time()
            
            if self.current_command == 'PLACE':
                # Notify coordinator that place is complete
                msg = Bool()
                msg.data = True
                self.operation_done_pub.publish(msg)
                self.get_logger().info('Place operation completed')
            
            self.current_command = None

def main(args=None):
    rclpy.init(args=args)
    manipulator = Manipulator()
    rclpy.spin(manipulator)
    manipulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()