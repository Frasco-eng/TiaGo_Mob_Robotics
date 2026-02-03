#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool

class ArmTucker(Node):

    def __init__(self):
        super().__init__('arm_tucker')
        
        # We connect to MoveIt (move_group) instead of play_motion2
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Define the 'home' (tucked) position for TIAGo
        # These are the standard joint values for safely tucking the arm
        self.home_joints = [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.00]
        self.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Configuration for the publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,         # Ensures message delivery
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # Keeps the last message for late subscribers
            history=HistoryPolicy.KEEP_LAST,                # Keep only the last message
            depth=1                                         # Depth of 1 for last message
        )

        # Publisher to notify when tuck is complete
        self.tuck_complete_pub = self.create_publisher(Bool, '/arm_tuck_complete', qos_profile)
        
        self.success = False

    def tuck_arm(self):
        self.get_logger().info('Waiting for MoveIt action server...')
        if not self._action_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('MoveIt server not available!')
            return False

        self.get_logger().info('Sending TUCK goal...')
        
        # 1. Create the Goal Message
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # 2. Define Constraints (The Target Joint Values)
        constraints = Constraints()
        for i in range(7):
            jc = JointConstraint()
            jc.joint_name = self.joint_names[i]
            jc.position = self.home_joints[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraints)

        # 3. Send the Goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by MoveIt')
            return

        self.get_logger().info('Goal accepted by MoveIt')
        self._get_result_future = goal_handle.get_result_async()
        
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        tuck_msg = Bool()
        # Error code 1 means SUCCESS
        if result.error_code.val == 1:
            self.get_logger().info('Arm TUCKED successfully!')
            
            tuck_msg.data = True
            self.tuck_complete_pub.publish(tuck_msg)

            self.success = True
        else:
            self.get_logger().error(f'Motion failed with error code: {result.error_code.val}')
            tuck_msg.data = False
            self.tuck_complete_pub.publish(tuck_msg)
            self.success = False
        
            
def main(args=None):
    rclpy.init(args=args)
    node = ArmTucker()

    success = False
    # Simple retry loop
    for i in range(3):
        node.tuck_arm()
        if node.success:
            success = True
            break
            
        node.get_logger().warn(f'Retrying... ({i+1}/3)')

    if success:
        node.get_logger().info('Tuck complete. Node staying alive to hold latch...')
        
        try:
            rclpy.spin(node) 
        except KeyboardInterrupt:
            pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()