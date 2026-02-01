#!/usr/bin/env python3

##################################
# Imports
##################################
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Custom Service
from exam_interfaces.srv import MoveHead

import time
JOINT_NAMES = [
    'head_1_joint',  # Pan
    'head_2_joint'   # Tilt
]

HEAD_UP = 0.7
HEAD_DOWN = -0.95
HEAD_LEFT = 1.2
HEAD_RIGHT = -1.2

##################################
# Head Motion Service Node
##################################
class HeadMotionService(Node):
    def __init__(self):
        super().__init__('head_motion_service')

        self.cb_group = ReentrantCallbackGroup()
        
        self.srv = self.create_service(MoveHead, 'move_head_srv', self.handle_move_head, callback_group=self.cb_group)
        self.head_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory', callback_group=self.cb_group)

        self.get_logger().info("Head Motion Service Ready.")

    def handle_move_head(self, request, response):
        # Check server availability
        if not self.head_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Head controller not available!")
            response.success = False
            return response

        # Prepare Goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(request.pan), float(request.tilt)]
        pt.time_from_start = Duration(sec=0, nanosec=500_000_000)
        goal.trajectory.points = [pt]

        # 1. Send Goal (Async -> Wait for result)
        send_goal_future = self.head_client.send_goal_async(goal)
        
        while not send_goal_future.done():
            time.sleep(0.1) 
            
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            response.success = False
            
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)

        nav_result = result_future.result() 

        # Status 4 = SUCCEEDED
        if nav_result.status == 4:
            response.success = True
        else:
            response.success = False
            
        return response
            
            

##################################
# Main Function
##################################
def main(args=None):
    rclpy.init(args=args)
    node = HeadMotionService()

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