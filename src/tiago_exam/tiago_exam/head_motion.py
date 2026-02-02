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

# Python Libraries
import time

##################################
# Joint Names
##################################
JOINT_NAMES = [
    'head_1_joint',  # Pan
    'head_2_joint'   # Tilt
]

##################################
# Joint Limits
##################################
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

        ##################################
        # Callback Group
        ##################################
        self.cb_group = ReentrantCallbackGroup()
        
        ##################################
        # Service Server
        ##################################
        self.srv = self.create_service(MoveHead, 'move_head_srv', self.handle_move_head, callback_group=self.cb_group)
        
        ##################################
        # Action Client
        ##################################
        self.head_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory', callback_group=self.cb_group)

        self.get_logger().info("Head Motion Service Ready.")

    ##################################
    # Callback
    ##################################
    def handle_move_head(self, request, response) -> MoveHead.Response:
        """
        Handle Move Head Service Request.

        Parameters
        ----------
        request : MoveHead.Request
            The service request containing pan and tilt angles.
        response : MoveHead.Response
            The service response indicating success or failure.

        Returns
        -------
        MoveHead.Response
            The response with success status.
        """

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

        # Send Goal
        send_goal_future = self.head_client.send_goal_async(goal)
        
        # Wait for the result
        while not send_goal_future.done():
            time.sleep(0.1) 
        
        # Get the goal handle
        goal_handle = send_goal_future.result()

        # Check if goal was accepted
        if not goal_handle.accepted:
            response.success = False
            return response
        
        # Get the result
        result_future = goal_handle.get_result_async()

        # Wait for the result
        while not result_future.done():
            time.sleep(0.1)

        # Get navigation result
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