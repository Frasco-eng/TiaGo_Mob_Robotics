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

# Custom Service
from exam_interfaces.srv import MoveHead

##################################
# Head Motion Service Node
##################################
class HeadMotionService(Node):
    def __init__(self):
        super().__init__('head_motion_service')
        
        ##################################
        # Service Server
        ##################################
        self.srv = self.create_service(MoveHead, 'move_head_srv', self.handle_move_head)
        
        ##################################
        # Action Client
        ##################################
        self.head_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

        self.get_logger().info("Head Motion Service Ready.")

    ##################################
    # Callbacks
    ##################################
    def handle_move_head(self, request, response):
        self.get_logger().info(f"Request received: Pan={request.pan}, Tilt={request.tilt}")

        # Check if the controller is available
        if not self.head_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Head controller not available!")
            response.success = False
            return response

        # Prepare the goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [float(request.pan), float(request.tilt)]
        pt.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [pt]

        # Send the goal
        send_goal_future = self.head_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            response.success = False
            return response
        
        # Wait for the final result of the motion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        response.success = True
        self.get_logger().info("Head motion completed.")
        return response

##################################
# Main Function
##################################
def main(args=None):
    rclpy.init(args=args)
    node = HeadMotionService()

    try:
        rclpy.spin(node)
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