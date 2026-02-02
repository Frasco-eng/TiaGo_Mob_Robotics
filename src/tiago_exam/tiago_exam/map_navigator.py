#!/usr/bin/env python3

##################################
# Imports
##################################
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped

# Custom service
from exam_interfaces.srv import StartNavigation

##################################
# Map Navigator Service
##################################
class MapNavigatorService(Node):
    def __init__(self):
        super().__init__('map_navigator_service')
        self.cb_group = ReentrantCallbackGroup()
        
        ##################################
        # Service Server
        ##################################
        self.srv = self.create_service(StartNavigation, 'start_nav_srv', self.handle_navigation, callback_group=self.cb_group)
        
        ##################################
        # Action Client
        ##################################
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)

    ##################################
    # Callback
    ##################################
    def handle_navigation(self, request: PoseStamped, response: bool) -> bool:
        """
        Handle navigation requests by sending goals to the NavigateToPose action server.

        Parameters
        ----------
        request : PoseStamped
            The target pose for navigation.
        response : bool
            The response indicating success or failure of the navigation.

        Returns
        -------
        response : bool
            True if navigation succeeded, False otherwise.
        """
        self.get_logger().info("Sending goal...")

        # Wait for the action server to be available
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            response.success = False
            return response

        # Create goal message
        goal = NavigateToPose.Goal()

        # Set the goal pose from the request
        goal.pose = request.pose
        
        self.get_logger().info(f"Goal Pose: {goal.pose}")
                
        # Wait for goal acceptance
        send_goal_future = self.nav_client.send_goal_async(goal)

        # Wait until the goal is accepted
        while not send_goal_future.done():
            time.sleep(0.1) 
        
        # Get the goal handle
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            response.success = False
            return response

        # Wait for the result
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)

        # Get the navigation result
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
    navigator = MapNavigatorService()
    
    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

##################################
# Entry Point
##################################
if __name__ == '__main__':
    main()