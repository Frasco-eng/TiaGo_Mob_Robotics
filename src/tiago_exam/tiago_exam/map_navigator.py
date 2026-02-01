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

    def handle_navigation(self, request, response):
        self.get_logger().info("Sending goal...")

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            response.success = False
            return response

        goal = NavigateToPose.Goal()
        goal.pose = request.pose
        
        self.get_logger().info(f"Goal Pose: {goal.pose}")
                
        # 1. Wait for goal acceptance
        send_goal_future = self.nav_client.send_goal_async(goal)
        # Correct way to wait for a future in a service callback:
        while not send_goal_future.done():
            time.sleep(0.1) 
        
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            response.success = False
            return response

        # 2. Wait for the result
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
# Main Functions
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