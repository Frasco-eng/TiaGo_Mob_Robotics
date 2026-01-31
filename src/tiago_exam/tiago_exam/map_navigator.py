#!/usr/bin/env python3

##################################
# Imports
##################################
import rclpy
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

        self.get_logger().info("Map Navigator Service Ready.")

    ##################################
    # Callbacks
    ##################################
    def handle_navigation(self, request, response):
        self.get_logger().info(f"Navigation request received to pose: {request.pose.pose.position.x:.2f}, {request.pose.pose.position.y:.2f}")
        
        # Check if Nav2 is available
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 not available!")
            response.success = False
            return response

        # Prepare Goal
        goal = NavigateToPose.Goal()
        goal.pose = request.pose
        
        # Send Goal to Nav2
        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            response.success = False
            return response
            
        self.get_logger().info("Navigation in progress...")
        
        # Wait for completion (Blocking for the service, but okay for sequential logic)
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        status = res_future.result().status
        if status == 4: # SUCCEEDED
            response.success = True
            self.get_logger().info("Destination reached!")
        else:
            response.success = False
            self.get_logger().warn("Navigation failed.")
            
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