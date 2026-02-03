#!/usr/bin/env python3

##################################
# Imports
##################################
import rclpy
import time
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

# Custom service
from exam_interfaces.srv import StartNavigation

##################################
# Map Navigator Service
##################################
class MapNavigatorService(Node):
    def __init__(self):
        super().__init__('map_navigator_service')
        self.cb_group = ReentrantCallbackGroup()
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        ##################################
        # Service Server
        ##################################
        self.srv = self.create_service(StartNavigation, 'start_nav_srv', self.handle_navigation, callback_group=self.cb_group)
        
        self.rotate_srv = self.create_service(Trigger, 'rotate_360', self.handle_rotate_360, callback_group=self.cb_group)
        
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
    
    
    def handle_rotate_360(self, request, response):
        """
        Rotates the robot 360 degrees using open-loop velocity commands.
        """
        self.get_logger().info("Request received: Starting 360 rotation...")

        # Configuration
        angular_speed = 0.5  # rad/s
        # Calculate duration: Full circle (2*PI) / speed
        target_duration = (2 * math.pi) / angular_speed
        # Add a small buffer to ensure full coverage
        target_duration += 0.5 

        # Create velocity message
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = angular_speed

        start_time = time.time()
        
        # Loop to publish velocity commands
        while (time.time() - start_time) < target_duration:
            # Check if ROS is still active
            if not rclpy.ok():
                response.success = False
                response.message = "Aborted: ROS shutdown"
                return response

            self.cmd_vel_pub.publish(vel_msg)
            time.sleep(0.1) # 10Hz publish rate

        # Stop the robot after rotation
        vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(vel_msg)

        self.get_logger().info("Rotation complete.")
        response.success = True
        response.message = "360 Rotation Completed"
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