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

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

# Custom service
from exam_interfaces.srv import StartNavigation

ANGULAR_SPEED = 1.0  # radians per second

##################################
# Map Navigator Service
##################################
class MapNavigatorService(Node):
    def __init__(self):
        super().__init__('map_navigator_service')
        self.cb_group = ReentrantCallbackGroup()
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.current_yaw = None
        
        ##################################
        # Subscriptions
        ##################################
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'mobile_base_controller/odom',
            self.odom_callback,
            10,
            callback_group=self.cb_group
        )
        
        
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
    
    def odom_callback(self, msg: PoseStamped) -> None:
        """
        Callback function for odometry updates.

        Parameters
        ----------
        msg : PoseStamped
            The current pose of the robot.
        """
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
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
        self.get_logger().debug("Sending goal...")

        # Wait for the action server to be available
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            response.success = False
            return response
        
        self.stop()

        # Create goal message
        goal = NavigateToPose.Goal()

        # Set the goal pose from the request
        goal.pose = request.pose
                
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
        angular_speed = ANGULAR_SPEED  # rad/s
        # Calculate duration: Full circle (2*PI) / speed
        target_duration = (2 * math.pi) / angular_speed
        # Add a small buffer to ensure full coverage
        
        initial_yaw = self.current_yaw
        angle_thresh = 0.1  # radians

        # Create velocity message
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = angular_speed

        start_time = time.time()
        
        # Loop to publish velocity commands
        while (time.time() - start_time) < target_duration or abs(self.current_yaw - initial_yaw) > angle_thresh:
            # Check if ROS is still active
            if not rclpy.ok():
                response.success = False
                response.message = "Aborted: ROS shutdown"
                return response

            self.cmd_vel_pub.publish(vel_msg)
            time.sleep(0.1) # 10Hz publish rate

        # Stop the robot after rotation
        self.stop()

        self.get_logger().info("Rotation complete.")
        response.success = True
        response.message = "360 Rotation Completed"
        return response
    
    def stop(self) -> None:
        """
        Stops the robot by publishing zero velocities.
        """
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)

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