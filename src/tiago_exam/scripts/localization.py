#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_srvs.srv import Empty
import time
import numpy as np

from std_msgs.msg import Bool

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

ROTATION_VELOCITY = 1.0  # radians per second 
TWO_PI = 2 * np.pi

class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('global_localization_node')
        self.get_logger().info('Global Localization node started.')

        # Configuration for the publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,         # Ensures message delivery
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # Keeps the last message for late subscribers
            history=HistoryPolicy.KEEP_LAST,                # Keep only the last message
            depth=1                                         # Depth of 1 for last message
        )


        # Subscribers
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.tuck_complete_sub = self.create_subscription(Bool, '/arm_tuck_complete', self.tuck_callback, qos_profile)

        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.loc_complete_pub = self.create_publisher(Bool, '/localization_complete', qos_profile)

        # Service Client for Global Localization
        # This service tells AMCL: "I am lost, spread particles everywhere!"
        self.global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')

        # Initializations
        # Threshold: We need a small variance to confirm we are found.
        # Note: You might need to relax this (e.g., 0.1 or 0.2) if it never finishes.
        self.covariance_threshold = 0.03 
        
        self.amcl_pose_received = False     # Flag to check if we received AMCL pose
        self.current_covariance = []        # Store the latest covariance
        self.tuck_complete_flag = False   # Flag to check if arm tuck is complete

        self.get_logger().info('Waiting for AMCL service...')
        while not self.global_loc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('AMCL service available.')

    def amcl_callback(self, msg):
        self.amcl_pose_received = True
        self.current_covariance = msg.pose.covariance

    def tuck_callback(self, msg):
        self.tuck_complete_flag = msg.data
        if self.tuck_complete_flag:
            self.get_logger().info('Arm tuck complete signal received.')

    def call_global_localization(self):
        self.get_logger().info('Calling Global Localization Service...')
        req = Empty.Request()
        future = self.global_loc_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Global Localization initialized. Particles are spread map-wide.')

    def localization_process(self):
        # Call global localization to spread particles
        self.call_global_localization()
        
        # Rotate until the particles converge
        self.get_logger().info('Checking for arm tuck status...')
        while rclpy.ok() and not self.tuck_complete_flag:
            self.get_logger().info('Waiting for arm tuck to complete before localizing...')
            rclpy.spin_once(self, timeout_sec=1.0)
            time.sleep(1.0)
    
        attempt = 0  
        while rclpy.ok(): 
            self.get_logger().info(f'--- Localization Attempt {attempt+1} ---')
            
            # Rotate to gather sensor data
            self.rotate()
            
            # Spin to process AMCL messages
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)

            # Check if we have converged
            if self.check_covariance(): 
                self.get_logger().info('SUCCESS: Robot is localized!')
                msg = Bool()
                msg.data = True
                self.loc_complete_pub.publish(msg)
                time.sleep(1.0)  # Ensure message is sent
                break
            else:
                self.get_logger().info('Not converged yet. Rotating again...')
            
            attempt += 1

    def rotate(self):
        vel_msg = Twist()
        vel_msg.angular.z = ROTATION_VELOCITY
        
        # Calculate duration based on speed
        duration = abs(TWO_PI / ROTATION_VELOCITY)
        freq = 10.0  # Hz
        self.get_logger().info(f'Rotating for {duration:.2f} seconds at {freq} Hz...')

        # Use the ROS clock for simulation-accurate timing
        start_time = self.get_clock().now()
        
        # Loop to keep the command alive for the twist_mux
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.vel_pub.publish(vel_msg)
            
            # Process callbacks (to receive AMCL updates while moving)
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Small sleep to maintain frequency
            time.sleep(1.0 / freq)
        
        # End rotation
        self.stop()
        self.get_logger().info('Rotation finished. Waiting for particles to settle...')
        time.sleep(1.0)
        
    def stop(self):
        vel_msg = Twist()
        self.vel_pub.publish(vel_msg)
    
    def check_covariance(self):
        if not self.amcl_pose_received:
            self.get_logger().warn('No AMCL pose received yet.')
            return False
        
        # In Global Localization, the covariance starts HUGE (e.g., 30.0)
        # We wait for it to drop below our threshold.
        # We look at x (idx 0) and y (idx 7) variance specifically.
        # cov_x = self.current_covariance[0]
        # cov_y = self.current_covariance[7]
        # cov_yaw = self.current_covariance[35]

        # Compute max metric
        max_cov = np.max(self.current_covariance)
        self.get_logger().info(f'Current Covariance (Max): {max_cov:.4f}')

        if max_cov < self.covariance_threshold:
            return True
        else: 
            return False
    
def main(args=None):
    rclpy.init(args=args)

    node = GlobalLocalizationNode()
    
    # Wait for the system to settle before starting
    time.sleep(3.0)

    node.localization_process()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist

# import time
# import numpy as np

# ROTATION_VELOCITY = 0.5  # radians per second 
# TWO_PI = 2 * np.pi

# # AMCL: Adaptive Monte Carlo Localization

# class LocalizationNode(Node):
#     def __init__(self):
#         super().__init__('localization_node')
#         self.get_logger().info('Localization node has been started.')

#         # Subscribers
#         self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
#         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

#         # Publishers
#         self.amcl_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initial_pose', 10)
#         self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Initializations
#         self.covariance_threshold = 0.07

#         self.covariance_msg = PoseWithCovarianceStamped()
#         self.covariance_values = [0.0] * 36 #PoseWithCovarianceStamped() 

#         self.tiago_pose = [0.0, 0.0, 0.0]  # x, y, z
#         self.tiago_orientation = [0.0, 0.0, 0.0, 1.0]  # quaternion x, y, z, w

#         self.get_logger().info('Waiting for the initial position ...')

#         # Spin once
#         rclpy.spin_once(self, timeout_sec=1.0)

#     def amcl_callback(self, msg):
#         self.amcl_position = msg.pose.pose.position
#         self.amcl_orientation = msg.pose.pose.orientation
#         self.covariance_msg.pose.covariance = msg.pose.covariance

#     def odom_callback(self, msg):
#         x_o = msg.pose.pose.position.x
#         y_o = msg.pose.pose.position.y
#         z_o = msg.pose.pose.position.z
#         w_o = msg.pose.pose.orientation.w

#         cov = np.zeros(36, dtype=np.float64)
#         cov[0] = 200  # x 
#         cov[7] = 200  # y
#         cov[35] = 1   # yaw
#         self.covariance_values = cov

#         self.tiago_orientation = [x_o, y_o, z_o, w_o]

#     def pub_initial_pose(self):
#         rclpy.spin_once(self, timeout_sec=1.0) # Spins each fcn once to get the latest data
#         initial_pose_msg = PoseWithCovarianceStamped()
#         initial_pose_msg.pose.pose.position.x = float(self.tiago_pose[0])
#         initial_pose_msg.pose.pose.position.y = float(self.tiago_pose[1])
#         initial_pose_msg.pose.pose.position.z = float(self.tiago_pose[2])

#         initial_pose_msg.pose.pose.orientation.x = float(self.tiago_orientation[0])
#         initial_pose_msg.pose.pose.orientation.y = float(self.tiago_orientation[1])
#         initial_pose_msg.pose.pose.orientation.z = float(self.tiago_orientation[2])
#         initial_pose_msg.pose.pose.orientation.w = float(self.tiago_orientation[3])

#         if hasattr(self.covariance_values, 'tolist'):
#             initial_pose_msg.pose.covariance = self.covariance_values.tolist()
#         else:
#             initial_pose_msg.pose.covariance = self.covariance_values

#         initial_pose_msg.header.frame_id = 'map'

#         initial_pose_msg.header.stamp = self.get_clock().now().to_msg()

#         self.amcl_pose_pub.publish(initial_pose_msg)

#     def localization(self):
#         while True:
#             self.get_logger().info('Localization in progress ...')
#             self.rotate() # Rotate to gather sensor data
#             rclpy.spin_once(self, timeout_sec=1.0) # Update the covariance message

#             if self.check_covariance(): # Check if covariance is below threshold
#                 self.get_logger().info('Localization successful!')
#                 break
        
#     def rotate(self):
#         vel_msg = Twist()
#         vel_msg.angular.z = ROTATION_VELOCITY
#         self.vel_pub.publish(vel_msg)
#         self.get_logger().info(f'Publishing velocity command: {vel_msg}')
#         time.sleep(abs(TWO_PI / ROTATION_VELOCITY)) # Rotate for a full circle
#         self.stop()

#     def stop(self):
#         vel_msg = Twist()
#         vel_msg.angular.z = 0.0
#         vel_msg.linear.x = 0.0
#         self.vel_pub.publish(vel_msg)
#         self.get_logger().info(f'Publishing stop: {vel_msg}')
    
#     def check_covariance(self):
#         covariance_values = self.covariance_msg.pose.covariance

#         if np.max(covariance_values) < self.covariance_threshold:
#             self.get_logger().info(f'Covariance below the threshold.')
#             self.get_logger().info(f'Robot is sufficiently localized.')
#             return True
#         else: 
#             self.get_logger().info(f'Covariance above the threshold.')
#             return False
    
# def main(args=None):
#     rclpy.init(args=args)

#     node = LocalizationNode()
#     time.sleep(5)

#     node.pub_initial_pose()
#     node.localization()

#     node.destroy_node()
#     rclpy.shutdown()
    

# if __name__ == '__main__':
#     main()
    
    