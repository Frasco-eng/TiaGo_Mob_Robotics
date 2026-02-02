#!/usr/bin/env python3

##################################
# Imports
##################################
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Python Libraries
import time
import numpy as np

##################################
# Constatnts
##################################
ROTATION_VELOCITY = 1.0     # Radians per second 
TWO_PI = 2 * np.pi          # Full rotation in radians

##################################
# Global Localization Node
##################################
class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('global_localization_node')

        ##################################
        # QoS Profile Setup
        ##################################
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,         # Ensures message delivery
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # Keeps the last message for late subscribers
            history=HistoryPolicy.KEEP_LAST,                # Keep only the last message
            depth=1                                         # Depth of 1 for last message
        )

        
        
        ##################################
        # Subscribers
        ##################################
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.tuck_complete_sub = self.create_subscription(Bool, '/arm_tuck_complete', self.tuck_callback, qos_profile)

        ##################################
        # Publishers
        ##################################
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.loc_complete_pub = self.create_publisher(Bool, '/localization_complete', qos_profile)
        
        ##################################
        # Service Clients
        ##################################
        self.global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')

        ##################################
        # Thresholds and Flags
        ##################################
        self.pos_threshold = 0.03               # Variance X e Y (square meters)
        self.yaw_threshold = 0.03               # Variance Yaw (square radians)
        self.max_attempt_before_reset = 7       # Max attempts before resetting particles
        
        self.amcl_pose_received = False         # Flag to check if we received AMCL pose
        self.current_covariance = []            # Store the latest covariance
        self.tuck_complete_flag = False         # Flag to check if arm tuck is complete

        ##################################
        # Initialization Wait
        ##################################
        self.get_logger().info('Waiting for AMCL service...')
        while not self.global_loc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('AMCL service available.')

        self.get_logger().info('Global Localization node started.')

    ##################################
    # Callbacks
    ##################################
    def amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """
        Callback for AMCL pose updates.
        Updates the current covariance and sets the received flag.

        Parameters
        ----------
        msg : PoseWithCovarianceStamped
            The AMCL pose message containing covariance information.
        """
        # Raise flag
        self.amcl_pose_received = True

        # Update covariance
        self.current_covariance = msg.pose.covariance

    def tuck_callback(self, msg: Bool) -> None:
        """
        Callback for arm tuck status updates.
        Updates the tuck complete flag.

        Parameters
        ----------
        msg : Bool
            The message indicating if the arm tuck is complete.
        """
        self.tuck_complete_flag = msg.data
        
        if self.tuck_complete_flag:
            self.get_logger().info('Arm tuck complete signal received.')

    ##################################
    # Localization Methods
    ##################################
    def call_global_localization(self) -> None:
        """
        Calls the global localization service to spread particles across the map.   
        """
        self.get_logger().info('Calling Global Localization Service...')
        req = Empty.Request()
        future = self.global_loc_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Global Localization initialized. Particles are spread map-wide.')

    def localization_process(self) -> None:
        """
        Main localization process.
        Rotates the robot and checks for localization convergence.
        """
        # Call global localization to spread particles
        self.call_global_localization()
        
        # Wait for arm tuck to complete
        self.get_logger().info('Checking for arm tuck status...')
        while rclpy.ok() and not self.tuck_complete_flag:
            self.get_logger().info('Waiting for arm tuck to complete before localizing...')
            rclpy.spin_once(self, timeout_sec=1.0)
            time.sleep(1.0)
        
        # Start localization attempts
        attempt = 0  

        while rclpy.ok(): 
            self.get_logger().info(f'--- Localization Attempt {attempt+1} ---')

            if attempt >= self.max_attempt_before_reset:
                self.get_logger().warn(f'Failed for {attempt} times! Relaunching Global Localization to reset particles...')
                self.call_global_localization()
                attempt = 0  # Reset the counter to give new particles time to converge
                time.sleep(1.0) # Small technical pause
            
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

    def rotate(self) -> None:
        """
        Rotates the robot 360 degrees in place to gather sensor data for localization.
        Publishes velocity commands at a fixed frequency to ensure twist_mux compatibility.
        """
        # Create a Twist message for rotation
        vel_msg = Twist()
        vel_msg.angular.z = ROTATION_VELOCITY
        
        # Calculate duration based on speed
        duration = abs(TWO_PI / ROTATION_VELOCITY)
        freq = 10.0  # Frequency to publish commands (Hz)
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
        
    def stop(self) -> None:
        """
        Stops the robot by publishing zero velocities.
        """
        vel_msg = Twist()
        self.vel_pub.publish(vel_msg)
    
    def check_covariance(self) -> bool:
        """
        Checks if the localization covariance has dropped below the threshold,
        indicating successful localization.
        """
        if not self.amcl_pose_received:
            self.get_logger().warn('No AMCL pose received yet.')
            return False
        
        # Extract variances from covariance matrix
        var_x = self.current_covariance[0]
        var_y = self.current_covariance[7]
        var_yaw = self.current_covariance[35]

        self.get_logger().info(f'Var_x: {var_x:.4f}, Var_y: {var_y:.4f}, Var_yaw: {var_yaw:.4f}')

        # Compute metric
        if (var_x < self.pos_threshold and 
            var_y < self.pos_threshold and 
            var_yaw < self.yaw_threshold):
            return True
        return False


##################################
# Main Function
##################################
def main(args=None):
    rclpy.init(args=args)

    node = GlobalLocalizationNode()
    
    time.sleep(3.0)

    node.localization_process()

    node.destroy_node()
    rclpy.shutdown()

##################################
# Entry Point
##################################
if __name__ == '__main__':
    main()
