#!/usr/bin/env python3

##################################
# Imports
##################################
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Python Libraries
import time
import numpy as np
import matplotlib.pyplot as plt
import os

# Custom Imports (Assuming this module exists in your environment)
from tiago_exam import utils

##################################
# Constants
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
        # Parameters & Path Setup
        ##################################
        # Resolve the path dynamically using the utils module
        try:
            ws_src_dir = utils.get_package_src_directory('tiago_exam')
            path_dir = os.path.join(ws_src_dir, 'temp')
            # Ensure the directory exists
            if not os.path.exists(path_dir):
                os.makedirs(path_dir)
            default_plot_path = os.path.join(path_dir, 'covariance_plot.png')
        except Exception as e:
            self.get_logger().warn(f'Could not resolve package path: {e}. Using default local path.')
            default_plot_path = 'covariance_plot.png'

        self.declare_parameter('covariance_plot_path', default_plot_path)
        self.plot_path = self.get_parameter('covariance_plot_path').get_parameter_value().string_value
        
        ##################################
        # QoS Profile Setup
        ##################################
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        ##################################
        # Subscribers
        ##################################
        # AMCL callback now handles logging and plotting directly
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, qos_profile)
        
        ##################################
        # Publishers
        ##################################
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        ##################################
        # Service Clients
        ##################################
        self.global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')

        ##################################
        # Thresholds and Flags
        ##################################
        self.pos_threshold = 0.03               # Variance X and Y
        self.yaw_threshold = 0.03               # Variance Yaw
        self.max_attempt_before_reset = 7       
        
        self.amcl_pose_received = False
        self.current_covariance = []

        # Data Logging for Plotting
        self.log_var_x = []
        self.log_var_y = []
        self.log_var_yaw = []
        self.log_steps = []
        self.msg_counter = 0

        ##################################
        # Initialization Wait
        ##################################
        self.get_logger().info('Waiting for AMCL service...')
        while not self.global_loc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('AMCL service available.')

        self.get_logger().info(f'Global Localization node started. Plot updating live at: {self.plot_path}')

    ##################################
    # Callbacks
    ##################################
    def amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """
        Callback for AMCL pose updates.
        Logs data and updates the plot on every message received.
        """
        self.amcl_pose_received = True
        self.current_covariance = msg.pose.covariance

        # Extract variances
        # Covariance matrix is 6x6 (36 elements). Index 0=X, 7=Y, 35=Yaw
        var_x = self.current_covariance[0]
        var_y = self.current_covariance[7]
        var_yaw = self.current_covariance[35]

        # Update Logs
        self.msg_counter += 1
        self.log_steps.append(self.msg_counter)
        self.log_var_x.append(var_x)
        self.log_var_y.append(var_y)
        self.log_var_yaw.append(var_yaw)

        # Trigger Plot Update (Real-time tracking)
        # Note: Saving a file every callback can be heavy. 
        # If AMCL is very fast (>10Hz), consider plotting every Nth message.
        # For now, we plot every message as requested.
        self.save_covariance_plot()

    ##################################
    # Localization Methods
    ##################################
    def call_global_localization(self) -> None:
        self.get_logger().info('Calling Global Localization Service...')
        req = Empty.Request()
        future = self.global_loc_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Global Localization initialized.')

        # Reset logs if we re-initialize globally
        self.log_var_x.clear()
        self.log_var_y.clear()
        self.log_var_yaw.clear()
        self.log_steps.clear()
        self.msg_counter = 0

    def localization_process(self) -> None:
        """
        Main Loop: Controls the robot motion. 
        The plotting is now handled asynchronously in the callback.
        """
        self.call_global_localization()
        
        attempt = 0  

        while rclpy.ok(): 
            self.get_logger().info(f'--- Localization Attempt {attempt+1} ---')

            if attempt >= self.max_attempt_before_reset:
                self.get_logger().warn(f'Failed for {attempt} times! Resetting particles...')
                self.call_global_localization()
                attempt = 0 
                time.sleep(1.0)
            
            self.rotate()
            
            # Spin to ensure callbacks are processed (and plots updated)
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)

            # Check convergence
            if self.check_covariance(): 
                self.get_logger().info('SUCCESS: Robot is localized!')
                break
            else:
                self.get_logger().info('Not converged yet. Rotating again...')
            
            attempt += 1

    def rotate(self) -> None:
        vel_msg = Twist()
        vel_msg.angular.z = ROTATION_VELOCITY
        duration = abs(TWO_PI / ROTATION_VELOCITY)
        freq = 10.0
        
        start_time = self.get_clock().now()
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.vel_pub.publish(vel_msg)
            # Spin is crucial here: it triggers amcl_callback -> saves plot
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(1.0 / freq)
        
        self.stop()
        self.get_logger().info('Rotation finished. Waiting for particles to settle...')
        time.sleep(1.0)
        
    def stop(self) -> None:
        vel_msg = Twist()
        self.vel_pub.publish(vel_msg)
    
    def check_covariance(self) -> bool:
        """
        Checks thresholds only. Logging is done in callback.
        """
        if not self.amcl_pose_received:
            self.get_logger().warn('No AMCL pose received yet.')
            return False
        
        var_x = self.current_covariance[0]
        var_y = self.current_covariance[7]
        var_yaw = self.current_covariance[35]

        self.get_logger().info(f'Current Variances -> X: {var_x:.4f}, Y: {var_y:.4f}, Yaw: {var_yaw:.4f}')

        if (var_x < self.pos_threshold and 
            var_y < self.pos_threshold and 
            var_yaw < self.yaw_threshold):
            return True
        return False

    def save_covariance_plot(self) -> None:
        """
        Generates and overwrites the plot file with the latest data.
        """
        try:
            plt.switch_backend('Agg') # Ensure headless mode
            plt.figure(figsize=(10, 6))
            
            # Plot the full history
            plt.plot(self.log_steps, self.log_var_x, label='Var X', color='blue', linewidth=1)
            plt.plot(self.log_steps, self.log_var_y, label='Var Y', color='orange', linewidth=1)
            plt.plot(self.log_steps, self.log_var_yaw, label='Var Yaw', color='green', linewidth=1)
            
            # Add Threshold lines
            plt.axhline(y=self.pos_threshold, color='red', linestyle='--', alpha=0.5, label='Pos Threshold')
            plt.axhline(y=self.yaw_threshold, color='purple', linestyle='--', alpha=0.5, label='Yaw Threshold')

            plt.title('AMCL Covariance Evolution (Live)')
            plt.xlabel('AMCL Updates (count)')
            plt.ylabel('Variance')
            plt.legend()
            plt.grid(True)
            
            # Save
            plt.savefig(self.plot_path)
            plt.close()
            
        except Exception as e:
            self.get_logger().error(f'Failed to update plot: {str(e)}')

##################################
# Main Function
##################################
def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()
    
    # Wait briefly for connection
    time.sleep(2.0)
    
    node.localization_process()
    
    node.destroy_node()
    rclpy.shutdown()

##################################
# Entry Point
##################################
if __name__ == '__main__':
    main()