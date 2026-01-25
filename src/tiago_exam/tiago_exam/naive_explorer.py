#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav2_msgs.action import NavigateToPose

from geometry_msgs.msg import PoseStamped

import numpy as np
import cv2 
import yaml
import os
import random

ROTATION_VELOCITY = 1
TWO_PI = 2 * np.pi

def generate_random_goal(map_path,  num_goals, clearance=0.4,):
    print("Generating random pose from map:", map_path)
    try: 
        with open(map_path, 'r') as f:
            map_data = yaml.safe_load(f)
            
        resolution = map_data['resolution']
        origin = map_data['origin']
        pgm_path = os.path.join(os.path.dirname(map_path), map_data['image'])
        
        # 1. Load image and threshold it
        # Map pixels: 255 (white) is free, 0 (black) is occupied
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Map Image not found: {pgm_path}")

        # 2. Create a binary mask where free space is 1 and obstacles are 0
        # In ROS maps, usually > 250 is free.
        _, binary_map = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)

        # 3. Compute Distance Transform
        # This calculates the distance from every white pixel to the nearest zero pixel
        dist_map = cv2.distanceTransform(binary_map, cv2.DIST_L2, 5)

        # 4. Convert clearance from meters to pixels
        clearance_px = clearance / resolution

        # 5. Find all pixels where distance to obstacle is > clearance
        safe_indices = np.where(dist_map >= clearance_px)
        
        # zip indices into (row, col) pairs
        whitelist_pixels = list(zip(safe_indices[0], safe_indices[1]))

        if len(whitelist_pixels) == 0:
            print("No valid pixels found with the given clearance.")
            return None
        
        height, width = img.shape

        world_x = []
        world_y = []
        
        for _ in range(num_goals):
            # 6. Randomly select a pixel from the whitelist
            pixel_r, pixel_c = random.choice(whitelist_pixels)

            x = origin[0] + (pixel_c * resolution)
            y = origin[1] + (height - 1 - pixel_r) * resolution
            world_x.append(x)
            world_y.append(y)    

        return world_x, world_y
    
    except Exception as e:
        print("Error generating random goal:", e)
        return None
  


class NaiveExplorerNode(Node):
    def __init__ (self):
        super().__init__('naive_search_node')

        # QoS Profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # Parameters
        self.clearance = 0.4  
        self.map_path = self.declare_parameter('map_path', 'tiago_exam/share/tiago_exam/maps/group29_env_map.yaml').get_parameter_value().string_value
        
        self.wx, self.wy = generate_random_goal(self.map_path, num_goals=10, clearance=self.clearance)
      
        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        
        # Action Clients
        self.move_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Waiting for Nav2 Action Server...')
        self.move_action_client.wait_for_server()
        self.get_logger().info('Nav2 Action Server connected!')

        # Start the search
        self.current_goal_index = 0

        self.move_to_marker_sub = self.create_subscription(
            PoseStamped, 
            'move_to_marker', 
            self.move_to_marker_callback, 
            10
        )
        

        # self.rotation_timer_period = 0.1 # 10Hz
        # self.rotation_timer = self.create_timer(self.rotation_timer_period, self.rotation_callback)
        # self.rotation_timer.cancel()
        # self.rotation_start_time = None
        # self.rotation_duration = 0.0
        
        self.is_tracking_marker = False  # New flag
        self.send_next_goal()

    def move_to_marker_callback(self, msg):
        self.get_logger().info('Switching to MARKER TRACKING mode.')
        self.is_tracking_marker = True
        self.send_goal(msg)
        

    def send_next_goal(self):
        if self.current_goal_index < len(self.wx):
            x = self.wx[self.current_goal_index]
            y = self.wy[self.current_goal_index]
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = 'map'
            pose_goal.header.stamp = self.get_clock().now().to_msg()
            pose_goal.pose.position.x = x
            pose_goal.pose.position.y = y
            pose_goal.pose.position.z = 0.0
            pose_goal.pose.orientation.x = 0.0
            pose_goal.pose.orientation.y = 0.0
            pose_goal.pose.orientation.z = 0.0
            pose_goal.pose.orientation.w = 1.0
            self.send_goal(pose_goal)
            self.current_goal_index += 1
        else:
            self.get_logger().info('All goals have been attempted.')


    def send_goal(self, pose_goal):
        goal_msg = NavigateToPose.Goal()
    
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose = pose_goal

        x = pose_goal.pose.position.x
        y = pose_goal.pose.position.y

        self.get_logger().info(f'Sending goal: x={x:.2f}, y={y:.2f}')

        send_goal_future = self.move_action_client.send_goal_async(goal_msg)

        send_goal_future.add_done_callback(self.goal_response_callback)
        

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if not self.is_tracking_marker:
            self.send_next_goal()
        else:
            if status == 4: 
                self.get_logger().info('Marker reached! Resuming exploration...')
                self.is_tracking_marker = False
                self.send_next_goal()

     

    # def start_rotation(self):
    #     """Prepares and starts the non-blocking rotation timer"""
    #     self.rotation_duration = abs(TWO_PI / ROTATION_VELOCITY) * 2
    #     self.get_logger().info(f'Starting rotation for {self.rotation_duration:.2f} seconds...')
        
    #     self.rotation_start_time = self.get_clock().now()
        
    #     # Start the timer
    #     self.rotation_timer.reset()

    # def rotation_callback(self):
    #     """Called every 0.1 seconds to publish velocity"""
    #     elapsed = (self.get_clock().now() - self.rotation_start_time).nanoseconds / 1e9
        
    #     if elapsed < self.rotation_duration:
    #         vel_msg = Twist()
    #         vel_msg.angular.z = ROTATION_VELOCITY
    #         self.vel_pub.publish(vel_msg)
    #     else:
    #         # Time is up
    #         self.stop_and_continue()


    # def stop_and_continue(self):
    #     """Stops the robot, cancels rotation timer, and triggers next goal"""
    #     self.rotation_timer.cancel()
        
    #     vel_msg = Twist() # Zero velocity
    #     self.vel_pub.publish(vel_msg)

    #     self.get_logger().info('Rotation complete.')
    #     self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)

    node = NaiveExplorerNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        