#!/usr/bin/env python3

##################################
# Imports
##################################
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped

# Custom Services
from exam_interfaces.srv import InitGoals
from exam_interfaces.srv import GetNextGoal

# Python Standard Libraries
import yaml
import os
import cv2
import numpy as np
import random
from threading import Thread

##################################
# Naive Explorer Node
##################################
class NaiveExplorer(Node):
    def __init__(self):
        super().__init__('naive_explorer')
        
        ##################################
        # Reentrant Callback Group
        ##################################
        self.cb_group = ReentrantCallbackGroup()
        
        #################################
        # Client Services
        #################################
        self.create_service(InitGoals, 'init_goals', self.init_goals_callback, callback_group=self.cb_group)
        self.create_service(GetNextGoal, 'get_next_goal', self.get_next_goal_callback, callback_group=self.cb_group)

        ##################################
        # Class Parameters
        ##################################
        # Retrieve paths
        ws_src_path = os.path.join(os.path.expanduser('~'), 'tiago_gh_ws/src/tiago_exam')
        map_path_dir = os.path.join(ws_src_path, 'maps')
        
        # Declare parameters fallback
        self.map_path = self.declare_parameter('map_path', os.path.join(map_path_dir, 'group29_env_map.yaml')).get_parameter_value().string_value
        
        ##################################
        # State Variables
        ##################################
        self.exploration_goals = []
        self.init_goals = None
        self.clearance = None
        self.current_expl_goal_idx = 0

        self.get_logger().info("Explorer Initialized.")

    ##################################
    # Callbacks
    ##################################
    def init_goals_callback(self, request, response):
        """
        Service callback to generate new exploration goals.

        Parameters
        ----------
        request : InitGoals.Request
            The service request (not used).
        response : InitGoals.Response
            The service response (not used).

        Returns
        -------
        InitGoals.Response
            The service response.
        """
        
        self.get_logger().info("Initializing Exploration Goals...")
        self.init_goals = request.init_goals 
        self.clearance = request.clearance
        
        wx, wy = self.generate_random_goals()
        self.exploration_goals = []
        
        if wx:
            for x, y in zip(wx, wy):
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.pose.position.x, p.pose.position.y = float(x), float(y)
                
                random_yaw = random.uniform(0, 2 * np.pi)

                # 2. Convert Yaw (Z-axis rotation) to Quaternion
                # Formula: x=0, y=0, z=sin(yaw/2), w=cos(yaw/2)
                p.pose.orientation.x = 0.0
                p.pose.orientation.y = 0.0
                p.pose.orientation.z = np.sin(random_yaw / 2.0)
                p.pose.orientation.w = np.cos(random_yaw / 2.0)
                
                self.exploration_goals.append(p)
            self.current_expl_goal_idx = 0
            
        success = len(self.exploration_goals) == self.init_goals
        response.success = success
        
        return response
    
    ##################################
    # Helper Functions
    ##################################
    def generate_random_goals(self) -> tuple[list, list]:
        """
        Generates random goal coordinates within the map that are safe based on the specified clearance.

        Parameters
        ----------
        map_path : str
            The file path to the map YAML file.
        num_goals : int
            The number of random goals to generate.
        clearance : float, optional
            The minimum clearance distance from obstacles in meters, by default 0.4

        Returns
        -------
        tuple[list, list]
            Two lists containing the x and y coordinates of the generated goals.
        """
        try:
            # Open and parse the map YAML file
            with open(self.map_path, 'r') as f:
                map_data = yaml.safe_load(f)

            # Extract map parameters
            resolution = map_data['resolution']
            origin = map_data['origin'] # world origin (bottom-left corner) ||| pixel origin (top-left corner)

            # Load the map image
            pgm_path = os.path.join(os.path.dirname(self.map_path), map_data['image'])
            img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            if img is None: return [], []

            # Create binary map
            _, binary_map = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)

            # Compute distance transform
            dist_map = cv2.distanceTransform(binary_map, cv2.DIST_L2, 5)

            # Identify safe pixels based on clearance
            safe_indices = np.where(dist_map >= (self.clearance / resolution))

            # Generate whitelist of safe pixels
            whitelist_pixels = list(zip(safe_indices[0], safe_indices[1]))
            if not whitelist_pixels: return [], []
            wx, wy = [], []
            h, w = img.shape

            # Randomly select goal pixels and convert to world coordinates
            for _ in range(self.init_goals):
                r, c = random.choice(whitelist_pixels) # row, col (pixel coordinates)
                wx.append(origin[0] + c * resolution) # x, y (world coordinates)
                wy.append(origin[1] + (h - r) * resolution -1.3)
            return wx, wy
        except Exception as e:
            print(f"Map Error: {e}")
            return [], []

    def get_next_goal_callback(self, request, response):
        response.success = True
        response.is_last_goal = False
        

        self.get_logger().info(f"Requesting Next Exploration Goal {self.current_expl_goal_idx+1}")

        # Add exploration goals if finished
        if self.current_expl_goal_idx >= len(self.exploration_goals):
            response.is_last_goal = True
            return response

        # Retrieve current exploration goal
        goal = self.exploration_goals[self.current_expl_goal_idx]
        self.get_logger().info(f"Exploration Goal {self.current_expl_goal_idx+1}")

        # Send navigation goal
        response.next_goal = goal
        self.current_expl_goal_idx += 1
        
        
        
        return response
                    

##################################
# Main Method
##################################
def main(args=None):
    rclpy.init(args=args)
    node = NaiveExplorer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
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
