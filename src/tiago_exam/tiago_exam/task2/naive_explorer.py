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
from tiago_exam_interfaces.srv import InitGoals
from tiago_exam_interfaces.srv import GetNextGoal

from tiago_exam import utils


# Python Standard Libraries
import yaml
import os
import cv2
import numpy as np
import random
import os
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
        ws_src_dir = utils.get_package_src_directory('tiago_exam')
        map_path_dir = os.path.join(ws_src_dir, 'maps')
        
        # Declare parameters fallback
        self.map_path = self.declare_parameter('map_path', os.path.join(map_path_dir, 'group29_env_map.yaml')).get_parameter_value().string_value
        
        ##################################
        # State Variables
        ##################################
        self.exploration_goals = []         # List of exploration goals
        self.init_goals = None              # Number of exploration goals to initialize
        self.clearance = None               # Clearance from obstacles for goal generation
        self.current_expl_goal_idx = 0      # Current exploration goal index

        self.get_logger().info("Explorer Initialized.")

    ##################################
    # Callbacks
    ##################################
    def init_goals_callback(self, request, response) -> InitGoals.Response:
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

        # Unpack request parameters
        self.init_goals = request.init_goals 
        self.clearance = request.clearance
        
        # Generate random exploration goals
        wx, wy = self.generate_random_goals()
        self.exploration_goals = []
        
        # Loop over valid goal coordinates and append PoseStamped messages
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
    
    def get_next_goal_callback(self, request, response) -> GetNextGoal.Response:
        """
        Service callback to provide the next exploration goal.
        Parameters
        ----------
        request : GetNextGoal.Request
            The service request (not used).
        response : GetNextGoal.Response
            The service response containing the next goal.

        Returns
        -------
        GetNextGoal.Response
            The service response with the next exploration goal.
        """
        # Initialize response
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
            origin = map_data['origin']

            # Default safe coordinates (x, y, z, theta)
            safe_coords = [0.0, -1.3, 0.0, 0.0] 

            # Load the map image
            pgm_path = os.path.join(os.path.dirname(self.map_path), map_data['image'])
            img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            if img is None: return [], []

            # Create binary map
            _, binary_map = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)

            # Compute distance transform
            dist_map = cv2.distanceTransform(binary_map, cv2.DIST_L2, 5)

            # Identify safe pixels based on clearance
            effective_clearance_px = (self.clearance + 0.1) / resolution
            
            safe_indices = np.where(dist_map >= effective_clearance_px)
            whitelist_pixels = list(zip(safe_indices[0], safe_indices[1]))

            # Generate whitelist of safe pixels
            whitelist_pixels = list(zip(safe_indices[0], safe_indices[1]))
            if not whitelist_pixels: return [], []
            mx, my = [], []
            h, w = img.shape

            # Randomly select goal pixels and convert to map coordinates
            # NOTE: this conversion happens in the map reference frame because the navigation stack will use the map frame as reference.
            for _ in range(self.init_goals):
                r, c = random.choice(whitelist_pixels) # row, col (pixel coordinates)
                mx.append(origin[0] + c * resolution) # x, y (map coordinates)
                my.append(origin[1] + (h - r) * resolution)


            # --- DEBUG SECTION START ---
            temp_dir_path = os.path.join(utils.get_package_src_directory('tiago_exam'), 'temp')
            if not os.path.exists(temp_dir_path):
                os.makedirs(temp_dir_path)
            # Create a 3-channel BGR image for colorful markers
            debug_img_bgr = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)

            # Highlight Safe Zone (Light Green)
            for r, c in whitelist_pixels:
                debug_img_bgr[r, c] = [200, 255, 200] 

            # Draw OpenCV Origin & Axes (Top-Left)
            # CV Origin is (0,0). X is Col (Right), Y is Row (Down)
            cv2.circle(debug_img_bgr, (5, 5), 5, (0, 0, 255), -1) 
            cv2.arrowedLine(debug_img_bgr, (5, 5), (55, 5), (0, 0, 255), 2, tipLength=0.3)  # Red X
            cv2.arrowedLine(debug_img_bgr, (5, 5), (5, 55), (0, 255, 0), 2, tipLength=0.3)  # Green Y
            cv2.putText(debug_img_bgr, "CV Origin", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw ROS World Origin & Axes
            ros_origin_px_x = int((0.0 - origin[0]) / resolution)
            ros_origin_px_y = int(h - ((0.0 - origin[1] - safe_coords[1]) / resolution))

            map_origin_px_x = int((0.0 - origin[0]) / resolution)
            map_origin_px_y = int(h - ((0.0 - origin[1]) / resolution))

            pgm_origin_px_x = int((origin[0] - origin[0]) / resolution)
            pgm_origin_px_y = int(h - ((origin[1] - origin[1]) / resolution))

            cv2.drawMarker(debug_img_bgr, (ros_origin_px_x, ros_origin_px_y), 
                        (255, 0, 0), cv2.MARKER_CROSS, 20, 1)
            
            # Length of axis in pixels (e.g., 1 meter)
            axis_len_px = int(1.0 / resolution)

            # Draw ROS X Axis (Blue) -> Points Right
            cv2.arrowedLine(debug_img_bgr, 
                            (ros_origin_px_x, ros_origin_px_y), 
                            (ros_origin_px_x + axis_len_px, ros_origin_px_y), 
                            (0, 0, 255), 2, tipLength=0.3)
            
            # Draw ROS Y Axis (Magenta) -> Points UP (which is decreasing row index in CV)
            cv2.arrowedLine(debug_img_bgr, 
                            (ros_origin_px_x, ros_origin_px_y), 
                            (ros_origin_px_x, ros_origin_px_y - axis_len_px), 
                            (0, 255, 0), 2, tipLength=0.3)

            cv2.putText(debug_img_bgr, "World", 
                        (ros_origin_px_x + 10, ros_origin_px_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Draw Map X Axis (Blue) -> Points Right
            cv2.arrowedLine(debug_img_bgr, 
                            (map_origin_px_x, map_origin_px_y), 
                            (map_origin_px_x + axis_len_px, map_origin_px_y), 
                            (0, 0, 255), 2, tipLength=0.3)
            
            # Draw Map Y Axis (Magenta) -> Points UP (which is decreasing row index in CV)
            cv2.arrowedLine(debug_img_bgr, 
                            (map_origin_px_x, map_origin_px_y), 
                            (map_origin_px_x, map_origin_px_y - axis_len_px), 
                            (0, 255, 0), 2, tipLength=0.3)

            cv2.putText(debug_img_bgr, "Map", 
                        (map_origin_px_x + 10, map_origin_px_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Draw pgm X Axis (Blue) -> Points Right
            cv2.arrowedLine(debug_img_bgr, 
                            (pgm_origin_px_x, pgm_origin_px_y), 
                            (pgm_origin_px_x + axis_len_px, pgm_origin_px_y), 
                            (0, 0, 255), 2, tipLength=0.3)
            
            # Draw pgm Y Axis (Magenta) -> Points UP (which is decreasing row index in CV)
            cv2.arrowedLine(debug_img_bgr, 
                            (pgm_origin_px_x, pgm_origin_px_y), 
                            (pgm_origin_px_x, pgm_origin_px_y - axis_len_px), 
                            (0, 255, 0), 2, tipLength=0.3)

            cv2.putText(debug_img_bgr, "pgm", 
                        (pgm_origin_px_x + 10, pgm_origin_px_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Save Debug Image
            debug_image_path = os.path.join(temp_dir_path, 'exploration_map.png')
            cv2.imwrite(debug_image_path, debug_img_bgr)
            
            print(f"Debug map saved to: {debug_image_path}")
            # --- DEBUG SECTION END ---

            return mx, my
        except Exception as e:
            print(f"Map Error: {e}")
            return [], []
                    

##################################
# Main Function
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
