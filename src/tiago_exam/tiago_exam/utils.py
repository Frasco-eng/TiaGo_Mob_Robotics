import os
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import random
import time
import yaml
import math

##################################
# Utility Functions
##################################
def get_package_src_directory(package_name):
    """
    Returns the source directory path of the given package within a standard ROS2 workspace.

    Parameters
    ----------
    package_name : str
        Name of the ROS2 package.
    """
    install_path = get_package_share_directory(package_name)
    
    # install_path: .../workspace/install/nome_pkg/share/pkg_name
    workspace_path = install_path
    for _ in range(4):
        workspace_path = os.path.dirname(workspace_path)
    
    # Build the src path
    src_path = os.path.join(workspace_path, 'src', package_name)
    
    # Safety check
    if not os.path.exists(src_path):
        print(f"[WARN] Cannot find {src_path}. Are you using a standard workspace?")
        return None
        
    return src_path

def map_pose_random_generator(map_path, safe_coords=[0.0, -1.3, 0.0, 0.0], clearance=1.0, task_name: str = "") -> list[float]:
    """
    Generates a random pose (x, y, theta) on the map defined by map_path,
    ensuring the pose is in free space with a specified clearance from obstacles.
    
    Parameters
    ----------
    map_path : str
        Path to the YAML map file.
    safe_coords : list of float
        Default safe coordinates to return in case of failure [x, y, z, theta].
    clearance : float
        Minimum distance from obstacles in meters.
    task_name : str
        Optional name of the task for logging purposes.
        
    Returns
    -------
    list of float
        Random pose [x, y, z, theta] in world coordinates.
    """
    # Seed random seed based on time
    random.seed(time.time())
    
    print(f"Generating random pose from map: {map_path} with clearance {clearance}m for task: {task_name}")

    try: 
        # Load map YAML
        with open(map_path, 'r') as f:
            map_data = yaml.safe_load(f)
        
        # Extract map parameters
        resolution = map_data['resolution']
        origin = map_data['origin']

        # Load the map image
        pgm_path = os.path.join(os.path.dirname(map_path), map_data['image'])
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Map Image not found: {pgm_path}")


        # Threshold the image to binary (Free: 255, Occupied: 0)
        _, binary_map = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)

        # Compute distance transform
        dist_map = cv2.distanceTransform(binary_map, cv2.DIST_L2, 5)

        # Convert clearance meters to pixels
        effective_clearance_px = (clearance + 0.1) / resolution

        # Filter pixels that satisfy the clearance
        safe_indices = np.where(dist_map >= effective_clearance_px)
        whitelist_pixels = list(zip(safe_indices[0], safe_indices[1]))

        # Create temp directory for debug images
        temp_dir_path = os.path.join(get_package_src_directory('tiago_exam'), 'temp')
        if not os.path.exists(temp_dir_path):
            os.makedirs(temp_dir_path)
        
        # If no safe pixels found, relax clearance requirement
        if not whitelist_pixels:
            print("No safe pixels found! Reducing clearance requirements...")
            # Fallback: find the max distance available if requested clearance is too high
            max_dist = np.max(dist_map)
            safe_indices = np.where(dist_map >= max_dist * 0.9)
            whitelist_pixels = list(zip(safe_indices[0], safe_indices[1]))

        # Pick and Convert
        pixel_r, pixel_c = random.choice(whitelist_pixels)
        height, width = img.shape
        
        # Standard ROS Map Transform
        world_x = origin[0] + (pixel_c * resolution) 
        world_y = origin[1] + ((height - pixel_r) * resolution) + safe_coords[1] # Using height-pixel_r
        
        world_theta = random.uniform(-math.pi, math.pi) 
        
        # --- DEBUG SECTION START ---
        # Create a 3-channel BGR image for colorful markers
        debug_img_bgr = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)

        # A. Highlight Safe Zone (Light Green)
        for r, c in whitelist_pixels:
            debug_img_bgr[r, c] = [200, 255, 200] 

        # B. Draw OpenCV Origin & Axes (Top-Left)
        # CV Origin is (0,0). X is Col (Right), Y is Row (Down)
        cv2.circle(debug_img_bgr, (5, 5), 5, (0, 0, 255), -1) 
        cv2.arrowedLine(debug_img_bgr, (5, 5), (55, 5), (0, 0, 255), 2, tipLength=0.3)  # Red X
        cv2.arrowedLine(debug_img_bgr, (5, 5), (5, 55), (0, 255, 0), 2, tipLength=0.3)  # Green Y
        cv2.putText(debug_img_bgr, "CV Origin", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # C. Draw ROS World Origin & Axes
        ros_origin_px_x = int((0.0 - origin[0]) / resolution)
        ros_origin_px_y = int(height - ((0.0 - origin[1] - safe_coords[1]) / resolution))

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
        
        # D. Mark Selected Spawn Point (Yellow)
        cv2.drawMarker(debug_img_bgr, (pixel_c, pixel_r), 
                    (0, 0, 0), cv2.MARKER_STAR, 20, 1)
        
        cv2.putText(debug_img_bgr, "Spawn", 
                    (pixel_c + 10, pixel_r - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                
        # Save Debug Spawn Map Image
        debug_image_path = os.path.join(temp_dir_path, f'spawn_map_with_frames_{task_name}.png')
        cv2.imwrite(debug_image_path, debug_img_bgr)
        
        print(f"Debug spawn map saved to: {debug_image_path}")
        # --- DEBUG SECTION END ---

        return [world_x, world_y, 0.0, world_theta]
    
    except Exception as e:
        print(f"Error generating random pose: {e}")
        return safe_coords