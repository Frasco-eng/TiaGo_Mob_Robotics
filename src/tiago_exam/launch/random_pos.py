#!/usr/bin/env python3

import yaml
import numpy as np
from PIL import Image
import os
import random
import math


def map_pose_random_generator(map_path, safe_coords=[0.0, -1.3, 0.0, 0.0]):
        
    try: 
        # Read Yaml file
        with open(map_path, 'r') as f:
            map_data = yaml.safe_load(f)
            
        resolution = map_data['resolution']
        origin = map_data['origin'] # [x, y, yaw] solitamente
        
        # Find PGM file path
        pgm_filename = map_data['image']
        pgm_path = os.path.join(os.path.dirname(map_path), pgm_filename)
        
        if not os.path.exists(pgm_path):
            raise FileNotFoundError(f"Map Image not found: {pgm_path}")
            
        # Loads PGM image and convert to numpy array
        img = Image.open(pgm_path)
        grid = np.array(img)
        height, width = grid.shape
        
        #pre calculates free pixels
        free_pixels = np.column_stack(np.where(grid >= 250))
        
        print(f"Map downloaded. Dimensions: {width}x{height}. Free pixels: {len(free_pixels)}")

        if len(free_pixels) == 0:
            return None

        
        # Tries a pixel random from free pixels
        choice_idx = random.randint(0, len(free_pixels) - 1)
        pixel_r, pixel_c = free_pixels[choice_idx]
        
        # From Pixel to World coordinates
    
        # Image coordinates:
        # px = pixel_c
        # py = (self.height - 1 - pixel_r) 
        
        # World coordinates:
        # x = origin_x + (col * res)
        # y = origin_y + ((height - 1 - row) * res)
        
        world_x = origin[0] + (pixel_c * resolution)
        world_y = origin[1] + ((height - 1 - pixel_r) * resolution)
        
        # Random jittering within the cell
        world_x += random.uniform(0, resolution)
        world_y += random.uniform(0, resolution)
        
        # 3. Orientamento Random
        world_theta = random.uniform(-math.pi, math.pi)
        
        coords = [world_x, world_y, 0.0, world_theta]
        return coords
    
    except Exception as e:
        
        return safe_coords
            


