#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger # Service type for triggering map save
import subprocess


class MapSaverWatcher(Node):
    def __init__(self):
        super().__init__('map_saver_watcher')

        # Parameters
        self.declare_parameter('map_full_path', 'my_map')
        self.map_path = self.get_parameter('map_full_path').get_parameter_value().string_value
        
        # Create the Service
        self.srv = self.create_service(Trigger, 'save_map', self.save_map_callback)
    
        self.get_logger().info(f'Map saver watcher started. Will save map to: {self.map_path}')

    def save_map_callback(self, request, response):
        self.get_logger().info('Received request to save map.')
        
        # Command to save the map
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', self.map_path,
            '--free', '0.25',  # Try --free or -free
            '--occ', '0.65',   # Try --occ or -occ
            '--fmt', 'pgm'
        ]
        
        try:
            # Execute the command
            subprocess.run(cmd, check=True)
            response.success = True
            response.message = f"Map saved successfully at {self.map_path}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Error during saving: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MapSaverWatcher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()