#!/usr/bin/env python3

##################################
# Imports
##################################
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav2_msgs.srv import SaveMap 
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

##################################
# MapSaverWatcher Class
##################################
class MapSaverWatcher(Node):

    def __init__(self):
        super().__init__('map_saver_watcher')

        ##################################
        # Callback Group
        ##################################
        self.callback_group = ReentrantCallbackGroup()

        ##################################
        # Class Parameters
        ##################################
        self.declare_parameter('map_full_path', 'my_map')
        self.map_path = self.get_parameter('map_full_path').get_parameter_value().string_value 
        
        ##################################
        # Service Client
        ##################################
        self.map_save_client = self.create_client(
            SaveMap, 
            '/map_saver/save_map', 
            callback_group=self.callback_group
        )
        
        ##################################
        # Service Server
        ##################################
        self.srv = self.create_service(
            Trigger, 
            'save_map', 
            self.save_map_callback, 
            callback_group=self.callback_group
        )
    
        self.get_logger().info(f'Map saver watcher started. Target path: {self.map_path}')

    ##################################
    # Callback
    ##################################
    def save_map_callback(self, request, response):
        self.get_logger().info('Received request to save map. Contacting map_saver_server...')

        # Check if the map saver service is available
        if not self.map_save_client.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = "map_saver_server not available! Is it running?"
            self.get_logger().error(response.message)
            return response

        # Prepare the request
        save_req = SaveMap.Request()
        save_req.map_url = self.map_path
        save_req.image_format = "pgm"
        save_req.map_mode = "trinary"
        save_req.free_thresh = 0.25
        save_req.occupied_thresh = 0.65
        save_req.map_topic = "map" 

        # Call the service synchronously 
        try:
            # Call the service and wait
            result = self.map_save_client.call(save_req)
            
            # Look at the result
            if result:
                response.success = True
                response.message = f"Map successfully saved to {self.map_path}"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = "map_saver_server returned Failure."
                self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Service call failed: {str(e)}"
            self.get_logger().error(response.message)
            
        return response


##################################
# Main Function
##################################
def main(args=None):
    rclpy.init(args=args)
    node = MapSaverWatcher()

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