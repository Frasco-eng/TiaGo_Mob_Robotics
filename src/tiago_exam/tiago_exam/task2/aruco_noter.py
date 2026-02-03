#!/usr/bin/env python3

##################################
# Imports
##################################
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Custom service
from tiago_exam_interfaces.srv import SaveMarker

# Python Libraries
import os
import csv
import datetime
from tiago_exam import utils

##################################
# ArucoNoter Class
##################################
class ArucoNoter(Node):

    def __init__(self):
        super().__init__('aruco_noter')

        ##################################
        # Callback Group
        ##################################
        self.callback_group = ReentrantCallbackGroup()

        ##################################
        # Class Parameters
        ##################################
        ws_src_dir = utils.get_package_src_directory('tiago_exam')
        data_path_dir = os.path.join(ws_src_dir, 'data')
        self.declare_parameter('marker_file_path', os.path.join(data_path_dir, 'marker_locations.csv'))
        self.marker_file_path = self.get_parameter('marker_file_path').get_parameter_value().string_value

        ##################################
        # State Variables
        ##################################
        self.logged_markers = set()

        ##################################
        # Initialization
        ##################################
        self._init_marker_file()
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        ##################################
        # Service Server
        ##################################
        self.save_marker_srv = self.create_service(SaveMarker, 'save_marker_srv', self.save_marker_callback, callback_group=self.callback_group)
        
        self.get_logger().info("Aruco Noter Service Ready. Waiting for requests...")

    ##################################
    # Callback
    ##################################
    def save_marker_callback(self, request, response) -> SaveMarker.Response:
        """
        Callback function to save marker data to CSV and broadcast static TF.

        Parameters
        ----------
        request : SaveMarker.Request
            The service request containing marker name and pose.
        response : SaveMarker.Response
            The service response indicating success or failure.
        """
        try:
            # Get current timestamp
            now = datetime.datetime.now()
            timestamp_str = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            # Write to CSV in append mode
            with open(self.marker_file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                
                # Extract pose data
                p = request.pose.position
                q = request.pose.orientation

                # Write row
                writer.writerow([
                    timestamp_str,
                    request.name,
                    f"{p.x:.4f}", f"{p.y:.4f}", f"{p.z:.4f}",
                    f"{q.x:.4f}", f"{q.y:.4f}", f"{q.z:.4f}", f"{q.w:.4f}"
                ])

            # Update logged markers
            self.logged_markers.add(request.name)
            self.get_logger().info(f"Saved: {request.name}")
            
            # Broadcast static TF
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = request.name 
            
            t.transform.translation.x = p.x
            t.transform.translation.y = p.y
            t.transform.translation.z = p.z
            t.transform.rotation = q
            
            self.tf_static_broadcaster.sendTransform(t)

            self.get_logger().info(f"Broadcasted static TF for: {request.name}")

            response.success = True

        except Exception as e:
            self.get_logger().error(f"Failed to write CSV: {e}")
            response.success = False

        return response
    
    ##################################
    # Helper Method
    ##################################
    def _init_marker_file(self):
        """
        Initializes the marker CSV file with headers.
        """
        # Ensure the directory exists
        output_folder = os.path.dirname(os.path.abspath(self.marker_file_path))
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
            self.get_logger().info(f"Folder created: {output_folder}")
            
        try:
            with open(self.marker_file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "name", "x", "y", "z", "qx", "qy", "qz", "qw"])
            self.get_logger().info(f"File CSV initialized at: {self.marker_file_path}")
        except Exception as e:
            self.get_logger().error(f"Error initializing file: {e}")
    

##################################
# Main Function
##################################
def main(args=None):
    rclpy.init(args=args)
    node = ArucoNoter()

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