#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from control_msgs.action import PointHead
from rclpy.action import ActionClient
from std_msgs.msg import Bool

class SearchManager(Node):
    def __init__(self):
        super().__init__('search_manager')
        
        # Variables
        self.pick_pose = None
        self.place_pose = None
        self.localized = False

        # Subscribers
        self.create_subscription(Bool, '/localization_complete', self.loc_done_cb, 10)
        self.create_subscription(PoseStamped, '/aruco_pick/pose', self.pick_cb, 10)
        self.create_subscription(PoseStamped, '/aruco_place/pose', self.place_cb, 10)

        # Navigator and Head Action Client
        self.nav = BasicNavigator()
        self.head_client = ActionClient(self, PointHead, '/head_controller/point_head_action')
        
        # Waypoints for searching
        self.search_waypoints = [
            [1.5, 0.0, 0.0],
            [3.0, 1.0, 1.57],
            [0.5, -2.0, -1.57]
        ]

    def pick_cb(self, msg):
        if self.pick_pose is None:
            self.get_logger().info("!!! Marker PICK localized !!!")
            self.pick_pose = msg

    def place_cb(self, msg):
        if self.place_pose is None:
            self.get_logger().info("!!! Marker PLACE localized !!!")
            self.place_pose = msg

    def look_around(self, side=0.0):
        if not self.head_client.wait_for_server(timeout_sec=1.0):
            return
        
        goal = PointHead.Goal()
        goal.target.header.frame_id = "base_link"
        goal.target.point.x = 2.0
        goal.target.point.y = side 
        goal.target.point.z = 0.8
        goal.pointing_frame = "head_front_camera_rgb_optical_frame"
        self.head_client.send_goal_async(goal)

    def loc_done_cb(self, msg):
        if msg.data and not self.localized:
            self.get_logger().info("Localization completed. Starting search...")
            self.localized = True

    def search_process(self):
        self.get_logger().info("Waiting for localization to complete...")
        while rclpy.ok() and not self.localized:
            rclpy.spin_once(self, timeout_sec=0.5)

        self.nav.waitUntilNav2Active()

        for wp in self.search_waypoints:
            if self.pick_pose and self.place_pose:
                self.get_logger().info("Both markers found. Ending search.")
                return

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = wp[0]
            goal_pose.pose.position.y = wp[1]
            goal_pose.pose.position.z = 0.0
            # Simple conversion yaw -> quaternion
            goal_pose.pose.orientation.z = math.sin(wp[2] / 2.0)
            goal_pose.pose.orientation.w = math.cos(wp[2] / 2.0)

            self.get_logger().info(f"Navigating to waypoint: {wp}")
            self.nav.goToPose(goal_pose)

            self.get_logger().info("Looking around for markers...")
            self.look_around()
            rclpy.spin_once(self, timeout_sec=2.0)  # Allow some time for head movement

        self.get_logger().info("Search completed.")

def main(args=None):
    rclpy.init(args=args)

    node = SearchManager()

    node.search_process()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()