#!/usr/bin/env python3

##################################
# Imports
##################################

# ROS2 Libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
import PyKDL as kdl
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException

# Python Standard Libraries
import math

##################################
# Aruco Noter Node
##################################
class ArucoNoter(Node):
    def __init__(self):
        super().__init__('aruco_noter')
        
        ##################################
        # TF Setup
        ##################################
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ###################################
        # Frame Names
        ##################################
        self.map_frame_name = "map"
        self.base_link_frame_name = "base_link"
        self.camera_frame_name = "head_front_camera_rgb_optical_frame"
        
        ##################################
        # Subscribers
        ##################################
        self.place_sub = self.create_subscription(TransformStamped, "aruco_place/transform", lambda msg: self.coord_callback(msg, "place"), 1)
        self.pick_sub = self.create_subscription(TransformStamped, "aruco_pick/transform", lambda msg: self.coord_callback(msg, "pick"), 1)

        ##################################
        # Publishers
        ##################################
        self.nav_goal_pub = self.create_publisher(PoseStamped, 'move_to_marker', 10)
        self.real_marker_pub = self.create_publisher(PoseStamped, 'detected_marker_real_pose', 10)

        self.get_logger().info("Aruco Noter Node Started.")

    ##################################
    # Callbacks
    ##################################
    def coord_callback(self, msg: TransformStamped, type: str = None) -> None:
        """
        This callback processes the detected ArUco marker transform, computes the real-world position of the marker,
        and calculates an approach position 1 meter away from the marker, facing it. It then publishes both the real marker position
        and the navigation goal for the robot.

        Parameters
        ----------
        msg : TransformStamped
            The transform of the detected ArUco marker relative to the camera frame.
        type : str
            The type of marker detected ("place" or "pick").
        """
        # Determine marker frame name based on type
        if type == "place":
            marker_frame_name = "aruco_place_marker_frame"
        elif type == "pick":
            marker_frame_name = "aruco_pick_marker_frame"
        else:
            return
       
       # Obtain necessary transforms
        try:
            # Camera -> Base (TransformStamped)
            base_T_camera_ts = self.tf_buffer.lookup_transform(self.base_link_frame_name, self.camera_frame_name, rclpy.time.Time())
            # Base -> Map (TransformStamped)
            map_T_base_ts = self.tf_buffer.lookup_transform(self.map_frame_name, self.base_link_frame_name, rclpy.time.Time())
        except TransformException:
            return

        # --- Real Marker Position Calculation ---
        camera_T_marker_kdl = self.transform_stamped_to_kdl(msg)
        base_T_camera_kdl = self.transform_stamped_to_kdl(base_T_camera_ts)
        map_T_base_kdl = self.transform_stamped_to_kdl(map_T_base_ts)

        # Transform Chain: Map -> Base -> Camera -> Marker
        map_T_marker_kdl = map_T_base_kdl * base_T_camera_kdl * camera_T_marker_kdl
        
        # Publish the real marker position
        real_pose_msg = self.kdl_to_pose_msg(map_T_marker_kdl, self.map_frame_name)
        real_pose_msg.header.frame_id = marker_frame_name # NOTE: this frame_id should be used by the explorer to identify the marker. The explorer knows it and after it will remap it to "map"
        self.real_marker_pub.publish(real_pose_msg)

        # --- Approach Position Calculation (0.5m behind) ---
        target_offset = kdl.Vector(0.0, 0.0, 0.5) # 0.5 meters ahead
        
        # Target position in Marker frame
        marker_T_target_kdl = kdl.Frame(kdl.Rotation.Identity(), target_offset)
        
        # Target position in Map frame
        map_T_target_kdl = map_T_marker_kdl * marker_T_target_kdl

        # --- Orientation Correction ("FACE THE MARKER") ---
        # Orientation of the robot to face the marker
        dx = map_T_marker_kdl.p.x() - map_T_target_kdl.p.x()
        dy = map_T_marker_kdl.p.y() - map_T_target_kdl.p.y()
        
        # The angle (Yaw) the robot must have to look at the marker
        yaw_to_face = math.atan2(dy, dx)
        
        # Create a pure rotation based on this Yaw
        rotation_to_face = kdl.Rotation.RotZ(yaw_to_face)
        
        # Assign this rotation to the final target
        map_T_target_kdl.M = rotation_to_face

        # Publish the goal for NAV2
        nav_goal_msg = self.kdl_to_pose_msg(map_T_target_kdl, self.map_frame_name)
        nav_goal_msg.header.frame_id = marker_frame_name
        self.nav_goal_pub.publish(nav_goal_msg)

    
    ##################################
    # Helper Functions
    ##################################

    def transform_stamped_to_kdl(self, ts: TransformStamped) -> kdl.Frame:
        """
        Converts a TransformStamped message to a KDL Frame.
        
        Parameters
        ----------
        ts : TransformStamped
            The TransformStamped message to convert.
            
        Returns
        -------
        frame_kdl : kdl.Frame
            The corresponding KDL Frame."""
        # Extract translation
        t = ts.transform.translation

        # Extract rotation
        q = ts.transform.rotation

        return kdl.Frame(kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w), kdl.Vector(t.x, t.y, t.z))
    
    def kdl_to_pose_msg(self, frame: kdl.Frame, frame_id: str) -> PoseStamped:
        """
        Converts a KDL Frame to a PoseStamped message.

        Parameters
        ----------
        frame : kdl.Frame
            The KDL Frame to convert.
        frame_id : str
            The frame ID to set in the PoseStamped message.

        Returns
        -------
        pose_msg : PoseStamped
            The corresponding PoseStamped message.
        """
        # Initialize PoseStamped message
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = frame_id

        # Set position
        p.pose.position.x = frame.p.x()
        p.pose.position.y = frame.p.y()
        p.pose.position.z = frame.p.z()

        # Set orientation
        qx, qy, qz, qw = frame.M.GetQuaternion()
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw

        return p


##################################
# Main Function
##################################
def main(args=None):
    rclpy.init(args=args)
    node = ArucoNoter()

    try:
        rclpy.spin(node)
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