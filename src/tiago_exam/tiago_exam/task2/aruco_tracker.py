#!/usr/bin/env python3

##################################
# Imports
##################################
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
import PyKDL as kdl
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Custom service
from exam_interfaces.msg import HeadPose

# Python Standard Libraries
import math

##################################
# Aruco Noter Node
##################################
class ArucoNoter(Node):

    def __init__(self):
        super().__init__('aruco_noter')

        ##################################
        # Callback Group
        ##################################
        self.callback_group = ReentrantCallbackGroup()
        
        ##################################
        # Configuration
        ##################################
        self.head_pan_joint_name = "head_1_joint"
        self.head_tilt_joint_name = "head_2_joint"
        self.approach_distance = 0.4 # Meters in front of the marker

        ##################################
        # State Variables
        ##################################
        self.current_pan = 0.0          # Current head pan position
        self.current_tilt = 0.0         # Current head tilt position
        self.joints_received = False    # Flag to check if joint states have been received

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
        self.place_sub = self.create_subscription(TransformStamped, "aruco_place/transform", lambda msg: self.coord_callback(msg, "place"), callback_group=self.callback_group, qos_profile=1)
        self.pick_sub = self.create_subscription(TransformStamped, "aruco_pick/transform", lambda msg: self.coord_callback(msg, "pick"), callback_group=self.callback_group, qos_profile=1)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10, callback_group=self.callback_group)

        ##################################
        # Publishers
        ##################################
        self.nav_goal_pub = self.create_publisher(PoseStamped, 'move_to_marker', 10, callback_group=self.callback_group)
        self.real_marker_pub = self.create_publisher(PoseStamped, 'detected_marker_real_pose', 10, callback_group=self.callback_group)
        self.move_head_pub = self.create_publisher(HeadPose, 'head_pose', 10, callback_group=self.callback_group)

        self.get_logger().info("Aruco Noter Node Started.")

    ##################################
    # Callbacks
    ##################################
    def joint_state_callback(self, msg: JointState) -> None:
        """
        Updates the internal state of the head joints.

        Parameters
        ----------
        msg : JointState
            The current joint states message.
        """
        try:
            # Update current pan position
            if self.head_pan_joint_name in msg.name:
                index = msg.name.index(self.head_pan_joint_name)
                self.current_pan = msg.position[index]
            
            # Update current tilt position
            if self.head_tilt_joint_name in msg.name:
                index = msg.name.index(self.head_tilt_joint_name)
                self.current_tilt = msg.position[index]

            self.joints_received = True
        except ValueError:
            pass

    def coord_callback(self, msg: TransformStamped, type: str = None) -> None:
        """
        Processes detected ArUco marker, calculates navigation goals and controls the head.

        Parameters
        ----------
        msg : TransformStamped
            The transform of the detected ArUco marker.
        type : str, optional
            The type of marker ("place" or "pick"), by default None.
        """
        if not self.joints_received:
            self.get_logger().warn("Waiting for joint states before tracking...")
        
        #############
        # Determine Marker frame name
        #############
        if type == "place":
            marker_frame_name = "aruco_place_marker_frame"
        elif type == "pick":
            marker_frame_name = "aruco_pick_marker_frame"
        else:
            return
       
        #############
        # Obtain Necessary Transformations
        #############
        try:
            base_T_camera_ts = self.tf_buffer.lookup_transform(self.base_link_frame_name, self.camera_frame_name, rclpy.time.Time())
            map_T_base_ts = self.tf_buffer.lookup_transform(self.map_frame_name, self.base_link_frame_name, rclpy.time.Time())
        except TransformException:
            return

        #############
        # Real Marker Pose Computation
        #############
        camera_T_marker_kdl = self.transform_stamped_to_kdl(msg)
        base_T_camera_kdl = self.transform_stamped_to_kdl(base_T_camera_ts)
        map_T_base_kdl = self.transform_stamped_to_kdl(map_T_base_ts)

        # Transform Chain: Map -> Base -> Camera -> Marker
        map_T_marker_kdl = map_T_base_kdl * base_T_camera_kdl * camera_T_marker_kdl
        
        real_pose_msg = self.kdl_to_pose_msg(map_T_marker_kdl, self.map_frame_name)
        real_pose_msg.header.frame_id = marker_frame_name 
        self.real_marker_pub.publish(real_pose_msg)

        #############
        # Approach Target Pose Computation
        #############
        target_offset = kdl.Vector(0.0, 0.0, self.approach_distance) 
        marker_T_target_kdl = kdl.Frame(kdl.Rotation.Identity(), target_offset)
        map_T_target_kdl = map_T_marker_kdl * marker_T_target_kdl

        # Orientation Correction
        dx = map_T_marker_kdl.p.x() - map_T_target_kdl.p.x()
        dy = map_T_marker_kdl.p.y() - map_T_target_kdl.p.y()
        yaw_to_face = math.atan2(dy, dx)
        map_T_target_kdl.M = kdl.Rotation.RotZ(yaw_to_face)

        # Publish Navigation Goal
        nav_goal_msg = self.kdl_to_pose_msg(map_T_target_kdl, self.map_frame_name)
        nav_goal_msg.header.frame_id = marker_frame_name
        self.nav_goal_pub.publish(nav_goal_msg)

        # ---------------------------------------------------------
        # --- Head Tracking Logic (Relative to Current Position) ---
        # ---------------------------------------------------------
        
        #############
        # Get RAW coordinates in Optical Frame
        #############
        # Standard: X=Right, Y=Down, Z=Forward
        raw_x = camera_T_marker_kdl.p.x()
        raw_y = camera_T_marker_kdl.p.y()
        raw_z = camera_T_marker_kdl.p.z()

        #############
        # Calculate Angular Error (Deltas)
        #############
        # Pan Delta: Angle on the X-Z plane
        # If X is positive (right), we need to turn Right.
        delta_pan = math.atan2(raw_x, raw_z)

        # Tilt Delta: Angle on the Y-Z plane (using ground distance)
        d_ground = math.sqrt(raw_x**2 + raw_z**2)
        delta_tilt = math.atan2(raw_y, d_ground)

        #############
        # Apply to Current Joint Positions
        #############
        # NOTE on Signs:
        # Pan: +Left / -Right. Since 'raw_x' is +Right, we ADD the delta.
        cmd_pan = self.current_pan - delta_pan
        
        # Tilt: +Down / -Up. Since 'raw_y' is +Down, we ADD the delta.
        cmd_tilt = self.current_tilt - delta_tilt

        #############
        # Publish Head Movement Command
        ##############
        self.send_head_pose(cmd_pan, cmd_tilt)
        
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
        kdl.Frame
            The corresponding KDL Frame.
        """
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
            The frame ID for the PoseStamped message.

        Returns
        -------
        PoseStamped
            The corresponding PoseStamped message.
        """
        # Create PoseStamped message
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()

        # Assign frame ID
        p.header.frame_id = frame_id

        # Assign position
        p.pose.position.x = frame.p.x()
        p.pose.position.y = frame.p.y()
        p.pose.position.z = frame.p.z()

        # Assign orientation
        qx, qy, qz, qw = frame.M.GetQuaternion()
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw

        return p
    
    def send_head_pose(self, pan: float, tilt: float) -> None:
        """
        Calls the MoveHead service to move the robot's head.

        Parameters
        ----------
        pan : float
            The desired pan angle in radians.
        tilt : float
            The desired tilt angle in radians.
        """
        # Create and publish HeadPose message
        head_pose_msg = HeadPose()
        head_pose_msg.pan = pan
        head_pose_msg.tilt = tilt
        self.move_head_pub.publish(head_pose_msg)


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