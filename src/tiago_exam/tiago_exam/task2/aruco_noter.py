#!/usr/bin/env python3

##################################
# Imports
##################################

# ROS2 Libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import PyKDL as kdl
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


from exam_interfaces.msg import MoveHead

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
        # UPDATE THESE to match your robot's exact joint names in the URDF
        self.head_pan_joint_name = "head_1_joint"
        self.head_tilt_joint_name = "head_2_joint"

        ##################################
        # State Variables
        ##################################
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.joints_received = False

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
        
        # Subscribe to joint states to know current head position
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10, callback_group=self.callback_group)

        
        ##################################
        # Publishers
        ##################################
        
        self.nav_goal_pub = self.create_publisher(PoseStamped, 'move_to_marker', 10, callback_group=self.callback_group)
        self.real_marker_pub = self.create_publisher(PoseStamped, 'detected_marker_real_pose', 10, callback_group=self.callback_group)
        
        self.move_head_pub = self.create_publisher(MoveHead, 'move_head', 10, callback_group=self.callback_group)
        ##################################
        # Service Client
        ##################################
        

        self.get_logger().info("Aruco Noter Node Started.")

    ##################################
    # Callbacks
    ##################################
    def joint_state_callback(self, msg: JointState):
        """
        Updates the internal state of the head joints.
        """
        try:
            if self.head_pan_joint_name in msg.name:
                index = msg.name.index(self.head_pan_joint_name)
                self.current_pan = msg.position[index]
            
            if self.head_tilt_joint_name in msg.name:
                index = msg.name.index(self.head_tilt_joint_name)
                self.current_tilt = msg.position[index]

            self.joints_received = True
        except ValueError:
            pass

    def coord_callback(self, msg: TransformStamped, type: str = None) -> None:
        """
        Processes detected ArUco marker, calculates navigation goals and controls the head.
        """
        if not self.joints_received:
            self.get_logger().warn("Waiting for joint states before tracking...")
            # We don't return here because we still want to publish nav goals, 
            # but head tracking might be inaccurate initially (assumes 0.0)
        
        # Determine marker frame name based on type
        if type == "place":
            marker_frame_name = "aruco_place_marker_frame"
        elif type == "pick":
            marker_frame_name = "aruco_pick_marker_frame"
        else:
            return
       
       # Obtain necessary transforms
        try:
            base_T_camera_ts = self.tf_buffer.lookup_transform(self.base_link_frame_name, self.camera_frame_name, rclpy.time.Time())
            map_T_base_ts = self.tf_buffer.lookup_transform(self.map_frame_name, self.base_link_frame_name, rclpy.time.Time())
        except TransformException:
            return

        # --- Real Marker Position Calculation ---
        camera_T_marker_kdl = self.transform_stamped_to_kdl(msg)
        base_T_camera_kdl = self.transform_stamped_to_kdl(base_T_camera_ts)
        map_T_base_kdl = self.transform_stamped_to_kdl(map_T_base_ts)

        # Transform Chain: Map -> Base -> Camera -> Marker
        map_T_marker_kdl = map_T_base_kdl * base_T_camera_kdl * camera_T_marker_kdl
        
        real_pose_msg = self.kdl_to_pose_msg(map_T_marker_kdl, self.map_frame_name)
        real_pose_msg.header.frame_id = marker_frame_name 
        self.real_marker_pub.publish(real_pose_msg)

        # --- Approach Position Calculation ---
        target_offset = kdl.Vector(0.0, 0.0, 0.5) 
        marker_T_target_kdl = kdl.Frame(kdl.Rotation.Identity(), target_offset)
        map_T_target_kdl = map_T_marker_kdl * marker_T_target_kdl

        # --- Orientation Correction ---
        dx = map_T_marker_kdl.p.x() - map_T_target_kdl.p.x()
        dy = map_T_marker_kdl.p.y() - map_T_target_kdl.p.y()
        yaw_to_face = math.atan2(dy, dx)
        map_T_target_kdl.M = kdl.Rotation.RotZ(yaw_to_face)

        nav_goal_msg = self.kdl_to_pose_msg(map_T_target_kdl, self.map_frame_name)
        nav_goal_msg.header.frame_id = marker_frame_name
        self.nav_goal_pub.publish(nav_goal_msg)

        # ---------------------------------------------------------
        # --- Head Tracking Logic (Relative to Current Position) ---
        # ---------------------------------------------------------
        
        # 1. Get RAW coordinates in Optical Frame
        # Standard: X=Right, Y=Down, Z=Forward
        raw_x = camera_T_marker_kdl.p.x()
        raw_y = camera_T_marker_kdl.p.y()
        raw_z = camera_T_marker_kdl.p.z()

        # 2. Calculate Angular Error (Deltas)
        # Pan Delta: Angle on the X-Z plane
        # If X is positive (right), we need to turn Right.
        delta_pan = math.atan2(raw_x, raw_z)

        # Tilt Delta: Angle on the Y-Z plane (using ground distance)
        d_ground = math.sqrt(raw_x**2 + raw_z**2)
        delta_tilt = math.atan2(raw_y, d_ground)

        self.get_logger().info(f"NOTER: Raw Coords (x: {raw_x:.2f}, y: {raw_y:.2f}, z: {raw_z:.2f})")
        self.get_logger().info(f"NOTER: Delta Pan: {math.degrees(delta_pan):.2f}, Delta Tilt: {math.degrees(delta_tilt):.2f}")

        # 3. Apply to Current Joint Positions
        # Note on Signs:
        # Pan: +Left / -Right. Since 'raw_x' is +Right, we ADD the delta.
        cmd_pan = self.current_pan - delta_pan
        
        # Tilt: +Down / -Up. Since 'raw_y' is +Down, we ADD the delta.
        cmd_tilt = self.current_tilt - delta_tilt

        # 4. Publish Head Movement Command
        move_head_msg = MoveHead()
        move_head_msg.pan = cmd_pan
        move_head_msg.tilt = cmd_tilt
        self.move_head_pub.publish(move_head_msg)
        
    #################################
    # Helper Functions
    ##################################

    def transform_stamped_to_kdl(self, ts: TransformStamped) -> kdl.Frame:
        t = ts.transform.translation
        q = ts.transform.rotation
        return kdl.Frame(kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w), kdl.Vector(t.x, t.y, t.z))
    
    def kdl_to_pose_msg(self, frame: kdl.Frame, frame_id: str) -> PoseStamped:
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = frame_id
        p.pose.position.x = frame.p.x()
        p.pose.position.y = frame.p.y()
        p.pose.position.z = frame.p.z()
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

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()