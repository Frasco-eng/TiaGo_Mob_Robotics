import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import PyKDL as kdl # work with frames always with pykdl
import numpy as np

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FramePublisher(Node):

    def __init__(self):
        super().__init__('frame_publisher')

        ############## TF BROADCASTER ##############
        self.tf_broadcaster = TransformBroadcaster(self)

        ############## BUFFER ##############
        self.tf_buffer = Buffer()

        ############## TRANSFORM LISTENER ##############
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Frames names
        self.base_link_frame = "base_link"
        self.camera_frame = "head_front_camera_rgb_optical_frame"
        self.target_frame_name = "aruco_marker_target_frame"
        self.approach_frame_name = "aruco_marker_approach_frame"

        # Store last detected marker pose as a KDL frame (camera -> marker)
        self.camera_to_marker = None
        self.marker_header = None

        # Subscriber to aruco_single/transform (camera -> marker)
        self.create_subscription(TransformStamped, '/aruco_single/transform', self.sub_callback, 1)

        # Create a timer to call publish_frame periodically
        self.timer = self.create_timer(0.1, self.publish_frames)  # 10 Hz

        self.get_logger().info("FramePublisher node started!")

    # TransformStamped Frame -> KDL Frame
    def transform_stamped_to_kdl(self, ts: TransformStamped) -> kdl.Frame:
        # Translation
        t = ts.transform.translation
        pos = kdl.Vector(t.x, t.y, t.z)

        # Rotation
        q = ts.transform.rotation
        rot = kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w)

        return kdl.Frame(rot, pos)
    
    # KDL Frame -> TransformStamped Frame
    def kdl_to_transform_stamped(self, frame: kdl.Frame, parent: str, child: str, stamp) -> TransformStamped:
        t = TransformStamped()

        # Basic information
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child

        # Position
        x = frame.p[0]
        y = frame.p[1]
        z = frame.p[2]

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        # Rotation
        qx, qy, qz, qw = frame.M.GetQuaternion()

        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        return t

    # Store the camera -> marker frame from the incoming message
    def sub_callback(self, msg: TransformStamped):
        try:
            self.camera_to_marker = self.transform_stamped_to_kdl(msg)
            self.marker_header = msg.header
        except Exception as e:
            self.get_logger().error(f"Failed to convert marker TransformStamped to KDL: {e}")

    # Publish and broadcast the frame 
    def publish_frames(self):
        # Need a detected marker and a TF from base_link to camera
        if self.camera_to_marker is None:
            return

        try:
            # lookup transform: base_link <- camera
            base_T_camera_ts = self.tf_buffer.lookup_transform(self.base_link_frame, self.camera_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warning(f"Could not lookup base_link<-camera transform: {e}")
            return

        # convert base_T_camera to KDL
        try:
            base_T_camera = self.transform_stamped_to_kdl(base_T_camera_ts)
        except Exception as e:
            self.get_logger().error(f"Failed to convert base_T_camera to KDL: {e}")
            return

        # compose: base_T_marker = base_T_camera * camera_T_marker
        base_T_marker = base_T_camera * self.camera_to_marker
        # rotate about np.pi/2 around y the target frame
        base_T_marker.M = base_T_marker.M * kdl.Rotation.RotY(np.pi/2)

        
        # approach frame: offset along marker's +z by 0.5 m (in marker frame) and Rotation of 90 about y axis
        approach_twist = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(-0.4, 0.0, 0.0))
        base_T_approach = base_T_marker * approach_twist

        now_stamp = self.get_clock().now().to_msg()

        # Create TransformStamped messages and broadcast them with parent = base_link
        t_target = self.kdl_to_transform_stamped(base_T_marker, self.base_link_frame, self.target_frame_name, now_stamp)
        t_approach = self.kdl_to_transform_stamped(base_T_approach, self.base_link_frame, self.approach_frame_name, now_stamp)

        # Send the transformations
        self.tf_broadcaster.sendTransform([t_target, t_approach])


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()