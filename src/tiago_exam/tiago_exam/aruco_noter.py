#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
import PyKDL as kdl
from tf2_ros import TransformBroadcaster
from tf2_ros import Buffer, TransformListener

from tf2_ros import TransformException

import numpy as np


#TODO: 1. Abbassare camera (magari puntando all'aruco automaticamente)
#      2. rendere nav2 molto più preciso sull'orientamento + per non fare perfere tempo durante l'esplorazione fare un preempt al goal successivo senza aspettare la convergenza nella posizione e soprattuto orientamento
#      3. Continuare a rifinire il goal mandato a exploration così da posizionarsi perfettamente. 
#      4. Verificare quando il tracking sull'orientamento è stato risolto (cambiando yaw_tolerance in goal) che effettivamente il goal abbia un orientamento corretto
#      5. Scrivere le pose degli aruco in un yaml
#      6. Rendere più intelligente l'aggiunta al list di markers visti (es. se ne vede uno nuovo, lo aggiunge, altrimenti no) ma rendendo possibile la rifinitura. 



class ArucoNoter(Node):
    def __init__(self):
        super().__init__('aruco_noter')
        
        ############## TF BROADCASTER ##############
        self.tf_broadcaster = TransformBroadcaster(self)

        ############## BUFFER ##############
        self.tf_buffer = Buffer()

        ############## TRANSFORM LISTENER ##############
        self.tf_listener = TransformListener(self.tf_buffer, self)

    
        # Frames names
        self.map_frame = "map"
        self.base_link_frame = "base_link"
        self.camera_frame = "head_front_camera_rgb_optical_frame"
        self.marker_frame = None
        self.markers_seen = []

        # Subscriber
        self.place_sub = self.create_subscription(
            TransformStamped, 
            "aruco_place/transform", 
            lambda msg: self.coord_callback(msg, "place"), 
            1
        )

        self.pick_sub = self.create_subscription(
            TransformStamped, 
            "aruco_pick/transform", 
            lambda msg: self.coord_callback(msg, "pick"), 
            1
        )

    

        # Service Client
        self.move_to_marker_pub = self.create_publisher(PoseStamped, 'move_to_marker', 10)

        self.markers_seen = []

        self.get_logger().info("ArucoTfSubscriber node ready!")

        self.is_tracking_marker = False  # New flag


    def coord_callback(self, msg: TransformStamped, type=None):
        
        # if type in self.markers_seen:
        #     return
        
        if type == "place":
            self.marker_frame = "aruco_place_marker_frame"
        elif type == "pick":
            self.marker_frame = "aruco_pick_marker_frame"
        else:
            self.get_logger().error("Unknown type in coord_callback")
            return

        # if type in self.markers_seen:
        #     self.get_logger().info(f"Marker of type {type} already seen, ignoring.")
        #     return False
        # else:
        #     self.markers_seen.append(type)
        #     self.get_logger().info(f"Marker of type {type} seen for the first time, processing.")

       

        # 2. GET TRANSFORMS (With Error Handling)
        try:
            # base -> camera
            base_T_camera_ts = self.tf_buffer.lookup_transform(
                self.base_link_frame, 
                self.camera_frame, 
                rclpy.time.Time()
            )
            
            # map -> base
            map_T_base_ts = self.tf_buffer.lookup_transform(
                self.map_frame, 
                self.base_link_frame, 
                rclpy.time.Time()
            )

        except TransformException as e:
            # This is normal at startup. Just return and wait for the next callback.
            self.get_logger().warn(f"TF not ready yet: {e}")
            return


        marker_T_target = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0.0, 0.0, 1.0))  # 20 cm above the marker (out of the marker plane)
        camera_to_marker = self.transform_stamped_to_kdl(msg)
        base_T_camera = self.transform_stamped_to_kdl(base_T_camera_ts)
        map_T_base = self.transform_stamped_to_kdl(map_T_base_ts)

        # If we successfully got transforms, mark this marker as seen
        self.markers_seen.append(type)
        self.get_logger().info(f"Marker of type {type} processed successfully.")
        

        base_T_target = base_T_camera * camera_to_marker * marker_T_target

        base_T_target.M = base_T_target.M *  kdl.Rotation.RotZ(np.pi) * kdl.Rotation.RotY(-np.pi/2)
    
        # map -> target
        map_T_target = map_T_base * base_T_target

            
        # Prepare PoseStamped message for target
        pose_wrt_map = PoseStamped()
        pose_wrt_map.header.frame_id = self.map_frame
        pose_wrt_map.header.stamp = msg.header.stamp

        pose_wrt_map.pose.position.x = map_T_target.p.x()
        pose_wrt_map.pose.position.y = map_T_target.p.y()
        pose_wrt_map.pose.position.z = 0.0 # keep z at ground level for navigation

        qx, qy, qz, qw = map_T_target.M.GetQuaternion()
        pose_wrt_map.pose.orientation.x = qx
        pose_wrt_map.pose.orientation.y = qy
        pose_wrt_map.pose.orientation.z = qz
        pose_wrt_map.pose.orientation.w = qw
    

        # Publish the target pose
        self.move_to_marker_pub.publish(pose_wrt_map)
        self.get_logger().info(f"Published move to marker pose for {type} marker.")

            
    # def publish_frames(self):
    #     now_stamp = self.get_clock().now().to_msg()
    #     # Create TransformStamped messages and broadcast them with parent = base_link
    #     t_target = self.kdl_to_transform_stamped(self.map_T_target, self.map_frame, self.target_frame_name, now_stamp)
    #     t_approach = self.kdl_to_transform_stamped(self.marker_T_target, self.marker_frame, self.target_frame_name, now_stamp)
    #     self.tf_broadcaster.sendTransform([t_target, t_approach])

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


if __name__ == '__main__':
    main()