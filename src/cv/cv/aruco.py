#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import cv2
import cv2.aruco as aruco
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import tf_transformations as tf


class CamNode(Node):
    def __init__(self):
        super().__init__("cam_node")

        self.bridge = CvBridge()
        self.cam_pose_pub = self.create_publisher(PoseStamped, 'marker_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.aruco_parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_parameters)
        self.marker_size = 0.15  # meters

        # Precompute 3D object points for the marker corners
        self.obj_points = np.array([
            [-self.marker_size / 2,  self.marker_size / 2, 0],
            [ self.marker_size / 2,  self.marker_size / 2, 0],
            [ self.marker_size / 2, -self.marker_size / 2, 0],
            [-self.marker_size / 2, -self.marker_size / 2, 0]
        ], dtype=np.float32)

        self.init_realsense()
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def init_realsense(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        profile = self.pipeline.start(config)
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()

        self.camera_matrix = np.array([
            [intr.fx, 0.0, intr.ppx],
            [0.0, intr.fy, intr.ppy],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array(intr.coeffs[:5])

        self.get_logger().info("RealSense calibration loaded.")

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            self.get_logger().warn('No color frame received.')
            return

        self.image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            aruco.drawDetectedMarkers(self.image, corners, ids)

            for i, corner in enumerate(corners):
                # Estimate pose using solvePnP (replaces estimatePoseSingleMarkers)
                _, rvec, tvec = cv2.solvePnP(
                    self.obj_points,
                    corner[0],
                    self.camera_matrix,
                    self.dist_coeffs
                )
                rvec = rvec.reshape(1, 1, 3)
                tvec = tvec.reshape(1, 1, 3)

                # ---- Convert OpenCV (camera) coordinates to ROS coordinates ----
                # OpenCV: X (right), Y (down), Z (forward)
                # ROS:    X (forward), Y (left), Z (up)
                tvec_cv = tvec[0][0]
                tvec_ros = np.array([
                    tvec_cv[2],   # Z -> X
                    -tvec_cv[0],  # -X -> Y
                    -tvec_cv[1]   # -Y -> Z
                ])

                rot_cv, _ = cv2.Rodrigues(rvec[0])
                rot_ros = R.from_matrix(rot_cv)
                quat_cv = rot_ros.as_quat()  # [x, y, z, w]

                # Swap to ROS convention using the same rotation logic
                rotation_matrix_cv = tf.quaternion_matrix(quat_cv)
                # Try a 180-degree rotation around X to flip the camera "upright"
                fix_rotation = tf.euler_matrix(np.pi, 0, 0, axes='sxyz')

                transform_matrix = tf.translation_matrix(tvec_ros) @ fix_rotation @ rotation_matrix_cv

                map_position = tf.translation_from_matrix(transform_matrix)
                map_quat = tf.quaternion_from_matrix(transform_matrix)

                # --- Publish pose in 'map' frame ---
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = map_position[0]
                goal_pose.pose.position.y = map_position[1]
                goal_pose.pose.position.z = map_position[2]
                goal_pose.pose.orientation.x = map_quat[0]
                goal_pose.pose.orientation.y = map_quat[1]
                goal_pose.pose.orientation.z = map_quat[2]
                goal_pose.pose.orientation.w = map_quat[3]

                self.cam_pose_pub.publish(goal_pose)

                # --- Publish TF ---
                aruco_link = TransformStamped()
                aruco_link.header.stamp = self.get_clock().now().to_msg()
                aruco_link.header.frame_id = 'base_link'
                aruco_link.child_frame_id = 'aruco'
                aruco_link.transform.translation.x = map_position[0]
                aruco_link.transform.translation.y = map_position[1]
                aruco_link.transform.translation.z = map_position[2]
                aruco_link.transform.rotation.x = map_quat[0]
                aruco_link.transform.rotation.y = map_quat[1]
                aruco_link.transform.rotation.z = map_quat[2]
                aruco_link.transform.rotation.w = map_quat[3]
                self.tf_broadcaster.sendTransform(aruco_link)

                # Draw the axes on the image (replaces aruco.drawAxis)
                cv2.drawFrameAxes(self.image, self.camera_matrix, self.dist_coeffs,
                  rvec[0], tvec[0], self.marker_size * 0.5)

                # Display debug text
                cv2.putText(self.image, f"ID: {ids[i][0]}", (10, 30 + 30 * i),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(self.image, f"Pos: {np.round(map_position, 2)}", (10, 60 + 30 * i),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # Show image with overlays
        #cv2.imshow("Camera", self.image)
        #cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CamNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
