import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import time
import math
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import Quaternion, TransformStamped
import tf_transformations as tft
import numpy as np


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.publish_static_transforms()

    def publish_static_transforms(self):
        # Static transform: base_footprint -> base_link
        costmap = TransformStamped()
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.header.frame_id = 'map'
        costmap.child_frame_id = 'odom'
        costmap.transform.translation.x = 0.0
        costmap.transform.translation.y = 0.0
        costmap.transform.translation.z = 0.0
        costmap.transform.rotation.x = 0.0
        costmap.transform.rotation.y = 0.0
        costmap.transform.rotation.z = 0.0
        costmap.transform.rotation.w = 1.0
        
        base_footprint_to_base_link = TransformStamped()
        base_footprint_to_base_link.header.stamp = self.get_clock().now().to_msg()
        base_footprint_to_base_link.header.frame_id = 'base_link'
        base_footprint_to_base_link.child_frame_id = 'base_scan'
        base_footprint_to_base_link.transform.translation.x = 0.0
        base_footprint_to_base_link.transform.translation.y = 0.0
        base_footprint_to_base_link.transform.translation.z = 0.17317
        base_footprint_to_base_link.transform.rotation.x = 0.0
        base_footprint_to_base_link.transform.rotation.y = 0.0
        base_footprint_to_base_link.transform.rotation.z = 0.0
        base_footprint_to_base_link.transform.rotation.w = 1.0
        
        base_footprint_to_camera_link = TransformStamped()
        base_footprint_to_camera_link.header.frame_id = 'base_link'
        base_footprint_to_camera_link.child_frame_id = 'camera_link'
        base_footprint_to_camera_link.transform.translation.x = -0.3
        base_footprint_to_camera_link.transform.translation.y = 0.0
        base_footprint_to_camera_link.transform.translation.z = 0.04


        quat = tft.quaternion_from_euler(
        0.0,   # roll
        0.0,              # pitch
        0.0   )

        base_footprint_to_camera_link.transform.rotation.x = quat[0]
        base_footprint_to_camera_link.transform.rotation.y = quat[1]
        base_footprint_to_camera_link.transform.rotation.z = quat[2]
        base_footprint_to_camera_link.transform.rotation.w = quat[3]
        
        self.static_broadcaster.sendTransform([base_footprint_to_base_link,base_footprint_to_camera_link,costmap])
        




def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
