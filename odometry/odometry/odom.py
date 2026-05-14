import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField, Imu
from std_msgs.msg import Header
from rover_msgs.msg import EncoderMsg
import tf_transformations
import tf2_ros
import math
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
import struct
from message_filters import Subscriber, ApproximateTimeSynchronizer


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Publisher for odometry data
        self.odom_publisher = self.create_publisher(Odometry, '/odom/wheel', 20)
        self.encoder_sub = self.create_subscription(EncoderMsg, '/encoder_data', self.update_odometry_state, 20)
        self.imu_data = self.create_subscription(Imu, '/unilidar/imu', self.imu_callback, 10)

        # Parameters
        self.declare_parameter('update_rate', 10)   # Hz
        self.declare_parameter('noise_level', 0.00) # Maximum noise in meters/radians

        self.quadx = 0.0
        self.quady = 0.0
        self.quadz = 0.0
        self.quadw = 1.0  # FIX: default to identity quaternion, not 0

        self.odom_to_base_footprint = TransformStamped()
        self.odom_msg = Odometry()

        # Initial position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.WHEEL_BASE = 0.92  # Distance between left and right wheels
        self.last_time = self.get_clock().now()

        # FIX: publish static transforms with zero time (required for static broadcaster)
        self.publish_static_transforms()

        # 60 Hz timer to keep TF buffer fresh
        self.create_timer(1.0 / 20.0, self.publish_odom)

    def publish_static_transforms(self):
        # FIX: use zero time (rclpy.time.Time()) for static transforms
        base_link_to_lidar = TransformStamped()
        base_link_to_lidar.header.stamp = rclpy.time.Time().to_msg()  # zero time = static
        base_link_to_lidar.header.frame_id = 'base_link'
        base_link_to_lidar.child_frame_id = 'unilidar_imu_initial'
        base_link_to_lidar.transform.translation.x = 0.63
        base_link_to_lidar.transform.translation.y = 0.0
        base_link_to_lidar.transform.translation.z = 0.44
        base_link_to_lidar.transform.rotation.x = 0.0
        base_link_to_lidar.transform.rotation.y = 0.0
        #base_link_to_lidar.transform.rotation.z = 0.0  # sin(90°/2)
        #base_link_to_lidar.transform.rotation.w = 1.0 
        base_link_to_lidar.transform.rotation.z = math.sin(math.pi/4) # sin(90°/2)
        base_link_to_lidar.transform.rotation.w = math.sin(math.pi/4 - 0.1)

        # FIX: map→odom is always identity — make it static, NOT dynamic
        # This was previously broadcast at ~10 Hz causing TF gaps and crashes
        
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = rclpy.time.Time().to_msg()  # zero time = static
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.translation.x = 0.0
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0
        
        self.static_broadcaster.sendTransform([base_link_to_lidar,map_to_odom])

    def imu_callback(self, sensor_msgs: Imu):
        self.quadx = sensor_msgs.orientation.x
        self.quady = sensor_msgs.orientation.y
        self.quadz = sensor_msgs.orientation.z
        self.quadw = sensor_msgs.orientation.w

    def update_odometry_state(self, encoder_msg: EncoderMsg):
        current_time = self.get_clock().now()

        V_left = (encoder_msg.sol_on + encoder_msg.sol_arka) / 2
        V_right = (encoder_msg.sag_on + encoder_msg.sag_arka) / 2

        V = (V_left + V_right) / 2
        omega = (V_right - V_left) / (self.WHEEL_BASE*2)

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        self.x += V * math.cos(self.theta) * dt
        self.y += V * math.sin(self.theta) * dt
        self.theta += omega * dt

        # Update shared state — timestamp will be refreshed in publish_odom
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        self.odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.odom_msg.twist.twist.linear.x = V
        self.odom_msg.twist.twist.linear.y = 0.0
        self.odom_msg.twist.twist.linear.z = 0.0
        self.odom_msg.twist.twist.angular.x = 0.0
        self.odom_msg.twist.twist.angular.y = 0.0
        self.odom_msg.twist.twist.angular.z = omega

        self.odom_to_base_footprint.header.frame_id = 'odom'
        self.odom_to_base_footprint.child_frame_id = 'base_link'
        self.odom_to_base_footprint.transform.translation.x = self.x
        self.odom_to_base_footprint.transform.translation.y = self.y
        self.odom_to_base_footprint.transform.translation.z = 0.0
        self.odom_to_base_footprint.transform.rotation.x = 0.0
        self.odom_to_base_footprint.transform.rotation.y = 0.0
        self.odom_to_base_footprint.transform.rotation.z = math.sin(self.theta / 2.0)
        self.odom_to_base_footprint.transform.rotation.w = math.cos(self.theta / 2.0)

        # FIX: removed dynamic costmap (map→odom) broadcast from here

    def publish_odom(self):
        # Always stamp with current time so TF buffer stays fresh at 60 Hz
        now = self.get_clock().now().to_msg()
        self.odom_msg.header.stamp = now
        self.odom_to_base_footprint.header.stamp = now

        self.tf_broadcaster.sendTransform(self.odom_to_base_footprint)
        self.odom_publisher.publish(self.odom_msg)


def main():
    rclpy.init()
    node = OdometryPublisher()
    # FIX: MultiThreadedExecutor prevents timer starvation when callbacks are slow
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
