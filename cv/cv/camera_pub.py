#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2


class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_image_publisher')

        self.publisher_ = self.create_publisher(Image, 'image', 10)
        self.bridge = CvBridge()

        # Configure and start the RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline.start(config)

        # Timer callback at 30Hz
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            self.get_logger().warn('No color frame received.')
            return

        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'camera_link'

        self.publisher_.publish(image_msg)
        self.get_logger().info('Published image frame.')

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

