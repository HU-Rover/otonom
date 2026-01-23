#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class CostmapRepublisher(Node):
    def __init__(self):
        super().__init__('costmap_republisher')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.subscription = self.create_subscription(
            OccupancyGrid,
            'elevation_grid',   # exact topic name
            self.listener_callback,
            qos
        )

        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos
        )

    def listener_callback(self, msg: OccupancyGrid):
        self.publisher.publish(msg)
        self.get_logger().info("Republished OccupancyGrid to /map")

def main(args=None):
    rclpy.init(args=args)
    node = CostmapRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

