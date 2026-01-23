#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math

from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class TraversabilityToCostmap(Node):

    def __init__(self):
        super().__init__('traversability_to_costmap')

        # ================= PARAMETERS =================
        self.traversability_layer = 'traversability'

        # Set True ONLY if:
        # 1.0 = obstacle, 0.0 = free
        self.invert_traversability = False

        # Traversability threshold
        self.occupied_threshold = 0.3

        # ================= SUBSCRIBER =================
        self.sub = self.create_subscription(
            GridMap,
            '/elevation_mapping_node/elevation_map_raw',
            self.map_callback,
            10
        )

        # ================= PUBLISHER =================
        # Nav2 REQUIRES TRANSIENT_LOCAL for /map
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(
            OccupancyGrid,
            '/map2',
            qos
        )

        self.get_logger().info("Traversability → OccupancyGrid (/map) node started")

    # =================================================
    # ================= CALLBACK ======================
    # =================================================

    def map_callback(self, msg: GridMap):

        trav = self.gridmap_to_numpy(msg, self.traversability_layer)
        if trav is None:
            self.get_logger().warn("Traversability layer not found")
            return

        # -------- Unknown handling --------
        unknown_mask = np.isnan(trav)

        # Replace NaNs temporarily for processing
        trav = np.nan_to_num(trav, nan=0.0)

        if self.invert_traversability:
            trav = 1.0 - trav

        trav = np.clip(trav, 0.0, 1.0)

        # -------- OccupancyGrid encoding --------
        # -1 = unknown
        #  0 = free
        # 100 = occupied
        occ = np.zeros(trav.shape, dtype=np.int8)

        occ[trav < self.occupied_threshold] = 100
        occ[unknown_mask] = -1

        # -------- Build OccupancyGrid --------
        og = OccupancyGrid()
        og.header.stamp = msg.header.stamp
        og.header.frame_id = "map_raw"

        og.info.resolution = msg.info.resolution
        og.info.width = occ.shape[1]
        og.info.height = occ.shape[0]

        # GridMap pose is CENTER
        # OccupancyGrid origin is BOTTOM-LEFT
        og.info.origin = Pose()
        og.info.origin.position.x = (
            msg.info.pose.position.x - msg.info.length_x / 2.0
        )
        og.info.origin.position.y = (
            msg.info.pose.position.y + msg.info.length_y / 2.0
        )
        og.info.origin.orientation.z = -math.sin(math.pi / 4.0)
        og.info.origin.orientation.w = math.cos(math.pi / 4.0)

        
        occ = occ.T
        occ = np.flipud(occ)

        og.data = occ.flatten(order='C').tolist()

        self.pub.publish(og)

    # =================================================
    # ================= HELPERS =======================
    # =================================================

    def gridmap_to_numpy(self, msg: GridMap, layer_name: str):

        if layer_name not in msg.layers:
            return None

        idx = msg.layers.index(layer_name)
        data = np.array(msg.data[idx].data, dtype=np.float32)

        size_x = int(round(msg.info.length_x / msg.info.resolution))
        size_y = int(round(msg.info.length_y / msg.info.resolution))

        if data.size != size_x * size_y:
            self.get_logger().error(
                f"Size mismatch: data={data.size}, expected={size_x * size_y}"
            )
            return None

        return data.reshape((size_y, size_x))


def main():
    rclpy.init()
    node = TraversabilityToCostmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

