#!/usr/bin/env python3
"""
orchard_map_node.py

Waits for the orchard's own tree list to arrive, builds ONE whole-orchard
occupancy grid from it, and publishes it as a standard nav_msgs/OccupancyGrid
on `map_topic` (default "orchard/occupancy_grid") -- see orchard_map.py's own
module docstring for the full rationale (built once at startup, matching
problog_project's own load-once map.yaml, rather than a fresh grid rebuilt on
every plan_service_node query).

Published with TRANSIENT_LOCAL durability, the same QoS Nav2's own map_server
uses, so a subscriber started after this node (plan_service_node, RViz, ...)
still receives the map on connection without needing to be listening at the
exact moment it was published.

This node builds and publishes EXACTLY ONCE per orchard payload received (the
orchard doesn't change shape mid-mission), not on a timer -- if the orchard is
ever republished with a different tree list (e.g. a new mission), it rebuilds
and republishes then.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

from amiga_ros2_planners.orchard_map import build_grid_map, grid_to_occupancy_grid_msg
from amiga_ros2_planners.orchard_obstacles import OrchardObstacleStore


class OrchardMapNode(Node):
    def __init__(self):
        super().__init__("orchard_map_node")

        self.declare_parameter("orchard_topic", "orchard/tree_info_json")
        self.declare_parameter("datum_lat", 37.3611)
        self.declare_parameter("datum_lon", -120.4322)
        self.declare_parameter("tree_obstacle_radius", 0.5)
        self.declare_parameter("map_topic", "orchard/occupancy_grid")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("resolution", 0.25)
        self.declare_parameter("margin", 5.0)
        self.declare_parameter("poll_period_s", 1.0)

        self._obstacles = OrchardObstacleStore(
            self,
            self.get_parameter("orchard_topic").value,
            self.get_parameter("datum_lat").value,
            self.get_parameter("datum_lon").value,
            self.get_parameter("tree_obstacle_radius").value,
        )

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._pub = self.create_publisher(
            OccupancyGrid, self.get_parameter("map_topic").value, qos)

        self._last_tree_count = None
        period = self.get_parameter("poll_period_s").value
        self._timer = self.create_timer(period, self._maybe_build_and_publish)

    def _maybe_build_and_publish(self):
        if not self._obstacles.has_received():
            return
        obstacles = self._obstacles.get_obstacles()
        # Rebuild only when the tree count actually changed -- a new
        # orchard payload, not every poll tick of an unchanged one.
        if len(obstacles) == self._last_tree_count:
            return
        self._last_tree_count = len(obstacles)

        grid = build_grid_map(
            obstacles,
            self.get_parameter("resolution").value,
            self.get_parameter("margin").value,
        )
        msg = grid_to_occupancy_grid_msg(
            grid, self.get_parameter("map_frame").value, self.get_clock().now().to_msg())
        self._pub.publish(msg)
        self.get_logger().info(
            f"published {grid.width}x{grid.height} occupancy grid "
            f"({len(obstacles)} trees) on "
            f"'{self.get_parameter('map_topic').value}'")


def main(args=None):
    rclpy.init(args=args)
    node = OrchardMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
