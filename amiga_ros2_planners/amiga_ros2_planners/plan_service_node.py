#!/usr/bin/env python3
"""
plan_service_node.py

Hosts PlanPath (amiga_interfaces/srv/PlanPath) -- the ROS2-service form
of problog_project's PlanWith BT node (see that project's
module/contracts/schema.yaml and bt_actions.py). One consolidated
service covering all four algorithms, dispatched on the request's own
`algorithm` field, exactly mirroring how problog_project collapsed them
into one BT.cpp action rather than four.

The request deliberately carries NO start position: PlanWith's own
contract is "plan from the CURRENT position" (schema.yaml's own words),
so this server resolves it itself via tf2 (PoseProvider) at call time --
same reasoning as every other live-state lookup in this package (see
pose.py's own docstring).

A* plans against the whole-orchard grid orchard_map_node publishes
(subscribed here with TRANSIENT_LOCAL durability, so it's picked up
whenever that node published it -- before or after this one started),
falling back to a query-scoped grid built on the spot
(planning_core.build_occupancy_grid) if that map hasn't arrived yet.
Voronoi and follow_boarder both work on obstacle_polygons (see
planning_core.py's own module docstring on why they're byte-for-byte
ports of problog_project's polygon algorithms, not a circle-specialized
rewrite) -- built fresh each call via
planning_core.obstacles_to_polygons, since a regular-polygon
approximation is cheap and these two algorithms don't need a cached
raster the way A* does.

This node does NOT execute any BT.cpp leaf itself -- it is the backend a
future BT::RosServiceNode<PlanPath> C++ leaf in amiga_ros2_behavior_tree
would dial (see this package's own README for the current status of
that leaf). Feeding it directly with `ros2 service call` also works for
standalone testing.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

from amiga_interfaces.srv import PlanPath
from amiga_ros2_planners import planning_core
from amiga_ros2_planners.frame_transform import ProblogFrameTransform
from amiga_ros2_planners.orchard_map import occupancy_grid_msg_to_grid
from amiga_ros2_planners.orchard_obstacles import OrchardObstacleStore
from amiga_ros2_planners.pose import PoseProvider


class PlanServiceNode(Node):
    def __init__(self):
        super().__init__("plan_service_node")

        self.declare_parameter("orchard_topic", "orchard/tree_info_json")
        # Same datum as amiga-ros2-nav/amiga_localization/config/
        # base_ekf.yaml's own `datum:` -- MUST match it, see gps.py's
        # own docstring and this package's README.
        self.declare_parameter("datum_lat", 37.3611)
        self.declare_parameter("datum_lon", -120.4322)
        self.declare_parameter("tree_obstacle_radius", 0.5)
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_topic", "orchard/occupancy_grid")
        # Identity by default -- see frame_transform.py's own docstring.
        # Set these to align a problog_project mission's own goal points
        # (authored in that problem's map.yaml frame) with this
        # simulation's live orchard/tf2 frame.
        self.declare_parameter("problog_frame_origin_x", 0.0)
        self.declare_parameter("problog_frame_origin_y", 0.0)
        self.declare_parameter("problog_frame_yaw_deg", 0.0)

        self._goal_transform = ProblogFrameTransform(
            self.get_parameter("problog_frame_origin_x").value,
            self.get_parameter("problog_frame_origin_y").value,
            self.get_parameter("problog_frame_yaw_deg").value,
        )

        self._obstacles = OrchardObstacleStore(
            self,
            self.get_parameter("orchard_topic").value,
            self.get_parameter("datum_lat").value,
            self.get_parameter("datum_lon").value,
            self.get_parameter("tree_obstacle_radius").value,
        )
        self._pose = PoseProvider(
            self,
            self.get_parameter("reference_frame").value,
            self.get_parameter("base_frame").value,
        )

        self._static_grid = None
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value,
            self._on_map, map_qos)

        self._srv = self.create_service(PlanPath, "plan_path", self._on_request)
        self.get_logger().info("plan_service_node ready on 'plan_path'")

    def _on_map(self, msg):
        self._static_grid = occupancy_grid_msg_to_grid(msg)

    def _on_request(self, request, response):
        xy = self._pose.get_xy()
        if xy is None:
            response.control_points = []
            response.reason = "no_pose"
            response.status = False
            return response
        sx, sy = xy
        obstacles = self._obstacles.get_obstacles()
        # Every algorithm below except follow_boarder takes a goal point,
        # which may be authored in a problog_project problem's own map
        # frame -- see frame_transform.py's own docstring. Identity
        # (default params) makes this a no-op for a mission already
        # authored against this sim's own frame.
        goal_x, goal_y = self._goal_transform.to_sim_frame(
            request.goal_x, request.goal_y)

        algorithm = request.algorithm
        if algorithm == "astar":
            if self._static_grid is not None:
                control_points = planning_core.plan_astar_points(
                    sx, sy, goal_x, goal_y, grid=self._static_grid)
            else:
                self.get_logger().warn(
                    "no orchard map received yet from orchard_map_node -- "
                    "falling back to a query-scoped grid")
                control_points = planning_core.plan_astar_points(
                    sx, sy, goal_x, goal_y, obstacles=obstacles)
        elif algorithm == "straight":
            control_points = planning_core.straight_control_points(
                sx, sy, goal_x, goal_y)
        elif algorithm == "voronoi":
            obstacle_polygons = planning_core.obstacles_to_polygons(obstacles)
            control_points = planning_core.plan_voronoi_points(
                sx, sy, goal_x, goal_y, obstacle_polygons)
        elif algorithm == "follow_boarder":
            # Resolved through OrchardObstacleStore's own lenient lookup
            # (exact id, falling back to a bare tree-index match) BEFORE
            # building the polygon list, so a problog_project tree's own
            # obstacle_id (e.g. "obs5", meaning nothing to this orchard)
            # still resolves to "tree_5" here -- see
            # OrchardObstacleStore.get_obstacle's own docstring for why.
            resolved = self._obstacles.get_obstacle(request.obstacle_id)
            if resolved is None:
                response.control_points = []
                response.reason = "no_obstacle"
                response.status = False
                return response
            obstacle_polygons = planning_core.obstacles_to_polygons(obstacles)
            control_points = planning_core.follow_boarder_points(
                sx, sy, resolved.id, request.offset, obstacle_polygons)
        else:
            response.control_points = []
            response.reason = f"unknown_algorithm({algorithm})"
            response.status = False
            return response

        if control_points is None:
            response.control_points = []
            response.reason = "no_obstacle" if algorithm == "follow_boarder" else "no_path"
            response.status = False
            return response

        response.control_points = [Point(x=float(x), y=float(y), z=0.0)
                                    for x, y in control_points]
        response.reason = "completed"
        response.status = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlanServiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
