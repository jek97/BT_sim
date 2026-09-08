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

Two obstacle sources, selected by `obstacle_source`:

  "orchard" (default) -- this simulation's own live orchard (circular
    tree canopies, orchard_obstacles.py), for a mission written against
    the real Gazebo world. A* plans against the whole-orchard grid
    orchard_map_node publishes (TRANSIENT_LOCAL, so it's picked up
    whenever published), falling back to a query-scoped grid
    (planning_core.build_occupancy_grid) if that hasn't arrived yet.
    Voronoi/follow_boarder work on obstacle_polygons built fresh each
    call via planning_core.obstacles_to_polygons (circle -> N-gon
    approximation).

  "problog_problem" -- a problog_project problem folder's own
    obstacles_generated.pl (genuine polygons, loaded once via
    problog_problem.load_obstacle_polygons at startup, from
    `problem_dir`), for running that problem's own scenario directly
    -- see this package's README's "Running a problog_project problem
    folder directly" section. A* rasterizes a grid from these polygons
    per call (planning_core.build_polygon_grid/
    plan_astar_points_polygons -- no live map topic involved at all in
    this mode); Voronoi/follow_boarder use the polygons as-is, no
    circle adapter needed since they already are polygons.

This node does NOT execute any BT.cpp leaf itself -- it is the backend
the `PlanWith` BT::RosServiceNode leaf in amiga_ros2_behavior_tree
dials. Feeding it directly with `ros2 service call` also works for
standalone testing.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

from amiga_interfaces.srv import PlanPath
from amiga_ros2_planners import planning_core, problog_problem
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
        # simulation's live orchard/tf2 frame. Auto-computed by
        # run_problog_problem.launch.py when using "problog_problem"
        # obstacle_source below.
        self.declare_parameter("problog_frame_origin_x", 0.0)
        self.declare_parameter("problog_frame_origin_y", 0.0)
        self.declare_parameter("problog_frame_yaw_deg", 0.0)
        # "orchard" (default) or "problog_problem" -- see module docstring.
        self.declare_parameter("obstacle_source", "orchard")
        self.declare_parameter("problem_dir", "")

        self._goal_transform = ProblogFrameTransform(
            self.get_parameter("problog_frame_origin_x").value,
            self.get_parameter("problog_frame_origin_y").value,
            self.get_parameter("problog_frame_yaw_deg").value,
        )

        self._problog_mode = (
            self.get_parameter("obstacle_source").value == "problog_problem")
        self._obstacles = None
        self._problem_obstacle_polygons = []
        if self._problog_mode:
            problem_dir = self.get_parameter("problem_dir").value
            self._problem_obstacle_polygons = problog_problem.load_obstacle_polygons(
                problem_dir)
            self.get_logger().info(
                f"obstacle_source=problog_problem: loaded "
                f"{len(self._problem_obstacle_polygons)} obstacles from "
                f"'{problem_dir}'")
        else:
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
        if not self._problog_mode:
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

    def _obstacle_polygons(self):
        if self._problog_mode:
            return self._problem_obstacle_polygons
        return planning_core.obstacles_to_polygons(self._obstacles.get_obstacles())

    def _on_request(self, request, response):
        xy = self._pose.get_xy()
        if xy is None:
            response.control_points = []
            response.reason = "no_pose"
            response.status = False
            return response
        sx, sy = xy
        # Every algorithm below except follow_boarder takes a goal point,
        # which may be authored in a problog_project problem's own map
        # frame -- see frame_transform.py's own docstring. Identity
        # (default params) makes this a no-op for a mission already
        # authored against this sim's own frame.
        goal_x, goal_y = self._goal_transform.to_sim_frame(
            request.goal_x, request.goal_y)

        algorithm = request.algorithm
        if algorithm == "astar":
            if self._problog_mode:
                control_points = planning_core.plan_astar_points_polygons(
                    sx, sy, goal_x, goal_y, self._problem_obstacle_polygons)
            elif self._static_grid is not None:
                control_points = planning_core.plan_astar_points(
                    sx, sy, goal_x, goal_y, grid=self._static_grid)
            else:
                self.get_logger().warn(
                    "no orchard map received yet from orchard_map_node -- "
                    "falling back to a query-scoped grid")
                control_points = planning_core.plan_astar_points(
                    sx, sy, goal_x, goal_y,
                    obstacles=self._obstacles.get_obstacles())
        elif algorithm == "straight":
            control_points = planning_core.straight_control_points(
                sx, sy, goal_x, goal_y)
        elif algorithm == "voronoi":
            control_points = planning_core.plan_voronoi_points(
                sx, sy, goal_x, goal_y, self._obstacle_polygons())
        elif algorithm == "follow_boarder":
            obstacle_polygons = self._obstacle_polygons()
            # In "orchard" mode, resolved through OrchardObstacleStore's
            # own lenient lookup (exact id, falling back to a bare
            # tree-index match) so a problog_project tree's own
            # obstacle_id (e.g. "obs5") still resolves to "tree_5" --
            # see that store's own docstring. In "problog_problem" mode
            # the id already names one of this problem's own polygons
            # directly, so no resolution step is needed.
            obstacle_id = request.obstacle_id
            if not self._problog_mode:
                resolved = self._obstacles.get_obstacle(obstacle_id)
                if resolved is None:
                    response.control_points = []
                    response.reason = "no_obstacle"
                    response.status = False
                    return response
                obstacle_id = resolved.id
            control_points = planning_core.follow_boarder_points(
                sx, sy, obstacle_id, request.offset, obstacle_polygons)
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
