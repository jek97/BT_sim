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
pose.py's own docstring). A caller that already knows it wants to plan
from somewhere other than "here" isn't calling PlanWith; that is a
different, hypothetical planning query this service doesn't serve.

This node does NOT execute any BT.cpp leaf itself -- it is the backend a
future BT::RosServiceNode<PlanPath> C++ leaf in amiga_ros2_behavior_tree
would dial (see this package's own README for the current status of
that leaf). Feeding it directly with `ros2 service call` also works for
standalone testing.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from amiga_interfaces.srv import PlanPath
from amiga_ros2_planners import planning_core
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

        self._srv = self.create_service(PlanPath, "plan_path", self._on_request)
        self.get_logger().info("plan_service_node ready on 'plan_path'")

    def _on_request(self, request, response):
        xy = self._pose.get_xy()
        if xy is None:
            response.control_points = []
            response.reason = "no_pose"
            response.status = False
            return response
        sx, sy = xy
        obstacles = self._obstacles.get_obstacles()

        algorithm = request.algorithm
        if algorithm == "astar":
            control_points = planning_core.plan_astar_points(
                sx, sy, request.goal_x, request.goal_y, obstacles)
        elif algorithm == "straight":
            control_points = planning_core.straight_control_points(
                sx, sy, request.goal_x, request.goal_y)
        elif algorithm == "voronoi":
            control_points = planning_core.plan_voronoi_points(
                sx, sy, request.goal_x, request.goal_y, obstacles)
        elif algorithm == "follow_boarder":
            control_points = planning_core.follow_boarder_points(
                sx, sy, request.obstacle_id, request.offset, obstacles)
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
