#!/usr/bin/env python3
"""
condition_service_node.py

Hosts EvaluateCondition (amiga_interfaces/srv/EvaluateCondition) -- the
ROS2-service form of every problog_project Condition node EXCEPT
HaltedWith (see that .srv's own header for why HaltedWith is excluded:
it needs a tree's own blackboard history, which is not this node's, or
any service's, to have).

problog_project's own bt_actions.py deliberately left these as
"interface_only": their real semantics there are holds/2 lookups against
a Prolog situation, which only exists inside a ProbLog inference run.
This node is the actual, executable counterpart for this simulation --
each condition below reads the SAME live state plan_service_node.py
reads (tf2 pose, an obstacle source) plus a live battery topic, rather
than a situation history:

  DistanceBelow/Equal/Over   -- tf2 current (x,y) vs. an explicit goal
                                 point, exactly as schema.yaml describes.
  ObstacleInBound            -- tf2 current (x,y) vs. the NEAREST
                                 obstacle's own boundary clearance --
                                 a direct port of schema.yaml's own
                                 semantics.
  ObstacleOnPath             -- NOT a full port -- see its own handler
                                 below for exactly what is and isn't
                                 implemented yet, and why.
  BatteryBelow/Equal/Over    -- the latest sensor_msgs/BatteryState
                                 percentage from battery_sim_node.
  LineOfSightClear           -- tf2 current (x,y) -> goal segment vs.
                                 `obstacle_id`'s own obstacle shape.
  SampleValueBelow/Equal/Over -- NOT a live reading: looks up `sample_id`'s
                                 own drawn value (0-10) from
                                 sample_service_node's own latched
                                 `sample_values` topic (a JSON id->value
                                 object) -- see that node's own module
                                 docstring. A value is fixed the instant
                                 TakeSample draws it, so this is a
                                 lookup against that cached state, never
                                 a fresh computation.
  CollisionDetected            -- genuine Gazebo physics contact, NOT
                                 this service's own tf2-vs-tracked-
                                 obstacle-list approximation every other
                                 condition above uses: reads whether
                                 amiga_kinova/model.sdf's own
                                 chassis_contact_<side> sensor for
                                 `side` ("front"/"back"/"left"/"right",
                                 or "any") has reported a contact within
                                 the last `contact_stale_after_s`
                                 seconds. Reacts to the robot's REAL
                                 collision shape touching the
                                 environment's REAL collision shape
                                 during actual motion (see this
                                 package's own README on why every
                                 other condition here is a simplified
                                 stand-in, not this one).
  Hitched/Deployed             -- reads tool_action_node's own latched
                                 tool_state/tool_deployed topics (the
                                 SAME state InstallTool/UninstallTool/
                                 DeployTool/RetractTool maintain), not a
                                 fresh computation of any kind.
  PloughedAt/PloughedBetween   -- reads move_to_node's own latched
                                 ploughed_cells topic (the set of
                                 macro-cells a deployed plow has
                                 actually passed through -- see that
                                 node's own _mark_ploughed_here
                                 docstring), discretized through this
                                 SAME `plough_cell_size` param
                                 move_to_node uses (ploughing.py's
                                 cell_index/bresenham_cells).

Two obstacle sources, selected by `obstacle_source` (see
plan_service_node.py's own module docstring for the full rationale --
both nodes take the SAME two params):

  "orchard" (default) -- this simulation's own live orchard, circular
    tree canopies (geometry.py's circle primitives).
  "problog_problem" -- a problog_project problem folder's own
    obstacles_generated.pl, genuine polygons loaded once at startup
    (polygon_geometry.py's polygon primitives).

Equality checks (DistanceEqual/BatteryEqual) use a small tolerance
(`equal_tolerance_m`/`equal_tolerance_pct`) rather than exact float
equality -- schema.yaml itself notes that in a continuous model, exact
equality is "only true at whatever instant" a value crosses the
threshold, which a discrete-time service poll will essentially never
land on exactly.
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from ros_gz_interfaces.msg import Contacts

from amiga_interfaces.srv import EvaluateCondition
from amiga_ros2_planners import polygon_geometry, problog_problem
from amiga_ros2_planners.frame_transform import ProblogFrameTransform
from amiga_ros2_planners.geometry import nearest_obstacle, line_of_sight_clear
from amiga_ros2_planners.orchard_obstacles import OrchardObstacleStore
from amiga_ros2_planners.ploughing import bresenham_cells, cell_index
from amiga_ros2_planners.pose import PoseProvider

# Conditions whose goal_x/goal_y is a point to transform from a
# problog_project problem's own map frame -- see frame_transform.py's
# own docstring. Battery*/ObstacleInBound/ObstacleOnPath don't take a
# goal point at all, so they're left out on purpose.
_GOAL_FRAME_CONDITIONS = frozenset({
    "DistanceBelow", "DistanceEqual", "DistanceOver", "LineOfSightClear",
    "PloughedAt", "PloughedBetween",
})

# PloughedBetween ALSO carries a second point (p2_x/p2_y) needing the
# SAME transform as goal_x/goal_y above -- every other _GOAL_FRAME_
# CONDITIONS member has only the one point.
_P2_FRAME_CONDITIONS = frozenset({"PloughedBetween"})

_CONTACT_SIDES = ("front", "back", "left", "right")


class ConditionServiceNode(Node):
    def __init__(self):
        super().__init__("condition_service_node")

        self.declare_parameter("orchard_topic", "orchard/tree_info_json")
        self.declare_parameter("datum_lat", 37.3611)
        self.declare_parameter("datum_lon", -120.4322)
        self.declare_parameter("tree_obstacle_radius", 0.5)
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("battery_topic", "battery_state")
        self.declare_parameter("equal_tolerance_m", 0.1)
        self.declare_parameter("equal_tolerance_pct", 1.0)
        self.declare_parameter("sample_values_topic", "sample_values")
        self.declare_parameter("contact_topic_front", "chassis/contact_front")
        self.declare_parameter("contact_topic_back", "chassis/contact_back")
        self.declare_parameter("contact_topic_left", "chassis/contact_left")
        self.declare_parameter("contact_topic_right", "chassis/contact_right")
        self.declare_parameter("contact_stale_after_s", 0.5)
        self.declare_parameter("tool_state_topic", "tool_state")
        self.declare_parameter("tool_deployed_topic", "tool_deployed")
        self.declare_parameter("ploughed_cells_topic", "ploughed_cells")
        # MUST match move_to_node's own plough_cell_size -- see that
        # node's own module docstring.
        self.declare_parameter("plough_cell_size", 1.0)
        # Identity by default -- MUST match plan_service_node's own
        # problog_frame_origin_x/y/yaw_deg params, or a DistanceBelow
        # checked against the "same" goal PlanWith just targeted would
        # silently disagree with it. See frame_transform.py.
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
        self._battery_percent = None
        self.create_subscription(
            BatteryState, self.get_parameter("battery_topic").value,
            self._on_battery, 10)

        # sample_service_node.py's own latched sample-values state --
        # see this file's own module docstring on SampleValueBelow/
        # Equal/Over. TRANSIENT_LOCAL so this node gets the CURRENT
        # full dict immediately on subscribing, regardless of node
        # startup order.
        self._sample_values = {}
        sample_values_qos = QoSProfile(depth=1)
        sample_values_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        sample_values_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            String, self.get_parameter("sample_values_topic").value,
            self._on_sample_values, sample_values_qos)

        # chassis_contact_{front,back,left,right}'s own bridged topics
        # (amiga_ros2_gazebo/models/amiga_kinova/model.sdf) -- see this
        # file's own module docstring on CollisionDetected. Each side's
        # own "currently touching" state is the LATEST message's own
        # contacts list being non-empty, tracked with a short staleness
        # timeout (contact_stale_after_s) rather than trusted forever:
        # correct whether the underlying gz Contact sensor publishes
        # continuously (empty-list messages included, the same
        # convention every other periodic sensor in this repo uses) or
        # only while a contact is actually active -- either way, no
        # message for longer than the timeout means "not touching",
        # not "still touching from 10 minutes ago".
        self._last_contact_time = {side: None for side in _CONTACT_SIDES}
        for side in _CONTACT_SIDES:
            self.create_subscription(
                Contacts, self.get_parameter(f"contact_topic_{side}").value,
                (lambda msg, side=side: self._on_contact(side, msg)), 10)

        # Hitched/Deployed -- tool_action_node's own latched tool_state/
        # tool_deployed topics (see this file's own module docstring).
        # "free"/False are tool_action_node's own initial published
        # values, so these defaults are correct even before this node's
        # subscriptions receive their first message.
        self._equipped_tool = "free"
        self.create_subscription(
            String, self.get_parameter("tool_state_topic").value,
            self._on_tool_state, 10)
        self._deployed = False
        self.create_subscription(
            String, self.get_parameter("tool_deployed_topic").value,
            self._on_tool_deployed, 10)

        # PloughedAt/PloughedBetween -- move_to_node's own latched
        # ploughed_cells topic (a JSON list of [cx,cy] pairs). TRANSIENT_
        # LOCAL so this node gets the CURRENT full set immediately on
        # subscribing, same reasoning as sample_values above.
        self._ploughed_cells = set()
        ploughed_cells_qos = QoSProfile(depth=1)
        ploughed_cells_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        ploughed_cells_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            String, self.get_parameter("ploughed_cells_topic").value,
            self._on_ploughed_cells, ploughed_cells_qos)

        self._handlers = {
            "DistanceBelow": self._distance_below,
            "DistanceEqual": self._distance_equal,
            "DistanceOver": self._distance_over,
            "ObstacleInBound": self._obstacle_in_bound,
            "ObstacleOnPath": self._obstacle_on_path,
            "BatteryBelow": self._battery_below,
            "BatteryEqual": self._battery_equal,
            "BatteryOver": self._battery_over,
            "LineOfSightClear": self._line_of_sight_clear,
            "SampleValueBelow": self._sample_value_below,
            "SampleValueEqual": self._sample_value_equal,
            "SampleValueOver": self._sample_value_over,
            "CollisionDetected": self._collision_detected,
            "Hitched": self._hitched,
            "Deployed": self._deployed_condition,
            "PloughedAt": self._ploughed_at,
            "PloughedBetween": self._ploughed_between,
        }

        # Wait for tf2's first pose AND the first BatteryState message
        # before advertising the service: both can arrive several seconds
        # after this node starts (Gazebo's diff_drive_controller/
        # battery_sim_node), well after a mission can already be ticking.
        # Advertising early would answer a condition's first tick with a
        # hard result=False/"no_pose"/"no_battery_reading" -- FAILURE for
        # whatever plain (non-reactive) Sequence/Fallback holds it, which
        # bt_runner then treats as the mission's own final, permanent
        # outcome. Not advertising yet makes BT.cpp's RosServiceNode see
        # "service unavailable", which it retries as RUNNING instead. See
        # pose.py's own PoseProvider.wait_ready docstring.
        pose_ready = self._pose.wait_ready()
        battery_deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=30.0)
        while self._battery_percent is None and self.get_clock().now() < battery_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not pose_ready or self._battery_percent is None:
            self.get_logger().warn(
                "condition_service_node: pose_ready=%s battery_ready=%s "
                "after startup timeout, advertising 'evaluate_condition' "
                "anyway" % (pose_ready, self._battery_percent is not None))
        self._srv = self.create_service(
            EvaluateCondition, "evaluate_condition", self._on_request)
        self.get_logger().info(
            "condition_service_node ready on 'evaluate_condition'")

    def _on_battery(self, msg):
        self._battery_percent = msg.percentage * 100.0

    def _on_sample_values(self, msg):
        self._sample_values = json.loads(msg.data)

    def _on_contact(self, side, msg):
        if msg.contacts:
            self._last_contact_time[side] = self.get_clock().now()

    def _on_tool_state(self, msg):
        self._equipped_tool = msg.data

    def _on_tool_deployed(self, msg):
        self._deployed = msg.data == "true"

    def _on_ploughed_cells(self, msg):
        self._ploughed_cells = {tuple(cell) for cell in json.loads(msg.data)}

    # -- Hitched/Deployed -------------------------------------------------
    def _hitched(self, request, response):
        if request.kind:
            response.result = self._equipped_tool == request.kind
        else:
            response.result = self._equipped_tool != "free"
        return response

    def _deployed_condition(self, request, response):
        response.result = self._deployed
        return response

    # -- PloughedAt/PloughedBetween ----------------------------------------
    def _ploughed_at(self, request, response):
        cell_size = self.get_parameter("plough_cell_size").value
        cell = (cell_index(request.goal_x, cell_size), cell_index(request.goal_y, cell_size))
        response.result = cell in self._ploughed_cells
        return response

    def _ploughed_between(self, request, response):
        cell_size = self.get_parameter("plough_cell_size").value
        cx0, cy0 = cell_index(request.goal_x, cell_size), cell_index(request.goal_y, cell_size)
        cx1, cy1 = cell_index(request.p2_x, cell_size), cell_index(request.p2_y, cell_size)
        cells = bresenham_cells(cx0, cy0, cx1, cy1)
        response.result = all(cell in self._ploughed_cells for cell in cells)
        return response

    # -- CollisionDetected ------------------------------------------------
    def _collision_detected(self, request, response):
        side = request.side
        sides = _CONTACT_SIDES if side == "any" else (side,)
        if side != "any" and side not in _CONTACT_SIDES:
            response.result = False
            response.reason = f"unknown_side({side})"
            return response
        stale_after = self.get_clock().now() - rclpy.duration.Duration(
            seconds=self.get_parameter("contact_stale_after_s").value)
        response.result = any(
            self._last_contact_time[s] is not None and self._last_contact_time[s] > stale_after
            for s in sides)
        return response

    # -- current pose helper, shared by every position-based condition --
    def _current_xy_or_none(self, response):
        xy = self._pose.get_xy()
        if xy is None:
            response.result = False
            response.reason = "no_pose"
            return None
        return xy

    def _nearest_clearance(self, x, y):
        """(clearance to the nearest obstacle) under whichever obstacle
        source is active -- the one piece of real branching every
        Obstacle* handler below shares."""
        if self._problog_mode:
            _obstacle_id, clearance = polygon_geometry.nearest_polygon_obstacle(
                x, y, self._problem_obstacle_polygons)
            return clearance
        _obstacle, clearance = nearest_obstacle(x, y, self._obstacles.get_obstacles())
        return clearance

    # -- Distance* ------------------------------------------------------
    def _distance_below(self, request, response):
        xy = self._current_xy_or_none(response)
        if xy is None:
            return response
        x, y = xy
        d = ((x - request.goal_x) ** 2 + (y - request.goal_y) ** 2) ** 0.5
        response.result = d < request.threshold
        return response

    def _distance_equal(self, request, response):
        xy = self._current_xy_or_none(response)
        if xy is None:
            return response
        x, y = xy
        d = ((x - request.goal_x) ** 2 + (y - request.goal_y) ** 2) ** 0.5
        tolerance = self.get_parameter("equal_tolerance_m").value
        response.result = abs(d - request.threshold) <= tolerance
        return response

    def _distance_over(self, request, response):
        xy = self._current_xy_or_none(response)
        if xy is None:
            return response
        x, y = xy
        d = ((x - request.goal_x) ** 2 + (y - request.goal_y) ** 2) ** 0.5
        response.result = d > request.threshold
        return response

    # -- Obstacle* --------------------------------------------------------
    def _obstacle_in_bound(self, request, response):
        xy = self._current_xy_or_none(response)
        if xy is None:
            return response
        x, y = xy
        clearance = self._nearest_clearance(x, y)
        response.result = clearance < request.threshold
        return response

    def _obstacle_on_path(self, request, response):
        """PARTIAL PORT -- see this package's README "Known limitations".

        schema.yaml's own ObstacleOnPath is a check against the CURRENT
        WALK's full future trajectory (does it ever enter an obstacle,
        not just come near one) -- this service has no notion of an
        in-progress walk to check against, since move_to_node doesn't
        expose its own planned trajectory anywhere. Until that exists,
        this falls back to the same "is the robot presently on top of
        an obstacle" check ObstacleInBound-with-a-tight-threshold
        already gives (clearance < 0, i.e. genuinely inside an
        obstacle) -- correct for "am I in an obstacle right now", not
        for "will my planned path ever enter one", which is the
        actually-documented semantics.
        """
        xy = self._current_xy_or_none(response)
        if xy is None:
            return response
        x, y = xy
        clearance = self._nearest_clearance(x, y)
        response.result = clearance < 0.0
        response.reason = "partial_port: current-position-only, not full-trajectory"
        return response

    # -- Battery* ---------------------------------------------------------
    def _battery_or_none(self, response):
        if self._battery_percent is None:
            response.result = False
            response.reason = "no_battery_reading"
            return None
        return self._battery_percent

    def _battery_below(self, request, response):
        percent = self._battery_or_none(response)
        if percent is None:
            return response
        response.result = percent < request.threshold
        return response

    def _battery_equal(self, request, response):
        percent = self._battery_or_none(response)
        if percent is None:
            return response
        tolerance = self.get_parameter("equal_tolerance_pct").value
        response.result = abs(percent - request.threshold) <= tolerance
        return response

    def _battery_over(self, request, response):
        percent = self._battery_or_none(response)
        if percent is None:
            return response
        response.result = percent > request.threshold
        return response

    # -- LineOfSightClear ---------------------------------------------------
    def _line_of_sight_clear(self, request, response):
        xy = self._current_xy_or_none(response)
        if xy is None:
            return response
        x, y = xy

        if self._problog_mode:
            rings = next(
                (r for oid, r in self._problem_obstacle_polygons
                 if oid == request.obstacle_id), None)
            if rings is None:
                response.result = False
                response.reason = "no_such_obstacle"
                return response
            response.result = polygon_geometry.line_of_sight_clear_polygon(
                x, y, request.goal_x, request.goal_y, rings)
            return response

        obstacle = self._obstacles.get_obstacle(request.obstacle_id)
        if obstacle is None:
            response.result = False
            response.reason = "no_such_obstacle"
            return response
        response.result = line_of_sight_clear(
            x, y, request.goal_x, request.goal_y, obstacle)
        return response

    # -- SampleValue* -------------------------------------------------------
    def _sample_value_or_none(self, request, response):
        value = self._sample_values.get(request.sample_id)
        if value is None:
            response.result = False
            response.reason = "unknown_sample_id"
            return None
        return value

    def _sample_value_below(self, request, response):
        value = self._sample_value_or_none(request, response)
        if value is None:
            return response
        response.result = value < request.threshold
        return response

    def _sample_value_equal(self, request, response):
        value = self._sample_value_or_none(request, response)
        if value is None:
            return response
        # Sample values are already discrete integers (0-10), drawn
        # exactly, not a continuous quantity crossing a threshold at
        # some instant -- unlike DistanceEqual/BatteryEqual, no
        # tolerance window is needed, just float-safe exact comparison.
        response.result = abs(value - request.threshold) < 1e-9
        return response

    def _sample_value_over(self, request, response):
        value = self._sample_value_or_none(request, response)
        if value is None:
            return response
        response.result = value > request.threshold
        return response

    def _on_request(self, request, response):
        response.result = False
        response.reason = ""
        handler = self._handlers.get(request.condition)
        if handler is None:
            response.reason = f"unknown_condition({request.condition})"
            return response
        if request.condition in _GOAL_FRAME_CONDITIONS:
            request.goal_x, request.goal_y = self._goal_transform.to_sim_frame(
                request.goal_x, request.goal_y)
        if request.condition in _P2_FRAME_CONDITIONS:
            request.p2_x, request.p2_y = self._goal_transform.to_sim_frame(
                request.p2_x, request.p2_y)
        return handler(request, response)


def main(args=None):
    rclpy.init(args=args)
    node = ConditionServiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
