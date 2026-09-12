#!/usr/bin/env python3
"""
tool_action_node.py

Hosts InstallTool/UninstallTool/DeployTool/RetractTool
(amiga_interfaces/action/{Install,Uninstall,Deploy,Retract}Tool) -- the
ROS2-action form of problog_project's install_tool_leg/uninstall_tool_leg/
deploy_tool_leg/retract_tool_leg BT nodes (see
module/contracts/schema.yaml's own entries for each). All four are
durative, same start/halt shape as move_to_node's own MoveTo -- a
fixed-Duration action, halting early on whichever of `triggers`
(battery-only, see TRIGGER_FUNCTOR_TO_CONDITION below) fires first, or
resolving a success/failure coin flip once the full Duration elapses
with nothing halting it early. The robot never moves during any of the
four -- no path, no odometry noise, no FollowPath goal at all, just a
wall-clock wait with periodic trigger polling (move_to_node's own
_execute() loop, minus the FollowPath half).

Also hosts HitchedId/NearestToolOfKind (amiga_interfaces/srv/
{HitchedId,NearestToolOfKind}) -- the ROS2-SERVICE form of
problog_project's own query leaves of the same name (schema.yaml),
INSTANTANEOUS and side-effect-free, unlike the four durative actions
above. Both read this SAME node's own in-memory _equipped_tool/
_equipped_instance_id/_tool_instances state (see _on_hitched_id/
_on_nearest_tool_of_kind below) -- exactly the state Install/Uninstall/
Deploy/RetractTool already maintain, not a second source of truth.

TOOL INSTANCES (problog_project's own tool-instance-id refactor): a BT
tree's `tool` port on any of the four actions above names a specific
tool INSTANCE id (e.g. "cart1"), not a kind -- multiple instances of
the same kind (cart/plow) can exist, each with its own fixed starting
position. This node reads them all once at startup from the
`tool_instances` param, a JSON object {id: {kind, x, y}, ...} (this
problem's own config.yaml, tool.instances -- see
problog_problem.tool_params). InstallTool resolves `tool`'s own kind
from this table (unknown id -> "unknown_tool_instance") and additionally
requires the robot's CURRENT position be within `install_range` metres
of that instance's declared (x, y) -- basic_action_theory.pl's own
poss(start_install_tool(...)) requires the identical
holds(distance_below(GX,GY,Range),S) check, reusing PoseProvider (the
same tf2 helper condition_service_node.py/move_to_node.py already use)
rather than a fresh geometry implementation.

DEPLOY/RETRACT: lower/raise a tool that's ALREADY installed --
currently restricted to kind=plow only (mirroring basic_action_theory.
pl's own poss(start_deploy_tool(...)) kind check: a future kind could
gain the same behaviour by relaxing _DEPLOYABLE_KINDS below). Between a
successful DeployTool and its matching RetractTool, this node's own
tracked `deployed` flag is true, published on a THIRD latched topic
(`tool_deployed`, "true"/"false") -- move_to_node reads it (alongside
tool_state) to select tool.equipped.<kind>.deployed_speed instead of
the regular .speed for a walk, battery_sim_node reads it the same way
for .deployed_moving_drain_rate. UninstallTool additionally requires
the tool NOT currently deployed (must RetractTool first).

Tracks the CURRENTLY EQUIPPED TOOL's KIND ("free"/"cart"/"plow") as
this node's own in-memory state -- a real run has exactly one of these
nodes, so this is the single source of truth -- and publishes it on a
latched (TRANSIENT_LOCAL) `tool_state` topic whenever it changes, same
as before the tool-instance-id refactor (hitch/2 itself stays KIND-
valued in basic_action_theory.pl too, for exactly the same "every
existing MoveTo-side kind-level physics caller needs zero changes"
reason). The specific INSTANCE id currently installed is tracked
separately (`_equipped_instance_id`, not published -- nothing outside
this node needs it) so UninstallTool/DeployTool/RetractTool can each
require THIS SPECIFIC instance, not just "something of its kind".
Also publishes `tool_activity`
("idle"/"installing"/"uninstalling"/"deploying"/"retracting") for the
duration of each action's own span, which battery_sim_node uses to
apply that action's own drain_rate instead of the normal idle/moving
rate.

Preconditions (see basic_action_theory.pl's own poss/2 for each):
InstallTool requires NO tool currently equipped AND the robot within
range of that instance's position; UninstallTool requires THIS
SPECIFIC instance currently equipped AND not deployed; DeployTool
requires THIS SPECIFIC instance currently equipped, of a deployable
kind, AND not already deployed; RetractTool requires THIS SPECIFIC
instance currently deployed. A live BT tree has no static guarantee
against violating these the way a ProbLog plan's own poss/2 check
does, so a violated precondition here just aborts the goal with a
descriptive reason -- a defensive addition, not a port of any
schema.yaml vocabulary (there isn't one for this case).

Same rclpy-under-MultiThreadedExecutor caveat as move_to_node.py: never
call rclpy.spin_once()/spin_until_future_complete(self, ...) from
inside an action's own execute callback -- see that module's own
constructor comment for the full story. This node reuses the exact
same _wait_for_future() pattern.
"""
import json
import random
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

from amiga_interfaces.action import InstallTool, UninstallTool, DeployTool, RetractTool
from amiga_interfaces.srv import EvaluateCondition, HitchedId, NearestToolOfKind
from amiga_ros2_planners.pose import PoseProvider

_TOOL_KINDS = ("cart", "plow")

# Kinds DeployTool/RetractTool currently accept -- mirrors
# basic_action_theory.pl's own poss(start_deploy_tool(...)) kind
# restriction exactly (see this module's own docstring).
_DEPLOYABLE_KINDS = ("plow",)

_ACTION_TYPES = {
    "install": InstallTool,
    "uninstall": UninstallTool,
    "deploy": DeployTool,
    "retract": RetractTool,
}

# Battery-only, same functor syntax move_to_node.py's own
# TRIGGER_PATTERN/TRIGGER_FUNCTOR_TO_CONDITION use, restricted to the
# battery-related entries -- schema.yaml's own triggers port on all
# four of these actions is a HARD translation-time error for any
# motion-based name in the formal pipeline; this node has no separate
# translation pass, so it degrades a disallowed name to a warning +
# ignore instead (same non-fatal treatment move_to_node.py already
# gives an unrecognized trigger).
TRIGGER_PATTERN = re.compile(r"^(\w+)\(\s*([-+]?[0-9]*\.?[0-9]+)\s*\)$")
TRIGGER_FUNCTOR_TO_CONDITION = {
    "battery_below": "BatteryBelow",
    "battery_over": "BatteryOver",
    "battery_equal": "BatteryEqual",
}

_LATCHED_QOS = QoSProfile(depth=1)
_LATCHED_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL
_LATCHED_QOS.reliability = ReliabilityPolicy.RELIABLE


class ToolActionNode(Node):
    def __init__(self):
        super().__init__("tool_action_node")

        self.declare_parameter("trigger_poll_period_s", 0.2)
        self.declare_parameter("install_duration_cart_s", 10.0)
        self.declare_parameter("install_duration_plow_s", 10.0)
        self.declare_parameter("uninstall_duration_cart_s", 10.0)
        self.declare_parameter("uninstall_duration_plow_s", 10.0)
        self.declare_parameter("deploy_duration_cart_s", 10.0)
        self.declare_parameter("deploy_duration_plow_s", 10.0)
        self.declare_parameter("retract_duration_cart_s", 10.0)
        self.declare_parameter("retract_duration_plow_s", 10.0)
        self.declare_parameter("install_success_probability", 0.9)
        self.declare_parameter("uninstall_success_probability", 0.9)
        self.declare_parameter("deploy_success_probability", 0.9)
        self.declare_parameter("retract_success_probability", 0.9)
        self.declare_parameter("tool_state_topic", "tool_state")
        self.declare_parameter("tool_activity_topic", "tool_activity")
        self.declare_parameter("tool_deployed_topic", "tool_deployed")
        # {"id": {"kind": "cart"|"plow", "x": ..., "y": ...}, ...} --
        # this problem's own config.yaml tool.instances, JSON-encoded
        # (ros2 launch has no clean way to pass a list-of-dicts
        # directly -- same reasoning problog_problem.tool_params's own
        # per-tool dicts get spelled out as individual launch args,
        # just JSON here since a variable-length instance list can't be
        # spelled out that way at all).
        self.declare_parameter("tool_instances", "{}")
        self.declare_parameter("install_range", 1.0)
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self._tool_instances = json.loads(self.get_parameter("tool_instances").value)

        # In-memory only -- this node's own lifetime IS the mission's
        # lifetime (same "one process, one source of truth" assumption
        # move_to_node.py's own trigger-checking loop already makes).
        self._equipped_tool = "free"
        self._equipped_instance_id = None
        self._deployed = False

        self._pose = PoseProvider(
            self,
            self.get_parameter("reference_frame").value,
            self.get_parameter("base_frame").value,
        )

        self._cb_group = ReentrantCallbackGroup()
        self._condition_client = self.create_client(
            EvaluateCondition, "evaluate_condition", callback_group=self._cb_group)

        self._tool_state_pub = self.create_publisher(
            String, self.get_parameter("tool_state_topic").value, _LATCHED_QOS)
        self._tool_activity_pub = self.create_publisher(
            String, self.get_parameter("tool_activity_topic").value, _LATCHED_QOS)
        self._tool_deployed_pub = self.create_publisher(
            String, self.get_parameter("tool_deployed_topic").value, _LATCHED_QOS)
        # Publish the initial state immediately -- a late-joining
        # subscriber (move_to_node/battery_sim_node, started in any
        # order) still gets "free"/"idle"/"false" via TRANSIENT_LOCAL
        # even if it subscribes before this line ever runs again.
        self._publish_tool_state()
        self._publish_tool_activity("idle")
        self._publish_tool_deployed()

        self._servers = [
            ActionServer(
                self, InstallTool, "install_tool",
                lambda gh: self._execute(gh, "install"),
                cancel_callback=lambda _gh: CancelResponse.ACCEPT,
                callback_group=self._cb_group),
            ActionServer(
                self, UninstallTool, "uninstall_tool",
                lambda gh: self._execute(gh, "uninstall"),
                cancel_callback=lambda _gh: CancelResponse.ACCEPT,
                callback_group=self._cb_group),
            ActionServer(
                self, DeployTool, "deploy_tool",
                lambda gh: self._execute(gh, "deploy"),
                cancel_callback=lambda _gh: CancelResponse.ACCEPT,
                callback_group=self._cb_group),
            ActionServer(
                self, RetractTool, "retract_tool",
                lambda gh: self._execute(gh, "retract"),
                cancel_callback=lambda _gh: CancelResponse.ACCEPT,
                callback_group=self._cb_group),
        ]

        # HitchedId/NearestToolOfKind -- INSTANTANEOUS, side-effect-free
        # queries against this SAME in-memory state (_equipped_tool/
        # _equipped_instance_id/_tool_instances), same "one node, one
        # source of truth" reasoning as everything else this node
        # tracks -- see each .srv's own header for the full rationale.
        self.create_service(HitchedId, "hitched_id", self._on_hitched_id)
        self.create_service(
            NearestToolOfKind, "nearest_tool_of_kind", self._on_nearest_tool_of_kind)

        self.get_logger().info(
            "tool_action_node ready on 'install_tool'/'uninstall_tool'/"
            "'deploy_tool'/'retract_tool'/'hitched_id'/'nearest_tool_of_kind'")

    def _on_hitched_id(self, request, response):
        if self._equipped_instance_id is None:
            response.reason, response.status, response.id = (
                "hitched_id_unavailable", False, "")
            return response
        response.reason = "hitched_id_found"
        response.status = True
        response.id = self._equipped_instance_id
        return response

    def _on_nearest_tool_of_kind(self, request, response):
        """Closest FREE (not currently hitched) instance of request.kind
        to the robot's own current position -- basic_action_theory.pl's
        own tool_position/4 excludes a hitched instance (see this
        module's own docstring), so _equipped_instance_id is skipped
        even if its own kind matches."""
        xy = self._pose.get_xy()
        if xy is None:
            response.reason, response.status = "no_tool_of_kind", False
            return response
        candidates = [
            (tool_id, instance) for tool_id, instance in self._tool_instances.items()
            if instance["kind"] == request.kind and tool_id != self._equipped_instance_id
        ]
        if not candidates:
            response.reason, response.status = "no_tool_of_kind", False
            return response
        tool_id, instance = min(
            candidates,
            key=lambda item: (item[1]["x"] - xy[0]) ** 2 + (item[1]["y"] - xy[1]) ** 2)
        response.reason = "nearest_tool_found"
        response.status = True
        response.id = tool_id
        response.x = instance["x"]
        response.y = instance["y"]
        return response

    def _publish_tool_state(self):
        self._tool_state_pub.publish(String(data=self._equipped_tool))

    def _publish_tool_activity(self, activity):
        self._tool_activity_pub.publish(String(data=activity))

    def _publish_tool_deployed(self):
        self._tool_deployed_pub.publish(String(data="true" if self._deployed else "false"))

    @staticmethod
    def _wait_for_future(future, timeout_sec, poll_interval_s=0.02):
        """See move_to_node.py's own _wait_for_future (identical
        rationale: never spin this node re-entrantly from inside an
        ActionServer execute callback running under main()'s own
        MultiThreadedExecutor)."""
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(poll_interval_s)
        return True

    def _parse_triggers(self, triggers):
        parsed = []
        for trigger in triggers:
            match = TRIGGER_PATTERN.match(trigger.strip())
            if not match:
                self.get_logger().warn(
                    f"unrecognized trigger syntax, ignoring: '{trigger}'")
                continue
            functor, value = match.group(1), float(match.group(2))
            condition = TRIGGER_FUNCTOR_TO_CONDITION.get(functor)
            if condition is None:
                self.get_logger().warn(
                    f"trigger '{trigger}' not supported by tool_action_node "
                    f"(battery-only; supported: "
                    f"{sorted(TRIGGER_FUNCTOR_TO_CONDITION)}), ignoring")
                continue
            parsed.append((trigger, condition, value))
        return parsed

    def _check_triggers(self, parsed_triggers):
        """Same shape as move_to_node.py's own _check_triggers -- one
        EvaluateCondition call per trigger per poll."""
        for original, condition, threshold in parsed_triggers:
            if not self._condition_client.service_is_ready():
                continue
            request = EvaluateCondition.Request()
            request.condition = condition
            request.threshold = threshold
            future = self._condition_client.call_async(request)
            self._wait_for_future(future, timeout_sec=1.0)
            response = future.result()
            if response is not None and response.result:
                return original
        return None

    def _check_precondition(self, mode, tool_id):
        """None if OK to proceed, else an abort reason string. Every
        one of the four actions' own preconditions lives here, kept
        together rather than spread across _execute's own mode
        branches -- see this module's own docstring for what each
        checks and why."""
        if mode == "install":
            if self._equipped_tool != "free":
                return "already_equipped"
            instance = self._tool_instances.get(tool_id)
            if instance is None:
                return "unknown_tool_instance"
            xy = self._pose.get_xy()
            if xy is None:
                return "no_pose"
            install_range = self.get_parameter("install_range").value
            dist = ((xy[0] - instance["x"]) ** 2 + (xy[1] - instance["y"]) ** 2) ** 0.5
            if dist > install_range:
                return "too_far_from_tool"
            return None
        if mode == "uninstall":
            if self._equipped_instance_id != tool_id:
                return "not_equipped"
            if self._deployed:
                return "still_deployed"
            return None
        if mode == "deploy":
            if self._equipped_instance_id != tool_id:
                return "not_equipped"
            if self._equipped_tool not in _DEPLOYABLE_KINDS:
                return "unsupported_tool_kind"
            if self._deployed:
                return "already_deployed"
            return None
        if mode == "retract":
            if self._equipped_instance_id != tool_id:
                return "not_equipped"
            if not self._deployed:
                return "not_deployed"
            return None
        raise ValueError(f"unknown mode {mode!r}")

    def _apply_effect(self, mode, tool_id):
        """Updates equipped/deployed state on a SUCCESSFUL action --
        the mirror image of _check_precondition above."""
        if mode == "install":
            self._equipped_tool = self._tool_instances[tool_id]["kind"]
            self._equipped_instance_id = tool_id
            self._publish_tool_state()
        elif mode == "uninstall":
            # tool_position(Id,...) becomes wherever the robot actually
            # is on a successful uninstall (basic_action_theory.pl's
            # own tool_position/4) -- kept for a later InstallTool of
            # the SAME instance elsewhere; in-memory only, same
            # lifetime assumption as everything else this node tracks.
            xy = self._pose.get_xy()
            if xy is not None and tool_id in self._tool_instances:
                self._tool_instances[tool_id]["x"], self._tool_instances[tool_id]["y"] = xy
            self._equipped_tool = "free"
            self._equipped_instance_id = None
            self._publish_tool_state()
        elif mode == "deploy":
            self._deployed = True
            self._publish_tool_deployed()
        elif mode == "retract":
            self._deployed = False
            self._publish_tool_deployed()

    def _execute(self, goal_handle, mode):
        goal = goal_handle.request
        tool_id = goal.tool
        action_type = _ACTION_TYPES[mode]
        label = mode.capitalize() + "Tool"
        result = action_type.Result()

        precondition_failure = self._check_precondition(mode, tool_id)
        if precondition_failure is not None:
            self.get_logger().error(
                f"{label}: precondition failed for '{tool_id}': {precondition_failure}")
            goal_handle.abort()
            result.reason, result.status = precondition_failure, False
            return result

        kind = (self._tool_instances[tool_id]["kind"] if mode == "install"
                else self._equipped_tool)
        duration = self.get_parameter(f"{mode}_duration_{kind}_s").value
        success_probability = self.get_parameter(f"{mode}_success_probability").value
        poll_period = self.get_parameter("trigger_poll_period_s").value

        parsed_triggers = self._parse_triggers(list(goal.triggers))
        activity = {"install": "installing", "uninstall": "uninstalling",
                    "deploy": "deploying", "retract": "retracting"}[mode]
        self._publish_tool_activity(activity)
        self.get_logger().info(f"{label}: starting '{tool_id}', duration={duration}s")

        try:
            start = time.monotonic()
            fired_trigger = None
            while True:
                elapsed = time.monotonic() - start
                fb = action_type.Feedback()
                fb.elapsed_s = elapsed
                goal_handle.publish_feedback(fb)

                if elapsed >= duration:
                    break
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.reason, result.status = "canceled", False
                    return result
                if parsed_triggers:
                    fired_trigger = self._check_triggers(parsed_triggers)
                    if fired_trigger is not None:
                        break
                time.sleep(poll_period)

            if fired_trigger is not None:
                goal_handle.succeed()
                result.reason, result.status = fired_trigger, False
                return result

            success = random.random() < success_probability
            if success:
                self._apply_effect(mode, tool_id)
                self.get_logger().info(
                    f"{label}: succeeded ('{tool_id}'), equipped="
                    f"'{self._equipped_tool}' deployed={self._deployed}")
            goal_handle.succeed()
            result.reason = f"{mode}_success" if success else f"{mode}_failure"
            result.status = success
            return result
        finally:
            self._publish_tool_activity("idle")


def main(args=None):
    rclpy.init(args=args)
    node = ToolActionNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
