#!/usr/bin/env python3
"""
tool_action_node.py

Hosts InstallTool/UninstallTool (amiga_interfaces/action/InstallTool,
amiga_interfaces/action/UninstallTool) -- the ROS2-action form of
problog_project's install_tool_leg/uninstall_tool_leg BT nodes (see
module/contracts/schema.yaml's own InstallTool/UninstallTool entries).
Durative, same start/halt shape as move_to_node's own MoveTo -- a
fixed-Duration action, halting early on whichever of `triggers`
(battery-only, see TRIGGER_FUNCTOR_TO_CONDITION below) fires first, or
resolving a success/failure coin flip once the full Duration elapses
with nothing halting it early. The robot never moves during either
action -- no path, no odometry noise, no FollowPath goal at all, just a
wall-clock wait with periodic trigger polling (move_to_node's own
_execute() loop, minus the FollowPath half).

Tracks the CURRENTLY EQUIPPED TOOL ("free"/"cart"/"plow") as this
node's own in-memory state -- a real run has exactly one of these
nodes, so this is the single source of truth -- and publishes it on a
latched (TRANSIENT_LOCAL) `tool_state` topic whenever it changes:
move_to_node reads it to select this tool's own speed (config.yaml's
tool.equipped.<tool>.speed), battery_sim_node reads it to select this
tool's own moving_drain_rate. Also publishes `tool_activity`
("idle"/"installing"/"uninstalling") for the duration of this action's
own span, which battery_sim_node uses to apply install_drain_rate_
pct_s/uninstall_drain_rate_pct_s instead of the normal idle/moving
rate -- config.yaml's own tool.install.drain_rate/tool.uninstall.
drain_rate, distinct from idle even though the robot isn't moving
either way (this feature's own request, per
module/translators/config_to_prolog.py's comment on install_tool_
drain_rate/1).

Preconditions (see basic_action_theory.pl's own
poss(start_install_tool(...))/poss(start_uninstall_tool(...))):
InstallTool requires NO tool currently equipped; UninstallTool requires
THIS SPECIFIC tool currently equipped. A live BT tree has no static
guarantee against violating these the way a ProbLog plan's own poss/2
check does, so a violated precondition here just aborts the goal
(reason="already_equipped"/"not_equipped") -- a defensive addition,
not a port of any schema.yaml vocabulary (there isn't one for this
case).

Same rclpy-under-MultiThreadedExecutor caveat as move_to_node.py: never
call rclpy.spin_once()/spin_until_future_complete(self, ...) from
inside an action's own execute callback -- see that module's own
constructor comment for the full story. This node reuses the exact
same _wait_for_future() pattern.
"""
import random
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

from amiga_interfaces.action import InstallTool, UninstallTool
from amiga_interfaces.srv import EvaluateCondition

_TOOL_KINDS = ("cart", "plow")

# Battery-only, same functor syntax move_to_node.py's own
# TRIGGER_PATTERN/TRIGGER_FUNCTOR_TO_CONDITION use, restricted to the
# battery-related entries -- schema.yaml's own InstallTool/UninstallTool
# triggers port is a HARD translation-time error for any motion-based
# name in the formal pipeline; this node has no separate translation
# pass, so it degrades a disallowed name to a warning + ignore instead
# (same non-fatal treatment move_to_node.py already gives an
# unrecognized trigger).
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
        self.declare_parameter("install_success_probability", 0.9)
        self.declare_parameter("uninstall_success_probability", 0.9)
        self.declare_parameter("tool_state_topic", "tool_state")
        self.declare_parameter("tool_activity_topic", "tool_activity")

        # In-memory only -- this node's own lifetime IS the mission's
        # lifetime (same "one process, one source of truth" assumption
        # move_to_node.py's own trigger-checking loop already makes).
        self._equipped_tool = "free"

        self._cb_group = ReentrantCallbackGroup()
        self._condition_client = self.create_client(
            EvaluateCondition, "evaluate_condition", callback_group=self._cb_group)

        self._tool_state_pub = self.create_publisher(
            String, self.get_parameter("tool_state_topic").value, _LATCHED_QOS)
        self._tool_activity_pub = self.create_publisher(
            String, self.get_parameter("tool_activity_topic").value, _LATCHED_QOS)
        # Publish the initial state immediately -- a late-joining
        # subscriber (move_to_node/battery_sim_node, started in any
        # order) still gets "free"/"idle" via TRANSIENT_LOCAL even if
        # it subscribes before this line ever runs again.
        self._publish_tool_state()
        self._publish_tool_activity("idle")

        self._install_server = ActionServer(
            self, InstallTool, "install_tool",
            lambda gh: self._execute(gh, installing=True),
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb_group)
        self._uninstall_server = ActionServer(
            self, UninstallTool, "uninstall_tool",
            lambda gh: self._execute(gh, installing=False),
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb_group)

        self.get_logger().info(
            "tool_action_node ready on 'install_tool'/'uninstall_tool'")

    def _publish_tool_state(self):
        self._tool_state_pub.publish(String(data=self._equipped_tool))

    def _publish_tool_activity(self, activity):
        self._tool_activity_pub.publish(String(data=activity))

    @staticmethod
    def _wait_for_future(future, timeout_sec, poll_interval_s=0.02):
        """See move_to_node.py's own _wait_for_future -- identical
        rationale (never spin this node re-entrantly from inside an
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
                    f"trigger '{trigger}' not supported by install_tool/"
                    f"uninstall_tool (battery-only; supported: "
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

    def _execute(self, goal_handle, installing):
        goal = goal_handle.request
        tool = goal.tool
        action_kind = InstallTool if installing else UninstallTool
        label = "InstallTool" if installing else "UninstallTool"
        result = action_kind.Result()

        if tool not in _TOOL_KINDS:
            self.get_logger().error(f"{label}: unknown tool '{tool}' (expected cart or plow)")
            goal_handle.abort()
            result.reason, result.status = "unknown_tool", False
            return result

        if installing and self._equipped_tool != "free":
            self.get_logger().error(
                f"InstallTool: already equipped with '{self._equipped_tool}', "
                f"cannot install '{tool}'")
            goal_handle.abort()
            result.reason, result.status = "already_equipped", False
            return result
        if not installing and self._equipped_tool != tool:
            self.get_logger().error(
                f"UninstallTool: '{tool}' is not the currently equipped "
                f"tool ('{self._equipped_tool}')")
            goal_handle.abort()
            result.reason, result.status = "not_equipped", False
            return result

        duration_param = f"{'install' if installing else 'uninstall'}_duration_{tool}_s"
        duration = self.get_parameter(duration_param).value
        success_probability = self.get_parameter(
            "install_success_probability" if installing
            else "uninstall_success_probability").value
        poll_period = self.get_parameter("trigger_poll_period_s").value

        parsed_triggers = self._parse_triggers(list(goal.triggers))
        activity = "installing" if installing else "uninstalling"
        self._publish_tool_activity(activity)
        self.get_logger().info(f"{label}: starting {tool}, duration={duration}s")

        try:
            start = time.monotonic()
            fired_trigger = None
            while True:
                elapsed = time.monotonic() - start
                fb = action_kind.Feedback()
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
                self._equipped_tool = tool if installing else "free"
                self._publish_tool_state()
                self.get_logger().info(
                    f"{label}: succeeded, equipped tool is now "
                    f"'{self._equipped_tool}'")
            goal_handle.succeed()
            if installing:
                result.reason = "install_success" if success else "install_failure"
            else:
                result.reason = "uninstall_success" if success else "uninstall_failure"
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
