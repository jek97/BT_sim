#!/usr/bin/env python3
"""
battery_sim_node.py

A simulated battery signal for BatteryBelow/Equal/Over to read --
NEWLY WRITTEN, not ported: nothing in this simulation publishes a
battery level today (sim_bringup.launch.py's own `batteries` argument is
a fixed, launch-time value used only for the fleet coordinator's own bid
scoring, never a live topic; see that launch file's own comment). This
node drains a percentage over time, at one of FOUR rates, clocked in
real seconds instead of problog_project's own config.yaml's abstract
time-units (same 1-time-unit-is-1-real-second mapping
problog_problem.battery_params/tool_params already use):

  idle_drain_rate_pct_s       -- nothing else applies (see below).
  tool_moving_drain_rate_*    -- the robot's own odometry shows it
    moving faster than moving_speed_threshold_mps, rate selected by
    whichever tool tool_action_node last reported equipped (see
    problog_problem.tool_params's own "speed"/"moving_drain_rate_pct_s"
    per-tool dict) -- "free" if InstallTool has never succeeded, or
    tool_action_node isn't even running (this topic's own subscription
    default, see _on_tool_state below).
  install_drain_rate_pct_s/uninstall_drain_rate_pct_s -- tool_action_node
    reports activity="installing"/"uninstalling" on its own latched
    `tool_activity` topic for the WHOLE span of that action (the robot
    never moves during it, so this pre-empts the moving/idle check
    above entirely, matching config_to_prolog.py's own install_tool_
    drain_rate/1 note: "a battery drain rate install_tool/
    uninstall_tool use for THEIR OWN span").

Publishes sensor_msgs/BatteryState on `battery_topic` (default
"battery_state", relative -- namespaces the same way every other topic
in this repo's sim launch files does).
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


class BatterySimNode(Node):
    def __init__(self):
        super().__init__("battery_sim_node")

        self.declare_parameter("odom_topic", "odometry/filtered/local")
        self.declare_parameter("battery_topic", "battery_state")
        self.declare_parameter("start_percent", 100.0)
        self.declare_parameter("idle_drain_rate_pct_s", 0.01)
        self.declare_parameter("moving_drain_rate_pct_s", 0.1)
        self.declare_parameter("moving_speed_threshold_mps", 0.05)
        self.declare_parameter("publish_rate_hz", 2.0)
        # Per-tool MoveTo drain rate while that tool is equipped --
        # problog_problem.tool_params's own "moving_drain_rate_pct_s"
        # dict; "free" isn't its own param since it's just
        # moving_drain_rate_pct_s above (see that function's own
        # docstring on why there's no separate config.yaml key for "no
        # tool equipped").
        self.declare_parameter("tool_moving_drain_rate_cart_pct_s", 0.1)
        self.declare_parameter("tool_moving_drain_rate_plow_pct_s", 0.1)
        # install_tool/uninstall_tool's own span-specific rates --
        # problog_problem.tool_params's own install_drain_rate_pct_s/
        # uninstall_drain_rate_pct_s (both already default to
        # idle_drain_rate_pct_s's OWN value if config.yaml doesn't
        # override them, so passing idle_drain_rate_pct_s's value
        # through unchanged here is the correct "tool: section absent"
        # behavior, not a coincidence).
        self.declare_parameter("install_drain_rate_pct_s", 0.01)
        self.declare_parameter("uninstall_drain_rate_pct_s", 0.01)
        self.declare_parameter("tool_state_topic", "tool_state")
        self.declare_parameter("tool_activity_topic", "tool_activity")

        self._percent = self.get_parameter("start_percent").value
        self._idle_rate = self.get_parameter("idle_drain_rate_pct_s").value
        self._moving_rate = self.get_parameter("moving_drain_rate_pct_s").value
        self._speed_threshold = self.get_parameter("moving_speed_threshold_mps").value
        self._tool_moving_rate = {
            "free": self._moving_rate,
            "cart": self.get_parameter("tool_moving_drain_rate_cart_pct_s").value,
            "plow": self.get_parameter("tool_moving_drain_rate_plow_pct_s").value,
        }
        self._install_rate = self.get_parameter("install_drain_rate_pct_s").value
        self._uninstall_rate = self.get_parameter("uninstall_drain_rate_pct_s").value
        self._last_speed = 0.0
        # Defaults matching tool_action_node's own initial state/first
        # publish -- correct even if this node subscribes before
        # tool_action_node exists at all (no InstallTool/UninstallTool
        # in this mission's own tree), not just before it's started.
        self._equipped_tool = "free"
        self._activity = "idle"

        self._odom_sub = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10)
        self._pub = self.create_publisher(
            BatteryState, self.get_parameter("battery_topic").value, 10)

        # TRANSIENT_LOCAL isn't declared explicitly here: tool_action_node
        # publishes with it, and rclpy subscriptions match a publisher's
        # QoS automatically when none is given -- a plain depth-10
        # subscription (like odom above) still receives that late,
        # latched initial message correctly.
        self.create_subscription(
            String, self.get_parameter("tool_state_topic").value, self._on_tool_state, 10)
        self.create_subscription(
            String, self.get_parameter("tool_activity_topic").value, self._on_tool_activity, 10)

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self._period_s = period
        self._timer = self.create_timer(period, self._on_tick)

        self.get_logger().info(
            f"battery_sim_node ready, publishing on "
            f"'{self.get_parameter('battery_topic').value}'")

    def _on_odom(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._last_speed = (vx * vx + vy * vy) ** 0.5

    def _on_tool_state(self, msg):
        self._equipped_tool = msg.data

    def _on_tool_activity(self, msg):
        self._activity = msg.data

    def _current_drain_rate(self):
        if self._activity == "installing":
            return self._install_rate
        if self._activity == "uninstalling":
            return self._uninstall_rate
        if self._last_speed > self._speed_threshold:
            return self._tool_moving_rate.get(self._equipped_tool, self._moving_rate)
        return self._idle_rate

    def _on_tick(self):
        rate = self._current_drain_rate()
        self._percent = max(0.0, self._percent - rate * self._period_s)

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = float(self._percent) / 100.0
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
