#!/usr/bin/env python3
"""
battery_sim_node.py

A simulated battery signal for BatteryBelow/Equal/Over to read --
NEWLY WRITTEN, not ported: nothing in this simulation publishes a
battery level today (sim_bringup.launch.py's own `batteries` argument is
a fixed, launch-time value used only for the fleet coordinator's own bid
scoring, never a live topic; see that launch file's own comment). This
node drains a percentage over time, at `moving_drain_rate` while the
robot's own odometry shows it moving faster than `moving_speed_threshold`
and `idle_drain_rate` otherwise -- the same two-rate idea
problog_project's config.yaml models (idle_drain_rate/moving_drain_rate),
just clocked in real seconds instead of the theory's own abstract
time-units, since this is driving a live topic rather than being
integrated symbolically.

Publishes sensor_msgs/BatteryState on `battery_topic` (default
"battery_state", relative -- namespaces the same way every other topic
in this repo's sim launch files does).
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState


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

        self._percent = self.get_parameter("start_percent").value
        self._idle_rate = self.get_parameter("idle_drain_rate_pct_s").value
        self._moving_rate = self.get_parameter("moving_drain_rate_pct_s").value
        self._speed_threshold = self.get_parameter("moving_speed_threshold_mps").value
        self._last_speed = 0.0

        self._odom_sub = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10)
        self._pub = self.create_publisher(
            BatteryState, self.get_parameter("battery_topic").value, 10)

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

    def _on_tick(self):
        rate = self._moving_rate if self._last_speed > self._speed_threshold \
            else self._idle_rate
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
