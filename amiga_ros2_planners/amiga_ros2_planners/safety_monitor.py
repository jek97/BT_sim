"""
safety_monitor.py

An ALWAYS-ON safety cutoff shared by every DURATIVE action node
(move_to_node.py, move_to_openloop_node.py, tool_action_node.py) --
deliberately SEPARATE from and independent of each action's own
`triggers` Goal field (move_to_node.py's TRIGGER_FUNCTOR_TO_CONDITION
and friends): that mechanism only checks whatever a mission's own tree
happens to list, via a service call to condition_service_node.
SafetyMonitor instead subscribes DIRECTLY to battery_sim_node's own
`battery_state` topic and the four bridged chassis_contact_<side>
topics (ros_gz_interfaces/msg/Contacts, genuine Gazebo physics contact
-- see condition_service_node.py's own CollisionDetected docstring for
why this is the real collision signal, not an approximation), with no
dependency on condition_service_node running at all, and applies to
EVERY durative action unconditionally, regardless of what that
mission's tree/Goal.triggers says.

Every durative action's own poll loop calls `tripped()` once per
iteration, alongside (not instead of) whatever `triggers` it already
checks -- the first non-None reason it returns ("battery_depleted" or
"collision_detected") is meant to be treated exactly like a fired
trigger: cancel/stop whatever this action is doing and report action-
level failure (result.status=False, result.reason=that string), the
same "stop and report failure" contract every other early-exit reason
already uses in these nodes.
"""
import rclpy
from sensor_msgs.msg import BatteryState
from ros_gz_interfaces.msg import Contacts

_CONTACT_SIDES = ("front", "back", "left", "right")


class SafetyMonitor:
    def __init__(self, node, battery_topic="battery_state",
                 battery_depleted_threshold_pct=0.0,
                 contact_topic_front="chassis/contact_front",
                 contact_topic_back="chassis/contact_back",
                 contact_topic_left="chassis/contact_left",
                 contact_topic_right="chassis/contact_right",
                 contact_stale_after_s=0.5):
        self._node = node
        self._threshold = battery_depleted_threshold_pct
        self._stale_after_s = contact_stale_after_s
        # 100.0, not 0.0: battery_sim_node's own first BatteryState hasn't
        # necessarily arrived yet when an action starts polling this --
        # assuming "full" until told otherwise is the same "don't trip a
        # safety cutoff on missing data" default PoseProvider.get_xy()
        # makes by returning None rather than (0, 0).
        self._battery_percent = 100.0
        self._last_contact_time = {side: None for side in _CONTACT_SIDES}

        node.create_subscription(BatteryState, battery_topic, self._on_battery, 10)
        contact_topics = {
            "front": contact_topic_front,
            "back": contact_topic_back,
            "left": contact_topic_left,
            "right": contact_topic_right,
        }
        for side in _CONTACT_SIDES:
            node.create_subscription(
                Contacts, contact_topics[side],
                (lambda msg, side=side: self._on_contact(side, msg)), 10)

    def _on_battery(self, msg):
        self._battery_percent = msg.percentage * 100.0

    def _on_contact(self, side, msg):
        if msg.contacts:
            self._last_contact_time[side] = self._node.get_clock().now()

    def battery_depleted(self):
        return self._battery_percent <= self._threshold

    def collision_detected(self):
        stale_after = self._node.get_clock().now() - rclpy.duration.Duration(
            seconds=self._stale_after_s)
        return any(
            t is not None and t > stale_after
            for t in self._last_contact_time.values())

    def tripped(self):
        """The first reason this action should stop right now, or None --
        battery checked first since a depleted battery is generally the
        more actionable/expected of the two."""
        if self.battery_depleted():
            return "battery_depleted"
        if self.collision_detected():
            return "collision_detected"
        return None
