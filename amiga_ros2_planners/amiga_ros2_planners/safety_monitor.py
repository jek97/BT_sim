"""
safety_monitor.py

An ALWAYS-ON safety cutoff shared by every DURATIVE action node
(move_to_node.py, tool_action_node.py) --
deliberately SEPARATE from and independent of each action's own
`triggers` Goal field (move_to_node.py's TRIGGER_FUNCTOR_TO_CONDITION
and friends): that mechanism only checks whatever a mission's own tree
happens to list, via a service call to condition_service_node.
SafetyMonitor instead subscribes DIRECTLY to battery_sim_node's own
`battery_state` topic and every bridged contact-sensor topic in
DEFAULT_CONTACT_TOPICS below (ros_gz_interfaces/msg/Contacts, genuine
Gazebo physics contact -- see condition_service_node.py's own
CollisionDetected docstring for why this is the real collision signal,
not an approximation), with no dependency on condition_service_node
running at all, and applies to EVERY durative action unconditionally,
regardless of what that mission's tree/Goal.triggers says.

Every durative action's own poll loop calls `tripped()` once per
iteration, alongside (not instead of) whatever `triggers` it already
checks -- the first non-None reason it returns ("battery_depleted" or
"collision_detected") is meant to be treated exactly like a fired
trigger: cancel/stop whatever this action is doing and report action-
level failure (result.status=False, result.reason=that string), the
same "stop and report failure" contract every other early-exit reason
already uses in these nodes.

Which topic a contact happened to arrive on is never used for
anything here (unlike condition_service_node.py's own CollisionDetected
condition, which supports querying one side specifically) -- tripped()
only ever needs "did ANY of them see contact recently", so this takes a
plain list of topics rather than four named sides, letting the set of
watched collisions (chassis bumpers, wheels, anything added later)
change without this class's own API changing too.
"""
import rclpy
from sensor_msgs.msg import BatteryState
from ros_gz_interfaces.msg import Contacts

# model.sdf's own contact sensors: 4 chassis bumpers (each flush
# against one face of chassis_link_collision, plus that box itself on
# the front one -- see that sensor's own comment) and 4 wheels (which
# sit lower, and can be wider, than the chassis collision box -- see
# model.sdf's own wheel sensor comments for why they need separate
# coverage). Single source of truth for every node that constructs a
# SafetyMonitor with its own contact_topics parameter's default value.
DEFAULT_CONTACT_TOPICS = (
    "chassis/contact_front", "chassis/contact_back",
    "chassis/contact_left", "chassis/contact_right",
    "wheel/contact_front_left", "wheel/contact_front_right",
    "wheel/contact_rear_left", "wheel/contact_rear_right",
)

# Every world file's own ground plane is a model literally named
# "ground_plane" (orchard_map_a.sdf and friends) -- a wheel resting on
# it is the normal, constant, expected state of a standing robot, not
# a collision, but the wheel/contact_* sensors above have no way to
# exclude it at the SDF level (a contact sensor's own <contact>
# element only lists which of ITS OWN collisions to watch, not which
# of the OTHER body's). Filtered out here instead: a Contacts message
# is only treated as a real hit if at least one of its own contact
# pairs involves something OTHER than ground_plane.
_GROUND_MODEL_NAME = "ground_plane"


def is_real_contact(msg):
    return any(
        _GROUND_MODEL_NAME not in c.collision1.name
        and _GROUND_MODEL_NAME not in c.collision2.name
        for c in msg.contacts)


class SafetyMonitor:
    def __init__(self, node, battery_topic="battery_state",
                 battery_depleted_threshold_pct=0.0,
                 contact_topics=DEFAULT_CONTACT_TOPICS,
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
        self._last_contact_time = {topic: None for topic in contact_topics}

        node.create_subscription(BatteryState, battery_topic, self._on_battery, 10)
        for topic in contact_topics:
            node.create_subscription(
                Contacts, topic,
                (lambda msg, topic=topic: self._on_contact(topic, msg)), 10)

    def _on_battery(self, msg):
        self._battery_percent = msg.percentage * 100.0

    def _on_contact(self, topic, msg):
        if msg.contacts and is_real_contact(msg):
            self._last_contact_time[topic] = self._node.get_clock().now()

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
