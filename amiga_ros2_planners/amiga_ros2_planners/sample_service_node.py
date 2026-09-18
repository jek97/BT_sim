#!/usr/bin/env python3
"""
sample_service_node.py

Hosts TakeSample (amiga_interfaces/srv/TakeSample) -- the ROS2-service
form of problog_project's TakeSample BT node (see
module/contracts/schema.yaml's own TakeSample entry). INSTANTANEOUS,
unlike InstallTool/UninstallTool/MoveTo: a single fixed-probability
coin flip, no Duration/Triggers, no continuous trajectory -- exactly
problog_project/module/contracts/bt_actions.py's own bt_take_sample
reference implementation (`random.random() < success_probability`),
reused here as the literal formula, not reimplemented.

success_probability is a ROS param (this problem's own config.yaml,
sample.success_probability -- see problog_problem.sample_params),
same "loaded once at startup, not per-call" convention every other
problog_problem-derived param in this package already uses.

VALUE DRAW (schema.yaml's tool-instance-id-refactor-era addition): on a
SUCCESSFUL sample, a second, independent draw produces a VALUE, shaped
ONE OF TWO WAYS by this problem's own config.yaml (problog_problem.
sample_params, threaded in as this node's own value_discretized/
value_mean/value_sigma params):

  - value_discretized (config.yaml's sample.value.discretized: a list
    of {value, weight} outcomes): a weighted-random pick among exactly
    those values (random.choices, which -- unlike config_to_prolog.py's
    own _explicit_discrete_block -- normalizes the weights itself, so
    they need not sum to exactly 1.0 here). Takes priority if
    non-empty.
  - value_mean/value_sigma (config.yaml's sample.value.mean/sigma,
    defaulting to 5.0/2.0) -- a discretized Normal(mean,sigma) over the
    integers 0-10, used whenever value_discretized is empty. Same
    distribution config_to_prolog.py's own _discretized_normal_block
    computes exactly (that function bins a continuous Normal(mean,sigma)
    into integers 0-10 via the real normal CDF, with the two boundary
    bins absorbing their own outer tail; round(gauss(mean,sigma))
    clipped to [0,10] is the identical distribution -- clipping IS the
    boundary-bin absorption, just done by draw-then-clip instead of by
    pre-computing 11 weights). The ORIGINAL, still-default shape.

Every id's own drawn value is kept in memory AND republished in full on
a latched (TRANSIENT_LOCAL) `sample_values` topic (a JSON object,
id -> value, growing one key at a time) every time a new one is drawn --
condition_service_node.py subscribes to this to answer
SampleValueBelow/Equal/Over without this node needing to expose a
service of its own for that (same "one node owns the state, publishes
it latched, everyone else just subscribes" shape tool_action_node.py's
own tool_state topic already uses for the equipped-tool state).
"""
import json
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

from amiga_interfaces.srv import TakeSample
from amiga_ros2_planners.safety_monitor import SafetyMonitor

_LATCHED_QOS = QoSProfile(depth=1)
_LATCHED_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL
_LATCHED_QOS.reliability = ReliabilityPolicy.RELIABLE


class SampleServiceNode(Node):
    def __init__(self):
        super().__init__("sample_service_node")

        self.declare_parameter("success_probability", 0.5)
        self.declare_parameter("value_mean", 5.0)
        self.declare_parameter("value_sigma", 2.0)
        # JSON-encoded [{"value": v, "weight": w}, ...] -- config.yaml's
        # sample.value.discretized (problog_problem.sample_params),
        # same "variable-length list can't be a plain launch arg"
        # reasoning tool_instances already uses. "[]" (the default)
        # means "not configured" -- falls back to value_mean/value_sigma.
        self.declare_parameter("value_discretized", "[]")
        self.declare_parameter("sample_values_topic", "sample_values")

        # TakeSample is INSTANTANEOUS (see this module's own docstring) --
        # there is no mid-execution to stop, so unlike the durative
        # actions (move_to_node.py/move_to_openloop_node.py/
        # tool_action_node.py, which check SafetyMonitor.tripped() on
        # every poll of an ongoing action) this is a one-shot precondition
        # check: refuse the whole call up front if either is already true
        # at the moment it's requested. See safety_monitor.py's own
        # module docstring.
        self.declare_parameter("battery_topic", "battery_state")
        self.declare_parameter("battery_depleted_threshold_pct", 0.0)
        self.declare_parameter("contact_topic_front", "chassis/contact_front")
        self.declare_parameter("contact_topic_back", "chassis/contact_back")
        self.declare_parameter("contact_topic_left", "chassis/contact_left")
        self.declare_parameter("contact_topic_right", "chassis/contact_right")
        self.declare_parameter("contact_stale_after_s", 0.5)
        self._safety = SafetyMonitor(
            self,
            battery_topic=self.get_parameter("battery_topic").value,
            battery_depleted_threshold_pct=self.get_parameter(
                "battery_depleted_threshold_pct").value,
            contact_topic_front=self.get_parameter("contact_topic_front").value,
            contact_topic_back=self.get_parameter("contact_topic_back").value,
            contact_topic_left=self.get_parameter("contact_topic_left").value,
            contact_topic_right=self.get_parameter("contact_topic_right").value,
            contact_stale_after_s=self.get_parameter("contact_stale_after_s").value,
        )

        self._value_discretized = json.loads(self.get_parameter("value_discretized").value)

        # In-memory only -- this node's own lifetime IS the mission's
        # lifetime, same assumption tool_action_node.py's own
        # _equipped_tool makes.
        self._sample_values = {}

        self._sample_values_pub = self.create_publisher(
            String, self.get_parameter("sample_values_topic").value, _LATCHED_QOS)
        # Publish the (empty) initial state immediately -- a late-joining
        # condition_service_node still gets a well-formed "{}" via
        # TRANSIENT_LOCAL even if it subscribes before any sample is
        # ever taken.
        self._publish_sample_values()

        self._srv = self.create_service(TakeSample, "take_sample", self._on_request)
        self.get_logger().info("sample_service_node ready on 'take_sample'")

    def _publish_sample_values(self):
        self._sample_values_pub.publish(String(data=json.dumps(self._sample_values)))

    def _draw_value(self):
        """See this module's own docstring for the two ways
        value_discretized/value_mean/value_sigma can shape this draw."""
        if self._value_discretized:
            values = [entry["value"] for entry in self._value_discretized]
            weights = [entry["weight"] for entry in self._value_discretized]
            return float(random.choices(values, weights=weights, k=1)[0])
        mean = self.get_parameter("value_mean").value
        sigma = self.get_parameter("value_sigma").value
        value = int(round(random.gauss(mean, sigma)))
        return float(max(0, min(10, value)))

    def _on_request(self, request, response):
        safety_reason = self._safety.tripped()
        if safety_reason is not None:
            response.reason = safety_reason
            response.status = False
            response.value = 0.0
            return response

        success_probability = self.get_parameter("success_probability").value
        success = random.random() < success_probability
        response.reason = "sample_success" if success else "sample_failure"
        response.status = success

        if success:
            value = self._draw_value()
            response.value = value
            self._sample_values[request.id] = value
            self._publish_sample_values()
            self.get_logger().info(
                f"TakeSample: id='{request.id}' succeeded, value={value}")
        else:
            response.value = 0.0
            self.get_logger().info(f"TakeSample: id='{request.id}' failed")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SampleServiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
