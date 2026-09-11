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
SUCCESSFUL sample, a second, independent draw produces a VALUE, 0-10 --
a discretized Normal(mean,sigma) (config.yaml's sample.value.mean/
sigma, defaulting to 5.0/2.0), same distribution
config_to_prolog.py's own _discretized_normal_block computes exactly
(that function bins a continuous Normal(mean,sigma) into integers 0-10
via the real normal CDF, with the two boundary bins absorbing their own
outer tail; round(gauss(mean,sigma)) clipped to [0,10] is the identical
distribution -- clipping IS the boundary-bin absorption, just done by
draw-then-clip instead of by pre-computing 11 weights).

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

_LATCHED_QOS = QoSProfile(depth=1)
_LATCHED_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL
_LATCHED_QOS.reliability = ReliabilityPolicy.RELIABLE


class SampleServiceNode(Node):
    def __init__(self):
        super().__init__("sample_service_node")

        self.declare_parameter("success_probability", 0.5)
        self.declare_parameter("value_mean", 5.0)
        self.declare_parameter("value_sigma", 2.0)
        self.declare_parameter("sample_values_topic", "sample_values")

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

    def _on_request(self, request, response):
        success_probability = self.get_parameter("success_probability").value
        success = random.random() < success_probability
        response.reason = "sample_success" if success else "sample_failure"
        response.status = success

        if success:
            mean = self.get_parameter("value_mean").value
            sigma = self.get_parameter("value_sigma").value
            value = int(round(random.gauss(mean, sigma)))
            value = max(0, min(10, value))
            response.value = value
            self._sample_values[request.id] = value
            self._publish_sample_values()
            self.get_logger().info(
                f"TakeSample: id='{request.id}' succeeded, value={value}")
        else:
            response.value = 0
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
