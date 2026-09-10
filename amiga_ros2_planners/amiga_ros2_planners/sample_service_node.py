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
"""
import random

import rclpy
from rclpy.node import Node

from amiga_interfaces.srv import TakeSample


class SampleServiceNode(Node):
    def __init__(self):
        super().__init__("sample_service_node")

        self.declare_parameter("success_probability", 0.5)

        self._srv = self.create_service(TakeSample, "take_sample", self._on_request)
        self.get_logger().info("sample_service_node ready on 'take_sample'")

    def _on_request(self, _request, response):
        success_probability = self.get_parameter("success_probability").value
        success = random.random() < success_probability
        response.reason = "sample_success" if success else "sample_failure"
        response.status = success
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
