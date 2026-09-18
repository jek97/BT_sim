#!/usr/bin/env python3
"""Republish the robot's own exact simulated pose as a proper
geometry_msgs/PoseStamped -- Ignition's own PosePublisher system
plugin (amiga_kinova/model.sdf's own "GROUND TRUTH" comment) publishes
this model's world-frame pose directly from the physics engine's own
state, no sensor model, no noise, no EKF filtering -- gazebo.launch.py's
own robot_bridge_args bridges that raw ignition::msgs::Pose into this
node's own `input_topic` as a bare geometry_msgs/msg/Pose (no header),
and this node just adds the stamp/frame_id a Pose is missing.

frame_id defaults to "map": this project's own established convention
throughout amiga_ros2_planners (see problog_problem.compute_frame_origin's
own docstring) is that the robot spawns at Gazebo's own world origin,
which IS this simulation's own map-frame origin -- i.e. Gazebo's world
frame and this sim's ROS "map" frame are the SAME frame by construction,
not two frames that happen to coincide by luck. This is the same
assumption every other position-reading node in this repo already
relies on; this node doesn't introduce a new one.

Purely diagnostic/comparison: nothing in amiga_ros2_planners reads this
topic today -- every planner/condition still reads the EKF-fused
map->base_link tf2 transform exactly as before this existed. This
topic exists so the two (EKF estimate vs. ground truth) CAN be
compared directly (e.g. logging/plotting localization error), not to
replace tf2 anywhere. Same "raw bridged Gazebo message -> properly-
shaped ROS message" shape as sim_gps_shim.py/sim_imu_shim.py.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped


class GroundTruthNode(Node):
    def __init__(self):
        super().__init__("ground_truth_node")
        self.declare_parameter("input_topic", "/model/amiga_kinova/pose")
        self.declare_parameter("output_topic", "ground_truth/pose")
        self.declare_parameter("frame_id", "map")

        self.frame_id = self.get_parameter("frame_id").value
        self.pub = self.create_publisher(
            PoseStamped, self.get_parameter("output_topic").value, 10
        )
        self.sub = self.create_subscription(
            Pose, self.get_parameter("input_topic").value, self.cb, 10
        )

    def cb(self, msg: Pose):
        stamped = PoseStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.pose = msg
        self.pub.publish(stamped)


def main():
    rclpy.init()
    rclpy.spin(GroundTruthNode())


if __name__ == "__main__":
    main()
