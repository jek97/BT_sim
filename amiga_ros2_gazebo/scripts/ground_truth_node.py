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

Publishes BOTH a topic and a tf, on purpose: `output_topic` (a plain
geometry_msgs/PoseStamped, read directly by amiga_ros2_planners' own
pose.py -- see its pose_topic param) AND, when `publish_tf` is true, a
`frame_id`->`child_frame_id` (map->base_link by default) tf2 transform,
so ANY consumer that instead reads position off tf2 -- the same
map->base_link lookup the EKF stack's own map->odom->base_link chain
answers -- gets ground truth too, with no code change on that
consumer's side. The two pipelines are meant to be mutually exclusive
tf authorities for base_link: gazebo.launch.py ties `publish_tf` to
sim_bringup.launch.py's own `launch_localization` being false (see
that file's own publish_ground_truth_tf forwarding), since
amiga_localization's EKF is the other publisher of a map-rooted
transform onto base_link and ros2_controllers_sim.yaml's own
diff_drive_controller already has enable_odom_tf: false specifically
so only one stack ever owns that edge at a time -- turning
launch_localization back on and this back off is meant to be a clean
swap, not two authorities fighting over the same frame.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class GroundTruthNode(Node):
    def __init__(self):
        super().__init__("ground_truth_node")
        self.declare_parameter("input_topic", "/model/amiga_kinova/pose")
        self.declare_parameter("output_topic", "ground_truth/pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)

        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.pub = self.create_publisher(
            PoseStamped, self.get_parameter("output_topic").value, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.sub = self.create_subscription(
            Pose, self.get_parameter("input_topic").value, self.cb, 10
        )

    def cb(self, msg: Pose):
        now = self.get_clock().now().to_msg()

        stamped = PoseStamped()
        stamped.header.stamp = now
        stamped.header.frame_id = self.frame_id
        stamped.pose = msg
        self.pub.publish(stamped)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = msg.position.x
            t.transform.translation.y = msg.position.y
            t.transform.translation.z = msg.position.z
            t.transform.rotation = msg.orientation
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(GroundTruthNode())


if __name__ == "__main__":
    main()
