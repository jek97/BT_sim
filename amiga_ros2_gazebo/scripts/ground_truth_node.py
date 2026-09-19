#!/usr/bin/env python3
"""Republish the robot's own exact simulated pose as a proper
geometry_msgs/PoseStamped -- sourced from Ignition's own SceneBroadcaster
system (every world loads this by default), which streams EVERY dynamic
entity's world-frame pose, physics-engine-exact, no sensor model, no
noise, no EKF filtering, on `/world/<world>/dynamic_pose/info`
(ignition.msgs.Pose_V). gazebo.launch.py's own robot_bridge_args bridges
that (world-level, once, not per-robot) to ROS as tf2_msgs/msg/TFMessage
via ros_gz_bridge's own Pose_V<->TFMessage conversion, onto this node's
own `input_topic` -- each entry's `child_frame_id` is that entity's own
name (this project's own smoke test: `child_frame_id: <model name>` for
the model's own root pose, a SEPARATE, additional entry per link, e.g.
`child_frame_id: chassis`). This node filters that stream down to the
ONE entry matching `entity_name` (this robot's own spawned model name)
and republishes just that as a properly-stamped PoseStamped.

NOT sourced from Ignition's PosePublisher system plugin (amiga_kinova/
model.sdf's own "GROUND TRUTH" comment used to describe that path) --
CONFIRMED BROKEN in this project's own ignition-gazebo6: with every
publish_*_pose flag false (as this project's config had them, wanting
"the model's own pose only"), the plugin advertises its topic but never
calls Publish() at all -- a live smoke test (a single-link test model,
same plugin block, isolated minimal world) reproduced zero messages
over 6+ seconds despite update_frequency=30, and only started
publishing once publish_link_pose was set true -- but that dumps EVERY
link's pose (wheels, arm joints, ~20 on this robot) onto the SAME
geometry_msgs/msg/Pose topic with no name field to filter by (Pose has
no header), an unusable mix for "this robot's own pose". dynamic_pose/
info sidesteps both problems: it already works (SceneBroadcaster is not
opt-in), and TFMessage's child_frame_id is exactly the name field this
node needs to pick the right entry out of a multi-entity stream.

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

ALSO publishes a static, one-shot `frame_id`->`odom` identity transform
(when `publish_tf` is true) -- nav2_params_sim.yaml's own local_costmap
still has global_frame: odom (the standard Nav2 split: local_costmap
tracks the driftless-but-jump-free odom frame, global_costmap tracks
map), and with the EKF stack off, nothing else in this "simplified"
branch ever publishes an "odom" frame at all -- local_costmap's own
`canTransform(base_link, odom)` would otherwise time out forever and
Nav2 could never activate. Ground truth has no drift to model, so
odom == map (identity) is exactly correct, not a placeholder: tf2
resolves base_link->odom by composing this static map->odom edge with
the dynamic map->base_link one above, through their common "map"
ancestor -- no second, competing authority over base_link itself.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class GroundTruthNode(Node):
    def __init__(self):
        super().__init__("ground_truth_node")
        self.declare_parameter("input_topic", "dynamic_pose/info")
        self.declare_parameter("entity_name", "amiga_kinova")
        self.declare_parameter("output_topic", "ground_truth/pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("publish_tf", True)

        self.entity_name = self.get_parameter("entity_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.pub = self.create_publisher(
            PoseStamped, self.get_parameter("output_topic").value, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        if self.publish_tf:
            odom_frame_id = self.get_parameter("odom_frame_id").value
            static_tf = TransformStamped()
            static_tf.header.stamp = self.get_clock().now().to_msg()
            static_tf.header.frame_id = self.frame_id
            static_tf.child_frame_id = odom_frame_id
            static_tf.transform.rotation.w = 1.0
            self._static_tf_broadcaster = StaticTransformBroadcaster(self)
            self._static_tf_broadcaster.sendTransform(static_tf)
        self.sub = self.create_subscription(
            TFMessage, self.get_parameter("input_topic").value, self.cb, 10
        )

    def cb(self, msg: TFMessage):
        # dynamic_pose/info carries every dynamic entity in the world (this
        # robot's own root pose AND a separate entry per link) in one
        # shared stream -- pick out only the one entry this node owns.
        entry = next(
            (t for t in msg.transforms if t.child_frame_id == self.entity_name), None)
        if entry is None:
            return
        now = self.get_clock().now().to_msg()
        translation = entry.transform.translation

        pose = Pose()
        pose.position.x = translation.x
        pose.position.y = translation.y
        pose.position.z = translation.z
        pose.orientation = entry.transform.rotation

        stamped = PoseStamped()
        stamped.header.stamp = now
        stamped.header.frame_id = self.frame_id
        stamped.pose = pose
        self.pub.publish(stamped)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id
            t.transform = entry.transform
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(GroundTruthNode())


if __name__ == "__main__":
    main()
