"""
pose.py

The robot's current (x,y) -- the live-simulation replacement for
problog_project's own now/2 + at/4 fluent lookup (there, "current
position" is just the latest entry in a Prolog situation history).

Two sources, picked per-instance via `pose_topic`:

* `pose_topic` set (the "simplified" branch's own default for every
  reference_frame-based PoseProvider -- see move_to_node.py/
  plan_service_node.py/condition_service_node.py/tool_action_node.py,
  all of which declare a `pose_topic` param defaulting to
  "ground_truth/pose"): subscribes a `geometry_msgs/PoseStamped` topic
  and reports its latest (x, y) directly -- amiga_ros2_gazebo's
  `ground_truth_node.py` publishes exactly that, straight from
  Ignition's own PosePublisher system plugin, no EKF/tf2 in the loop
  at all. `reference_frame`/`base_frame` are then unused (the topic's
  own frame_id, "map", already matches this project's convention that
  Gazebo's world origin IS the sim's map-frame origin).
* `pose_topic` left None: the original tf2 `reference_frame`-
  >`base_frame` lookup -- still used by move_to_node.py's own
  `_odom_pose` (gates sending a FollowPath goal on odom->base_link
  existing, which has nothing to do with map-frame localization and so
  is untouched by the ground-truth switch).

`reference_frame`/`base_frame` default to "map"/"base_link" (Nav2's own
convention), each declared as a ROS param by the node that owns a
PoseProvider so a namespaced robot can override them exactly like every
other frame-name parameter in this repo (see
amiga_ros2_gazebo/launch/sim_bringup.launch.py's own frame_prefix note).
"""
import rclpy
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped


class PoseProvider:
    def __init__(self, node, reference_frame="map", base_frame="base_link",
                 pose_topic=None):
        self._node = node
        self._reference_frame = reference_frame
        self._base_frame = base_frame
        self._xy = None
        if pose_topic:
            self._buffer = None
            self._listener = None
            self._sub = node.create_subscription(
                PoseStamped, pose_topic, self._on_pose, 10)
        else:
            self._buffer = Buffer()
            self._listener = TransformListener(self._buffer, node)

    def _on_pose(self, msg: PoseStamped):
        self._xy = (msg.pose.position.x, msg.pose.position.y)

    def get_xy(self):
        """(x, y) in `reference_frame`, or None if no pose is available
        yet (e.g. localization/ground truth hasn't published one)."""
        if self._buffer is None:
            return self._xy
        try:
            transform = self._buffer.lookup_transform(
                self._reference_frame, self._base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = transform.transform.translation
        return t.x, t.y

    def wait_ready(self, timeout_sec=30.0):
        """Spin this provider's own node until get_xy() first succeeds,
        or `timeout_sec` elapses (returns whether it became ready).

        Startup-ordering fix: a mission can arrive and start ticking
        (run_problog_problem.launch.py's own send_mission retries
        independently of the rest of bringup) well before Gazebo's
        diff_drive_controller/localization stack has published the
        first base_frame->reference_frame transform. Call this BEFORE
        advertising a service/action that answers from get_xy() so
        BT.cpp's RosServiceNode/RosActionNode sees "service not yet
        available" (which it retries as RUNNING) instead of a
        one-shot FAILURE response with reason="no_pose" that would
        permanently fail a plain (non-reactive) Sequence/Fallback."""
        deadline = self._node.get_clock().now() + rclpy.duration.Duration(
            seconds=timeout_sec)
        while self.get_xy() is None:
            if self._node.get_clock().now() >= deadline:
                return False
            rclpy.spin_once(self._node, timeout_sec=0.1)
        return True
