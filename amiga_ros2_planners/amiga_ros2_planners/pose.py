"""
pose.py

The robot's current (x,y) via tf2 -- the live-simulation replacement
for problog_project's own now/2 + at/4 fluent lookup (there, "current
position" is just the latest entry in a Prolog situation history;
here, it is whatever this robot's own localization stack has published
to tf2, same convention as every other node in this repo that needs a
live pose -- e.g. amiga_navigation's lidar_object_navigator, which reads
`base_frame`/`lidar_link` off tf2 the same way).

`reference_frame`/`base_frame` default to "map"/"base_link" (Nav2's own
convention), each declared as a ROS param by the node that owns a
PoseProvider so a namespaced robot can override them exactly like every
other frame-name parameter in this repo (see
amiga_ros2_gazebo/launch/sim_bringup.launch.py's own frame_prefix note).
"""
import rclpy
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException


class PoseProvider:
    def __init__(self, node, reference_frame="map", base_frame="base_link"):
        self._node = node
        self._reference_frame = reference_frame
        self._base_frame = base_frame
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, node)

    def get_xy(self):
        """(x, y) in `reference_frame`, or None if tf2 has no
        transform yet (e.g. localization hasn't published one)."""
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
