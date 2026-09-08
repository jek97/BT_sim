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
