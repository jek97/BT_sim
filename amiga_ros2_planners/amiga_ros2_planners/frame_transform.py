"""
frame_transform.py

Maps a goal point authored in a problog_project problem's OWN map frame
(that problem's map.yaml origin -- an arbitrary local frame with no
relationship to this simulation's orchard/tf2 frame at all) into this
simulation's live frame, so a `goal="11.675;11.525"` literal copied
straight out of a problog_project behavior_tree.xml lands on the
correct physical point instead of "wherever that number happens to be
in this sim's own frame".

A plain 2D rigid transform (translate + rotate) rather than a tf2
lookup: both frames are static for the lifetime of a run (no robot ever
moves "the problog map"), so there is nothing dynamic a tf2 buffer would
buy here -- just one fixed offset + rotation, calibrated once by
whoever aligns the two maps (see this package's README's own "Running a
problog_project BT" section for how to pick these values: typically by
locating one shared physical landmark, or a robot start pose, in both
frames and solving for the offset/rotation between them).

Identity (origin (0,0), yaw 0) by default -- i.e. "the tree's own goal
points are already expressed in this sim's frame", which is the
existing behavior for a mission authored directly against this
simulation's own orchard (as opposed to a `problog_project` mission
being replayed here).
"""
import math


class ProblogFrameTransform:
    def __init__(self, origin_x=0.0, origin_y=0.0, yaw_deg=0.0):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self._cos_yaw = math.cos(math.radians(yaw_deg))
        self._sin_yaw = math.sin(math.radians(yaw_deg))

    def to_sim_frame(self, x, y):
        """(x, y) in the problog map's own frame -> (x, y) in this
        simulation's frame: rotate by yaw, then translate by
        (origin_x, origin_y) -- the standard "where does this problog
        frame's origin sit, and how is it rotated, in MY frame" rigid
        transform."""
        rotated_x = x * self._cos_yaw - y * self._sin_yaw
        rotated_y = x * self._sin_yaw + y * self._cos_yaw
        return self.origin_x + rotated_x, self.origin_y + rotated_y
