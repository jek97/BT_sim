#!/usr/bin/env python3
"""Drive `trajectory_camera`'s pose in Gazebo, every tick, via the world's
own /world/<world_name>/set_pose service (ros_gz_interfaces/srv/
SetEntityPose) -- bridged in gazebo.launch.py. That service is exposed by
every Ignition world through the always-loaded
ignition-gazebo-user-commands-system plugin (the same one the GUI's own
transform-control drag tool calls), and teleports a named entity's pose
directly, with no joints, controllers, or physics involved -- confirmed
live: calling it against a static model moves it instantly and the
model's own camera sensor reflects the new pose on the very next frame.

Modes (select with sim_bringup.launch.py's own `visual` argument, which
IS the mode name -- "false" means no camera at all):
  * "third_person" (default): offset behind/above whichever robot
    publishes on `target_pose_topic`, always turned to look at it. A
    fixed offset in the robot's OWN frame, so it reads as "chasing" the
    robot from over its shoulder.
  * "orbit": circles the robot in the horizontal plane, at a similar
    height but a bit further out than third_person, always turned to
    look at the robot's center. The orbit's own center re-tracks the
    robot's (smoothed) position every tick, so it stays centered on a
    moving robot rather than orbiting a fixed point.
  * "custom": scripted, loaded from `waypoints_file` (a YAML file -- see
    _load_custom_waypoints's own docstring). Two dialects:
      - flat `keyframes`: interpolates through a list of (t, x, y, z,
        yaw, pitch) points -- config/example_camera_waypoints.yaml is a
        worked example.
      - `phases`: an ordered list of phases -- "orbit" (identical to
        orbit mode, but can end on a named BT event instead of running
        forever), "climb" (rises straight up while staying centered on
        and looking at the tracked robot), and "flythrough" (a timed,
        continuous move to a fixed pose, starting from wherever the
        camera already is -- no jump). config/
        example_orbit_then_overhead.yaml and config/
        example_orbit_then_fence_overhead.yaml are worked examples: orbit
        the robot until it takes its first sample, climb while staying
        centered on it, then fly the rest of the way to a fixed overhead
        shot (of the whole map, or of the whole fenced perimeter,
        respectively). See _tick_phased's own docstring for the phase
        schema.

Adding a new mode is: a `_tick_<mode>` method following the same
pattern, a branch in `_tick`, and a line in gazebo.launch.py's own
`visual` argument description. Adding a new *phase type* (within
"custom") is a `_phase_pose_<type>` method and a branch in
_tick_phased.

Orientation convention: this node always commands roll=0 and sends
(pitch, yaw) through tf_transformations.quaternion_from_euler(0, pitch,
yaw) ('sxyz' axes, the tf_transformations default). Empirically verified
against a live camera sensor: with that quaternion and identity applied,
the sensor looks along its own local +X axis, so yaw is the standard
atan2(dy, dx) heading and a POSITIVE pitch tilts the view DOWN (derived
from the same 'sxyz' rotation composition, then confirmed against a
marker in frame) -- see _look_at's own comment for the exact formula.
"""
import json
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion, quaternion_from_euler


def _look_at(cam_xyz, target_xyz):
    """(yaw, pitch) so the camera's local +X axis points at target_xyz.

    pitch = atan2(cz - tz, horizontal_distance): positive pitch tilts the
    view down, matching a camera ABOVE its target (cz > tz) needing to
    look down -- see this module's own docstring for how that sign was
    derived and verified.
    """
    cx, cy, cz = cam_xyz
    tx, ty, tz = target_xyz
    dx, dy, dz = tx - cx, ty - cy, tz - cz
    yaw = math.atan2(dy, dx)
    horiz = math.hypot(dx, dy)
    pitch = math.atan2(-dz, horiz)
    return yaw, pitch


def _smooth_towards(current, target, alpha):
    """In-place exponential smoothing of a [x, y, z] list towards target.
    alpha=0 freezes current in place, alpha=1 snaps instantly -- shared by
    third_person (smooths the camera's own position) and orbit (smooths
    the orbit's tracked center), so both take the edge off the robot's
    own small odometry jitter the same way.
    """
    for i in range(3):
        current[i] += alpha * (target[i] - current[i])


def _lerp_angle(a, b, frac):
    """Interpolate a to b the SHORT way around the circle (never more than
    +-pi), so a flythrough phase's yaw/pitch never spins the long way
    around just because the orbit phase it continues from happened to
    end near +-pi.
    """
    delta = ((b - a + math.pi) % (2.0 * math.pi)) - math.pi
    return a + delta * frac


class CameraTrajectoryNode(Node):
    def __init__(self):
        super().__init__("camera_trajectory_node")
        self.declare_parameter("world_name", "orchard_nbv")
        self.declare_parameter("camera_name", "trajectory_camera")
        self.declare_parameter("mode", "third_person")
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("target_pose_topic", "ground_truth/pose")
        # Shared between third_person and orbit: how high above the
        # robot's own base_link origin to aim (roughly chassis height,
        # not straight down at the ground contact point), and how much
        # to smooth the tracked robot position each tick (0 = frozen,
        # 1 = snaps instantly to the raw target) to take the edge off its
        # own small odometry jitter.
        self.declare_parameter("look_at_height", 0.6)
        self.declare_parameter("position_smoothing", 0.15)

        # third_person mode: behind (-x) and above (+z) the robot, in the
        # robot's OWN frame.
        self.declare_parameter("third_person_offset", [-4.0, 0.0, 2.5])

        # orbit mode: a similar height to third_person's own offset but a
        # bit further out (radius > third_person_offset's horizontal
        # distance), one full revolution every orbit_period_s seconds.
        self.declare_parameter("orbit_radius", 7.0)
        self.declare_parameter("orbit_height", 3.0)
        # One revolution every 30s (slowed from an earlier 20s default to
        # 2/3 of that angular velocity, i.e. 1.5x the period).
        self.declare_parameter("orbit_period_s", 30.0)

        # custom mode
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("waypoints_loop", False)

        self.camera_name = self.get_parameter("camera_name").value
        self.mode = self.get_parameter("mode").value
        self.look_at_height = self.get_parameter("look_at_height").value
        self.position_smoothing = self.get_parameter("position_smoothing").value
        self.third_person_offset = tuple(self.get_parameter("third_person_offset").value)
        self.orbit_radius = self.get_parameter("orbit_radius").value
        self.orbit_height = self.get_parameter("orbit_height").value
        self.orbit_period_s = self.get_parameter("orbit_period_s").value

        world_name = self.get_parameter("world_name").value
        self._cli = self.create_client(SetEntityPose, f"/world/{world_name}/set_pose")
        self._pending = False

        self._target_pose = None
        self._tracked_pos = None  # smoothed robot position, shared by both tracking modes
        self._start_time = self.get_clock().now()
        self._keyframes = []
        self._waypoints_loop = self.get_parameter("waypoints_loop").value

        # "custom" mode, phases dialect (see _tick_phased's own docstring)
        self._phases = None
        self._phase_index = 0
        self._phase_start_time_s = 0.0
        self._phase_start_pose = None  # captured on entering a flythrough phase
        self._last_sent_pose = None  # updated on every _send_pose call, any mode
        self._bt_events_seen = set()  # {(node_name, status), ...}

        if self.mode in ("third_person", "orbit"):
            self.create_subscription(
                PoseStamped,
                self.get_parameter("target_pose_topic").value,
                self._on_target_pose,
                10,
            )
        elif self.mode == "custom":
            self._load_custom_waypoints(self.get_parameter("waypoints_file").value)
        else:
            self.get_logger().error(
                f"Unknown mode '{self.mode}' -- expected 'third_person', "
                "'orbit', or 'custom'. No pose commands will be sent."
            )

        rate = self.get_parameter("update_rate").value
        self.create_timer(1.0 / rate, self._tick)

    def _on_target_pose(self, msg: PoseStamped):
        self._target_pose = msg

    def _now_s(self):
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _tick(self):
        if self.mode == "third_person":
            self._tick_third_person()
        elif self.mode == "orbit":
            self._tick_orbit()
        elif self.mode == "custom":
            if self._phases is not None:
                self._tick_phased()
            else:
                self._tick_custom()

    def _tracked_robot_pose(self):
        """Smoothed (x, y, z, yaw) of the tracked robot, or None until the
        first pose arrives -- shared setup for third_person and orbit.
        """
        if self._target_pose is None:
            return None
        p = self._target_pose.pose.position
        q = self._target_pose.pose.orientation
        _, _, robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self._tracked_pos is None:
            self._tracked_pos = [p.x, p.y, p.z]
        else:
            _smooth_towards(self._tracked_pos, [p.x, p.y, p.z], self.position_smoothing)
        return (*self._tracked_pos, robot_yaw)

    def _tick_third_person(self):
        tracked = self._tracked_robot_pose()
        if tracked is None:
            return
        rx, ry, rz, robot_yaw = tracked

        ox, oy, oz = self.third_person_offset
        cos_y, sin_y = math.cos(robot_yaw), math.sin(robot_yaw)
        cam_pos = (
            rx + ox * cos_y - oy * sin_y,
            ry + ox * sin_y + oy * cos_y,
            rz + oz,
        )
        look_at = (rx, ry, rz + self.look_at_height)
        yaw, pitch = _look_at(cam_pos, look_at)
        self._send_pose(*cam_pos, yaw, pitch)

    def _tick_orbit(self):
        tracked = self._tracked_robot_pose()
        if tracked is None:
            return
        rx, ry, rz, _robot_yaw = tracked

        angle = 2.0 * math.pi * self._now_s() / self.orbit_period_s
        cam_pos = (
            rx + self.orbit_radius * math.cos(angle),
            ry + self.orbit_radius * math.sin(angle),
            rz + self.orbit_height,
        )
        look_at = (rx, ry, rz + self.look_at_height)
        yaw, pitch = _look_at(cam_pos, look_at)
        self._send_pose(*cam_pos, yaw, pitch)

    def _load_custom_waypoints(self, path: str):
        """`waypoints_file` is a YAML file in one of two dialects.

        Flat `keyframes` (see config/example_camera_waypoints.yaml):

            loop: false
            keyframes:
              - {t: 0.0,  x: 0.0, y: 0.0, z: 5.0, yaw: 0.0,    pitch: 0.3}
              - {t: 10.0, x: 8.0, y: 0.0, z: 5.0, yaw: 1.57,   pitch: 0.3}

        `t` is seconds since this node started, strictly increasing. The
        camera holds at the first keyframe's pose before t=0 and the last
        keyframe's pose after the end, unless `loop` (or the
        waypoints_loop parameter) is set, in which case time wraps at the
        last keyframe's `t`.

        `phases` (see _tick_phased's own docstring for the schema and
        config/example_orbit_then_overhead.yaml for a worked example) --
        used instead of `keyframes` when both this key is present.
        """
        if not path:
            self.get_logger().error(
                "mode=custom but waypoints_file was not set -- no pose "
                "commands will be sent."
            )
            return
        with open(path) as f:
            data = yaml.safe_load(f) or {}

        if "phases" in data:
            self._load_phases(data["phases"])
            return

        self._waypoints_loop = bool(data.get("loop", self._waypoints_loop))
        self._keyframes = sorted(data.get("keyframes", []), key=lambda k: k["t"])
        if not self._keyframes:
            self.get_logger().error(f"waypoints_file '{path}' has no keyframes")

    def _tick_custom(self):
        kfs = self._keyframes
        if not kfs:
            return
        t = self._now_s()
        if self._waypoints_loop:
            t %= kfs[-1]["t"]

        if t <= kfs[0]["t"]:
            kf = kfs[0]
        elif t >= kfs[-1]["t"]:
            kf = kfs[-1]
        else:
            a, b = next((a, b) for a, b in zip(kfs, kfs[1:]) if a["t"] <= t <= b["t"])
            span = b["t"] - a["t"]
            frac = (t - a["t"]) / span if span > 0 else 0.0
            kf = {k: a[k] + frac * (b[k] - a[k]) for k in ("x", "y", "z", "yaw", "pitch")}

        self._send_pose(kf["x"], kf["y"], kf["z"], kf["yaw"], kf["pitch"])

    def _load_phases(self, phases: list):
        """See _tick_phased's own docstring for the phase schema. Called
        once at startup; also subscribes to whatever this phase list
        actually needs (a robot pose topic for any "orbit"/"climb" phase,
        a BT event topic for any phase with `until_event`) -- so a
        phases file with no such phase, say, never pays for a
        subscription it has no use for.
        """
        self._phases = phases
        self._phase_index = 0
        self._phase_start_time_s = self._now_s()

        if any(p.get("type") in ("orbit", "climb") for p in phases):
            self.create_subscription(
                PoseStamped,
                self.get_parameter("target_pose_topic").value,
                self._on_target_pose,
                10,
            )
        event_topics = {p["until_event"].get("topic", "bt/status_change")
                        for p in phases if "until_event" in p}
        for topic in event_topics:
            self.create_subscription(String, topic, self._on_bt_event, 10)

    def _on_bt_event(self, msg: String):
        # Same payload shape amiga_ros2_behavior_tree's FaultReporter
        # publishes on bt/status_change: {"node": <name>, "status":
        # <"SUCCESS"|"FAILURE"|...>, ...}. Only the (node, status) pair
        # matters here; everything else in the payload is ignored.
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._bt_events_seen.add((payload.get("node"), payload.get("status")))

    def _tick_phased(self):
        """`phases` schema -- an ORDERED list, each entry one of:

        - type: orbit
          radius: 7.0        # optional, defaults to the orbit_radius param
          height: 3.0        # optional, defaults to orbit_height
          period_s: 30.0     # optional, defaults to orbit_period_s
          until_event: {node: "TakeSample", status: "SUCCESS"}
          # ^ advances to the next phase the first time a BT leaf named
          # "node" reports this "status" on bt/status_change (add
          # `topic: "..."` to that dict to listen on a different, e.g.
          # namespaced, topic). Runs forever (never advances) if
          # until_event is omitted.

        - type: climb
          duration_s: 5.0
          target_height: 30.0
          # ^ rises straight up from wherever the camera's LAST commanded
          # height was to target_height (world Z) over duration_s
          # seconds, while its (x, y) stays locked directly above the
          # tracked robot's OWN current position every tick and its
          # orientation keeps looking straight at the robot -- the robot
          # stays centered in frame as the shot pulls back, the same
          # continuity guarantee flythrough gives position. Deliberately
          # NOT just a flythrough to a fixed high pose: interpolating
          # yaw/pitch by angle alone toward a fixed downward-looking
          # target would stop tracking the robot the moment it moved,
          # and computing look-at all the way to a perfectly overhead
          # finish is itself degenerate right at that finish (yaw is
          # undefined once the camera is exactly above its own look-at
          # point) -- which is exactly why THIS phase only climbs partway
          # and a flythrough phase (below) covers the rest, onto a fixed,
          # explicit final orientation instead of one more look-at call.

        - type: flythrough
          duration_s: 10.0
          to: {x: 39.0, y: 0.0, z: 85.0, yaw: 0.0, pitch: 1.5708}
          # ^ smoothly interpolates from wherever the camera's LAST
          # commanded pose was -- the previous phase's own final pose,
          # so the path this joins onto never jumps -- to `to`, over
          # duration_s seconds, then advances.

        The last phase's own end (duration_s elapsed, or until_event
        fired) simply stops sending pose commands at all: since set_pose
        teleports rather than continuously driving the camera, Gazebo
        just keeps showing it at that final pose -- no explicit "hold"
        phase type needed.
        """
        if self._phase_index >= len(self._phases):
            return
        phase = self._phases[self._phase_index]
        ptype = phase.get("type")
        t_in_phase = self._now_s() - self._phase_start_time_s

        if ptype == "orbit":
            pose = self._phase_pose_orbit(phase)
            if pose is None:
                return
        elif ptype == "climb":
            pose = self._phase_pose_climb(phase, t_in_phase)
            if pose is None:
                return
        elif ptype == "flythrough":
            pose = self._phase_pose_flythrough(phase, t_in_phase)
        else:
            self.get_logger().error(f"Unknown phase type '{ptype}' -- skipping it")
            self._phase_index += 1
            return

        self._send_pose(*pose)

        if self._phase_advance_ready(phase, t_in_phase):
            self._phase_index += 1
            self._phase_start_time_s = self._now_s()
            self._phase_start_pose = None

    def _phase_advance_ready(self, phase: dict, t_in_phase: float) -> bool:
        if "until_event" in phase:
            ev = phase["until_event"]
            return (ev["node"], ev["status"]) in self._bt_events_seen
        duration = phase.get("duration_s")
        return duration is not None and t_in_phase >= duration

    def _phase_pose_orbit(self, phase: dict):
        tracked = self._tracked_robot_pose()
        if tracked is None:
            return None
        rx, ry, rz, _robot_yaw = tracked
        radius = phase.get("radius", self.orbit_radius)
        height = phase.get("height", self.orbit_height)
        period_s = phase.get("period_s", self.orbit_period_s)

        angle = 2.0 * math.pi * (self._now_s() - self._phase_start_time_s) / period_s
        cam_pos = (rx + radius * math.cos(angle), ry + radius * math.sin(angle), rz + height)
        look_at = (rx, ry, rz + self.look_at_height)
        yaw, pitch = _look_at(cam_pos, look_at)
        return (*cam_pos, yaw, pitch)

    def _phase_pose_climb(self, phase: dict, t_in_phase: float):
        tracked = self._tracked_robot_pose()
        if tracked is None:
            return None
        rx, ry, rz, _robot_yaw = tracked

        if self._phase_start_pose is None:
            # Continuity: start climbing from whatever height the camera
            # was already at (the orbit phase before this one, typically)
            # -- not an independently-chosen start height.
            self._phase_start_pose = self._last_sent_pose or (rx, ry, rz + self.orbit_height, 0.0, 0.0)
        start_z = self._phase_start_pose[2]
        target_z = phase.get("target_height", 30.0)
        duration = phase.get("duration_s", 5.0)
        frac = min(1.0, t_in_phase / duration) if duration > 0 else 1.0
        z = start_z + (target_z - start_z) * frac

        cam_pos = (rx, ry, z)
        look_at = (rx, ry, rz + self.look_at_height)
        yaw, pitch = _look_at(cam_pos, look_at)
        return (*cam_pos, yaw, pitch)

    def _phase_pose_flythrough(self, phase: dict, t_in_phase: float):
        if self._phase_start_pose is None:
            # Continuity: start exactly where the camera already was,
            # not some independently-specified point -- a genuinely
            # continuous path, not a cut.
            self._phase_start_pose = self._last_sent_pose or (
                0.0, 0.0, 5.0, 0.0, 0.0
            )
        to = phase["to"]
        target = (to["x"], to["y"], to["z"], to["yaw"], to["pitch"])
        duration = phase.get("duration_s", 10.0)
        frac = min(1.0, t_in_phase / duration) if duration > 0 else 1.0

        sx, sy, sz, syaw, spitch = self._phase_start_pose
        tx, ty, tz, tyaw, tpitch = target
        return (
            sx + (tx - sx) * frac,
            sy + (ty - sy) * frac,
            sz + (tz - sz) * frac,
            _lerp_angle(syaw, tyaw, frac),
            _lerp_angle(spitch, tpitch, frac),
        )

    def _send_pose(self, x, y, z, yaw, pitch):
        self._last_sent_pose = (x, y, z, yaw, pitch)
        # Never queue a second request while one's in flight -- at this
        # node's own update rate that would only pile up stale commands
        # behind a slow/unready service, not make the camera move faster.
        if self._pending or not self._cli.service_is_ready():
            return
        qx, qy, qz, qw = quaternion_from_euler(0.0, pitch, yaw)

        req = SetEntityPose.Request()
        req.entity.name = self.camera_name
        req.entity.type = req.entity.MODEL
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z
        req.pose.orientation.x = qx
        req.pose.orientation.y = qy
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw

        self._pending = True
        future = self._cli.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        self._pending = False
        exc = future.exception()
        if exc is not None:
            self.get_logger().warn(f"set_pose call failed: {exc}")
        elif not future.result().success:
            self.get_logger().warn("set_pose reported success=false")


def main():
    rclpy.init()
    rclpy.spin(CameraTrajectoryNode())


if __name__ == "__main__":
    main()
