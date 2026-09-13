"""
open_loop_trajectory.py

Pure-Python core for move_to_openloop_node.py -- turns a sampled
(x, y, yaw) polyline (bezier.sample_bezier_chain's own output) into a
sequence of constant (linear_x, angular_z) commands, each held for its
own fixed duration, and integrates the UNICYCLE MODEL forward to track
where those commands are nominally expected to put the robot.

"Open loop" here means exactly this: every number below comes from the
PLANNED path and a chosen speed alone -- nothing here ever reads the
robot's actual measured position/heading to correct a discrepancy (the
node built on top of this, move_to_openloop_node.py, never subscribes
to odometry or tf2 at all). This is Nav2 FollowPath's own controller_
server pursuit-controller counterpart with the "pursuit" (closed-loop
correction) removed -- see that node's own module docstring for the
full rationale on why you'd want this.

No ROS import here on purpose -- unit-testable standalone, same
"pure-Python core, ROS node calls in" shape as planning_core.py/
bezier.py/ploughing.py.
"""
import math


def wrap_to_pi(angle):
    """`angle` (radians), wrapped into (-pi, pi] -- the shortest signed
    turn a constant angular_z command should cover, never the long way
    around (e.g. a jump from +179 deg to -179 deg is a 2 deg turn, not
    a 358 deg one)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def build_velocity_segments(waypoints, speed, max_angular_speed):
    """[(duration_s, linear_x, angular_z), ...], one entry per
    consecutive pair in `waypoints` ([(x,y,yaw), ...], e.g.
    bezier.sample_bezier_chain's own output; needs at least 2 points).

    Each segment's own duration is whichever is LONGER: the time this
    segment's straight-line distance takes at `speed`, or the time its
    own yaw change takes at `max_angular_speed` (never exceeding that
    cap, even if that means running slower than `speed` on a sharply
    curving/rotate-in-place segment) -- then BOTH linear_x and
    angular_z are set to exactly cover that segment's own distance/yaw
    change in that one shared duration. This is two independent
    constant rates timed to finish together, not a coordinated-turn
    (curvature) computation -- accurate enough once the segments
    themselves are already short (whatever samples_per_segment
    produced), same spirit as move_to_node.py's own polyline
    approximation of the underlying Bezier curve.

    A segment whose distance AND yaw change are both ~0 is dropped
    entirely (nothing to command) -- can happen at a chain's own
    shared segment-boundary sample if the path briefly doubles back
    exactly, though sample_bezier_chain itself already de-duplicates
    the common case (consecutive segments sharing one boundary point).
    Returns [] if `waypoints` has fewer than 2 points (nothing to
    traverse) -- the degenerate "already there" case."""
    segments = []
    for (x0, y0, yaw0), (x1, y1, yaw1) in zip(waypoints, waypoints[1:]):
        dist = math.hypot(x1 - x0, y1 - y0)
        dyaw = wrap_to_pi(yaw1 - yaw0)
        duration_for_distance = dist / speed if speed > 1.0e-9 else 0.0
        duration_for_rotation = (
            abs(dyaw) / max_angular_speed if max_angular_speed > 1.0e-9 else 0.0)
        duration = max(duration_for_distance, duration_for_rotation)
        if duration <= 1.0e-9:
            continue
        segments.append((duration, dist / duration, dyaw / duration))
    return segments


def integrate_unicycle_step(x, y, yaw, linear_x, angular_z, dt):
    """(x, y, yaw) after `dt` seconds at a CONSTANT (linear_x,
    angular_z) command -- the exact closed-form unicycle-model arc
    (not a small-angle/Euler approximation, so this stays accurate
    even for a long dt at a large angular_z): a straight line when
    angular_z is ~0, otherwise a genuine circular arc of radius
    linear_x/angular_z."""
    if abs(angular_z) < 1.0e-9:
        return x + linear_x * dt * math.cos(yaw), y + linear_x * dt * math.sin(yaw), yaw
    yaw1 = yaw + angular_z * dt
    radius = linear_x / angular_z
    x1 = x + radius * (math.sin(yaw1) - math.sin(yaw))
    y1 = y - radius * (math.cos(yaw1) - math.cos(yaw))
    return x1, y1, yaw1
