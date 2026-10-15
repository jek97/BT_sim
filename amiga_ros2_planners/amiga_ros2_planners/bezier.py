"""
bezier.py

Resamples a chained-cubic-Bezier control point list -- the format every
planner in planning_core.py returns, length 3k+1 for k segments, the
SAME convention problog_project's basic_action_theory.pl consumes via
its own spline_point/4 -- into a piecewise-linear (x, y, yaw) polyline,
ready to become a nav_msgs/Path for move_to_node.py to hand to Nav2's
FollowPath. Pure Python/math, no ROS import, so it's testable standalone
like every other core module in this package.

Evaluated via the direct cubic Bezier formula (De Casteljau's would work
too; the closed form is just as simple at degree 3 and avoids an extra
loop), one segment at a time. `yaw` at each sampled point comes from the
curve's own analytic tangent (the Bezier derivative), not from
finite-differencing the resampled points afterward -- exact, and well
defined even at a segment's own endpoints.
"""
import math


def evaluate_cubic_bezier(p0, p1, p2, p3, t):
    """(x, y) at parameter t in [0, 1] along the cubic Bezier segment
    p0->p1->p2->p3."""
    mt = 1.0 - t
    x = mt ** 3 * p0[0] + 3 * mt ** 2 * t * p1[0] + 3 * mt * t ** 2 * p2[0] + t ** 3 * p3[0]
    y = mt ** 3 * p0[1] + 3 * mt ** 2 * t * p1[1] + 3 * mt * t ** 2 * p2[1] + t ** 3 * p3[1]
    return x, y


def cubic_bezier_tangent(p0, p1, p2, p3, t):
    """(dx, dy) -- the cubic Bezier's own derivative at t, i.e. the
    curve's direction of travel (not normalized)."""
    mt = 1.0 - t
    dx = 3 * mt ** 2 * (p1[0] - p0[0]) + 6 * mt * t * (p2[0] - p1[0]) + 3 * t ** 2 * (p3[0] - p2[0])
    dy = 3 * mt ** 2 * (p1[1] - p0[1]) + 6 * mt * t * (p2[1] - p1[1]) + 3 * t ** 2 * (p3[1] - p2[1])
    return dx, dy


def sample_bezier_chain(control_points, samples_per_segment=10):
    """[(x,y), ...] of length 3k+1 -> [(x, y, yaw), ...]: `samples_per_segment`
    points per internal segment, plus one extra (the chain's own final
    endpoint) on the last segment -- so every internal waypoint join
    appears exactly ONCE in the output, not as a duplicated zero-distance
    pair.

    Used to duplicate the shared boundary point at every internal join
    (old segment's own final sample AND the new segment's own first
    sample, both at the identical (x, y), with the old leg's heading and
    the new leg's heading respectively) for the benefit of this
    project's own now-removed open-loop MoveTo backend, which fit a
    constant-rate arc between consecutive samples and needed an explicit
    dist=0 pure-rotation pair to represent a kink exactly. Nav2's
    RegulatedPurePursuitController is now the only consumer, and its own
    carrot-point/closest-point-on-path search does not expect two
    samples at the identical position -- confirmed via a live /
    lookahead_point trace showing the carrot snapping to a completely
    different point (e.g. from ~0.8m ahead to ~0.8m behind the robot in
    a single control cycle) right around an internal corner's own
    duplicate pair. A kink in the PATH's own geometry is still exactly
    representable here (the single sample right at the join still gets
    the new segment's own tangent, an abrupt orientation change versus
    the previous sample), it just doesn't duplicate the POSITION
    alongside it anymore.

    `yaw` falls back to the previous sampled point's own yaw at a
    zero-length segment (e.g. the degenerate "already there" chain
    every planning_core.py planner returns as [(x,y)]*4), or 0.0 for the
    very first point of a chain that is zero-length everywhere.

    Raises ValueError if `control_points` isn't a valid 3k+1-length
    chain (k >= 1)."""
    n = len(control_points)
    if n < 4 or (n - 1) % 3 != 0:
        raise ValueError(
            f"sample_bezier_chain: expected a 3k+1-length control point "
            f"list (k>=1 cubic segments), got length {n}")
    num_segments = (n - 1) // 3

    points = []
    for seg in range(num_segments):
        p0, p1, p2, p3 = control_points[3 * seg:3 * seg + 4]
        # Every segment but the last stops one sample short of its own
        # t=1.0 endpoint -- the NEXT segment's own t=0.0 sample covers
        # that exact position instead (with the new segment's own
        # tangent), so the shared join appears once, not twice.
        is_last_segment = (seg == num_segments - 1)
        count = samples_per_segment + 1 if is_last_segment else samples_per_segment
        for i in range(count):
            t = i / samples_per_segment
            x, y = evaluate_cubic_bezier(p0, p1, p2, p3, t)
            dx, dy = cubic_bezier_tangent(p0, p1, p2, p3, t)
            if dx != 0.0 or dy != 0.0:
                yaw = math.atan2(dy, dx)
            else:
                yaw = points[-1][2] if points else 0.0
            points.append((x, y, yaw))
    return points
