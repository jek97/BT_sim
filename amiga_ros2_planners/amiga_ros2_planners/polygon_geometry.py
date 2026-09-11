"""
polygon_geometry.py

Polygon-obstacle geometry -- the analogue of geometry.py's circle
primitives, for obstacles that are genuine polygons rather than
circular tree canopies. Needed because a problog_project PROBLEM's own
obstacles_generated.pl describes arbitrary polygons (see
problog_problem.py's own docstring for where these come from), not
trees -- reusing geometry.py's circle formulas for them would be wrong,
not just imprecise.

Every obstacle here is `rings` = [outer_points, hole1_points, ...]
(rings[0] always the outer boundary, rings[1:] its own hole(s) if any
-- see problog_problem.load_obstacle_polygons's own docstring), NOT a
flat point list: an obstacle with a hollow, walkable interior (e.g. a
perimeter fence's own inner face) needs its hole ring(s) folded into
every containment/clearance test here, or that hollow interior would
incorrectly read as solid -- see planning_core.py's own _inside_polygon
docstring for the full even-odd-over-combined-rings argument this
module leans on.

Reuses planning_core.py's own _polygon_edges/_all_edges/_inside_polygon
(already byte-for-byte ports of problog_project's own polygon
primitives) rather than duplicating them -- this module only adds what
those don't already cover: boundary DISTANCE (not just inside/outside),
the argmin-over-obstacles search geometry.py's nearest_obstacle does for
circles, and segment-vs-polygon intersection.
"""
import math

from amiga_ros2_planners.planning_core import _all_edges, _inside_polygon


def _closest_point_on_segment(px, py, ax, ay, bx, by):
    sdx, sdy = bx - ax, by - ay
    len2 = sdx * sdx + sdy * sdy
    if len2 <= 1.0e-12:
        return ax, ay
    t = max(0.0, min(1.0, ((px - ax) * sdx + (py - ay) * sdy) / len2))
    return ax + t * sdx, ay + t * sdy


def distance_to_polygon_boundary(x, y, rings):
    """Signed-ish distance from (x,y) to the obstacle's own FULL
    boundary -- outer ring AND every hole ring, since the obstacle's
    actual solid material is bounded by both. Positive outside,
    negative inside (point_inside_polygon), magnitude = distance to the
    nearest edge either way. The polygon analogue of geometry.py's
    distance_to_obstacle_boundary."""
    best = math.inf
    for (ax, ay), (bx, by) in _all_edges(rings):
        cx, cy = _closest_point_on_segment(x, y, ax, ay, bx, by)
        best = min(best, math.hypot(x - cx, y - cy))
    if point_inside_polygon(x, y, rings):
        return -best
    return best


def point_inside_polygon(x, y, rings):
    """True iff (x,y) is inside the obstacle's own solid material
    (rings) -- thin wrapper over planning_core._inside_polygon, kept
    here so callers of this module never need to import a "private"
    planning_core name directly."""
    return _inside_polygon(x, y, rings)


def nearest_polygon_obstacle(x, y, obstacle_polygons):
    """(id, clearance) for whichever of `obstacle_polygons`
    ([(id, rings), ...]) is closest to (x,y) by boundary clearance, or
    (None, math.inf) if the list is empty -- the polygon analogue of
    geometry.py's nearest_obstacle."""
    best_id = None
    best_clearance = math.inf
    for obstacle_id, rings in obstacle_polygons:
        clearance = distance_to_polygon_boundary(x, y, rings)
        if clearance < best_clearance:
            best_clearance = clearance
            best_id = obstacle_id
    return best_id, best_clearance


def _segments_intersect(ax, ay, bx, by, cx, cy, dx, dy):
    """True iff segment (a,b) properly or improperly intersects segment
    (c,d) -- the standard orientation-based test."""
    def orientation(px, py, qx, qy, rx, ry):
        val = (qx - px) * (ry - py) - (qy - py) * (rx - px)
        if abs(val) < 1.0e-12:
            return 0
        return 1 if val > 0 else -1

    def on_segment(px, py, qx, qy, rx, ry):
        return (min(px, rx) - 1.0e-12 <= qx <= max(px, rx) + 1.0e-12 and
                min(py, ry) - 1.0e-12 <= qy <= max(py, ry) + 1.0e-12)

    o1 = orientation(ax, ay, bx, by, cx, cy)
    o2 = orientation(ax, ay, bx, by, dx, dy)
    o3 = orientation(cx, cy, dx, dy, ax, ay)
    o4 = orientation(cx, cy, dx, dy, bx, by)

    if o1 != o2 and o3 != o4:
        return True
    if o1 == 0 and on_segment(ax, ay, cx, cy, bx, by):
        return True
    if o2 == 0 and on_segment(ax, ay, dx, dy, bx, by):
        return True
    if o3 == 0 and on_segment(cx, cy, ax, ay, dx, dy):
        return True
    if o4 == 0 and on_segment(cx, cy, bx, by, dx, dy):
        return True
    return False


def segment_intersects_polygon(ax, ay, bx, by, rings):
    """True iff the segment (ax,ay)-(bx,by) crosses ANY edge of the
    obstacle's own rings (outer + holes), OR lies entirely inside its
    solid material (a segment wholly inside a polygon crosses none of
    its edges but still needs to count as blocked) -- the exact polygon
    analogue of geometry.py's segment_intersects_circle."""
    for (px, py), (qx, qy) in _all_edges(rings):
        if _segments_intersect(ax, ay, bx, by, px, py, qx, qy):
            return True
    return point_inside_polygon(ax, ay, rings)


def line_of_sight_clear_polygon(from_x, from_y, to_x, to_y, rings):
    """True iff the straight segment from (from_x,from_y) to
    (to_x,to_y) does NOT clip the obstacle's own rings -- the polygon
    analogue of geometry.py's line_of_sight_clear."""
    return not segment_intersects_polygon(from_x, from_y, to_x, to_y, rings)
