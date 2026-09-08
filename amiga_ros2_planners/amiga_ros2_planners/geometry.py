"""
geometry.py

Circle-based obstacle geometry for this package's planners and
conditions -- the orchard analogue of problog_project's
collision_geometry.py, which works against arbitrary
obstacle_polygon/2 polygons. Trees in this simulation are circular
canopies (see orchard_obstacles.py's Obstacle type: center + radius),
not arbitrary polygons, so every primitive below is the closed-form
circle version of the corresponding polygon check there -- exact, not
an approximation, and considerably simpler than porting polygon
clipping for a shape this domain doesn't actually have.

Every distance/threshold here is to the obstacle's OWN boundary
(circle radius), matching problog_project's own convention that a
Threshold is raw clearance to an obstacle's boundary, with any
robot-radius/safety-buffer margin already folded in by the caller.
"""
import math


def distance_to_obstacle_boundary(x, y, obstacle):
    """Signed-ish distance from (x,y) to `obstacle`'s own boundary:
    positive outside the circle, negative inside it. Matches
    collision_geometry.py's own point-to-polygon-boundary clearance,
    just via the closed-form circle formula instead of a polygon
    nearest-edge search."""
    return math.hypot(x - obstacle.x, y - obstacle.y) - obstacle.radius


def nearest_obstacle(x, y, obstacles):
    """(obstacle, clearance) for whichever of `obstacles` is closest to
    (x,y) by boundary clearance, or (None, math.inf) if `obstacles` is
    empty -- the argmin collision_geometry.py's own crashed/
    obstacle_in_bound Reason needs to name WHICH obstacle fired."""
    best_obstacle = None
    best_clearance = math.inf
    for obstacle in obstacles:
        clearance = distance_to_obstacle_boundary(x, y, obstacle)
        if clearance < best_clearance:
            best_clearance = clearance
            best_obstacle = obstacle
    return best_obstacle, best_clearance


def point_inside_obstacle(x, y, obstacle):
    """True iff (x,y) is inside `obstacle`'s own circle -- the circle
    analogue of collision_geometry.py's _inside_polygon."""
    return distance_to_obstacle_boundary(x, y, obstacle) < 0.0


def segment_intersects_circle(ax, ay, bx, by, cx, cy, radius):
    """True iff the segment (ax,ay)-(bx,by) passes within `radius` of
    (cx,cy) -- the standard closed-form point-to-segment distance,
    compared against radius rather than the segment-crosses-polygon
    scan-line test collision_geometry.py needs for arbitrary polygons.
    Used by both LineOfSightClear (radius = the tree's own canopy
    radius, i.e. "does this line of sight clip the tree at all") and by
    the astar/voronoi/follow_boarder planners below (radius = canopy
    radius + planning inflation, i.e. "does this candidate edge pass
    too close to this tree")."""
    sdx, sdy = bx - ax, by - ay
    seg_len2 = sdx * sdx + sdy * sdy
    if seg_len2 <= 1.0e-12:
        return math.hypot(ax - cx, ay - cy) <= radius
    t = max(0.0, min(1.0, ((cx - ax) * sdx + (cy - ay) * sdy) / seg_len2))
    px, py = ax + t * sdx, ay + t * sdy
    return math.hypot(px - cx, py - cy) <= radius


def line_of_sight_clear(from_x, from_y, to_x, to_y, obstacle):
    """True iff the straight segment from (from_x,from_y) to (to_x,to_y)
    does NOT clip `obstacle`'s own canopy -- the circle analogue of
    collision_geometry.py's line_of_sight_clear/segment_crosses_polygon
    (Bug0's own boundary-leave rule; see schema.yaml's LineOfSightClear
    condition)."""
    return not segment_intersects_circle(
        from_x, from_y, to_x, to_y, obstacle.x, obstacle.y, obstacle.radius)
