"""
planning_core.py

The four PlanWith algorithms (astar/straight/voronoi/follow_boarder)
ported from problog_project/module/theory/planners.py onto this
simulation's own orchard, for plan_service_node.py to call.

_straight_control_points, the A* search itself (astar()/
_reconstruct_path), the B-spline-to-Bezier-chain fit
(fit_spline/bspline_to_bezier_chain), AND -- as of this revision --
Voronoi (_voronoi_sites/_segment_crosses_any_obstacle/
_voronoi_roadmap_edges/_insert_point_into_roadmap/
_dijkstra_shortest_path/_voronoi_control_points) and follow_boarder
(_polygon_edges/_edge_crosses/_inside_polygon/_signed_polygon_area/
_offset_boundary_clockwise/_follow_boarder_control_points) are ALL
BYTE-FOR-BYTE ports of problog_project's own functions of the same
name -- not the circle-simplified rewrite an earlier revision of this
file used. Kept polygon-general on purpose: this orchard's obstacles
happen to be circular trees today, but the whole point of porting the
ORIGINAL polygon algorithm rather than a circle-specialized shortcut is
that a future obstacle source (a hedgerow, a parked implement, another
robot's own footprint) need not be circular at all -- obstacle_polygons
below is a plain [(id, [(x,y), ...]), ...] list, exactly
problog_project's own _OBSTACLE_POLYGONS shape, just passed as a
parameter here (problog_project loads it once from
obstacles_generated.pl at import time; this simulation has no such
file, so plan_service_node builds this list itself each call from
whatever OrchardObstacleStore currently reports -- see
circle_to_polygon/obstacles_to_polygons below for the one adapter step
that differs).

Only two things are genuinely NEW rather than ported:
  - circle_to_polygon/obstacles_to_polygons: today's only obstacle
    source (orchard_obstacles.py) reports circular canopies, not
    polygons, so every circle is approximated once as a regular N-gon
    before being handed to the byte-for-byte-ported polygon functions
    above -- these two are the ENTIRE adapter, nothing downstream of
    them needs to know a tree was ever a circle.
  - build_occupancy_grid/OccupancyGridMap: A* needs a grid, and there is
    no map.pgm/map.yaml here -- see orchard_map.py for the whole-orchard,
    built-once-at-startup counterpart to problog_project's own
    load_map_yaml (this module's own build_occupancy_grid remains
    available as a query-scoped fallback -- e.g. before orchard_map_node
    has published anything yet, or for standalone testing).
"""
import heapq
import math

import numpy as np
from scipy.interpolate import splprep, insert
from scipy.spatial import Voronoi

# Same defaults as problog_project's planners.py (PLANNING_INFLATE_M/
# OCC_THRESH/CONNECTIVITY there) -- planner-internal tuning, not a
# simulation config value.
PLANNING_INFLATE_M = 0.3
GRID_RESOLUTION_M = 0.25
GRID_MARGIN_M = 3.0
OCC_THRESH = 50
CONNECTIVITY = 8

# Same per-edge sample density problog_project's own
# _BOUNDARY_SAMPLES_PER_EDGE/_VORONOI_SAMPLES_PER_EDGE use.
_BOUNDARY_SAMPLES_PER_EDGE = 8
_NORMAL_PROBE_EPS = 1.0e-3
_VORONOI_SAMPLES_PER_EDGE = 6
_VORONOI_EDGE_CHECK_SAMPLES = 6

# Circle -> regular-polygon approximation, this simulation's one
# adapter step -- see module docstring.
_CIRCLE_POLYGON_SIDES = 16


# =====================================================================
# STRAIGHT -- unchanged from problog_project's _straight_control_points.
# =====================================================================
def straight_control_points(sx, sy, gx, gy):
    """A single cubic Bezier segment, collinear control points -- reduces
    exactly to a straight line."""
    dx, dy = gx - sx, gy - sy
    p0 = (sx, sy)
    p1 = (sx + dx / 3.0, sy + dy / 3.0)
    p2 = (sx + 2.0 * dx / 3.0, sy + 2.0 * dy / 3.0)
    p3 = (gx, gy)
    return [p0, p1, p2, p3]


# =====================================================================
# Spline fitting -- unchanged from problog_project's planners.py.
# =====================================================================
def fit_spline(path_xy, degree=3, smoothing=0.0):
    """Fit a parametric B-spline through the (x, y) waypoints. Returns
    (tck, u) as produced by scipy.interpolate.splprep."""
    path_xy = np.asarray(path_xy, dtype=float)
    x, y = path_xy[:, 0], path_xy[:, 1]
    n = len(x)
    if n < 2:
        raise ValueError("Need at least 2 waypoints to fit a spline")
    k = max(1, min(degree, n - 1))
    tck, u = splprep([x, y], k=k, s=smoothing)
    return tck, u


def bspline_to_bezier_chain(tck):
    """Convert a scipy parametric B-spline tck=(t,c,k) into an exact
    chain of degree-k Bezier segments via full knot insertion -- see
    problog_project/module/theory/planners.py's own docstring for the
    full derivation; this is a byte-for-byte port, only cubic (k=3)
    input supported."""
    t, c, k = tck
    if k != 3:
        raise ValueError(
            f"bspline_to_bezier_chain: got degree k={k}, but only cubic "
            f"(k=3) Bezier segments are supported.")

    t = list(t)
    c = [list(comp) for comp in c]
    tck2 = (t, c, k)

    interior_knots = sorted(set(t[k + 1: len(t) - (k + 1)]))
    for knot in interior_knots:
        current_mult = t.count(knot)
        needed = k - current_mult
        if needed > 0:
            tck2 = insert(knot, tck2, m=needed, per=0)
            t = list(tck2[0])

    t_final, c_final, k_final = tck2
    n_ctrl = len(t_final) - k_final - 1
    cx, cy = c_final[0][:n_ctrl], c_final[1][:n_ctrl]
    control_points = list(zip(cx, cy))

    if (len(control_points) - 1) % k_final != 0:
        raise ValueError(
            "bspline_to_bezier_chain: extraction did not yield a clean "
            "3k+1-length control point list.")

    return control_points, k_final


def _fit_or_straight(path_xy, sx, sy, gx, gy):
    """fit_spline+bspline_to_bezier_chain, falling back to a straight
    line between the actual endpoints on a degenerate/too-short path --
    same fallback every problog_project planner uses at its own call
    site, pulled out once here since all three grid/roadmap planners
    below need it identically."""
    if len(path_xy) < 2:
        return [(sx, sy)] * 4
    try:
        tck, _u = fit_spline(path_xy, degree=3, smoothing=0.0)
        control_points, _k = bspline_to_bezier_chain(tck)
        return control_points
    except ValueError:
        return straight_control_points(sx, sy, gx, gy)


# =====================================================================
# Circle -> polygon adapter -- the ONE step that is new rather than
# ported (see module docstring). Everything past this point takes
# obstacle_polygons, exactly problog_project's own [(id, [(x,y), ...]),
# ...] shape, and knows nothing about circles at all.
# =====================================================================
def circle_to_polygon(obstacle, num_sides=_CIRCLE_POLYGON_SIDES):
    """One Obstacle(id, x, y, radius) -> (id, [(x,y), ...]) -- a regular
    num_sides-gon approximating its canopy circle, vertices in
    counterclockwise order (either winding works: every polygon
    function below re-derives its own via _signed_polygon_area, same as
    problog_project's own obstacle_polygon/2 facts make no winding
    guarantee either)."""
    vertices = [
        (obstacle.x + obstacle.radius * math.cos(2.0 * math.pi * k / num_sides),
         obstacle.y + obstacle.radius * math.sin(2.0 * math.pi * k / num_sides))
        for k in range(num_sides)
    ]
    return obstacle.id, vertices


def obstacles_to_polygons(obstacles, num_sides=_CIRCLE_POLYGON_SIDES):
    """[Obstacle, ...] -> [(id, [(x,y), ...]), ...] -- see
    circle_to_polygon above."""
    return [circle_to_polygon(o, num_sides) for o in obstacles]


# =====================================================================
# A* -- astar()/_reconstruct_path unchanged from problog_project's
# planners.py; OccupancyGridMap/build_occupancy_grid replace its
# load_map_yaml (see module docstring, and orchard_map.py, for why).
# =====================================================================
class OccupancyGridMap:
    """Same shape as problog_project's own OccupancyGridMap (a minimal
    stand-in for nav_msgs/OccupancyGrid)."""

    def __init__(self, data, resolution, origin_x, origin_y):
        self.data = data
        self.resolution = float(resolution)
        self.origin = (float(origin_x), float(origin_y))
        self.height, self.width = data.shape

    def world_to_grid(self, x, y):
        col = int(math.floor((x - self.origin[0]) / self.resolution))
        row = int(math.floor((y - self.origin[1]) / self.resolution))
        return row, col

    def grid_to_world(self, row, col):
        x = self.origin[0] + (col + 0.5) * self.resolution
        y = self.origin[1] + (row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def is_free(self, row, col, occ_thresh=OCC_THRESH):
        if not self.in_bounds(row, col):
            return False
        return self.data[row, col] < occ_thresh


def build_occupancy_grid(obstacles, sx, sy, gx, gy,
                          resolution=GRID_RESOLUTION_M,
                          margin=GRID_MARGIN_M,
                          inflate=PLANNING_INFLATE_M):
    """Rasterize a query-scoped occupancy grid: bounds are the bounding
    box of (start, goal, every obstacle's own extent) plus `margin`.
    Used as a fallback when no whole-orchard map has been published yet
    (see orchard_map.py/orchard_map_node.py for the preferred,
    built-once path) and directly by this module's own unit tests."""
    min_x = min(sx, gx, *(o.x - o.radius for o in obstacles)) - margin \
        if obstacles else min(sx, gx) - margin
    max_x = max(sx, gx, *(o.x + o.radius for o in obstacles)) + margin \
        if obstacles else max(sx, gx) + margin
    min_y = min(sy, gy, *(o.y - o.radius for o in obstacles)) - margin \
        if obstacles else min(sy, gy) - margin
    max_y = max(sy, gy, *(o.y + o.radius for o in obstacles)) + margin \
        if obstacles else max(sy, gy) + margin

    width = max(1, int(math.ceil((max_x - min_x) / resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution)))
    data = np.zeros((height, width), dtype=np.int8)
    grid = OccupancyGridMap(data, resolution, min_x, min_y)

    yy, xx = np.mgrid[0:height, 0:width]
    world_x = min_x + (xx + 0.5) * resolution
    world_y = min_y + (yy + 0.5) * resolution
    for obstacle in obstacles:
        occupied_radius = obstacle.radius + inflate
        mask = (world_x - obstacle.x) ** 2 + (world_y - obstacle.y) ** 2 \
            <= occupied_radius ** 2
        data[mask] = 100

    return grid


def build_grid_map(obstacles, resolution, margin, inflate=PLANNING_INFLATE_M):
    """Rasterize ONE OccupancyGridMap covering every obstacle's own
    extent plus `margin` -- the whole-orchard analogue of
    build_occupancy_grid above, sized once from every known obstacle
    rather than per (start, goal) query. Used by orchard_map.py (ROS) and
    by scripts/export_orchard_map.py (deliberately ROS-free, so it can
    run standalone against a checked-in orchard JSON fixture with no
    ROS2 environment at all -- see that script's own header) alike;
    kept here rather than in orchard_map.py so neither caller needs to
    import anything ROS-specific just to rasterize a grid. Returns a 1x1
    empty grid at the origin if `obstacles` is empty, so a caller always
    gets something usable."""
    if not obstacles:
        data = np.zeros((1, 1), dtype=np.int8)
        return OccupancyGridMap(data, resolution, -margin, -margin)

    min_x = min(o.x - o.radius for o in obstacles) - margin
    max_x = max(o.x + o.radius for o in obstacles) + margin
    min_y = min(o.y - o.radius for o in obstacles) - margin
    max_y = max(o.y + o.radius for o in obstacles) + margin

    width = max(1, int(math.ceil((max_x - min_x) / resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution)))
    data = np.zeros((height, width), dtype=np.int8)
    grid = OccupancyGridMap(data, resolution, min_x, min_y)

    yy, xx = np.mgrid[0:height, 0:width]
    world_x = min_x + (xx + 0.5) * resolution
    world_y = min_y + (yy + 0.5) * resolution
    for obstacle in obstacles:
        occupied_radius = obstacle.radius + inflate
        mask = (world_x - obstacle.x) ** 2 + (world_y - obstacle.y) ** 2 \
            <= occupied_radius ** 2
        data[mask] = 100

    return grid


def build_polygon_grid(obstacle_polygons, sx, sy, gx, gy,
                        resolution=GRID_RESOLUTION_M, margin=GRID_MARGIN_M):
    """Rasterize a query-scoped occupancy grid from GENUINE polygon
    obstacles (a problog_project problem's own obstacles_generated.pl,
    via problog_obstacles.py -- see that module's docstring) rather than
    circular trees: build_occupancy_grid/build_grid_map above both stamp
    in filled DISKS, which is wrong for an arbitrary polygon, so this is
    a separate rasterizer, not a parametrized version of those two. Each
    cell is marked occupied by an exact point-in-polygon test
    (_inside_polygon, already ported byte-for-byte from
    problog_project's own planners.py) rather than a distance
    threshold -- correct for a non-convex/non-circular shape, where
    "distance to center" means nothing.

    No separate robot-clearance inflation term (unlike
    build_occupancy_grid's own `inflate`): a problog_project problem's
    own obstacle polygons are typically already the FULL inflated
    footprint (occgrid_to_problog.py's own extraction already accounts
    for robot radius/safety_buffer at the source -- see
    obstacles_generated.pl's own header comment) -- inflating again here
    would double-count it."""
    xs = [x for _oid, poly in obstacle_polygons for x, _y in poly]
    ys = [y for _oid, poly in obstacle_polygons for _x, y in poly]
    min_x = min(sx, gx, *xs) - margin if xs else min(sx, gx) - margin
    max_x = max(sx, gx, *xs) + margin if xs else max(sx, gx) + margin
    min_y = min(sy, gy, *ys) - margin if ys else min(sy, gy) - margin
    max_y = max(sy, gy, *ys) + margin if ys else max(sy, gy) + margin

    width = max(1, int(math.ceil((max_x - min_x) / resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution)))
    data = np.zeros((height, width), dtype=np.int8)
    grid = OccupancyGridMap(data, resolution, min_x, min_y)

    for row in range(height):
        for col in range(width):
            world_x, world_y = grid.grid_to_world(row, col)
            for _obstacle_id, polygon in obstacle_polygons:
                if _inside_polygon(world_x, world_y, polygon):
                    data[row, col] = 100
                    break

    return grid


def plan_astar_points_polygons(sx, sy, gx, gy, obstacle_polygons):
    """plan_astar_points's own polygon-obstacle counterpart -- see
    build_polygon_grid's own docstring for why this needs a separate
    rasterizer rather than reusing plan_astar_points(obstacles=...)."""
    sx, sy, gx, gy = float(sx), float(sy), float(gx), float(gy)
    grid = build_polygon_grid(obstacle_polygons, sx, sy, gx, gy)
    start_rc = grid.world_to_grid(sx, sy)
    goal_rc = grid.world_to_grid(gx, gy)

    if not grid.in_bounds(*start_rc) or not grid.in_bounds(*goal_rc):
        return None
    if start_rc == goal_rc:
        return [(sx, sy)] * 4

    path_rc = astar(grid, start_rc, goal_rc)
    if path_rc is None:
        return None

    path_xy = [grid.grid_to_world(r, c) for r, c in path_rc]
    return _fit_or_straight(path_xy, sx, sy, gx, gy)


def astar(grid_map, start_rc, goal_rc, occ_thresh=OCC_THRESH,
          connectivity=CONNECTIVITY):
    """8- or 4-connected A* over grid_map -- unchanged from
    problog_project's planners.py. Returns a list of (row, col) cells
    from start to goal (inclusive), or None if no path exists."""
    if not grid_map.is_free(*start_rc, occ_thresh):
        return None
    if not grid_map.is_free(*goal_rc, occ_thresh):
        return None

    if connectivity == 8:
        steps = [(-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)),
                 (0, -1, 1.0), (0, 1, 1.0),
                 (1, -1, math.sqrt(2)), (1, 0, 1.0), (1, 1, math.sqrt(2))]
    else:
        steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]

    def heuristic(rc):
        return math.hypot(rc[0] - goal_rc[0], rc[1] - goal_rc[1])

    open_heap = [(heuristic(start_rc), 0.0, start_rc)]
    came_from = {}
    g_score = {start_rc: 0.0}
    closed = set()

    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal_rc:
            return _reconstruct_path(came_from, current)
        closed.add(current)

        for dr, dc, step_cost in steps:
            neighbor = (current[0] + dr, current[1] + dc)
            if not grid_map.is_free(*neighbor, occ_thresh):
                continue
            if dr != 0 and dc != 0:
                if not grid_map.is_free(current[0] + dr, current[1], occ_thresh):
                    continue
                if not grid_map.is_free(current[0], current[1] + dc, occ_thresh):
                    continue

            tentative_g = g + step_cost
            if tentative_g < g_score.get(neighbor, float("inf")):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + heuristic(neighbor)
                heapq.heappush(open_heap, (f_score, tentative_g, neighbor))

    return None


def _reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def plan_astar_points(sx, sy, gx, gy, obstacles=None, grid=None):
    """Grid-based A* planner. Pass a pre-built `grid` (an
    OccupancyGridMap -- e.g. orchard_map.occupancy_grid_msg_to_grid's
    output, cached from orchard_map_node's own published map) to plan
    against a grid built ONCE, matching problog_project's own
    load-once-at-import-time map; omit it (pass `obstacles` instead) to
    fall back to a query-scoped grid rasterized on the spot via
    build_occupancy_grid, e.g. before that map has been published yet.
    Returns [(x,y), ...] control points, or None if start/goal fall on
    an obstacle cell or no path exists."""
    sx, sy, gx, gy = float(sx), float(sy), float(gx), float(gy)
    if grid is None:
        grid = build_occupancy_grid(obstacles or [], sx, sy, gx, gy)

    start_rc = grid.world_to_grid(sx, sy)
    goal_rc = grid.world_to_grid(gx, gy)

    if not grid.in_bounds(*start_rc) or not grid.in_bounds(*goal_rc):
        return None

    if start_rc == goal_rc:
        return [(sx, sy)] * 4

    path_rc = astar(grid, start_rc, goal_rc)
    if path_rc is None:
        return None

    path_xy = [grid.grid_to_world(r, c) for r, c in path_rc]
    return _fit_or_straight(path_xy, sx, sy, gx, gy)


# =====================================================================
# Polygon geometry -- byte-for-byte ports of problog_project's own
# _polygon_edges/_edge_crosses/_inside_polygon/_signed_polygon_area.
# =====================================================================
def _polygon_edges(points):
    closed = list(points) + [points[0]]
    return list(zip(closed[:-1], closed[1:]))


def _edge_crosses(px, py, ax, ay, bx, by):
    if (ay > py and by <= py) or (by > py and ay <= py):
        x_cross = ax + (py - ay) / (by - ay) * (bx - ax)
        return px < x_cross
    return False


def _inside_polygon(px, py, points):
    count = sum(1 for (ax, ay), (bx, by) in _polygon_edges(points)
                if _edge_crosses(px, py, ax, ay, bx, by))
    return count % 2 == 1


def _signed_polygon_area(points):
    return sum(ax * by - bx * ay for (ax, ay), (bx, by) in _polygon_edges(points)) / 2.0


# =====================================================================
# FOLLOW_BOARDER -- byte-for-byte port of problog_project's own
# _offset_boundary_clockwise/_follow_boarder_control_points, taking
# `obstacle_polygons` as a parameter (problog_project's own module-level
# _OBSTACLE_POLYGONS, loaded once from a file, has no equivalent here --
# see module docstring).
# =====================================================================
def _offset_boundary_clockwise(polygon, offset):
    """Dense samples along `polygon`'s own boundary, each pushed
    outward by `offset` along its own edge's outward normal -- see
    problog_project/module/theory/planners.py's own docstring for the
    full rationale (per-edge offset, not a true mitred polygon offset;
    outward direction picked empirically per edge, robust to either
    winding). Unchanged from there."""
    vertices = list(polygon)
    if _signed_polygon_area(vertices) > 0.0:
        vertices = list(reversed(vertices))

    samples = []
    for (ax, ay), (bx, by) in _polygon_edges(vertices):
        edx, edy = bx - ax, by - ay
        elen = math.hypot(edx, edy)
        if elen <= 1.0e-9:
            continue
        edx, edy = edx / elen, edy / elen
        n1 = (-edy, edx)
        n2 = (edy, -edx)
        mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
        probe_x, probe_y = mx + n1[0] * _NORMAL_PROBE_EPS, my + n1[1] * _NORMAL_PROBE_EPS
        outward = n1 if not _inside_polygon(probe_x, probe_y, vertices) else n2
        for k in range(_BOUNDARY_SAMPLES_PER_EDGE):
            frac = k / _BOUNDARY_SAMPLES_PER_EDGE
            px, py = ax + edx * elen * frac, ay + edy * elen * frac
            samples.append((px + outward[0] * offset, py + outward[1] * offset))
    return samples


def _follow_boarder_control_points(sx, sy, obstacle_id, offset, obstacle_polygons):
    """Core computation -- unchanged from problog_project's own function
    of the same name, except obstacle_polygons arrives as a parameter
    instead of a module-level global (see module docstring)."""
    polygon = None
    for oid, pts in obstacle_polygons:
        if oid == obstacle_id:
            polygon = pts
            break
    if polygon is None or len(polygon) < 3:
        return None

    boundary = _offset_boundary_clockwise(polygon, offset)
    if not boundary:
        return None

    n = len(boundary)
    start_idx = min(range(n),
                     key=lambda i: (boundary[i][0] - sx) ** 2 + (boundary[i][1] - sy) ** 2)
    path_xy = [(sx, sy)] + [boundary[(start_idx + i) % n] for i in range(n + 1)]

    try:
        tck, _u = fit_spline(path_xy, degree=3, smoothing=0.0)
        control_points, _k = bspline_to_bezier_chain(tck)
    except ValueError:
        control_points = straight_control_points(sx, sy, path_xy[-1][0], path_xy[-1][1])

    return control_points


def follow_boarder_points(sx, sy, obstacle_id, offset, obstacle_polygons):
    """Traces a full clockwise loop around `obstacle_id`'s own boundary,
    offset outward by `offset`, starting and ending at whichever sample
    is nearest the robot's current position -- byte-for-byte port of
    problog_project's own follow_boarder_points (same "no stopping
    condition of its own" contract: which Bug variant a leg implements
    is entirely the subsequent MoveTo leg's own triggers, not this
    planner's decision). `obstacle_polygons` is a
    [(id, [(x,y), ...]), ...] list -- see obstacles_to_polygons above
    for building one from this simulation's circular trees. Returns
    None if obstacle_id names no known obstacle."""
    return _follow_boarder_control_points(
        float(sx), float(sy), str(obstacle_id), float(offset), obstacle_polygons)


# =====================================================================
# VORONOI -- byte-for-byte port of problog_project's own
# _voronoi_sites/_segment_crosses_any_obstacle/_voronoi_roadmap_edges/
# _insert_point_into_roadmap/_dijkstra_shortest_path/
# _voronoi_control_points, taking `obstacle_polygons` as a parameter.
# =====================================================================
def _voronoi_sites(obstacle_polygons):
    """Dense points sampled along every obstacle polygon's own boundary
    -- unchanged from problog_project's own function of the same name."""
    sites = []
    for _oid, poly in obstacle_polygons:
        for (ax, ay), (bx, by) in _polygon_edges(poly):
            for k in range(_VORONOI_SAMPLES_PER_EDGE):
                frac = k / _VORONOI_SAMPLES_PER_EDGE
                sites.append((ax + (bx - ax) * frac, ay + (by - ay) * frac))
    return sites


def _segment_crosses_any_obstacle(ax, ay, bx, by, obstacle_polygons):
    """Unchanged from problog_project's own function of the same name."""
    for k in range(_VORONOI_EDGE_CHECK_SAMPLES + 1):
        frac = k / _VORONOI_EDGE_CHECK_SAMPLES
        x, y = ax + (bx - ax) * frac, ay + (by - ay) * frac
        for _oid, poly in obstacle_polygons:
            if _inside_polygon(x, y, poly):
                return True
    return False


def _closest_point_on_segment(px, py, ax, ay, bx, by):
    sdx, sdy = bx - ax, by - ay
    len2 = sdx * sdx + sdy * sdy
    if len2 <= 1.0e-9:
        return ax, ay
    t = max(0.0, min(1.0, ((px - ax) * sdx + (py - ay) * sdy) / len2))
    return ax + t * sdx, ay + t * sdy


def _voronoi_roadmap_edges(obstacle_polygons):
    """[(p1,p2), ...] -- every Voronoi ridge between two finite vertices
    whose connecting segment doesn't cross any obstacle. Returns None if
    there are too few sites to build a diagram at all. Unchanged from
    problog_project's own function of the same name."""
    sites = _voronoi_sites(obstacle_polygons)
    if len(sites) < 4:
        return None
    try:
        vor = Voronoi(sites)
    except Exception:
        return None

    edges = []
    for v1, v2 in vor.ridge_vertices:
        if v1 == -1 or v2 == -1:
            continue
        p1 = tuple(vor.vertices[v1])
        p2 = tuple(vor.vertices[v2])
        if _segment_crosses_any_obstacle(p1[0], p1[1], p2[0], p2[1], obstacle_polygons):
            continue
        edges.append((p1, p2))
    return edges


def _insert_point_into_roadmap(edges, px, py):
    """Splits the closest edge's closest POINT (not vertex) to (px,py)
    and connects (px,py) to it -- unchanged from problog_project's
    planners.py."""
    if not edges:
        return None
    best = None
    for i, (p1, p2) in enumerate(edges):
        cx, cy = _closest_point_on_segment(px, py, p1[0], p1[1], p2[0], p2[1])
        d2 = (cx - px) ** 2 + (cy - py) ** 2
        if best is None or d2 < best[0]:
            best = (d2, i, cx, cy)
    _d2, idx, cx, cy = best
    p1, p2 = edges[idx]
    new_edges = edges[:idx] + edges[idx + 1:]
    new_edges.append((p1, (cx, cy)))
    new_edges.append(((cx, cy), p2))
    new_edges.append(((px, py), (cx, cy)))
    return new_edges


def _dijkstra_shortest_path(edges, start, goal):
    """Plain Dijkstra over an undirected weighted graph -- unchanged
    from problog_project's planners.py."""
    adj = {}
    for p1, p2 in edges:
        w = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        adj.setdefault(p1, []).append((p2, w))
        adj.setdefault(p2, []).append((p1, w))

    dist = {start: 0.0}
    prev = {}
    visited = set()
    heap = [(0.0, start)]
    while heap:
        d, u = heapq.heappop(heap)
        if u in visited:
            continue
        visited.add(u)
        if u == goal:
            break
        for v, w in adj.get(u, []):
            nd = d + w
            if v not in dist or nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))
    if goal not in dist:
        return None
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    path.reverse()
    return path


def _voronoi_control_points(sx, sy, gx, gy, obstacle_polygons):
    """Core computation -- unchanged from problog_project's own function
    of the same name."""
    start = (sx, sy)
    goal = (gx, gy)
    edges = _voronoi_roadmap_edges(obstacle_polygons)
    if not edges:
        return straight_control_points(sx, sy, gx, gy)

    edges = _insert_point_into_roadmap(edges, sx, sy)
    edges = _insert_point_into_roadmap(edges, gx, gy)

    path = _dijkstra_shortest_path(edges, start, goal)
    if path is None:
        return None

    if len(path) < 2:
        return [(sx, sy)] * 4
    try:
        tck, _u = fit_spline(path, degree=3, smoothing=0.0)
        control_points, _k = bspline_to_bezier_chain(tck)
    except ValueError:
        control_points = straight_control_points(sx, sy, gx, gy)

    return control_points


def plan_voronoi_points(sx, sy, gx, gy, obstacle_polygons):
    """Generalized-Voronoi-diagram planner over `obstacle_polygons` (a
    [(id, [(x,y), ...]), ...] list -- see obstacles_to_polygons above
    for building one from this simulation's circular trees). Degrades
    to a straight line when there are too few obstacles to route
    around; returns None only if a roadmap exists but start/goal are
    genuinely disconnected within it. Byte-for-byte port of
    problog_project's own plan_voronoi_points."""
    return _voronoi_control_points(
        float(sx), float(sy), float(gx), float(gy), obstacle_polygons)
