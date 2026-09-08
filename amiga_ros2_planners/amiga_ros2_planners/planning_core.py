"""
planning_core.py

The four PlanWith algorithms (astar/straight/voronoi/follow_boarder)
ported from problog_project/module/theory/planners.py onto this
simulation's own orchard, for plan_service_node.py to call. Ported, not
copied: problog_project's planners work against a static map.pgm/
map.yaml occupancy grid and hand-authored obstacle_polygon/2 facts
loaded once from a problem directory; this simulation has neither --
its obstacles are the orchard's own trees, learned at runtime from
whatever orchard_management_node has cached (see orchard_obstacles.py),
each one circular (canopy center + radius) rather than an arbitrary
polygon.

What transferred UNCHANGED (still the exact same computation, just
copied over): _straight_control_points, the A* search itself (astar()/
_reconstruct_path), and the B-spline-to-Bezier-chain fit
(fit_spline/bspline_to_bezier_chain) -- none of those three cared what
shape the obstacles were.

What changed and WHY:
  - A* needs a grid. There is no map.yaml here, so build_occupancy_grid
    below rasterizes one on the fly from the current obstacle list plus
    the start/goal points, sized just large enough to cover the query
    (see its own docstring) -- a query-scoped grid instead of a
    once-per-process static one, since which trees are relevant changes
    with the robot's own position instead of being fixed at import
    time.
  - Voronoi and follow_boarder both worked from problog_project's own
    _OBSTACLE_POLYGONS (arbitrary polygons, boundary-sampled for sites/
    offsetting). A circular tree's boundary has a closed form, so both
    are SIMPLER here than a mechanical port of the polygon code would
    have been: follow_boarder is exactly a concentric circle at
    radius+offset (no per-edge outward-normal probing needed, see
    problog_project's own _offset_boundary_clockwise for the polygon
    version this replaces), and Voronoi sites are evenly-spaced samples
    around each tree's own circle instead of along its polygon's edges.
    The roadmap-building machinery downstream of "a list of boundary
    sample points" (ridge filtering, closest-point-on-edge insertion,
    Dijkstra) is otherwise the same idea, just no longer needing an
    arbitrary-polygon inside-test -- geometry.py's circle primitives
    stand in for collision_geometry.py's polygon ones.
"""
import heapq
import math

import numpy as np
from scipy.interpolate import splprep, insert
from scipy.spatial import Voronoi

from amiga_ros2_planners.geometry import segment_intersects_circle

# Same defaults as problog_project's planners.py (PLANNING_INFLATE_M/
# OCC_THRESH/CONNECTIVITY there) -- planner-internal tuning, not a
# simulation config value.
PLANNING_INFLATE_M = 0.3
GRID_RESOLUTION_M = 0.25
GRID_MARGIN_M = 3.0
OCC_THRESH = 50
CONNECTIVITY = 8

_VORONOI_SAMPLES_PER_TREE = 12
_VORONOI_EDGE_CHECK_SAMPLES = 6


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
# A* -- astar()/_reconstruct_path unchanged from problog_project's
# planners.py; OccupancyGridMap/build_occupancy_grid replace its
# load_map_yaml (see module docstring for why).
# =====================================================================
class OccupancyGridMap:
    """Same shape as problog_project's own OccupancyGridMap (a minimal
    stand-in for nav_msgs/OccupancyGrid), built here from a live
    obstacle list instead of loaded from a map.pgm/map.yaml pair."""

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
    box of (start, goal, every obstacle's own extent) plus `margin`, so
    the grid always covers this one A* query without needing a
    world-size parameter or a pre-built map. Each obstacle is stamped in
    as a filled disk of its own radius plus `inflate` (the same
    robot-clearance inflation problog_project's planners.py applies via
    inflate_obstacles, just baked directly into the rasterization here
    instead of a separate dilation pass, since obstacles are already
    circles and inflating a circle is just a bigger circle)."""
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


def plan_astar_points(sx, sy, gx, gy, obstacles):
    """Grid-based A* planner over a grid rasterized from `obstacles`
    (see build_occupancy_grid). Returns [(x,y), ...] control points, or
    None if start/goal fall on an obstacle cell or no path exists."""
    sx, sy, gx, gy = float(sx), float(sy), float(gx), float(gy)
    grid = build_occupancy_grid(obstacles, sx, sy, gx, gy)
    start_rc = grid.world_to_grid(sx, sy)
    goal_rc = grid.world_to_grid(gx, gy)

    if start_rc == goal_rc:
        return [(sx, sy)] * 4

    path_rc = astar(grid, start_rc, goal_rc)
    if path_rc is None:
        return None

    path_xy = [grid.grid_to_world(r, c) for r, c in path_rc]
    return _fit_or_straight(path_xy, sx, sy, gx, gy)


# =====================================================================
# VORONOI -- roadmap machinery ported from problog_project's planners.py
# (_voronoi_roadmap_edges/_insert_point_into_roadmap/
# _dijkstra_shortest_path are the same idea, unchanged in spirit);
# _voronoi_sites/_segment_crosses_any_obstacle rewritten for circular
# obstacles (see module docstring).
# =====================================================================
def _voronoi_sites(obstacles):
    """Points evenly sampled around every obstacle's own circle -- the
    circle analogue of problog_project's own per-polygon-edge boundary
    sampling."""
    sites = []
    for obstacle in obstacles:
        for k in range(_VORONOI_SAMPLES_PER_TREE):
            angle = 2.0 * math.pi * k / _VORONOI_SAMPLES_PER_TREE
            sites.append((obstacle.x + obstacle.radius * math.cos(angle),
                          obstacle.y + obstacle.radius * math.sin(angle)))
    return sites


def _segment_crosses_any_obstacle(ax, ay, bx, by, obstacles):
    """True iff the segment (ax,ay)-(bx,by) passes through ANY
    obstacle's own circle -- the filter that turns a plain Voronoi
    tessellation into a free-space roadmap."""
    return any(segment_intersects_circle(ax, ay, bx, by, o.x, o.y, o.radius)
               for o in obstacles)


def _closest_point_on_segment(px, py, ax, ay, bx, by):
    sdx, sdy = bx - ax, by - ay
    len2 = sdx * sdx + sdy * sdy
    if len2 <= 1.0e-9:
        return ax, ay
    t = max(0.0, min(1.0, ((px - ax) * sdx + (py - ay) * sdy) / len2))
    return ax + t * sdx, ay + t * sdy


def _voronoi_roadmap_edges(obstacles):
    """[(p1,p2), ...] -- every Voronoi ridge between two finite vertices
    whose connecting segment doesn't cross any obstacle. Returns None if
    there are too few sites to build a diagram at all."""
    sites = _voronoi_sites(obstacles)
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
        if _segment_crosses_any_obstacle(p1[0], p1[1], p2[0], p2[1], obstacles):
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


def plan_voronoi_points(sx, sy, gx, gy, obstacles):
    """Generalized-Voronoi-diagram planner over `obstacles`. Degrades to
    a straight line when there are too few obstacles to route around;
    returns None only if a roadmap exists but start/goal are genuinely
    disconnected within it."""
    sx, sy, gx, gy = float(sx), float(sy), float(gx), float(gy)
    start, goal = (sx, sy), (gx, gy)
    edges = _voronoi_roadmap_edges(obstacles)
    if not edges:
        return straight_control_points(sx, sy, gx, gy)

    edges = _insert_point_into_roadmap(edges, sx, sy)
    edges = _insert_point_into_roadmap(edges, gx, gy)

    path = _dijkstra_shortest_path(edges, start, goal)
    if path is None:
        return None

    return _fit_or_straight(path, sx, sy, gx, gy)


# =====================================================================
# FOLLOW_BOARDER -- a concentric offset circle, sampled clockwise
# starting nearest the robot's own position, replaces
# problog_project's own per-edge outward-normal boundary offset (see
# module docstring for why the circular case needs no such machinery).
# =====================================================================
_FOLLOW_BOARDER_SAMPLES = 24


def follow_boarder_points(sx, sy, obstacle_id, offset, obstacles):
    """Traces a full clockwise loop around `obstacle_id`'s own canopy,
    offset outward by `offset`, starting and ending at whichever sample
    is nearest the robot's current position -- same "no stopping
    condition of its own" contract as problog_project's
    follow_boarder_points (see that function's own docstring: which Bug
    variant a leg implements is entirely the subsequent MoveTo leg's own
    triggers, not this planner's decision). Returns None if obstacle_id
    names no known tree."""
    sx, sy, offset = float(sx), float(sy), float(offset)
    obstacle = next((o for o in obstacles if o.id == obstacle_id), None)
    if obstacle is None:
        return None

    radius = obstacle.radius + offset
    n = _FOLLOW_BOARDER_SAMPLES
    # Angle from the tree's own center to the robot decides where the
    # loop starts; walking DECREASING angle traces the circle clockwise
    # in a standard x-right/y-up frame.
    start_angle = math.atan2(sy - obstacle.y, sx - obstacle.x)
    boundary = [
        (obstacle.x + radius * math.cos(start_angle - 2.0 * math.pi * k / n),
         obstacle.y + radius * math.sin(start_angle - 2.0 * math.pi * k / n))
        for k in range(n + 1)
    ]
    path_xy = [(sx, sy)] + boundary

    return _fit_or_straight(path_xy, sx, sy, boundary[-1][0], boundary[-1][1])
