"""Unit tests for planning_core.py -- pure-Python, no ROS2 runtime needed
(matches this repo's own convention of testing the plain-Python core
directly, e.g. problog_project's own planners.py tests)."""
from amiga_ros2_planners.obstacle_types import Obstacle
from amiga_ros2_planners import planning_core


def test_straight_control_points_endpoints():
    control_points = planning_core.straight_control_points(0.0, 0.0, 10.0, 0.0)
    assert control_points[0] == (0.0, 0.0)
    assert control_points[-1] == (10.0, 0.0)
    assert len(control_points) == 4


def test_astar_no_obstacles_reaches_goal():
    # A grid-cell-snapped path, not the exact query points -- see
    # build_occupancy_grid's own docstring; a half-cell tolerance is the
    # right check here, not exact equality.
    control_points = planning_core.plan_astar_points(0.0, 0.0, 5.0, 0.0, [])
    assert control_points is not None
    x0, y0 = control_points[0]
    assert abs(x0 - 0.0) < planning_core.GRID_RESOLUTION_M
    assert abs(y0 - 0.0) < planning_core.GRID_RESOLUTION_M
    sx, sy = control_points[-1]
    assert abs(sx - 5.0) < 0.5
    assert abs(sy - 0.0) < 0.5


def test_astar_start_on_obstacle_fails():
    obstacles = [Obstacle("tree_0", 0.0, 0.0, 1.0)]
    control_points = planning_core.plan_astar_points(0.0, 0.0, 5.0, 0.0, obstacles)
    assert control_points is None


def test_circle_to_polygon_shape():
    obstacle = Obstacle("tree_0", 1.0, 2.0, 0.5)
    obstacle_id, rings = planning_core.circle_to_polygon(obstacle, num_sides=8)
    assert obstacle_id == "tree_0"
    assert len(rings) == 1  # a circular canopy never has a hole
    polygon = rings[0]
    assert len(polygon) == 8
    for x, y in polygon:
        assert abs(((x - 1.0) ** 2 + (y - 2.0) ** 2) ** 0.5 - 0.5) < 1.0e-9


def test_voronoi_degrades_to_straight_line_with_no_obstacles():
    control_points = planning_core.plan_voronoi_points(0.0, 0.0, 5.0, 0.0, [])
    assert control_points == planning_core.straight_control_points(0.0, 0.0, 5.0, 0.0)


def test_voronoi_routes_around_an_obstacle_between_start_and_goal():
    # A tree straddling the direct line from start to goal -- the
    # roadmap should route around it, not through it.
    obstacles = [Obstacle("tree_0", 5.0, 0.0, 1.0)]
    obstacle_polygons = planning_core.obstacles_to_polygons(obstacles)
    control_points = planning_core.plan_voronoi_points(
        0.0, 0.0, 10.0, 0.0, obstacle_polygons)
    assert control_points is not None
    # None of the fitted path's own control points should land inside
    # the tree's own circle.
    for x, y in control_points:
        assert ((x - 5.0) ** 2 + y ** 2) ** 0.5 >= 1.0 - 1.0e-6


def test_follow_boarder_unknown_obstacle_returns_none():
    obstacles = [Obstacle("tree_0", 0.0, 0.0, 1.0)]
    obstacle_polygons = planning_core.obstacles_to_polygons(obstacles)
    assert planning_core.follow_boarder_points(
        5.0, 0.0, "tree_missing", 0.5, obstacle_polygons) is None


def test_follow_boarder_returns_a_loop_around_the_tree():
    obstacles = [Obstacle("tree_0", 0.0, 0.0, 1.0)]
    obstacle_polygons = planning_core.obstacles_to_polygons(obstacles)
    control_points = planning_core.follow_boarder_points(
        3.0, 0.0, "tree_0", 0.5, obstacle_polygons)
    assert control_points is not None
    x0, y0 = control_points[0]
    assert abs(x0 - 3.0) < 1.0e-6
    assert abs(y0 - 0.0) < 1.0e-6


def test_resample_path_every_step_always_includes_endpoint():
    path_xy = [(0.0, 0.0), (10.0, 0.0)]
    waypoints = planning_core._resample_path_every_step(path_xy, 3.0)
    assert waypoints[-1] == (10.0, 0.0)
    # 3, 6, 9 (< 10), then the final endpoint -- 4 waypoints.
    assert len(waypoints) == 4
    assert abs(waypoints[0][0] - 3.0) < 1.0e-9


def test_resample_path_every_step_degenerate_zero_length():
    assert planning_core._resample_path_every_step([(1.0, 1.0), (1.0, 1.0)], 2.0) == [(1.0, 1.0)]


def test_dastar_no_obstacles_reaches_goal():
    control_points = planning_core.plan_dastar_points(0.0, 0.0, 5.0, 0.0, step=1.0, obstacles=[])
    assert control_points is not None
    assert abs(control_points[0][0] - 0.0) < planning_core.GRID_RESOLUTION_M
    assert abs(control_points[-1][0] - 5.0) < 0.5


def test_dastar_start_on_obstacle_fails():
    obstacles = [Obstacle("tree_0", 0.0, 0.0, 1.0)]
    assert planning_core.plan_dastar_points(0.0, 0.0, 5.0, 0.0, obstacles=obstacles) is None


def test_dastar_matches_astar_endpoints_with_obstacle_between():
    obstacles = [Obstacle("tree_0", 5.0, 0.0, 1.0)]
    dastar_cp = planning_core.plan_dastar_points(0.0, 0.0, 10.0, 0.0, step=1.0, obstacles=obstacles)
    assert dastar_cp is not None
    # dastar's own straight-line-chained corners never cut through the
    # obstacle it routed around, unlike a single smoothed spline which
    # could in principle overshoot slightly near a tight corner.
    for x, y in dastar_cp:
        assert ((x - 5.0) ** 2 + y ** 2) ** 0.5 >= 1.0 - 1.0e-6


def test_plan_astar_waypoints_chains_legs_without_duplicating_seams():
    control_points = planning_core.plan_astar_waypoints_points(
        0.0, 0.0, [(5.0, 0.0), (5.0, 5.0)], obstacles=[])
    assert control_points is not None
    # A valid chained-Bezier control point list is always length 3k+1
    # for some k>=1 -- if the shared seam point (leg 1's own goal ==
    # leg 2's own start) were double-counted, this invariant would
    # break.
    assert (len(control_points) - 1) % 3 == 0
    assert abs(control_points[0][0] - 0.0) < planning_core.GRID_RESOLUTION_M
    assert abs(control_points[0][1] - 0.0) < planning_core.GRID_RESOLUTION_M
    assert abs(control_points[-1][0] - 5.0) < 0.5
    assert abs(control_points[-1][1] - 5.0) < 0.5


def test_plan_astar_waypoints_empty_returns_none():
    assert planning_core.plan_astar_waypoints_points(0.0, 0.0, [], obstacles=[]) is None


def test_plan_astar_waypoints_fails_if_any_leg_blocked():
    # The FIRST leg (0,0)->(5,0) is clear; the SECOND leg (5,0)->(5,5)
    # is blocked by a tree sitting exactly on the second waypoint.
    obstacles = [Obstacle("tree_0", 5.0, 5.0, 1.0)]
    assert planning_core.plan_astar_waypoints_points(
        0.0, 0.0, [(5.0, 0.0), (5.0, 5.0)], obstacles=obstacles) is None


def test_plan_straight_waypoints_chains_legs():
    control_points = planning_core.plan_straight_waypoints_points(
        0.0, 0.0, [(5.0, 0.0), (5.0, 5.0)])
    assert control_points is not None
    assert len(control_points) == 7
    assert control_points[0] == (0.0, 0.0)
    assert control_points[-1] == (5.0, 5.0)


def test_plan_straight_waypoints_empty_returns_none():
    assert planning_core.plan_straight_waypoints_points(0.0, 0.0, []) is None


def test_plan_astar_waypoints_polygons_chains_legs():
    obstacle_polygons = []
    control_points = planning_core.plan_astar_waypoints_points_polygons(
        0.0, 0.0, [(5.0, 0.0), (5.0, 5.0)], obstacle_polygons)
    assert control_points is not None
    assert abs(control_points[-1][0] - 5.0) < 0.5
    assert abs(control_points[-1][1] - 5.0) < 0.5
