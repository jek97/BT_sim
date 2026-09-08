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


def test_voronoi_degrades_to_straight_line_with_no_obstacles():
    control_points = planning_core.plan_voronoi_points(0.0, 0.0, 5.0, 0.0, [])
    assert control_points == planning_core.straight_control_points(0.0, 0.0, 5.0, 0.0)


def test_follow_boarder_unknown_obstacle_returns_none():
    obstacles = [Obstacle("tree_0", 0.0, 0.0, 1.0)]
    assert planning_core.follow_boarder_points(
        5.0, 0.0, "tree_missing", 0.5, obstacles) is None


def test_follow_boarder_returns_a_loop_around_the_tree():
    obstacles = [Obstacle("tree_0", 0.0, 0.0, 1.0)]
    control_points = planning_core.follow_boarder_points(
        3.0, 0.0, "tree_0", 0.5, obstacles)
    assert control_points is not None
    x0, y0 = control_points[0]
    assert abs(x0 - 3.0) < 1.0e-6
    assert abs(y0 - 0.0) < 1.0e-6
