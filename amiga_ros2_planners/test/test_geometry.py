from amiga_ros2_planners.geometry import (
    distance_to_obstacle_boundary, nearest_obstacle, point_inside_obstacle,
    segment_intersects_circle, line_of_sight_clear,
)
from amiga_ros2_planners.obstacle_types import Obstacle


def test_distance_to_obstacle_boundary_outside():
    o = Obstacle("tree_0", 0.0, 0.0, 1.0)
    assert distance_to_obstacle_boundary(3.0, 0.0, o) == 2.0


def test_distance_to_obstacle_boundary_inside_is_negative():
    o = Obstacle("tree_0", 0.0, 0.0, 1.0)
    assert distance_to_obstacle_boundary(0.5, 0.0, o) < 0.0


def test_point_inside_obstacle():
    o = Obstacle("tree_0", 0.0, 0.0, 1.0)
    assert point_inside_obstacle(0.0, 0.0, o)
    assert not point_inside_obstacle(5.0, 0.0, o)


def test_nearest_obstacle_picks_closest():
    near = Obstacle("tree_near", 1.0, 0.0, 0.3)
    far = Obstacle("tree_far", 10.0, 0.0, 0.3)
    obstacle, clearance = nearest_obstacle(0.0, 0.0, [far, near])
    assert obstacle.id == "tree_near"
    assert clearance == 0.7


def test_nearest_obstacle_empty_list():
    obstacle, clearance = nearest_obstacle(0.0, 0.0, [])
    assert obstacle is None
    assert clearance == float("inf")


def test_segment_intersects_circle():
    assert segment_intersects_circle(-5.0, 0.0, 5.0, 0.0, 0.0, 0.0, 1.0)
    assert not segment_intersects_circle(-5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 1.0)


def test_line_of_sight_clear():
    blocking = Obstacle("tree_0", 0.0, 0.0, 1.0)
    assert not line_of_sight_clear(-5.0, 0.0, 5.0, 0.0, blocking)
    assert line_of_sight_clear(-5.0, 5.0, 5.0, 5.0, blocking)
