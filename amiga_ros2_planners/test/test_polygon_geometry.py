from amiga_ros2_planners.polygon_geometry import (
    distance_to_polygon_boundary, point_inside_polygon, nearest_polygon_obstacle,
    segment_intersects_polygon, line_of_sight_clear_polygon,
)

SQUARE = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]


def test_point_inside_polygon():
    assert point_inside_polygon(1.0, 1.0, SQUARE)
    assert not point_inside_polygon(5.0, 5.0, SQUARE)


def test_distance_to_polygon_boundary_outside():
    d = distance_to_polygon_boundary(3.0, 1.0, SQUARE)
    assert abs(d - 1.0) < 1.0e-9


def test_distance_to_polygon_boundary_inside_is_negative():
    d = distance_to_polygon_boundary(1.0, 1.0, SQUARE)
    assert d < 0.0


def test_nearest_polygon_obstacle():
    far = ("far", [(x + 100.0, y) for x, y in SQUARE])
    near = ("near", SQUARE)
    obstacle_id, clearance = nearest_polygon_obstacle(3.0, 1.0, [far, near])
    assert obstacle_id == "near"
    assert abs(clearance - 1.0) < 1.0e-9


def test_nearest_polygon_obstacle_empty():
    obstacle_id, clearance = nearest_polygon_obstacle(0.0, 0.0, [])
    assert obstacle_id is None
    assert clearance == float("inf")


def test_segment_intersects_polygon_crossing():
    assert segment_intersects_polygon(-1.0, 1.0, 3.0, 1.0, SQUARE)


def test_segment_intersects_polygon_miss():
    assert not segment_intersects_polygon(-1.0, 5.0, 3.0, 5.0, SQUARE)


def test_segment_intersects_polygon_wholly_inside():
    assert segment_intersects_polygon(0.5, 0.5, 1.5, 1.5, SQUARE)


def test_line_of_sight_clear_polygon():
    assert not line_of_sight_clear_polygon(-1.0, 1.0, 3.0, 1.0, SQUARE)
    assert line_of_sight_clear_polygon(-1.0, 5.0, 3.0, 5.0, SQUARE)
