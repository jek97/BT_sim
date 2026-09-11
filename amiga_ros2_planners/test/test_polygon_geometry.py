from amiga_ros2_planners.polygon_geometry import (
    distance_to_polygon_boundary, point_inside_polygon, nearest_polygon_obstacle,
    segment_intersects_polygon, line_of_sight_clear_polygon,
)

SQUARE = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
RING = [SQUARE]

# A 10x10 square ring with a 2x2 hole in its middle -- a perimeter-fence
# style obstacle with a hollow, walkable interior.
OUTER = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
HOLE = [(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)]
RING_WITH_HOLE = [OUTER, HOLE]


def test_point_inside_polygon():
    assert point_inside_polygon(1.0, 1.0, RING)
    assert not point_inside_polygon(5.0, 5.0, RING)


def test_point_inside_polygon_hole_is_free():
    # Inside the outer boundary but inside the hole too -- the hollow
    # interior must read as free (not part of the obstacle's solid
    # material), not "inside the obstacle".
    assert not point_inside_polygon(5.0, 5.0, RING_WITH_HOLE)
    # Inside the outer boundary and outside the hole -- the obstacle's
    # actual solid material (the ring itself).
    assert point_inside_polygon(1.0, 1.0, RING_WITH_HOLE)
    # Outside the outer boundary entirely.
    assert not point_inside_polygon(50.0, 50.0, RING_WITH_HOLE)


def test_distance_to_polygon_boundary_outside():
    d = distance_to_polygon_boundary(3.0, 1.0, RING)
    assert abs(d - 1.0) < 1.0e-9


def test_distance_to_polygon_boundary_inside_is_negative():
    d = distance_to_polygon_boundary(1.0, 1.0, RING)
    assert d < 0.0


def test_nearest_polygon_obstacle():
    far = ("far", [[(x + 100.0, y) for x, y in SQUARE]])
    near = ("near", RING)
    obstacle_id, clearance = nearest_polygon_obstacle(3.0, 1.0, [far, near])
    assert obstacle_id == "near"
    assert abs(clearance - 1.0) < 1.0e-9


def test_nearest_polygon_obstacle_empty():
    obstacle_id, clearance = nearest_polygon_obstacle(0.0, 0.0, [])
    assert obstacle_id is None
    assert clearance == float("inf")


def test_segment_intersects_polygon_crossing():
    assert segment_intersects_polygon(-1.0, 1.0, 3.0, 1.0, RING)


def test_segment_intersects_polygon_miss():
    assert not segment_intersects_polygon(-1.0, 5.0, 3.0, 5.0, RING)


def test_segment_intersects_polygon_wholly_inside():
    assert segment_intersects_polygon(0.5, 0.5, 1.5, 1.5, RING)


def test_segment_intersects_polygon_wholly_inside_hole_is_clear():
    # A segment sitting entirely inside the hole's own free interior
    # must NOT count as blocked.
    assert not segment_intersects_polygon(4.5, 4.5, 5.5, 5.5, RING_WITH_HOLE)


def test_line_of_sight_clear_polygon():
    assert not line_of_sight_clear_polygon(-1.0, 1.0, 3.0, 1.0, RING)
    assert line_of_sight_clear_polygon(-1.0, 5.0, 3.0, 5.0, RING)
