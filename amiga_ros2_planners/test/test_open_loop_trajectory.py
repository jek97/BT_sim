import math

from amiga_ros2_planners.open_loop_trajectory import (
    build_velocity_segments, integrate_unicycle_step, wrap_to_pi,
)


def test_wrap_to_pi_no_change_within_range():
    assert abs(wrap_to_pi(0.5) - 0.5) < 1.0e-9
    assert abs(wrap_to_pi(-0.5) - (-0.5)) < 1.0e-9


def test_wrap_to_pi_takes_shortest_turn():
    # +179 deg -> -179 deg is a 2 deg turn, not 358 deg.
    near_pi = math.radians(179)
    near_neg_pi = math.radians(-179)
    assert abs(wrap_to_pi(near_neg_pi - near_pi) - math.radians(2)) < 1.0e-6


def test_build_velocity_segments_straight_line():
    waypoints = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
    segments = build_velocity_segments(waypoints, speed=1.0, max_angular_speed=1.0)
    assert len(segments) == 1
    duration, linear_x, angular_z = segments[0]
    assert abs(duration - 2.0) < 1.0e-9
    assert abs(linear_x - 1.0) < 1.0e-9
    assert abs(angular_z - 0.0) < 1.0e-9


def test_build_velocity_segments_pure_rotation_capped_by_max_angular_speed():
    waypoints = [(0.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2)]
    segments = build_velocity_segments(waypoints, speed=1.0, max_angular_speed=1.0)
    assert len(segments) == 1
    duration, linear_x, angular_z = segments[0]
    assert abs(duration - (math.pi / 2)) < 1.0e-9
    assert abs(linear_x - 0.0) < 1.0e-9
    assert abs(angular_z - 1.0) < 1.0e-9


def test_build_velocity_segments_mixed_uses_longer_duration():
    # 1m forward (1s at speed=1) but a 150 deg turn needing 5pi/6
    # seconds at max_angular_speed=1 -- the rotation dominates, so
    # linear_x ends up SLOWER than the nominal speed to stay
    # synchronized. (150, not 180, deg to avoid the +-pi boundary's
    # own direction ambiguity -- either sign is an equally valid 180
    # deg turn, which would make this test's own sign assertion
    # meaningless.)
    turn = math.radians(150)
    waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, turn)]
    segments = build_velocity_segments(waypoints, speed=1.0, max_angular_speed=1.0)
    duration, linear_x, angular_z = segments[0]
    assert abs(duration - turn) < 1.0e-9
    assert abs(linear_x - (1.0 / turn)) < 1.0e-9
    assert abs(angular_z - 1.0) < 1.0e-9


def test_build_velocity_segments_drops_degenerate_zero_segment():
    waypoints = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    segments = build_velocity_segments(waypoints, speed=1.0, max_angular_speed=1.0)
    assert len(segments) == 1


def test_build_velocity_segments_empty_for_single_waypoint():
    assert build_velocity_segments([(0.0, 0.0, 0.0)], speed=1.0, max_angular_speed=1.0) == []


def test_integrate_unicycle_step_straight_line():
    x, y, yaw = integrate_unicycle_step(0.0, 0.0, 0.0, linear_x=2.0, angular_z=0.0, dt=1.5)
    assert abs(x - 3.0) < 1.0e-9
    assert abs(y - 0.0) < 1.0e-9
    assert abs(yaw - 0.0) < 1.0e-9


def test_integrate_unicycle_step_quarter_circle():
    # linear_x=1, angular_z=1 for pi/2 seconds traces a unit-radius
    # quarter circle from (0,0) heading 0 to (1,1) heading pi/2.
    x, y, yaw = integrate_unicycle_step(
        0.0, 0.0, 0.0, linear_x=1.0, angular_z=1.0, dt=math.pi / 2)
    assert abs(x - 1.0) < 1.0e-9
    assert abs(y - 1.0) < 1.0e-9
    assert abs(yaw - math.pi / 2) < 1.0e-9


def test_integrate_unicycle_step_composable_over_sub_steps():
    # Splitting one step into several smaller ones (same total dt)
    # must land on the exact same final pose -- required for the
    # node's own "advance across a segment boundary mid-tick" logic to
    # be safe.
    whole = integrate_unicycle_step(0.0, 0.0, 0.3, linear_x=1.5, angular_z=0.7, dt=1.0)
    x, y, yaw = 0.0, 0.0, 0.3
    for _ in range(10):
        x, y, yaw = integrate_unicycle_step(x, y, yaw, 1.5, 0.7, 0.1)
    assert abs(x - whole[0]) < 1.0e-9
    assert abs(y - whole[1]) < 1.0e-9
    assert abs(yaw - whole[2]) < 1.0e-9
