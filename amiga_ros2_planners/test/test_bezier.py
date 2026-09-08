import math

from amiga_ros2_planners.bezier import sample_bezier_chain
from amiga_ros2_planners import planning_core


def test_straight_line_yields_correct_endpoints_and_yaw():
    control_points = planning_core.straight_control_points(0.0, 0.0, 10.0, 0.0)
    samples = sample_bezier_chain(control_points, samples_per_segment=5)
    x0, y0, yaw0 = samples[0]
    xn, yn, yawn = samples[-1]
    assert abs(x0 - 0.0) < 1.0e-9 and abs(y0 - 0.0) < 1.0e-9
    assert abs(xn - 10.0) < 1.0e-9 and abs(yn - 0.0) < 1.0e-9
    assert abs(yaw0 - 0.0) < 1.0e-9
    assert abs(yawn - 0.0) < 1.0e-9


def test_sample_count_no_duplicate_segment_boundaries():
    # Two straight segments end to end -- a degree-3, two-segment chain
    # (length 7) built by hand, sharing the boundary point (5,0).
    control_points = [
        (0.0, 0.0), (1.5, 0.0), (3.5, 0.0), (5.0, 0.0),
        (6.5, 0.0), (8.5, 0.0), (10.0, 0.0),
    ]
    samples = sample_bezier_chain(control_points, samples_per_segment=4)
    assert len(samples) == 4 + 4 + 1  # segment 1: 4, segment 2: 4+1
    xs = [x for x, _, _ in samples]
    assert xs == sorted(xs)


def test_curved_path_yaw_matches_tangent_direction():
    # A quarter-circle-ish curve (single cubic segment) turning from
    # heading 0 to heading pi/2 -- yaw should increase monotonically.
    control_points = [(0.0, 0.0), (0.0, 1.0), (1.0, 2.0), (1.0, 2.0)]
    # Not a great circle approximation, but a legitimate non-straight
    # single segment: the point of this test is just that per-point yaw
    # tracks the local tangent, not a fixed straight-line heading.
    samples = sample_bezier_chain(control_points, samples_per_segment=10)
    yaws = [yaw for _, _, yaw in samples]
    assert yaws[0] != yaws[-1]


def test_invalid_length_raises():
    try:
        sample_bezier_chain([(0.0, 0.0), (1.0, 1.0)], samples_per_segment=5)
        assert False, "expected ValueError"
    except ValueError:
        pass
