from amiga_ros2_planners.frame_transform import ProblogFrameTransform


def test_identity_transform_is_a_no_op():
    t = ProblogFrameTransform()
    assert t.to_sim_frame(3.0, 4.0) == (3.0, 4.0)


def test_pure_translation():
    t = ProblogFrameTransform(origin_x=10.0, origin_y=-5.0)
    x, y = t.to_sim_frame(1.0, 2.0)
    assert abs(x - 11.0) < 1.0e-9
    assert abs(y - -3.0) < 1.0e-9


def test_pure_rotation_90_degrees():
    t = ProblogFrameTransform(yaw_deg=90.0)
    x, y = t.to_sim_frame(1.0, 0.0)
    assert abs(x - 0.0) < 1.0e-9
    assert abs(y - 1.0) < 1.0e-9


def test_translation_and_rotation_combined():
    t = ProblogFrameTransform(origin_x=5.0, origin_y=5.0, yaw_deg=180.0)
    x, y = t.to_sim_frame(1.0, 0.0)
    assert abs(x - 4.0) < 1.0e-9
    assert abs(y - 5.0) < 1.0e-9
