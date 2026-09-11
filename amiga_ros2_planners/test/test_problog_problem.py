import os
import tempfile

from amiga_ros2_planners import problog_problem as pp


def test_parse_obstacle_polygons_simple():
    text = """
    % a comment, should be stripped
    obstacle_polygon(obs1, [point(0.0,0.0), point(1.0,0.0), point(1.0,1.0)]).
    obstacle_polygon(obs2, [point(2.0,2.0), point(3.0,2.0), point(3.0,3.0), point(2.0,3.0)]).
    """
    polygons = pp._parse_obstacle_polygons(pp._strip_prolog_comments(text))
    assert [oid for oid, _pts in polygons] == ["obs1", "obs2"]
    assert polygons[0][1] == [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]


def test_load_obstacle_polygons_missing_file_returns_empty():
    with tempfile.TemporaryDirectory() as d:
        assert pp.load_obstacle_polygons(d) == []


def test_load_obstacle_polygons_from_disk():
    with tempfile.TemporaryDirectory() as d:
        with open(os.path.join(d, "obstacles_generated.pl"), "w") as f:
            f.write("obstacle_polygon(obsA, [point(0,0), point(1,0), point(1,1)]).\n")
        polygons = pp.load_obstacle_polygons(d)
        assert len(polygons) == 1
        assert polygons[0][0] == "obsA"
        # rings = [outer_points] -- no obstacle_hole/2 fact for obsA, so
        # this hole-less obstacle carries only its own outer boundary.
        assert polygons[0][1] == [[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]]


def test_load_obstacle_polygons_folds_holes_into_rings():
    with tempfile.TemporaryDirectory() as d:
        with open(os.path.join(d, "obstacles_generated.pl"), "w") as f:
            f.write(
                "obstacle_polygon(obs1, [point(0,0), point(10,0), point(10,10), point(0,10)]).\n"
                "obstacle_hole(obs1, [point(4,4), point(6,4), point(6,6), point(4,6)]).\n"
            )
        polygons = pp.load_obstacle_polygons(d)
        assert len(polygons) == 1
        obstacle_id, rings = polygons[0]
        assert obstacle_id == "obs1"
        # rings[0] is always the outer boundary, rings[1:] its holes.
        assert len(rings) == 2
        assert rings[0] == [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
        assert rings[1] == [(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)]


def test_load_config_missing_file_returns_empty_dict():
    with tempfile.TemporaryDirectory() as d:
        assert pp.load_config(d) == {}


def test_compute_frame_origin_from_config():
    config = {"initial_situation": {"start_x": 2.275, "start_y": 2.075}}
    origin_x, origin_y, yaw = pp.compute_frame_origin(config)
    assert abs(origin_x - -2.275) < 1.0e-9
    assert abs(origin_y - -2.075) < 1.0e-9
    assert yaw == 0.0


def test_compute_frame_origin_defaults_to_identity():
    assert pp.compute_frame_origin({}) == (0.0, 0.0, 0.0)


def test_battery_params_from_config():
    config = {"battery": {"start": 80, "idle_drain_rate": 0.05, "moving_drain_rate": 0.5}}
    params = pp.battery_params(config)
    assert params["start_percent"] == 80.0
    assert params["idle_drain_rate_pct_s"] == 0.05
    assert params["moving_drain_rate_pct_s"] == 0.5


def test_battery_params_defaults_when_missing():
    params = pp.battery_params({})
    assert params["start_percent"] == 100.0


def test_sample_params_from_config():
    assert pp.sample_params({"sample": {"success_probability": 0.75}}) == {
        "success_probability": 0.75, "value_mean": 5.0, "value_sigma": 2.0}


def test_sample_params_defaults_when_missing():
    assert pp.sample_params({}) == {
        "success_probability": 0.5, "value_mean": 5.0, "value_sigma": 2.0}


def test_tool_params_defaults_when_missing():
    params = pp.tool_params({})
    assert params["install_duration_s"] == {"cart": 10.0, "plow": 10.0}
    assert params["uninstall_duration_s"] == {"cart": 10.0, "plow": 10.0}
    assert params["install_success_probability"] == 0.9
    assert params["uninstall_success_probability"] == 0.9
    # No battery: section either -- drain rate falls back to
    # battery_params' own idle_drain_rate default.
    assert params["install_drain_rate_pct_s"] == 0.01
    assert params["uninstall_drain_rate_pct_s"] == 0.01
    assert params["speed"] == {"free": 1.0, "cart": 1.0, "plow": 1.0}
    assert params["moving_drain_rate_pct_s"] == {"free": 0.1, "cart": 0.1, "plow": 0.1}


def test_tool_params_per_tool_overrides():
    config = {
        "motion": {"speed": 2.0},
        "battery": {"idle_drain_rate": 0.02, "moving_drain_rate": 0.2},
        "tool": {
            "install": {
                "success_probability": 0.8,
                "drain_rate": 0.03,
                "duration_seconds": {"cart": 5.0},
            },
            "uninstall": {"duration_seconds": {"plow": 7.0}},
            "equipped": {
                "cart": {"speed": 0.5, "moving_drain_rate": 0.4},
            },
        },
    }
    params = pp.tool_params(config)
    # cart overridden, plow falls back to the default duration.
    assert params["install_duration_s"] == {"cart": 5.0, "plow": 10.0}
    # uninstall has no success_probability/drain_rate overrides of its
    # own -- still defaults independently of install's own overrides.
    assert params["uninstall_duration_s"] == {"cart": 10.0, "plow": 7.0}
    assert params["uninstall_success_probability"] == 0.9
    assert params["uninstall_drain_rate_pct_s"] == 0.02
    assert params["install_drain_rate_pct_s"] == 0.03
    # cart overridden, plow and free fall back to motion.speed/
    # battery.moving_drain_rate.
    assert params["speed"] == {"free": 2.0, "cart": 0.5, "plow": 2.0}
    assert params["moving_drain_rate_pct_s"] == {"free": 0.2, "cart": 0.4, "plow": 0.2}
