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
