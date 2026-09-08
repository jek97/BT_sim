"""
problog_problem.py

Everything needed to load a problog_project problems/<name>/ directory
directly -- the loader behind "give me a problem folder and run it" (as
opposed to earlier pieces of this package, which required each
adaptation done and calibrated by hand). Pure Python, no ROS import, so
it's usable both from a launch file (computed at launch-generation time,
before any node exists) and from a ROS node (loaded once at startup),
and is unit-testable on its own.

Two files matter for RUNNING a mission (as opposed to verifying one,
which is the rest of problog_project's own pipeline -- goal_formula.pl,
basic_action_theory.pl, ProbLog inference itself, none of which this
loads, since bt_runner never needs a formal proof to tick a tree):

  obstacles_generated.pl -- obstacle_polygon(Id, [point(X,Y), ...])
    facts. Parsed with the SAME regex-based parser problog_project's own
    planners.py/collision_geometry.py use (see _parse_obstacle_polygons
    below -- a direct port, not a reimplementation).

  config.yaml -- specifically initial_situation.start_x/start_y (used
    to auto-calibrate ProblogFrameTransform -- see
    compute_frame_origin's own docstring) and battery.* (used to drive
    battery_sim_node with the SAME drain rates this problem's own theory
    assumes, rather than this package's made-up defaults).

map.yaml/map.pgm are deliberately NOT loaded here: build_polygon_grid
rasterizes A*'s own grid directly from obstacles_generated.pl's
polygons at whatever resolution this simulation wants, so the
problem's own raster image is redundant for RUNNING (as opposed to
verifying) a mission. goal_formula.pl is also not loaded -- it is a
ProbLog verification artifact tied to that problem's own
basic_action_theory.pl, not something bt_runner's execution needs (the
mission's own goal points already live in behavior_tree.xml's own
goal="..." attributes).
"""
import os
import re

import yaml

_POINT_RE = re.compile(r"point\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)")


def _strip_prolog_comments(text):
    return re.sub(r"%.*$", "", text, flags=re.MULTILINE)


def _parse_obstacle_polygons(text):
    """[(id, [(x,y), ...]), ...] -- byte-for-byte port of
    problog_project/module/theory/planners.py's own
    _parse_obstacle_polygons (and collision_geometry.py's identical
    copy of it)."""
    polygons = []
    for m in re.finditer(r"obstacle_polygon\(([^,]+),\s*\[(.*?)\]\s*\)\s*\.", text, re.S):
        obstacle_id = m.group(1).strip()
        points = [(float(x), float(y)) for x, y in _POINT_RE.findall(m.group(2))]
        if len(points) >= 3:
            polygons.append((obstacle_id, points))
    return polygons


def load_obstacle_polygons(problem_dir):
    """[(id, [(x,y), ...]), ...] from <problem_dir>/obstacles_generated.pl,
    or [] if that file doesn't exist (a problem with no obstacles at
    all is valid -- straight-line/no-avoidance scenarios)."""
    path = os.path.join(problem_dir, "obstacles_generated.pl")
    try:
        with open(path) as f:
            text = f.read()
    except FileNotFoundError:
        return []
    return _parse_obstacle_polygons(_strip_prolog_comments(text))


def load_config(problem_dir):
    """The problem's own config.yaml as a plain dict, or {} if it
    doesn't exist."""
    path = os.path.join(problem_dir, "config.yaml")
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}


def compute_frame_origin(config):
    """(origin_x, origin_y, yaw_deg) for ProblogFrameTransform, derived
    from config.yaml's own initial_situation.start_x/start_y with NO
    manual calibration needed: this simulation's robot spawns at this
    sim's own local-frame origin (0,0) (see amiga_ros2_planners' own
    README on the datum/frame conventions elsewhere in this package),
    so mapping the problem's own assumed start point onto (0,0) is
    exactly "the robot starts where the problem's own theory assumed it
    starts" -- the natural, zero-input registration between the two
    frames. Rotation is always 0 -- config.yaml carries no heading for
    its own start pose, so there is nothing to derive a yaw from; pass
    a non-zero yaw to ProblogFrameTransform by hand (or extend this
    function) if a specific problem's own map is known to be rotated
    relative to this simulation's frame.

    Returns (0.0, 0.0, 0.0) -- identity -- if config.yaml has no
    initial_situation section at all."""
    situation = config.get("initial_situation", {})
    start_x = situation.get("start_x", 0.0)
    start_y = situation.get("start_y", 0.0)
    return -start_x, -start_y, 0.0


def battery_params(config):
    """{start_percent, idle_drain_rate_pct_s, moving_drain_rate_pct_s}
    from config.yaml's own battery section, mapped 1:1 onto
    battery_sim_node's own per-second params -- config.yaml documents
    its own rates as "percent per time-unit", and this project's theory
    never fixes how long a time-unit is in real seconds (motion.speed
    is metres per time-unit, not seconds per time-unit); treating one
    time-unit as one real second is the simplest, most direct mapping
    available and is what this function does. Retune
    idle_drain_rate_pct_s/moving_drain_rate_pct_s by hand afterward if a
    specific problem's own pacing turns out to feel wrong in real time.

    Returns battery_sim_node's own defaults, unchanged, if config.yaml
    has no battery section, or if battery.enabled is false (in which
    case problog_project's own semantics are "never halts a walk for a
    battery reason" -- the closest approximation without changing
    battery_sim_node itself is a very slow default drain, which these
    defaults already are)."""
    battery = config.get("battery", {})
    return {
        "start_percent": float(battery.get("start", 100.0)),
        "idle_drain_rate_pct_s": float(battery.get("idle_drain_rate", 0.01)),
        "moving_drain_rate_pct_s": float(battery.get("moving_drain_rate", 0.1)),
    }
