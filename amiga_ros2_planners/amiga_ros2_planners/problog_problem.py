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

  obstacles_generated.pl -- obstacle_polygon(Id, [point(X,Y), ...]) facts,
    plus zero or more obstacle_hole(Id, [point(X,Y), ...]) facts per Id
    (a hollow, walkable interior within that same obstacle -- e.g. a
    perimeter fence's own inner face). Parsed with the SAME regex-based
    parsers problog_project's own planners.py/collision_geometry.py use
    (see _parse_obstacle_polygons/_parse_obstacle_holes below -- direct
    ports, not reimplementations). load_obstacle_polygons folds each
    Id's own hole(s) back in as extra rings, exactly like those two
    modules' own OBSTACLE_POLYGONS/_OBSTACLE_POLYGONS module-level
    globals -- see load_obstacle_polygons's own docstring for the
    [(id, rings), ...] shape every consumer here now expects.

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
    """[(id, [(x,y), ...]), ...] -- each obstacle's own OUTER boundary
    only. Byte-for-byte port of
    problog_project/module/theory/planners.py's own
    _parse_obstacle_polygons (and collision_geometry.py's identical
    copy of it) -- see _parse_obstacle_holes below for how a hole (if
    any) gets folded back in."""
    polygons = []
    for m in re.finditer(r"obstacle_polygon\(([^,]+),\s*\[(.*?)\]\s*\)\s*\.", text, re.S):
        obstacle_id = m.group(1).strip()
        points = [(float(x), float(y)) for x, y in _POINT_RE.findall(m.group(2))]
        if len(points) >= 3:
            polygons.append((obstacle_id, points))
    return polygons


def _parse_obstacle_holes(text):
    """{id: [hole_points, ...]} -- byte-for-byte port of
    problog_project's own _parse_obstacle_holes (planners.py/
    collision_geometry.py's identical copies), matching
    obstacle_hole(Id, [point(X,Y), ...]) facts."""
    holes = {}
    for m in re.finditer(r"obstacle_hole\(([^,]+),\s*\[(.*?)\]\s*\)\s*\.", text, re.S):
        obstacle_id = m.group(1).strip()
        points = [(float(x), float(y)) for x, y in _POINT_RE.findall(m.group(2))]
        if len(points) >= 3:
            holes.setdefault(obstacle_id, []).append(points)
    return holes


def load_obstacle_polygons(problem_dir):
    """[(id, rings), ...] from <problem_dir>/obstacles_generated.pl, or
    [] if that file doesn't exist (a problem with no obstacles at all
    is valid -- straight-line/no-avoidance scenarios). rings is
    [outer_points, hole1_points, ...] -- rings[0] is ALWAYS the
    obstacle's own outer boundary, rings[1:] its own hole(s), if any
    (empty for the common, hole-less case) -- exactly the shape
    problog_project's own OBSTACLE_POLYGONS/_OBSTACLE_POLYGONS
    module-level globals use (collision_geometry.py/planners.py), so
    every consumer here (planning_core.py's polygon rasterizer/
    planners, polygon_geometry.py's clearance/line-of-sight checks)
    ports their ring+hole-aware containment test byte-for-byte too,
    rather than re-introducing the "perimeter fence's own hollow
    interior reads as solid" bug problog_project itself just fixed
    (see occgrid_to_problog.py's own module docstring)."""
    path = os.path.join(problem_dir, "obstacles_generated.pl")
    try:
        with open(path) as f:
            text = f.read()
    except FileNotFoundError:
        return []
    text = _strip_prolog_comments(text)
    holes_by_id = _parse_obstacle_holes(text)
    return [(obstacle_id, [outer_points] + holes_by_id.get(obstacle_id, []))
            for obstacle_id, outer_points in _parse_obstacle_polygons(text)]


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


_TOOL_KINDS = ("cart", "plow")
_DEFAULT_TOOL_DURATION_S = 10.0
_DEFAULT_TOOL_SUCCESS_PROBABILITY = 0.9


def sample_params(config):
    """{success_probability, value_mean, value_sigma} for
    sample_service_node's own TakeSample backend, from config.yaml's
    own sample.success_probability/sample.value.mean/sample.value.sigma
    -- same 1:1 mapping and defaults (0.5/5.0/2.0) as
    module/translators/config_to_prolog.py's own render_prolog/
    _discretized_normal_block (the ProbLog-facing counterpart of these
    same knobs)."""
    sample_cfg = config.get("sample", {})
    value_cfg = sample_cfg.get("value", {})
    return {
        "success_probability": float(sample_cfg.get("success_probability", 0.5)),
        "value_mean": float(value_cfg.get("mean", 5.0)),
        "value_sigma": float(value_cfg.get("sigma", 2.0)),
    }


def ploughing_params(config):
    """{cell_size} for move_to_node's/condition_service_node's own
    shared `plough_cell_size` param, from config.yaml's own ploughing.
    cell_size -- 1.0 (metres) if missing entirely, a harmless default
    for a problem that never configures ploughing (and so never uses
    PloughedAt/PloughedBetween either -- see basic_action_theory.pl's
    own plough_cell_size/1 note on why that predicate must still be
    'known', even undefined, for such a problem)."""
    ploughing_cfg = config.get("ploughing", {})
    return {"cell_size": float(ploughing_cfg.get("cell_size", 1.0))}


def tool_params(config):
    """Every config.yaml knob tool_action_node/move_to_node/
    battery_sim_node need for InstallTool/UninstallTool and their
    effect on a SUBSEQUENT MoveTo -- mirrors
    module/translators/config_to_prolog.py's own render_prolog (the
    ProbLog-facing counterpart of every one of these) key-for-key, same
    per-tool-independent-default style throughout:

      install_duration_s/uninstall_duration_s: {cart, plow} seconds,
        from tool.install.duration_seconds.<tool>/tool.uninstall.
        duration_seconds.<tool>, each defaulting independently to
        _DEFAULT_TOOL_DURATION_S if its own key (or the whole tool:
        section) is missing.
      install_success_probability/uninstall_success_probability: from
        tool.install.success_probability/tool.uninstall.
        success_probability, defaulting to
        _DEFAULT_TOOL_SUCCESS_PROBABILITY.
      install_drain_rate_pct_s/uninstall_drain_rate_pct_s: from
        tool.install.drain_rate/tool.uninstall.drain_rate, defaulting to
        battery.idle_drain_rate (config_to_prolog.py's own "a problem
        with no tool: section at all keeps behaving exactly as before"
        rationale).
      speed/moving_drain_rate_pct_s: {free, cart, plow}, from
        tool.equipped.<tool>.speed/.moving_drain_rate, each tool
        defaulting INDEPENDENTLY to motion.speed/battery.
        moving_drain_rate if its own key is missing -- free ALWAYS
        equals the base value (there is no config.yaml key for "no
        tool equipped", same as config_to_prolog.py's own
        tool_speed(free,_)/tool_moving_drain_rate(free,_) facts).
      deploy_duration_s/retract_duration_s/deploy_success_probability/
        retract_success_probability/deploy_drain_rate_pct_s/
        retract_drain_rate_pct_s: exact mirror of the install/uninstall
        knobs above, from tool.deploy.*/tool.retract.* -- see
        DeployTool/RetractTool's own schema.yaml entry.
      deployed_speed/deployed_moving_drain_rate_pct_s: {cart, plow}
        (no "free" entry -- nothing is ever deployed while unequipped),
        from tool.equipped.<tool>.deployed_speed/.deployed_moving_
        drain_rate, each defaulting to that SAME tool's own regular
        speed/moving_drain_rate above if not separately overridden
        (basic_action_theory.pl's own effective_tool_speed/3 default).
      tool_instances: {id: {kind, x, y}}, from tool.instances (a list
        of {id, kind, x, y} entries in config.yaml, reshaped into a
        dict keyed by id here since that's how tool_action_node.py
        looks them up) -- id NOT validated against duplicates/kind
        values here (config_to_prolog.py already does that at
        generation time for the ProbLog side; a live BT run just fails
        the relevant action's own precondition on a bad entry instead,
        same "unsatisfied precondition, not a validation error" shape
        as everything else in this function).
      install_range: metres, from tool.install.range, defaulting to
        robot.radius+robot.safety_buffer (config_to_prolog.py's own
        install_tool_range/1 default -- "close enough that the robot's
        own body reaches it" is the least arbitrary default available
        without a real robot/tool geometry model)."""
    battery = config.get("battery", {})
    motion = config.get("motion", {})
    robot = config.get("robot", {})
    install_cfg = config.get("tool", {}).get("install", {})
    uninstall_cfg = config.get("tool", {}).get("uninstall", {})
    deploy_cfg = config.get("tool", {}).get("deploy", {})
    retract_cfg = config.get("tool", {}).get("retract", {})
    equipped_cfg = config.get("tool", {}).get("equipped", {})

    install_duration_cfg = install_cfg.get("duration_seconds", {})
    uninstall_duration_cfg = uninstall_cfg.get("duration_seconds", {})
    deploy_duration_cfg = deploy_cfg.get("duration_seconds", {})
    retract_duration_cfg = retract_cfg.get("duration_seconds", {})

    base_speed = float(motion.get("speed", 1.0))
    base_moving_drain_rate = float(battery.get("moving_drain_rate", 0.1))
    idle_drain_rate = float(battery.get("idle_drain_rate", 0.01))

    speed = {
        "free": base_speed,
        **{tool: float(equipped_cfg.get(tool, {}).get("speed", base_speed))
           for tool in _TOOL_KINDS},
    }
    moving_drain_rate_pct_s = {
        "free": base_moving_drain_rate,
        **{tool: float(equipped_cfg.get(tool, {}).get("moving_drain_rate", base_moving_drain_rate))
           for tool in _TOOL_KINDS},
    }

    tool_instances = {}
    for entry in config.get("tool", {}).get("instances", []):
        tool_instances[str(entry["id"])] = {
            "kind": str(entry["kind"]),
            "x": float(entry["x"]),
            "y": float(entry["y"]),
        }

    return {
        "install_duration_s": {
            tool: float(install_duration_cfg.get(tool, _DEFAULT_TOOL_DURATION_S))
            for tool in _TOOL_KINDS
        },
        "uninstall_duration_s": {
            tool: float(uninstall_duration_cfg.get(tool, _DEFAULT_TOOL_DURATION_S))
            for tool in _TOOL_KINDS
        },
        "deploy_duration_s": {
            tool: float(deploy_duration_cfg.get(tool, _DEFAULT_TOOL_DURATION_S))
            for tool in _TOOL_KINDS
        },
        "retract_duration_s": {
            tool: float(retract_duration_cfg.get(tool, _DEFAULT_TOOL_DURATION_S))
            for tool in _TOOL_KINDS
        },
        "install_success_probability": float(
            install_cfg.get("success_probability", _DEFAULT_TOOL_SUCCESS_PROBABILITY)),
        "uninstall_success_probability": float(
            uninstall_cfg.get("success_probability", _DEFAULT_TOOL_SUCCESS_PROBABILITY)),
        "deploy_success_probability": float(
            deploy_cfg.get("success_probability", _DEFAULT_TOOL_SUCCESS_PROBABILITY)),
        "retract_success_probability": float(
            retract_cfg.get("success_probability", _DEFAULT_TOOL_SUCCESS_PROBABILITY)),
        "install_drain_rate_pct_s": float(install_cfg.get("drain_rate", idle_drain_rate)),
        "uninstall_drain_rate_pct_s": float(uninstall_cfg.get("drain_rate", idle_drain_rate)),
        "deploy_drain_rate_pct_s": float(deploy_cfg.get("drain_rate", idle_drain_rate)),
        "retract_drain_rate_pct_s": float(retract_cfg.get("drain_rate", idle_drain_rate)),
        "speed": speed,
        "moving_drain_rate_pct_s": moving_drain_rate_pct_s,
        "deployed_speed": {
            tool: float(equipped_cfg.get(tool, {}).get("deployed_speed", speed[tool]))
            for tool in _TOOL_KINDS
        },
        "deployed_moving_drain_rate_pct_s": {
            tool: float(equipped_cfg.get(tool, {}).get(
                "deployed_moving_drain_rate", moving_drain_rate_pct_s[tool]))
            for tool in _TOOL_KINDS
        },
        "tool_instances": tool_instances,
        "install_range": float(install_cfg.get(
            "range", round(float(robot.get("radius", 0.5)) +
                            float(robot.get("safety_buffer", 0.5)), 6))),
    }
