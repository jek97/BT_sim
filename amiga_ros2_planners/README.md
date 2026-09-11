# amiga_ros2_planners

Ports the path-planning and BT-condition logic from the `problog_project`
research repo (see its `module/theory/planners.py`,
`module/contracts/schema.yaml`, `module/contracts/bt_actions.py`) onto
this simulation's own live state — the orchard's own trees (instead of a
static `map.pgm`/`obstacles_generated.pl`) and this robot's own tf2 pose
(instead of a Prolog situation history) — and exposes `PlanWith`, `MoveTo`,
and every Condition except `HaltedWith` as ROS2 services/actions for a
future BT.cpp leaf to call.

The corresponding BT.cpp leaves are now registered in
`amiga_ros2_behavior_tree/src/bt.cpp` (see that package's own README),
but **have not been compiled or run** — no ROS2/`behaviortree_ros2`
toolchain was available in this session — and `MoveTo`'s own
`FollowPath` wiring has not been exercised against a live Nav2 stack
either. See "Known limitations / what's next" below.

## Running a `problog_project` problem folder directly

```bash
ros2 launch amiga_ros2_planners run_problog_problem.launch.py \
    problem_dir:=/path/to/BT_project/problems/problem0
```

One command, one required argument. This launch file reads the problem
folder itself and does everything else on its own:
- Computes the goal-point frame alignment automatically from
  `config.yaml`'s own `initial_situation.start_x`/`start_y` — no manual
  calibration (see "Goal-point frame alignment" below for the exact
  reasoning).
- Reads that problem's own `battery.*` drain rates from `config.yaml`
  and drives `battery_sim_node` with them, instead of this package's
  made-up defaults.
- Adapts `behavior_tree.xml`'s root element automatically
  (`adapt_tree.py`) and brings up Gazebo + Nav2 +
  `amiga_ros2_behavior_tree` + every node in this package, with
  `obstacle_source:=problog_problem` so planning/conditions reason
  against THAT PROBLEM's own `obstacles_generated.pl` polygons —
  not the live orchard's trees.
- Sends the adapted mission over TCP itself (`send_mission.py`,
  retrying until `tcp_demux_node` is up) — no manual `nc` step.

What's still not automatic: `HaltedWith` (if the tree uses it) and,
above all, **the BT.cpp leaves are registered but unbuilt** in this
session — see "Running a `problog_project` BT in this simulation —
checklist" and "Known limitations" below for the complete, honest
state. This launch file gets every OTHER piece running for you; it
can't make an unbuilt leaf tick.

## What's in here

| File | Status | What it is |
|---|---|---|
| `amiga_ros2_planners/geometry.py` | new | Circle-obstacle geometry (distance/nearest/line-of-sight) used by `condition_service_node.py`. |
| `amiga_ros2_planners/planning_core.py` | **ported + adapted** | `astar()`, `fit_spline`/`bspline_to_bezier_chain`, `straight_control_points`, and the full Voronoi roadmap machinery and `follow_boarder` (`_polygon_edges`, `_edge_crosses`, `_inside_polygon`, `_signed_polygon_area`, `_offset_boundary_clockwise`, `_voronoi_sites`, `_voronoi_roadmap_edges`, `_dijkstra_shortest_path`, ...) are **byte-for-byte ports** of `problog_project/module/theory/planners.py` — same function names, same bodies, operating on the same `[(id, [(x,y), ...]), ...]` polygon shape `problog_project`'s own `_OBSTACLE_POLYGONS` uses. The only genuinely new pieces are `circle_to_polygon`/`obstacles_to_polygons` (this simulation's one obstacle source is circular tree canopies, not polygons) and `build_occupancy_grid`/`build_grid_map`/`OccupancyGridMap` (A* needs a grid; there's no `map.yaml` here — see `orchard_map.py` below). |
| `amiga_ros2_planners/bezier.py` | new | Resamples a chained-cubic-Bezier control point list into an `(x, y, yaw)` polyline via the analytic Bezier derivative — what turns `PlanPath`'s own output into something `nav_msgs/Path` (and so `MoveTo`) can use. Pure math, no ROS import, unit-tested. |
| `amiga_ros2_planners/orchard_map.py` | new | Converts `planning_core.OccupancyGridMap` to/from `nav_msgs/OccupancyGrid` (the actual grid-building is `planning_core.build_grid_map`, kept ROS-free — see below). |
| `amiga_ros2_planners/orchard_map_node.py` | new | Publishes the whole-orchard grid once at startup (and again if the orchard is ever republished with a different tree count) — see "2D map extraction" below. |
| `amiga_ros2_planners/gps.py` | new | lat/lon → local ENU metres, the forward direction of the same equirectangular approximation `orchard_management.cpp` already uses in reverse. |
| `amiga_ros2_planners/obstacle_types.py` | new | The plain `Obstacle(id, x, y, radius)` shape, split out so the planning/geometry core has no ROS import. |
| `amiga_ros2_planners/orchard_obstacles.py` | new | Subscribes to the same tree-info JSON topic `orchard_management_node` already caches; converts every tree into a circular `Obstacle`. Accepts both JSON shapes this repo's own fixtures carry (see its own docstring). |
| `amiga_ros2_planners/pose.py` | new | tf2-based current-position lookup. This simulation's replacement for problog_project's `now/2 + at/4` situation fluent. |
| `amiga_ros2_planners/plan_service_node.py` | new | Hosts `PlanPath.srv` — the ROS2-service form of `PlanWith`, dispatching to `planning_core.py` by `algorithm`. |
| `amiga_ros2_planners/condition_service_node.py` | new | Hosts `EvaluateCondition.srv` — the ROS2-service form of every `schema.yaml` Condition **except `HaltedWith`**. |
| `amiga_ros2_planners/battery_sim_node.py` | new | A simulated battery percentage (this simulation has no real one) so `Battery*` conditions have something to read. |
| `amiga_ros2_planners/move_to_node.py` | new | Hosts `MoveTo` (action) — samples `control_points` into a `nav_msgs/Path` and drives it through Nav2's `controller_server` `FollowPath` action; polls a real subset of `triggers` against `condition_service_node` and cancels early if one fires. Also applies this walk's own tool-dependent speed (`tool_speed_*_mps`) as a live `desired_linear_vel` override on `controller_server`, based on `tool_action_node`'s own latched `tool_state` topic. |
| `amiga_ros2_planners/sample_service_node.py` | new | Hosts `TakeSample.srv` — the ROS2-service form of `TakeSample`: one `random.random() < success_probability` draw, the literal formula `problog_project/module/contracts/bt_actions.py`'s own `bt_take_sample` uses. On success, also draws a VALUE (0-10, `sample.value.mean`/`.sigma`, a discretized Normal) keyed by the tree's own `id="..."` port, and republishes every id's own value in full on a latched `sample_values` topic (JSON) — `condition_service_node` subscribes to answer `SampleValueBelow`/`Equal`/`Over`. |
| `amiga_ros2_planners/tool_action_node.py` | new | Hosts `InstallTool`/`UninstallTool`/`DeployTool`/`RetractTool` (actions) — fixed-Duration, no motion, same start/halt/triggers shape as `MoveTo` minus the trajectory. `tool` names a specific tool INSTANCE id (config.yaml's `tool.instances: [{id,kind,x,y}]`, resolved to a kind here), and `InstallTool` additionally requires the robot be within `install_range` of that instance's own declared position. `DeployTool`/`RetractTool` lower/raise an already-installed tool (currently plow-only) between which `ploughed`-style effects would apply. Tracks the currently-equipped tool's kind ("free"/"cart"/"plow"), which specific instance id, and whether it's deployed, publishing all of it on latched `tool_state`/`tool_activity`/`tool_deployed` topics `move_to_node`/`battery_sim_node` both subscribe to (selecting `tool.equipped.<kind>.deployed_speed`/`.deployed_moving_drain_rate` instead of the regular value while deployed). |
| `amiga_interfaces/srv/PlanPath.srv`, `EvaluateCondition.srv`, `TakeSample.srv`, `action/MoveTo.action`, `InstallTool.action`, `UninstallTool.action`, `DeployTool.action`, `RetractTool.action` | new | Interfaces for the backends above. |
| `schemas/amiga_btcpp_planners.xsd` | new (local copy) | `amiga_ros2_behavior_tree`'s own `amiga_btcpp.xsd`, extended with `PlanWith`/`MoveTo`/`TakeSample`/`InstallTool`/`UninstallTool`/`DeployTool`/`RetractTool`/`Repeat`/every Condition except `HaltedWith` (deliberately excluded — see the file's own header), plus `Inverter` and a broadened `Fallback` (see "Running a problog_project BT" below for why). A **local copy**, not an edit to the submodule in place. |
| `scripts/export_orchard_map.py` | new | Standalone (no ROS2 needed), extracts a real orchard from a checked-in mission fixture and writes a `map.pgm`/`map.yaml` pair — see "Saved orchard map instance" below. |
| `maps/orchard_map.pgm`, `maps/orchard_map.yaml`, `maps/orchard_map_trees.json` | new (checked in) | A saved instance of the real 144-tree orchard, in `problog_project`'s own `map.yaml` format — see below. |
| `amiga_ros2_planners/frame_transform.py` | new | Maps a goal point from a `problog_project` problem's own map frame into this sim's live frame — see "Goal-point frame alignment" below. |
| `amiga_ros2_planners/obstacle_types.py`'s `resolve_obstacle_id` | new | Resolves an obstacle id against this orchard's own tree ids, falling back to a bare tree-index match (`"obs5"`/`"5"` → `tree_5`) — see "Resolving obstacle ids against tree ids" below. |
| `amiga_ros2_planners/polygon_geometry.py` | new | Polygon-obstacle geometry (distance/nearest/segment-intersection), unit-tested — the analogue of `geometry.py`'s circle primitives, for a `problog_project` problem's own genuine polygon obstacles. |
| `amiga_ros2_planners/planning_core.py`'s `build_polygon_grid`/`plan_astar_points_polygons` | new | A*'s own polygon-obstacle rasterizer/planner, alongside the existing circle-based `build_occupancy_grid` — used when `obstacle_source:=problog_problem`. |
| `amiga_ros2_planners/problog_problem.py` | new, unit-tested | Loads a problem folder directly: `obstacles_generated.pl` (byte-for-byte port of `problog_project`'s own regex parser) into polygons, `config.yaml`'s `initial_situation`/`battery` sections into the frame-origin and battery-rate auto-calibration — see "Running a `problog_project` problem folder directly" above. |
| `amiga_ros2_planners/adapt_tree.py` | new (moved from `scripts/adapt_problog_tree.py`) | The tree root-element adapter, now a plain importable function (`adapt_and_write`) so `run_problog_problem.launch.py` can call it directly, plus a `ros2 run amiga_ros2_planners adapt_tree` CLI. |
| `amiga_ros2_planners/send_mission.py` | new | Sends a mission XML file to `tcp_demux_node` over TCP, retrying until it's up — the automated form of the `nc` step, a `ros2 run amiga_ros2_planners send_mission` console script. |
| `plan_service_node.py`/`condition_service_node.py`'s `obstacle_source` param | new | `"orchard"` (default) or `"problog_problem"` — switches every planner/condition to reason against a problem folder's own `obstacles_generated.pl` polygons instead of the live orchard's circular trees. |
| `amiga_ros2_planners/launch/problog_sim_bringup.launch.py` | new | Brings up Gazebo + Nav2 + `bt.launch.py` (pointed at this package's schema) + every node in `planners.launch.py`, for a single robot, with `obstacle_source`/frame/battery params threaded through — see checklist item 7 below. |
| `amiga_ros2_planners/launch/run_problog_problem.launch.py` | new | The one-argument, fully automated entry point — see "Running a `problog_project` problem folder directly" above. |
| `amiga_ros2_behavior_tree/src/actions/plan_with.{hpp,cpp}`, `move_to.{hpp,cpp}`, `evaluate_condition_base.{hpp,cpp}`, `evaluate_conditions.{hpp,cpp}`, `point_port.hpp` | new | The BT.cpp leaves themselves, registered in `bt.cpp` — see that package's own README for their current (untested) status. |

## Running it

```bash
ros2 launch amiga_ros2_planners planners.launch.py
# or, namespaced for a multi-robot sim:
ros2 launch amiga_ros2_planners planners.launch.py namespace:=amiga2
```

Starts `orchard_map_node`, `plan_service_node`, `condition_service_node`,
`move_to_node`, and `battery_sim_node`. All need the orchard already
published (i.e. run alongside `amiga_ros2_behavior_tree`'s own
`bt.launch.py`, or at least `orchard_management_node`) and a live tf2
pose (Nav2/AMCL) to answer anything meaningfully — before either exists,
requests return `reason: "no_pose"` (and `astar` falls back to a
query-scoped grid with a logged warning until `orchard_map_node` has
published one). `move_to_node` additionally needs Nav2's
`controller_server` running and reachable at its own `follow_path_action`
param (default `follow_path`).

Standalone testing without a robot at all:
```bash
ros2 service call /plan_path amiga_interfaces/srv/PlanPath \
  "{algorithm: 'astar', goal_x: 10.0, goal_y: 5.0}"
ros2 service call /evaluate_condition amiga_interfaces/srv/EvaluateCondition \
  "{condition: 'DistanceBelow', goal_x: 10.0, goal_y: 5.0, threshold: 0.5}"
ros2 topic echo /orchard/occupancy_grid --once  # the extracted 2D map
ros2 action send_goal /move_to amiga_interfaces/action/MoveTo \
  "{control_points: [{x: 0.0, y: 0.0}, {x: 1.0, y: 0.0}, {x: 2.0, y: 0.0}, {x: 3.0, y: 0.0}]}"
```

### The datum parameter — read this before trusting a planned path

`plan_service_node`/`condition_service_node`/`orchard_map_node` all take
`datum_lat`/`datum_lon` params (defaulting to `37.3611`/`-120.4322`,
copied from `amiga-ros2-nav/amiga_localization/config/base_ekf.yaml`'s
own `datum:`). Tree obstacles are converted from lat/lon into local
(x,y) around this same point. **If your localization stack uses a
different datum, set these params to match it** — otherwise tree
obstacles will not line up with the robot's own tf2 pose.

## Do we reason over a 3D map of the orchard?

**No.** Nothing in this package reads the Gazebo world's own mesh, point
clouds, or any other 3D representation. "The orchard" has always meant
the tree list's own lat/lon metadata (the same JSON
`orchard_management_node` caches from `tcp_demux_node`'s second TCP
frame), which `gps.py` converts straight into flat local `(x,y)` — a 2D
abstraction from the start, the same way `problog_project`'s own
`map.pgm` never had a Z axis either.

### 2D map extraction at startup

`orchard_map_node.py` builds a single whole-orchard occupancy grid once
(waiting for the orchard's first JSON payload, rebuilding only if the
tree count changes) and publishes it as a standard
`nav_msgs/OccupancyGrid` on `orchard/occupancy_grid` with
`TRANSIENT_LOCAL` durability (matching Nav2's own `map_server`).
`plan_service_node`'s `astar` consumes this directly, falling back to a
per-query grid with a logged warning if it hasn't arrived yet. Voronoi,
`follow_boarder`, and every condition still reason against the exact
circle/polygon geometry rather than this raster grid, for precision —
see `planning_core.py`'s own module docstring.

### Saved orchard map instance

`maps/orchard_map.pgm` + `maps/orchard_map.yaml` are a checked-in,
ready-to-use instance of this simulation's real 144-tree orchard (18
columns × 8 rows), extracted from
`amiga_ros2_behavior_tree/examples/mv_10_60_sample.bin`'s own orchard
JSON frame — the exact same fixture `scripts/demo_llm_auction.sh`
describes as "the full 144-tree orchard". Written by
`scripts/export_orchard_map.py`, in the **exact format**
`problog_project`'s own `load_map_yaml` expects (verified directly by
round-tripping the file through that same load logic in this session).
`maps/orchard_map_trees.json` carries the raw tree list plus the datum
used, for provenance/regeneration.

**To use it in `BT_project`**: point `BT_PROBLEM_DIR` at a directory
containing `map.yaml` (rename/symlink `orchard_map.yaml`→`map.yaml`,
`orchard_map.pgm`→`map.pgm`, or edit the `image:` key in the `.yaml`),
alongside that problem's own `obstacles_generated.pl`/`config.yaml`/
`goal_formula.pl`. **The datum used here (the orchard's own first tree's
lat/lon — see the script's own `--datum-lat`/`--datum-lon` docs) is an
arbitrary local origin, not this simulation's own live `datum_lat`/
`datum_lon`** — so `config.yaml`'s own `start_x`/`start_y` and any
`goal="X;Y"` value need to be expressed in THIS map's frame (origin at
`maps/orchard_map.yaml`'s own `origin:` value), not carried over from
whatever frame a different map used.

To regenerate against a different fixture or resolution:
```bash
python3 scripts/export_orchard_map.py \
    --bin ../amiga_ros2_behavior_tree/examples/sample_20_64.bin \
    --out maps --map-name orchard_map --resolution 0.1
```

## Goal-point frame alignment

Yes — `frame_transform.py`'s `ProblogFrameTransform` maps a goal point
authored in a `problog_project` problem's own map frame into this
sim's live frame. It's a plain 2D rigid transform (translate by
`problog_frame_origin_x`/`_y`, rotate by `problog_frame_yaw_deg`), not a
tf2 lookup: both frames are static for a run's whole lifetime (no robot
ever moves "the problog map"), so a fixed, once-calibrated offset is all
there is to it. `plan_service_node`/`condition_service_node` both apply
it to every `goal_x`/`goal_y` they receive (for `astar`/`straight`/
`voronoi` and for `DistanceBelow`/`Equal`/`Over`/`LineOfSightClear`
respectively) — identity by default (params default to `0.0`), so a
mission already authored against this sim's own orchard is unaffected.

**To calibrate it**: locate one shared physical landmark — or just the
robot's own start pose — in both the problem's map (`config.yaml`'s own
`start_x`/`start_y`, or `maps/orchard_map.yaml`'s `origin:` if you used
"Saved orchard map instance" below) and this sim's live frame (read the
robot's own tf2 `map`→`base_link` transform at that same physical spot),
then set `problog_frame_origin_x`/`_y` to where the problog map's own
`(0,0)` sits in this sim's frame, and `problog_frame_yaw_deg` to the
rotation between the two. **Both nodes' params must agree** — a launch
file passing the same three values to both (as `planners.launch.py`
already does) is the way to guarantee that, rather than setting them
node-by-node with `ros2 param set`.

## Resolving obstacle ids against tree ids

Yes — `orchard_obstacles.py`'s `get_obstacle` (via
`obstacle_types.resolve_obstacle_id`) tries an exact id match first
(this simulation's own `tree_<tree_index>` convention), then falls back
to extracting the first run of digits from whatever id it was given and
matching that against a tree_index. So a `problog_project` tree's own
`obstacle_id="obs5"` (meaningless here — `obs5` names a hand-authored
polygon in that problem's own `obstacles_generated.pl`, not anything
this orchard has) resolves to `tree_5` anyway, on the working assumption
that "obstacle number 5" is what was meant and this orchard's own
numbering is the only one that actually exists at runtime. A bare `"5"`
resolves the same way, and an already-correct `"tree_5"` matches
exactly without ever touching the fallback. Both `follow_boarder`
(`plan_service_node`) and `LineOfSightClear` (`condition_service_node`)
go through this resolution, so checklist item 4 below is now handled
automatically for any tree using the `obsN`/bare-number convention —
only a genuinely different naming scheme would still need remapping.

## Is battery drain action-dependent?

It already is, correctly — but through motion, not through which BT
node happens to be ticking, which is the more faithful choice:
`problog_project`'s own `config.yaml` only ever models TWO drain rates,
`idle_drain_rate` and `moving_drain_rate` — it has no notion of
per-action rates at all, only "is the base currently walking a spline or
not." `battery_sim_node` mirrors exactly that: it watches the robot's
own `odometry/filtered/local` speed and switches between
`idle_drain_rate_pct_s`/`moving_drain_rate_pct_s` accordingly. This
already produces the right behavior with zero action-awareness: a
`MoveTo` leg driving real `cmd_vel` through `FollowPath` drains at the
moving rate; `PlanWith` (instantaneous, no motion) and `SampleLeaf`
(stationary) drain at the idle rate; even a `MoveTo` that stalls for
some reason correctly falls back to the idle rate, which an
action-identity-based rule would miss entirely. The one thing this
doesn't (and, per `problog_project`'s own model, shouldn't need to)
account for is the Kinova arm's own motion during
`MoveArmToPosition` — arm movement never appears in `problog_project`'s
theory at all, so `battery_sim_node` not distinguishing it is consistent
with the model being ported, not a gap in porting it. Say if you want a
separate arm-motion drain rate added on top; it isn't part of the
ported model as it stands.

## Running a `problog_project` BT in this simulation — checklist

`run_problog_problem.launch.py` (above) now automates every item below
marked "automated" -- this checklist is what it does FOR you, and what's
still genuinely left, not a set of manual steps you need to perform. Use
it if you want to understand or debug what's happening, or if you're
running `problog_sim_bringup.launch.py` by hand against a mission you
adapted yourself.

1. **Root element shape** — automated (`adapt_tree.adapt_and_write`,
   called by `run_problog_problem.launch.py`; standalone CLI:
   ```bash
   ros2 run amiga_ros2_planners adapt_tree -- \
       --in .../problems/problem0/behavior_tree.xml --out adapted.xml
   ```
   ). Adds the `<Mission>` element and `schema_location` attribute this
   repo's schema requires; `main_tree_to_execute` is left as-is
   (`amiga_btcpp_planners.xsd` now accepts it, since `bt_runner` never
   reads it for a single-`<BehaviorTree>` file anyway). Verified against
   all five checked-in `problog_project` problems (`problem0`–`problem4`)
   — every one now validates against `amiga_btcpp_planners.xsd` after
   this step alone, **including two base-schema gaps this revision also
   fixed while testing that**: `<Inverter>` (BT.cpp's own negation
   decorator, used inline by `problem3`) wasn't declared as a node type
   at all, and `<Fallback>` only accepted `<Sequence>` children (`problem4`
   puts a bare `<ReactiveSequence>` there) — both now fixed in
   `amiga_btcpp_planners.xsd` for every tree, not just `problog_project`'s.

2. **BT.cpp leaf registration — done, but UNCOMPILED.** `PlanWith`,
   `MoveTo`, and the nine implemented conditions are now registered in
   `bt_runner`'s `BehaviorTreeFactory` (`amiga_ros2_behavior_tree/src/bt.cpp`,
   backed by `src/actions/plan_with.cpp`/`move_to.cpp`/
   `evaluate_condition_base.cpp`/`evaluate_conditions.cpp`). **This has
   not been built** — no ROS2/`behaviortree_ros2` toolchain was available
   in this session — so treat it as "should work, following this
   package's own existing leaf patterns exactly," not "verified." Build
   `amiga_ros2_behavior_tree` and fix whatever the compiler finds before
   trusting it.

3. **Goal points — automated.** A `goal="11.675;11.525"` value is
   authored in that PROBLEM's own map frame, not this simulation's live
   orchard/tf2 frame. `run_problog_problem.launch.py` computes
   `problog_frame_origin_x`/`_y` itself from `config.yaml`'s own
   `initial_situation.start_x`/`start_y` (see "Goal-point frame
   alignment" above for exactly why that's the right registration, with
   no manual calibration); `plan_service_node`/`condition_service_node`
   then apply `ProblogFrameTransform` to every goal they receive.
   Running `problog_sim_bringup.launch.py` directly still needs these
   three params set by hand.

4. **`obstacle_id` values — automated, and more precisely than before.**
   With `obstacle_source:=problog_problem` (which
   `run_problog_problem.launch.py` sets), `follow_boarder`/
   `LineOfSightClear` resolve ids directly against that PROBLEM's own
   `obstacles_generated.pl` names — exact matches, since the mission and
   the obstacle list come from the same file. (The `orchard` obstacle
   source's own numeric fallback -- see "Resolving obstacle ids against
   tree ids" above -- still exists for the other case: a mission written
   against the live orchard whose `obstacle_id`s don't match this sim's
   own `tree_<n>` naming.)

5. **`HaltedWith`** — if the tree uses it, it needs to be removed or
   replaced: `amiga_btcpp_planners.xsd` rejects it outright, and nothing
   implements it (see "Do we reason over a 3D map" section's sibling
   note in `EvaluateCondition.srv` for why it's structurally excluded,
   not just missing). Not automated — no safe automatic rewrite exists.

6. **Battery rates — automated.** `run_problog_problem.launch.py` reads
   that problem's own `config.yaml` `battery.start`/`idle_drain_rate`/
   `moving_drain_rate` and drives `battery_sim_node` with them (see "Is
   battery drain action-dependent" above for why the underlying
   idle/moving model was already correct — this just makes the RATES
   match that specific problem instead of this package's own defaults).
   `battery.enabled: false` has no dedicated handling; `battery_sim_node`
   always publishes, which is harmless if the tree never checks it.

7. **Everything running at once — automated, one command:**
   ```bash
   ros2 launch amiga_ros2_planners run_problog_problem.launch.py \
       problem_dir:=/path/to/BT_project/problems/problem0
   ```
   (or `problog_sim_bringup.launch.py` directly, with every param above
   set by hand, if you've already adapted the tree yourself). Brings up
   Gazebo + Nav2 (`launch_nav:=true`, for `MoveTo`'s own `FollowPath` —
   see that launch file's own note on why this is Nav2's full stack
   rather than a bespoke `controller_server`-only bringup) + `bt.launch.py`
   (pointed at `amiga_btcpp_planners.xsd`, `expect_json`/
   `payload_length_included:=false`) + every node in `planners.launch.py`
   (`obstacle_source:=problog_problem`), single robot — then sends the
   adapted mission itself. `launch_coordination`/`launch_agents` are off
   — this is a fleet-of-one test bench, not a multi-robot auction
   scenario.

## The `MoveTo` → Nav2 `FollowPath` wrapper

Yes, this needed exactly the wrapper you were describing, and it's what
`move_to_node.py` now is: `MoveTo`'s own "trajectory" is a
chained-cubic-Bezier control point list (`PlanPath`'s own output shape),
but Nav2's `controller_server` `FollowPath` action (the "extremely low
level" Nav2 primitive this simulation uses for `MoveTo`, bypassing
`planner_server`/`bt_navigator`/recoveries entirely) takes a
`nav_msgs/Path` — a plain sequence of stamped poses, not Bezier control
points. `move_to_node.py`:

1. Samples `control_points` into an `(x, y, yaw)` polyline via
   `bezier.py` (the curve's own analytic tangent → yaw, not
   finite-differenced), and builds a `nav_msgs/Path` from it.
2. Sends that `Path` as a `FollowPath` goal, forwarding its own
   `distance_to_goal` feedback back out as `MoveTo`'s own feedback.
3. While `FollowPath` runs, polls a real subset of `triggers`
   (`obstacle_in_bound(T)`, `obstacle_on_path(T)`, `battery_below(T)`,
   `battery_over(T)`, `battery_equal(T)` — see `TRIGGER_FUNCTOR_TO_CONDITION`
   in the module) against `condition_service_node`'s `EvaluateCondition`,
   cancelling `FollowPath` and reporting that trigger as `MoveTo`'s own
   `reason` the moment one fires.

**Not carried over** (see `move_to_node.py`'s own module docstring for
the full reasoning): the automatic collision/battery triggers
`problog_project`'s own `bt_to_prolog.py` injects into every leg on the
Prolog side (nothing injects them here — only what `triggers` the Goal
itself lists gets checked); `line_of_sight_clear(...)`/
`crosses_segment(...)` (need a goal point this action's own Goal has no
slot for); and the structural ReactiveSequence-sibling guard derivation
schema.yaml describes (a BT.cpp-tree-structure concept, meaningless at
this node's level — it belongs in a future BT.cpp `MoveTo` leaf, not
here).

**This has not been run against a live Nav2 `controller_server`** — no
ROS2/Nav2 environment was available in this session to exercise it.
`bezier.py`'s own sampling is unit-tested and verified (endpoint
accuracy, yaw-follows-tangent, no duplicated segment-boundary points);
the `FollowPath` action-client integration itself should be smoke-tested
against your own sim before relying on it — in particular the nested
`rclpy.spin_until_future_complete`/`spin_once` calls inside the action's
own execute callback, a common but not bulletproof rclpy pattern for
"call a service/action from inside another action's callback" (see the
node's own comment on why a `ReentrantCallbackGroup` + per-goal execute
thread makes it safe here, rather than a genuinely async rewrite).

## Known limitations / what's next

- **The C++ BT.cpp leaves are unbuilt** — see checklist item 2 above.
  This is now the single largest gap between "backends + leaves exist"
  and "a `problog_project` tree verifiably runs end to end in Gazebo":
  a build, plus an actual tick of a real tree, hasn't happened.
- **`move_to_node`'s `FollowPath` integration is untested** — see above.
- **`HaltedWith` is not implemented anywhere**, and is excluded from
  `xml_validation` entirely.
- **`ObstacleOnPath` is a partial port** — see
  `condition_service_node.py`'s own `_obstacle_on_path` docstring. Its
  documented semantics need a walk's own future trajectory, which
  `move_to_node` doesn't expose anywhere yet; the current implementation
  only checks the robot's *current* position.
- **`MoveTo`'s own trigger vocabulary is a real subset**, not the full
  one `schema.yaml` documents — see "Not carried over" above.
- **The circle→polygon approximation (`_CIRCLE_POLYGON_SIDES = 16`) is a
  tunable, not exact** — raise it in `planning_core.py` if a future
  scenario needs tighter precision very close to a tree's own canopy edge.
- **`run_problog_problem.launch.py`'s own orchestration is unexercised**
  — no ROS2/`launch` runtime + Gazebo/Nav2 were available together in
  this session to actually run it end to end. Its own building blocks
  ARE individually verified: `adapt_tree`/`problog_problem` were run
  directly against all five checked-in problems (see the checklist
  above), and `send_mission.py` was smoke-tested against a plain TCP
  server. What's unverified is specifically the launch-file plumbing
  itself (the `OpaqueFunction`/`ExecuteProcess` wiring, and whether
  `mission_send_timeout`'s default is actually enough time for Gazebo+
  Nav2 to come up on a given machine) — expect to tune timing before it
  works unattended on yours.
- **`build_polygon_grid`'s point-in-polygon rasterization is O(cells ×
  edges)** — fine at `problog_project` problem scale (tens of metres,
  single-digit-vertex obstacles), untested at anything larger; lower
  `GRID_RESOLUTION_M` or add a bounding-box pre-check per obstacle if a
  much bigger/finer problem ever makes this slow.
