# amiga_ros2_planners

Ports the path-planning and BT-condition logic from the `problog_project`
research repo (see its `module/theory/planners.py`,
`module/contracts/schema.yaml`, `module/contracts/bt_actions.py`) onto
this simulation's own live state — the orchard's own trees (instead of a
static `map.pgm`/`obstacles_generated.pl`) and this robot's own tf2 pose
(instead of a Prolog situation history) — and exposes `PlanWith`, `MoveTo`,
and every Condition except `HaltedWith` as ROS2 services/actions for a
future BT.cpp leaf to call.

This package does **not** register any BT.cpp leaf node in
`amiga_ros2_behavior_tree`, and `MoveTo`'s own `FollowPath` wiring has not
been exercised against a live Nav2 stack. It is the backend layer
underneath a BT.cpp tree — see "Known limitations / what's next" below.

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
| `amiga_ros2_planners/move_to_node.py` | new | Hosts `MoveTo` (action) — samples `control_points` into a `nav_msgs/Path` and drives it through Nav2's `controller_server` `FollowPath` action; polls a real subset of `triggers` against `condition_service_node` and cancels early if one fires. **Not yet exercised against a live Nav2 stack** — see its own module docstring and "Known limitations" below. |
| `amiga_interfaces/srv/PlanPath.srv`, `EvaluateCondition.srv`, `action/MoveTo.action` | new | Interfaces for the three backends above. |
| `schemas/amiga_btcpp_planners.xsd` | new (local copy) | `amiga_ros2_behavior_tree`'s own `amiga_btcpp.xsd`, extended with `PlanWith`/`MoveTo`/every Condition except `HaltedWith` (deliberately excluded — see the file's own header), plus `Inverter` and a broadened `Fallback` (see "Running a problog_project BT" below for why). A **local copy**, not an edit to the submodule in place. |
| `scripts/export_orchard_map.py` | new | Standalone (no ROS2 needed), extracts a real orchard from a checked-in mission fixture and writes a `map.pgm`/`map.yaml` pair — see "Saved orchard map instance" below. |
| `scripts/adapt_problog_tree.py` | new | Standalone, adapts a `problog_project` tree's root element to this repo's own convention — see "Running a problog_project BT" below. |
| `maps/orchard_map.pgm`, `maps/orchard_map.yaml`, `maps/orchard_map_trees.json` | new (checked in) | A saved instance of the real 144-tree orchard, in `problog_project`'s own `map.yaml` format — see below. |

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

## Running a `problog_project` BT in this simulation — checklist

Given one of `BT_project/problems/<name>/behavior_tree.xml`, here's
everything between it and actually ticking in `bt_runner`:

1. **Root element shape** — mechanical, automated:
   ```bash
   python3 scripts/adapt_problog_tree.py \
       --in .../problems/problem0/behavior_tree.xml --out adapted.xml
   ```
   Adds the `<Mission>` element and `schema_location` attribute this
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

2. **BT.cpp leaf registration — the largest remaining gap.** `PlanWith`,
   `MoveTo`, and the eight implemented conditions have **ROS2
   service/action backends** (this package) but **no C++ leaf node**
   registered in `bt_runner`'s `BehaviorTreeFactory`. Until leaves like
   `BT::RosServiceNode<PlanPath>` / `BT::RosActionNode<MoveTo>` are
   written (following the exact pattern of every existing leaf in
   `amiga_ros2_behavior_tree/src/actions/`) and added to `bt.cpp`'s own
   `registerNodeType<...>()` calls, `bt_runner` will fail to build the
   tree (`factory.createTreeFromText` throws "unknown node type") even
   though the XML itself now validates. Nothing in this package does
   this — it's the next concrete step, and hasn't been started.

3. **Goal points are in the wrong frame.** A `goal="11.675;11.525"`
   value is expressed in that PROBLEM's own `map.yaml` frame (or,
   equivalently, whatever frame you exported in "Saved orchard map
   instance" above) — **not** this simulation's live orchard/tf2 frame.
   Either re-express every goal point in this sim's own local (x,y) (the
   same frame `plan_service_node`'s `datum_lat`/`datum_lon` establish),
   or make sure the map you're testing against uses the SAME frame the
   goal points were authored in.

4. **`obstacle_id` values won't resolve.** `follow_boarder`/
   `LineOfSightClear` reference obstacle ids from that problem's own
   `obstacles_generated.pl` (e.g. `obs5`). This simulation's own ids are
   `tree_<tree_index>` (`orchard_obstacles.py`). Remap any such id before
   running the tree, or the planner/condition will report
   `no_obstacle`/`no_such_obstacle`.

5. **`HaltedWith`** — if the tree uses it, it needs to be removed or
   replaced: `amiga_btcpp_planners.xsd` now rejects it outright (per your
   own instruction), and nothing implements it.

6. **Battery is always live here**, unlike `problog_project`'s own
   per-problem `config.yaml` `battery.enabled` toggle — `battery_sim_node`
   always publishes a draining percentage, so `Battery*`
   conditions/triggers are always evaluable regardless of what that
   problem's own config said. Harmless if the tree never checks battery;
   worth noting if a problem was authored assuming `battery.enabled: false`.

7. **Everything needs to actually be running**: this package's own
   `planners.launch.py`, `amiga_ros2_behavior_tree`'s `bt.launch.py`
   (pointed at `amiga_btcpp_planners.xsd` via `mission_schema:=...`),
   Nav2's `controller_server` (for `MoveTo`), and a live localization
   stack publishing tf2 (every planner/condition call resolves "current
   position" from tf2, `problog_project`'s own `now/2 + at/4`
   equivalent).

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

- **No BT.cpp leaf nodes exist yet** for `PlanWith`/`MoveTo`/the eight
  implemented conditions — see checklist item 2 above. This is now the
  single largest gap between "backends exist" and "a `problog_project`
  tree actually runs end to end in Gazebo."
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
