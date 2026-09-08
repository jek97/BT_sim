# amiga_ros2_planners

Ports the path-planning and BT-condition logic from the `problog_project`
research repo (see its `module/theory/planners.py`,
`module/contracts/schema.yaml`, `module/contracts/bt_actions.py`) onto
this simulation's own live state — the orchard's own trees (instead of a
static `map.pgm`/`obstacles_generated.pl`) and this robot's own tf2 pose
(instead of a Prolog situation history) — and exposes both as ROS2
services for a future BT.cpp leaf to call.

This package does **not** implement `MoveTo` or `HaltedWith`, does not
register any BT.cpp leaf node in `amiga_ros2_behavior_tree`, and does
not run a robot. It is the backend layer underneath those — see "Known
limitations / what's next" below for exactly what's left.

## What's in here

| File | Status | What it is |
|---|---|---|
| `amiga_ros2_planners/geometry.py` | **new** | Circle-obstacle geometry (distance/nearest/line-of-sight) — the circular-canopy analogue of problog_project's polygon-based `collision_geometry.py`. |
| `amiga_ros2_planners/planning_core.py` | **ported + adapted** | `astar()`, `fit_spline`/`bspline_to_bezier_chain`, `straight_control_points` are byte-for-byte ports of `problog_project/module/theory/planners.py`. `build_occupancy_grid`, the Voronoi site sampling, and `follow_boarder_points` are rewritten for circular tree obstacles instead of arbitrary polygons/a static map file — see the module's own docstring for exactly what changed and why. |
| `amiga_ros2_planners/gps.py` | new | lat/lon → local ENU metres, the forward direction of the same equirectangular approximation `orchard_management.cpp` already uses in reverse. |
| `amiga_ros2_planners/obstacle_types.py` | new | The plain `Obstacle(id, x, y, radius)` shape, split out so the planning/geometry core has no ROS import (same testability goal `bt_actions.py` documents for its own planners.py dependency). |
| `amiga_ros2_planners/orchard_obstacles.py` | new | Subscribes to the same tree-info JSON topic `orchard_management_node` already caches; converts every tree into a circular `Obstacle`. This simulation's replacement for problog_project's static `obstacles_generated.pl`. |
| `amiga_ros2_planners/pose.py` | new | tf2-based current-position lookup. This simulation's replacement for problog_project's `now/2 + at/4` situation fluent. |
| `amiga_ros2_planners/plan_service_node.py` | new | Hosts `PlanPath.srv` — the ROS2-service form of `PlanWith`, dispatching to `planning_core.py` by `algorithm`. |
| `amiga_ros2_planners/condition_service_node.py` | new | Hosts `EvaluateCondition.srv` — the ROS2-service form of every `schema.yaml` Condition **except `HaltedWith`** (excluded on purpose; see its own docstring). |
| `amiga_ros2_planners/battery_sim_node.py` | new | A simulated battery percentage (this simulation has no real one) so `Battery*` conditions have something to read. |
| `amiga_interfaces/srv/PlanPath.srv` | new | Request/response for the planner service. |
| `amiga_interfaces/srv/EvaluateCondition.srv` | new | Request/response for the condition service. |
| `schemas/amiga_btcpp_planners.xsd` | new (local copy) | `amiga_ros2_behavior_tree`'s own `amiga_btcpp.xsd`, extended with `PlanWith`/`MoveTo`/every Condition. A **local copy**, not an edit to the submodule in place — see the file's own header for why (it's vendored from a separate repo this project can't push to). |

## Running it

```bash
ros2 launch amiga_ros2_planners planners.launch.py
# or, namespaced for a multi-robot sim:
ros2 launch amiga_ros2_planners planners.launch.py namespace:=amiga2
```

Starts `plan_service_node`, `condition_service_node`, and
`battery_sim_node`. All three need the orchard already published (i.e.
run alongside `amiga_ros2_behavior_tree`'s own `bt.launch.py`, or at
least `orchard_management_node`) and a live tf2 pose (Nav2/AMCL, or
whatever localization stack is running) to answer anything meaningfully
— before either exists, requests return `reason: "no_pose"`.

Standalone testing without a robot at all:
```bash
ros2 service call /plan_path amiga_interfaces/srv/PlanPath \
  "{algorithm: 'astar', goal_x: 10.0, goal_y: 5.0}"
ros2 service call /evaluate_condition amiga_interfaces/srv/EvaluateCondition \
  "{condition: 'DistanceBelow', goal_x: 10.0, goal_y: 5.0, threshold: 0.5}"
```
(both will answer `no_pose` until something publishes the `map`→`base_link`
transform these default params look for).

### The datum parameter — read this before trusting a planned path

`plan_service_node`/`condition_service_node` both take `datum_lat`/
`datum_lon` params (defaulting to `37.3611`/`-120.4322`, copied from
`amiga-ros2-nav/amiga_localization/config/base_ekf.yaml`'s own `datum:`).
Tree obstacles are converted from lat/lon into local (x,y) around this
same point. **If your localization stack uses a different datum, set
these params to match it** — otherwise tree obstacles will not line up
with the robot's own tf2 pose, and the planners will confidently plan
straight through (or around empty space instead of) real trees.

## Extending `xml_validation` (point 1)

`amiga_ros2_behavior_tree/schemas/amiga_btcpp.xsd` is a **submodule**
(`gpt-mission-planner-schemas`, a separate, externally-owned repo this
project has no push access to) — it was not edited in place. Instead,
`schemas/amiga_btcpp_planners.xsd` here is a full local copy of it with
`PlanWith`, `MoveTo`, and every Condition (including `HaltedWith`, for
tree-authoring completeness even though it's not implemented — see the
table above) added. Every original node type is unchanged, so a mission
using only the original node set still validates against this file.

To run a mission using the new nodes through `bt_runner`, point it at
this schema instead of the submodule's default:
```bash
ros2 launch amiga_ros2_behavior_tree bt.launch.py \
    mission_schema:=$(ros2 pkg prefix amiga_ros2_planners)/share/amiga_ros2_planners/schemas/amiga_btcpp_planners.xsd
```

**Note on `problog_project`'s own tree XML**: its root element
(`<root BTCPP_format="4" main_tree_to_execute="MainTree">`) uses vanilla
BT.cpp conventions, which differ from this repo's own
(`<root BTCPP_format="4" schema_location="...">` plus a required
`<Mission>` description, no `main_tree_to_execute` — see any file in
`amiga_ros2_behavior_tree/examples/` for the shape `bt_runner` actually
expects). A `problog_project` tree needs its root element adapted to
this repo's own convention before `bt_runner` will accept it; the leaf
nodes themselves (`PlanWith`, `MoveTo`, `DistanceBelow`, ...) need no
changes and validate as-is against `amiga_btcpp_planners.xsd` (verified
directly against `problems/problem0/behavior_tree.xml`'s own tree body).

## Known limitations / what's next

- **`MoveTo` has no backend.** This pass was explicitly scoped to
  planners + conditions. A previous conversation in this session sketched
  the intended approach: a `BT::RosServiceNode`/`RosActionNode` leaf in
  `amiga_ros2_behavior_tree` that samples `PlanWith`'s Bezier
  `control_points` into a `nav_msgs/Path` and calls Nav2's
  `controller_server`-only `FollowPath` action (no planner/costmap/
  recovery layers) for open-loop trajectory tracking.
- **`HaltedWith` is not implemented anywhere.** It needs a tree's own
  blackboard/situation history, not live simulation state — a future
  BT.cpp leaf should read this from its own tree directly rather than a
  service (see `EvaluateCondition.srv`'s own header).
- **No BT.cpp leaf nodes exist yet** for any of `PlanWith`/`MoveTo`/the
  nine conditions — `plan_service_node`/`condition_service_node` are
  ROS2 service *backends*, callable today via `ros2 service call` or a
  quick test client, but not yet wired into `bt_runner`'s own
  `BehaviorTreeFactory::registerNodeType<...>()` calls.
- **`ObstacleOnPath` is a partial port** — see
  `condition_service_node.py`'s own `_obstacle_on_path` docstring. Its
  documented semantics need a walk's own future trajectory (i.e.
  `MoveTo`'s own output), which doesn't exist yet; the current
  implementation only checks the robot's *current* position, not its
  planned path.
- **`astar`'s grid is rebuilt on every call** (query-scoped, from the
  live obstacle list — see `build_occupancy_grid`'s own docstring), not
  cached like problog_project's own once-per-process static map. Fine
  for this pass's orchard sizes; worth profiling before scaling to a
  much larger orchard.
