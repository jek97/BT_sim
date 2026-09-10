"""
obstacle_types.py

Just the Obstacle shape (id, x, y, radius) plus obstacle_id resolution,
split out from orchard_obstacles.py so planning_core.py/geometry.py and
their tests never need to import rclpy/std_msgs -- the same
"plain-Python core with no ROS import anywhere" testability goal
problog_project's own bt_actions.py documents for its planners.py
dependency.
"""
import re
from collections import namedtuple

Obstacle = namedtuple("Obstacle", ["id", "x", "y", "radius"])


def resolve_obstacle_id(obstacles, obstacle_id):
    """Resolves `obstacle_id` against `obstacles`' own "tree_<tree_index>"
    ids -- EXACT match first, so a caller that already knows this
    simulation's own naming always gets exactly what it asked for.

    Falls back to matching the tree_index NUMBER embedded in
    `obstacle_id` (its first run of digits) against any known tree's
    own tree_index -- this is the "resolve the obstacle id with the
    tree id" piece: a problog_project tree's own obstacle_id (e.g.
    "obs5", from that problem's own obstacles_generated.pl, meaning
    nothing to this orchard) is treated as "tree index 5" instead, on
    the working assumption that whoever authored/adapted the tree meant
    "the obstacle numbered 5" and this orchard's own numbering is the
    only one that actually exists at runtime. A bare "5" resolves the
    same way. Returns None if no tree carries a matching tree_index
    either."""
    exact = next((o for o in obstacles if o.id == obstacle_id), None)
    if exact is not None:
        return exact

    match = re.search(r"\d+", str(obstacle_id))
    if match is None:
        return None
    return next((o for o in obstacles if o.id == f"tree_{match.group()}"), None)
