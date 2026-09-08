"""
orchard_obstacles.py

Turns this robot's own orchard into the obstacle list planning_core.py
and the condition checks need -- the live-simulation replacement for
problog_project's own obstacles_generated.pl (a static, hand-authored
polygon list loaded once at import time). There is no such file here:
the orchard is whatever tcp_demux_node's second TCP frame delivered at
mission-start, cached and republished by orchard_management_node on
`orchard_topic` (default "orchard/tree_info_json", relative -- see
amiga_ros2_behavior_tree/README.md's own note on that topic). This
module subscribes to the SAME raw topic directly (not the
GetTreeInfo service, which answers a specific index lookup rather than
"give me every tree") and treats every tree as a circular obstacle: its
canopy, not the trunk alone, since that is what a planned path actually
needs to clear.
"""
import json

from std_msgs.msg import String

from amiga_ros2_planners.gps import latlon_to_local
from amiga_ros2_planners.obstacle_types import Obstacle, resolve_obstacle_id


class OrchardObstacleStore:
    """Subscribes once, caches the latest orchard payload, and converts
    on demand rather than on receipt -- so a datum parameter change
    (e.g. via `ros2 param set`) takes effect on the very next
    get_obstacles() call with no need to re-publish the orchard."""

    def __init__(self, node, topic, datum_lat, datum_lon, tree_radius):
        self._node = node
        self._datum_lat = datum_lat
        self._datum_lon = datum_lon
        self._tree_radius = tree_radius
        self._trees = []  # raw dicts from the last "trees" array received
        self._received = False  # distinguishes "no message yet" from "0 trees"
        self._sub = node.create_subscription(String, topic, self._on_json, 10)

    def has_received(self):
        """True once at least one orchard payload has arrived -- lets a
        one-shot caller (orchard_map_node) tell "nothing published yet"
        apart from "published, genuinely zero trees"."""
        return self._received

    def _on_json(self, msg):
        self._received = True
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self._node.get_logger().warn(f"orchard JSON parse failed: {exc}")
            return
        # Two payload shapes exist in this repo: the "new" wrapped object
        # {"trees": [...], "aisle_entrances": [...], ...}
        # orchard_management.cpp's own handle_request expects, and a
        # bare [...] array of tree records -- the shape every checked-in
        # amiga_ros2_behavior_tree/examples/*.bin fixture actually
        # carries (verified directly against mv_10_60_sample.bin's own
        # second frame). orchard_management.cpp itself only accepts the
        # wrapped form (data.is_object() check), so those fixtures would
        # come back empty through THAT node's own /orchard/get_tree_info
        # service -- accepting both here means this module still gets a
        # real obstacle list from them regardless of that mismatch.
        if isinstance(data, dict):
            trees = data.get("trees", [])
        elif isinstance(data, list):
            trees = data
        else:
            trees = []
        self._trees = [t for t in trees if "lat" in t and "lon" in t]

    def get_obstacles(self):
        """[Obstacle, ...] -- one per tree currently cached, id
        "tree_<tree_index>" (falling back to the tree's position in the
        list if the orchard payload carries no tree_index field)."""
        obstacles = []
        for i, tree in enumerate(self._trees):
            x, y = latlon_to_local(
                tree["lat"], tree["lon"], self._datum_lat, self._datum_lon)
            tree_id = tree.get("tree_index", i)
            obstacles.append(Obstacle(f"tree_{tree_id}", x, y, self._tree_radius))
        return obstacles

    def get_obstacle(self, obstacle_id):
        """See obstacle_types.resolve_obstacle_id for the actual
        resolution logic (exact id match, falling back to a bare
        tree-index match) -- kept there so it's testable without ROS."""
        return resolve_obstacle_id(self.get_obstacles(), obstacle_id)
