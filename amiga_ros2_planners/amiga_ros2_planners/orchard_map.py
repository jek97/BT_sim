"""
orchard_map.py

Builds ONE whole-orchard 2D occupancy grid from the same circular
obstacle list orchard_obstacles.py already produces, and converts it
to/from nav_msgs/OccupancyGrid -- the standard ROS map representation,
the same wire format problog_project's own map.pgm/map.yaml is loaded
into via load_map_yaml.

This is the answer to "do we reason over a 3D map of the orchard": no,
neither before nor after this module -- nothing in this package ever
reads the Gazebo world's own mesh, point clouds, or any other 3D
representation. "The orchard" has always been the tree list's own
lat/lon metadata (published once over TCP into
orchard_management_node, see amiga_ros2_behavior_tree's own README),
which gps.py already converts into flat local (x,y) -- a 2D abstraction
from the start. What this module adds is WHEN that gets turned into a
grid: orchard_map_node.py builds one ONCE, at startup, from the whole
orchard's own extent, instead of plan_service_node rebuilding a
query-scoped one on every A* call the way an earlier revision of this
package did -- the closer match to problog_project's own
load-once-at-import-time map.yaml, and, published as a standard
nav_msgs/OccupancyGrid, something RViz/Foxglove/Nav2's own costmap
layers can all consume too.
"""
import numpy as np
from nav_msgs.msg import MapMetaData, OccupancyGrid

# Re-exported for backwards compatibility with anything importing
# build_grid_map from here -- the actual implementation moved to
# planning_core.py so it has no ROS import and scripts/export_orchard_map.py
# can call it standalone (see that function's own docstring for why).
from amiga_ros2_planners.planning_core import build_grid_map, OccupancyGridMap  # noqa: F401


def grid_to_occupancy_grid_msg(grid, frame_id, stamp):
    """OccupancyGridMap -> nav_msgs/OccupancyGrid, standard ROS map
    convention (row-major int8, row 0 = min y, unoccupied = 0)."""
    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.info = MapMetaData()
    msg.info.resolution = grid.resolution
    msg.info.width = grid.width
    msg.info.height = grid.height
    msg.info.origin.position.x = grid.origin[0]
    msg.info.origin.position.y = grid.origin[1]
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.data.flatten().tolist()
    return msg


def occupancy_grid_msg_to_grid(msg):
    """nav_msgs/OccupancyGrid -> OccupancyGridMap, the reverse of
    grid_to_occupancy_grid_msg -- what plan_service_node feeds astar()
    once orchard_map_node has published one."""
    data = np.array(msg.data, dtype=np.int8).reshape(
        (msg.info.height, msg.info.width))
    return OccupancyGridMap(
        data, msg.info.resolution,
        msg.info.origin.position.x, msg.info.origin.position.y)
