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
import math

import numpy as np
from nav_msgs.msg import MapMetaData, OccupancyGrid

from amiga_ros2_planners.planning_core import OccupancyGridMap, PLANNING_INFLATE_M


def build_grid_map(obstacles, resolution, margin, inflate=PLANNING_INFLATE_M):
    """Rasterize ONE OccupancyGridMap covering every obstacle's own
    extent plus `margin` -- the whole-orchard analogue of
    planning_core.py's own (query-scoped) build_occupancy_grid, sized
    once from every known tree rather than per (start, goal) query.
    Returns a 1x1 empty grid at the origin if `obstacles` is empty (no
    orchard published yet), so a caller always gets something usable."""
    if not obstacles:
        data = np.zeros((1, 1), dtype=np.int8)
        return OccupancyGridMap(data, resolution, -margin, -margin)

    min_x = min(o.x - o.radius for o in obstacles) - margin
    max_x = max(o.x + o.radius for o in obstacles) + margin
    min_y = min(o.y - o.radius for o in obstacles) - margin
    max_y = max(o.y + o.radius for o in obstacles) + margin

    width = max(1, int(math.ceil((max_x - min_x) / resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution)))
    data = np.zeros((height, width), dtype=np.int8)
    grid = OccupancyGridMap(data, resolution, min_x, min_y)

    yy, xx = np.mgrid[0:height, 0:width]
    world_x = min_x + (xx + 0.5) * resolution
    world_y = min_y + (yy + 0.5) * resolution
    for obstacle in obstacles:
        occupied_radius = obstacle.radius + inflate
        mask = (world_x - obstacle.x) ** 2 + (world_y - obstacle.y) ** 2 \
            <= occupied_radius ** 2
        data[mask] = 100

    return grid


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
