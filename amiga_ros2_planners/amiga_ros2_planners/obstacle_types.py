"""
obstacle_types.py

Just the Obstacle shape (id, x, y, radius), split out from
orchard_obstacles.py so planning_core.py/geometry.py and their tests
never need to import rclpy/std_msgs to construct one -- the same
"plain-Python core with no ROS import anywhere" testability goal
problog_project's own bt_actions.py documents for its planners.py
dependency.
"""
from collections import namedtuple

Obstacle = namedtuple("Obstacle", ["id", "x", "y", "radius"])
