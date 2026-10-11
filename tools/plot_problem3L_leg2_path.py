#!/usr/bin/env python3
"""Standalone, no-ROS, no-sim visualization of problem3L's planned paths.

Imports planning_core.py/bezier.py directly (both are plain Python, no
rclpy import, by design -- see their own docstrings) to recompute exactly
what plan_service_node would hand back for each PlanWithWaypoints call in
problem3L/behavior_tree.xml, then samples and plots it the same way
move_to_node._build_path() would turn it into a nav_msgs/Path. This lets
you look at the raw planned geometry for leg2 (the one that oscillates)
without needing Gazebo/Nav2 running at all.

leg1 (works fine): PlanWithWaypoints algorithm="straight"
    waypoints="22.0;-15.0|22.0;-3.3|32.0;-3.3"
    -- start (sx,sy) is the robot's pose when this leg's PlanWithWaypoints
    runs. In problem3L this is the very first MoveTo of the mission, and
    the robot's sim spawn pose is used as (sx,sy) below -- see the
    SPAWN_XY note if your robot doesn't start there.

leg2 (oscillates): PlanWithWaypoints algorithm="straight"
    waypoints="32.0;-27.0|45.5;-27.0|45.5;-36.0"
    -- start (sx,sy) is wherever the robot is after leg1 + TakeSample.
    TakeSample doesn't translate the robot, so this is approximated as
    leg1's own final waypoint, (32.0, -3.3).

Usage: python3 plot_problem3L_leg2_path.py [--out PATH.png]
"""
import argparse
import math
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(
    0, os.path.join(_REPO_ROOT, "amiga_ros2_planners", "amiga_ros2_planners"))
import planning_core  # noqa: E402
import bezier  # noqa: E402

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# Robot's sim spawn pose -- adjust if problem3L's own launch/config spawns
# it somewhere else; leg1 is only plotted for comparison against leg2, so
# an approximate spawn point doesn't affect the leg2 diagnosis.
SPAWN_XY = (22.0, -15.0)

LEGS = [
    ("leg1_cp1_works", SPAWN_XY, [(22.0, -15.0), (22.0, -3.3), (32.0, -3.3)]),
    ("leg2_cp2_oscillates", (32.0, -3.3),
     [(32.0, -27.0), (45.5, -27.0), (45.5, -36.0)]),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "problem3L_paths.png"))
    args = parser.parse_args()

    fig, axes = plt.subplots(1, len(LEGS), figsize=(14, 7))
    if len(LEGS) == 1:
        axes = [axes]

    for ax, (label, (sx, sy), waypoints) in zip(axes, LEGS):
        cp = planning_core.plan_straight_waypoints_points(sx, sy, waypoints)
        if cp is None:
            ax.set_title(f"{label}: planner returned None")
            continue
        samples = bezier.sample_bezier_chain(cp, samples_per_segment=20)
        xs = [p[0] for p in samples]
        ys = [p[1] for p in samples]
        yaws = [p[2] for p in samples]

        ax.plot(xs, ys, "-o", markersize=2, linewidth=1, label="sampled path")
        ax.plot([sx] + [w[0] for w in waypoints],
                 [sy] + [w[1] for w in waypoints],
                 "r*", markersize=14, label="start + waypoints")

        # Yaw arrows every few samples, so abrupt heading flips are visible.
        step = max(1, len(samples) // 40)
        for i in range(0, len(samples), step):
            x, y, yaw = samples[i]
            ax.annotate(
                "", xy=(x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw)),
                xytext=(x, y),
                arrowprops=dict(arrowstyle="->", color="tab:blue", alpha=0.6))

        # Flag consecutive samples that are at (near) the same (x,y) but
        # have a large yaw jump -- the duplicated-boundary-point signature
        # sample_bezier_chain's own docstring describes for a kinked join.
        for i in range(1, len(samples)):
            x0, y0, yaw0 = samples[i - 1]
            x1, y1, yaw1 = samples[i]
            dist = math.hypot(x1 - x0, y1 - y0)
            dyaw = abs(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))
            if dist < 1e-6 and dyaw > math.radians(5):
                ax.plot(x0, y0, "kx", markersize=16, markeredgewidth=3)
                ax.annotate(
                    f"kink: dyaw={math.degrees(dyaw):.0f} deg",
                    (x0, y0), textcoords="offset points", xytext=(8, 8))

        ax.set_title(label)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.axis("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
