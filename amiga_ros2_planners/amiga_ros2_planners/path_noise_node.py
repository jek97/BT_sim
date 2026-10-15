#!/usr/bin/env python3
"""
path_noise_node.py

Hosts AddPathNoise (amiga_interfaces/srv/AddPathNoise) -- sits between
the planner and Nav2's FollowPath. move_to_node.py calls this once per
walk (when its own enable_path_noise parameter is true), right after
building its nominal nav_msgs/Path (from planning_core.py's own
control_points, via bezier.py's sampling) and before sending the
FollowPath goal, and sends FollowPath the path THIS node returns
instead of the nominal one. See path_noise.py's own module docstring
for the exact noise model.

sigma_lateral/sigma_tangential are problog_project's own config.yaml
position.lateral.sigma/position.tangential.sigma, and nominal_speed_mps
is that same config.yaml's own motion.speed (see problog_problem.
position_noise_params) -- all three default to 0.0/0.0/1.0 (no noise,
response byte-for-byte equal to the nominal path, after resampling --
see path_noise.py's own module docstring for exactly why) when a
problem's own config.yaml has no `position`/`motion` section, so this
node is a safe no-op for every problem that doesn't opt into it.

step_dt_s is run_problog_problem.launch.py's own 1 / controller_server's
controller_frequency (Nav2's own nav2_params_sim.yaml) -- a per-problem
constant resolved once at launch time, not something to recompute on
every call, which is why it (like nominal_speed_mps) is a NODE
parameter and not part of the service request.

seed <0 (the default) means an unseeded, genuinely-random generator,
reseeded fresh on every service call (so repeated walks in one mission
don't share a draw sequence); seed >=0 makes EVERY call in this node's
own lifetime draw from a generator reseeded to that exact value, for
reproducibility -- the SAME realization every run, not just the same
statistics. Still an open-loop realization per walk either way, not a
single process-lifetime random walk.
"""
import rclpy
from rclpy.node import Node
import numpy as np

from amiga_interfaces.srv import AddPathNoise
from amiga_ros2_planners.path_noise import add_path_noise, resample_polyline_by_arc_length


def _yaw_from_quaternion(q):
    """Decodes a pure-Z-axis-rotation quaternion back into yaw (rad) --
    the exact inverse of bezier.py/move_to_node.py's own
    z=sin(yaw/2), w=cos(yaw/2) encoding."""
    return 2.0 * np.arctan2(q.z, q.w)


class PathNoiseNode(Node):
    def __init__(self):
        super().__init__("path_noise_node")

        self.declare_parameter("sigma_lateral", 0.0)
        self.declare_parameter("sigma_tangential", 0.0)
        self.declare_parameter("nominal_speed_mps", 1.0)
        self.declare_parameter("step_dt_s", 0.2)
        self.declare_parameter("seed", -1)

        self._srv = self.create_service(AddPathNoise, "add_path_noise", self._on_request)
        self.get_logger().info("path_noise_node ready on 'add_path_noise'")

    def _on_request(self, request, response):
        sigma_lateral = self.get_parameter("sigma_lateral").value
        sigma_tangential = self.get_parameter("sigma_tangential").value
        speed = self.get_parameter("nominal_speed_mps").value
        dt = self.get_parameter("step_dt_s").value
        seed = self.get_parameter("seed").value

        response.noisy_path.header = request.nominal_path.header

        if ((sigma_lateral <= 0.0 and sigma_tangential <= 0.0) or
                speed <= 0.0 or len(request.nominal_path.poses) < 2):
            # Exact pass-through -- see this module's own docstring on
            # why this is a hard guarantee, not just "noise happens to
            # average out".
            response.noisy_path.poses = list(request.nominal_path.poses)
            return response

        poses_xyyaw = [
            (pose.pose.position.x, pose.pose.position.y,
             _yaw_from_quaternion(pose.pose.orientation))
            for pose in request.nominal_path.poses
        ]
        ds = speed * dt
        resampled = resample_polyline_by_arc_length(poses_xyyaw, ds)

        rng = np.random.default_rng(seed if seed >= 0 else None)
        noisy = add_path_noise(resampled, sigma_lateral, sigma_tangential, dt, rng=rng)

        pose_type = type(request.nominal_path.poses[0])
        for x, y, yaw in noisy:
            pose = pose_type()
            pose.header = request.nominal_path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = float(np.sin(yaw / 2.0))
            pose.pose.orientation.w = float(np.cos(yaw / 2.0))
            response.noisy_path.poses.append(pose)

        self.get_logger().info(
            f"AddPathNoise: {len(poses_xyyaw)} nominal poses -> "
            f"{len(noisy)} noisy poses (sigma_lateral={sigma_lateral}, "
            f"sigma_tangential={sigma_tangential}, speed={speed}m/s, "
            f"dt={dt}s, ds={ds:.3f}m)")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathNoiseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
