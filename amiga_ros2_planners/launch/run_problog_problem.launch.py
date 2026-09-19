"""
run_problog_problem.launch.py

Give this launch file a problog_project problem folder; it does
everything else on its own:

    ros2 launch amiga_ros2_planners run_problog_problem.launch.py \
        problem_dir:=/path/to/BT_project/problems/problem0

Concretely, at launch-generation time (before any node starts --
launch_setup below runs as plain Python, no ROS context needed for any
of this):

  1. Reads that problem's own config.yaml (problog_problem.load_config)
     and computes:
       - robot1_x/y (config.yaml's own initial_situation.start_x/start_y,
         taken as-is) -- every goal/obstacle coordinate in a
         problog_project problem is already authored directly against
         this sim's own orchard_map_a/b/c frame (see e.g. problem3L's
         own config.yaml comment: "every PlanWith goal in this tree is a
         literal coordinate tuned to orchard_map_a's real layout"), so
         spawning the robot at that SAME real point -- not some fixed,
         problem-independent spot -- is what actually lines the robot up
         with where the problem's own theory assumed it started, with NO
         manual calibration step. problog_frame_origin_x/y/yaw_deg stay
         identity (0,0,0) accordingly: ProblogFrameTransform
         (frame_transform.py) exists for a problem authored in a
         genuinely SEPARATE local frame, which this current crop of
         problems is not.
       - battery_start_percent/idle_drain_rate_pct_s/
         moving_drain_rate_pct_s (problog_problem.battery_params) --
         so battery_sim_node drains at the SAME rates that problem's
         own theory assumes, not this package's made-up defaults.
       - plough_cell_size (problog_problem.ploughing_params) -- so
         PloughedAt/PloughedBetween (condition_service_node) discretize
         positions at the SAME resolution move_to_node marks them
         ploughed at.
  2. Adapts <problem_dir>/behavior_tree.xml into a temp file
     (adapt_tree.adapt_and_write) -- <Mission>/schema_location added,
     everything else byte-for-byte unchanged (see that module's own
     docstring for exactly what "adapt" does and does not do -- it is
     NOT a semantic translation, just the root-element shape).
  3. Includes problog_sim_bringup.launch.py with
     obstacle_source:=problog_problem, problem_dir:=<problem_dir>, and
     every computed param above -- Gazebo, Nav2, bt.launch.py (with
     expect_json/payload_length_included:=false, matching step 4's
     single-frame send), and every amiga_ros2_planners node, all reading
     THIS problem's own obstacles_generated.pl rather than the live
     orchard.
  4. Starts send_mission (an installed console_script,
     amiga_ros2_planners' own send_mission.py) sending the adapted tree
     to tcp_demux_node's own port, retrying for `mission_send_timeout`
     seconds so it doesn't matter that the rest of the launch tree is
     still coming up (Gazebo+Nav2 startup is slow and variable) -- no
     manual `nc` step.

Still NOT automatic -- see amiga_ros2_planners/README.md's own "Running
a problog_project BT" checklist for the full, current list:
  - HaltedWith, if the tree uses it -- still rejected by
    amiga_btcpp_planners.xsd outright, on purpose.
  - The BT.cpp leaves (PlanWith/MoveTo/the nine conditions) are
    registered in bt_runner but UNCOMPILED in this session -- see
    amiga_ros2_behavior_tree/README.md's own caveat. This launch file
    gets every OTHER piece running; it can't make an unbuilt leaf work.
  - obstacle_id values: resolved automatically for BOTH obstacle
    sources (OrchardObstacleStore.get_obstacle's numeric fallback for
    "orchard" mode; a direct name match against this problem's own
    obstacles_generated.pl ids for "problog_problem" mode) -- only a
    genuinely unusual naming scheme would still need attention.
"""
import json
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from amiga_ros2_planners import adapt_tree, problog_problem


def launch_setup(context, *args, **kwargs):
    problem_dir = LaunchConfiguration("problem_dir").perform(context)
    mission_port = LaunchConfiguration("mission_port").perform(context)
    mission_send_timeout = LaunchConfiguration("mission_send_timeout").perform(context)
    headless = LaunchConfiguration("headless")
    move_to_backend = LaunchConfiguration("move_to_backend")

    world_override = LaunchConfiguration("world").perform(context)
    worlds_dir = os.path.join(get_package_share_directory("amiga_ros2_gazebo"), "worlds")
    if world_override:
        # Explicit `world:=` always wins -- either a bare stem (matched
        # against worlds_dir, same as the auto-detected case below) or a
        # full/relative path to a .sdf file (passed straight through).
        world = (world_override if os.sep in world_override or
                 world_override.endswith(".sdf")
                 else os.path.join(worlds_dir, f"{world_override}.sdf"))
    else:
        # Auto-detect from this problem's own map.yaml (problog_problem.
        # world_for_problem) so problem3L opens orchard_map_a, not
        # problog_sim_bringup.launch.py's own orchard_nbv.sdf default (the
        # full 144-tree environment) -- see that function's own docstring.
        # Falls back to the orchard_nbv default if map.yaml is missing/
        # unrecognized, so a problem folder with no map.yaml behaves
        # exactly as before this world-selection logic existed.
        stem = problog_problem.world_for_problem(problem_dir)
        candidate = os.path.join(worlds_dir, f"{stem}.sdf") if stem else None
        world = candidate if candidate and os.path.isfile(candidate) else os.path.join(
            worlds_dir, "orchard_nbv.sdf")

    config = problog_problem.load_config(problem_dir)
    # Real spawn point, not a frame shift -- see this file's own module
    # docstring and problog_sim_bringup.launch.py's own robot1_x
    # description for why: this problem's own goal/obstacle coordinates
    # are already in this sim's own frame, so the robot needs to
    # physically start where config.yaml says the theory assumed it did.
    situation = config.get("initial_situation", {})
    robot1_x = situation.get("start_x", 0.0)
    robot1_y = situation.get("start_y", 0.0)
    battery = problog_problem.battery_params(config)
    sample = problog_problem.sample_params(config)
    tool = problog_problem.tool_params(config)
    ploughing = problog_problem.ploughing_params(config)

    problem_name = os.path.basename(os.path.normpath(problem_dir)) or "problog_problem"
    tree_path = os.path.join(problem_dir, "behavior_tree.xml")
    adapted_path = os.path.join(tempfile.gettempdir(), f"{problem_name}_adapted.xml")
    schema_path = os.path.join(
        get_package_share_directory("amiga_ros2_planners"),
        "schemas", "amiga_btcpp_planners.xsd")
    adapt_tree.adapt_and_write(
        tree_path, adapted_path,
        mission_text=f"problog_project problem '{problem_name}' (auto-adapted "
                     f"by run_problog_problem.launch.py)",
        schema_location=schema_path)

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("amiga_ros2_planners"),
                "launch", "problog_sim_bringup.launch.py")),
        launch_arguments={
            "world": world,
            "headless": headless,
            "obstacle_source": "problog_problem",
            "problem_dir": problem_dir,
            "robot1_x": str(robot1_x),
            "robot1_y": str(robot1_y),
            "problog_frame_origin_x": "0.0",
            "problog_frame_origin_y": "0.0",
            "problog_frame_yaw_deg": "0.0",
            "battery_start_percent": str(battery["start_percent"]),
            "battery_idle_drain_rate_pct_s": str(battery["idle_drain_rate_pct_s"]),
            "battery_moving_drain_rate_pct_s": str(battery["moving_drain_rate_pct_s"]),
            "sample_success_probability": str(sample["success_probability"]),
            "sample_value_mean": str(sample["value_mean"]),
            "sample_value_sigma": str(sample["value_sigma"]),
            "sample_value_discretized": json.dumps(sample["value_discretized"]),
            "install_duration_cart_s": str(tool["install_duration_s"]["cart"]),
            "install_duration_plow_s": str(tool["install_duration_s"]["plow"]),
            "uninstall_duration_cart_s": str(tool["uninstall_duration_s"]["cart"]),
            "uninstall_duration_plow_s": str(tool["uninstall_duration_s"]["plow"]),
            "deploy_duration_cart_s": str(tool["deploy_duration_s"]["cart"]),
            "deploy_duration_plow_s": str(tool["deploy_duration_s"]["plow"]),
            "retract_duration_cart_s": str(tool["retract_duration_s"]["cart"]),
            "retract_duration_plow_s": str(tool["retract_duration_s"]["plow"]),
            "install_success_probability": str(tool["install_success_probability"]),
            "uninstall_success_probability": str(tool["uninstall_success_probability"]),
            "deploy_success_probability": str(tool["deploy_success_probability"]),
            "retract_success_probability": str(tool["retract_success_probability"]),
            "install_drain_rate_pct_s": str(tool["install_drain_rate_pct_s"]),
            "uninstall_drain_rate_pct_s": str(tool["uninstall_drain_rate_pct_s"]),
            "deploy_drain_rate_pct_s": str(tool["deploy_drain_rate_pct_s"]),
            "retract_drain_rate_pct_s": str(tool["retract_drain_rate_pct_s"]),
            "tool_speed_free_mps": str(tool["speed"]["free"]),
            "tool_speed_cart_mps": str(tool["speed"]["cart"]),
            "tool_speed_plow_mps": str(tool["speed"]["plow"]),
            "tool_speed_cart_deployed_mps": str(tool["deployed_speed"]["cart"]),
            "tool_speed_plow_deployed_mps": str(tool["deployed_speed"]["plow"]),
            "tool_moving_drain_rate_cart_pct_s": str(tool["moving_drain_rate_pct_s"]["cart"]),
            "tool_moving_drain_rate_plow_pct_s": str(tool["moving_drain_rate_pct_s"]["plow"]),
            "tool_moving_drain_rate_cart_deployed_pct_s": str(
                tool["deployed_moving_drain_rate_pct_s"]["cart"]),
            "tool_moving_drain_rate_plow_deployed_pct_s": str(
                tool["deployed_moving_drain_rate_pct_s"]["plow"]),
            "tool_instances": json.dumps(tool["tool_instances"]),
            "install_range": str(tool["install_range"]),
            "plough_cell_size": str(ploughing["cell_size"]),
            "move_to_backend": move_to_backend,
            "expect_json": "false",
            "payload_length_included": "false",
        }.items(),
    )

    send_mission = ExecuteProcess(
        cmd=[
            "ros2", "run", "amiga_ros2_planners", "send_mission",
            "--port", mission_port,
            "--file", adapted_path,
            "--timeout", mission_send_timeout,
        ],
        output="screen",
    )

    return [bringup, send_mission]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "problem_dir",
            description="Path to a problog_project problems/<name>/ "
            "directory (containing behavior_tree.xml, config.yaml, "
            "obstacles_generated.pl) -- REQUIRED."),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument(
            "world", default_value="",
            description="Gazebo world to bring up -- a bare stem among "
            "amiga_ros2_gazebo/worlds/ (e.g. \"orchard_map_b\") or a full "
            ".sdf path. Left empty (default), auto-detected from this "
            "problem's own map.yaml (problog_problem.world_for_problem), "
            "e.g. problem3L -> orchard_map_a.sdf -- NOT "
            "problog_sim_bringup.launch.py's own orchard_nbv.sdf default "
            "(the full 144-tree environment)."),
        DeclareLaunchArgument(
            "move_to_backend", default_value="nav2",
            description="Forwarded to planners.launch.py's own arg of "
            "the same name -- \"nav2\" (default) or \"openloop\" (see "
            "move_to_openloop_node.py's own module docstring)."),
        DeclareLaunchArgument(
            "mission_port", default_value="12346",
            description="Must match bt.launch.py's own tcp_demux_node "
            "port -- sim_bringup.launch.py's own default for robot 1."),
        DeclareLaunchArgument(
            "mission_send_timeout", default_value="120.0",
            description="How long send_mission keeps retrying the "
            "connection to tcp_demux_node, seconds -- generous, since "
            "Gazebo+Nav2 can take a while to finish coming up."),
        OpaqueFunction(function=launch_setup),
    ])
