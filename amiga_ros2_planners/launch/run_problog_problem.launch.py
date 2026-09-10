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
       - problog_frame_origin_x/y (problog_problem.compute_frame_origin)
         -- so the robot, which spawns at this sim's own frame origin,
         lines up with wherever that problem's own theory assumed it
         started (config.yaml's initial_situation.start_x/start_y),
         with NO manual calibration step.
       - battery_start_percent/idle_drain_rate_pct_s/
         moving_drain_rate_pct_s (problog_problem.battery_params) --
         so battery_sim_node drains at the SAME rates that problem's
         own theory assumes, not this package's made-up defaults.
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

    config = problog_problem.load_config(problem_dir)
    origin_x, origin_y, yaw_deg = problog_problem.compute_frame_origin(config)
    battery = problog_problem.battery_params(config)
    sample = problog_problem.sample_params(config)
    tool = problog_problem.tool_params(config)

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
            "headless": headless,
            "obstacle_source": "problog_problem",
            "problem_dir": problem_dir,
            "problog_frame_origin_x": str(origin_x),
            "problog_frame_origin_y": str(origin_y),
            "problog_frame_yaw_deg": str(yaw_deg),
            "battery_start_percent": str(battery["start_percent"]),
            "battery_idle_drain_rate_pct_s": str(battery["idle_drain_rate_pct_s"]),
            "battery_moving_drain_rate_pct_s": str(battery["moving_drain_rate_pct_s"]),
            "sample_success_probability": str(sample["success_probability"]),
            "install_duration_cart_s": str(tool["install_duration_s"]["cart"]),
            "install_duration_plow_s": str(tool["install_duration_s"]["plow"]),
            "uninstall_duration_cart_s": str(tool["uninstall_duration_s"]["cart"]),
            "uninstall_duration_plow_s": str(tool["uninstall_duration_s"]["plow"]),
            "install_success_probability": str(tool["install_success_probability"]),
            "uninstall_success_probability": str(tool["uninstall_success_probability"]),
            "install_drain_rate_pct_s": str(tool["install_drain_rate_pct_s"]),
            "uninstall_drain_rate_pct_s": str(tool["uninstall_drain_rate_pct_s"]),
            "tool_speed_free_mps": str(tool["speed"]["free"]),
            "tool_speed_cart_mps": str(tool["speed"]["cart"]),
            "tool_speed_plow_mps": str(tool["speed"]["plow"]),
            "tool_moving_drain_rate_cart_pct_s": str(tool["moving_drain_rate_pct_s"]["cart"]),
            "tool_moving_drain_rate_plow_pct_s": str(tool["moving_drain_rate_pct_s"]["plow"]),
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
