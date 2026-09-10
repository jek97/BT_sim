"""
problog_sim_bringup.launch.py

The single command for checklist item 7 in this package's own README
("Running a problog_project BT in this simulation"): everything a
problog_project mission needs running at once, for a single robot --

    ros2 launch amiga_ros2_planners problog_sim_bringup.launch.py

Brings up, in order:
  1. amiga_ros2_gazebo's own sim_bringup.launch.py (robot_count:=1,
     launch_coordination/launch_agents:=false -- this is a fleet-of-one
     test bench, not a multi-robot auction scenario) -- Gazebo, the
     robot, Nav2 (launch_nav:=true, for MoveTo's own FollowPath;
     see this file's own note below on why that's the whole Nav2 stack
     and not just controller_server), and amiga_ros2_behavior_tree's
     bt.launch.py, pointed at amiga_ros2_planners' OWN
     amiga_btcpp_planners.xsd by default (override `mission_schema:=`
     to go back to the stock schema for a mission using none of the new
     node types).
  2. amiga_ros2_planners' own planners.launch.py -- orchard_map_node,
     plan_service_node, condition_service_node, move_to_node,
     battery_sim_node.

Both get the SAME namespace (empty, single-robot) and the SAME
datum/frame parameters, so nothing has to be kept in sync by hand across
two separate `ros2 launch` invocations.

Why launch_nav:=true (Nav2's FULL stack) rather than just
controller_server: amiga_navigation's own navigation.launch.py is the
only tested, working Nav2 bringup already in this repo (costmap
configs, params, TF wiring all included) -- assembling a bespoke
controller_server-only launch file would mean re-deriving those same
params untested. planner_server/bt_navigator/recoveries simply go
unused by this pipeline (MoveTo only ever dials controller_server's own
FollowPath action directly); harmless, just not minimal.

Once running, feed a mission the normal way:
    nc 0.0.0.0 12346 < your_adapted_tree.xml
(see amiga_ros2_behavior_tree/README.md's own "Quick demo" for the
expect_json/payload_length_included framing options, and
`ros2 run amiga_ros2_planners adapt_tree` to produce your_adapted_tree.xml
from a problog_project problem in the first place).

For a problog_project PROBLEM folder specifically (that problem's own
obstacles, not the live orchard, plus zero manual steps -- calibration,
adaptation, and feeding the mission all done automatically), use
run_problog_problem.launch.py instead of this file directly; it wraps
this file with `obstacle_source:=problog_problem` and everything else
computed from the problem folder itself.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, **launch_args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        ),
        launch_arguments={k: v for k, v in launch_args.items()}.items(),
    )


def generate_launch_description():
    datum_lat = LaunchConfiguration("datum_lat")
    datum_lon = LaunchConfiguration("datum_lon")
    tree_obstacle_radius = LaunchConfiguration("tree_obstacle_radius")
    problog_frame_origin_x = LaunchConfiguration("problog_frame_origin_x")
    problog_frame_origin_y = LaunchConfiguration("problog_frame_origin_y")
    problog_frame_yaw_deg = LaunchConfiguration("problog_frame_yaw_deg")
    mission_schema = LaunchConfiguration("mission_schema")
    obstacle_source = LaunchConfiguration("obstacle_source")
    problem_dir = LaunchConfiguration("problem_dir")
    battery_start_percent = LaunchConfiguration("battery_start_percent")
    battery_idle_drain_rate_pct_s = LaunchConfiguration("battery_idle_drain_rate_pct_s")
    battery_moving_drain_rate_pct_s = LaunchConfiguration("battery_moving_drain_rate_pct_s")
    expect_json = LaunchConfiguration("expect_json")
    payload_length_included = LaunchConfiguration("payload_length_included")
    sample_success_probability = LaunchConfiguration("sample_success_probability")
    install_duration_cart_s = LaunchConfiguration("install_duration_cart_s")
    install_duration_plow_s = LaunchConfiguration("install_duration_plow_s")
    uninstall_duration_cart_s = LaunchConfiguration("uninstall_duration_cart_s")
    uninstall_duration_plow_s = LaunchConfiguration("uninstall_duration_plow_s")
    install_success_probability = LaunchConfiguration("install_success_probability")
    uninstall_success_probability = LaunchConfiguration("uninstall_success_probability")
    install_drain_rate_pct_s = LaunchConfiguration("install_drain_rate_pct_s")
    uninstall_drain_rate_pct_s = LaunchConfiguration("uninstall_drain_rate_pct_s")
    tool_speed_free_mps = LaunchConfiguration("tool_speed_free_mps")
    tool_speed_cart_mps = LaunchConfiguration("tool_speed_cart_mps")
    tool_speed_plow_mps = LaunchConfiguration("tool_speed_plow_mps")
    tool_moving_drain_rate_cart_pct_s = LaunchConfiguration("tool_moving_drain_rate_cart_pct_s")
    tool_moving_drain_rate_plow_pct_s = LaunchConfiguration("tool_moving_drain_rate_plow_pct_s")

    return LaunchDescription([
        DeclareLaunchArgument(
            "datum_lat", default_value="37.3611",
            description="MUST match amiga_localization's own EKF datum "
            "latitude -- shared by the Gazebo/Nav2 bringup (indirectly, "
            "via amiga_localization's own config) and every "
            "amiga_ros2_planners node below."),
        DeclareLaunchArgument("datum_lon", default_value="-120.4322"),
        DeclareLaunchArgument("tree_obstacle_radius", default_value="0.5"),
        DeclareLaunchArgument(
            "problog_frame_origin_x", default_value="0.0",
            description="See amiga_ros2_planners/README.md's own "
            "'Running a problog_project BT' section for how to "
            "calibrate this against a specific problem's own map."),
        DeclareLaunchArgument("problog_frame_origin_y", default_value="0.0"),
        DeclareLaunchArgument("problog_frame_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument(
            "mission_schema",
            default_value=os.path.join(
                get_package_share_directory("amiga_ros2_planners"),
                "schemas", "amiga_btcpp_planners.xsd",
            ),
            description="Defaults to THIS package's own extended schema "
            "(PlanWith/MoveTo/conditions), unlike sim_bringup.launch.py's "
            "own default of the stock amiga_btcpp.xsd -- this launch "
            "file's whole point is running a problog_project mission."),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Passed straight through to sim_bringup.launch.py."),
        DeclareLaunchArgument(
            "obstacle_source", default_value="orchard",
            description="\"orchard\" (default) or \"problog_problem\" -- "
            "forwarded to planners.launch.py's own arg of the same name."),
        DeclareLaunchArgument("problem_dir", default_value=""),
        DeclareLaunchArgument("battery_start_percent", default_value="100.0"),
        DeclareLaunchArgument("battery_idle_drain_rate_pct_s", default_value="0.01"),
        DeclareLaunchArgument("battery_moving_drain_rate_pct_s", default_value="0.1"),
        DeclareLaunchArgument(
            "sample_success_probability", default_value="0.5",
            description="Forwarded to planners.launch.py's own arg of "
            "the same name -- TakeSample's coin-flip probability."),
        DeclareLaunchArgument("install_duration_cart_s", default_value="10.0"),
        DeclareLaunchArgument("install_duration_plow_s", default_value="10.0"),
        DeclareLaunchArgument("uninstall_duration_cart_s", default_value="10.0"),
        DeclareLaunchArgument("uninstall_duration_plow_s", default_value="10.0"),
        DeclareLaunchArgument("install_success_probability", default_value="0.9"),
        DeclareLaunchArgument("uninstall_success_probability", default_value="0.9"),
        DeclareLaunchArgument("install_drain_rate_pct_s", default_value="0.01"),
        DeclareLaunchArgument("uninstall_drain_rate_pct_s", default_value="0.01"),
        DeclareLaunchArgument("tool_speed_free_mps", default_value="0.5"),
        DeclareLaunchArgument("tool_speed_cart_mps", default_value="0.5"),
        DeclareLaunchArgument("tool_speed_plow_mps", default_value="0.5"),
        DeclareLaunchArgument("tool_moving_drain_rate_cart_pct_s", default_value="0.1"),
        DeclareLaunchArgument("tool_moving_drain_rate_plow_pct_s", default_value="0.1"),
        DeclareLaunchArgument(
            "expect_json", default_value="true",
            description="Forwarded to sim_bringup.launch.py -- set false "
            "for a mission with no second (orchard JSON) frame, e.g. a "
            "problog_project problem."),
        DeclareLaunchArgument("payload_length_included", default_value="true"),

        _include(
            "amiga_ros2_gazebo", "sim_bringup.launch.py",
            robot_count="1",
            launch_coordination="false",
            launch_agents="false",
            launch_nav="true",
            launch_bt="true",
            headless=LaunchConfiguration("headless"),
            mission_schema=mission_schema,
            expect_json=expect_json,
            payload_length_included=payload_length_included,
        ),
        _include(
            "amiga_ros2_planners", "planners.launch.py",
            datum_lat=datum_lat,
            datum_lon=datum_lon,
            tree_obstacle_radius=tree_obstacle_radius,
            problog_frame_origin_x=problog_frame_origin_x,
            problog_frame_origin_y=problog_frame_origin_y,
            problog_frame_yaw_deg=problog_frame_yaw_deg,
            obstacle_source=obstacle_source,
            problem_dir=problem_dir,
            battery_start_percent=battery_start_percent,
            battery_idle_drain_rate_pct_s=battery_idle_drain_rate_pct_s,
            battery_moving_drain_rate_pct_s=battery_moving_drain_rate_pct_s,
            sample_success_probability=sample_success_probability,
            install_duration_cart_s=install_duration_cart_s,
            install_duration_plow_s=install_duration_plow_s,
            uninstall_duration_cart_s=uninstall_duration_cart_s,
            uninstall_duration_plow_s=uninstall_duration_plow_s,
            install_success_probability=install_success_probability,
            uninstall_success_probability=uninstall_success_probability,
            install_drain_rate_pct_s=install_drain_rate_pct_s,
            uninstall_drain_rate_pct_s=uninstall_drain_rate_pct_s,
            tool_speed_free_mps=tool_speed_free_mps,
            tool_speed_cart_mps=tool_speed_cart_mps,
            tool_speed_plow_mps=tool_speed_plow_mps,
            tool_moving_drain_rate_cart_pct_s=tool_moving_drain_rate_cart_pct_s,
            tool_moving_drain_rate_plow_pct_s=tool_moving_drain_rate_plow_pct_s,
        ),
    ])
