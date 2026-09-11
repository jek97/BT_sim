"""
planners.launch.py

Brings up this package's three nodes for one robot:
    ros2 launch amiga_ros2_planners planners.launch.py
    ros2 launch amiga_ros2_planners planners.launch.py namespace:=amiga2

`namespace` places all three under one robot, same convention as
amiga_ros2_behavior_tree/launch/bt.launch.py -- every topic/param
default here is relative, so it resolves against the node's own
namespace with no caller cooperation needed.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace", default_value="",
            description="Robot namespace for all three nodes."),
        DeclareLaunchArgument(
            "orchard_topic", default_value="orchard/tree_info_json",
            description="Same topic orchard_management_node caches from."),
        DeclareLaunchArgument(
            "datum_lat", default_value="37.3611",
            description="MUST match amiga_localization's own EKF datum "
            "latitude, or converted tree obstacles won't line up with "
            "the robot's own tf2 pose."),
        DeclareLaunchArgument("datum_lon", default_value="-120.4322"),
        DeclareLaunchArgument(
            "tree_obstacle_radius", default_value="0.5",
            description="Metres -- canopy radius planners/conditions "
            "treat every tree as."),
        DeclareLaunchArgument("reference_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "odom_frame", default_value="odom",
            description="move_to_node's own pre-flight check: the frame "
            "local_costmap needs a base_frame transform into before "
            "controller_server's FollowPath has any real robot state to "
            "run against (see move_to_node.py's own constructor comment)."),
        DeclareLaunchArgument("battery_topic", default_value="battery_state"),
        DeclareLaunchArgument("odom_topic", default_value="odometry/filtered/local"),
        DeclareLaunchArgument("map_topic", default_value="orchard/occupancy_grid"),
        DeclareLaunchArgument(
            "map_resolution", default_value="0.25",
            description="Metres per cell of the whole-orchard occupancy "
            "grid orchard_map_node builds once at startup."),
        DeclareLaunchArgument("map_margin", default_value="5.0"),
        DeclareLaunchArgument(
            "follow_path_action", default_value="follow_path",
            description="Nav2 controller_server's own FollowPath action "
            "name -- move_to_node dials this directly, no planner_server/"
            "bt_navigator/recoveries involved."),
        DeclareLaunchArgument(
            "controller_id", default_value="",
            description="Nav2 controller plugin id to request (empty = "
            "controller_server's own default)."),
        DeclareLaunchArgument("samples_per_segment", default_value="10"),
        DeclareLaunchArgument(
            "problog_frame_origin_x", default_value="0.0",
            description="Where a problog_project problem's own map "
            "origin sits in this sim's frame (identity: goal points are "
            "already in this sim's own frame). Shared by plan_service "
            "and condition_service so a goal PlanWith targets and the "
            "same goal a DistanceBelow checks against always agree."),
        DeclareLaunchArgument("problog_frame_origin_y", default_value="0.0"),
        DeclareLaunchArgument("problog_frame_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument(
            "obstacle_source", default_value="orchard",
            description="\"orchard\" (default, this sim's own live trees) "
            "or \"problog_problem\" (a problog_project problem folder's "
            "own obstacles_generated.pl, loaded once from problem_dir) "
            "-- see plan_service_node.py's own module docstring."),
        DeclareLaunchArgument(
            "problem_dir", default_value="",
            description="Required when obstacle_source:=problog_problem."),
        DeclareLaunchArgument("battery_start_percent", default_value="100.0"),
        DeclareLaunchArgument("battery_idle_drain_rate_pct_s", default_value="0.01"),
        DeclareLaunchArgument("battery_moving_drain_rate_pct_s", default_value="0.1"),
        DeclareLaunchArgument(
            "sample_success_probability", default_value="0.5",
            description="TakeSample's own coin-flip probability -- "
            "problog_problem.sample_params's own config.yaml mapping "
            "(sample.success_probability)."),
        DeclareLaunchArgument(
            "sample_value_mean", default_value="5.0",
            description="TakeSample's own drawn-value distribution "
            "(config.yaml sample.value.mean/.sigma), on success only."),
        DeclareLaunchArgument("sample_value_sigma", default_value="2.0"),
        DeclareLaunchArgument(
            "tool_state_topic", default_value="tool_state",
            description="Latched (TRANSIENT_LOCAL) topic tool_action_node "
            "publishes its own tracked equipped-tool KIND on -- "
            "move_to_node/battery_sim_node both subscribe."),
        DeclareLaunchArgument(
            "tool_activity_topic", default_value="tool_activity",
            description="Latched topic tool_action_node publishes "
            "\"idle\"/\"installing\"/\"uninstalling\"/\"deploying\"/"
            "\"retracting\" on -- battery_sim_node subscribes, to apply "
            "that action's own drain rate for its own span."),
        DeclareLaunchArgument(
            "tool_deployed_topic", default_value="tool_deployed",
            description="Latched topic tool_action_node publishes "
            "\"true\"/\"false\" on between a successful DeployTool and its "
            "matching RetractTool -- move_to_node/battery_sim_node both "
            "subscribe, to select tool.equipped.<kind>.deployed_speed/"
            ".deployed_moving_drain_rate instead of the regular value."),
        DeclareLaunchArgument(
            "tool_instances", default_value="{}",
            description="JSON object {id: {kind, x, y}, ...} -- "
            "problog_problem.tool_params's own config.yaml mapping "
            "(tool.instances), passed as one JSON blob since ros2 launch "
            "has no clean way to pass a variable-length list of dicts."),
        DeclareLaunchArgument(
            "install_range", default_value="1.0",
            description="Metres -- how close the robot must be to a tool "
            "instance's own declared position before InstallTool can "
            "start. problog_problem.tool_params's own config.yaml mapping "
            "(tool.install.range, defaulting to robot.radius+"
            "robot.safety_buffer)."),
        DeclareLaunchArgument(
            "controller_server_set_parameters_service",
            default_value="controller_server/set_parameters",
            description="move_to_node's own live desired_linear_vel "
            "override target (see its own _apply_tool_speed)."),
        DeclareLaunchArgument(
            "install_duration_cart_s", default_value="10.0",
            description="problog_problem.tool_params's own "
            "install_duration_s/uninstall_duration_s dicts, spelled out "
            "as individual launch args (ros2 launch has no clean way to "
            "pass a per-tool dict as one argument) -- config.yaml's own "
            "tool.install.duration_seconds.<tool>/tool.uninstall."
            "duration_seconds.<tool>."),
        DeclareLaunchArgument("install_duration_plow_s", default_value="10.0"),
        DeclareLaunchArgument("uninstall_duration_cart_s", default_value="10.0"),
        DeclareLaunchArgument("uninstall_duration_plow_s", default_value="10.0"),
        DeclareLaunchArgument(
            "deploy_duration_cart_s", default_value="10.0",
            description="DeployTool/RetractTool's own duration/success/"
            "drain-rate knobs -- exact mirror of install/uninstall's own "
            "above, from config.yaml's tool.deploy.*/tool.retract.*. "
            "Currently only plow can actually be deployed (see "
            "tool_action_node.py's own _DEPLOYABLE_KINDS), but these are "
            "declared per-kind like install/uninstall for the same "
            "future-proofing schema.yaml's own DeployTool entry notes."),
        DeclareLaunchArgument("deploy_duration_plow_s", default_value="10.0"),
        DeclareLaunchArgument("retract_duration_cart_s", default_value="10.0"),
        DeclareLaunchArgument("retract_duration_plow_s", default_value="10.0"),
        DeclareLaunchArgument("install_success_probability", default_value="0.9"),
        DeclareLaunchArgument("uninstall_success_probability", default_value="0.9"),
        DeclareLaunchArgument("deploy_success_probability", default_value="0.9"),
        DeclareLaunchArgument("retract_success_probability", default_value="0.9"),
        DeclareLaunchArgument(
            "install_drain_rate_pct_s", default_value="0.01",
            description="config.yaml's own tool.install.drain_rate, "
            "defaulting to battery_idle_drain_rate_pct_s's own value -- "
            "see problog_problem.tool_params's own docstring."),
        DeclareLaunchArgument("uninstall_drain_rate_pct_s", default_value="0.01"),
        DeclareLaunchArgument("deploy_drain_rate_pct_s", default_value="0.01"),
        DeclareLaunchArgument("retract_drain_rate_pct_s", default_value="0.01"),
        DeclareLaunchArgument(
            "tool_speed_free_mps", default_value="0.5",
            description="config.yaml's own motion.speed / "
            "tool.equipped.<tool>.speed, applied live to "
            "controller_server's own desired_linear_vel per walk -- see "
            "move_to_node.py's own _apply_tool_speed."),
        DeclareLaunchArgument("tool_speed_cart_mps", default_value="0.5"),
        DeclareLaunchArgument("tool_speed_plow_mps", default_value="0.5"),
        DeclareLaunchArgument(
            "tool_speed_cart_deployed_mps", default_value="0.5",
            description="config.yaml's own tool.equipped.<tool>."
            "deployed_speed -- move_to_node's own THIRD speed state, "
            "applied instead of tool_speed_<tool>_mps while deployed."),
        DeclareLaunchArgument("tool_speed_plow_deployed_mps", default_value="0.5"),
        DeclareLaunchArgument(
            "tool_moving_drain_rate_cart_pct_s", default_value="0.1",
            description="config.yaml's own battery.moving_drain_rate / "
            "tool.equipped.<tool>.moving_drain_rate -- battery_sim_node's "
            "own per-tool MoveTo drain rate."),
        DeclareLaunchArgument("tool_moving_drain_rate_plow_pct_s", default_value="0.1"),
        DeclareLaunchArgument(
            "tool_moving_drain_rate_cart_deployed_pct_s", default_value="0.1",
            description="config.yaml's own tool.equipped.<tool>."
            "deployed_moving_drain_rate -- battery_sim_node's own THIRD "
            "moving-drain-rate state, applied instead of "
            "tool_moving_drain_rate_<tool>_pct_s while deployed."),
        DeclareLaunchArgument(
            "tool_moving_drain_rate_plow_deployed_pct_s", default_value="0.1"),

        # Pointless (and noisy -- it would wait forever for an orchard
        # JSON that never arrives) in problog_problem mode, where A*
        # rasterizes its own grid from that problem's own obstacles
        # instead -- see plan_service_node.py's own module docstring.
        Node(
            package="amiga_ros2_planners",
            executable="orchard_map",
            name="orchard_map_node",
            namespace=namespace,
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("obstacle_source"), "' == 'orchard'"])),
            parameters=[{
                "orchard_topic": LaunchConfiguration("orchard_topic"),
                "datum_lat": LaunchConfiguration("datum_lat"),
                "datum_lon": LaunchConfiguration("datum_lon"),
                "tree_obstacle_radius": LaunchConfiguration("tree_obstacle_radius"),
                "map_topic": LaunchConfiguration("map_topic"),
                "map_frame": LaunchConfiguration("reference_frame"),
                "resolution": LaunchConfiguration("map_resolution"),
                "margin": LaunchConfiguration("map_margin"),
            }],
        ),
        Node(
            package="amiga_ros2_planners",
            executable="plan_service",
            name="plan_service_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "orchard_topic": LaunchConfiguration("orchard_topic"),
                "datum_lat": LaunchConfiguration("datum_lat"),
                "datum_lon": LaunchConfiguration("datum_lon"),
                "tree_obstacle_radius": LaunchConfiguration("tree_obstacle_radius"),
                "reference_frame": LaunchConfiguration("reference_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "map_topic": LaunchConfiguration("map_topic"),
                "problog_frame_origin_x": LaunchConfiguration("problog_frame_origin_x"),
                "problog_frame_origin_y": LaunchConfiguration("problog_frame_origin_y"),
                "problog_frame_yaw_deg": LaunchConfiguration("problog_frame_yaw_deg"),
                "obstacle_source": LaunchConfiguration("obstacle_source"),
                "problem_dir": LaunchConfiguration("problem_dir"),
            }],
        ),
        Node(
            package="amiga_ros2_planners",
            executable="condition_service",
            name="condition_service_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "orchard_topic": LaunchConfiguration("orchard_topic"),
                "datum_lat": LaunchConfiguration("datum_lat"),
                "datum_lon": LaunchConfiguration("datum_lon"),
                "tree_obstacle_radius": LaunchConfiguration("tree_obstacle_radius"),
                "reference_frame": LaunchConfiguration("reference_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "battery_topic": LaunchConfiguration("battery_topic"),
                "problog_frame_origin_x": LaunchConfiguration("problog_frame_origin_x"),
                "problog_frame_origin_y": LaunchConfiguration("problog_frame_origin_y"),
                "problog_frame_yaw_deg": LaunchConfiguration("problog_frame_yaw_deg"),
                "obstacle_source": LaunchConfiguration("obstacle_source"),
                "problem_dir": LaunchConfiguration("problem_dir"),
            }],
        ),
        Node(
            package="amiga_ros2_planners",
            executable="move_to",
            name="move_to_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "reference_frame": LaunchConfiguration("reference_frame"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "samples_per_segment": LaunchConfiguration("samples_per_segment"),
                "follow_path_action": LaunchConfiguration("follow_path_action"),
                "controller_id": LaunchConfiguration("controller_id"),
                "tool_state_topic": LaunchConfiguration("tool_state_topic"),
                "tool_deployed_topic": LaunchConfiguration("tool_deployed_topic"),
                "controller_server_set_parameters_service": LaunchConfiguration(
                    "controller_server_set_parameters_service"),
                "tool_speed_free_mps": LaunchConfiguration("tool_speed_free_mps"),
                "tool_speed_cart_mps": LaunchConfiguration("tool_speed_cart_mps"),
                "tool_speed_plow_mps": LaunchConfiguration("tool_speed_plow_mps"),
                "tool_speed_cart_deployed_mps": LaunchConfiguration(
                    "tool_speed_cart_deployed_mps"),
                "tool_speed_plow_deployed_mps": LaunchConfiguration(
                    "tool_speed_plow_deployed_mps"),
            }],
        ),
        Node(
            package="amiga_ros2_planners",
            executable="battery_sim",
            name="battery_sim_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "odom_topic": LaunchConfiguration("odom_topic"),
                "battery_topic": LaunchConfiguration("battery_topic"),
                "start_percent": LaunchConfiguration("battery_start_percent"),
                "idle_drain_rate_pct_s": LaunchConfiguration(
                    "battery_idle_drain_rate_pct_s"),
                "moving_drain_rate_pct_s": LaunchConfiguration(
                    "battery_moving_drain_rate_pct_s"),
                "tool_state_topic": LaunchConfiguration("tool_state_topic"),
                "tool_activity_topic": LaunchConfiguration("tool_activity_topic"),
                "tool_deployed_topic": LaunchConfiguration("tool_deployed_topic"),
                "tool_moving_drain_rate_cart_pct_s": LaunchConfiguration(
                    "tool_moving_drain_rate_cart_pct_s"),
                "tool_moving_drain_rate_plow_pct_s": LaunchConfiguration(
                    "tool_moving_drain_rate_plow_pct_s"),
                "tool_moving_drain_rate_cart_deployed_pct_s": LaunchConfiguration(
                    "tool_moving_drain_rate_cart_deployed_pct_s"),
                "tool_moving_drain_rate_plow_deployed_pct_s": LaunchConfiguration(
                    "tool_moving_drain_rate_plow_deployed_pct_s"),
                "install_drain_rate_pct_s": LaunchConfiguration(
                    "install_drain_rate_pct_s"),
                "uninstall_drain_rate_pct_s": LaunchConfiguration(
                    "uninstall_drain_rate_pct_s"),
                "deploy_drain_rate_pct_s": LaunchConfiguration(
                    "deploy_drain_rate_pct_s"),
                "retract_drain_rate_pct_s": LaunchConfiguration(
                    "retract_drain_rate_pct_s"),
            }],
        ),
        Node(
            package="amiga_ros2_planners",
            executable="sample_service",
            name="sample_service_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "success_probability": LaunchConfiguration("sample_success_probability"),
                "value_mean": LaunchConfiguration("sample_value_mean"),
                "value_sigma": LaunchConfiguration("sample_value_sigma"),
            }],
        ),
        Node(
            package="amiga_ros2_planners",
            executable="tool_action",
            name="tool_action_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "reference_frame": LaunchConfiguration("reference_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "tool_state_topic": LaunchConfiguration("tool_state_topic"),
                "tool_activity_topic": LaunchConfiguration("tool_activity_topic"),
                "tool_deployed_topic": LaunchConfiguration("tool_deployed_topic"),
                "tool_instances": LaunchConfiguration("tool_instances"),
                "install_range": LaunchConfiguration("install_range"),
                "install_duration_cart_s": LaunchConfiguration("install_duration_cart_s"),
                "install_duration_plow_s": LaunchConfiguration("install_duration_plow_s"),
                "uninstall_duration_cart_s": LaunchConfiguration("uninstall_duration_cart_s"),
                "uninstall_duration_plow_s": LaunchConfiguration("uninstall_duration_plow_s"),
                "deploy_duration_cart_s": LaunchConfiguration("deploy_duration_cart_s"),
                "deploy_duration_plow_s": LaunchConfiguration("deploy_duration_plow_s"),
                "retract_duration_cart_s": LaunchConfiguration("retract_duration_cart_s"),
                "retract_duration_plow_s": LaunchConfiguration("retract_duration_plow_s"),
                "install_success_probability": LaunchConfiguration(
                    "install_success_probability"),
                "uninstall_success_probability": LaunchConfiguration(
                    "uninstall_success_probability"),
                "deploy_success_probability": LaunchConfiguration(
                    "deploy_success_probability"),
                "retract_success_probability": LaunchConfiguration(
                    "retract_success_probability"),
            }],
        ),
    ])
