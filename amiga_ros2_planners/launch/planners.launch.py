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
from launch.substitutions import LaunchConfiguration
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

        Node(
            package="amiga_ros2_planners",
            executable="orchard_map",
            name="orchard_map_node",
            namespace=namespace,
            output="screen",
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
                "samples_per_segment": LaunchConfiguration("samples_per_segment"),
                "follow_path_action": LaunchConfiguration("follow_path_action"),
                "controller_id": LaunchConfiguration("controller_id"),
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
            }],
        ),
    ])
