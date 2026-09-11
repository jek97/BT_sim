#include <behaviortree_cpp/bt_factory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <optional>
#include <string>

#include "amiga_ros2_behavior_tree/actions/assert_true.hpp"
#include "amiga_ros2_behavior_tree/actions/check_value.hpp"
#include "amiga_ros2_behavior_tree/actions/detect_object.hpp"
#include "amiga_ros2_behavior_tree/actions/approach_gps_waypoint.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to_gps_location.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to_aisle_head.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to_tree_id.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to_relative_location.hpp"
#include "amiga_ros2_behavior_tree/actions/orient_robot_heading.hpp"
#include "amiga_ros2_behavior_tree/actions/sample_leaf.hpp"
#include "amiga_ros2_behavior_tree/actions/follow_person.hpp"
#include "amiga_ros2_behavior_tree/actions/arm_move_to.hpp"
#include "amiga_ros2_behavior_tree/actions/plan_with.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to.hpp"
#include "amiga_ros2_behavior_tree/actions/evaluate_conditions.hpp"
#include "amiga_ros2_behavior_tree/actions/take_sample.hpp"
#include "amiga_ros2_behavior_tree/actions/install_tool.hpp"
#include "amiga_ros2_behavior_tree/actions/uninstall_tool.hpp"
#include "amiga_ros2_behavior_tree/actions/deploy_tool.hpp"
#include "amiga_ros2_behavior_tree/actions/retract_tool.hpp"
#include "amiga_ros2_behavior_tree/fault_reporter.hpp"
#include "amiga_ros2_behavior_tree/xml_validation.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using namespace BT;
using namespace amiga_bt;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("bt_runner");

  std::srand(static_cast<unsigned int>(std::time(nullptr)));

  nh->declare_parameter<std::string>("mission_topic", std::string("/mission/xml"));
  nh->declare_parameter<bool>("xml_validation", true);
  // The tree is the catalyst for the whole replanning and coordination
  // pipeline; this is the topic it fires on. See fault_reporter.hpp.
  nh->declare_parameter<std::string>("fault_topic", std::string("/bt/status_change"));
  nh->declare_parameter<bool>("fault_reporting", true);
  // One report per node per interval. A ReactiveSequence re-ticks a failing
  // condition at the tick rate, and every report downstream costs an LLM call.
  nh->declare_parameter<double>("fault_min_interval_sec", 5.0);
  std::string mission_topic;
  bool xml_validation_enabled;
  std::string fault_topic;
  bool fault_reporting_enabled;
  double fault_min_interval_sec;
  nh->get_parameter("mission_topic", mission_topic);
  nh->get_parameter("xml_validation", xml_validation_enabled);
  nh->get_parameter("fault_topic", fault_topic);
  nh->get_parameter("fault_reporting", fault_reporting_enabled);
  nh->get_parameter("fault_min_interval_sec", fault_min_interval_sec);

  BehaviorTreeFactory factory;
  RosNodeParams ros_params;
  ros_params.nh = nh;

  factory.registerNodeType<MoveToGPSLocation>("MoveToGPSLocation", ros_params);
  factory.registerNodeType<ApproachGPSWaypoint>("ApproachGPSWaypoint", ros_params);
  factory.registerNodeType<MoveToTreeID>("MoveToTreeID", ros_params);
  factory.registerNodeType<MoveToAisleHead>("MoveToAisleHead", ros_params);
  factory.registerNodeType<MoveToRelativeLocation>("MoveToRelativeLocation",
                                                   ros_params);
  factory.registerNodeType<OrientRobotHeading>("OrientRobotHeading",
                                                   ros_params);
  factory.registerNodeType<FollowPerson>("FollowPerson", ros_params);
  factory.registerNodeType<SampleLeaf>("SampleLeaf", ros_params);
  factory.registerNodeType<MoveArmToPosition>("MoveArmToPosition", ros_params);
  factory.registerNodeType<DetectObject>("DetectObject");
  // conditional nodes
  factory.registerNodeType<AssertTrue>("AssertTrue");
  factory.registerNodeType<CheckValue>("CheckValue");

  // problog_project-ported nodes -- ROS2 service/action backends in
  // amiga_ros2_planners (plan_service_node/move_to_node/
  // condition_service_node); see that package's own README for what
  // each one does and doesn't carry over from problog_project's own
  // semantics. HaltedWith has no leaf here on purpose -- see
  // evaluate_condition_base.hpp's own header.
  //
  // Unlike the pre-existing leaves above, problog-derived mission XML
  // never carries a literal service_name/action_name attribute (that's
  // not a concept in problog_project's own schema), so each of these
  // needs its own RosNodeParams with default_port_value set to the
  // fixed service/action name its Python backend actually advertises
  // (plan_service_node.py/move_to_node.py/condition_service_node.py) --
  // otherwise RosServiceNode/RosActionNode has no client to dial and
  // throws at tick time.
  // plan_service_node.py/condition_service_node.py deliberately delay
  // advertising plan_path/evaluate_condition until they have a real tf2
  // pose (and, for evaluate_condition, a first battery reading) -- see
  // their own PoseProvider.wait_ready() comments -- which can take
  // several seconds past this node's own startup while Gazebo/Nav2 are
  // still coming up. RosNodeParams' own default timeouts (the
  // service/action-existence check done once at tree-construction
  // time, wait_for_server_timeout, and the per-call response wait,
  // server_timeout) are far shorter than that, so createTreeFromText
  // would otherwise log "Service ... is not reachable" and hand back a
  // never-connected client -- the first real tick then hits onFailure
  // and fails BatteryOver/PlanWith outright on a startup race, not a
  // real condition/plan answer, which can steer an entire plain
  // Fallback down the wrong branch before its backend ever got a
  // chance to answer for real. Give these two (PlanWith, the
  // condition leaves -- NOT MoveTo, see its own comment below) the
  // same generous budget the Python side already waits up to (set
  // both timeout
  // fields since it's the existence check, wait_for_server_timeout,
  // that this specific error comes from, but a slow first response
  // right after startup is plausible too).
  auto backend_timeout = std::chrono::milliseconds(30000);

  RosNodeParams plan_params = ros_params;
  plan_params.default_port_value = "plan_path";
  plan_params.wait_for_server_timeout = backend_timeout;
  plan_params.server_timeout = backend_timeout;
  // move_to_node.py's own "move_to" ActionServer, unlike plan_path/
  // evaluate_condition, is advertised immediately at startup (its own
  // pose wait happens later, per-goal, inside _execute() -- see that
  // file's own comment) -- it never needed the 30s budget above, and
  // RosActionNode's server_timeout doubles as its no-feedback
  // watchdog, so 30s here meant every BatteryOver-triggered cancel
  // took a full 30 real seconds to resolve before the outer Fallback
  // could ever reach GoHome.
  //
  // The library's own DEFAULT (1s) turned out to be the opposite
  // mistake: rclpy's ActionServer (move_to_node.py) does not start
  // executing a NEW goal until the PREVIOUS one's own execute()
  // callback has fully returned, and that can legitimately take
  // close to a second after a cancel (move_to_node.py's own
  // cancel_goal_async + get_result_async waits, now capped at 2s
  // each). GoHome's very next MoveTo goal was hitting bt.cpp's 1s
  // watchdog before move_to_node.py had even started producing
  // feedback for it -- "BT fault: MoveTo (MoveTo) failed" with no
  // controller_server activity at all. 5s comfortably covers that
  // worst-case serialized handoff while still catching a genuinely
  // stuck walk far sooner than the mistaken 30s did.
  RosNodeParams move_to_params = ros_params;
  move_to_params.default_port_value = "move_to";
  move_to_params.server_timeout = std::chrono::milliseconds(5000);
  RosNodeParams condition_params = ros_params;
  condition_params.default_port_value = "evaluate_condition";
  condition_params.wait_for_server_timeout = backend_timeout;
  condition_params.server_timeout = backend_timeout;

  // sample_service_node.py's own "take_sample" service, like move_to_node's
  // "move_to" action, is advertised immediately at startup (no pose/battery
  // wait of its own) -- the library default is the right choice here, no
  // MoveTo-style backend_timeout/tuned-watchdog override needed.
  RosNodeParams sample_params = ros_params;
  sample_params.default_port_value = "take_sample";

  // tool_action_node.py's own "install_tool"/"uninstall_tool" actions are
  // ALSO advertised immediately at startup, same as "move_to" -- but they
  // share the exact same rclpy ActionServer-per-goal-name serialization
  // MoveTo's own server_timeout fix addressed (see move_to_params's own
  // comment above): a goal on either action right after a PRECEDING goal
  // on that SAME action was cancelled won't start producing feedback until
  // that previous execute() callback fully returns. Same 5s budget as
  // MoveTo, same reasoning, applied proactively rather than waiting to
  // reproduce the identical failure.
  auto tool_action_timeout = std::chrono::milliseconds(5000);
  RosNodeParams install_tool_params = ros_params;
  install_tool_params.default_port_value = "install_tool";
  install_tool_params.server_timeout = tool_action_timeout;
  RosNodeParams uninstall_tool_params = ros_params;
  uninstall_tool_params.default_port_value = "uninstall_tool";
  uninstall_tool_params.server_timeout = tool_action_timeout;
  RosNodeParams deploy_tool_params = ros_params;
  deploy_tool_params.default_port_value = "deploy_tool";
  deploy_tool_params.server_timeout = tool_action_timeout;
  RosNodeParams retract_tool_params = ros_params;
  retract_tool_params.default_port_value = "retract_tool";
  retract_tool_params.server_timeout = tool_action_timeout;

  factory.registerNodeType<PlanWith>("PlanWith", plan_params);
  factory.registerNodeType<MoveTo>("MoveTo", move_to_params);
  factory.registerNodeType<DistanceBelow>("DistanceBelow", condition_params);
  factory.registerNodeType<DistanceEqual>("DistanceEqual", condition_params);
  factory.registerNodeType<DistanceOver>("DistanceOver", condition_params);
  factory.registerNodeType<ObstacleInBound>("ObstacleInBound", condition_params);
  factory.registerNodeType<ObstacleOnPath>("ObstacleOnPath", condition_params);
  factory.registerNodeType<BatteryBelow>("BatteryBelow", condition_params);
  factory.registerNodeType<BatteryEqual>("BatteryEqual", condition_params);
  factory.registerNodeType<BatteryOver>("BatteryOver", condition_params);
  factory.registerNodeType<LineOfSightClear>("LineOfSightClear", condition_params);
  factory.registerNodeType<TakeSample>("TakeSample", sample_params);
  factory.registerNodeType<InstallTool>("InstallTool", install_tool_params);
  factory.registerNodeType<UninstallTool>("UninstallTool", uninstall_tool_params);
  factory.registerNodeType<DeployTool>("DeployTool", deploy_tool_params);
  factory.registerNodeType<RetractTool>("RetractTool", retract_tool_params);
  factory.registerNodeType<SampleValueBelow>("SampleValueBelow", condition_params);
  factory.registerNodeType<SampleValueEqual>("SampleValueEqual", condition_params);
  factory.registerNodeType<SampleValueOver>("SampleValueOver", condition_params);
  factory.registerNodeType<CollisionDetected>("CollisionDetected", condition_params);

  std::string schema_path;
  try {
    std::string share_dir = ament_index_cpp::get_package_share_directory(
        "amiga_ros2_behavior_tree");
    schema_path = share_dir + AMIGA_SCHEMA_DEFAULT_PATH;
  } catch (const std::exception &e) {
    RCLCPP_WARN(nh->get_logger(),
                "Could not resolve package share for default schema: %s",
                e.what());
    schema_path = AMIGA_SCHEMA_DEFAULT_PATH;
  }
  nh->declare_parameter<std::string>("mission_schema", schema_path);
  nh->get_parameter("mission_schema", schema_path);

  // Created once, outside the mission loop: a fault must outlive the tree that
  // produced it, because what reads it starts up in response to it.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub;
  if (fault_reporting_enabled) {
    fault_pub = makeFaultPublisher(nh, fault_topic);
  }

  std::mutex mtx;
  std::optional<std::string> pending_mission;
  auto sub = nh->create_subscription<std_msgs::msg::String>(
      mission_topic, 10,
      [&](const std_msgs::msg::String &msg) {
        std::lock_guard<std::mutex> lk(mtx);
        pending_mission = msg.data;
      });

  rclcpp::Rate spin_rate(20);
  while (rclcpp::ok()) {
    std::optional<std::string> mission_in_opt;
    {
      std::lock_guard<std::mutex> lk(mtx);
      mission_in_opt.swap(pending_mission);
    }
    if (!mission_in_opt.has_value()) {
      rclcpp::spin_some(nh);
      spin_rate.sleep();
      continue;
    }
    const std::string &mission_in = *mission_in_opt;

    Tree tree;
    try {
      std::string err;
      if (xml_validation_enabled && !xml_validation::validate(mission_in, schema_path, err)) {
        RCLCPP_ERROR(nh->get_logger(),
                     "Mission XML schema validation failed: %s", err.c_str());
        continue;
      }
      tree = factory.createTreeFromText(mission_in);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(nh->get_logger(), "Failed to create BT from mission: %s",
                   e.what());
      continue;
    }

    // Constructed per mission, because it subscribes to this tree's nodes and
    // the previous tree no longer exists. Destroyed at the end of the scope,
    // which unsubscribes before the tree is replaced.
    std::unique_ptr<FaultReporter> fault_reporter;
    if (fault_pub) {
      fault_reporter = std::make_unique<FaultReporter>(
          tree, nh, fault_pub, fault_min_interval_sec);
    }

    RCLCPP_INFO(nh->get_logger(), "Starting mission execution...");

    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
      auto status = tree.tickOnce();
      if (status == BT::NodeStatus::SUCCESS ||
          status == BT::NodeStatus::FAILURE) {
        RCLCPP_INFO(nh->get_logger(), "Mission finished with status: %s",
                    toStr(status, true).c_str());
        if (fault_reporter) {
          fault_reporter->reportTreeOutcome(status);
        }
        break;
      }
      rclcpp::spin_some(nh);
      rate.sleep();
    }
  }

  rclcpp::shutdown();
  return 0;
}
