#include "amiga_ros2_behavior_tree/actions/move_to.hpp"

#include <sstream>

namespace amiga_bt {

MoveTo::MoveTo(const std::string &name, const BT::NodeConfig &config,
               const BT::RosNodeParams &params)
    : BT::RosActionNode<MoveToAction>(name, config, params) {}

BT::PortsList MoveTo::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<std::vector<geometry_msgs::msg::Point>>("control_points"),
      // Semicolon-separated, e.g. "obstacle_in_bound(0.6);battery_below(20)"
      // -- same literal syntax schema.yaml's own MoveTo.triggers port
      // documents; split here into the action Goal's own string[]. See
      // move_to_node.py's own TRIGGER_FUNCTOR_TO_CONDITION for exactly
      // which functors this actually checks.
      BT::InputPort<std::string>("triggers", "", "semicolon-separated trigger list"),
      BT::OutputPort<std::string>("reason"),
      BT::OutputPort<bool>("status"),
  });
}

bool MoveTo::setGoal(Goal &goal) {
  std::vector<geometry_msgs::msg::Point> control_points;
  if (!getInput("control_points", control_points) || control_points.empty()) {
    RCLCPP_ERROR(logger(), "MoveTo: missing required input [control_points]");
    return false;
  }
  goal.control_points = control_points;

  std::string triggers_text;
  getInput("triggers", triggers_text);
  goal.triggers.clear();
  std::stringstream ss(triggers_text);
  std::string trigger;
  while (std::getline(ss, trigger, ';')) {
    if (!trigger.empty()) {
      goal.triggers.push_back(trigger);
    }
  }

  RCLCPP_INFO(logger(), "MoveTo: starting walk (%zu control points, %zu triggers)",
              control_points.size(), goal.triggers.size());
  return true;
}

BT::NodeStatus MoveTo::onResultReceived(const WrappedResult &result) {
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED &&
      result.code != rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_ERROR(logger(), "MoveTo: action aborted");
    setOutput("reason", std::string("aborted"));
    setOutput("status", false);
    return BT::NodeStatus::FAILURE;
  }

  setOutput("reason", result.result->reason);
  setOutput("status", result.result->status);

  if (result.result->status) {
    RCLCPP_INFO(logger(), "MoveTo: completed");
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(logger(), "MoveTo: halted, reason=%s", result.result->reason.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveTo::onFeedback(const std::shared_ptr<const Feedback> feedback) {
  // Throttled: FollowPath feedback arrives at controller_server's own
  // control frequency (5-20Hz typical), so an unthrottled RCLCPP_INFO
  // here floods the terminal with one line per feedback message for
  // the entire length of every walk.
  static int feedback_count = 0;
  if (feedback_count++ % 20 == 0) {
    RCLCPP_INFO(logger(), "MoveTo: distance to goal: %.2f m", feedback->distance_to_goal);
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
