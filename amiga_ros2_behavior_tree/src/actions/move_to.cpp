#include "amiga_ros2_behavior_tree/actions/move_to.hpp"

namespace amiga_bt {

MoveTo::MoveTo(const std::string &name, const BT::NodeConfig &config,
               const BT::RosNodeParams &params)
    : BT::RosActionNode<MoveToAction>(name, config, params) {}

BT::PortsList MoveTo::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<std::vector<geometry_msgs::msg::Point>>("control_points"),
      BT::OutputPort<bool>("status"),
      // See MoveTo.action's own Result comment for the full set of
      // values "reason" can take, and when "collision_obstacle" is set.
      BT::OutputPort<std::string>("reason"),
      BT::OutputPort<std::string>("collision_obstacle"),
  });
}

bool MoveTo::setGoal(Goal &goal) {
  std::vector<geometry_msgs::msg::Point> control_points;
  if (!getInput("control_points", control_points) || control_points.empty()) {
    RCLCPP_ERROR(logger(), "MoveTo: missing required input [control_points]");
    return false;
  }
  goal.control_points = control_points;

  RCLCPP_INFO(logger(), "MoveTo: starting walk (%zu control points)",
              control_points.size());
  return true;
}

BT::NodeStatus MoveTo::onResultReceived(const WrappedResult &result) {
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED &&
      result.code != rclcpp_action::ResultCode::CANCELED) {
    // move_to_node.py's own abort paths all fill in `reason` before
    // calling goal_handle.abort() -- result.result is populated here
    // too, not just on SUCCEEDED/CANCELED.
    const std::string reason = result.result ? result.result->reason : "";
    setOutput("status", false);
    setOutput("reason", reason);
    RCLCPP_ERROR(logger(), "MoveTo: action aborted (%s)", reason.c_str());
    return BT::NodeStatus::FAILURE;
  }

  setOutput("status", result.result->status);
  setOutput("reason", result.result->reason);
  setOutput("collision_obstacle", result.result->collision_obstacle);

  if (result.result->status) {
    RCLCPP_INFO(logger(), "MoveTo: completed");
    return BT::NodeStatus::SUCCESS;
  }
  if (result.result->reason == "collision_detected") {
    RCLCPP_WARN(logger(), "MoveTo: halted (collision_detected, hit %s)",
                result.result->collision_obstacle.c_str());
  } else {
    RCLCPP_WARN(logger(), "MoveTo: halted (%s)", result.result->reason.c_str());
  }
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
