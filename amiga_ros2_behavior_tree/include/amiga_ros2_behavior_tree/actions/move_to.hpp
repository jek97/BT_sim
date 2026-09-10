#pragma once

#include <amiga_interfaces/action/move_to.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <vector>

namespace amiga_bt {

using MoveToAction = amiga_interfaces::action::MoveTo;

// The ROS2-action form of problog_project's MoveTo -- dials
// amiga_ros2_planners' move_to_node, which samples control_points
// into a nav_msgs/Path and drives Nav2's controller_server FollowPath
// action (see MoveTo.action's own header for the full contract, and
// move_to_node.py's own module docstring for what it does and does not
// carry over from problog_project's own MoveTo semantics).
//
// control_points arrives via the shared blackboard key a preceding
// PlanWith wrote (never a literal in any tree seen so far), so reading
// it back as std::vector<geometry_msgs::msg::Point> needs no string
// conversion -- see plan_with.hpp's own note on the same convention.
class MoveTo : public BT::RosActionNode<MoveToAction> {
 public:
  MoveTo(const std::string &name, const BT::NodeConfig &config,
         const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal &goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult &result) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};

}  // namespace amiga_bt
