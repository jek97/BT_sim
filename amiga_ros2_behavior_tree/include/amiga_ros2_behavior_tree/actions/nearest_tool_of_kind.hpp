#pragma once

#include <chrono>
#include <string>

#include <amiga_interfaces/srv/nearest_tool_of_kind.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/rclcpp.hpp>

namespace amiga_bt {

using NearestToolOfKindSrv = amiga_interfaces::srv::NearestToolOfKind;

// The ROS2-service form of problog_project's NearestToolOfKind -- see
// module/contracts/schema.yaml's own entry (problog_project
// submodule). INSTANTANEOUS, like TakeSample/HitchedId/PlanWith: a
// pure, side-effect-free lookup against tool_action_node's own tracked
// tool_instances table, no Duration/Triggers.
//
// Deliberately a plain BT::SyncActionNode with a BLOCKING service
// call, NOT BT::RosServiceNode -- same rationale as TakeSample/
// EvaluateConditionBase (see evaluate_condition_base.hpp's own header
// for the full story).
class NearestToolOfKind : public BT::SyncActionNode {
 public:
  using Request = NearestToolOfKindSrv::Request;
  using Response = NearestToolOfKindSrv::Response;

  NearestToolOfKind(const std::string &name, const BT::NodeConfig &config,
                     const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  rclcpp::Logger logger() const { return node_->get_logger(); }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<NearestToolOfKindSrv>::SharedPtr client_;
  std::string service_name_;
  std::chrono::milliseconds timeout_;
};

}  // namespace amiga_bt
