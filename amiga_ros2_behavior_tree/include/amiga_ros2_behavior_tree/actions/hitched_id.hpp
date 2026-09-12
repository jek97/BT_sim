#pragma once

#include <chrono>
#include <string>

#include <amiga_interfaces/srv/hitched_id.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/rclcpp.hpp>

namespace amiga_bt {

using HitchedIdSrv = amiga_interfaces::srv::HitchedId;

// The ROS2-service form of problog_project's HitchedId -- see
// module/contracts/schema.yaml's own HitchedId entry (problog_project
// submodule). INSTANTANEOUS, like TakeSample/PlanWith: a pure,
// side-effect-free lookup against tool_action_node's own tracked hitch
// state, no Duration/Triggers.
//
// Deliberately a plain BT::SyncActionNode with a BLOCKING service
// call, NOT BT::RosServiceNode -- same rationale as TakeSample/
// EvaluateConditionBase (see evaluate_condition_base.hpp's own header
// for the full story). No input ports at all -- see HitchedId.srv's
// own header for why (it reports on the hitch's own CURRENT state
// directly, not a lookup keyed by an id/kind the caller supplies).
class HitchedId : public BT::SyncActionNode {
 public:
  using Request = HitchedIdSrv::Request;
  using Response = HitchedIdSrv::Response;

  HitchedId(const std::string &name, const BT::NodeConfig &config,
            const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  rclcpp::Logger logger() const { return node_->get_logger(); }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<HitchedIdSrv>::SharedPtr client_;
  std::string service_name_;
  std::chrono::milliseconds timeout_;
};

}  // namespace amiga_bt
