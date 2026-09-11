#pragma once

#include <chrono>
#include <string>

#include <amiga_interfaces/srv/take_sample.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/rclcpp.hpp>

namespace amiga_bt {

using TakeSampleSrv = amiga_interfaces::srv::TakeSample;

// The ROS2-service form of problog_project's TakeSample -- see
// module/contracts/schema.yaml's own TakeSample entry (problog_project
// submodule). INSTANTANEOUS, like PlanWith: a single fixed-probability
// coin flip (amiga_ros2_planners' own sample_service_node.py), no
// Duration/Triggers, no continuous trajectory.
//
// Deliberately a plain BT::SyncActionNode with a BLOCKING service
// call, NOT BT::RosServiceNode -- same rationale as PlanWith/
// EvaluateConditionBase (see evaluate_condition_base.hpp's own header
// for the full story): an async node here would flap RUNNING/SUCCESS
// across ticks and could tear down a LATER sibling's own in-flight
// progress inside a ReactiveSequence. TakeSample takes one required
// input port beyond the shared service_name every problog-ported leaf
// gets: `id`, this occurrence's own name (schema.yaml's tool-instance-
// id-refactor-era addition) -- a later SampleValueBelow/Equal/Over
// condition node references this same id to read back the value this
// occurrence drew.
class TakeSample : public BT::SyncActionNode {
 public:
  using Request = TakeSampleSrv::Request;
  using Response = TakeSampleSrv::Response;

  TakeSample(const std::string &name, const BT::NodeConfig &config,
             const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  rclcpp::Logger logger() const { return node_->get_logger(); }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<TakeSampleSrv>::SharedPtr client_;
  std::string service_name_;
  std::chrono::milliseconds timeout_;
};

}  // namespace amiga_bt
