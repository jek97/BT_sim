#pragma once

#include <chrono>
#include <string>

#include <amiga_interfaces/srv/evaluate_condition.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/rclcpp.hpp>

namespace amiga_bt {

using EvaluateCondition = amiga_interfaces::srv::EvaluateCondition;

// Shared request/response plumbing for every problog_project Condition
// leaf EXCEPT HaltedWith (see EvaluateCondition.srv's own header for
// why HaltedWith has no service-backed leaf at all). Each concrete
// condition below (evaluate_conditions.hpp) only supplies its own
// providedPorts()/setRequest().
//
// Deliberately a plain BT::SyncActionNode with a BLOCKING service
// call, NOT BT::RosServiceNode (async, multi-tick): these leaves sit
// inside problog-derived ReactiveSequence trees ALONGSIDE PlanWith and
// MoveTo (both genuinely long-running), and BT.cpp's ReactiveSequence
// re-ticks every earlier sibling from scratch on every single tree
// tick, halting every LATER child the moment an EARLIER one reports
// RUNNING. An async RosServiceNode condition flips between RUNNING
// (the tick it sends the request) and SUCCESS/FAILURE (the tick it
// reads the response) even when the answer never changes -- and every
// time that happens, it tears down PlanWith's own in-flight request
// before PlanWith's response could ever be read, so PlanWith can never
// complete (observed as bt_runner logging "PlanWith: requesting..."
// forever, never "completed"/"failed"). A Condition is semantically
// supposed to resolve within one tick anyway (BT.cpp's own convention:
// conditions never return RUNNING), so this blocks synchronously
// instead -- the same pattern move_to_node.py's own _check_triggers
// already uses for this exact service (call_async +
// spin_until_future_complete).
class EvaluateConditionBase : public BT::SyncActionNode {
 public:
  using Request = EvaluateCondition::Request;
  using Response = EvaluateCondition::Response;

  EvaluateConditionBase(const std::string &name, const BT::NodeConfig &config,
                         const BT::RosNodeParams &params);

  // Mirrors BT::RosServiceNode::providedBasicPorts: every concrete
  // condition's own providedPorts() calls this to add the "service_name"
  // port (defaulted to whatever RosNodeParams::default_port_value the
  // registration in bt.cpp supplied) on top of its own ports.
  static BT::PortsList providedBasicPorts(BT::PortsList addition);

  BT::NodeStatus tick() final;

  virtual bool setRequest(Request::SharedPtr &request) = 0;
  virtual BT::NodeStatus onResponseReceived(const Response::SharedPtr &response);
  virtual BT::NodeStatus onFailure(const std::string &error_detail);

  rclcpp::Logger logger() const { return node_->get_logger(); }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<EvaluateCondition>::SharedPtr client_;
  std::string service_name_;
  std::chrono::milliseconds timeout_;
};

}  // namespace amiga_bt
