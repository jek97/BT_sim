#pragma once

#include <amiga_interfaces/srv/evaluate_condition.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>

namespace amiga_bt {

using EvaluateCondition = amiga_interfaces::srv::EvaluateCondition;

// Shared response handling for every problog_project Condition leaf
// EXCEPT HaltedWith (see EvaluateCondition.srv's own header for why
// HaltedWith has no service-backed leaf at all: it needs a tree's own
// blackboard history, which belongs in a leaf that reads THIS tree's
// blackboard directly, not a service call). Each concrete condition
// below (evaluate_conditions.hpp) only supplies its own providedPorts()
// and setRequest() -- which fields of EvaluateCondition::Request it
// fills in -- and inherits this tick-to-SUCCESS/FAILURE mapping
// unchanged, mirroring PlanWith's own "one shared mechanism, several
// thin variants" shape at the C++ registration level.
class EvaluateConditionBase : public BT::RosServiceNode<EvaluateCondition> {
 public:
  using BT::RosServiceNode<EvaluateCondition>::RosServiceNode;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr &response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

}  // namespace amiga_bt
