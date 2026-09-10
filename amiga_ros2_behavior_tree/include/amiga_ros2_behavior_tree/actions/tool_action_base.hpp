#pragma once

#include <sstream>
#include <string>

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>

namespace amiga_bt {

// Shared implementation for InstallTool/UninstallTool -- see
// module/contracts/schema.yaml's own entries for both (problog_project
// submodule): mirror-image durative actions (same start/halt shape as
// MoveTo, a fixed-Duration analogue of one startMoveto/haltMoveto
// pair), differing only in which ROS2 action type/server they dial
// (amiga_ros2_planners' own tool_action_node hosts both, "install_tool"
// and "uninstall_tool"). A template, not a plain base class, because
// BT::RosActionNode<T> is itself templated on the action message type
// -- InstallTool.action and UninstallTool.action are genuinely
// distinct types (matching schema.yaml's own two separate BT node
// IDs), so ActionT varies per concrete subclass below even though
// every line of logic here is identical.
//
// Same ReactiveSequence caveat as MoveTo (see move_to.hpp's own note
// via evaluate_condition_base.hpp's header): this is a genuinely
// multi-tick RosActionNode, safe only as the LAST async node in
// whichever ReactiveSequence it appears in.
template <typename ActionT>
class ToolActionBase : public BT::RosActionNode<ActionT> {
 public:
  using BT::RosActionNode<ActionT>::RosActionNode;
  using Goal = typename ActionT::Goal;
  using WrappedResult = typename BT::RosActionNode<ActionT>::WrappedResult;
  using Feedback = typename ActionT::Feedback;

  static BT::PortsList providedPorts() {
    return BT::RosActionNode<ActionT>::providedBasicPorts({
        BT::InputPort<std::string>("tool", "\"cart\" or \"plow\""),
        BT::InputPort<std::string>(
            "triggers", "", "semicolon-separated, battery-only trigger list"),
        BT::OutputPort<std::string>("reason"),
        BT::OutputPort<bool>("status"),
    });
  }

  bool setGoal(Goal &goal) override {
    std::string tool;
    if (!this->getInput("tool", tool)) {
      RCLCPP_ERROR(this->logger(), "%s: missing required input [tool]",
                   this->name().c_str());
      return false;
    }
    goal.tool = tool;

    std::string triggers_text;
    this->getInput("triggers", triggers_text);
    goal.triggers.clear();
    std::stringstream ss(triggers_text);
    std::string trigger;
    while (std::getline(ss, trigger, ';')) {
      if (!trigger.empty()) {
        goal.triggers.push_back(trigger);
      }
    }

    RCLCPP_INFO(this->logger(), "%s: requesting tool=%s",
                this->name().c_str(), tool.c_str());
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult &result) override {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED &&
        result.code != rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_ERROR(this->logger(), "%s: action aborted", this->name().c_str());
      this->setOutput("reason", std::string("aborted"));
      this->setOutput("status", false);
      return BT::NodeStatus::FAILURE;
    }

    this->setOutput("reason", result.result->reason);
    this->setOutput("status", result.result->status);

    if (result.result->status) {
      RCLCPP_INFO(this->logger(), "%s: completed", this->name().c_str());
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(this->logger(), "%s: halted, reason=%s", this->name().c_str(),
                result.result->reason.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override {
    RCLCPP_INFO(this->logger(), "%s: elapsed %.1fs", this->name().c_str(),
                feedback->elapsed_s);
    return BT::NodeStatus::RUNNING;
  }
};

}  // namespace amiga_bt
