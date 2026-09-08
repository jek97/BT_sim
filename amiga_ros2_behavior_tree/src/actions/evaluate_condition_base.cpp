#include "amiga_ros2_behavior_tree/actions/evaluate_condition_base.hpp"

namespace amiga_bt {

BT::NodeStatus EvaluateConditionBase::onResponseReceived(
    const Response::SharedPtr &response) {
  if (!response->result) {
    RCLCPP_INFO(logger(), "condition false%s%s",
                response->reason.empty() ? "" : ", reason=",
                response->reason.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus EvaluateConditionBase::onFailure(BT::ServiceNodeErrorCode error) {
  RCLCPP_ERROR(logger(), "condition service call failed, error code %d", int(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace amiga_bt
