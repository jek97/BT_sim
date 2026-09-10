#include "amiga_ros2_behavior_tree/actions/evaluate_condition_base.hpp"

namespace amiga_bt {

EvaluateConditionBase::EvaluateConditionBase(const std::string &name,
                                              const BT::NodeConfig &config,
                                              const BT::RosNodeParams &params)
    : BT::SyncActionNode(name, config), node_(params.nh) {
  service_name_ = params.default_port_value;
  getInput("service_name", service_name_);
  client_ = node_->create_client<EvaluateCondition>(service_name_);
  timeout_ = params.server_timeout;
}

BT::PortsList EvaluateConditionBase::providedBasicPorts(BT::PortsList addition) {
  // Deliberately no default value on this port: getInput() must return
  // false (leaving service_name_ untouched) unless the mission XML
  // literally sets service_name="..." itself -- problog-derived trees
  // never do, so the constructor's own params.default_port_value
  // fallback (set BEFORE this getInput() call) is what actually wins.
  // A declared default of "" here would make getInput() report success
  // with an empty string every time and silently blank that fallback out.
  BT::PortsList basic = {
      BT::InputPort<std::string>("service_name", "ROS2 service name"),
  };
  basic.insert(addition.begin(), addition.end());
  return basic;
}

BT::NodeStatus EvaluateConditionBase::tick() {
  Request::SharedPtr request = std::make_shared<Request>();
  if (!setRequest(request)) {
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_service(timeout_)) {
    return onFailure("service '" + service_name_ + "' is not reachable");
  }

  // Humble's Client::async_send_request() returns a plain
  // std::shared_future<Response::SharedPtr> directly -- the
  // FutureAndRequestId wrapper (.future member) is a later
  // (Iron+) rclcpp addition, not present on this project's Humble target.
  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return onFailure("call to '" + service_name_ + "' failed or timed out");
  }
  return onResponseReceived(future.get());
}

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

BT::NodeStatus EvaluateConditionBase::onFailure(const std::string &error_detail) {
  RCLCPP_ERROR(logger(), "condition service call failed: %s", error_detail.c_str());
  return BT::NodeStatus::FAILURE;
}

}  // namespace amiga_bt
