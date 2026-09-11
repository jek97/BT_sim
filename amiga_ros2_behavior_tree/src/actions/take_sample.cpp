#include "amiga_ros2_behavior_tree/actions/take_sample.hpp"

namespace amiga_bt {

TakeSample::TakeSample(const std::string &name, const BT::NodeConfig &config,
                        const BT::RosNodeParams &params)
    : BT::SyncActionNode(name, config), node_(params.nh) {
  service_name_ = params.default_port_value;
  // Deliberately no default value on the "service_name" port itself --
  // see evaluate_condition_base.cpp's own comment on the identical
  // pattern: a declared default would make getInput() always report
  // success and silently blank out the params.default_port_value
  // fallback above.
  getInput("service_name", service_name_);
  client_ = node_->create_client<TakeSampleSrv>(service_name_);
  timeout_ = params.server_timeout;
}

BT::PortsList TakeSample::providedPorts() {
  return {
      BT::InputPort<std::string>("service_name", "ROS2 service name"),
      BT::InputPort<std::string>(
          "id", "This occurrence's own name (e.g. \"soil1\") -- a later "
          "SampleValueBelow/Equal/Over references this same id."),
      BT::OutputPort<std::string>("reason"),
      BT::OutputPort<bool>("status"),
      BT::OutputPort<int>("value", "The drawn value, 0-10 -- only meaningful on success."),
  };
}

BT::NodeStatus TakeSample::tick() {
  Request::SharedPtr request = std::make_shared<Request>();

  std::string id;
  if (!getInput("id", id) || id.empty()) {
    RCLCPP_ERROR(logger(), "TakeSample: missing required input [id]");
    return BT::NodeStatus::FAILURE;
  }
  request->id = id;

  if (!client_->wait_for_service(timeout_)) {
    RCLCPP_ERROR(logger(), "TakeSample: service '%s' is not reachable",
                 service_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Humble's Client::async_send_request() returns a plain
  // std::shared_future<Response::SharedPtr> directly -- see
  // evaluate_condition_base.cpp's own comment on the same rclcpp
  // version detail.
  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger(), "TakeSample: call to '%s' failed or timed out",
                 service_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  Response::SharedPtr response = future.get();
  setOutput("reason", response->reason);
  setOutput("status", response->status);
  setOutput("value", static_cast<int>(response->value));
  RCLCPP_INFO(logger(), "TakeSample: %s (id=%s)", response->reason.c_str(), id.c_str());
  return response->status ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace amiga_bt
