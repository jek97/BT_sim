#include "amiga_ros2_behavior_tree/actions/hitched_id.hpp"

namespace amiga_bt {

HitchedId::HitchedId(const std::string &name, const BT::NodeConfig &config,
                      const BT::RosNodeParams &params)
    : BT::SyncActionNode(name, config), node_(params.nh) {
  service_name_ = params.default_port_value;
  // Deliberately no default value on the "service_name" port itself --
  // see evaluate_condition_base.cpp's own comment on the identical
  // pattern.
  getInput("service_name", service_name_);
  client_ = node_->create_client<HitchedIdSrv>(service_name_);
  timeout_ = params.server_timeout;
}

BT::PortsList HitchedId::providedPorts() {
  return {
      BT::InputPort<std::string>("service_name", "ROS2 service name"),
      BT::OutputPort<std::string>("reason"),
      BT::OutputPort<bool>("status"),
      BT::OutputPort<std::string>("id", "The currently-hitched instance's own id, if any."),
  };
}

BT::NodeStatus HitchedId::tick() {
  Request::SharedPtr request = std::make_shared<Request>();

  if (!client_->wait_for_service(timeout_)) {
    RCLCPP_ERROR(logger(), "HitchedId: service '%s' is not reachable",
                 service_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger(), "HitchedId: call to '%s' failed or timed out",
                 service_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  Response::SharedPtr response = future.get();
  setOutput("reason", response->reason);
  setOutput("status", response->status);
  setOutput("id", response->id);
  RCLCPP_INFO(logger(), "HitchedId: %s", response->reason.c_str());
  return response->status ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace amiga_bt
