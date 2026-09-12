#include "amiga_ros2_behavior_tree/actions/nearest_tool_of_kind.hpp"

#include "amiga_ros2_behavior_tree/point_port.hpp"

namespace amiga_bt {

NearestToolOfKind::NearestToolOfKind(const std::string &name, const BT::NodeConfig &config,
                                      const BT::RosNodeParams &params)
    : BT::SyncActionNode(name, config), node_(params.nh) {
  service_name_ = params.default_port_value;
  getInput("service_name", service_name_);
  client_ = node_->create_client<NearestToolOfKindSrv>(service_name_);
  timeout_ = params.server_timeout;
}

BT::PortsList NearestToolOfKind::providedPorts() {
  return {
      BT::InputPort<std::string>("service_name", "ROS2 service name"),
      BT::InputPort<std::string>("kind", "\"cart\", \"plow\", or any future kind"),
      BT::OutputPort<std::string>("reason"),
      BT::OutputPort<bool>("status"),
      BT::OutputPort<std::string>("id", "The closest matching instance's own id."),
      BT::OutputPort<std::string>(
          "position", "That instance's own current position, \"X;Y\"."),
  };
}

BT::NodeStatus NearestToolOfKind::tick() {
  Request::SharedPtr request = std::make_shared<Request>();

  std::string kind;
  if (!getInput("kind", kind) || kind.empty()) {
    RCLCPP_ERROR(logger(), "NearestToolOfKind: missing required input [kind]");
    return BT::NodeStatus::FAILURE;
  }
  request->kind = kind;

  if (!client_->wait_for_service(timeout_)) {
    RCLCPP_ERROR(logger(), "NearestToolOfKind: service '%s' is not reachable",
                 service_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger(), "NearestToolOfKind: call to '%s' failed or timed out",
                 service_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  Response::SharedPtr response = future.get();
  setOutput("reason", response->reason);
  setOutput("status", response->status);
  setOutput("id", response->id);
  setOutput("position", formatPoint(response->x, response->y));
  RCLCPP_INFO(logger(), "NearestToolOfKind: %s (kind=%s)", response->reason.c_str(),
              kind.c_str());
  return response->status ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace amiga_bt
