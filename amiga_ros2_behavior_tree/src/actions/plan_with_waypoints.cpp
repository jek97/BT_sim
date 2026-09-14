#include "amiga_ros2_behavior_tree/actions/plan_with_waypoints.hpp"

#include "amiga_ros2_behavior_tree/point_port.hpp"

namespace amiga_bt {

PlanWithWaypoints::PlanWithWaypoints(const std::string &name, const BT::NodeConfig &config,
                                     const BT::RosNodeParams &params)
    : BT::SyncActionNode(name, config), node_(params.nh) {
  service_name_ = params.default_port_value;
  // Deliberately no default value on the "service_name" port itself --
  // see evaluate_condition_base.cpp's own comment on the identical
  // pattern.
  getInput("service_name", service_name_);
  client_ = node_->create_client<PlanPathWaypoints>(service_name_);
  timeout_ = params.server_timeout;
}

BT::PortsList PlanWithWaypoints::providedPorts() {
  return {
      BT::InputPort<std::string>("service_name", "ROS2 service name"),
      BT::InputPort<std::string>("algorithm", "\"astar\" or \"straight\""),
      BT::InputPort<std::string>(
          "waypoints", "\"X;Y|X;Y|...\" -- ordered points to visit in order"),
      BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("control_points"),
      BT::OutputPort<std::string>("reason"),
      BT::OutputPort<bool>("status"),
  };
}

bool PlanWithWaypoints::setRequest(Request::SharedPtr &request) {
  std::string algorithm;
  if (!getInput("algorithm", algorithm)) {
    RCLCPP_ERROR(logger(), "PlanWithWaypoints: missing required input [algorithm]");
    return false;
  }
  request->algorithm = algorithm;

  std::string waypoints_text;
  std::vector<std::pair<double, double>> waypoints;
  if (!getInput("waypoints", waypoints_text) || !parseWaypoints(waypoints_text, waypoints)) {
    RCLCPP_ERROR(logger(),
                 "PlanWithWaypoints: missing/malformed waypoints port '%s', "
                 "expected \"X;Y|X;Y|...\"",
                 waypoints_text.c_str());
    return false;
  }
  request->waypoints.clear();
  for (const auto &[x, y] : waypoints) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = 0.0;
    request->waypoints.push_back(point);
  }

  RCLCPP_INFO_THROTTLE(logger(), *node_->get_clock(), 2000,
                       "PlanWithWaypoints: requesting algorithm=%s, %zu waypoints",
                       algorithm.c_str(), waypoints.size());
  return true;
}

BT::NodeStatus PlanWithWaypoints::tick() {
  Request::SharedPtr request = std::make_shared<Request>();
  if (!setRequest(request)) {
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_service(timeout_)) {
    return onFailure("service '" + service_name_ + "' is not reachable");
  }

  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return onFailure("call to '" + service_name_ + "' failed or timed out");
  }
  return onResponseReceived(future.get());
}

BT::NodeStatus PlanWithWaypoints::onResponseReceived(const Response::SharedPtr &response) {
  setOutput("control_points", response->control_points);
  setOutput("reason", response->reason);
  setOutput("status", response->status);

  if (response->status) {
    RCLCPP_INFO_THROTTLE(logger(), *node_->get_clock(), 2000,
                         "PlanWithWaypoints: completed (%zu control points)",
                         response->control_points.size());
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(logger(), "PlanWithWaypoints: failed, reason=%s", response->reason.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PlanWithWaypoints::onFailure(const std::string &error_detail) {
  RCLCPP_ERROR(logger(), "PlanWithWaypoints: service call failed: %s", error_detail.c_str());
  setOutput("reason", std::string("service_call_failed"));
  setOutput("status", false);
  return BT::NodeStatus::FAILURE;
}

}  // namespace amiga_bt
