#include "amiga_ros2_behavior_tree/actions/plan_with.hpp"

#include "amiga_ros2_behavior_tree/point_port.hpp"

namespace amiga_bt {

PlanWith::PlanWith(const std::string &name, const BT::NodeConfig &config,
                   const BT::RosNodeParams &params)
    : BT::SyncActionNode(name, config), node_(params.nh) {
  service_name_ = params.default_port_value;
  // Deliberately no default value on the "service_name" port itself: see
  // evaluate_condition_base.cpp's own comment on the identical pattern --
  // a declared default would make getInput() always report success and
  // silently blank out the params.default_port_value fallback above.
  getInput("service_name", service_name_);
  client_ = node_->create_client<PlanPath>(service_name_);
  timeout_ = params.server_timeout;
}

BT::PortsList PlanWith::providedPorts() {
  return {
      BT::InputPort<std::string>("service_name", "ROS2 service name"),
      BT::InputPort<std::string>("algorithm"),
      // "X;Y" -- required for astar/straight/voronoi, omitted for
      // follow_boarder (see PlanPath.srv's own header).
      BT::InputPort<std::string>("goal", "", "target point, \"X;Y\""),
      BT::InputPort<std::string>("obstacle_id", "", "required for follow_boarder"),
      BT::InputPort<double>("offset", 0.0, "required for follow_boarder"),
      // Output ports: control_points is always wired via a shared
      // blackboard key ({cp}), never a literal, so storing the raw
      // std::vector<geometry_msgs::msg::Point> directly needs no string
      // conversion -- see move_to.hpp's own note on reading it back.
      BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("control_points"),
      BT::OutputPort<std::string>("reason"),
      BT::OutputPort<bool>("status"),
  };
}

bool PlanWith::setRequest(Request::SharedPtr &request) {
  std::string algorithm;
  if (!getInput("algorithm", algorithm)) {
    RCLCPP_ERROR(logger(), "PlanWith: missing required input [algorithm]");
    return false;
  }
  request->algorithm = algorithm;

  std::string goal_text;
  if (getInput("goal", goal_text) && !goal_text.empty()) {
    double gx = 0.0, gy = 0.0;
    if (!parsePoint(goal_text, gx, gy)) {
      RCLCPP_ERROR(logger(), "PlanWith: malformed goal port '%s', expected \"X;Y\"",
                   goal_text.c_str());
      return false;
    }
    request->goal_x = gx;
    request->goal_y = gy;
  }

  std::string obstacle_id;
  getInput("obstacle_id", obstacle_id);
  request->obstacle_id = obstacle_id;

  double offset = 0.0;
  getInput("offset", offset);
  request->offset = offset;

  // Throttled: this leaf sits inside a ReactiveSequence ahead of MoveTo
  // (see this class's own header), so it's re-run and re-logged on
  // EVERY tree tick (~20-50ms) for as long as MoveTo is walking, not
  // just once -- an unthrottled RCLCPP_INFO here floods the terminal
  // with an unchanging message for the entire length of every walk.
  RCLCPP_INFO_THROTTLE(logger(), *node_->get_clock(), 2000,
                       "PlanWith: requesting algorithm=%s", algorithm.c_str());
  return true;
}

BT::NodeStatus PlanWith::tick() {
  Request::SharedPtr request = std::make_shared<Request>();
  if (!setRequest(request)) {
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_service(timeout_)) {
    return onFailure("service '" + service_name_ + "' is not reachable");
  }

  // Humble's Client::async_send_request() returns a plain
  // std::shared_future<Response::SharedPtr> directly -- see
  // evaluate_condition_base.cpp's own comment on the same rclcpp
  // version detail.
  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return onFailure("call to '" + service_name_ + "' failed or timed out");
  }
  return onResponseReceived(future.get());
}

BT::NodeStatus PlanWith::onResponseReceived(const Response::SharedPtr &response) {
  setOutput("control_points", response->control_points);
  setOutput("reason", response->reason);
  setOutput("status", response->status);

  if (response->status) {
    RCLCPP_INFO_THROTTLE(logger(), *node_->get_clock(), 2000,
                         "PlanWith: completed (%zu control points)",
                         response->control_points.size());
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(logger(), "PlanWith: failed, reason=%s", response->reason.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PlanWith::onFailure(const std::string &error_detail) {
  RCLCPP_ERROR(logger(), "PlanWith: service call failed: %s", error_detail.c_str());
  setOutput("reason", std::string("service_call_failed"));
  setOutput("status", false);
  return BT::NodeStatus::FAILURE;
}

}  // namespace amiga_bt
