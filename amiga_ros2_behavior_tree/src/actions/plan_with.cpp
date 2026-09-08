#include "amiga_ros2_behavior_tree/actions/plan_with.hpp"

#include "amiga_ros2_behavior_tree/point_port.hpp"

namespace amiga_bt {

PlanWith::PlanWith(const std::string &name, const BT::NodeConfig &config,
                   const BT::RosNodeParams &params)
    : BT::RosServiceNode<PlanPath>(name, config, params) {}

BT::PortsList PlanWith::providedPorts() {
  return providedBasicPorts({
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
  });
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

  RCLCPP_INFO(logger(), "PlanWith: requesting algorithm=%s", algorithm.c_str());
  return true;
}

BT::NodeStatus PlanWith::onResponseReceived(const Response::SharedPtr &response) {
  setOutput("control_points", response->control_points);
  setOutput("reason", response->reason);
  setOutput("status", response->status);

  if (response->status) {
    RCLCPP_INFO(logger(), "PlanWith: completed (%zu control points)",
                response->control_points.size());
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(logger(), "PlanWith: failed, reason=%s", response->reason.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PlanWith::onFailure(BT::ServiceNodeErrorCode error) {
  RCLCPP_ERROR(logger(), "PlanWith: service call failed, error code %d", int(error));
  setOutput("reason", std::string("service_call_failed"));
  setOutput("status", false);
  return BT::NodeStatus::FAILURE;
}

}  // namespace amiga_bt
