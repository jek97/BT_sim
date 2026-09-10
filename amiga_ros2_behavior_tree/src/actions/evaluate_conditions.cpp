#include "amiga_ros2_behavior_tree/actions/evaluate_conditions.hpp"

#include "amiga_ros2_behavior_tree/point_port.hpp"

namespace amiga_bt {

// -- Distance* (goal + threshold) ---------------------------------------

BT::PortsList DistanceBelow::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<std::string>("goal", "target point, \"X;Y\""),
      BT::InputPort<double>("threshold"),
  });
}

bool DistanceBelow::setRequest(Request::SharedPtr &request) {
  std::string goal_text;
  double gx = 0.0, gy = 0.0, threshold = 0.0;
  if (!getInput("goal", goal_text) || !parsePoint(goal_text, gx, gy) ||
      !getInput("threshold", threshold)) {
    RCLCPP_ERROR(logger(), "DistanceBelow: missing/malformed goal or threshold");
    return false;
  }
  request->condition = "DistanceBelow";
  request->goal_x = gx;
  request->goal_y = gy;
  request->threshold = threshold;
  return true;
}

BT::PortsList DistanceEqual::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<std::string>("goal", "target point, \"X;Y\""),
      BT::InputPort<double>("threshold"),
  });
}

bool DistanceEqual::setRequest(Request::SharedPtr &request) {
  std::string goal_text;
  double gx = 0.0, gy = 0.0, threshold = 0.0;
  if (!getInput("goal", goal_text) || !parsePoint(goal_text, gx, gy) ||
      !getInput("threshold", threshold)) {
    RCLCPP_ERROR(logger(), "DistanceEqual: missing/malformed goal or threshold");
    return false;
  }
  request->condition = "DistanceEqual";
  request->goal_x = gx;
  request->goal_y = gy;
  request->threshold = threshold;
  return true;
}

BT::PortsList DistanceOver::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<std::string>("goal", "target point, \"X;Y\""),
      BT::InputPort<double>("threshold"),
  });
}

bool DistanceOver::setRequest(Request::SharedPtr &request) {
  std::string goal_text;
  double gx = 0.0, gy = 0.0, threshold = 0.0;
  if (!getInput("goal", goal_text) || !parsePoint(goal_text, gx, gy) ||
      !getInput("threshold", threshold)) {
    RCLCPP_ERROR(logger(), "DistanceOver: missing/malformed goal or threshold");
    return false;
  }
  request->condition = "DistanceOver";
  request->goal_x = gx;
  request->goal_y = gy;
  request->threshold = threshold;
  return true;
}

// -- Obstacle* (threshold only) ------------------------------------------

BT::PortsList ObstacleInBound::providedPorts() {
  return providedBasicPorts({BT::InputPort<double>("threshold")});
}

bool ObstacleInBound::setRequest(Request::SharedPtr &request) {
  double threshold = 0.0;
  if (!getInput("threshold", threshold)) {
    RCLCPP_ERROR(logger(), "ObstacleInBound: missing required input [threshold]");
    return false;
  }
  request->condition = "ObstacleInBound";
  request->threshold = threshold;
  return true;
}

BT::PortsList ObstacleOnPath::providedPorts() {
  return providedBasicPorts({BT::InputPort<double>("threshold")});
}

bool ObstacleOnPath::setRequest(Request::SharedPtr &request) {
  double threshold = 0.0;
  if (!getInput("threshold", threshold)) {
    RCLCPP_ERROR(logger(), "ObstacleOnPath: missing required input [threshold]");
    return false;
  }
  request->condition = "ObstacleOnPath";
  request->threshold = threshold;
  return true;
}

// -- Battery* (threshold only, percent) -----------------------------------

BT::PortsList BatteryBelow::providedPorts() {
  return providedBasicPorts({BT::InputPort<double>("threshold")});
}

bool BatteryBelow::setRequest(Request::SharedPtr &request) {
  double threshold = 0.0;
  if (!getInput("threshold", threshold)) {
    RCLCPP_ERROR(logger(), "BatteryBelow: missing required input [threshold]");
    return false;
  }
  request->condition = "BatteryBelow";
  request->threshold = threshold;
  return true;
}

BT::PortsList BatteryEqual::providedPorts() {
  return providedBasicPorts({BT::InputPort<double>("threshold")});
}

bool BatteryEqual::setRequest(Request::SharedPtr &request) {
  double threshold = 0.0;
  if (!getInput("threshold", threshold)) {
    RCLCPP_ERROR(logger(), "BatteryEqual: missing required input [threshold]");
    return false;
  }
  request->condition = "BatteryEqual";
  request->threshold = threshold;
  return true;
}

BT::PortsList BatteryOver::providedPorts() {
  return providedBasicPorts({BT::InputPort<double>("threshold")});
}

bool BatteryOver::setRequest(Request::SharedPtr &request) {
  double threshold = 0.0;
  if (!getInput("threshold", threshold)) {
    RCLCPP_ERROR(logger(), "BatteryOver: missing required input [threshold]");
    return false;
  }
  request->condition = "BatteryOver";
  request->threshold = threshold;
  return true;
}

// -- LineOfSightClear (obstacle_id + goal) --------------------------------

BT::PortsList LineOfSightClear::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<std::string>("obstacle_id"),
      BT::InputPort<std::string>("goal", "target point, \"X;Y\""),
  });
}

bool LineOfSightClear::setRequest(Request::SharedPtr &request) {
  std::string obstacle_id, goal_text;
  double gx = 0.0, gy = 0.0;
  if (!getInput("obstacle_id", obstacle_id) || obstacle_id.empty() ||
      !getInput("goal", goal_text) || !parsePoint(goal_text, gx, gy)) {
    RCLCPP_ERROR(logger(), "LineOfSightClear: missing/malformed obstacle_id or goal");
    return false;
  }
  request->condition = "LineOfSightClear";
  request->obstacle_id = obstacle_id;
  request->goal_x = gx;
  request->goal_y = gy;
  return true;
}

}  // namespace amiga_bt
