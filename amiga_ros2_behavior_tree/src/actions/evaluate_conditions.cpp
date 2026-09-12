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

// -- SampleValue* (sample id + threshold) ---------------------------------

BT::PortsList SampleValueBelow::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<std::string>("id", "must match an earlier <TakeSample id=\"...\">"),
      BT::InputPort<double>("threshold"),
  });
}

bool SampleValueBelow::setRequest(Request::SharedPtr &request) {
  std::string id;
  double threshold = 0.0;
  if (!getInput("id", id) || id.empty() || !getInput("threshold", threshold)) {
    RCLCPP_ERROR(logger(), "SampleValueBelow: missing/empty id or threshold");
    return false;
  }
  request->condition = "SampleValueBelow";
  request->sample_id = id;
  request->threshold = threshold;
  return true;
}

BT::PortsList SampleValueEqual::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<std::string>("id", "must match an earlier <TakeSample id=\"...\">"),
      BT::InputPort<double>("threshold"),
  });
}

bool SampleValueEqual::setRequest(Request::SharedPtr &request) {
  std::string id;
  double threshold = 0.0;
  if (!getInput("id", id) || id.empty() || !getInput("threshold", threshold)) {
    RCLCPP_ERROR(logger(), "SampleValueEqual: missing/empty id or threshold");
    return false;
  }
  request->condition = "SampleValueEqual";
  request->sample_id = id;
  request->threshold = threshold;
  return true;
}

BT::PortsList SampleValueOver::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<std::string>("id", "must match an earlier <TakeSample id=\"...\">"),
      BT::InputPort<double>("threshold"),
  });
}

bool SampleValueOver::setRequest(Request::SharedPtr &request) {
  std::string id;
  double threshold = 0.0;
  if (!getInput("id", id) || id.empty() || !getInput("threshold", threshold)) {
    RCLCPP_ERROR(logger(), "SampleValueOver: missing/empty id or threshold");
    return false;
  }
  request->condition = "SampleValueOver";
  request->sample_id = id;
  request->threshold = threshold;
  return true;
}

// -- CollisionDetected (side) ----------------------------------------------

BT::PortsList CollisionDetected::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<std::string>(
          "side", "any", "\"front\", \"back\", \"left\", \"right\", or \"any\""),
  });
}

bool CollisionDetected::setRequest(Request::SharedPtr &request) {
  std::string side = "any";
  getInput("side", side);
  request->condition = "CollisionDetected";
  request->side = side;
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

// -- Hitched (optional kind) / Deployed (no ports) ------------------------

BT::PortsList Hitched::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<std::string>(
          "kind", "", "\"cart\" or \"plow\" -- omit to check \"is ANYTHING attached\"."),
  });
}

bool Hitched::setRequest(Request::SharedPtr &request) {
  std::string kind;
  getInput("kind", kind);
  request->condition = "Hitched";
  request->kind = kind;
  return true;
}

BT::PortsList Deployed::providedPorts() { return providedBasicPorts({}); }

bool Deployed::setRequest(Request::SharedPtr &request) {
  request->condition = "Deployed";
  return true;
}

// -- PloughedAt (goal) / PloughedBetween (p1 + p2) -------------------------

BT::PortsList PloughedAt::providedPorts() {
  return providedBasicPorts({BT::InputPort<std::string>("goal", "point to check, \"X;Y\"")});
}

bool PloughedAt::setRequest(Request::SharedPtr &request) {
  std::string goal_text;
  double gx = 0.0, gy = 0.0;
  if (!getInput("goal", goal_text) || !parsePoint(goal_text, gx, gy)) {
    RCLCPP_ERROR(logger(), "PloughedAt: missing/malformed goal");
    return false;
  }
  request->condition = "PloughedAt";
  request->goal_x = gx;
  request->goal_y = gy;
  return true;
}

BT::PortsList PloughedBetween::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<std::string>("p1", "one end of the swath to check, \"X;Y\""),
      BT::InputPort<std::string>("p2", "the other end of the swath to check, \"X;Y\""),
  });
}

bool PloughedBetween::setRequest(Request::SharedPtr &request) {
  std::string p1_text, p2_text;
  double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0;
  if (!getInput("p1", p1_text) || !parsePoint(p1_text, x1, y1) ||
      !getInput("p2", p2_text) || !parsePoint(p2_text, x2, y2)) {
    RCLCPP_ERROR(logger(), "PloughedBetween: missing/malformed p1 or p2");
    return false;
  }
  request->condition = "PloughedBetween";
  request->goal_x = x1;
  request->goal_y = y1;
  request->p2_x = x2;
  request->p2_y = y2;
  return true;
}

}  // namespace amiga_bt
