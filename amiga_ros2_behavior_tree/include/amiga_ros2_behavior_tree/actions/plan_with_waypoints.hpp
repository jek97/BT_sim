#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <amiga_interfaces/srv/plan_path_waypoints.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

namespace amiga_bt {

using PlanPathWaypoints = amiga_interfaces::srv::PlanPathWaypoints;

// The ROS2-service form of problog_project's PlanWithWaypoints -- the
// multi-waypoint generalization of PlanWith's own astar/straight
// algorithms, dialing amiga_ros2_planners' plan_service_node's own
// SEPARATE "plan_path_waypoints" service (a distinct service from
// PlanWith's own "plan_path" -- see PlanPathWaypoints.srv's own header
// for why). See module/contracts/schema.yaml's own PlanWithWaypoints
// entry (problog_project submodule) for the full "why this exists"
// rationale.
//
// Deliberately a plain BT::SyncActionNode with a BLOCKING service
// call, NOT BT::RosServiceNode -- same rationale as PlanWith (see
// that class's own header): this leaf sits BEFORE MoveTo in a
// ReactiveSequence, and an async node here would tear down MoveTo's
// own in-flight FollowPath goal on every re-tick.
class PlanWithWaypoints : public BT::SyncActionNode {
 public:
  using Request = PlanPathWaypoints::Request;
  using Response = PlanPathWaypoints::Response;

  PlanWithWaypoints(const std::string &name, const BT::NodeConfig &config,
                    const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  bool setRequest(Request::SharedPtr &request);
  BT::NodeStatus onResponseReceived(const Response::SharedPtr &response);
  BT::NodeStatus onFailure(const std::string &error_detail);

  rclcpp::Logger logger() const { return node_->get_logger(); }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<PlanPathWaypoints>::SharedPtr client_;
  std::string service_name_;
  std::chrono::milliseconds timeout_;
};

}  // namespace amiga_bt
