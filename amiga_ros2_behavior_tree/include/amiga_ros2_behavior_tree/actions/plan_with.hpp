#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <amiga_interfaces/srv/plan_path.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>

namespace amiga_bt {

using PlanPath = amiga_interfaces::srv::PlanPath;

// The ROS2-service form of problog_project's PlanWith -- one
// consolidated leaf covering astar/straight/voronoi/follow_boarder, all
// dialing amiga_ros2_planners' plan_service_node (same
// consolidation problog_project's own schema.yaml/bt_actions.py
// describe for that project's BT.cpp action). See PlanPath.srv's own
// header for the service contract this leaf just forwards to/from.
//
// Deliberately a plain BT::SyncActionNode with a BLOCKING service
// call, NOT BT::RosServiceNode (async, multi-tick) -- see
// evaluate_condition_base.hpp's own header for the full story: this
// leaf sits BEFORE MoveTo in problog-derived ReactiveSequence trees,
// and BT.cpp's ReactiveSequence re-ticks every earlier sibling from
// scratch (back to IDLE, not served from a cached SUCCESS) on every
// single tree tick. An async PlanWith would therefore go RUNNING
// again the very first tick after MoveTo starts walking, and
// ReactiveSequence halts every child AFTER whichever one just
// returned RUNNING -- cancelling MoveTo's own in-flight FollowPath
// goal every time, before the robot could ever finish a single leg
// (observed as "Goal was canceled. Stopping the robot." moments after
// "MoveTo: starting walk", repeating forever). Only the LAST child in
// a ReactiveSequence is safe to be genuinely multi-tick; every earlier
// one -- this one included -- must resolve within its own tick(),
// exactly like EvaluateConditionBase now does.
class PlanWith : public BT::SyncActionNode {
 public:
  using Request = PlanPath::Request;
  using Response = PlanPath::Response;

  PlanWith(const std::string &name, const BT::NodeConfig &config,
           const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  bool setRequest(Request::SharedPtr &request);
  BT::NodeStatus onResponseReceived(const Response::SharedPtr &response);
  BT::NodeStatus onFailure(const std::string &error_detail);

  rclcpp::Logger logger() const { return node_->get_logger(); }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<PlanPath>::SharedPtr client_;
  std::string service_name_;
  std::chrono::milliseconds timeout_;
};

}  // namespace amiga_bt
