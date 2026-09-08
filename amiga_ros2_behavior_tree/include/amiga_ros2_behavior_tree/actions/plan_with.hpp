#pragma once

#include <amiga_interfaces/srv/plan_path.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <vector>

namespace amiga_bt {

using PlanPath = amiga_interfaces::srv::PlanPath;

// The ROS2-service form of problog_project's PlanWith -- one
// consolidated leaf covering astar/straight/voronoi/follow_boarder, all
// dialing amiga_ros2_planners' plan_service_node (same
// consolidation problog_project's own schema.yaml/bt_actions.py
// describe for that project's BT.cpp action). See PlanPath.srv's own
// header for the service contract this leaf just forwards to/from.
class PlanWith : public BT::RosServiceNode<PlanPath> {
 public:
  PlanWith(const std::string &name, const BT::NodeConfig &config,
           const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr &request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr &response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

}  // namespace amiga_bt
