// Publishes this tree's full structure once, then every node's every
// status transition after that, as small JSON strings on one topic --
// visual:=true's "record the BT execution too" feature (see
// gazebo.launch.py's own video_path docstring for the camera half of
// that flag).
//
// Deliberately split into two processes, not one: this class does
// nothing but publish a JSON string per transition, the only cost paid
// inside bt_runner's own tick loop. All the actual work -- laying the
// tree out with Graphviz, colouring nodes by status, encoding video
// frames -- happens in a separate process (bt_visualizer_node.py,
// amiga_ros2_gazebo), subscribed to that topic. A renderer that falls
// behind, stalls, or crashes can therefore never slow down or block
// tree ticking itself, which is what a synchronous "render every
// transition inline" design could not promise. bt_visualizer_node.py
// also appends every raw event to BT_<name>.jsonl regardless of
// whether it's keeping up with rendering, and can rebuild the same
// video from that file later even with no simulation running at all --
// the "save something similar to a bag file" half of the same request.
//
// Structure derivation mirrors BT.CPP's own
// flatbuffers/bt_flatbuffer_helper.h (the exact serialization its own
// Groot2Publisher uses): applyRecursiveVisitor over the tree, each
// ControlNode's/DecoratorNode's children collected by UID. Copied
// rather than reused directly because pulling in BT.CPP's own
// zmq+flatbuffers Groot2 wire format here would mean giving
// bt_visualizer_node.py a flatbuffer parser and a zmq client just to
// get JSON back out; publishing our own JSON directly over a ROS2
// topic this repo already depends on is the same information with a
// much smaller dependency footprint on the Python side.
#ifndef AMIGA_ROS2_BEHAVIOR_TREE_BT_VISUALIZATION_LOGGER_HPP
#define AMIGA_ROS2_BEHAVIOR_TREE_BT_VISUALIZATION_LOGGER_HPP

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace amiga_bt {

class BTVisualizationLogger : public BT::StatusChangeLogger {
public:
  // Publishes this tree's structure immediately (on construction) -- a
  // fresh bt_visualizer_node.py subscriber that starts after that point
  // still gets it, since the publisher below is TRANSIENT_LOCAL.
  BTVisualizationLogger(
      BT::Tree &tree, rclcpp::Node::SharedPtr node,
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub);

  ~BTVisualizationLogger() override = default;

  // BT::StatusChangeLogger
  void callback(BT::Duration timestamp, const BT::TreeNode &node,
                BT::NodeStatus prev_status, BT::NodeStatus status) override;
  void flush() override {}

private:
  void publishStructure(BT::TreeNode *root_node);
  void publish(const std::string &payload);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

// One publisher for the life of the process, like makeFaultPublisher's own
// (see fault_reporter.hpp) -- a logger is rebuilt per mission, but the
// structure message it just sent must survive that rebuild for a
// visualizer that hasn't connected yet.
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
makeBtVisualizationPublisher(const rclcpp::Node::SharedPtr &node,
                             const std::string &topic);

} // namespace amiga_bt

#endif // AMIGA_ROS2_BEHAVIOR_TREE_BT_VISUALIZATION_LOGGER_HPP
