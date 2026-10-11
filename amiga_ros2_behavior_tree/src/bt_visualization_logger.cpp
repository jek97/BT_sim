#include "amiga_ros2_behavior_tree/bt_visualization_logger.hpp"

#include <nlohmann/json.hpp>

#include "behaviortree_cpp/control_node.h"
#include "behaviortree_cpp/decorator_node.h"

namespace amiga_bt {

using json = nlohmann::json;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
makeBtVisualizationPublisher(const rclcpp::Node::SharedPtr &node,
                             const std::string &topic) {
  // TRANSIENT_LOCAL, same reasoning as makeFaultPublisher's own: the one
  // structure message a mission ever sends must still reach a
  // bt_visualizer_node.py that starts (or restarts) after it.
  rclcpp::QoS qos(50);
  qos.transient_local();
  return node->create_publisher<std_msgs::msg::String>(topic, qos);
}

BTVisualizationLogger::BTVisualizationLogger(
    BT::Tree &tree, rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub)
    : StatusChangeLogger(tree.rootNode()), node_(std::move(node)),
      pub_(std::move(pub)) {
  publishStructure(tree.rootNode());
}

void BTVisualizationLogger::publishStructure(BT::TreeNode *root_node) {
  json nodes = json::array();
  // Same traversal BT.CPP's own flatbuffers/bt_flatbuffer_helper.h uses to
  // feed its Groot2Publisher -- see this file's own header comment for why
  // this reimplements that walk instead of reusing BT.CPP's flatbuffer/zmq
  // wire format directly.
  BT::applyRecursiveVisitor(root_node, [&](BT::TreeNode *node) {
    std::vector<uint16_t> child_uids;
    if (auto *control = dynamic_cast<BT::ControlNode *>(node)) {
      for (const auto *child : control->children()) {
        child_uids.push_back(child->UID());
      }
    } else if (auto *decorator = dynamic_cast<BT::DecoratorNode *>(node)) {
      if (const auto *child = decorator->child()) {
        child_uids.push_back(child->UID());
      }
    }

    json entry;
    entry["uid"] = node->UID();
    entry["name"] = node->name();
    entry["registration_name"] = node->registrationName();
    entry["node_type"] = BT::toStr(node->type());
    entry["children"] = child_uids;
    nodes.push_back(entry);
  });

  json msg;
  msg["kind"] = "structure";
  msg["root_uid"] = root_node->UID();
  msg["nodes"] = nodes;
  msg["timestamp_ms"] = static_cast<int64_t>(node_->now().seconds() * 1000.0);
  publish(msg.dump());
}

void BTVisualizationLogger::callback(BT::Duration /*timestamp*/,
                                     const BT::TreeNode &node,
                                     BT::NodeStatus prev_status,
                                     BT::NodeStatus status) {
  json msg;
  msg["kind"] = "event";
  msg["uid"] = node.UID();
  msg["prev_status"] = BT::toStr(prev_status, false);
  msg["status"] = BT::toStr(status, false);
  msg["timestamp_ms"] = static_cast<int64_t>(node_->now().seconds() * 1000.0);
  publish(msg.dump());
}

void BTVisualizationLogger::publish(const std::string &payload) {
  std_msgs::msg::String out;
  out.data = payload;
  pub_->publish(out);
}

} // namespace amiga_bt
