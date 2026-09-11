#pragma once

#include <amiga_interfaces/action/deploy_tool.hpp>

#include "amiga_ros2_behavior_tree/actions/tool_action_base.hpp"

namespace amiga_bt {

// See tool_action_base.hpp's own header for the full story -- every
// line of actual logic lives there; this class only pins ActionT to
// amiga_interfaces::action::DeployTool (dialing amiga_ros2_planners'
// tool_action_node's own "deploy_tool" action).
class DeployTool : public ToolActionBase<amiga_interfaces::action::DeployTool> {
 public:
  using ToolActionBase<amiga_interfaces::action::DeployTool>::ToolActionBase;
};

}  // namespace amiga_bt
