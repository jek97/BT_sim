#pragma once

#include <amiga_interfaces/action/install_tool.hpp>

#include "amiga_ros2_behavior_tree/actions/tool_action_base.hpp"

namespace amiga_bt {

// See tool_action_base.hpp's own header for the full story -- every
// line of actual logic lives there; this class only pins ActionT to
// amiga_interfaces::action::InstallTool (dialing amiga_ros2_planners'
// tool_action_node's own "install_tool" action).
class InstallTool : public ToolActionBase<amiga_interfaces::action::InstallTool> {
 public:
  using ToolActionBase<amiga_interfaces::action::InstallTool>::ToolActionBase;
};

}  // namespace amiga_bt
