#pragma once

#include <amiga_interfaces/action/retract_tool.hpp>

#include "amiga_ros2_behavior_tree/actions/tool_action_base.hpp"

namespace amiga_bt {

// See tool_action_base.hpp's own header for the full story -- every
// line of actual logic lives there; this class only pins ActionT to
// amiga_interfaces::action::RetractTool (dialing amiga_ros2_planners'
// tool_action_node's own "retract_tool" action).
class RetractTool : public ToolActionBase<amiga_interfaces::action::RetractTool> {
 public:
  using ToolActionBase<amiga_interfaces::action::RetractTool>::ToolActionBase;
};

}  // namespace amiga_bt
