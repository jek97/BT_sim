#pragma once

#include <sstream>
#include <stdexcept>
#include <string>

namespace amiga_bt {

// Parses a Point port's own "X;Y" literal encoding -- the same
// convention every problog_project tree already uses for goal="..."
// ports (e.g. goal="11.675;11.525", see
// problems/problem0/behavior_tree.xml), which amiga_ros2_planners'
// amiga_btcpp_planners.xsd types as a plain string (portValueType) for
// exactly this reason: BT.cpp has no native "Point" attribute type, and
// a combined "X;Y" string is what these trees already write. Shared by
// PlanWith and every goal-taking condition (DistanceBelow/Equal/Over,
// LineOfSightClear) rather than parsed once per leaf.
inline bool parsePoint(const std::string &text, double &x, double &y) {
  auto pos = text.find(';');
  if (pos == std::string::npos) {
    return false;
  }
  try {
    x = std::stod(text.substr(0, pos));
    y = std::stod(text.substr(pos + 1));
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

// The inverse of parsePoint above -- formats (x,y) as the SAME "X;Y"
// literal encoding, for a leaf whose OWN output port is a Point (e.g.
// NearestToolOfKind's own `position`, typically bound straight into a
// LATER leaf's own goal="{...}"/p1="{...}" Point-typed input port, no
// separate conversion step needed on the tree author's own side).
inline std::string formatPoint(double x, double y) {
  std::ostringstream oss;
  oss << x << ';' << y;
  return oss.str();
}

}  // namespace amiga_bt
