#pragma once

#include "amiga_ros2_behavior_tree/actions/evaluate_condition_base.hpp"

// Twelve small leaves, one per problog_project Condition EXCEPT
// HaltedWith (see evaluate_condition_base.hpp's own header). Every
// class here differs from its siblings only in providedPorts()/
// setRequest() -- which EvaluateCondition::Request fields it fills in
// and which `condition` name it sends -- SUCCESS/FAILURE mapping is
// entirely inherited from EvaluateConditionBase, unchanged.

namespace amiga_bt {

class DistanceBelow : public EvaluateConditionBase {
 public:
  using EvaluateConditionBase::EvaluateConditionBase;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr &request) override;
};

class DistanceEqual : public EvaluateConditionBase {
 public:
  using EvaluateConditionBase::EvaluateConditionBase;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr &request) override;
};

class DistanceOver : public EvaluateConditionBase {
 public:
  using EvaluateConditionBase::EvaluateConditionBase;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr &request) override;
};

class ObstacleInBound : public EvaluateConditionBase {
 public:
  using EvaluateConditionBase::EvaluateConditionBase;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr &request) override;
};

class ObstacleOnPath : public EvaluateConditionBase {
 public:
  using EvaluateConditionBase::EvaluateConditionBase;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr &request) override;
};

class BatteryBelow : public EvaluateConditionBase {
 public:
  using EvaluateConditionBase::EvaluateConditionBase;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr &request) override;
};

class BatteryEqual : public EvaluateConditionBase {
 public:
  using EvaluateConditionBase::EvaluateConditionBase;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr &request) override;
};

class BatteryOver : public EvaluateConditionBase {
 public:
  using EvaluateConditionBase::EvaluateConditionBase;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr &request) override;
};

class LineOfSightClear : public EvaluateConditionBase {
 public:
  using EvaluateConditionBase::EvaluateConditionBase;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr &request) override;
};

class SampleValueBelow : public EvaluateConditionBase {
 public:
  using EvaluateConditionBase::EvaluateConditionBase;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr &request) override;
};

class SampleValueEqual : public EvaluateConditionBase {
 public:
  using EvaluateConditionBase::EvaluateConditionBase;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr &request) override;
};

class SampleValueOver : public EvaluateConditionBase {
 public:
  using EvaluateConditionBase::EvaluateConditionBase;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr &request) override;
};

}  // namespace amiga_bt
