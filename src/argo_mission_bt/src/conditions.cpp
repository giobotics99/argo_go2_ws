// Condition nodes of the mission BT (README Phase 5) plus two "wait until" nodes used by R1 / R2.

#include <string>

#include "argo_mission_bt/context.hpp"

namespace argo
{

// SOC >= min_soc (an unknown battery level counts as OK)
class BatteryOK : public BT::ConditionNode
{
public:
  BatteryOK(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<double>("min_soc", 15.0, "%")};}
  BT::NodeStatus tick() override
  {
    const auto ctx = ctx_of(*this);
    const double min = getInput<double>("min_soc").value_or(15.0);
    return (ctx->soc < 0 || ctx->soc >= min) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class BatteryLow : public BT::ConditionNode
{
public:
  BatteryLow(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<double>("min_soc", 15.0, "%")};}
  BT::NodeStatus tick() override
  {
    const auto ctx = ctx_of(*this);
    const double min = getInput<double>("min_soc").value_or(15.0);
    return (ctx->soc >= 0 && ctx->soc < min) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// /argo/health from health_monitor: FATAL -> FAILURE with the faulty sources in "fault".
// No message (or older than 5 s, health_monitor not running) counts as OK.
class SensorsOK : public BT::ConditionNode
{
public:
  SensorsOK(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() {return {BT::OutputPort<std::string>("fault")};}
  BT::NodeStatus tick() override
  {
    const auto ctx = ctx_of(*this);
    std::lock_guard<std::mutex> lock(ctx->mutex);
    const bool fresh = ctx->health_stamp.nanoseconds() > 0 &&
      (ctx->node->now() - ctx->health_stamp).seconds() < 5.0;
    if (fresh && ctx->health_status == "FATAL") {
      setOutput("fault", ctx->health_faults);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

// A mission goal is set in the problem expert
class MissionActive : public BT::ConditionNode
{
public:
  MissionActive(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    return ctx_of(*this)->goal_active() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class GoalSatisfied : public BT::ConditionNode
{
public:
  GoalSatisfied(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    const auto ctx = ctx_of(*this);
    auto goal = ctx->problem->getGoal();
    return (!goal.nodes.empty() && ctx->problem->isGoalSatisfied(goal)) ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsTrue : public BT::ConditionNode
{
public:
  IsTrue(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<bool>("value")};}
  BT::NodeStatus tick() override
  {
    return getInput<bool>("value").value_or(false) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// R1: stay here (RUNNING) until the battery is above min_soc again
class WaitBatteryOK : public BT::StatefulActionNode
{
public:
  WaitBatteryOK(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<double>("min_soc", 20.0, "%")};}
  BT::NodeStatus onStart() override {return onRunning();}
  BT::NodeStatus onRunning() override
  {
    const auto ctx = ctx_of(*this);
    const double min = getInput<double>("min_soc").value_or(20.0);
    return (ctx->soc < 0 || ctx->soc >= min) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}
};

// R2: stay here (RUNNING) until /argo/health is no longer FATAL
class WaitHealthy : public BT::StatefulActionNode
{
public:
  WaitHealthy(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override {return onRunning();}
  BT::NodeStatus onRunning() override
  {
    const auto ctx = ctx_of(*this);
    std::lock_guard<std::mutex> lock(ctx->mutex);
    return ctx->health_status == "FATAL" ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {}
};

void register_conditions(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<BatteryOK>("BatteryOK");
  factory.registerNodeType<BatteryLow>("BatteryLow");
  factory.registerNodeType<SensorsOK>("SensorsOK");
  factory.registerNodeType<MissionActive>("MissionActive");
  factory.registerNodeType<GoalSatisfied>("GoalSatisfied");
  factory.registerNodeType<IsTrue>("IsTrue");
  factory.registerNodeType<WaitBatteryOK>("WaitBatteryOK");
  factory.registerNodeType<WaitHealthy>("WaitHealthy");
}

}  // namespace argo
