// PlanSys2 nodes of the mission BT (README Phase 5): SyncKB, SetGoals, SetHomeGoal, ClearMission,
// ComputePlan, ExecutePlan (non-blocking), CancelPlan and ApplyStrategy (the recovery switch).

#include <algorithm>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "argo_mission_bt/context.hpp"
#include "plansys2_pddl_parser/Utils.h"

namespace argo
{

static std::vector<std::string> split_csv(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(0, item.find_first_not_of(" \t"));
    item.erase(item.find_last_not_of(" \t") + 1);
    if (!item.empty()) {out.push_back(item);}
  }
  return out;
}

static std::vector<std::string> args_of(const std::string & action)
{
  // "(navigate go2 hub_west vito_door):13601" -> {navigate, go2, hub_west, vito_door}
  std::string a = action.substr(0, action.find(')'));
  a.erase(std::remove(a.begin(), a.end(), '('), a.end());
  std::vector<std::string> out;
  std::stringstream ss(a);
  std::string w;
  while (ss >> w) {out.push_back(w);}
  return out;
}

// Write the facts and the goal of the current mission (ctx->targets minus ctx->dropped).
// Achieved facts (announced, message_delivered, inspected) are never removed here.
static bool write_mission(const ContextPtr & ctx)
{
  const std::string r = ctx->onto.robot;
  std::vector<std::string> remaining;
  for (const auto & t : ctx->targets) {
    if (std::find(ctx->dropped.begin(), ctx->dropped.end(), t) == ctx->dropped.end()) {
      remaining.push_back(t);
    }
  }
  std::vector<std::string> nodes;   // where each target is served (door of an office)
  for (const auto & t : remaining) {nodes.push_back(ctx->onto.delivery_node(t));}

  std::vector<std::string> goal;
  if (ctx->mission == "coffee_call") {
    int to_call = 0;
    const auto announced = ctx->predicates("announced");
    for (const auto & n : nodes) {
      if (std::find(announced.begin(), announced.end(), std::vector<std::string>{n}) == announced.end()) {
        ctx->add("(needs_call " + n + ")");
        ++to_call;
      }
      goal.push_back("(announced " + n + ")");
    }
    ctx->set_function("(to_call " + r + ")", to_call);
    const auto social = ctx->onto.social_spot();
    if (!social.empty()) {goal.push_back("(socialized " + social + ")");}
  } else if (ctx->mission == "message" || ctx->mission == "inspect") {
    if (ctx->ordered && nodes.size() > 1) {
      ctx->add("(unlocked " + nodes.front() + ")");
      for (size_t i = 0; i + 1 < nodes.size(); ++i) {
        ctx->add("(precedes " + nodes[i] + " " + nodes[i + 1] + ")");
      }
    } else {
      for (const auto & n : nodes) {ctx->add("(unlocked " + n + ")");}
    }
    for (const auto & n : nodes) {
      if (ctx->mission == "message") {
        ctx->add("(has_message " + n + ")");
        goal.push_back("(message_delivered " + n + ")");
      } else {
        goal.push_back("(inspected " + n + ")");
      }
    }
    if (ctx->mission == "message") {ctx->publish_messages();}
  }
  return ctx->set_goal(goal);
}

// robot_at = nearest node to the AMCL pose; battery from the SOC
class SyncKB : public BT::SyncActionNode
{
public:
  SyncKB(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    const auto ctx = ctx_of(*this);
    const std::string r = ctx->onto.robot;
    // AMCL when available; otherwise keep what the KB says (home only if nothing is known)
    const std::string known = ctx->robot_node();
    const std::string here = ctx->robot_xy ? ctx->nearest_node() : (known.empty() ? ctx->onto.home : known);
    for (const auto & args : ctx->predicates("robot_at")) {
      if (args.size() == 2) {ctx->remove("(robot_at " + args[0] + " " + args[1] + ")");}
    }
    ctx->add("(robot_at " + r + " " + here + ")");
    if (ctx->soc >= 0) {ctx->set_function("(battery " + r + ")", ctx->soc);}
    RCLCPP_INFO(ctx->node->get_logger(), "[SyncKB] robot_at %s, batteria %.0f%%", here.c_str(), ctx->soc);
    return BT::NodeStatus::SUCCESS;
  }
};

class SetGoals : public BT::SyncActionNode
{
public:
  SetGoals(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("mission"), BT::InputPort<std::string>("goals"),
      BT::InputPort<bool>("ordered"), BT::InputPort<std::string>("message")};
  }
  BT::NodeStatus tick() override
  {
    const auto ctx = ctx_of(*this);
    ctx->mission = getInput<std::string>("mission").value_or("none");
    ctx->targets = split_csv(getInput<std::string>("goals").value_or(""));
    ctx->ordered = getInput<bool>("ordered").value_or(false);
    ctx->message_text = getInput<std::string>("message").value_or("");
    ctx->dropped.clear();
    ctx->blocked.clear();
    ctx->attempts = 0;
    ctx->failed_action.clear();
    ctx->returned_home_early = false;
    ctx->clear_mission_facts();
    ctx->set_function("(reserve " + ctx->onto.robot + ")", ctx->onto.reserve);
    const bool ok = write_mission(ctx);
    RCLCPP_INFO(
      ctx->node->get_logger(), "[SetGoals] %s %s ordered=%d -> %s", ctx->mission.c_str(),
      getInput<std::string>("goals").value_or("").c_str(), ctx->ordered,
      parser::pddl::toString(ctx->problem->getGoal()).c_str());
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// Goal = home only (R1, ReturnHome). reserve 0 = the R1 trip home may use the battery reserve.
class SetHomeGoal : public BT::SyncActionNode
{
public:
  SetHomeGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<double>("reserve", -1.0, "-1 = ontology")};}
  BT::NodeStatus tick() override
  {
    const auto ctx = ctx_of(*this);
    double reserve = getInput<double>("reserve").value_or(-1.0);
    if (reserve < 0) {reserve = ctx->onto.reserve;}
    ctx->set_function("(reserve " + ctx->onto.robot + ")", reserve);
    if (ctx->mission != "none") {ctx->returned_home_early = true;}
    return ctx->set_goal({}) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class ClearMission : public BT::SyncActionNode
{
public:
  ClearMission(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    const auto ctx = ctx_of(*this);
    ctx->problem->clearGoal();
    ctx->clear_mission_facts();
    for (const auto & [a, b] : ctx->blocked) {   // forget blocked passages for the next mission
      ctx->add("(path_free " + a + " " + b + ")");
      ctx->add("(path_free " + b + " " + a + ")");
    }
    ctx->blocked.clear();
    ctx->set_function("(reserve " + ctx->onto.robot + ")", ctx->onto.reserve);
    ctx->mission = "none";
    ctx->targets.clear();
    ctx->dropped.clear();
    ctx->plan.reset();
    return BT::NodeStatus::SUCCESS;
  }
};

class ComputePlan : public BT::SyncActionNode
{
public:
  ComputePlan(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    const auto ctx = ctx_of(*this);
    // Goal already true (e.g. RETURN_HOME while already at home): nothing to plan, not a failure
    auto goal = ctx->problem->getGoal();
    if (!goal.nodes.empty() && ctx->problem->isGoalSatisfied(goal)) {
      ctx->plan = plansys2_msgs::msg::Plan();
      RCLCPP_INFO(ctx->node->get_logger(), "[ComputePlan] obiettivo già raggiunto: niente da fare");
      return BT::NodeStatus::SUCCESS;
    }
    ctx->plan = ctx->planner->getPlan(ctx->domain->getDomain(), ctx->problem->getProblem());
    if (!ctx->plan || ctx->plan->items.empty()) {
      // empty plan with an unsatisfied goal, or no plan at all
      RCLCPP_WARN(ctx->node->get_logger(), "[ComputePlan] nessun piano");
      ctx->plan.reset();
      ctx->failed_action = "(plan)";
      return BT::NodeStatus::FAILURE;
    }
    std::string text;
    for (const auto & it : ctx->plan->items) {
      text += "\n  " + std::to_string(it.time) + ": " + it.action + " [" + std::to_string(it.duration) + "]";
    }
    RCLCPP_INFO(ctx->node->get_logger(), "[ComputePlan] %zu azioni:%s", ctx->plan->items.size(), text.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

// Runs the plan in the PlanSys2 executor without blocking the tick; on halt the plan is cancelled.
class ExecutePlan : public BT::StatefulActionNode
{
public:
  ExecutePlan(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override
  {
    const auto ctx = ctx_of(*this);
    if (!ctx->plan) {return BT::NodeStatus::FAILURE;}
    ctx->failed_action.clear();
    if (ctx->plan->items.empty()) {return BT::NodeStatus::SUCCESS;}   // goal already reached
    if (!ctx->executor->start_plan_execution(*ctx->plan)) {
      RCLCPP_ERROR(ctx->node->get_logger(), "[ExecutePlan] l'executor non accetta il piano");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const auto ctx = ctx_of(*this);
    if (ctx->executor->execute_and_check_plan()) {
      return BT::NodeStatus::RUNNING;
    }
    auto result = ctx->executor->getResult();
    if (result && result->success) {
      RCLCPP_INFO(ctx->node->get_logger(), "[ExecutePlan] piano completato");
      return BT::NodeStatus::SUCCESS;
    }
    ctx->failed_action = "(unknown)";
    if (result) {
      for (const auto & info : result->action_execution_status) {
        if (info.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
          ctx->failed_action = info.action_full_name;
          break;
        }
      }
    }
    // A failed navigate means that passage is blocked for now: tell the planner. Its "at start"
    // effect removed robot_at: the robot is still near where it left from (SyncKB refines it with AMCL).
    const auto args = args_of(ctx->failed_action);
    if (args.size() == 4 && args[0] == "navigate") {
      if (ctx->robot_node().empty()) {ctx->add("(robot_at " + args[1] + " " + args[2] + ")");}
      ctx->remove("(path_free " + args[2] + " " + args[3] + ")");
      ctx->remove("(path_free " + args[3] + " " + args[2] + ")");
      ctx->blocked.emplace_back(args[2], args[3]);
    }
    RCLCPP_WARN(ctx->node->get_logger(), "[ExecutePlan] fallita: %s", ctx->failed_action.c_str());
    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    const auto ctx = ctx_of(*this);
    RCLCPP_WARN(ctx->node->get_logger(), "[ExecutePlan] interrotta: annullo il piano");
    ctx->executor->cancel_plan_execution();
  }
};

class CancelPlan : public BT::SyncActionNode
{
public:
  CancelPlan(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    ctx_of(*this)->executor->cancel_plan_execution();
    return BT::NodeStatus::SUCCESS;
  }
};

// Apply Gemma's recovery strategy (G3), then the BT retries with the updated knowledge base.
class ApplyStrategy : public BT::StatefulActionNode
{
public:
  ApplyStrategy(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("strategy")};}

  BT::NodeStatus onStart() override
  {
    const auto ctx = ctx_of(*this);
    strategy_ = getInput<std::string>("strategy").value_or("RETURN_HOME");
    ctx->attempts++;
    start_ = ctx->node->now();
    RCLCPP_INFO(ctx->node->get_logger(), "[ApplyStrategy] %s (tentativo %d)", strategy_.c_str(), ctx->attempts);
    if (strategy_ == "WAIT_RETRY") {
      wait_s_ = 20.0;
      return BT::NodeStatus::RUNNING;
    }
    if (strategy_ == "ASK_CLEAR") {
      ctx->say("Mi scusi, posso passare per favore?");
      wait_s_ = 10.0;
      return BT::NodeStatus::RUNNING;
    }
    if (strategy_ == "DROP_UNREACHABLE") {
      drop_unreachable(ctx);
      return BT::NodeStatus::SUCCESS;
    }
    // RETURN_HOME, ABORT or anything unexpected: give up the mission and go home
    ctx->clear_mission_facts();
    ctx->returned_home_early = true;
    ctx->set_goal({});
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onRunning() override
  {
    const auto ctx = ctx_of(*this);
    if ((ctx->node->now() - start_).seconds() < wait_s_) {return BT::NodeStatus::RUNNING;}
    for (const auto & [a, b] : ctx->blocked) {   // try the blocked passages again
      ctx->add("(path_free " + a + " " + b + ")");
      ctx->add("(path_free " + b + " " + a + ")");
    }
    ctx->blocked.clear();
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

private:
  void drop_unreachable(const ContextPtr & ctx)
  {
    const auto reach = ctx->reachable(ctx->robot_node());
    for (const auto & t : ctx->targets) {
      const auto n = ctx->onto.delivery_node(t);
      if (!reach.count(n) && std::find(ctx->dropped.begin(), ctx->dropped.end(), t) == ctx->dropped.end()) {
        ctx->dropped.push_back(t);
        ctx->remove("(needs_call " + n + ")");
        ctx->remove("(has_message " + n + ")");
        ctx->remove("(unlocked " + n + ")");
      }
    }
    write_mission(ctx);
    RCLCPP_INFO(ctx->node->get_logger(), "[ApplyStrategy] abbandonate: %zu stanze", ctx->dropped.size());
  }

  std::string strategy_;
  rclcpp::Time start_{0, 0, RCL_ROS_TIME};
  double wait_s_{0};
};

void register_plansys2_nodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<SyncKB>("SyncKB");
  factory.registerNodeType<SetGoals>("SetGoals");
  factory.registerNodeType<SetHomeGoal>("SetHomeGoal");
  factory.registerNodeType<ClearMission>("ClearMission");
  factory.registerNodeType<ComputePlan>("ComputePlan");
  factory.registerNodeType<ExecutePlan>("ExecutePlan");
  factory.registerNodeType<CancelPlan>("CancelPlan");
  factory.registerNodeType<ApplyStrategy>("ApplyStrategy");
}

}  // namespace argo
