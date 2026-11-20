// Gemma nodes of the mission BT (README Phase 5): asynchronous calls to gemma_server.
// They never block the tick: the request is sent in onStart and the answer read in onRunning.
// If gemma_server does not answer, each node has a deterministic fallback.

#include <algorithm>
#include <chrono>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "argo_mission_bt/context.hpp"
#include "nlohmann/json.hpp"

namespace argo
{

static constexpr double SERVICE_TIMEOUT = 90.0;   // s (gemma_server answers with a fallback in ~10 s)

// G1+G2: command -> accepted, mission, goals (comma list of place IDs), ordered, message, reply
class GemmaFormulateGoals : public BT::StatefulActionNode
{
public:
  using Srv = go2_interfaces::srv::FormulateGoals;
  GemmaFormulateGoals(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("command"), BT::OutputPort<bool>("accepted"),
      BT::OutputPort<std::string>("mission"), BT::OutputPort<std::string>("goals"),
      BT::OutputPort<bool>("ordered"), BT::OutputPort<std::string>("message"),
      BT::OutputPort<std::string>("reply")};
  }

  BT::NodeStatus onStart() override
  {
    const auto ctx = ctx_of(*this);
    start_ = ctx->node->now();
    sent_ = false;
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    const auto ctx = ctx_of(*this);
    const double elapsed = (ctx->node->now() - start_).seconds();
    if (!sent_) {
      if (!ctx->formulate_cli->service_is_ready()) {
        if (elapsed > 15.0) {return give_up("Il mio modulo di linguaggio non risponde: riprova tra poco.");}
        return BT::NodeStatus::RUNNING;
      }
      auto req = std::make_shared<Srv::Request>();
      req->command = getInput<std::string>("command").value_or("");
      nlohmann::json context = {{"robot_at", ctx->robot_node()}, {"battery", ctx->soc},
        {"mission_running", ctx->mission}};
      req->context_json = context.dump();
      future_ = ctx->formulate_cli->async_send_request(req);
      sent_ = true;
      return BT::NodeStatus::RUNNING;
    }
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return elapsed > SERVICE_TIMEOUT ? give_up("Non sono riuscito a capire il comando in tempo.") :
             BT::NodeStatus::RUNNING;
    }
    auto res = future_.get();
    std::string goals;
    for (const auto & t : res->targets) {goals += (goals.empty() ? "" : ",") + t;}
    setOutput("accepted", static_cast<bool>(res->accepted));
    setOutput("mission", res->mission);
    setOutput("goals", goals);
    setOutput("ordered", static_cast<bool>(res->ordered));
    setOutput("message", res->message_text);
    setOutput("reply", res->clarify_question.empty() ? res->reply : res->clarify_question);
    RCLCPP_INFO(
      ctx->node->get_logger(), "[G1] accepted=%d mission=%s goals=%s fallback=%d (%.1f s)",
      res->accepted, res->mission.c_str(), goals.c_str(), res->used_fallback, elapsed);
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

private:
  BT::NodeStatus give_up(const std::string & reply)
  {
    setOutput("accepted", false);
    setOutput("reply", reply);
    return BT::NodeStatus::SUCCESS;
  }

  rclcpp::Time start_{0, 0, RCL_ROS_TIME};
  bool sent_{false};
  rclcpp::Client<Srv>::SharedFuture future_;
};

// G3: after a failure, choose a strategy from the closed set (and the sentence to say)
class GemmaAssessFailure : public BT::StatefulActionNode
{
public:
  using Srv = go2_interfaces::srv::AssessFailure;
  GemmaAssessFailure(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<std::string>("strategy"), BT::OutputPort<std::string>("sentence")};
  }

  BT::NodeStatus onStart() override
  {
    const auto ctx = ctx_of(*this);
    start_ = ctx->node->now();
    // Reachable / unreachable targets on the tree, with the blocked passages removed
    const auto reach = ctx->reachable(ctx->robot_node());
    auto req = std::make_shared<Srv::Request>();
    for (const auto & t : ctx->targets) {
      if (std::find(ctx->dropped.begin(), ctx->dropped.end(), t) != ctx->dropped.end()) {continue;}
      (reach.count(ctx->onto.delivery_node(t)) ? req->reachable_targets : req->unreachable_targets).push_back(t);
    }
    const auto open = ctx->failed_action.find('(');
    const auto close = ctx->failed_action.find(')');
    std::string action = (open != std::string::npos && close != std::string::npos) ?
      ctx->failed_action.substr(open + 1, close - open - 1) : ctx->failed_action;
    std::stringstream ss(action);
    ss >> req->failed_action;
    for (std::string w; ss >> w; ) {req->action_args.push_back(w);}
    req->battery = static_cast<float>(ctx->soc);
    req->attempts = ctx->attempts;
    req->detections_json = "{}";
    request_ = req;
    if (!ctx->assess_cli->service_is_ready()) {return fallback();}
    future_ = ctx->assess_cli->async_send_request(req);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const auto ctx = ctx_of(*this);
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return (ctx->node->now() - start_).seconds() > SERVICE_TIMEOUT ? fallback() : BT::NodeStatus::RUNNING;
    }
    auto res = future_.get();
    setOutput("strategy", res->strategy);
    setOutput("sentence", res->sentence);
    RCLCPP_INFO(
      ctx->node->get_logger(), "[G3] %s -> %s: %s", ctx->failed_action.c_str(), res->strategy.c_str(),
      res->sentence.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

private:
  // Same rule as gemma_server's fallback: wait once, then drop the unreachable rooms, else go home
  BT::NodeStatus fallback()
  {
    const auto ctx = ctx_of(*this);
    std::string strategy = "RETURN_HOME", sentence = "Non riesco a proseguire: torno in laboratorio.";
    if (ctx->soc >= 0 && ctx->soc < ctx->onto.reserve) {
      sentence = "Ho poca batteria: interrompo e torno in laboratorio.";
    } else if (request_ && request_->attempts < 1 && request_->failed_action == "navigate") {
      strategy = "WAIT_RETRY";
      sentence = "Il passaggio è bloccato: aspetto un momento e riprovo.";
    } else if (request_ && !request_->reachable_targets.empty() && !request_->unreachable_targets.empty()) {
      strategy = "DROP_UNREACHABLE";
      sentence = "Il passaggio resta bloccato: salto le stanze che non posso raggiungere.";
    }
    setOutput("strategy", strategy);
    setOutput("sentence", sentence);
    RCLCPP_WARN(ctx->node->get_logger(), "[G3] fallback locale -> %s", strategy.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  rclcpp::Time start_{0, 0, RCL_ROS_TIME};
  Srv::Request::SharedPtr request_;
  rclcpp::Client<Srv>::SharedFuture future_;
};

// G4: after an inspection, judge the findings (never a planning goal: report only)
class GemmaAppraiseFindings : public BT::StatefulActionNode
{
public:
  using Srv = go2_interfaces::srv::AppraiseFindings;
  GemmaAppraiseFindings(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override
  {
    const auto ctx = ctx_of(*this);
    std::string findings;
    {
      std::lock_guard<std::mutex> lock(ctx->mutex);
      findings.swap(ctx->findings);
    }
    if (findings.empty() || !ctx->appraise_cli->service_is_ready()) {return BT::NodeStatus::SUCCESS;}
    auto req = std::make_shared<Srv::Request>();
    try {
      auto js = nlohmann::json::parse(findings);
      req->room = js.value("room", "");
      req->detections_json = js.contains("detections") ? js["detections"].dump() : "{}";
    } catch (const std::exception &) {
      return BT::NodeStatus::SUCCESS;
    }
    req->time_of_day = std::to_string(ctx->node->now().seconds());
    start_ = ctx->node->now();
    future_ = ctx->appraise_cli->async_send_request(req);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const auto ctx = ctx_of(*this);
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return (ctx->node->now() - start_).seconds() > SERVICE_TIMEOUT ? BT::NodeStatus::SUCCESS :
             BT::NodeStatus::RUNNING;
    }
    auto res = future_.get();
    if (res->verdict == "REPORT") {
      ctx->say(res->sentence);
      std_msgs::msg::String msg;
      msg.data = res->sentence;
      ctx->report_pub->publish(msg);
    }
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

private:
  rclcpp::Time start_{0, 0, RCL_ROS_TIME};
  rclcpp::Client<Srv>::SharedFuture future_;
};

void register_gemma_nodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<GemmaFormulateGoals>("GemmaFormulateGoals");
  factory.registerNodeType<GemmaAssessFailure>("GemmaAssessFailure");
  factory.registerNodeType<GemmaAppraiseFindings>("GemmaAppraiseFindings");
}

}  // namespace argo
