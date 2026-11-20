// Interaction nodes of the mission BT (README Phase 5): WaitCommand, Say, MissionReport, LieDown.

#include <algorithm>
#include <string>

#include "argo_mission_bt/context.hpp"

namespace argo
{

// RUNNING until a command arrives on /argo/command
class WaitCommand : public BT::StatefulActionNode
{
public:
  WaitCommand(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {BT::OutputPort<std::string>("command")};}
  BT::NodeStatus onStart() override {return onRunning();}
  BT::NodeStatus onRunning() override
  {
    const auto ctx = ctx_of(*this);
    std::lock_guard<std::mutex> lock(ctx->mutex);
    if (ctx->commands.empty()) {return BT::NodeStatus::RUNNING;}
    setOutput("command", ctx->commands.front());
    RCLCPP_INFO(ctx->node->get_logger(), "[WaitCommand] '%s'", ctx->commands.front().c_str());
    ctx->commands.pop_front();
    return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {}
};

// Publish on /argo/say and wait until the voice has finished (argo_voice_node: /argo/voice_busy).
// Without a voice node it does not block: it goes on after 8 s.
class Say : public BT::StatefulActionNode
{
public:
  Say(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("text")};}

  BT::NodeStatus onStart() override
  {
    const auto ctx = ctx_of(*this);
    const std::string text = getInput<std::string>("text").value_or("");
    if (text.empty()) {return BT::NodeStatus::SUCCESS;}
    ctx->say(text);
    start_ = ctx->node->now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const auto ctx = ctx_of(*this);
    const double elapsed = (ctx->node->now() - start_).seconds();
    if (!ctx->voice_seen_busy) {
      if (elapsed > 8.0) {
        RCLCPP_WARN(ctx->node->get_logger(), "[Say] la voce non è partita: proseguo");
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::RUNNING;
    }
    return (!ctx->voice_busy || elapsed > 90.0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  rclcpp::Time start_{0, 0, RCL_ROS_TIME};
};

// Sentence that closes a mission (what was done, what was dropped)
class MissionReport : public BT::SyncActionNode
{
public:
  MissionReport(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<std::string>("text"), BT::InputPort<bool>("stuck", false, "could not go home")};
  }
  BT::NodeStatus tick() override
  {
    const auto ctx = ctx_of(*this);
    if (getInput<bool>("stuck").value_or(false)) {
      setOutput("text", std::string("Missione interrotta: sono fermo e aspetto aiuto."));
      return BT::NodeStatus::SUCCESS;
    }
    std::string done;
    for (const auto & t : ctx->targets) {
      if (std::find(ctx->dropped.begin(), ctx->dropped.end(), t) == ctx->dropped.end()) {
        // a person's name for messages to an office ("a Vito"), the place otherwise
        const auto & occ = ctx->onto.places.count(t) ? ctx->onto.places.at(t).occupants :
          std::vector<std::string>{};
        const std::string who = (ctx->mission == "message" && !occ.empty()) ? occ.front() : ctx->spoken(t);
        done += (done.empty() ? "" : ", ") + who;
      }
    }
    std::string skipped;
    for (const auto & t : ctx->dropped) {skipped += (skipped.empty() ? "" : ", ") + ctx->spoken(t);}
    std::string text;
    if (ctx->returned_home_early) {
      text = "Sono tornato in laboratorio senza completare la missione.";
    } else if (!skipped.empty()) {
      text = "Missione completata in parte: non ho potuto raggiungere " + skipped + ".";
    } else if (ctx->mission == "message") {
      text = "Missione completata: ho consegnato il messaggio a " + done + ".";
    } else if (ctx->mission == "coffee_call") {
      text = "Missione completata: ho chiamato tutti per il caffè.";
    } else if (ctx->mission == "inspect") {
      text = "Missione completata: ho ispezionato " + done + ".";
    } else {
      text = "Sono di nuovo in laboratorio.";
    }
    setOutput("text", text);
    return BT::NodeStatus::SUCCESS;
  }
};

// Lie down (R1, at the dock): driver /mode "stand_down"
class LieDown : public BT::SyncActionNode
{
public:
  LieDown(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    const auto ctx = ctx_of(*this);
    if (!ctx->mode_cli->service_is_ready()) {
      RCLCPP_WARN(ctx->node->get_logger(), "[LieDown] servizio /mode non disponibile");
      return BT::NodeStatus::SUCCESS;
    }
    auto req = std::make_shared<go2_interfaces::srv::Mode::Request>();
    req->mode = "stand_down";
    ctx->mode_cli->async_send_request(req);
    return BT::NodeStatus::SUCCESS;
  }
};

void register_interaction_nodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<WaitCommand>("WaitCommand");
  factory.registerNodeType<Say>("Say");
  factory.registerNodeType<MissionReport>("MissionReport");
  factory.registerNodeType<LieDown>("LieDown");
}

}  // namespace argo
