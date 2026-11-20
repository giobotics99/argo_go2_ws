// Executor of "announce ?r ?l" (M1 coffee call, README Phase 3).
// Asks gemma_server for the sentence (Speak, event "announce"), says it on /argo/say and finishes
// when the voice has finished. The sentence starts with the people of the room (or the room's name),
// so two rooms never get the identical sentence (argo_voice_node drops repeated sentences).
// Fails if the voice does not start ("speaker down").

#include <memory>
#include <string>

#include "go2_interfaces/srv/speak.hpp"
#include "go2_patrol_mission/speaking_action.hpp"
#include "nlohmann/json.hpp"

using namespace std::chrono_literals;
using Speak = go2_interfaces::srv::Speak;

class AnnounceAction : public go2_patrol_mission::SpeakingAction
{
public:
  AnnounceAction()
  : SpeakingAction("announce_action_node", 200ms)
  {
    speak_cli_ = create_client<Speak>("/argo/gemma/speak");
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override
  {
    stage_ = Stage::Ask;
    return SpeakingAction::on_activate(state);
  }

private:
  enum class Stage {Ask, WaitSentence, Speaking, Done};

  void do_work() override
  {
    const auto & args = get_arguments();   // (robot, place)
    const std::string place = args.size() > 1 ? args[1] : "";
    switch (stage_) {
      case Stage::Ask: {
          started_ = now();
          if (speak_cli_->service_is_ready()) {
            auto req = std::make_shared<Speak::Request>();
            req->event_type = "announce";
            req->facts_json = nlohmann::json{{"room", spoken(place)}, {"names", occupants(place)}}.dump();
            future_ = speak_cli_->async_send_request(req);
            stage_ = Stage::WaitSentence;
          } else {
            start_speaking(place, "");
          }
          break;
        }
      case Stage::WaitSentence:
        if (future_.wait_for(0s) == std::future_status::ready) {
          start_speaking(place, future_.get()->sentence);
        } else if ((now() - started_).seconds() > 15.0) {
          start_speaking(place, "");   // gemma_server too slow: fixed sentence
        }
        break;
      case Stage::Speaking: {
          bool timed_out = false;
          if (voice_started(timed_out) && voice_finished()) {
            stage_ = Stage::Done;
            finish(true, 1.0, "annunciato");
          } else if (timed_out) {
            RCLCPP_ERROR(get_logger(), "La voce non è partita: argo_voice_node è acceso?");
            stage_ = Stage::Done;
            finish(false, 0.0, "voce non attiva");
          }
          break;
        }
      case Stage::Done:
        break;   // the timer may tick once more before PlanSys2 deactivates the node
    }
  }

  void start_speaking(const std::string & place, std::string sentence)
  {
    if (sentence.empty()) {sentence = "Pausa caffè! Vi aspetto alla macchinetta del caffè.";}
    const auto people = occupants(place);
    std::string who;
    for (size_t i = 0; i < people.size(); ++i) {
      who += (i == 0 ? "" : (i + 1 == people.size() ? " e " : ", ")) + people[i];
    }
    if (who.empty()) {who = "Ciao a tutti in " + spoken(place);}
    say(who + ": " + sentence);
    send_feedback(0.5, "speaking");
    stage_ = Stage::Speaking;
  }

  rclcpp::Client<Speak>::SharedPtr speak_cli_;
  rclcpp::Client<Speak>::SharedFuture future_;
  Stage stage_{Stage::Ask};
  rclcpp::Time started_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AnnounceAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "announce"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
