// Executor of "chat ?r ?l" (M1, at the coffee area, README Phase 3).
// For chat_time seconds (ontology.yaml, same value as the PDDL duration; parameter chat_time to
// override), every line_period seconds: asks gemma_server for one line (Chat: what YOLO sees + the
// lines already said), says it, and waits for the voice. Never fails: it ends when the time is up.
// Without gemma_server it uses a fixed list of lines.

#include <algorithm>
#include <map>
#include <mutex>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "go2_interfaces/srv/chat.hpp"
#include "go2_patrol_mission/speaking_action.hpp"
#include "nlohmann/json.hpp"

using namespace std::chrono_literals;
using Chat = go2_interfaces::srv::Chat;

// Coffee jokes, used only when gemma_server is not running (same list as gemma_server.py)
static const std::vector<std::string> FALLBACK_LINES = {
  "Perché il caffè non va mai in palestra? Perché è già abbastanza forte!",
  "Cosa dice una tazzina all'altra? Senza di te la mia vita sarebbe amara!",
  "Qual è il caffè preferito dei programmatori? Il Java, ovviamente!",
  "Perché il cucchiaino è sempre stressato? Perché tutti lo fanno girare!",
  "Il barista mi ha chiesto: lo vuoi corto o lungo? Io ho risposto: a quattro zampe!",
  "Sapete perché la moka è sempre puntuale? Perché sale quando è ora!",
  "Io e il caffè abbiamo una cosa in comune: senza la giusta carica non partiamo!",
  "Perché il caffè non litiga mai? Perché preferisce restare ristretto!",
};

class ChatAction : public go2_patrol_mission::SpeakingAction
{
public:
  ChatAction()
  : SpeakingAction("chat_action_node", 200ms)
  {
    declare_parameter<double>("chat_time", -1.0);        // s; -1 = chat_time of ontology.yaml
    declare_parameter<double>("line_period", 25.0);      // s between two lines
    chat_cli_ = create_client<Chat>("/argo/gemma/chat");
    yolo_sub_ = create_subscription<std_msgs::msg::String>(
      "/yolo/detections", 10, [this](std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        detections_ = m->data;
      });
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override
  {
    stage_ = Stage::Idle;
    start_ = now();
    next_line_ = start_;
    history_.clear();
    return SpeakingAction::on_activate(state);
  }

private:
  enum class Stage {Idle, WaitLine, Speaking, Done};

  void do_work() override
  {
    if (stage_ == Stage::Done) {return;}
    const double duration = get_parameter("chat_time").as_double() > 0 ?
      get_parameter("chat_time").as_double() : chat_time_;
    const double elapsed = (now() - start_).seconds();
    send_feedback(static_cast<float>(std::min(1.0, elapsed / duration)), "chatting");

    switch (stage_) {
      case Stage::Idle:
        if (elapsed >= duration) {
          stage_ = Stage::Done;
          RCLCPP_INFO(get_logger(), "Fine della chiacchierata (%zu battute)", history_.size());
          finish(true, 1.0, "chiacchierata finita");
        } else if (now() >= next_line_) {
          ask_line();
        }
        break;
      case Stage::WaitLine:
        if (future_.wait_for(0s) == std::future_status::ready) {
          speak_line(future_.get()->line);
        } else if ((now() - asked_).seconds() > 15.0) {
          speak_line("");
        }
        break;
      case Stage::Speaking: {
          bool timed_out = false;
          if ((voice_started(timed_out) && voice_finished()) || timed_out) {
            stage_ = Stage::Idle;   // never fails: without a voice it just waits for the next line
          }
          break;
        }
      case Stage::Done:
        break;
    }
  }

  void ask_line()
  {
    asked_ = now();
    next_line_ = asked_ + rclcpp::Duration::from_seconds(get_parameter("line_period").as_double());
    if (!chat_cli_->service_is_ready()) {
      speak_line("");
      return;
    }
    auto req = std::make_shared<Chat::Request>();
    req->detections_json = counts_json();
    std::string history;
    for (const auto & h : history_) {history += h + "\n";}
    req->history = history;
    future_ = chat_cli_->async_send_request(req);
    stage_ = Stage::WaitLine;
  }

  void speak_line(std::string line)
  {
    if (line.empty()) {line = FALLBACK_LINES[history_.size() % FALLBACK_LINES.size()];}
    history_.push_back(line);
    say(line);
    stage_ = Stage::Speaking;
  }

  // "person,person,cup" (yolo_detector.py) -> {"person": 2, "cup": 1}
  std::string counts_json()
  {
    std::string data;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      data = detections_;
    }
    std::map<std::string, int> counts;
    std::stringstream ss(data);
    for (std::string label; std::getline(ss, label, ','); ) {
      label.erase(0, label.find_first_not_of(" "));
      label.erase(label.find_last_not_of(" ") + 1);
      if (!label.empty()) {counts[label]++;}
    }
    return nlohmann::json(counts).dump();
  }

  rclcpp::Client<Chat>::SharedPtr chat_cli_;
  rclcpp::Client<Chat>::SharedFuture future_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
  std::mutex mutex_;
  std::string detections_;
  std::vector<std::string> history_;
  Stage stage_{Stage::Idle};
  rclcpp::Time start_{0, 0, RCL_ROS_TIME}, next_line_{0, 0, RCL_ROS_TIME}, asked_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChatAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "chat"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
