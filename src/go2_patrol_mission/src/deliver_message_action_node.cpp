// Executor of the PDDL action "deliver_message ?r ?l" (Argo v2, README Phase 3).
//
// The text for each room comes from the latched topic /argo/messages (std_msgs/String, JSON written
// by the mission BT, or by tools/set_message.py in tests):
//     {"vito_room": "la riunione è alle 15"}
//     {"vito_room": {"name": "Vito", "text": "la riunione è alle 15"}}
// The robot says "<name>, ho un messaggio per te: <text>" on /argo/say (argo_voice_node speaks it)
// and the action finishes when /argo/voice_busy goes back to false. The message words are never
// rewritten. Without a name, the first occupant of the room in ontology.yaml is used.
// Fails if there is no message for the room, or if the voice does not start within 8 s.

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nlohmann/json.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

class DeliverMessageAction : public plansys2::ActionExecutorClient
{
public:
  DeliverMessageAction()
  : plansys2::ActionExecutorClient("deliver_message_action_node", 200ms)
  {
    declare_parameter<std::string>(
      "ontology_file",
      ament_index_cpp::get_package_share_directory("go2_patrol_mission") + "/config/ontology.yaml");
    declare_parameter<double>("voice_start_timeout", 8.0);   // s to see /argo/voice_busy go true
    declare_parameter<double>("voice_max_time", 90.0);       // s at most for the whole sentence
    load_occupants(get_parameter("ontology_file").as_string());

    messages_sub_ = create_subscription<std_msgs::msg::String>(
      "/argo/messages", rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {on_messages(msg->data);});
    voice_busy_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/argo/voice_busy", rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        voice_busy_ = msg->data;
        if (msg->data) {
          voice_seen_busy_ = true;
        }
      });
    say_pub_ = create_publisher<std_msgs::msg::String>("/argo/say", 10);
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override
  {
    stage_ = Stage::Say;
    say_pub_->on_activate();   // lifecycle publisher: drops messages until activated
    return ActionExecutorClient::on_activate(state);
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    say_pub_->on_deactivate();
    return ActionExecutorClient::on_deactivate(state);
  }

private:
  enum class Stage {Say, WaitStart, WaitEnd, Done};

  void load_occupants(const std::string & path)
  {
    try {
      YAML::Node onto = YAML::LoadFile(path);
      for (const auto & it : onto["places"]) {
        std::vector<std::string> names;
        if (it.second["occupant"]) {
          for (const auto & n : it.second["occupant"]) {
            names.push_back(n.as<std::string>());
          }
        }
        occupants_[it.first.as<std::string>()] = names;
      }
      // A door (door_of: <office>) is where the office's people are addressed
      for (const auto & it : onto["places"]) {
        if (it.second["door_of"]) {
          occupants_[it.first.as<std::string>()] = occupants_[it.second["door_of"].as<std::string>()];
        }
      }
    } catch (const YAML::Exception & e) {
      RCLCPP_ERROR(get_logger(), "Errore nel leggere %s: %s", path.c_str(), e.what());
    }
  }

  void on_messages(const std::string & data)
  {
    try {
      auto js = nlohmann::json::parse(data);
      std::lock_guard<std::mutex> lock(mutex_);
      messages_.clear();
      for (auto & [room, value] : js.items()) {
        if (value.is_string()) {
          messages_[room] = {"", value.get<std::string>()};
        } else if (value.is_object()) {
          messages_[room] = {value.value("name", ""), value.value("text", "")};
        }
      }
      RCLCPP_INFO(get_logger(), "Messaggi ricevuti per %zu stanze", messages_.size());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "/argo/messages non è un JSON valido: %s", e.what());
    }
  }

  void do_work() override
  {
    const double elapsed = (now() - stage_start_).seconds();
    switch (stage_) {
      case Stage::Say: {
          const auto & args = get_arguments();   // (robot, room)
          const std::string room = args.size() > 1 ? args[1] : "";
          std::string name, text;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = messages_.find(room);
            if (it != messages_.end()) {
              name = it->second.first;
              text = it->second.second;
            }
          }
          if (text.empty()) {
            RCLCPP_ERROR(get_logger(), "Nessun messaggio per %s su /argo/messages", room.c_str());
            stage_ = Stage::Done;
            finish(false, 0.0, "nessun messaggio per " + room);
            return;
          }
          if (name.empty() && !occupants_[room].empty()) {
            name = occupants_[room].front();
          }
          std_msgs::msg::String say;
          // A place without occupants (e.g. openspace): the message is for everybody there
          say.data = name.empty() ? "Ho un messaggio per tutti: " + text :
            name + ", ho un messaggio per te: " + text;
          voice_seen_busy_ = false;
          say_pub_->publish(say);
          RCLCPP_INFO(get_logger(), "Dico a %s: \"%s\"", room.c_str(), say.data.c_str());
          send_feedback(0.2, "speaking");
          stage_ = Stage::WaitStart;
          stage_start_ = now();
          break;
        }
      case Stage::WaitStart:
        if (voice_seen_busy_) {
          stage_ = Stage::WaitEnd;
        } else if (elapsed > get_parameter("voice_start_timeout").as_double()) {
          RCLCPP_ERROR(get_logger(), "La voce non è partita: argo_voice_node è acceso?");
          stage_ = Stage::Done;
          finish(false, 0.0, "voce non attiva");
        }
        break;
      case Stage::WaitEnd:
        if (!voice_busy_) {
          RCLCPP_INFO(get_logger(), "Messaggio consegnato");
          stage_ = Stage::Done;
          finish(true, 1.0, "messaggio consegnato");
        } else if (elapsed > get_parameter("voice_max_time").as_double()) {
          RCLCPP_WARN(get_logger(), "La voce non ha finito in tempo: considero il messaggio consegnato");
          stage_ = Stage::Done;
          finish(true, 1.0, "messaggio consegnato (timeout voce)");
        } else {
          send_feedback(0.6, "speaking");
        }
        break;
      case Stage::Done:
        break;   // the timer may tick once more before PlanSys2 deactivates the node
    }
  }

  std::map<std::string, std::vector<std::string>> occupants_;
  std::map<std::string, std::pair<std::string, std::string>> messages_;   // room -> (name, text)
  std::mutex mutex_;
  std::atomic<bool> voice_busy_{false};
  std::atomic<bool> voice_seen_busy_{false};
  Stage stage_{Stage::Say};
  rclcpp::Time stage_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr messages_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr voice_busy_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr say_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeliverMessageAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "deliver_message"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
