// Base class for the executors that speak (Argo v2, README Phase 3): announce, chat, ...
// - say(text): publishes on /argo/say (argo_voice_node speaks it);
// - voice_started() / voice_finished(): follow /argo/voice_busy (latched);
// The /argo/say publisher is a lifecycle publisher: it is activated with the action, otherwise it
// silently drops the messages.

#ifndef GO2_PATROL_MISSION__SPEAKING_ACTION_HPP_
#define GO2_PATROL_MISSION__SPEAKING_ACTION_HPP_

#include <atomic>
#include <map>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "yaml-cpp/yaml.h"

namespace go2_patrol_mission
{

class SpeakingAction : public plansys2::ActionExecutorClient
{
public:
  SpeakingAction(const std::string & node_name, const std::chrono::nanoseconds & rate)
  : plansys2::ActionExecutorClient(node_name, rate)
  {
    declare_parameter<std::string>(
      "ontology_file",
      ament_index_cpp::get_package_share_directory("go2_patrol_mission") + "/config/ontology.yaml");
    declare_parameter<double>("voice_start_timeout", 8.0);   // s to see /argo/voice_busy go true
    say_pub_ = create_publisher<std_msgs::msg::String>("/argo/say", 10);
    voice_busy_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/argo/voice_busy", rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        voice_busy_ = msg->data;
        if (msg->data) {voice_seen_busy_ = true;}
      });
    load_ontology(get_parameter("ontology_file").as_string());
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override
  {
    say_pub_->on_activate();
    return ActionExecutorClient::on_activate(state);
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    say_pub_->on_deactivate();
    return ActionExecutorClient::on_deactivate(state);
  }

protected:
  void say(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    voice_seen_busy_ = false;
    said_at_ = now();
    say_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Dico: \"%s\"", text.c_str());
  }

  // true once the voice has started; false while waiting; timed_out set if it never starts
  bool voice_started(bool & timed_out)
  {
    timed_out = !voice_seen_busy_ &&
      (now() - said_at_).seconds() > get_parameter("voice_start_timeout").as_double();
    return voice_seen_busy_;
  }

  bool voice_finished() {return voice_seen_busy_ && !voice_busy_;}

  // People of a place; a door (door_of: office) gives the office's people
  std::vector<std::string> occupants(const std::string & place)
  {
    auto it = occupants_.find(place);
    return it == occupants_.end() ? std::vector<std::string>{} : it->second;
  }

  // Italian name of a place (first synonym), for sentences
  std::string spoken(const std::string & place)
  {
    auto it = spoken_.find(place);
    return it == spoken_.end() ? place : it->second;
  }

  double chat_time_{180.0};

private:
  void load_ontology(const std::string & path)
  {
    try {
      YAML::Node o = YAML::LoadFile(path);
      if (o["chat_time"]) {chat_time_ = o["chat_time"].as<double>();}
      for (const auto & it : o["places"]) {
        const auto name = it.first.as<std::string>();
        if (it.second["occupant"]) {
          for (const auto & n : it.second["occupant"]) {occupants_[name].push_back(n.as<std::string>());}
        }
        if (it.second["synonyms"] && it.second["synonyms"].size() > 0) {
          spoken_[name] = it.second["synonyms"][0].as<std::string>();
        }
      }
      for (const auto & it : o["places"]) {
        if (it.second["door_of"]) {
          const auto office = it.second["door_of"].as<std::string>();
          occupants_[it.first.as<std::string>()] = occupants_[office];
          if (spoken_.count(office)) {spoken_[it.first.as<std::string>()] = spoken_[office];}
        }
      }
    } catch (const YAML::Exception & e) {
      RCLCPP_ERROR(get_logger(), "Errore nel leggere %s: %s", path.c_str(), e.what());
    }
  }

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr say_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr voice_busy_sub_;
  std::atomic<bool> voice_busy_{false}, voice_seen_busy_{false};
  rclcpp::Time said_at_{0, 0, RCL_ROS_TIME};
  std::map<std::string, std::vector<std::string>> occupants_;
  std::map<std::string, std::string> spoken_;
};

}  // namespace go2_patrol_mission

#endif  // GO2_PATROL_MISSION__SPEAKING_ACTION_HPP_
