#include <atomic>
#include <memory>
#include <string>
#include <map>
#include <cmath>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "go2_interfaces/srv/mode.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

class NavigateAction : public plansys2::ActionExecutorClient
{
public:
  NavigateAction()
  : plansys2::ActionExecutorClient("navigate", 500ms)
  {
    this->declare_parameter<std::string>("waypoints_file", "");
    // Il cane parla da fermo: prima di partire aspetto che argo_voice abbia finito.
    // voice_wait_min lascia arrivare le frasi di partenza; voice_wait_max evita di restare
    // bloccati se la voce non risponde.
    this->declare_parameter<double>("voice_wait_min", 1.5);
    this->declare_parameter<double>("voice_wait_max", 45.0);
    voice_busy_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/argo/voice_busy", rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {voice_busy_ = msg->data;});

    // Dopo un'ispezione (o un uso del telecomando) il Go2 resta fermo in mode 0 e ignora i
    // comandi Move finché non lo si rimette in BalanceStand (come il tasto Start del telecomando).
    // Lo faccio solo se il robot è in piedi: un robot messo a cuccia di proposito non si rialza.
    this->declare_parameter<bool>("unlock_before_move", true);
    this->declare_parameter<double>("min_standing_height", 0.2);
    mode_client_ = this->create_client<go2_interfaces::srv::Mode>("/mode");
    sport_state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "/sportmodestate", rclcpp::SensorDataQoS(),
      [this](unitree_go::msg::SportModeState::SharedPtr msg) {
        body_height_ = msg->body_height;
        last_state_ns_ = std::chrono::steady_clock::now().time_since_epoch().count();
      });
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    std::string waypoints_file = this->get_parameter("waypoints_file").as_string();
    if (waypoints_file.empty()) {
      RCLCPP_ERROR(this->get_logger(), "waypoints_file parameter is empty!");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    try {
      YAML::Node config = YAML::LoadFile(waypoints_file);
      if (config["waypoints"]) {
        for (YAML::const_iterator it = config["waypoints"].begin(); it != config["waypoints"].end(); ++it) {
          std::string name = it->first.as<std::string>();
          YAML::Node wp = it->second;
          std::vector<double> coords = {wp["x"].as<double>(), wp["y"].as<double>(), wp["theta"].as<double>()};
          waypoints_[name] = coords;
          RCLCPP_INFO(this->get_logger(), "Loaded waypoint: %s", name.c_str());
        }
      }
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error parsing YAML file: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this,
      "navigate_to_pose");

    events_pub_ = this->create_publisher<std_msgs::msg::String>("/patrol/action_events", 10);
    events_pub_->on_activate();

    progress_ = 0.0;
    goal_sent_ = false;
    wait_start_ = this->now();
    voice_wait_logged_ = false;
    unlock_state_ = UNLOCK_TODO;

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    events_pub_->on_deactivate();
    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void do_work() override
  {
    if (!goal_sent_) {
      // Prima parlo, poi parto (sintesi vocale + movimento insieme saturano la CPU)
      const double waited = (this->now() - wait_start_).seconds();
      if (waited < this->get_parameter("voice_wait_min").as_double()) {
        return;
      }
      if (voice_busy_) {
        if (waited < this->get_parameter("voice_wait_max").as_double()) {
          if (!voice_wait_logged_) {
            RCLCPP_INFO(this->get_logger(), "Aspetto che il cane finisca di parlare prima di partire");
            voice_wait_logged_ = true;
          }
          return;
        }
        RCLCPP_WARN(this->get_logger(), "La voce non ha finito dopo %.0f s: parto comunque", waited);
      }

      if (!unlock_robot()) {
        return;  // sblocco in corso: riprovo al prossimo do_work
      }

      // Get arguments from PlanSys2
      // e.g. navigate robot from to
      auto args = get_arguments();
      if (args.size() < 3) {
        RCLCPP_ERROR(this->get_logger(), "Not enough arguments for navigate");
        finish(false, 0.0, "Not enough arguments");
        return;
      }
      
      std::string target_room = args[2];
      std::string from_room = args[1];
      if (waypoints_.find(target_room) == waypoints_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Waypoint %s not found in config", target_room.c_str());
        finish(false, 0.0, "Waypoint not found");
        return;
      }

      if (!nav2_client_->wait_for_action_server(5s)) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        finish(false, 0.0, "Nav2 action server not available");
        return;
      }

      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = this->now();
      
      std::vector<double> wp = waypoints_[target_room];
      goal_msg.pose.pose.position.x = wp[0];
      goal_msg.pose.pose.position.y = wp[1];
      goal_msg.pose.pose.position.z = 0.0;
      
      // Convert theta to quaternion
      double theta = wp[2];
      goal_msg.pose.pose.orientation.x = 0.0;
      goal_msg.pose.pose.orientation.y = 0.0;
      goal_msg.pose.pose.orientation.z = std::sin(theta / 2.0);
      goal_msg.pose.pose.orientation.w = std::cos(theta / 2.0);

      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.goal_response_callback =
        std::bind(&NavigateAction::goal_response_callback, this, std::placeholders::_1);
      // Nessun feedback_callback: con Nav2 Humble (nel container) il feedback di NavigateToPose
      // ha un campo in più (estimated_time_remaining) e distance_remaining arriverebbe sballato
      // a questo nodo Foxy. Goal e risultato sono identici tra Foxy e Humble.
      send_goal_options.result_callback =
        std::bind(&NavigateAction::result_callback, this, std::placeholders::_1);

      RCLCPP_INFO(this->get_logger(), "Sending goal to Nav2: %s", target_room.c_str());
      nav2_client_->async_send_goal(goal_msg, send_goal_options);
      
      // Publish start event
      auto msg = std::make_shared<std_msgs::msg::String>();
      msg->data = "start:navigate:" + args[0] + ":" + from_room + ":" + target_room;
      events_pub_->publish(*msg);

      goal_sent_ = true;
    }
  }

  // true = si può mandare il goal; false = aspetto che BalanceStand abbia effetto
  bool unlock_robot()
  {
    if (unlock_state_ == UNLOCK_DONE || !this->get_parameter("unlock_before_move").as_bool()) {
      return true;
    }
    if (unlock_state_ == UNLOCK_SENT) {
      if ((this->now() - unlock_time_).seconds() < 1.0) {
        return false;
      }
      unlock_state_ = UNLOCK_DONE;
      return true;
    }

    unlock_state_ = UNLOCK_DONE;  // qualsiasi esito sotto: un solo tentativo per navigazione
    const auto age = std::chrono::steady_clock::now().time_since_epoch().count() - last_state_ns_;
    const double min_height = this->get_parameter("min_standing_height").as_double();
    if (last_state_ns_ == 0 || age > std::chrono::nanoseconds(2s).count()) {
      RCLCPP_WARN(this->get_logger(), "Nessuno stato recente da /sportmodestate: non sblocco il robot");
      return true;
    }
    if (body_height_ < min_height) {
      RCLCPP_WARN(
        this->get_logger(), "Il robot è a terra (altezza %.2f m): non lo sblocco", body_height_.load());
      return true;
    }
    if (!mode_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Servizio /mode non disponibile: non sblocco il robot");
      return true;
    }
    auto req = std::make_shared<go2_interfaces::srv::Mode::Request>();
    req->mode = "balance_stand";
    mode_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "Sblocco il robot (balance_stand) prima di partire");
    unlock_time_ = this->now();
    unlock_state_ = UNLOCK_SENT;
    return false;
  }

  void goal_response_callback(
      std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future)
    {
      auto goal_handle = future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        
        auto msg = std::make_shared<std_msgs::msg::String>();
        auto args = get_arguments();
        msg->data = "failure:navigate:" + args[0] + ":" + args[1] + ":" + args[2];
        events_pub_->publish(*msg);

        finish(false, 0.0, "Goal rejected by Nav2");
        goal_sent_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    }

  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
  {
    auto args = get_arguments();
    auto msg = std::make_shared<std_msgs::msg::String>();

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Nav2 Goal Succeeded!");
        msg->data = "success:navigate:" + args[0] + ":" + args[1] + ":" + args[2];
        events_pub_->publish(*msg);
        finish(true, 1.0, "Navigation completed");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Nav2 Goal was aborted");
        msg->data = "failure:navigate:" + args[0] + ":" + args[1] + ":" + args[2];
        events_pub_->publish(*msg);
        finish(false, 0.0, "Navigation aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Nav2 Goal was canceled");
        msg->data = "failure:navigate:" + args[0] + ":" + args[1] + ":" + args[2];
        events_pub_->publish(*msg);
        finish(false, 0.0, "Navigation canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        msg->data = "failure:navigate:" + args[0] + ":" + args[1] + ":" + args[2];
        events_pub_->publish(*msg);
        finish(false, 0.0, "Navigation unknown error");
        break;
    }
    goal_sent_ = false;
  }

  std::map<std::string, std::vector<double>> waypoints_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr events_pub_;
  float progress_;
  bool goal_sent_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr voice_busy_sub_;
  std::atomic<bool> voice_busy_{false};
  rclcpp::Time wait_start_;
  bool voice_wait_logged_{false};

  enum UnlockState {UNLOCK_TODO, UNLOCK_SENT, UNLOCK_DONE};
  UnlockState unlock_state_{UNLOCK_TODO};
  rclcpp::Time unlock_time_;
  rclcpp::Client<go2_interfaces::srv::Mode>::SharedPtr mode_client_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
  std::atomic<float> body_height_{0.0f};
  std::atomic<int64_t> last_state_ns_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "navigate"));
  
  // 1. Configura il nodo
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  
  // 2. AGGIUNGI QUESTA RIGA per attivare publisher e client lifecycle!
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}