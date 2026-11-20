#include <atomic>
#include <memory>
#include <string>
#include <map>
#include <cmath>

#include "ament_index_cpp/get_package_share_directory.hpp"
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
    // Poses: v1 passes waypoints_file (rooms only); v2 reads hubs and places from ontology.yaml.
    this->declare_parameter<std::string>("waypoints_file", "");
    this->declare_parameter<std::string>(
      "ontology_file",
      ament_index_cpp::get_package_share_directory("go2_patrol_mission") + "/config/ontology.yaml");
    // Il cane parla da fermo: prima di partire aspetto che argo_voice abbia finito.
    // voice_wait_min lascia arrivare le frasi di partenza; voice_wait_max evita di restare
    // bloccati se la voce non risponde.
    // Give up when Nav2 does not arrive in time (e.g. a closed door: Nav2 keeps trying recoveries
    // for minutes). Deadline = max(nav_timeout_min, nav_timeout_factor * edge_length / speed).
    this->declare_parameter<double>("nav_timeout_min", 90.0);
    this->declare_parameter<double>("nav_timeout_factor", 3.0);
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
    this->declare_parameter<bool>("check_standing", false);
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
    if (!load_poses()) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this,
      "navigate_to_pose");

    events_pub_ = this->create_publisher<std_msgs::msg::String>("/patrol/action_events", 10);
    events_pub_->on_activate();

    progress_ = 0.0;
    goal_sent_ = false;
    finished_ = false;
    goal_handle_.reset();
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
  // Fill waypoints_ (name -> x, y, theta). waypoints_file wins (v1 compatibility), otherwise
  // the hubs and the place poses of ontology.yaml (v2: navigate goes to places AND hubs).
  bool load_poses()
  {
    const std::string waypoints_file = this->get_parameter("waypoints_file").as_string();
    const std::string ontology_file = this->get_parameter("ontology_file").as_string();
    waypoints_.clear();
    try {
      if (!waypoints_file.empty()) {
        YAML::Node config = YAML::LoadFile(waypoints_file);
        for (const auto & it : config["waypoints"]) {
          const YAML::Node & wp = it.second;
          waypoints_[it.first.as<std::string>()] =
          {wp["x"].as<double>(), wp["y"].as<double>(), wp["theta"].as<double>()};
        }
      } else {
        YAML::Node onto = YAML::LoadFile(ontology_file);
        for (const auto & it : onto["hubs"]) {
          const YAML::Node & h = it.second;
          waypoints_[it.first.as<std::string>()] =
          {h["x"].as<double>(), h["y"].as<double>(), h["theta"].as<double>()};
        }
        for (const auto & it : onto["places"]) {
          const YAML::Node & p = it.second["pose"];
          waypoints_[it.first.as<std::string>()] =
          {p["x"].as<double>(), p["y"].as<double>(), p["theta"].as<double>()};
        }
        edge_length_.clear();
        for (const auto & e : onto["edges"]) {
          if (e.size() >= 3) {
            const auto a = e[0].as<std::string>(), b = e[1].as<std::string>();
            edge_length_[a + "|" + b] = edge_length_[b + "|" + a] = e[2].as<double>();
          }
        }
        if (onto["robot"] && onto["robot"]["speed"]) {
          speed_ = onto["robot"]["speed"].as<double>();
        }
      }
    } catch (const YAML::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Errore nel leggere le pose: %s", e.what());
      return false;
    }
    if (waypoints_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Nessuna posa caricata (waypoints_file e ontology_file)");
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(), "%zu pose caricate da %s", waypoints_.size(),
      (waypoints_file.empty() ? ontology_file : waypoints_file).c_str());
    return true;
  }

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

      // Deadline from the edge length (ontology) or the straight line from the current target
      double length = 0.0;
      auto edge = edge_length_.find(from_room + "|" + target_room);
      if (edge != edge_length_.end()) {
        length = edge->second;
      } else if (waypoints_.count(from_room)) {
        length = std::hypot(wp[0] - waypoints_[from_room][0], wp[1] - waypoints_[from_room][1]);
      }
      const double timeout = std::max(
        this->get_parameter("nav_timeout_min").as_double(),
        this->get_parameter("nav_timeout_factor").as_double() * length / std::max(speed_, 0.05));
      deadline_ = this->now() + rclcpp::Duration::from_seconds(timeout);

      RCLCPP_INFO(
        this->get_logger(), "Sending goal to Nav2: %s (%.1f m, limite %.0f s)", target_room.c_str(),
        length, timeout);
      nav2_client_->async_send_goal(goal_msg, send_goal_options);
      
      // Publish start event
      auto msg = std::make_shared<std_msgs::msg::String>();
      msg->data = "start:navigate:" + args[0] + ":" + from_room + ":" + target_room;
      events_pub_->publish(*msg);

      goal_sent_ = true;
    } else if (!finished_ && this->now() > deadline_) {
      // Nav2 is still trying (closed door, person in the way...): give up and let the mission
      // decide (the BT asks Gemma for a recovery strategy)
      auto args = get_arguments();
      RCLCPP_WARN(
        this->get_logger(), "Non arrivo a %s nel tempo previsto: annullo il goal Nav2",
        args.size() > 2 ? args[2].c_str() : "?");
      finished_ = true;
      if (goal_handle_) {
        nav2_client_->async_cancel_goal(goal_handle_);
      }
      auto msg = std::make_shared<std_msgs::msg::String>();
      msg->data = "failure:navigate:" + args[0] + ":" + args[1] + ":" + args[2];
      events_pub_->publish(*msg);
      finish(false, 0.0, "Navigation timeout");
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
    // Off by default: in the stall state the Go2 reports a wrong body height (0.09 m while
    // standing, 20/11), so the check kept the robot blocked. A navigate goal means "move" anyway.
    if (this->get_parameter("check_standing").as_bool()) {
      if (last_state_ns_ == 0 || age > std::chrono::nanoseconds(2s).count()) {
        RCLCPP_WARN(this->get_logger(), "Nessuno stato recente da /sportmodestate: non sblocco il robot");
        return true;
      }
      if (body_height_ < min_height) {
        RCLCPP_WARN(
          this->get_logger(), "Il robot è a terra (altezza %.2f m): non lo sblocco", body_height_.load());
        return true;
      }
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
      goal_handle_ = goal_handle;
      if (finished_) {
        return;
      }
      if (!goal_handle) {
        finished_ = true;
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
    goal_handle_.reset();
    if (finished_) {   // already finished by the timeout
      goal_sent_ = false;
      return;
    }
    finished_ = true;
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
  std::map<std::string, double> edge_length_;   // "a|b" -> metres, from ontology.yaml
  double speed_{0.5};
  rclcpp::Time deadline_{0, 0, RCL_ROS_TIME};
  std::atomic<bool> finished_{false};
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
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