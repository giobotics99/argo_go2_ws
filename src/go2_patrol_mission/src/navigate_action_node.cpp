#include <memory>
#include <string>
#include <map>
#include <cmath>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

class NavigateAction : public plansys2::ActionExecutorClient
{
public:
  NavigateAction()
  : plansys2::ActionExecutorClient("navigate", 500ms)
  {
    this->declare_parameter<std::string>("waypoints_file", "");
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
      send_goal_options.feedback_callback =
        std::bind(&NavigateAction::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
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

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {
    // Optionally update progress based on distance remaining
    send_feedback(std::max(0.0f, std::min(1.0f, 1.0f - (float)feedback->distance_remaining / 10.0f)), "Moving");
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