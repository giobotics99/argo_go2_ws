#include <memory>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "go2_interfaces/srv/euler.hpp"

using namespace std::chrono_literals;

class InspectAction : public plansys2::ActionExecutorClient
{
public:
  InspectAction()
  : plansys2::ActionExecutorClient("inspect", 500ms)
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    euler_client_ = this->create_client<go2_interfaces::srv::Euler>("/euler");
    
    inspect_state_ = 0; // 0: init, 1: tilt down, 2: wait, 3: tilt back, 4: done
    start_time_ = this->now();

    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  void do_work() override
  {
    if (inspect_state_ == 0) {
      // Tilt down
      auto request = std::make_shared<go2_interfaces::srv::Euler::Request>();
      request->roll = 0.0;
      request->pitch = 0.3;
      request->yaw = 0.0;

      if (!euler_client_->wait_for_service(2s)) {
        RCLCPP_ERROR(this->get_logger(), "Euler service not available");
        finish(false, 0.0, "Service unavailable");
        return;
      }
      
      RCLCPP_INFO(this->get_logger(), "Starting inspection: Tilting body down...");
      euler_client_->async_send_request(request);
      
      start_time_ = this->now();
      inspect_state_ = 1;
      send_feedback(0.1, "Tilting down");
      
    } else if (inspect_state_ == 1) {
      if ((this->now() - start_time_).seconds() > 1.0) {
        inspect_state_ = 2;
        start_time_ = this->now();
      }
    } else if (inspect_state_ == 2) {
      // Waiting / Inspecting
      double elapsed = (this->now() - start_time_).seconds();
      send_feedback(0.2 + 0.6 * (elapsed / 3.0), "Inspecting");
      
      if (elapsed > 3.0) {
        inspect_state_ = 3;
      }
    } else if (inspect_state_ == 3) {
      // Tilt back up
      auto request = std::make_shared<go2_interfaces::srv::Euler::Request>();
      request->roll = 0.0;
      request->pitch = 0.0;
      request->yaw = 0.0;

      RCLCPP_INFO(this->get_logger(), "Finishing inspection: Restoring body position...");
      euler_client_->async_send_request(request);
      
      start_time_ = this->now();
      inspect_state_ = 4;
      send_feedback(0.9, "Restoring position");
    } else if (inspect_state_ == 4) {
      if ((this->now() - start_time_).seconds() > 1.0) {
        RCLCPP_INFO(this->get_logger(), "Inspection completed successfully");
        finish(true, 1.0, "Inspection completed");
        inspect_state_ = 0; // Reset for next time
      }
    }
  }

  rclcpp::Client<go2_interfaces::srv::Euler>::SharedPtr euler_client_;
  int inspect_state_;
  rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InspectAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "inspect"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
