// Fake PlanSys2 action executors (tutorial step 5).
// One ActionExecutorClient per name in the "actions" parameter: each one accepts any instance of
// its action, prints name and arguments, and finishes with success after "duration_s" seconds.
// Used to run whole plans (domain v2) without moving the robot.
//
//   ros2 run go2_patrol_mission fake_action_executors --ros-args -p duration_s:=1.0
//     -p actions:="['navigate','announce']"      (default: all 8 actions of patrol_domain_v2)

#include <memory>
#include <string>
#include <vector>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class FakeAction : public plansys2::ActionExecutorClient
{
public:
  FakeAction(const std::string & action, double duration_s, const std::vector<std::string> & fail_on)
  : plansys2::ActionExecutorClient("fake_" + action, 100ms), duration_s_(duration_s), fail_on_(fail_on)
  {
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override
  {
    started_ = false;
    return ActionExecutorClient::on_activate(state);
  }

private:
  void do_work() override
  {
    if (!started_) {
      started_ = true;
      start_ = now();
      std::string args;
      for (const auto & a : get_arguments()) {
        args += " " + a;
      }
      RCLCPP_INFO(get_logger(), "[FINTO] inizio (%s%s)", get_action_name().c_str(), args.c_str());
      full_ = get_action_name() + args;
    }
    const double elapsed = (now() - start_).seconds();
    if (elapsed < duration_s_) {
      send_feedback(static_cast<float>(elapsed / duration_s_), "fake running");
      return;
    }
    started_ = false;
    for (const auto & f : fail_on_) {   // offline tests: make chosen actions fail
      if (!f.empty() && full_.find(f) != std::string::npos) {
        RCLCPP_WARN(get_logger(), "[FINTO] fallisce apposta (%s)", full_.c_str());
        finish(false, 0.0, "fake failure");
        return;
      }
    }
    RCLCPP_INFO(get_logger(), "[FINTO] fine (%s)", get_action_name().c_str());
    finish(true, 1.0, "fake done");
  }

  double duration_s_;
  std::vector<std::string> fail_on_;
  std::string full_;
  bool started_{false};
  rclcpp::Time start_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Parameters are read from a small helper node (the executors are separate nodes)
  auto params = std::make_shared<rclcpp::Node>("fake_action_executors");
  const auto actions = params->declare_parameter<std::vector<std::string>>(
    "actions", {"navigate", "announce", "chat", "deliver_message", "inspect_room",
      "ask_permission", "recharge", "advance"});
  const double duration_s = params->declare_parameter<double>("duration_s", 1.0);
  // e.g. ["navigate go2 hub_west vito_door"]: those actions fail (to test the recovery)
  const auto fail_on = params->declare_parameter<std::vector<std::string>>("fail_on", {""});

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(params);
  std::vector<std::shared_ptr<FakeAction>> nodes;
  for (const auto & action : actions) {
    auto node = std::make_shared<FakeAction>(action, duration_s, fail_on);
    node->set_parameter(rclcpp::Parameter("action_name", action));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    executor.add_node(node->get_node_base_interface());
    nodes.push_back(node);
    RCLCPP_INFO(params->get_logger(), "Executor finto pronto: %s (%.1f s)", action.c_str(), duration_s);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
