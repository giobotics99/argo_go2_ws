// Executor of the PDDL action "advance ?p ?n" (Argo v2, README Phase 3).
// "advance" only moves the order chain of a mission forward ((done ?p) and (precedes ?p ?n) give
// (unlocked ?n)): nothing happens on the robot, so it finishes with success at once.

#include <memory>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class AdvanceAction : public plansys2::ActionExecutorClient
{
public:
  AdvanceAction()
  : plansys2::ActionExecutorClient("advance_action_node", 100ms)
  {
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override
  {
    done_ = false;
    return ActionExecutorClient::on_activate(state);
  }

private:
  void do_work() override
  {
    if (done_) {
      return;  // the timer may tick once more before PlanSys2 deactivates the node
    }
    done_ = true;
    const auto & args = get_arguments();
    RCLCPP_INFO(
      get_logger(), "advance: %s -> %s", args.size() > 0 ? args[0].c_str() : "?",
      args.size() > 1 ? args[1].c_str() : "?");
    finish(true, 1.0, "advanced");
  }

  bool done_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdvanceAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "advance"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
