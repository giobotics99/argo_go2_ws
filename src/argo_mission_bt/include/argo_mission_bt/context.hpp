// Shared context of the Argo mission BT (README_instruction.md, Phase 5).
// One instance lives on the blackboard ("ctx"): ontology, PlanSys2 clients, cached sensor data,
// publishers/clients towards voice and Gemma, and the state of the current mission.

#ifndef ARGO_MISSION_BT__CONTEXT_HPP_
#define ARGO_MISSION_BT__CONTEXT_HPP_

#include <array>
#include <atomic>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "go2_interfaces/srv/appraise_findings.hpp"
#include "go2_interfaces/srv/assess_failure.hpp"
#include "go2_interfaces/srv/formulate_goals.hpp"
#include "go2_interfaces/srv/mode.hpp"
#include "go2_interfaces/srv/speak.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace argo
{

struct Place
{
  std::string cls;
  double x{0}, y{0};
  bool camera_allowed{false}, needs_permission{false}, dock{false}, social_spot{false}, works_here{false};
  std::vector<std::string> occupants;
  std::string entrance, door_of;
  int viewpoints{0};
};

// ontology.yaml (the single source of truth for places, hubs, edges and robot parameters)
struct Ontology
{
  std::string home{"lab_robotics"}, robot{"go2"};
  double speed{0.5}, drain{0.03}, reserve{15.0}, chat_time{180.0};
  std::map<std::string, Place> places;
  std::map<std::string, std::array<double, 2>> hubs;
  std::vector<std::tuple<std::string, std::string, double>> edges;

  bool load(const std::string & path, std::string & error);
  std::vector<std::string> nodes() const;
  std::array<double, 2> xy(const std::string & node) const;
  std::string delivery_node(const std::string & place) const;   // door of an office, or the place
  std::string social_spot() const;
};

class Context
{
public:
  Context(rclcpp::Node::SharedPtr node, const std::string & ontology_file);

  rclcpp::Node::SharedPtr node;
  Ontology onto;
  std::shared_ptr<plansys2::ProblemExpertClient> problem;
  std::shared_ptr<plansys2::DomainExpertClient> domain;
  std::shared_ptr<plansys2::PlannerClient> planner;
  std::shared_ptr<plansys2::ExecutorClient> executor;

  // ---- cached data (filled by subscriptions spun in the main loop)
  std::optional<std::array<double, 2>> robot_xy;   // AMCL pose on the map
  double soc{-1.0};                                 // battery %, -1 = unknown
  std::string health_status{"OK"}, health_faults;
  rclcpp::Time health_stamp{0, 0, RCL_ROS_TIME};
  std::atomic<bool> voice_busy{false}, voice_seen_busy{false};
  std::mutex mutex;
  std::deque<std::string> commands;                 // from /argo/command

  // ---- outputs
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr say_pub, messages_pub, report_pub;
  rclcpp::Client<go2_interfaces::srv::FormulateGoals>::SharedPtr formulate_cli;
  rclcpp::Client<go2_interfaces::srv::AssessFailure>::SharedPtr assess_cli;
  rclcpp::Client<go2_interfaces::srv::AppraiseFindings>::SharedPtr appraise_cli;
  rclcpp::Client<go2_interfaces::srv::Speak>::SharedPtr speak_cli;
  rclcpp::Client<go2_interfaces::srv::Mode>::SharedPtr mode_cli;

  // ---- current mission
  std::string mission{"none"};                      // coffee_call | message | inspect | home
  std::vector<std::string> targets;                 // place IDs as given by Gemma
  std::vector<std::string> dropped;                 // targets given up by DROP_UNREACHABLE
  bool ordered{false};
  std::string message_text;
  std::optional<plansys2_msgs::msg::Plan> plan;
  std::string failed_action;                        // full PDDL action that failed, "" = none
  std::vector<std::pair<std::string, std::string>> blocked;   // edges set path_free=false after a failed navigate
  int attempts{0};
  bool returned_home_early{false};
  std::string findings;                             // last /argo/findings (JSON), consumed by G4

  // ---- knowledge base helpers (plansys2 problem expert)
  bool load_kb();                                   // instances, rigid facts, initial flexible facts
  bool add(const std::string & predicate);
  bool remove(const std::string & predicate);
  bool set_function(const std::string & function, double value);
  std::vector<std::vector<std::string>> predicates(const std::string & name);   // args of each
  std::string robot_node();                         // from (robot_at go2 ?) in the KB
  std::string nearest_node() const;                 // from the AMCL pose
  std::set<std::string> reachable(const std::string & from);   // through path_free edges
  void clear_mission_facts();
  bool set_goal(const std::vector<std::string> & facts);   // (and ... (robot_at go2 home))
  bool goal_active();
  void publish_messages();

  void say(const std::string & text);
  std::string spoken(const std::string & place) const;     // first synonym, for sentences

private:
  std::map<std::string, std::vector<std::string>> synonyms_;
};

using ContextPtr = std::shared_ptr<Context>;

// Registration of the node families (one file each)
void register_conditions(BT::BehaviorTreeFactory & factory);
void register_plansys2_nodes(BT::BehaviorTreeFactory & factory);
void register_gemma_nodes(BT::BehaviorTreeFactory & factory);
void register_interaction_nodes(BT::BehaviorTreeFactory & factory);

// Every node gets the context from the blackboard
inline ContextPtr ctx_of(const BT::TreeNode & node)
{
  return node.config().blackboard->get<ContextPtr>("ctx");
}

}  // namespace argo

#endif  // ARGO_MISSION_BT__CONTEXT_HPP_
