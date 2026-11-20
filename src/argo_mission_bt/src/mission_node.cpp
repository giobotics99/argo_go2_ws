// argo_mission_bt/mission_node — the executive of Argo v2 (README_instruction.md, Phase 5).
//
// Loads ontology.yaml into the PlanSys2 knowledge base, offers /argo/command (GemmaCommand.srv, so
// `argo_cmd` keeps working) and ticks behavior_trees/mission.xml: command -> Gemma (G1) -> goals ->
// plan -> execution; on failure Gemma (G3) picks a recovery strategy; R1 (battery) and R2 (sensors)
// interrupt any mission. Groot monitoring (ZMQ, ports 1666/1667) when the "groot" parameter is true.

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "argo_mission_bt/context.hpp"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "go2_interfaces/srv/gemma_command.hpp"
#include "nlohmann/json.hpp"
#include "unitree_go/msg/low_state.hpp"

using namespace std::chrono_literals;

namespace argo
{

// Root of the tree: like ReactiveSequence (every tick restarts from the first child), but when a
// child is RUNNING all the following children are halted explicitly. So when R1/R2 take over in
// the first child, the running mission (ExecutePlan) below is halted and its plan cancelled.
class SafeSequence : public BT::ControlNode
{
public:
  SafeSequence(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);
    for (size_t i = 0; i < children_nodes_.size(); ++i) {
      const auto status = children_nodes_[i]->executeTick();
      if (status == BT::NodeStatus::RUNNING) {
        for (size_t j = i + 1; j < children_nodes_.size(); ++j) {haltChild(j);}
        return BT::NodeStatus::RUNNING;
      }
      if (status == BT::NodeStatus::FAILURE) {
        haltChildren();
        return BT::NodeStatus::FAILURE;
      }
    }
    haltChildren();
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace argo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("argo_mission");
  const std::string share = ament_index_cpp::get_package_share_directory("argo_mission_bt");
  const std::string ontology_file = node->declare_parameter<std::string>(
    "ontology_file",
    ament_index_cpp::get_package_share_directory("go2_patrol_mission") + "/config/ontology.yaml");
  const std::string bt_xml = node->declare_parameter<std::string>("bt_xml", share + "/behavior_trees/mission.xml");
  const bool groot = node->declare_parameter<bool>("groot", true);
  const double tick_hz = node->declare_parameter<double>("tick_hz", 10.0);
  const std::string lowstate_topic = node->declare_parameter<std::string>("lowstate_topic", "/lowstate");

  auto ctx = std::make_shared<argo::Context>(node, ontology_file);

  // ---- inputs cached in the context
  auto amcl_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", rclcpp::QoS(1).transient_local().reliable(),
    [ctx](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
      ctx->robot_xy = std::array<double, 2>{m->pose.pose.position.x, m->pose.pose.position.y};
    });
  auto lowstate_sub = node->create_subscription<unitree_go::msg::LowState>(
    lowstate_topic, rclcpp::SensorDataQoS(),
    [ctx](unitree_go::msg::LowState::SharedPtr m) {ctx->soc = m->bms_state.soc;});
  auto health_sub = node->create_subscription<std_msgs::msg::String>(
    "/argo/health", 10, [ctx](std_msgs::msg::String::SharedPtr m) {
      try {
        auto js = nlohmann::json::parse(m->data);
        std::string faults;
        for (const auto & f : js.value("faults", nlohmann::json::array())) {
          faults += (faults.empty() ? "" : ", ") + f.value("source", std::string("?"));
        }
        std::lock_guard<std::mutex> lock(ctx->mutex);
        ctx->health_status = js.value("status", std::string("OK"));
        ctx->health_faults = faults;
        ctx->health_stamp = ctx->node->now();
      } catch (const std::exception &) {}
    });
  auto voice_sub = node->create_subscription<std_msgs::msg::Bool>(
    "/argo/voice_busy", rclcpp::QoS(1).transient_local().reliable(),
    [ctx](std_msgs::msg::Bool::SharedPtr m) {
      ctx->voice_busy = m->data;
      if (m->data) {ctx->voice_seen_busy = true;}
    });
  auto findings_sub = node->create_subscription<std_msgs::msg::String>(
    "/argo/findings", 10, [ctx](std_msgs::msg::String::SharedPtr m) {
      std::lock_guard<std::mutex> lock(ctx->mutex);
      ctx->findings = m->data;
    });

  // ---- entry point for commands (same service type as v1, so `argo_cmd` keeps working)
  auto command_srv = node->create_service<go2_interfaces::srv::GemmaCommand>(
    "/argo/command",
    [ctx](const std::shared_ptr<go2_interfaces::srv::GemmaCommand::Request> req,
    std::shared_ptr<go2_interfaces::srv::GemmaCommand::Response> res) {
      std::lock_guard<std::mutex> lock(ctx->mutex);
      ctx->commands.push_back(req->command);
      res->success = true;
      res->message = "Comando ricevuto: lo interpreto e ti rispondo a voce.";
      RCLCPP_INFO(ctx->node->get_logger(), "[/argo/command] '%s'", req->command.c_str());
    });

  // ---- wait for PlanSys2 and load the ontology into the knowledge base
  RCLCPP_INFO(node->get_logger(), "Aspetto PlanSys2 e carico l'ontologia nel KB...");
  while (rclcpp::ok() && !ctx->load_kb()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(2s);
  }
  RCLCPP_INFO(node->get_logger(), "KB pronto: %zu luoghi, %zu archi", ctx->onto.nodes().size(), ctx->onto.edges.size());

  // ---- behavior tree
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<argo::SafeSequence>("SafeSequence");
  argo::register_conditions(factory);
  argo::register_plansys2_nodes(factory);
  argo::register_gemma_nodes(factory);
  argo::register_interaction_nodes(factory);
  auto blackboard = BT::Blackboard::create();
  blackboard->set<argo::ContextPtr>("ctx", ctx);
  auto tree = factory.createTreeFromFile(bt_xml, blackboard);
  std::unique_ptr<BT::PublisherZMQ> groot_pub;
  if (groot) {
    try {
      groot_pub = std::make_unique<BT::PublisherZMQ>(tree);
      RCLCPP_INFO(node->get_logger(), "Groot: monitor su tcp://<robot>:1666");
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(), "Groot non disponibile: %s", e.what());
    }
  }
  RCLCPP_INFO(node->get_logger(), "Argo pronto: aspetto comandi su /argo/command (argo_cmd \"...\")");

  rclcpp::Rate rate(tick_hz);
  while (rclcpp::ok()) {
    tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
