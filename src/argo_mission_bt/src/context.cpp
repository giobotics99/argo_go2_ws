// Context of the mission BT: ontology loading and knowledge-base helpers.
// The KB facts are the same as tools/gen_problem.py (README: "the KB loader generates the same facts").

#include "argo_mission_bt/context.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "nlohmann/json.hpp"
#include "yaml-cpp/yaml.h"

namespace argo
{

static std::string num(double v)
{
  std::ostringstream os;
  os << v;
  return os.str();
}

bool Ontology::load(const std::string & path, std::string & error)
{
  try {
    YAML::Node o = YAML::LoadFile(path);
    home = o["home"].as<std::string>();
    robot = o["robot"]["id"].as<std::string>();
    speed = o["robot"]["speed"].as<double>();
    drain = o["robot"]["drain"].as<double>();
    reserve = o["robot"]["reserve"] ? o["robot"]["reserve"].as<double>() : 0.0;
    chat_time = o["chat_time"].as<double>();
    std::map<std::string, bool> camera;
    for (const auto & c : o["classes"]) {
      camera[c.first.as<std::string>()] = c.second["camera_allowed"].as<bool>(false);
    }
    for (const auto & h : o["hubs"]) {
      hubs[h.first.as<std::string>()] = {h.second["x"].as<double>(), h.second["y"].as<double>()};
    }
    for (const auto & it : o["places"]) {
      const auto & p = it.second;
      Place pl;
      pl.cls = p["class"].as<std::string>();
      pl.x = p["pose"]["x"].as<double>();
      pl.y = p["pose"]["y"].as<double>();
      pl.camera_allowed = camera[pl.cls];
      pl.needs_permission = p["needs_permission"].as<bool>(false);
      pl.dock = p["dock"].as<bool>(false);
      pl.social_spot = p["social_spot"].as<bool>(false);
      pl.works_here = p["works_here"].as<bool>(false);
      pl.entrance = p["entrance"].as<std::string>("");
      pl.door_of = p["door_of"].as<std::string>("");
      pl.viewpoints = p["viewpoints"] ? static_cast<int>(p["viewpoints"].size()) : 0;
      if (p["occupant"]) {
        for (const auto & n : p["occupant"]) {
          pl.occupants.push_back(n.as<std::string>());
        }
      }
      places[it.first.as<std::string>()] = pl;
    }
    for (const auto & e : o["edges"]) {
      edges.emplace_back(e[0].as<std::string>(), e[1].as<std::string>(), e[2].as<double>());
    }
  } catch (const std::exception & e) {
    error = e.what();
    return false;
  }
  return true;
}

std::vector<std::string> Ontology::nodes() const
{
  std::vector<std::string> out;
  for (const auto & h : hubs) {out.push_back(h.first);}
  for (const auto & p : places) {out.push_back(p.first);}
  return out;
}

std::array<double, 2> Ontology::xy(const std::string & n) const
{
  auto h = hubs.find(n);
  if (h != hubs.end()) {return h->second;}
  const auto & p = places.at(n);
  return {p.x, p.y};
}

// Offices are served from their door (privacy); the other rooms are entered through it
std::string Ontology::delivery_node(const std::string & place) const
{
  auto it = places.find(place);
  if (it == places.end() || it->second.cls != "Office") {return place;}
  for (const auto & p : places) {
    if (p.second.door_of == place) {return p.first;}
  }
  return place;
}

std::string Ontology::social_spot() const
{
  for (const auto & p : places) {
    if (p.second.social_spot) {return p.first;}
  }
  return "";
}

Context::Context(rclcpp::Node::SharedPtr n, const std::string & ontology_file)
: node(n)
{
  std::string error;
  if (!onto.load(ontology_file, error)) {
    throw std::runtime_error("ontology " + ontology_file + ": " + error);
  }
  YAML::Node o = YAML::LoadFile(ontology_file);
  for (const auto & it : o["places"]) {
    for (const auto & s : it.second["synonyms"]) {
      synonyms_[it.first.as<std::string>()].push_back(s.as<std::string>());
    }
  }
  problem = std::make_shared<plansys2::ProblemExpertClient>();
  domain = std::make_shared<plansys2::DomainExpertClient>();
  planner = std::make_shared<plansys2::PlannerClient>();
  executor = std::make_shared<plansys2::ExecutorClient>();

  auto latched = rclcpp::QoS(1).transient_local().reliable();
  say_pub = node->create_publisher<std_msgs::msg::String>("/argo/say", 10);
  messages_pub = node->create_publisher<std_msgs::msg::String>("/argo/messages", latched);
  report_pub = node->create_publisher<std_msgs::msg::String>("/argo/report", 10);
  formulate_cli = node->create_client<go2_interfaces::srv::FormulateGoals>("/argo/gemma/formulate_goals");
  assess_cli = node->create_client<go2_interfaces::srv::AssessFailure>("/argo/gemma/assess_failure");
  appraise_cli = node->create_client<go2_interfaces::srv::AppraiseFindings>("/argo/gemma/appraise_findings");
  speak_cli = node->create_client<go2_interfaces::srv::Speak>("/argo/gemma/speak");
  mode_cli = node->create_client<go2_interfaces::srv::Mode>("/mode");
}

bool Context::add(const std::string & p) {return problem->addPredicate(plansys2::Predicate(p));}
bool Context::remove(const std::string & p) {return problem->removePredicate(plansys2::Predicate(p));}

bool Context::set_function(const std::string & f, double value)
{
  plansys2::Function fn("(= " + f + " " + num(value) + ")");
  return problem->existFunction(fn) ? problem->updateFunction(fn) : problem->addFunction(fn);
}

bool Context::load_kb()
{
  const std::string r = onto.robot;
  bool ok = problem->clearKnowledge();
  ok = ok && problem->addInstance(plansys2::Instance(r, "robot"));
  for (const auto & n : onto.nodes()) {
    ok = ok && problem->addInstance(plansys2::Instance(n, "location"));
  }
  for (const auto & [a, b, d] : onto.edges) {
    for (const auto & [x, y] : {std::make_pair(a, b), std::make_pair(b, a)}) {
      ok = ok && add("(connected " + x + " " + y + ")");
      ok = ok && add("(path_free " + x + " " + y + ")");
      ok = ok && set_function("(distance " + x + " " + y + ")", d);
    }
  }
  for (const auto & [name, p] : onto.places) {
    if (p.camera_allowed) {ok = ok && add("(camera_allowed " + name + ")");}
    if (p.needs_permission) {ok = ok && add("(needs_permission " + name + ")");}
    if (p.dock) {ok = ok && add("(is_dock " + name + ")");}
    if (p.social_spot) {ok = ok && add("(social_spot " + name + ")");}
  }
  for (const auto & n : onto.nodes()) {
    auto p = onto.places.find(n);
    if (p == onto.places.end() || !p->second.needs_permission) {
      ok = ok && add("(may_enter " + n + ")");
    }
    const int vps = p == onto.places.end() ? 0 : p->second.viewpoints;
    ok = ok && set_function("(inspect_time " + n + ")", 20.0 * std::max(1, vps));
  }
  ok = ok && set_function("(speed " + r + ")", onto.speed);
  ok = ok && set_function("(drain " + r + ")", onto.drain);
  ok = ok && set_function("(battery " + r + ")", soc >= 0 ? soc : 100.0);
  ok = ok && set_function("(reserve " + r + ")", onto.reserve);
  ok = ok && set_function("(chat_time " + r + ")", onto.chat_time);
  ok = ok && set_function("(to_call " + r + ")", 0);
  ok = ok && add("(robot_at " + r + " " + (robot_xy ? nearest_node() : onto.home) + ")");
  return ok;
}

std::vector<std::vector<std::string>> Context::predicates(const std::string & name)
{
  std::vector<std::vector<std::string>> out;
  for (const auto & p : problem->getPredicates()) {
    if (p.name != name) {continue;}
    std::vector<std::string> args;
    for (const auto & param : p.parameters) {args.push_back(param.name);}
    out.push_back(args);
  }
  return out;
}

std::string Context::robot_node()
{
  for (const auto & args : predicates("robot_at")) {
    if (args.size() == 2 && args[0] == onto.robot) {return args[1];}
  }
  return "";
}

std::string Context::nearest_node() const
{
  if (!robot_xy) {return onto.home;}
  std::string best = onto.home;
  double best_d = std::numeric_limits<double>::max();
  for (const auto & n : onto.nodes()) {
    const auto p = onto.xy(n);
    const double d = std::hypot(p[0] - (*robot_xy)[0], p[1] - (*robot_xy)[1]);
    if (d < best_d) {best_d = d; best = n;}
  }
  return best;
}

std::set<std::string> Context::reachable(const std::string & from)
{
  std::map<std::string, std::vector<std::string>> adj;
  for (const auto & args : predicates("path_free")) {
    if (args.size() == 2) {adj[args[0]].push_back(args[1]);}
  }
  std::set<std::string> seen{from};
  std::vector<std::string> stack{from};
  while (!stack.empty()) {
    auto n = stack.back();
    stack.pop_back();
    for (const auto & m : adj[n]) {
      if (seen.insert(m).second) {stack.push_back(m);}
    }
  }
  return seen;
}

void Context::clear_mission_facts()
{
  for (const std::string name : {"announced", "message_delivered", "inspected", "socialized", "done",
      "unlocked", "precedes", "needs_call", "has_message"})
  {
    for (const auto & args : predicates(name)) {
      std::string p = "(" + name;
      for (const auto & a : args) {p += " " + a;}
      remove(p + ")");
    }
  }
  set_function("(to_call " + onto.robot + ")", 0);
}

bool Context::set_goal(const std::vector<std::string> & facts)
{
  std::string goal = "(and";
  for (const auto & f : facts) {goal += " " + f;}
  goal += " (robot_at " + onto.robot + " " + onto.home + "))";
  problem->clearGoal();
  return problem->setGoal(plansys2::Goal(goal));
}

bool Context::goal_active()
{
  return !problem->getGoal().nodes.empty();
}

void Context::publish_messages()
{
  nlohmann::json js = nlohmann::json::object();
  for (const auto & t : targets) {
    const auto & occ = onto.places.count(t) ? onto.places.at(t).occupants : std::vector<std::string>{};
    js[onto.delivery_node(t)] = {{"name", occ.empty() ? "" : occ.front()}, {"text", message_text}};
  }
  std_msgs::msg::String msg;
  msg.data = js.dump();
  messages_pub->publish(msg);
}

void Context::say(const std::string & text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  voice_seen_busy = false;
  say_pub->publish(msg);
  RCLCPP_INFO(node->get_logger(), "[SAY] %s", text.c_str());
}

std::string Context::spoken(const std::string & place) const
{
  auto it = synonyms_.find(place);
  return (it != synonyms_.end() && !it->second.empty()) ? it->second.front() : place;
}

}  // namespace argo
