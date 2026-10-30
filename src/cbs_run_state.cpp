#include "concrete_block_rviz_plugins/cbs_run_state.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <exception>
#include <sstream>
#include <string>
#include <utility>

namespace concrete_block_rviz_plugins
{

namespace
{

constexpr const char * kWaitingPrefix = "waiting:";
// The gate subtree_place_block.xml's correction loop stops at (`miss^2 >= 1e-4`).
constexpr double kMissGateM = 0.01;

bool is_measure(const std::string & command)
{
  return command == "pickup_measure" || command == "measure";
}

bool is_correct(const std::string & command)
{
  return command == "pickup_correct" || command == "correct";
}

bool is_approval(const std::string & node_name)
{
  return node_name == "GetUserApproval" || node_name == "CheckUserApproval";
}

using Phase = CBSRunState::Phase;

/// BT node name -> phase. The names are the trees' `name=` attributes, or the node type
/// where the tree gives none; `descend_to` and `lift` recur, so the bar is coarse by design.
const std::array<std::pair<const char *, Phase>, 18> kPhases{{
  {"pick_block", Phase::Pick},
  {"pick_from_hover", Phase::Pick},
  {"regrip", Phase::Pick},
  {"set_down", Phase::Pick},
  {"lift", Phase::Lift},
  {"move_above", Phase::Carry},
  {"resume_carry", Phase::Carry},
  {"place_block", Phase::Approach},
  {"place_approach", Phase::Approach},
  {"Plan approach", Phase::Approach},
  {"placement_refinement_loop", Phase::Correct},
  {"MeasureBlockFromGripper", Phase::Correct},
  {"place_finish", Phase::Descend},
  {"descend_to", Phase::Descend},
  {"Plan descend", Phase::Descend},
  {"touchdown", Phase::Touchdown},
  {"WaitForLoadOff", Phase::Touchdown},
  {"GetNextAssemblyTask", Phase::Return},
}};

const std::array<const char *, 8> kPhaseNames{
  {"pick", "lift", "carry", "approach", "correct", "descend", "touchdown", "return"}};

Phase phase_of(const std::string & node_name)
{
  for (const auto & entry : kPhases) {
    if (node_name == entry.first) {
      return entry.second;
    }
  }
  return Phase::Unknown;
}

std::string upper(const std::string & text)
{
  std::string out = text;
  std::transform(
    out.begin(), out.end(), out.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  return out;
}

std::string value(
  const std::map<std::string, std::string> & status, const char * key,
  const char * fallback = "")
{
  const auto found = status.find(key);
  return found == status.end() ? std::string(fallback) : found->second;
}

/// The key as a double, or `fallback` when absent or unparsable.
double number(
  const std::map<std::string, std::string> & status, const char * key, double fallback = 0.0)
{
  const auto found = status.find(key);
  if (found == status.end()) {
    return fallback;
  }
  try {
    return std::stod(found->second);
  } catch (const std::exception &) {
    return fallback;
  }
}

std::string formatted(const char * format, ...)
__attribute__((format(printf, 1, 2)));

std::string formatted(const char * format, ...)
{
  char line[320];
  va_list args;
  va_start(args, format);
  std::vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  return std::string(line);
}

}  // namespace

std::map<std::string, std::string> parse_status(const std::string & text)
{
  std::map<std::string, std::string> status;
  std::istringstream tokens(text);
  std::string token;
  std::string key;
  while (tokens >> token) {
    const auto split = token.find('=');
    if (split == std::string::npos) {
      // Continuation of a free-text value; the first token has to carry a key.
      if (!key.empty()) {
        status[key] += " " + token;
      }
      continue;
    }
    key = token.substr(0, split);
    status[key] = token.substr(split + 1);
  }
  return status;
}

void CBSRunState::startRequested(const std::string & tree_name)
{
  tree_name_ = tree_name;
  run_ = Run::Starting;
  node_name_.clear();
  node_status_.clear();
  approval_node_.clear();
  phase_ = Phase::Unknown;
}

void CBSRunState::goalAccepted(bool accepted)
{
  run_ = accepted ? Run::Running : Run::Stopped;
}

void CBSRunState::feedback(const std::string & node_name, const std::string & current_status)
{
  node_name_ = node_name;
  node_status_ = current_status;
  if (is_approval(node_name)) {
    approval_node_ = current_status == "RUNNING" ? node_name : std::string();
  }
  // Entering a leg is the phase change; leaving it is the next leg's news.
  const Phase reached = phase_of(node_name);
  if (reached != Phase::Unknown && current_status == "RUNNING") {
    phase_ = reached;
  }
}

void CBSRunState::goalFinished(bool succeeded)
{
  run_ = succeeded ? Run::Complete : Run::Stopped;
  node_name_.clear();
  node_status_.clear();
  approval_node_.clear();
}

void CBSRunState::operatorState(const std::string & latched)
{
  waiting_ = false;
  continue_command_.clear();
  measure_command_.clear();
  correct_command_.clear();
  if (latched.rfind(kWaitingPrefix, 0) != 0) {
    return;
  }
  waiting_ = true;
  std::istringstream commands(latched.substr(std::string(kWaitingPrefix).size()));
  std::string command;
  while (std::getline(commands, command, '|')) {
    if (command.empty()) {
      continue;
    }
    if (is_measure(command)) {
      if (measure_command_.empty()) {measure_command_ = command;}
    } else if (is_correct(command)) {
      if (correct_command_.empty()) {correct_command_ = command;}
    } else if (continue_command_.empty()) {
      continue_command_ = command;
    }
  }
}

void CBSRunState::taskStatus(const std::string & line) {task_ = parse_status(line);}
void CBSRunState::planStatus(const std::string & line) {plan_ = parse_status(line);}
void CBSRunState::inAirStatus(const std::string & line) {in_air_ = parse_status(line);}
void CBSRunState::loadOffStatus(const std::string & line) {load_off_ = parse_status(line);}

void CBSRunState::solverHealth(int outcome, double solve_time_s)
{
  solver_outcome_ = outcome;
  solve_time_s_ = solve_time_s;
  solver_seen_ = true;
}

void CBSRunState::swaySettled(int settled) {settled_ = settled;}

std::string CBSRunState::run_text() const
{
  switch (run_) {
    case Run::Starting: return "Starting";
    case Run::Running: return "Running";
    case Run::Complete: return "Complete";
    case Run::Stopped: return "Stopped";
    case Run::Idle: break;
  }
  return "Idle";
}

std::string CBSRunState::phase_text() const
{
  if (node_status_.empty() || node_status_ == "IDLE") {
    return "Phase: idle";
  }
  return "Phase: " + node_name_ + ": " + node_status_;
}

std::string CBSRunState::phase_bar() const
{
  const auto reached = static_cast<std::size_t>(phase_);
  std::string bar;
  for (std::size_t index = 0; index < kPhaseNames.size(); ++index) {
    if (index != 0) {
      bar += " › ";
    }
    const std::string name = kPhaseNames[index];
    bar += index + 1 == reached ? upper(name) : name;
  }
  return bar;
}

bool CBSRunState::deadman_wait() const
{
  return run_ == Run::Running && !approval_node_.empty();
}

std::string CBSRunState::prompt() const
{
  if (!deadman_wait()) {
    return {};
  }
  // CheckUserApproval gates one move on a fresh press; GetUserApproval is the decorator
  // that keeps the move alive, so its ask is to keep holding.
  return approval_node_ == "CheckUserApproval" ?
         "Approve next move - press button12 / TUI 'z'" :
         "Hold deadman while moving";
}

std::string CBSRunState::header_text() const
{
  if (task_.empty()) {
    return "No task served yet";
  }
  return formatted(
    "%s · block %s · %s → %s", value(task_, "plan", "?").c_str(),
    value(task_, "task", "?").c_str(), value(task_, "block", "?").c_str(),
    value(task_, "slot", "?").c_str());
}

std::string CBSRunState::planner_text() const
{
  if (plan_.empty()) {
    return "Planner: no request answered yet";
  }
  if (value(plan_, "ok") != "1") {
    return "Planner refused: " + value(plan_, "msg", "no reason given");
  }
  return formatted(
    "Planner: %.2f s, %.0f SQP it, arrives %.2f deg off rest at %.3f rad/s",
    number(plan_, "duration"), number(plan_, "iterations"),
    number(plan_, "sway_rad") * 180.0 / M_PI, number(plan_, "sway_rate"));
}

std::string CBSRunState::in_air_text() const
{
  if (in_air_.empty()) {
    return "In air: not measured";
  }
  std::string text = formatted(
    "In air: pass %.0f, (%.3f, %.3f, %.3f), tilt %.2f deg", number(in_air_, "pass"),
    number(in_air_, "x"), number(in_air_, "y"), number(in_air_, "z"),
    number(in_air_, "tilt_deg"));
  // Only against the waypoint the tree itself corrects to; without it there is no miss.
  if (task_.count("approach_x") == 0 || task_.count("approach_y") == 0) {
    return text;
  }
  const double miss = std::hypot(
    number(task_, "approach_x") - number(in_air_, "x"),
    number(task_, "approach_y") - number(in_air_, "y"));
  return text + formatted(
    ", miss %.1f cm %s %.1f cm gate", miss * 100.0, miss < kMissGateM ? "within" : "over",
    kMissGateM * 100.0);
}

double CBSRunState::unload_fraction() const {return number(load_off_, "unload");}

double CBSRunState::unload_gate() const {return number(load_off_, "gate");}

std::string CBSRunState::touchdown_text() const
{
  if (load_off_.empty()) {
    return "Touchdown: not descending";
  }
  return formatted(
    "Touchdown: unload %.0f %% of %.0f %% · centre %+.3f m from target%s",
    unload_fraction() * 100.0, unload_gate() * 100.0, number(load_off_, "above"),
    value(load_off_, "landed") == "1" ? " · landed" : "");
}

std::string CBSRunState::mpc_text() const
{
  const char * outcome = "unknown";
  switch (solver_outcome_) {
    case 1: outcome = "converged"; break;
    case 2: outcome = "over budget"; break;
    case 3: outcome = "failed"; break;
    default: break;
  }
  const char * sway = settled_ == 2 ? "settled" : (settled_ == 1 ? "swinging" : "unknown");
  if (!solver_seen_) {
    return formatted("MPC: not driving · sway %s", sway);
  }
  return formatted(
    "MPC: %s in %.0f ms · sway %s", outcome, solve_time_s_ * 1e3, sway);
}

std::string CBSRunState::continue_label() const
{
  if (continue_command_.empty()) {
    return "Continue";
  }
  if (continue_command_ == "acquire") {return "Continue: acquire next task";}
  if (continue_command_ == "pickup_hover") {return "Continue: move to pickup hover";}
  if (continue_command_ == "pick") {return "Continue: grasp block";}
  if (continue_command_ == "hover") {return "Continue: move to placement hover";}
  if (continue_command_ == "place") {return "Continue: place block";}
  return "Continue: " + continue_command_;
}

}  // namespace concrete_block_rviz_plugins
