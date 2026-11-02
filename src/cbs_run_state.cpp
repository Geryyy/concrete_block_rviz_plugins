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
// EvaluatePlacementResidual's max_correction_m in operator_guided_single_block.xml.
constexpr double kMaxCorrectionM = 0.30;

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

constexpr const char * kInAirSentence =
  "In-air correction: lidar measures the carried block, shifts the waypoint by the miss";

// Same order as Phase, Unknown dropped.
const std::array<const char *, 8> kPhaseSentences{{
  "Picking the block: descend, grip, check the grasp",
  "Lifting the load clear",
  "Moving above the next target",
  "Descending to the waypoint 30 cm over the slot",
  kInAirSentence,
  "Angled entry to 5 cm over the slot",
  "Setting the block down: released once the hook is unloaded",
  "Fetching the next task",
}};

/// RUNNING node -> sentence; the settle wait and the planners are worded in activity_text.
const std::array<std::pair<const char *, const char *>, 16> kNodeSentences{{
  {"MeasureBlockFromGripper", kInAirSentence},
  {"RunPoseEstimation", "Lidar is measuring the block"},
  {"CaptureBlockGraspOffset", "Updating the grasp offset from the measured block"},
  {"Capture grasp offset", "Updating the grasp offset from the gripped block"},
  {"Capture grasp offset after vision", "Updating the grasp offset from the measured block"},
  {"WaitForLoadOff", "Touchdown: lowering at half speed until the hook is unloaded"},
  {"Write placed pose", "Block landed: writing its pose to the world model"},
  {"WaitForMoveComplete", "Crane moving: MPC tracks the planned trajectory"},
  {"FollowJointTrajectory", "Crane moving: MPC tracks the planned trajectory"},
  {"Wait for grasp detector", "Waiting for the grasp detector to confirm the grip"},
  {"GetNextAssemblyTask", "Fetching the next task from the plan"},
  {"EvaluatePlacementResidual", "Comparing the carried block with the placement hover"},
  {"EvaluatePickupResidual", "Comparing the pickup block with its planned pose"},
  {"CheckUserApproval", "Waiting for your approval"},
  {"GetUserApproval", "Moving while you hold the deadman"},
  {"SetMpcPayload", "Telling the MPC what the hook carries"},
}};

/// Awaited step-by-step command -> sentence.
const std::array<std::pair<const char *, const char *>, 9> kCommandSentences{{
  {"acquire", "Step by step: waiting for you to fetch the next task"},
  {"pickup_hover", "Task served: waiting for you to send the crane above the pickup block"},
  {"pickup_measure", "Above the pickup block: check the pickup, or grasp the block"},
  {"pickup_correct", "Pickup checked: correct the hover by it, or grasp the block"},
  {"pick", "Waiting for you to grasp the block"},
  {"hover", "Block gripped: waiting for you to carry it to the placement hover"},
  {"measure", "At the placement hover: measure the residual, or place the block"},
  {"correct", "Residual measured: correct the hover by it, or place the block"},
  {"place", "Waiting for you to place the block"},
}};

bool is_planner(const std::string & node_name)
{
  return node_name == "CalcA2BMovement" || node_name == "CalcGripMovement" ||
         node_name == "Calc approach target" || node_name.rfind("Plan ", 0) == 0;
}

template<std::size_t N>
const char * lookup(
  const std::array<std::pair<const char *, const char *>, N> & table, const std::string & key)
{
  for (const auto & entry : table) {
    if (key == entry.first) {
      return entry.second;
    }
  }
  return nullptr;
}

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

const char * source_text(CBSRunState::Source source)
{
  switch (source) {
    case CBSRunState::Source::Lidar: return " · lidar";
    case CBSRunState::Source::Fk: return " · FK fallback";
    case CBSRunState::Source::Unknown: break;
  }
  return "";
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
  pass_rows_.clear();
  placed_.clear();
  grasped_ = Source::Unknown;
  placement_measured_ = false;
  placement_failed_ = false;
}

void CBSRunState::goalAccepted(bool accepted)
{
  run_ = accepted ? Run::Running : Run::Stopped;
}

void CBSRunState::feedback(
  const std::string & node_name, const std::string & current_status, double now_s)
{
  if (node_name != node_name_ || current_status != node_status_) {
    node_since_s_ = now_s;
  }
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
  expected_command_.clear();
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
    if (expected_command_.empty()) {
      expected_command_ = command;
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

void CBSRunState::taskStatus(const std::string & line)
{
  const Status next = parse_status(line);
  const auto key = [](const Status & status) {
      return value(status, "plan") + "|" + value(status, "task") + "|" + value(status, "block");
    };
  // A re-served task (retry) keeps its history; a new block, or the first task line after
  // the latched replay, starts a fresh one.
  if (key(next) != key(task_)) {
    if (!task_.empty() && (!pass_rows_.empty() || !placed_.empty())) {
      last_block_ = block_summary();
    }
    pass_rows_.clear();
    placed_.clear();
    grasped_ = Source::Unknown;
    placement_measured_ = false;
    placement_failed_ = false;
  }
  task_ = next;
}

void CBSRunState::planStatus(const std::string & line) {plan_ = parse_status(line);}

void CBSRunState::inAirStatus(const std::string & line) {in_air_ = parse_status(line);}

void CBSRunState::placePassStatus(const std::string & line)
{
  const Status pass = parse_status(line);
  const std::string verdict = value(pass, "verdict");
  std::string action = verdict;
  if (verdict == "shift") {
    action = formatted(
      "shift (%+.0f, %+.0f) mm", number(pass, "shift_x") * 1e3, number(pass, "shift_y") * 1e3);
  } else if (verdict == "within") {
    action = "within, done";
  } else if (verdict == "height") {
    action = formatted("height off %+.0f mm, approach fails", number(pass, "miss_z") * 1e3);
  } else if (verdict == "final_fail") {
    action = "final check still off, approach fails";
  }
  last_miss_m_ = std::hypot(number(pass, "miss_x"), number(pass, "miss_y"));
  pass_rows_.push_back(
    {formatted(
        "attempt %.0f pass %.0f: miss (%+.0f, %+.0f) = %.0f mm%s · %s · total (%+.0f, %+.0f) mm"
        " (K0)", number(pass, "attempt"), number(pass, "pass"), number(pass, "miss_x") * 1e3,
        number(pass, "miss_y") * 1e3, last_miss_m_ * 1e3, source_text(grasped_), action.c_str(),
        number(pass, "off_x") * 1e3, number(pass, "off_y") * 1e3), grasped_ == Source::Fk});
  grasped_ = Source::Unknown;
}

void CBSRunState::settleStatus(const std::string & line) {settle_ = parse_status(line);}

void CBSRunState::loadOffStatus(const std::string & line) {load_off_ = parse_status(line);}

void CBSRunState::poseStatus(const std::string & line)
{
  const Status pose = parse_status(line);
  const std::string mode = value(pose, "mode");
  if (mode == "REFINE_GRASPED") {
    grasped_ = value(pose, "ok") == "1" ? Source::Lidar : Source::Fk;
    return;
  }
  // The pickup's REFINE_BLOCK of the same block comes before any pass.
  if (
    mode == "REFINE_BLOCK" && !pass_rows_.empty() &&
    value(pose, "block") == value(task_, "block"))
  {
    placed_ = pose;
  }
}

void CBSRunState::placementResidual(const std::string & text)
{
  placement_failed_ = text.rfind("Placement residual failed", 0) == 0;
  if (placement_failed_) {
    placement_measured_ = false;
    return;
  }
  const auto at = text.find("dxyz=[");
  double dx = 0.0;
  double dy = 0.0;
  if (
    at != std::string::npos &&
    std::sscanf(text.c_str() + at, "dxyz=[%lf, %lf", &dx, &dy) == 2)
  {
    placement_dx_ = dx;
    placement_dy_ = dy;
  }
}

void CBSRunState::placementError(double error_m)
{
  // Paired here, the last of the residual's topics, after the REFINE_GRASPED line.
  placement_source_ = grasped_;
  grasped_ = Source::Unknown;
  placement_error_m_ = error_m;
  placement_measured_ = true;
}

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

std::string CBSRunState::activity_text(double now_s) const
{
  if (waiting_) {
    const char * sentence = lookup(kCommandSentences, expected_command_);
    return sentence ? std::string(sentence) : "Waiting for your command: " + expected_command_;
  }
  if (node_status_.empty() || node_status_ == "IDLE") {
    return run_text();
  }
  if (node_status_ == "RUNNING") {
    const double running_s = now_s - node_since_s_;
    if (node_name_ == "Wait for the load to settle" && !settle_.empty()) {
      // WaitForMoveComplete's own gate: its sway_bound on the rates, or 0 and the MPC verdict.
      const double rate = number(settle_, "rate", NAN);
      const double bound = number(settle_, "bound");
      std::string text = "Waiting for the load to settle: sway ";
      text += std::isnan(rate) ? "unknown" : formatted("%.3f rad/s", rate);
      text += bound > 0.0 ? formatted(" → %.3f rad/s", bound) : " → MPC verdict settled";
      if (value(settle_, "still") == "1") {
        text += formatted(
          ", held %.1f of %.1f s", number(settle_, "held"), number(settle_, "hold"));
      }
      return text + formatted(" (%.0f s)", number(settle_, "elapsed"));
    }
    if (is_planner(node_name_)) {
      return formatted("Planning the next move… %.0f s", running_s);
    }
    if (const char * sentence = lookup(kNodeSentences, node_name_)) {
      return sentence;
    }
  }
  if (phase_ != Phase::Unknown) {
    return kPhaseSentences[static_cast<std::size_t>(phase_) - 1];
  }
  return node_name_;
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
  return formatted(
    "In air: (%.3f, %.3f, %.3f) %s, tilt %.2f deg", number(in_air_, "x"), number(in_air_, "y"),
    number(in_air_, "z"), value(in_air_, "frame", "?").c_str(), number(in_air_, "tilt_deg"));
}

std::string CBSRunState::placed_text() const
{
  if (placed_.empty()) {
    return {};
  }
  const std::string block = value(placed_, "block", "?");
  if (value(placed_, "ok") != "1" || placed_.count("x") == 0) {
    return "Placed " + block + ": not re-measured, the lidar did not find it";
  }
  if (task_.count("target_x") == 0 || task_.count("target_y") == 0) {
    return formatted(
      "Placed %s: at (%.3f, %.3f)", block.c_str(), number(placed_, "x"), number(placed_, "y"));
  }
  const double dx = number(placed_, "x") - number(task_, "target_x");
  const double dy = number(placed_, "y") - number(task_, "target_y");
  return formatted(
    "Placed %s: %.0f mm off slot (%+.0f, %+.0f)", block.c_str(), std::hypot(dx, dy) * 1e3,
    dx * 1e3, dy * 1e3);
}

std::string CBSRunState::block_summary() const
{
  std::string text = "Last block " + value(task_, "block", "?") + ":";
  if (!pass_rows_.empty()) {
    text += formatted(
      " in-air miss %.0f mm after %zu pass%s,", last_miss_m_ * 1e3, pass_rows_.size(),
      pass_rows_.size() == 1 ? "" : "es");
  }
  const std::string placed = placed_text();
  const auto colon = placed.find(": ");
  return text + " placed " +
         (colon == std::string::npos ? std::string("not re-measured") : placed.substr(colon + 2));
}

CBSRunState::Light CBSRunState::placement_light() const
{
  // A new measurement is awaited: the last one was taken before the correction moved.
  if (expected_command_ == "measure") {
    return Light::Grey;
  }
  if (placement_failed_) {
    return Light::Red;
  }
  if (!placement_measured_) {
    return Light::Grey;
  }
  // <= as EvaluatePlacementResidual's translation_tolerance_m; FK and clamped still correct.
  return placement_error_m_ <= kMissGateM ? Light::Green : Light::Yellow;
}

std::string CBSRunState::placement_text() const
{
  const std::string head = formatted(
    "Residual %.0f mm%s", placement_error_m_ * 1e3,
    source_text(placement_source_));
  switch (placement_light()) {
    case Light::Grey: return "Residual: not measured";
    case Light::Green: return head + ", within 1 cm (yaw not checked): place";
    case Light::Red: return "Residual: the measurement failed, nothing to correct";
    case Light::Yellow: break;
  }
  // The same clamp EvaluatePlacementResidual applies to the move.
  const double d = std::hypot(placement_dx_, placement_dy_);
  const double scale = d > kMaxCorrectionM ? kMaxCorrectionM / d : 1.0;
  return head + formatted(
    ", over 1 cm: Correct moves hover by (%+.0f, %+.0f) mm (K0)%s",
    placement_dx_ * scale * 1e3, placement_dy_ * scale * 1e3,
    scale < 1.0 ? ", clamped to 30 cm" : "");
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
