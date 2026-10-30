#include "concrete_block_rviz_plugins/cbs_run_state.hpp"

#include <sstream>
#include <string>

namespace concrete_block_rviz_plugins
{

namespace
{

constexpr const char * kWaitingPrefix = "waiting:";

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

}  // namespace

void CBSRunState::startRequested(const std::string & tree_name)
{
  tree_name_ = tree_name;
  run_ = Run::Starting;
  node_name_.clear();
  node_status_.clear();
  approval_node_.clear();
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

bool CBSRunState::deadman_wait() const
{
  return run_ == Run::Running && !approval_node_.empty();
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
