#pragma once

#include <string>

namespace concrete_block_rviz_plugins
{

/// What CBSRunPanel shows and enables, derived from ROS state alone.
/**
 * Qt-free so it can be tested without an RViz session. Two inputs: the
 * /execute_bt goal lifecycle with its feedback, and the latched
 * /assembly_operator/state. Nothing here remembers that *this* panel pressed
 * Start -- the latched state is what survives an RViz restart.
 */
class CBSRunState
{
public:
  enum class Run
  {
    Idle,
    Starting,
    Running,
    Complete,
    Stopped
  };

  void startRequested(const std::string & tree_name);
  void goalAccepted(bool accepted);
  void feedback(const std::string & node_name, const std::string & current_status);
  void goalFinished(bool succeeded);

  // "waiting:<expected>[|<exit command>…]" or "running:<command>"
  void operatorState(const std::string & latched);

  Run run() const {return run_;}
  const std::string & tree_name() const {return tree_name_;}
  std::string run_text() const;
  std::string phase_text() const;
  // Amber banner: the tree is parked on an approval node, so the deadman decides.
  bool deadman_wait() const;
  bool stop_available() const {return run_ == Run::Running;}
  bool retry_available() const {return run_ == Run::Running && !tree_name_.empty();}
  bool step_section_visible() const {return waiting_;}
  // The one command that advances the run; the refinement pair is offered apart.
  const std::string & continue_command() const {return continue_command_;}
  std::string continue_label() const;
  const std::string & measure_command() const {return measure_command_;}
  const std::string & correct_command() const {return correct_command_;}

private:
  Run run_{Run::Idle};
  std::string tree_name_;
  std::string node_name_;
  std::string node_status_;
  // The approval node that is still RUNNING, if any. Latched, not read off the
  // last change: GetUserApproval is a decorator and stays RUNNING while its
  // child ticks, so the last change names the child for most of the gated move.
  std::string approval_node_;
  bool waiting_{false};
  std::string continue_command_;
  std::string measure_command_;
  std::string correct_command_;
};

}  // namespace concrete_block_rviz_plugins
