#pragma once

#include <map>
#include <string>

namespace concrete_block_rviz_plugins
{

/// A `key=value` status line; a token without '=' continues the previous value.
/**
 * So a trailing `msg=…` may hold the publisher's own sentence, spaces and all,
 * without a second encoding.
 */
std::map<std::string, std::string> parse_status(const std::string & text);

/// What CBSRunPanel shows and enables, derived from ROS state alone.
/**
 * Qt-free so it can be tested without an RViz session. Inputs: the /execute_bt
 * goal lifecycle with its feedback, the latched /assembly_operator/state, the
 * four latched `/cbs/status` lines, and the MPC's two report streams. Nothing
 * here remembers that *this* panel pressed Start -- the latched state is what
 * survives an RViz restart.
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

  /// The phase bar of docs/features/cbs-arch; Unknown leaves the bar where it was.
  enum class Phase
  {
    Unknown,
    Pick,
    Lift,
    Carry,
    Approach,
    Correct,
    Descend,
    Touchdown,
    Return
  };

  void startRequested(const std::string & tree_name);
  void goalAccepted(bool accepted);
  void feedback(const std::string & node_name, const std::string & current_status);
  void goalFinished(bool succeeded);

  // "waiting:<expected>[|<exit command>…]" or "running:<command>"
  void operatorState(const std::string & latched);

  void taskStatus(const std::string & line);
  void planStatus(const std::string & line);
  void inAirStatus(const std::string & line);
  void loadOffStatus(const std::string & line);
  // crane_msgs/SolverHealth SOLVE_* and crane_msgs/SwaySettled SETTLED_* wire values; the
  // numbering is copied rather than included, to keep this class free of message packages.
  void solverHealth(int outcome, double solve_time_s);
  void swaySettled(int settled);

  Run run() const {return run_;}
  Phase phase() const {return phase_;}
  const std::string & tree_name() const {return tree_name_;}
  std::string run_text() const;
  std::string phase_text() const;
  // `pick › lift › …`, the reached phase upper-cased.
  std::string phase_bar() const;
  // Amber banner: the tree is parked on an approval node, so the deadman decides.
  bool deadman_wait() const;
  // What the operator has to do right now; empty when nothing is awaited.
  std::string prompt() const;
  // `<plan> · block k/N · <block> → <slot>`
  std::string header_text() const;
  std::string planner_text() const;
  std::string in_air_text() const;
  std::string touchdown_text() const;
  std::string mpc_text() const;
  // Share of m g r already off the hook, and the gate the tree releases at. Both 0 unawaited.
  double unload_fraction() const;
  double unload_gate() const;
  bool touchdown_visible() const {return !load_off_.empty();}
  bool stop_available() const {return run_ == Run::Running;}
  bool retry_available() const {return run_ == Run::Running && !tree_name_.empty();}
  bool step_section_visible() const {return waiting_;}
  // The one command that advances the run; the refinement pair is offered apart.
  const std::string & continue_command() const {return continue_command_;}
  std::string continue_label() const;
  const std::string & measure_command() const {return measure_command_;}
  const std::string & correct_command() const {return correct_command_;}

private:
  using Status = std::map<std::string, std::string>;

  Run run_{Run::Idle};
  Phase phase_{Phase::Unknown};
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

  Status task_;
  Status plan_;
  Status in_air_;
  Status load_off_;
  int solver_outcome_{0};
  double solve_time_s_{0.0};
  bool solver_seen_{false};
  int settled_{0};
};

}  // namespace concrete_block_rviz_plugins
