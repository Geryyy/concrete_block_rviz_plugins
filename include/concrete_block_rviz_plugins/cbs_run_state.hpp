#pragma once

#include <cmath>
#include <map>
#include <string>
#include <vector>

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

  // Residual light: not measured, within the 1 cm gate, correctable, do not correct.
  enum class Light
  {
    Grey,
    Green,
    Yellow,
    Red
  };

  // What measured the carried block: the lidar, or nothing and the FK pose stood.
  enum class Source
  {
    Unknown,
    Lidar,
    Fk
  };

  // One in-air pass, as the operator reads it; `fk` when the lidar did not measure it.
  struct PassRow
  {
    std::string text;
    bool fk{false};
  };

  void startRequested(const std::string & tree_name);
  void goalAccepted(bool accepted);
  // `now_s` any monotonic clock; it only times how long the running node has run.
  void feedback(
    const std::string & node_name, const std::string & current_status, double now_s = 0.0);
  void goalFinished(bool succeeded);

  // "waiting:<expected>[|<exit command>…]" or "running:<command>"
  void operatorState(const std::string & latched);

  void taskStatus(const std::string & line);
  void planStatus(const std::string & line);
  void inAirStatus(const std::string & line);
  void loadOffStatus(const std::string & line);
  // `/cbs/status/pose`: RunPoseEstimation's mode, block, ok and the block pose after it.
  void poseStatus(const std::string & line);
  // `/assembly_operator/refinement_status` and `placement_translation_error_m`.
  void placementResidual(const std::string & text);
  void placementError(double error_m);
  // crane_msgs/SolverHealth SOLVE_* and crane_msgs/SwaySettled SETTLED_* wire values; the
  // numbering is copied rather than included, to keep this class free of message packages.
  void solverHealth(int outcome, double solve_time_s);
  // `max_rate` the larger |velocity| [rad/s]; `verdict` the message, which names the bound.
  void swaySettled(int settled, double max_rate = NAN, const std::string & verdict = "");

  Run run() const {return run_;}
  Phase phase() const {return phase_;}
  const std::string & tree_name() const {return tree_name_;}
  std::string run_text() const;
  std::string phase_text() const;
  // One plain sentence for what the run is doing now.
  std::string activity_text(double now_s = 0.0) const;
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
  // This block's passes: miss, measurement source, the shift the tree applied after it.
  std::vector<PassRow> pass_rows() const;
  // The post-release REFINE_BLOCK against the slot; empty until it ran for this block.
  std::string placed_text() const;
  std::string last_block_text() const {return last_block_;}
  Light placement_light() const;
  std::string placement_text() const;
  // Only an amber light has a correction worth applying.
  bool correct_available() const {return placement_light() == Light::Yellow;}
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

  std::string block_summary() const;

  Run run_{Run::Idle};
  Phase phase_{Phase::Unknown};
  std::string tree_name_;
  std::string node_name_;
  std::string node_status_;
  double node_since_s_{0.0};
  std::string expected_command_;
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

  // In `world`, the task line's frame; NaN without a world pose.
  struct Pass
  {
    double x, y, z;
    Source source;
  };
  std::vector<Pass> passes_;
  // The last REFINE_GRASPED, waiting for the measurement it fed.
  Source grasped_{Source::Unknown};
  Status placed_;
  std::string last_block_;
  bool placement_measured_{false};
  double placement_error_m_{0.0};
  double placement_dx_{0.0};
  double placement_dy_{0.0};
  Source placement_source_{Source::Unknown};
  double sway_rate_{NAN};
  double sway_bound_{NAN};
  int solver_outcome_{0};
  double solve_time_s_{0.0};
  bool solver_seen_{false};
  int settled_{0};
};

}  // namespace concrete_block_rviz_plugins
