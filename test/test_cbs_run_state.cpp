#include <gtest/gtest.h>

#include "concrete_block_rviz_plugins/cbs_run_state.hpp"

using concrete_block_rviz_plugins::CBSRunState;

TEST(CBSRunState, deadman_wait_while_the_tree_sits_on_an_approval_node)
{
  CBSRunState state;
  state.startRequested("wall_assembly.xml");
  state.goalAccepted(true);
  EXPECT_FALSE(state.deadman_wait());

  state.feedback("CheckUserApproval", "RUNNING");
  EXPECT_TRUE(state.deadman_wait());
  EXPECT_EQ(state.phase_text(), "Phase: CheckUserApproval: RUNNING");

  state.feedback("GetUserApproval", "RUNNING");
  EXPECT_TRUE(state.deadman_wait());

  // GetUserApproval is a decorator: it stays RUNNING while its child ticks, and
  // the deadman has to stay held for all of it. The banner must survive the
  // child's status changes.
  state.feedback("FollowJointTrajectory", "RUNNING");
  EXPECT_TRUE(state.deadman_wait());

  // The node left RUNNING: the approval is in, the banner has to go.
  state.feedback("GetUserApproval", "SUCCESS");
  EXPECT_FALSE(state.deadman_wait());

  // A stale approval feedback must not keep the banner up after the run ended.
  state.feedback("CheckUserApproval", "RUNNING");
  state.goalFinished(true);
  EXPECT_FALSE(state.deadman_wait());
}

TEST(CBSRunState, one_continue_for_the_awaited_command)
{
  CBSRunState state;
  state.operatorState("waiting:a|b");
  EXPECT_TRUE(state.step_section_visible());
  EXPECT_EQ(state.continue_command(), "a");
  EXPECT_EQ(state.continue_label(), "Continue: a");
  EXPECT_TRUE(state.measure_command().empty());
  EXPECT_TRUE(state.correct_command().empty());

  state.operatorState("waiting:acquire");
  EXPECT_EQ(state.continue_command(), "acquire");
  EXPECT_EQ(state.continue_label(), "Continue: acquire next task");

  // The refinement loop: measuring does not advance the run, `pick` does.
  state.operatorState("waiting:pickup_measure|pick");
  EXPECT_EQ(state.continue_command(), "pick");
  EXPECT_EQ(state.measure_command(), "pickup_measure");
  EXPECT_TRUE(state.correct_command().empty());

  state.operatorState("waiting:correct|place");
  EXPECT_EQ(state.continue_command(), "place");
  EXPECT_EQ(state.correct_command(), "correct");
  EXPECT_TRUE(state.measure_command().empty());

  // Not a wait: no step section, no command.
  state.operatorState("running:pick");
  EXPECT_FALSE(state.step_section_visible());
  EXPECT_TRUE(state.continue_command().empty());
}

TEST(CBSRunState, recovers_from_the_latched_state_without_a_start)
{
  // RViz restarted mid-run: the panel never pressed Start, the latched topic is
  // all it has.
  CBSRunState state;
  state.operatorState("waiting:pickup_correct|pick");
  EXPECT_TRUE(state.step_section_visible());
  EXPECT_EQ(state.continue_command(), "pick");
  EXPECT_EQ(state.correct_command(), "pickup_correct");
  EXPECT_EQ(state.run(), CBSRunState::Run::Idle);
}

TEST(CBSRunState, stop_only_while_a_goal_runs)
{
  CBSRunState state;
  EXPECT_FALSE(state.stop_available());
  EXPECT_FALSE(state.retry_available());

  state.startRequested("wall_assembly.xml");
  EXPECT_FALSE(state.stop_available());

  state.goalAccepted(true);
  EXPECT_TRUE(state.stop_available());
  EXPECT_TRUE(state.retry_available());

  state.goalFinished(false);
  EXPECT_FALSE(state.stop_available());
  EXPECT_FALSE(state.retry_available());
  EXPECT_EQ(state.run_text(), "Stopped");

  state.startRequested("wall_assembly.xml");
  state.goalAccepted(false);
  EXPECT_FALSE(state.stop_available());
}
