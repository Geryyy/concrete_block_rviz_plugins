#include <gtest/gtest.h>

#include <string>

#include "concrete_block_rviz_plugins/cbs_run_state.hpp"

using concrete_block_rviz_plugins::CBSRunState;
using concrete_block_rviz_plugins::parse_status;

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

TEST(CBSRunState, the_phase_bar_follows_the_running_leg_and_ignores_the_rest)
{
  CBSRunState state;
  state.startRequested("wall_assembly.xml");
  state.goalAccepted(true);
  EXPECT_EQ(state.phase(), CBSRunState::Phase::Unknown);

  state.feedback("pick_block", "RUNNING");
  EXPECT_EQ(state.phase(), CBSRunState::Phase::Pick);
  EXPECT_NE(state.phase_bar().find("PICK"), std::string::npos);

  // Not in the table: the bar stays where it was, it does not blank.
  state.feedback("CaptureBlockGraspOffset", "RUNNING");
  EXPECT_EQ(state.phase(), CBSRunState::Phase::Pick);

  // Leaving a leg is not the next leg's news either.
  state.feedback("pick_block", "SUCCESS");
  EXPECT_EQ(state.phase(), CBSRunState::Phase::Pick);

  state.feedback("lift", "RUNNING");
  EXPECT_EQ(state.phase(), CBSRunState::Phase::Lift);
  state.feedback("move_above", "RUNNING");
  EXPECT_EQ(state.phase(), CBSRunState::Phase::Carry);
  state.feedback("MeasureBlockFromGripper", "RUNNING");
  EXPECT_EQ(state.phase(), CBSRunState::Phase::Correct);
  state.feedback("WaitForLoadOff", "RUNNING");
  EXPECT_EQ(state.phase(), CBSRunState::Phase::Touchdown);
  EXPECT_NE(state.phase_bar().find("TOUCHDOWN"), std::string::npos);
  EXPECT_NE(state.phase_bar().find("return"), std::string::npos);
}

TEST(CBSRunState, each_approval_node_asks_for_its_own_thing)
{
  CBSRunState state;
  state.startRequested("wall_assembly.xml");
  state.goalAccepted(true);
  EXPECT_TRUE(state.prompt().empty());

  state.feedback("CheckUserApproval", "RUNNING");
  EXPECT_EQ(state.prompt(), "Approve next move - press button12 / TUI 'z'");

  state.feedback("GetUserApproval", "RUNNING");
  EXPECT_EQ(state.prompt(), "Hold deadman while moving");

  // Approval in: no ask left, same latch the banner uses.
  state.feedback("GetUserApproval", "SUCCESS");
  EXPECT_TRUE(state.prompt().empty());
}

TEST(CBSRunState, a_status_line_value_may_hold_the_publishers_own_sentence)
{
  const auto status = parse_status("ok=0 msg=the goal is inside the headboard");
  EXPECT_EQ(status.at("ok"), "0");
  EXPECT_EQ(status.at("msg"), "the goal is inside the headboard");
  // A line that opens with free text has no key to hang it on; it is dropped, not guessed.
  EXPECT_TRUE(parse_status("refused outright").empty());
}

TEST(CBSRunState, the_task_line_becomes_the_run_header)
{
  CBSRunState state;
  EXPECT_EQ(state.header_text(), "No task served yet");
  state.taskStatus(
    "plan=wall1 task=3/7 slot=c0_b0 block=block_2 "
    "pickup_x=4.123 pickup_y=-1.568 pickup_z=0.250 "
    "target_x=6.000 target_y=2.000 target_z=1.050 "
    "approach_x=6.174 approach_y=2.000 msg=Task 3/7: slot c0_b0 -> block_2");
  EXPECT_EQ(state.header_text(), "wall1 · block 3/7 · block_2 → c0_b0");
}

TEST(CBSRunState, the_plan_line_shows_the_numbers_or_the_refusal)
{
  CBSRunState state;
  EXPECT_EQ(state.planner_text(), "Planner: no request answered yet");
  state.planStatus(
    "ok=1 duration=7.423 iterations=12 sway_rad=0.02146 sway_rate=0.01234 "
    "msg=curved_over; 7.42 s");
  EXPECT_EQ(
    state.planner_text(),
    "Planner: 7.42 s, 12 SQP it, arrives 1.23 deg off rest at 0.012 rad/s");

  state.planStatus("ok=0 msg=the goal is inside the headboard");
  EXPECT_EQ(state.planner_text(), "Planner refused: the goal is inside the headboard");
}

TEST(CBSRunState, the_in_air_miss_needs_the_waypoint_the_tree_corrects_to)
{
  CBSRunState state;
  EXPECT_EQ(state.in_air_text(), "In air: not measured");
  state.inAirStatus(
    "pass=2 frame=K0_mounting_base x=8.102 y=3.350 z=-0.550 tilt_deg=1.20 samples=78 "
    "world_x=-15.162 world_y=-3.000 world_z=0.590");
  // No task line yet, so no target: the measurement alone, no invented miss.
  EXPECT_EQ(
    state.in_air_text(), "In air: pass 2, (8.102, 3.350, -0.550), tilt 1.20 deg");

  state.taskStatus("plan=wall1 task=3/7 approach_x=-15.174 approach_y=-3.000");
  EXPECT_NE(state.in_air_text().find("miss 1.2 cm over 1.0 cm gate"), std::string::npos);

  state.inAirStatus(
    "pass=3 frame=K0_mounting_base x=8.110 y=3.350 z=-0.550 tilt_deg=1.20 samples=78 "
    "world_x=-15.170 world_y=-3.000 world_z=0.590");
  EXPECT_NE(state.in_air_text().find("miss 0.4 cm within 1.0 cm gate"), std::string::npos);
}

TEST(CBSRunState, the_touchdown_bar_is_drawn_against_the_release_gate)
{
  CBSRunState state;
  EXPECT_FALSE(state.touchdown_visible());
  EXPECT_EQ(state.touchdown_text(), "Touchdown: not descending");

  state.loadOffStatus(
    "effort=111000 reference=111000 drop_needed=35367 unload=0.000 gate=0.800 r=5.12 "
    "above=+0.312 landed=0");
  EXPECT_TRUE(state.touchdown_visible());
  EXPECT_DOUBLE_EQ(state.unload_gate(), 0.8);
  EXPECT_EQ(
    state.touchdown_text(), "Touchdown: unload 0 % of 80 % · centre +0.312 m from target");

  state.loadOffStatus(
    "effort=55000 reference=111000 drop_needed=35367 unload=0.630 gate=0.800 r=5.12 "
    "above=-0.004 landed=1");
  EXPECT_DOUBLE_EQ(state.unload_fraction(), 0.63);
  EXPECT_EQ(
    state.touchdown_text(),
    "Touchdown: unload 63 % of 80 % · centre -0.004 m from target · landed");
}

TEST(CBSRunState, the_mpc_row_says_unknown_until_the_node_is_driving)
{
  CBSRunState state;
  EXPECT_EQ(state.mpc_text(), "MPC: not driving · sway unknown");

  // SolverHealth.SOLVE_CONVERGED, SwaySettled.SETTLED_YES.
  state.solverHealth(1, 0.0112);
  state.swaySettled(2);
  EXPECT_EQ(state.mpc_text(), "MPC: converged in 11 ms · sway settled");

  // SOLVE_BUDGET_EXCEEDED with the load still swinging: both halves have to show.
  state.solverHealth(2, 0.0631);
  state.swaySettled(1);
  EXPECT_EQ(state.mpc_text(), "MPC: over budget in 63 ms · sway swinging");
}

TEST(CBSRunState, one_row_per_in_air_pass_with_the_shift_the_tree_applied)
{
  // The 281 sim's frames: the task line is in `world`, the tree measures in K0_mounting_base,
  // which is world turned by pi and shifted (world (-15.00, -3.00, 0.29) is K0 (8.06, 3.35,
  // -0.85)). Compared across the two, every miss read ~23 m.
  CBSRunState state;
  state.taskStatus(
    "plan=wall1 task=3/7 block=block_2 target_x=-15.000 target_y=-3.000 target_z=0.290 "
    "approach_x=-15.174 approach_y=-3.000");
  state.poseStatus("mode=REFINE_GRASPED block=block_2 ok=1");
  state.inAirStatus(
    "pass=7 frame=K0_mounting_base x=8.210 y=3.340 z=-0.550 "
    "world_x=-15.150 world_y=-2.990 world_z=0.590");
  // ForceSuccess hid this failure from the tree; the row must not.
  state.poseStatus("mode=REFINE_GRASPED block=block_2 ok=0");
  state.inAirStatus(
    "pass=8 frame=K0_mounting_base x=8.230 y=3.347 z=-0.550 "
    "world_x=-15.170 world_y=-2.997 world_z=0.590");

  const auto rows = state.pass_rows();
  ASSERT_EQ(rows.size(), 2u);
  // Numbered per block, not by the node's own counter.
  EXPECT_EQ(
    rows[0].text,
    "pass 1: miss (-24, -10) = 26 mm · lidar · shift (-24, -10) mm · total (-24, -10) mm"
    " (world)");
  EXPECT_FALSE(rows[0].fk);
  EXPECT_EQ(
    rows[1].text,
    "pass 2: miss (-4, -3) = 5 mm · FK fallback · within, done · total (-24, -10) mm (world)");
  EXPECT_TRUE(rows[1].fk);

  state.poseStatus(
    "mode=REFINE_BLOCK block=block_2 ok=1 frame=world x=-14.990 y=-3.005 z=0.29");
  EXPECT_EQ(state.placed_text(), "Placed block_2: 11 mm off slot (+10, -5)");

  // The next block starts empty and the last one is summed up.
  state.taskStatus("plan=wall1 task=4/7 block=block_3 approach_x=7.0 approach_y=2.0");
  EXPECT_TRUE(state.pass_rows().empty());
  EXPECT_TRUE(state.placed_text().empty());
  EXPECT_EQ(
    state.last_block_text(),
    "Last block block_2: in-air miss 5 mm after 2 passes, placed 11 mm off slot (+10, -5)");
}

TEST(CBSRunState, the_pickup_refine_of_the_same_block_is_not_its_placed_result)
{
  CBSRunState state;
  state.taskStatus("plan=wall1 task=3/7 block=block_2 target_x=6.0 target_y=2.0");
  state.poseStatus("mode=REFINE_BLOCK block=block_2 ok=1 frame=world x=4.1 y=-1.5 z=0.3");
  EXPECT_TRUE(state.placed_text().empty());
}

TEST(CBSRunState, the_placement_light_agrees_with_the_auto_loops_gate)
{
  using Light = CBSRunState::Light;
  CBSRunState state;
  state.operatorState("waiting:measure|place");
  EXPECT_EQ(state.placement_light(), Light::Grey);
  EXPECT_FALSE(state.correct_available());

  // 2.3 cm: the old 4 cm tolerance read this as within; the auto loop would move.
  state.poseStatus("mode=REFINE_GRASPED block=block_2 ok=1");
  state.placementResidual(
    "Placement residual (FK XY+yaw, block_2): dxyz=[0.020, -0.011, 0.000] m, |d|=0.023 m, "
    "dyaw=0.1 deg; correction recommended");
  state.placementError(0.023);
  EXPECT_EQ(state.placement_light(), Light::Yellow);
  EXPECT_TRUE(state.correct_available());
  EXPECT_EQ(
    state.placement_text(),
    "Residual 23 mm · lidar, over 1 cm: Correct moves hover by (+20, -11) mm (K0)");

  // After the correction the old number is stale until measured again.
  state.operatorState("waiting:measure|place");
  EXPECT_EQ(state.placement_light(), Light::Grey);

  state.poseStatus("mode=REFINE_GRASPED block=block_2 ok=1");
  state.placementResidual("dxyz=[0.004, 0.002, 0.000] m");
  state.placementError(0.0045);
  EXPECT_EQ(state.placement_light(), Light::Green);
  EXPECT_FALSE(state.correct_available());

  // The lidar missed: the number is FK's, and Correct would move on it.
  state.poseStatus("mode=REFINE_GRASPED block=block_2 ok=0");
  state.placementResidual("dxyz=[0.020, -0.011, 0.000] m");
  state.placementError(0.023);
  EXPECT_EQ(state.placement_light(), Light::Red);
  EXPECT_FALSE(state.correct_available());

  // Beyond what EvaluatePlacementResidual would apply unclamped.
  state.poseStatus("mode=REFINE_GRASPED block=block_2 ok=1");
  state.placementResidual("dxyz=[0.400, 0.000, 0.000] m");
  state.placementError(0.40);
  EXPECT_EQ(state.placement_light(), Light::Red);
}

TEST(CBSRunState, the_activity_line_says_why_the_crane_is_not_moving)
{
  CBSRunState state;
  state.startRequested("wall_assembly.xml");
  state.goalAccepted(true);

  state.feedback("Plan approach", "RUNNING", 100.0);
  EXPECT_EQ(state.activity_text(104.2), "Planning the next move… 4 s");

  state.swaySettled(
    1, 0.031,
    "the measured sway rate [0.031, 0.012] rad/s is outside dq_u_settled [0.02, 0.02] on the "
    "passive pair, measured 0.010 s ago");
  state.feedback("Wait for the load to settle", "RUNNING", 110.0);
  EXPECT_EQ(
    state.activity_text(122.0),
    "Waiting for the load to settle: sway 0.031 rad/s → 0.020 rad/s (12 s)");

  state.feedback("MeasureBlockFromGripper", "RUNNING", 130.0);
  EXPECT_EQ(
    state.activity_text(130.0),
    "In-air correction: lidar measures the carried block, shifts the waypoint by the miss");

  // A node with no sentence reads as the leg it is in.
  state.feedback("SetBlackboard", "SUCCESS", 131.0);
  EXPECT_EQ(
    state.activity_text(131.0),
    "In-air correction: lidar measures the carried block, shifts the waypoint by the miss");

  // No leg either: the node name.
  CBSRunState fresh;
  fresh.feedback("SomethingNew", "RUNNING");
  EXPECT_EQ(fresh.activity_text(), "SomethingNew");

  fresh.operatorState("waiting:correct|place");
  EXPECT_EQ(
    fresh.activity_text(), "Residual measured: correct the hover by it, or place the block");
}
