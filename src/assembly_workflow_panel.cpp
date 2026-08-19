#include "concrete_block_rviz_plugins/assembly_workflow_panel.hpp"

#include <chrono>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QVBoxLayout>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace concrete_block_rviz_plugins
{

AssemblyWorkflowPanel::AssemblyWorkflowPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  buildUi();
}

void AssemblyWorkflowPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus("RViz ROS node unavailable", true);
    return;
  }
  node_ = abstraction->get_raw_node();
  execute_client_ = rclcpp_action::create_client<ExecuteBT>(node_, "/execute_bt");
  run_pose_client_ = node_->create_client<RunPoseEstimation>(
    "/world_model_node/run_pose_estimation");
  command_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/assembly_operator/command", rclcpp::QoS(1).transient_local());
  state_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/assembly_operator/state", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::String::SharedPtr message) {
      QMetaObject::invokeMethod(this, [this, state = QString::fromStdString(message->data)]() {
        state_label_->setText(state);
        if (state.startsWith("waiting:")) {
          setExpectedCommand(state.mid(QString("waiting:").size()));
          setStatus("Ready for " + expected_command_);
        }
      }, Qt::QueuedConnection);
    });
  refinement_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/assembly_operator/refinement_status", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::String::SharedPtr message) {
      QMetaObject::invokeMethod(
        this, [this, text = QString::fromStdString(message->data)]() {
          refinement_label_->setText(text);
        }, Qt::QueuedConnection);
    });
  residual_indicator_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/assembly_operator/placement_residual_within_indicator", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::Bool::SharedPtr message) {
      QMetaObject::invokeMethod(this, [this, within = message->data]() {
        residual_indicator_measured_ = true;
        residual_within_indicator_threshold_ = within;
        setResidualLight(within, true);
      }, Qt::QueuedConnection);
    });
  residual_indicator_threshold_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/assembly_operator/placement_indicator_threshold_m", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::Float64::SharedPtr message) {
      QMetaObject::invokeMethod(this, [this, threshold = message->data]() {
        residual_indicator_threshold_m_ = threshold;
        setResidualLight(residual_within_indicator_threshold_, residual_indicator_measured_);
      }, Qt::QueuedConnection);
    });
  residual_translation_error_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/assembly_operator/placement_translation_error_m", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::Float64::SharedPtr message) {
      QMetaObject::invokeMethod(this, [this, error_m = message->data]() {
        residual_translation_error_m_ = error_m;
        setResidualLight(residual_within_indicator_threshold_, residual_indicator_measured_);
      }, Qt::QueuedConnection);
    });
  pickup_refinement_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/assembly_operator/pickup_refinement_status", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::String::SharedPtr message) {
      QMetaObject::invokeMethod(this, [this, text = QString::fromStdString(message->data)]() {
        pickup_refinement_label_->setText(text);
      }, Qt::QueuedConnection);
    });
  pickup_residual_indicator_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/assembly_operator/pickup_residual_within_indicator", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::Bool::SharedPtr message) {
      QMetaObject::invokeMethod(this, [this, within = message->data]() {
        pickup_residual_measured_ = true;
        pickup_residual_within_tolerance_ = within;
        setPickupResidualLight(within, true);
      }, Qt::QueuedConnection);
    });
  pickup_translation_error_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/assembly_operator/pickup_translation_error_m", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::Float64::SharedPtr message) {
      QMetaObject::invokeMethod(this, [this, error_m = message->data]() {
        pickup_translation_error_m_ = error_m;
        setPickupResidualLight(pickup_residual_within_tolerance_, pickup_residual_measured_);
      }, Qt::QueuedConnection);
    });
  setStatus("Execute the next step of the active plan");
}

void AssemblyWorkflowPanel::buildUi()
{
  auto * layout = new QVBoxLayout(this);
  auto * execution = new QGroupBox("Operator-guided assembly", this);
  auto * buttons = new QVBoxLayout(execution);
  start_button_ = new QPushButton("Execute next plan step", execution);
  acquire_button_ = new QPushButton("1. Acquire next task", execution);
  pickup_hover_button_ = new QPushButton("2. Move to pickup hover", execution);
  pickup_measure_button_ = new QPushButton("3. Measure pickup", execution);
  pickup_correct_button_ = new QPushButton("Correct pickup", execution);
  pick_button_ = new QPushButton("4. Grasp block", execution);
  hover_button_ = new QPushButton("5. Move to placement hover", execution);
  measure_button_ = new QPushButton("6. Measure placement residual", execution);
  correct_button_ = new QPushButton("Apply hover correction", execution);
  refresh_scene_button_ = new QPushButton("Refresh LiDAR scene estimate (optional)", execution);
  place_button_ = new QPushButton("5. Place block", execution);
  for (auto * button : {
      start_button_, acquire_button_, pickup_hover_button_, pickup_measure_button_,
      pickup_correct_button_, pick_button_, hover_button_, measure_button_,
      correct_button_, refresh_scene_button_, place_button_}) {
    buttons->addWidget(button);
  }
  state_label_ = new QLabel("No active session", this);
  refinement_label_ = new QLabel("Placement residual: not measured", this);
  refinement_label_->setWordWrap(true);
  pickup_refinement_label_ = new QLabel("Pickup residual: not measured", this);
  pickup_refinement_label_->setWordWrap(true);
  auto * residual_row = new QHBoxLayout();
  residual_light_ = new QLabel(this);
  residual_light_->setFixedSize(18, 18);
  residual_light_label_ = new QLabel("Residual: not measured", this);
  residual_row->addWidget(residual_light_);
  residual_row->addWidget(residual_light_label_);
  residual_row->addStretch(1);
  setResidualLight(false, false);
  auto * pickup_residual_row = new QHBoxLayout();
  pickup_residual_light_ = new QLabel(this);
  pickup_residual_light_->setFixedSize(18, 18);
  pickup_residual_light_label_ = new QLabel("Pickup residual: not measured", this);
  pickup_residual_row->addWidget(pickup_residual_light_);
  pickup_residual_row->addWidget(pickup_residual_light_label_);
  pickup_residual_row->addStretch(1);
  setPickupResidualLight(false, false);
  auto * plan_hint = new QLabel(
    "Plan: choose it in Plan Control and press Load before acquiring a task.", this);
  plan_hint->setWordWrap(true);
  execution_label_ = new QLabel("Execution: idle", this);
  execution_label_->setWordWrap(true);
  status_label_ = new QLabel("Not initialized", this);
  status_label_->setWordWrap(true);
  layout->addWidget(execution);
  layout->addWidget(plan_hint);
  layout->addWidget(state_label_);
  layout->addWidget(pickup_refinement_label_);
  layout->addLayout(pickup_residual_row);
  layout->addWidget(refinement_label_);
  layout->addLayout(residual_row);
  layout->addWidget(execution_label_);
  layout->addWidget(status_label_);
  layout->addStretch(1);
  connect(start_button_, &QPushButton::clicked, this, [this]() {startSession();});
  connect(acquire_button_, &QPushButton::clicked, this, [this]() {sendCommand("acquire");});
  connect(pickup_hover_button_, &QPushButton::clicked, this, [this]() {sendCommand("pickup_hover");});
  connect(pickup_measure_button_, &QPushButton::clicked, this, [this]() {sendCommand("pickup_measure");});
  connect(pickup_correct_button_, &QPushButton::clicked, this, [this]() {sendCommand("pickup_correct");});
  connect(pick_button_, &QPushButton::clicked, this, [this]() {sendCommand("pick");});
  connect(hover_button_, &QPushButton::clicked, this, [this]() {sendCommand("hover");});
  connect(measure_button_, &QPushButton::clicked, this, [this]() {sendCommand("measure");});
  connect(correct_button_, &QPushButton::clicked, this, [this]() {sendCommand("correct");});
  connect(refresh_scene_button_, &QPushButton::clicked, this, [this]() {refreshSceneEstimate();});
  connect(place_button_, &QPushButton::clicked, this, [this]() {sendCommand("place");});
  updateButtonEnablement();
}

void AssemblyWorkflowPanel::startSession()
{
  if (!execute_client_ || !execute_client_->wait_for_action_server(std::chrono::seconds(2))) {
    setStatus("Action unavailable: /execute_bt", true);
    return;
  }
  ExecuteBT::Goal goal;
  goal.behavior_tree_path = ament_index_cpp::get_package_share_directory(
    "concrete_block_behavior_tree") + "/behavior_trees/operator_guided_single_block.xml";
  // A stage button authorizes progression through the operator workflow; it
  // never replaces the physical approval/deadman gate immediately before a
  // crane trajectory.
  goal.autonomy_mode = ExecuteBT::Goal::SEMI_AUTONOMOUS;
  rclcpp_action::Client<ExecuteBT>::SendGoalOptions options;
  options.goal_response_callback = [this](GoalHandleExecuteBT::SharedPtr goal_handle) {
      QMetaObject::invokeMethod(this, [this, accepted = static_cast<bool>(goal_handle)]() {
        if (!accepted) {
          session_active_ = false;
          setStatus("Session rejected by /execute_bt", true);
          updateButtonEnablement();
          return;
        }
        setStatus("Session started; wait for the Acquire next task state");
      }, Qt::QueuedConnection);
    };
  options.feedback_callback =
    [this](GoalHandleExecuteBT::SharedPtr,
      const std::shared_ptr<const ExecuteBT::Feedback> feedback) {
      const auto & change = feedback->last_change;
      QMetaObject::invokeMethod(
        this,
        [this,
          node_name = QString::fromStdString(change.node_name),
          current_status = QString::fromStdString(change.current_status)]() {
          execution_label_->setText(
            current_status == "IDLE" ? "Execution: idle" :
            "Execution: " + node_name + ": " + current_status);
        },
        Qt::QueuedConnection);
    };
  options.result_callback = [this](const GoalHandleExecuteBT::WrappedResult & result) {
      QMetaObject::invokeMethod(this, [this, code = result.code]() {
        session_active_ = false;
        setExpectedCommand("");
        execution_label_->setText(
          code == rclcpp_action::ResultCode::SUCCEEDED ?
          "Execution: complete" : "Execution: stopped");
        setStatus(code == rclcpp_action::ResultCode::SUCCEEDED ? "Task complete" : "Task stopped", code != rclcpp_action::ResultCode::SUCCEEDED);
      }, Qt::QueuedConnection);
    };
  session_active_ = true;
  setExpectedCommand("");
  refinement_label_->setText("Placement residual: not measured");
  pickup_refinement_label_->setText("Pickup residual: not measured");
  residual_indicator_measured_ = false;
  residual_translation_error_m_ = 0.0;
  setResidualLight(false, false);
  pickup_residual_measured_ = false;
  pickup_translation_error_m_ = 0.0;
  setPickupResidualLight(false, false);
  execution_label_->setText("Execution: starting");
  execute_client_->async_send_goal(goal, options);
  setStatus("Starting session");
}

void AssemblyWorkflowPanel::sendCommand(const std::string & command)
{
  if (!command_pub_) {
    setStatus("RViz ROS node unavailable", true);
    return;
  }
  if (!session_active_ || !expected_commands_.contains(QString::fromStdString(command))) {
    setStatus(
      expected_command_.isEmpty() ? "Wait for the next workflow stage" :
      "Current stage is " + expected_command_, true);
    return;
  }
  std_msgs::msg::String message;
  message.data = command;
  command_pub_->publish(message);
  setExpectedCommand("");
  setStatus("Sent command: " + QString::fromStdString(command));
}

void AssemblyWorkflowPanel::refreshSceneEstimate()
{
  if (!session_active_ || !expected_commands_.contains("place")) {
    setStatus("Enter the placement-refinement stage before refreshing the scene", true);
    return;
  }
  if (!run_pose_client_ ||
    !run_pose_client_->wait_for_service(std::chrono::seconds(2)))
  {
    setStatus("Service unavailable: /world_model_node/run_pose_estimation", true);
    return;
  }

  auto request = std::make_shared<RunPoseEstimation::Request>();
  request->mode = "SCENE_DISCOVERY";
  request->enable_debug = true;
  request->timeout_s = 8.0F;
  refresh_in_flight_ = true;
  updateButtonEnablement();
  setStatus("Refreshing LiDAR scene estimate (no crane correction is commanded)");
  run_pose_client_->async_send_request(
    request,
    [this](rclcpp::Client<RunPoseEstimation>::SharedFuture future) {
      const auto response = future.get();
      QMetaObject::invokeMethod(
        this,
        [this, success = response->success,
          message = QString::fromStdString(response->message)]() {
          refresh_in_flight_ = false;
          updateButtonEnablement();
          setStatus(
            (success ? "Scene estimate refreshed: " : "Scene estimate failed: ") + message,
            !success);
        },
        Qt::QueuedConnection);
    });
}

void AssemblyWorkflowPanel::setExpectedCommand(const QString & command)
{
  expected_command_ = command;
  expected_commands_ = command.split('|', Qt::SkipEmptyParts);
  updateButtonEnablement();
}

void AssemblyWorkflowPanel::updateButtonEnablement()
{
  start_button_->setEnabled(!session_active_);
  acquire_button_->setEnabled(session_active_ && expected_command_ == "acquire");
  pickup_hover_button_->setEnabled(session_active_ && expected_command_ == "pickup_hover");
  pickup_measure_button_->setEnabled(
    session_active_ && !refresh_in_flight_ && expected_commands_.contains("pickup_measure"));
  pickup_correct_button_->setEnabled(
    session_active_ && !refresh_in_flight_ && expected_commands_.contains("pickup_correct"));
  pick_button_->setEnabled(session_active_ && expected_commands_.contains("pick"));
  hover_button_->setEnabled(session_active_ && expected_command_ == "hover");
  measure_button_->setEnabled(
    session_active_ && !refresh_in_flight_ && expected_commands_.contains("measure"));
  correct_button_->setEnabled(
    session_active_ && !refresh_in_flight_ && expected_commands_.contains("correct"));
  const bool in_refinement = session_active_ &&
    (expected_commands_.contains("measure") || expected_commands_.contains("correct") ||
    expected_commands_.contains("place"));
  refresh_scene_button_->setEnabled(in_refinement && !refresh_in_flight_);
  place_button_->setEnabled(
    session_active_ && !refresh_in_flight_ && expected_commands_.contains("place"));
}

void AssemblyWorkflowPanel::setPickupResidualLight(bool within_tolerance, bool measured)
{
  const auto residual_cm = QString::number(pickup_translation_error_m_ * 100.0, 'f', 1);
  if (!measured) {
    pickup_residual_light_->setStyleSheet(
      "QLabel { background-color: #808080; border: 1px solid #404040; border-radius: 9px; }");
    pickup_residual_light_label_->setText("Pickup residual: not measured");
    return;
  }
  pickup_residual_light_->setStyleSheet(
    within_tolerance ?
    "QLabel { background-color: #26a269; border: 1px solid #1b6f47; border-radius: 9px; }" :
    "QLabel { background-color: #e01b24; border: 1px solid #8a1016; border-radius: 9px; }");
  pickup_residual_light_label_->setText(
    "Pickup residual: " + residual_cm + " cm" +
    (within_tolerance ? " (within)" : " (correction recommended)"));
}

void AssemblyWorkflowPanel::setStatus(const QString & text, bool error)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(error ? "QLabel { color: #b00020; }" : "");
}

void AssemblyWorkflowPanel::setResidualLight(bool within_threshold, bool measured)
{
  const auto threshold_cm = QString::number(residual_indicator_threshold_m_ * 100.0, 'f', 1);
  const auto residual_cm = QString::number(residual_translation_error_m_ * 100.0, 'f', 1);
  if (!measured) {
    residual_light_->setStyleSheet(
      "QLabel { background-color: #808080; border: 1px solid #404040; border-radius: 9px; }");
    residual_light_label_->setText("Residual: not measured (threshold: " + threshold_cm + " cm)");
    return;
  }
  residual_light_->setStyleSheet(
    within_threshold ?
    "QLabel { background-color: #26a269; border: 1px solid #1b6f47; border-radius: 9px; }" :
    "QLabel { background-color: #e01b24; border: 1px solid #8a1016; border-radius: 9px; }");
  residual_light_label_->setText(
    "Residual: " + residual_cm + " cm / " + threshold_cm + " cm" +
    (within_threshold ? " (within)" : " (above)"));
}

}  // namespace concrete_block_rviz_plugins

PLUGINLIB_EXPORT_CLASS(concrete_block_rviz_plugins::AssemblyWorkflowPanel, rviz_common::Panel)
