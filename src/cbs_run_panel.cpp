#include "concrete_block_rviz_plugins/cbs_run_panel.hpp"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QStringList>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "yaml-cpp/yaml.h"

namespace concrete_block_rviz_plugins
{

namespace
{

constexpr const char * kPlanControlService = "/concrete_block_wall_plan_server/plan_control";
constexpr const char * kRunPoseService = "/world_model_node/run_pose_estimation";
constexpr const char * kClearWorldService = "/world_model_node/clear_world_model";
constexpr const char * kGreyLight =
  "QLabel { background-color: #808080; border: 1px solid #404040; border-radius: 9px; }";
constexpr const char * kGreenLight =
  "QLabel { background-color: #26a269; border: 1px solid #1b6f47; border-radius: 9px; }";
constexpr const char * kRedLight =
  "QLabel { background-color: #e01b24; border: 1px solid #8a1016; border-radius: 9px; }";

std::string environment_or(const char * name, const std::string & fallback)
{
  const char * from_env = std::getenv(name);
  if (from_env != nullptr && *from_env != '\0') {
    return std::string(from_env);
  }
  return fallback;
}

/// A package share, or an empty string. Never a throw out through pluginlib:
/// one missing package must not cost the whole RViz session.
std::string share_or_empty(const char * package)
{
  try {
    return ament_index_cpp::get_package_share_directory(package);
  } catch (const std::exception &) {
    return std::string();
  }
}

QString measure_label(const std::string & command)
{
  return command == "pickup_measure" ? "Check pickup" : "Measure placement residual";
}

QString correct_label(const std::string & command)
{
  return command == "pickup_correct" ? "Correct pickup" : "Apply hover correction";
}

}  // namespace

CBSRunPanel::CBSRunPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  buildUi();
}

void CBSRunPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus("RViz ROS node unavailable", true);
    return;
  }
  node_ = abstraction->get_raw_node();

  execute_client_ = rclcpp_action::create_client<ExecuteBT>(node_, "/execute_bt");
  plan_client_ = node_->create_client<PlanControl>(kPlanControlService);
  run_pose_client_ = node_->create_client<RunPoseEstimation>(kRunPoseService);
  clear_world_client_ = node_->create_client<ClearWorldModel>(kClearWorldService);
  command_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/assembly_operator/command", rclcpp::QoS(1).transient_local());

  state_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/assembly_operator/state", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::String::SharedPtr message) {
      QMetaObject::invokeMethod(
        this, [this, text = QString::fromStdString(message->data)]() {
          onOperatorState(text);
        }, Qt::QueuedConnection);
    });

  // Both refinement loops report the same three things; the step section shows
  // whichever loop the latched state is in.
  const auto subscribe_residual = [this](const char * topic, Residual * residual) {
      residual_subs_.push_back(
        node_->create_subscription<std_msgs::msg::String>(
          std::string("/assembly_operator/") + topic, rclcpp::QoS(1).transient_local(),
          [this, residual](const std_msgs::msg::String::SharedPtr message) {
            QMetaObject::invokeMethod(
              this, [this, residual, text = QString::fromStdString(message->data)]() {
                residual->text = text;
                refresh();
              }, Qt::QueuedConnection);
          }));
    };
  const auto subscribe_within = [this](const char * topic, Residual * residual) {
      residual_subs_.push_back(
        node_->create_subscription<std_msgs::msg::Bool>(
          std::string("/assembly_operator/") + topic, rclcpp::QoS(1).transient_local(),
          [this, residual](const std_msgs::msg::Bool::SharedPtr message) {
            QMetaObject::invokeMethod(
              this, [this, residual, within = message->data]() {
                residual->measured = true;
                residual->within = within;
                refresh();
              }, Qt::QueuedConnection);
          }));
    };
  const auto subscribe_error = [this](const char * topic, Residual * residual) {
      residual_subs_.push_back(
        node_->create_subscription<std_msgs::msg::Float64>(
          std::string("/assembly_operator/") + topic, rclcpp::QoS(1).transient_local(),
          [this, residual](const std_msgs::msg::Float64::SharedPtr message) {
            QMetaObject::invokeMethod(
              this, [this, residual, error_m = message->data]() {
                residual->error_m = error_m;
                refresh();
              }, Qt::QueuedConnection);
          }));
    };
  subscribe_residual("pickup_refinement_status", &pickup_);
  subscribe_within("pickup_residual_within_indicator", &pickup_);
  subscribe_error("pickup_translation_error_m", &pickup_);
  subscribe_residual("refinement_status", &placement_);
  subscribe_within("placement_residual_within_indicator", &placement_);
  subscribe_error("placement_translation_error_m", &placement_);

  // The run's own numbers, published next to the log lines that already carried them.
  subscribeStatus("/cbs/status/task", &CBSRunState::taskStatus);
  subscribeStatus("/cbs/status/plan", &CBSRunState::planStatus);
  subscribeStatus("/cbs/status/inair", &CBSRunState::inAirStatus);
  subscribeStatus("/cbs/status/load_off", &CBSRunState::loadOffStatus);
  // Volatile depth 1, matching crane_mpc's publishers: these report, they do not latch.
  status_subs_.push_back(
    node_->create_subscription<crane_msgs::msg::SolverHealth>(
      "/crane/mpc/solver_health", rclcpp::QoS(1),
      [this](const crane_msgs::msg::SolverHealth::SharedPtr message) {
        QMetaObject::invokeMethod(
          this, [this, outcome = message->outcome, solve = message->solve_time]() {
            state_.solverHealth(outcome, solve);
            refresh();
          }, Qt::QueuedConnection);
      }));
  status_subs_.push_back(
    node_->create_subscription<crane_msgs::msg::SwaySettled>(
      "/crane/sway_settled", rclcpp::QoS(1),
      [this](const crane_msgs::msg::SwaySettled::SharedPtr message) {
        QMetaObject::invokeMethod(
          this, [this, settled = message->settled]() {
            state_.swaySettled(settled);
            refresh();
          }, Qt::QueuedConnection);
      }));

  const std::string catalog = environment_or(
    "BEHAVIOR_TREE_PANEL_BT_CATALOG",
    share_or_empty("concrete_block_behavior_tree") + "/config/bt_panel_catalog.yaml");
  loadCatalog(catalog);

  // The wall plan server is a separate node and is normally not up yet when
  // RViz builds its panels. Poll until the service appears, then fill the plan
  // list once -- calling straight from here left a "Service unavailable" that
  // nothing ever cleared.
  setStatus("Waiting for the wall plan server…");
  connect_timer_ = new QTimer(this);
  connect(
    connect_timer_, &QTimer::timeout, this, [this]() {
      if (plan_client_ && plan_client_->service_is_ready()) {
        connect_timer_->stop();
        sendPlanCommand("reload", "", plans_file_edit_->text().trimmed().toStdString());
      }
    });
  connect_timer_->start(1000);
  refresh();
}

void CBSRunPanel::subscribeStatus(
  const char * topic, void (CBSRunState::* into)(const std::string &))
{
  status_subs_.push_back(
    node_->create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(1).transient_local(),
      [this, into](const std_msgs::msg::String::SharedPtr message) {
        QMetaObject::invokeMethod(
          this, [this, into, line = message->data]() {
            (state_.*into)(line);
            refresh();
          }, Qt::QueuedConnection);
      }));
}

void CBSRunPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString value;
  if (config.mapGetString("WallPlansFile", &value) && !value.isEmpty()) {
    plans_file_edit_->setText(value);
  }
}

void CBSRunPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("WallPlansFile", plans_file_edit_->text());
}

void CBSRunPanel::buildUi()
{
  auto * root = new QVBoxLayout(this);

  // ── 1. status header ──────────────────────────────────────────────────
  auto * header = new QGroupBox("Run", this);
  auto * header_layout = new QVBoxLayout(header);
  header_label_ = new QLabel("No task served yet", header);
  header_label_->setWordWrap(true);
  header_label_->setStyleSheet("QLabel { font-weight: bold; }");
  run_label_ = new QLabel("Run: idle", header);
  run_label_->setWordWrap(true);
  // Two phase lines: the coarse bar an operator reads at a glance, and the BT node
  // behind it, which is what a log or a bug report needs.
  phase_bar_label_ = new QLabel(header);
  phase_bar_label_->setWordWrap(true);
  phase_label_ = new QLabel("Phase: idle", header);
  phase_label_->setWordWrap(true);
  auto * header_buttons = new QHBoxLayout();
  stop_button_ = new QPushButton("Stop", header);
  retry_button_ = new QPushButton("Retry block", header);
  header_buttons->addWidget(stop_button_);
  header_buttons->addWidget(retry_button_);
  header_layout->addWidget(header_label_);
  header_layout->addWidget(run_label_);
  header_layout->addWidget(phase_bar_label_);
  header_layout->addWidget(phase_label_);
  header_layout->addLayout(header_buttons);

  // ── 2. prompt banner ──────────────────────────────────────────────────
  prompt_banner_ = new QLabel(this);
  prompt_banner_->setWordWrap(true);
  prompt_banner_->setAlignment(Qt::AlignCenter);
  prompt_banner_->setStyleSheet(
    "QLabel { background-color: #f5c211; color: #241f31; font-weight: bold; padding: 6px; }");
  prompt_banner_->setVisible(false);

  // ── 2b. what the run reports ──────────────────────────────────────────
  auto * reports = new QGroupBox("Status", this);
  auto * reports_layout = new QVBoxLayout(reports);
  planner_label_ = new QLabel("Planner: no request answered yet", reports);
  planner_label_->setWordWrap(true);
  in_air_label_ = new QLabel("In air: not measured", reports);
  in_air_label_->setWordWrap(true);
  touchdown_bar_ = new QProgressBar(reports);
  touchdown_bar_->setTextVisible(false);
  touchdown_label_ = new QLabel("Touchdown: not descending", reports);
  touchdown_label_->setWordWrap(true);
  mpc_label_ = new QLabel(reports);
  mpc_label_->setWordWrap(true);
  reports_layout->addWidget(planner_label_);
  reports_layout->addWidget(in_air_label_);
  reports_layout->addWidget(touchdown_bar_);
  reports_layout->addWidget(touchdown_label_);
  reports_layout->addWidget(mpc_label_);

  // ── 3. setup ──────────────────────────────────────────────────────────
  auto * setup = new QGroupBox("Setup", this);
  auto * setup_layout = new QVBoxLayout(setup);
  auto * setup_form = new QFormLayout();
  // Package share, not a workspace path: the two differ devcontainer vs native.
  const std::string plans_share = share_or_empty("concrete_block_assembly_planning");
  plans_file_edit_ = new QLineEdit(
    QString::fromStdString(plans_share.empty() ? "" : plans_share + "/config/wall_plans.yaml"),
    setup);
  plan_combo_ = new QComboBox(setup);
  setup_form->addRow("Plans YAML", plans_file_edit_);
  setup_form->addRow("Plan", plan_combo_);
  auto * plan_buttons = new QHBoxLayout();
  reload_button_ = new QPushButton("Reload", setup);
  activate_button_ = new QPushButton("Activate", setup);
  clear_plan_button_ = new QPushButton("Clear", setup);
  plan_buttons->addWidget(reload_button_);
  plan_buttons->addWidget(activate_button_);
  plan_buttons->addWidget(clear_plan_button_);
  auto * world_buttons = new QHBoxLayout();
  scene_discovery_button_ = new QPushButton("Scene discovery", setup);
  clear_world_button_ = new QPushButton("Clear world model", setup);
  world_buttons->addWidget(scene_discovery_button_);
  world_buttons->addWidget(clear_world_button_);
  setup_layout->addLayout(setup_form);
  setup_layout->addLayout(plan_buttons);
  setup_layout->addLayout(world_buttons);

  // ── 4. run ────────────────────────────────────────────────────────────
  tree_box_ = new QGroupBox("Behavior tree", this);
  tree_layout_ = new QVBoxLayout(tree_box_);
  approve_every_move_check_ = new QCheckBox("Approve every move (fresh press)", tree_box_);
  // Today's unchecked "Full-Autonomy": SEMI_AUTONOMOUS, so every approval node
  // wants a rising edge. Unchecking this holds the deadman instead.
  approve_every_move_check_->setChecked(true);
  start_button_ = new QPushButton("Start", tree_box_);
  tree_layout_->addWidget(approve_every_move_check_);
  tree_layout_->addWidget(start_button_);

  // ── 5. step by step ───────────────────────────────────────────────────
  step_box_ = new QGroupBox("Operator step", this);
  auto * step_layout = new QVBoxLayout(step_box_);
  continue_button_ = new QPushButton("Continue", step_box_);
  auto * refine_buttons = new QHBoxLayout();
  measure_button_ = new QPushButton("Measure", step_box_);
  correct_button_ = new QPushButton("Correct", step_box_);
  refine_buttons->addWidget(measure_button_);
  refine_buttons->addWidget(correct_button_);
  auto * light_row = new QHBoxLayout();
  residual_light_ = new QLabel(step_box_);
  residual_light_->setFixedSize(18, 18);
  residual_light_->setStyleSheet(kGreyLight);
  residual_label_ = new QLabel("Residual: not measured", step_box_);
  light_row->addWidget(residual_light_);
  light_row->addWidget(residual_label_);
  light_row->addStretch(1);
  refinement_label_ = new QLabel("", step_box_);
  refinement_label_->setWordWrap(true);
  step_layout->addWidget(continue_button_);
  step_layout->addLayout(refine_buttons);
  step_layout->addLayout(light_row);
  step_layout->addWidget(refinement_label_);
  step_box_->setVisible(false);

  status_label_ = new QLabel("Not initialized", this);
  status_label_->setWordWrap(true);
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);

  root->addWidget(header);
  root->addWidget(prompt_banner_);
  root->addWidget(reports);
  root->addWidget(setup);
  root->addWidget(tree_box_);
  root->addWidget(step_box_);
  root->addWidget(status_label_);
  root->addStretch(1);

  connect(stop_button_, &QPushButton::clicked, this, [this]() {stopRun();});
  connect(retry_button_, &QPushButton::clicked, this, [this]() {retryBlock();});
  connect(
    reload_button_, &QPushButton::clicked, this, [this]() {
      sendPlanCommand("reload", "", plans_file_edit_->text().trimmed().toStdString());
    });
  connect(
    activate_button_, &QPushButton::clicked, this, [this]() {
      sendPlanCommand("activate", plan_combo_->currentText().trimmed().toStdString());
    });
  connect(
    clear_plan_button_, &QPushButton::clicked, this, [this]() {sendPlanCommand("clear", "");});
  connect(
    scene_discovery_button_, &QPushButton::clicked, this, [this]() {runSceneDiscovery();});
  connect(clear_world_button_, &QPushButton::clicked, this, [this]() {clearWorldModel();});
  connect(start_button_, &QPushButton::clicked, this, [this]() {startRun();});
  connect(
    continue_button_, &QPushButton::clicked, this, [this]() {
      sendCommand(state_.continue_command());
    });
  connect(
    measure_button_, &QPushButton::clicked, this, [this]() {
      sendCommand(state_.measure_command());
    });
  connect(
    correct_button_, &QPushButton::clicked, this, [this]() {
      sendCommand(state_.correct_command());
    });
}

void CBSRunPanel::loadCatalog(const std::string & catalog_file)
{
  tree_radios_.clear();
  try {
    const YAML::Node config = YAML::LoadFile(catalog_file);
    std::string catalog_share = share_or_empty("concrete_block_behavior_tree");
    if (config["package"]) {
      catalog_share = share_or_empty(config["package"].as<std::string>().c_str());
    }
    for (const auto & tree : config["behavior_trees"]) {
      const auto name = QString::fromStdString(tree["name"].as<std::string>());
      const auto path = QString::fromStdString(catalog_share + tree["path"].as<std::string>());
      auto * radio = new QRadioButton(name, tree_box_);
      tree_layout_->insertWidget(static_cast<int>(tree_radios_.size()), radio);
      tree_radios_.emplace_back(radio, path);
      connect(radio, &QRadioButton::toggled, this, [this]() {refresh();});
    }
  } catch (const std::exception & e) {
    setStatus(
      QString("Could not read the BT catalog %1: %2")
      .arg(QString::fromStdString(catalog_file), QString::fromStdString(e.what())), true);
    return;
  }
  if (tree_radios_.empty()) {
    setStatus(
      QString("No behavior tree in %1").arg(QString::fromStdString(catalog_file)), true);
    return;
  }
  // The field run, preselected; any other tree is one click away.
  auto * preselect = tree_radios_.front().first;
  for (const auto & entry : tree_radios_) {
    if (entry.first->text().contains("Run plan (all blocks)")) {
      preselect = entry.first;
    }
  }
  preselect->setChecked(true);
}

void CBSRunPanel::startRun()
{
  active_tree_path_.clear();
  for (const auto & entry : tree_radios_) {
    if (entry.first->isChecked()) {
      active_tree_path_ = entry.second.toStdString();
    }
  }
  if (active_tree_path_.empty()) {
    setStatus("No behavior tree selected", true);
    return;
  }
  retry_pending_ = false;
  state_.startRequested(std::filesystem::path(active_tree_path_).filename().string());
  refresh();
  sendGoal();
}

void CBSRunPanel::sendGoal()
{
  if (!execute_client_ || !execute_client_->wait_for_action_server(std::chrono::seconds(2))) {
    setStatus("Action unavailable: /execute_bt", true);
    state_.goalAccepted(false);
    refresh();
    return;
  }
  ExecuteBT::Goal goal;
  goal.behavior_tree_path = active_tree_path_;
  // Checked means every approval node waits for a rising edge
  // (lsrl_behavior_tree wait_msg_condition.hpp:129), i.e. a fresh press per move.
  goal.autonomy_mode = approve_every_move_check_->isChecked() ?
    ExecuteBT::Goal::SEMI_AUTONOMOUS : ExecuteBT::Goal::AUTONOMOUS;

  rclcpp_action::Client<ExecuteBT>::SendGoalOptions options;
  options.goal_response_callback = [this](GoalHandleExecuteBT::SharedPtr goal_handle) {
      QMetaObject::invokeMethod(
        this, [this, accepted = static_cast<bool>(goal_handle)]() {
          state_.goalAccepted(accepted);
          if (!accepted) {
            setStatus("Goal rejected by /execute_bt", true);
          }
          refresh();
        }, Qt::QueuedConnection);
    };
  options.feedback_callback =
    [this](GoalHandleExecuteBT::SharedPtr,
      const std::shared_ptr<const ExecuteBT::Feedback> feedback) {
      QMetaObject::invokeMethod(
        this,
        [this, node_name = feedback->last_change.node_name,
          current_status = feedback->last_change.current_status]() {
          state_.feedback(node_name, current_status);
          refresh();
        }, Qt::QueuedConnection);
    };
  options.result_callback = [this](const GoalHandleExecuteBT::WrappedResult & result) {
      QMetaObject::invokeMethod(
        this, [this, code = result.code]() {
          const bool succeeded = code == rclcpp_action::ResultCode::SUCCEEDED;
          state_.goalFinished(succeeded);
          command_in_flight_ = false;
          const bool retry = retry_pending_;
          retry_pending_ = false;
          refresh();
          // Only on CANCELED: the cancel can lose a race against a goal that
          // ended by itself, and that must not restart the crane unasked. The
          // server is free only now that the cancelled goal has terminated.
          if (retry && code == rclcpp_action::ResultCode::CANCELED) {
            state_.startRequested(std::filesystem::path(active_tree_path_).filename().string());
            refresh();
            sendGoal();
            return;
          }
          setStatus(succeeded ? "Run complete" : "Run stopped", !succeeded);
        }, Qt::QueuedConnection);
    };
  execute_client_->async_send_goal(goal, options);
  setStatus(QString("Starting %1").arg(QString::fromStdString(state_.tree_name())));
}

void CBSRunPanel::stopRun()
{
  if (!execute_client_) {
    setStatus("RViz ROS node unavailable", true);
    return;
  }
  retry_pending_ = false;
  execute_client_->async_cancel_all_goals();
  setStatus("Cancel requested");
}

void CBSRunPanel::retryBlock()
{
  if (!execute_client_ || active_tree_path_.empty()) {
    setStatus("Nothing to retry", true);
    return;
  }
  // wall_plan_server re-serves the outstanding task, so the same tree picks the
  // same block up again (issue 268).
  retry_pending_ = true;
  execute_client_->async_cancel_all_goals();
  setStatus("Retrying the outstanding block");
}

void CBSRunPanel::sendCommand(const std::string & command)
{
  if (!command_pub_) {
    setStatus("RViz ROS node unavailable", true);
    return;
  }
  if (command.empty()) {
    setStatus("No operator command is awaited", true);
    return;
  }
  std_msgs::msg::String message;
  message.data = command;
  command_pub_->publish(message);
  // The wait node is edge-triggered on the command sequence; hold the buttons
  // until the latched state actually moves on, so one click is one step.
  command_in_flight_ = true;
  refresh();
  setStatus(QString("Sent command: %1").arg(QString::fromStdString(command)));
}

void CBSRunPanel::sendPlanCommand(
  const std::string & command, const std::string & plan_name, const std::string & plans_file)
{
  if (!plan_client_ || !plan_client_->service_is_ready()) {
    setStatus(QString("Service unavailable: %1").arg(kPlanControlService), true);
    return;
  }
  if (command == "activate" && plan_name.empty()) {
    setStatus("No plan selected", true);
    return;
  }
  auto request = std::make_shared<PlanControl::Request>();
  request->command = command;
  request->wall_plan_name = plan_name;
  request->wall_plans_file = plans_file;

  setBusy(true);
  setStatus(QString::fromStdString("Sending '" + command + "'…"));
  plan_client_->async_send_request(
    request,
    [this, command](rclcpp::Client<PlanControl>::SharedFuture future) {
      const auto response = future.get();
      QStringList plans;
      for (const auto & name : response->plan_names) {
        plans << QString::fromStdString(name);
      }
      QMetaObject::invokeMethod(
        this,
        [this, command, plans, ok = response->success,
          active = QString::fromStdString(response->active_plan),
          loaded_file = QString::fromStdString(response->wall_plans_file),
          message = QString::fromStdString(response->message)]() {
          setBusy(false);
          if (command == "reload" && ok) {
            const QString keep = plan_combo_->currentText();
            plan_combo_->clear();
            plan_combo_->addItems(plans);
            int index = plan_combo_->findText(keep);
            if (index < 0 && !active.isEmpty()) {
              index = plan_combo_->findText(active);
            }
            if (index >= 0) {
              plan_combo_->setCurrentIndex(index);
            }
            if (!loaded_file.isEmpty()) {
              plans_file_edit_->setText(loaded_file);
            }
          }
          setStatus(message, !ok);
        }, Qt::QueuedConnection);
    });
}

void CBSRunPanel::runSceneDiscovery()
{
  if (!run_pose_client_ || !run_pose_client_->service_is_ready()) {
    setStatus(QString("Service unavailable: %1").arg(kRunPoseService), true);
    return;
  }
  auto request = std::make_shared<RunPoseEstimation::Request>();
  request->mode = "SCENE_DISCOVERY";
  request->enable_debug = true;
  request->timeout_s = 8.0F;

  setBusy(true);
  setStatus("Scene discovery running (no crane motion is commanded)");
  run_pose_client_->async_send_request(
    request,
    [this](rclcpp::Client<RunPoseEstimation>::SharedFuture future) {
      const auto response = future.get();
      QMetaObject::invokeMethod(
        this,
        [this, ok = response->success, message = QString::fromStdString(response->message)]() {
          setBusy(false);
          setStatus(
            QString(ok ? "Scene discovery: " : "Scene discovery failed: ") + message, !ok);
        }, Qt::QueuedConnection);
    });
}

void CBSRunPanel::clearWorldModel()
{
  if (!clear_world_client_ || !clear_world_client_->service_is_ready()) {
    setStatus(QString("Service unavailable: %1").arg(kClearWorldService), true);
    return;
  }
  setBusy(true);
  setStatus("Clearing the world model");
  clear_world_client_->async_send_request(
    std::make_shared<ClearWorldModel::Request>(),
    [this](rclcpp::Client<ClearWorldModel>::SharedFuture future) {
      const auto response = future.get();
      QMetaObject::invokeMethod(
        this,
        [this, ok = response->success, cleared = response->cleared_count,
          message = QString::fromStdString(response->message)]() {
          setBusy(false);
          setStatus(
            QString("Clear world model: cleared=%1 %2").arg(cleared).arg(message), !ok);
        }, Qt::QueuedConnection);
    });
}

void CBSRunPanel::onOperatorState(const QString & latched)
{
  // The wait node republishes its state every tick; only a real change is news.
  if (latched == last_state_text_) {
    return;
  }
  last_state_text_ = latched;
  command_in_flight_ = false;
  state_.operatorState(latched.toStdString());
  refresh();
}

void CBSRunPanel::refresh()
{
  const QString tree = QString::fromStdString(state_.tree_name());
  header_label_->setText(QString::fromStdString(state_.header_text()));
  run_label_->setText(
    QString("Run: %1%2").arg(QString::fromStdString(state_.run_text()))
    .arg(tree.isEmpty() ? QString() : " — " + tree));
  phase_bar_label_->setText(QString::fromStdString(state_.phase_bar()));
  phase_label_->setText(QString::fromStdString(state_.phase_text()));
  const QString prompt = QString::fromStdString(state_.prompt());
  prompt_banner_->setText(prompt);
  prompt_banner_->setVisible(!prompt.isEmpty());
  stop_button_->setEnabled(state_.stop_available());
  retry_button_->setEnabled(state_.retry_available());

  planner_label_->setText(QString::fromStdString(state_.planner_text()));
  in_air_label_->setText(QString::fromStdString(state_.in_air_text()));
  touchdown_label_->setText(QString::fromStdString(state_.touchdown_text()));
  // Full bar is the release gate, not 100 % of m g r: that is the number the tree acts on.
  const int gate = static_cast<int>(state_.unload_gate() * 100.0);
  touchdown_bar_->setVisible(state_.touchdown_visible());
  touchdown_bar_->setMaximum(gate > 0 ? gate : 100);
  touchdown_bar_->setValue(static_cast<int>(state_.unload_fraction() * 100.0));
  mpc_label_->setText(QString::fromStdString(state_.mpc_text()));

  bool tree_selected = false;
  for (const auto & entry : tree_radios_) {
    tree_selected = tree_selected || entry.first->isChecked();
  }
  const bool run_busy = state_.run() == CBSRunState::Run::Running ||
    state_.run() == CBSRunState::Run::Starting;
  start_button_->setEnabled(tree_selected && !run_busy);
  for (const auto & entry : tree_radios_) {
    entry.first->setEnabled(!run_busy);
  }
  approve_every_move_check_->setEnabled(!run_busy);

  step_box_->setVisible(state_.step_section_visible());
  const auto & measure = state_.measure_command();
  const auto & correct = state_.correct_command();
  continue_button_->setText(QString::fromStdString(state_.continue_label()));
  continue_button_->setVisible(!state_.continue_command().empty());
  continue_button_->setEnabled(!command_in_flight_);
  measure_button_->setText(measure_label(measure));
  measure_button_->setVisible(!measure.empty());
  measure_button_->setEnabled(!command_in_flight_);
  correct_button_->setText(correct_label(correct));
  correct_button_->setVisible(!correct.empty());
  correct_button_->setEnabled(!command_in_flight_);

  // Which loop the latched state is in decides which residual is on show.
  const bool in_pickup = measure == "pickup_measure" || correct == "pickup_correct";
  const bool refining = !measure.empty() || !correct.empty();
  const Residual & residual = in_pickup ? pickup_ : placement_;
  residual_light_->setVisible(refining);
  residual_label_->setVisible(refining);
  refinement_label_->setVisible(refining);
  refinement_label_->setText(residual.text);
  if (residual.measured) {
    residual_light_->setStyleSheet(residual.within ? kGreenLight : kRedLight);
    residual_label_->setText(
      QString("Residual: %1 cm%2").arg(residual.error_m * 100.0, 0, 'f', 1)
      .arg(residual.within ? " (within)" : " (correction recommended)"));
  } else {
    residual_light_->setStyleSheet(kGreyLight);
    residual_label_->setText("Residual: not measured");
  }
}

void CBSRunPanel::setBusy(bool busy)
{
  busy_ = busy;
  reload_button_->setEnabled(!busy_);
  activate_button_->setEnabled(!busy_);
  clear_plan_button_->setEnabled(!busy_);
  scene_discovery_button_->setEnabled(!busy_);
  clear_world_button_->setEnabled(!busy_);
}

void CBSRunPanel::setStatus(const QString & text, bool error)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(error ? "QLabel { color: #b00020; }" : "");
}

}  // namespace concrete_block_rviz_plugins

PLUGINLIB_EXPORT_CLASS(concrete_block_rviz_plugins::CBSRunPanel, rviz_common::Panel)
