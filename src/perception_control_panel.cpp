#include "concrete_block_perception_rviz_plugins/perception_control_panel.hpp"

#include <chrono>

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QVBoxLayout>

#include "concrete_block_world_model_interfaces/msg/block.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace concrete_block_perception_rviz_plugins
{

PerceptionControlPanel::PerceptionControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  buildUi();
}

void PerceptionControlPanel::onInitialize()
{
  const auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    setStatus("RViz ROS node unavailable", true);
    return;
  }

  node_ = ros_node_abstraction->get_raw_node();
  ensureClients();
  setStatus("Ready");
}

void PerceptionControlPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString value;
  if (config.mapGetString("RunPoseService", &value)) {
    run_pose_service_edit_->setText(value);
  }
  if (config.mapGetString("TaskStatusService", &value)) {
    task_status_service_edit_->setText(value);
  }
  if (config.mapGetString("TargetBlockId", &value)) {
    target_block_edit_->setText(value);
  }

  float timeout = 8.0F;
  if (config.mapGetFloat("TimeoutS", &timeout)) {
    timeout_spin_->setValue(timeout);
  }

  bool checked = true;
  if (config.mapGetBool("EnableDebug", &checked)) {
    enable_debug_check_->setChecked(checked);
  }
  checked = false;
  if (config.mapGetBool("RefineGraspedUseTarget", &checked)) {
    refine_grasped_use_target_check_->setChecked(checked);
  }
}

void PerceptionControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("RunPoseService", run_pose_service_edit_->text());
  config.mapSetValue("TaskStatusService", task_status_service_edit_->text());
  config.mapSetValue("TargetBlockId", target_block_edit_->text());
  config.mapSetValue("TimeoutS", static_cast<float>(timeout_spin_->value()));
  config.mapSetValue("EnableDebug", enable_debug_check_->isChecked());
  config.mapSetValue("RefineGraspedUseTarget", refine_grasped_use_target_check_->isChecked());
}

void PerceptionControlPanel::buildUi()
{
  auto * root_layout = new QVBoxLayout(this);

  auto * services_box = new QGroupBox("Services", this);
  auto * services_layout = new QFormLayout(services_box);
  run_pose_service_edit_ = new QLineEdit(
    QString::fromStdString(run_pose_service_name_), services_box);
  task_status_service_edit_ = new QLineEdit(
    QString::fromStdString(task_status_service_name_), services_box);
  services_layout->addRow("Pose", run_pose_service_edit_);
  services_layout->addRow("Task", task_status_service_edit_);

  auto * request_box = new QGroupBox("Request", this);
  auto * request_layout = new QFormLayout(request_box);
  target_block_edit_ = new QLineEdit("wm_block_1", request_box);
  timeout_spin_ = new QDoubleSpinBox(request_box);
  timeout_spin_->setRange(0.1, 120.0);
  timeout_spin_->setSingleStep(0.5);
  timeout_spin_->setDecimals(1);
  timeout_spin_->setValue(8.0);
  timeout_spin_->setSuffix(" s");
  enable_debug_check_ = new QCheckBox(request_box);
  enable_debug_check_->setChecked(true);
  refine_grasped_use_target_check_ = new QCheckBox(request_box);
  refine_grasped_use_target_check_->setChecked(false);
  request_layout->addRow("Target block", target_block_edit_);
  request_layout->addRow("Timeout", timeout_spin_);
  request_layout->addRow("Debug", enable_debug_check_);
  request_layout->addRow("Refine grasped target", refine_grasped_use_target_check_);

  auto * buttons_box = new QGroupBox("Actions", this);
  auto * buttons_layout = new QVBoxLayout(buttons_box);
  scene_discovery_button_ = new QPushButton("Scene Discovery", buttons_box);
  refine_block_button_ = new QPushButton("Refine Block", buttons_box);
  set_task_move_button_ = new QPushButton("Set TASK_MOVE", buttons_box);
  refine_grasped_button_ = new QPushButton("Refine Grasped", buttons_box);
  buttons_layout->addWidget(scene_discovery_button_);
  buttons_layout->addWidget(refine_block_button_);
  buttons_layout->addWidget(set_task_move_button_);
  buttons_layout->addWidget(refine_grasped_button_);

  status_label_ = new QLabel("Not initialized", this);
  status_label_->setWordWrap(true);
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);

  root_layout->addWidget(services_box);
  root_layout->addWidget(request_box);
  root_layout->addWidget(buttons_box);
  root_layout->addWidget(status_label_);
  root_layout->addStretch(1);

  connect(scene_discovery_button_, &QPushButton::clicked, this, [this]() {
    sendPoseRequest("SCENE_DISCOVERY", "");
  });
  connect(refine_block_button_, &QPushButton::clicked, this, [this]() {
    sendPoseRequest("REFINE_BLOCK", target_block_edit_->text().trimmed().toStdString());
  });
  connect(set_task_move_button_, &QPushButton::clicked, this, [this]() {
    sendTaskMoveRequest();
  });
  connect(refine_grasped_button_, &QPushButton::clicked, this, [this]() {
    const std::string target = refine_grasped_use_target_check_->isChecked() ?
      target_block_edit_->text().trimmed().toStdString() : "";
    sendPoseRequest("REFINE_GRASPED", target);
  });
}

void PerceptionControlPanel::sendPoseRequest(
  const std::string & mode,
  const std::string & target_block_id)
{
  if (!node_) {
    setStatus("RViz ROS node unavailable", true);
    return;
  }

  ensureClients();
  if (!run_pose_client_ || !run_pose_client_->service_is_ready()) {
    setStatus(
      QString("Service unavailable: %1").arg(run_pose_service_edit_->text()), true);
    return;
  }

  auto request = std::make_shared<RunPoseEstimation::Request>();
  request->mode = mode;
  request->target_block_id = target_block_id;
  request->enable_debug = enable_debug_check_->isChecked();
  request->timeout_s = static_cast<float>(timeout_spin_->value());

  setBusy(true);
  setStatus(QString("Sent %1").arg(QString::fromStdString(mode)));

  run_pose_client_->async_send_request(
    request,
    [this, mode](rclcpp::Client<RunPoseEstimation>::SharedFuture future) {
      const auto response = future.get();
      const QString message = QString("%1: %2\n%3")
        .arg(QString::fromStdString(mode))
        .arg(response->success ? "success" : "failed")
        .arg(QString::fromStdString(response->message));
      QMetaObject::invokeMethod(
        this,
        [this, message, success = response->success]() {
          setBusy(false);
          setStatus(message, !success);
        },
        Qt::QueuedConnection);
    });
}

void PerceptionControlPanel::sendTaskMoveRequest()
{
  if (!node_) {
    setStatus("RViz ROS node unavailable", true);
    return;
  }

  const auto block_id = target_block_edit_->text().trimmed().toStdString();
  if (block_id.empty()) {
    setStatus("Target block is empty", true);
    return;
  }

  ensureClients();
  if (!task_status_client_ || !task_status_client_->service_is_ready()) {
    setStatus(
      QString("Service unavailable: %1").arg(task_status_service_edit_->text()), true);
    return;
  }

  auto request = std::make_shared<SetBlockTaskStatus::Request>();
  request->block_id = block_id;
  request->task_status = concrete_block_world_model_interfaces::msg::Block::TASK_MOVE;

  setBusy(true);
  setStatus(QString("Sent TASK_MOVE for %1").arg(QString::fromStdString(block_id)));

  task_status_client_->async_send_request(
    request,
    [this, block_id](rclcpp::Client<SetBlockTaskStatus>::SharedFuture future) {
      const auto response = future.get();
      const QString message = QString("TASK_MOVE %1: %2\n%3")
        .arg(QString::fromStdString(block_id))
        .arg(response->success ? "success" : "failed")
        .arg(QString::fromStdString(response->message));
      QMetaObject::invokeMethod(
        this,
        [this, message, success = response->success]() {
          setBusy(false);
          setStatus(message, !success);
        },
        Qt::QueuedConnection);
    });
}

void PerceptionControlPanel::ensureClients()
{
  if (!node_) {
    return;
  }

  const std::string run_pose_name = run_pose_service_edit_->text().trimmed().toStdString();
  if (!run_pose_client_ || run_pose_name != run_pose_service_name_) {
    run_pose_service_name_ = run_pose_name;
    run_pose_client_ = node_->create_client<RunPoseEstimation>(run_pose_service_name_);
  }

  const std::string task_status_name = task_status_service_edit_->text().trimmed().toStdString();
  if (!task_status_client_ || task_status_name != task_status_service_name_) {
    task_status_service_name_ = task_status_name;
    task_status_client_ = node_->create_client<SetBlockTaskStatus>(task_status_service_name_);
  }
}

void PerceptionControlPanel::setBusy(bool busy)
{
  busy_ = busy;
  scene_discovery_button_->setEnabled(!busy_);
  refine_block_button_->setEnabled(!busy_);
  set_task_move_button_->setEnabled(!busy_);
  refine_grasped_button_->setEnabled(!busy_);
}

void PerceptionControlPanel::setStatus(const QString & text, bool error)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(error ? "QLabel { color: #b00020; }" : "");
}

}  // namespace concrete_block_perception_rviz_plugins

PLUGINLIB_EXPORT_CLASS(
  concrete_block_perception_rviz_plugins::PerceptionControlPanel,
  rviz_common::Panel)
