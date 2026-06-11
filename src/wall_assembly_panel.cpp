#include "concrete_block_rviz_plugins/wall_assembly_panel.hpp"

#include <QFormLayout>
#include <QGroupBox>
#include <QMetaObject>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace concrete_block_rviz_plugins
{

WallAssemblyPanel::WallAssemblyPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  buildUi();
}

void WallAssemblyPanel::onInitialize()
{
  const auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    setStatus("RViz ROS node unavailable", true);
    return;
  }
  node_ = ros_node_abstraction->get_raw_node();
  ensureClient();
  setStatus("Ready — place the origin in RViz, then Confirm & Freeze");
}

void WallAssemblyPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString value;
  if (config.mapGetString("ConfirmService", &value)) {
    confirm_service_edit_->setText(value);
  }
  if (config.mapGetString("WallPlanName", &value)) {
    wall_plan_edit_->setText(value);
  }
  bool checked = false;
  if (config.mapGetBool("Persist", &checked)) {
    persist_check_->setChecked(checked);
  }
}

void WallAssemblyPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("ConfirmService", confirm_service_edit_->text());
  config.mapSetValue("WallPlanName", wall_plan_edit_->text());
  config.mapSetValue("Persist", persist_check_->isChecked());
}

void WallAssemblyPanel::buildUi()
{
  auto * root_layout = new QVBoxLayout(this);

  auto * config_box = new QGroupBox("Wall setup", this);
  auto * config_layout = new QFormLayout(config_box);
  confirm_service_edit_ = new QLineEdit(
    QString::fromStdString(confirm_service_name_), config_box);
  wall_plan_edit_ = new QLineEdit("example_wall", config_box);
  persist_check_ = new QCheckBox("Write expanded plan to disk", config_box);
  persist_check_->setChecked(false);
  config_layout->addRow("Confirm service", confirm_service_edit_);
  config_layout->addRow("Wall plan name", wall_plan_edit_);
  config_layout->addRow("Persist", persist_check_);

  confirm_button_ = new QPushButton("Confirm Origin && Freeze", this);

  status_label_ = new QLabel("Not initialized", this);
  status_label_->setWordWrap(true);
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);

  root_layout->addWidget(config_box);
  root_layout->addWidget(confirm_button_);
  root_layout->addWidget(status_label_);
  root_layout->addStretch(1);

  connect(confirm_button_, &QPushButton::clicked, this, [this]() {sendConfirm();});
}

void WallAssemblyPanel::sendConfirm()
{
  if (!node_) {
    setStatus("RViz ROS node unavailable", true);
    return;
  }

  ensureClient();
  if (!confirm_client_ || !confirm_client_->service_is_ready()) {
    setStatus(
      QString("Service unavailable: %1").arg(confirm_service_edit_->text()), true);
    return;
  }

  auto request = std::make_shared<ConfirmWallOrigin::Request>();
  request->wall_plan_name = wall_plan_edit_->text().trimmed().toStdString();
  request->persist = persist_check_->isChecked();
  // Leave origin at default (zero) so the node keeps the current interactive
  // marker pose rather than overriding it.

  setBusy(true);
  setStatus("Confirming origin…");

  confirm_client_->async_send_request(
    request,
    [this](rclcpp::Client<ConfirmWallOrigin>::SharedFuture future) {
      const auto response = future.get();
      const QString message = QString("%1 — %2 blocks, valid=%3\n%4")
        .arg(response->success ? "Confirmed" : "Failed")
        .arg(response->num_blocks)
        .arg(response->valid ? "yes" : "no")
        .arg(QString::fromStdString(response->message));
      const bool error = !response->success || !response->valid;
      QMetaObject::invokeMethod(
        this,
        [this, message, error]() {
          setBusy(false);
          setStatus(message, error);
        },
        Qt::QueuedConnection);
    });
}

void WallAssemblyPanel::ensureClient()
{
  if (!node_) {
    return;
  }
  const std::string name = confirm_service_edit_->text().trimmed().toStdString();
  if (!confirm_client_ || name != confirm_service_name_) {
    confirm_service_name_ = name;
    confirm_client_ = node_->create_client<ConfirmWallOrigin>(confirm_service_name_);
  }
}

void WallAssemblyPanel::setBusy(bool busy)
{
  busy_ = busy;
  confirm_button_->setEnabled(!busy_);
}

void WallAssemblyPanel::setStatus(const QString & text, bool error)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(error ? "QLabel { color: #b00020; }" : "");
}

}  // namespace concrete_block_rviz_plugins

PLUGINLIB_EXPORT_CLASS(
  concrete_block_rviz_plugins::WallAssemblyPanel,
  rviz_common::Panel)
