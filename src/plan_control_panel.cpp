#include "concrete_block_rviz_plugins/plan_control_panel.hpp"

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QStringList>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace concrete_block_rviz_plugins
{

PlanControlPanel::PlanControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  buildUi();
}

void PlanControlPanel::onInitialize()
{
  const auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    setStatus("RViz ROS node unavailable", true);
    return;
  }
  node_ = ros_node_abstraction->get_raw_node();
  ensureClient();
  setStatus("Ready — press Refresh to list plans");
  refreshPlans();
}

void PlanControlPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString value;
  if (config.mapGetString("PlanControlService", &value)) {
    service_edit_->setText(value);
  }
}

void PlanControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("PlanControlService", service_edit_->text());
}

void PlanControlPanel::buildUi()
{
  auto * root_layout = new QVBoxLayout(this);

  auto * config_box = new QGroupBox("Wall plan control", this);
  auto * config_layout = new QFormLayout(config_box);
  service_edit_ = new QLineEdit(QString::fromStdString(service_name_), config_box);
  plan_combo_ = new QComboBox(config_box);
  plan_combo_->setEditable(false);
  config_layout->addRow("Plan control service", service_edit_);
  config_layout->addRow("Plan", plan_combo_);

  auto * button_row = new QHBoxLayout();
  refresh_button_ = new QPushButton("Refresh", this);
  load_button_ = new QPushButton("Load", this);
  clear_button_ = new QPushButton("Clear", this);
  button_row->addWidget(refresh_button_);
  button_row->addWidget(load_button_);
  button_row->addWidget(clear_button_);

  status_label_ = new QLabel("Not initialized", this);
  status_label_->setWordWrap(true);
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);

  root_layout->addWidget(config_box);
  root_layout->addLayout(button_row);
  root_layout->addWidget(status_label_);
  root_layout->addStretch(1);

  connect(refresh_button_, &QPushButton::clicked, this, [this]() {refreshPlans();});
  connect(
    load_button_, &QPushButton::clicked, this, [this]() {
      sendCommand("activate", plan_combo_->currentText().trimmed().toStdString());
    });
  connect(
    clear_button_, &QPushButton::clicked, this, [this]() {sendCommand("clear", "");});
}

void PlanControlPanel::refreshPlans()
{
  sendCommand("list", "");
}

void PlanControlPanel::sendCommand(const std::string & command, const std::string & plan_name)
{
  if (!node_) {
    setStatus("RViz ROS node unavailable", true);
    return;
  }

  ensureClient();
  if (!client_ || !client_->service_is_ready()) {
    setStatus(QString("Service unavailable: %1").arg(service_edit_->text()), true);
    return;
  }

  if (command == "activate" && plan_name.empty()) {
    setStatus("No plan selected", true);
    return;
  }

  auto request = std::make_shared<PlanControl::Request>();
  request->command = command;
  request->wall_plan_name = plan_name;

  setBusy(true);
  setStatus(QString::fromStdString("Sending '" + command + "'…"));

  client_->async_send_request(
    request,
    [this, command](rclcpp::Client<PlanControl>::SharedFuture future) {
      const auto response = future.get();
      QStringList plans;
      for (const auto & name : response->plan_names) {
        plans << QString::fromStdString(name);
      }
      const QString active = QString::fromStdString(response->active_plan);
      const bool ok = response->success;
      const QString message = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(
        this,
        [this, command, plans, active, ok, message]() {
          setBusy(false);
          if (command == "list" && ok) {
            const QString keep = plan_combo_->currentText();
            plan_combo_->clear();
            plan_combo_->addItems(plans);
            int idx = plan_combo_->findText(keep);
            if (idx < 0 && !active.isEmpty()) {
              idx = plan_combo_->findText(active);
            }
            if (idx >= 0) {
              plan_combo_->setCurrentIndex(idx);
            }
          }
          setStatus(message, !ok);
        },
        Qt::QueuedConnection);
    });
}

void PlanControlPanel::ensureClient()
{
  if (!node_) {
    return;
  }
  const std::string name = service_edit_->text().trimmed().toStdString();
  if (!client_ || name != service_name_) {
    service_name_ = name;
    client_ = node_->create_client<PlanControl>(service_name_);
  }
}

void PlanControlPanel::setBusy(bool busy)
{
  busy_ = busy;
  refresh_button_->setEnabled(!busy_);
  load_button_->setEnabled(!busy_);
  clear_button_->setEnabled(!busy_);
}

void PlanControlPanel::setStatus(const QString & text, bool error)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(error ? "QLabel { color: #b00020; }" : "");
}

}  // namespace concrete_block_rviz_plugins

PLUGINLIB_EXPORT_CLASS(
  concrete_block_rviz_plugins::PlanControlPanel,
  rviz_common::Panel)
