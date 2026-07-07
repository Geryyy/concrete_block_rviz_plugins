#pragma once

#include <memory>
#include <string>

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include "concrete_block_assembly_interfaces/srv/plan_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

namespace concrete_block_rviz_plugins
{

// RViz panel to list, load (activate) and clear wall-assembly plans in the
// world model via the wall_plan_server's ~/plan_control service.
class PlanControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PlanControlPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  using PlanControl = concrete_block_assembly_interfaces::srv::PlanControl;

  void buildUi();
  void sendCommand(const std::string & command, const std::string & plan_name);
  void refreshPlans();
  void ensureClient();
  void setBusy(bool busy);
  void setStatus(const QString & text, bool error = false);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<PlanControl>::SharedPtr client_;
  std::string service_name_{"/concrete_block_wall_plan_server/plan_control"};

  QLineEdit * service_edit_{nullptr};
  QComboBox * plan_combo_{nullptr};
  QPushButton * refresh_button_{nullptr};
  QPushButton * load_button_{nullptr};
  QPushButton * clear_button_{nullptr};
  QLabel * status_label_{nullptr};
  bool busy_{false};
};

}  // namespace concrete_block_rviz_plugins
