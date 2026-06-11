#pragma once

#include <memory>
#include <string>

#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include "concrete_block_assembly_interfaces/srv/confirm_wall_origin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

namespace concrete_block_rviz_plugins
{

/// Control surface for the wall-building setup: select the plan, confirm/freeze
/// the interactively-placed origin (optionally persisting the expanded plan),
/// and report the validation result. Run/abort of the behavior tree itself is
/// handled by the existing BehaviorTreePanel.
class WallAssemblyPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WallAssemblyPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  using ConfirmWallOrigin = concrete_block_assembly_interfaces::srv::ConfirmWallOrigin;

  void buildUi();
  void sendConfirm();
  void ensureClient();
  void setBusy(bool busy);
  void setStatus(const QString & text, bool error = false);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<ConfirmWallOrigin>::SharedPtr confirm_client_;
  std::string confirm_service_name_{"/wall_setup_node/confirm_wall_origin"};

  QLineEdit * confirm_service_edit_{nullptr};
  QLineEdit * wall_plan_edit_{nullptr};
  QCheckBox * persist_check_{nullptr};
  QPushButton * confirm_button_{nullptr};
  QLabel * status_label_{nullptr};
  bool busy_{false};
};

}  // namespace concrete_block_rviz_plugins
