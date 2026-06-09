#pragma once

#include <memory>
#include <string>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include "concrete_block_world_model_interfaces/srv/run_pose_estimation.hpp"
#include "concrete_block_world_model_interfaces/srv/set_block_task_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

namespace concrete_block_rviz_plugins
{

class PerceptionControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PerceptionControlPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  using RunPoseEstimation = concrete_block_world_model_interfaces::srv::RunPoseEstimation;
  using SetBlockTaskStatus = concrete_block_world_model_interfaces::srv::SetBlockTaskStatus;

  void buildUi();
  void sendPoseRequest(const std::string & mode, const std::string & target_block_id);
  void sendTaskMoveRequest();
  void ensureClients();
  void setBusy(bool busy);
  void setStatus(const QString & text, bool error = false);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<RunPoseEstimation>::SharedPtr run_pose_client_;
  rclcpp::Client<SetBlockTaskStatus>::SharedPtr task_status_client_;
  std::string run_pose_service_name_{"/world_model_node/run_pose_estimation"};
  std::string task_status_service_name_{"/world_model_node/set_block_task_status"};

  QLineEdit * run_pose_service_edit_{nullptr};
  QLineEdit * task_status_service_edit_{nullptr};
  QLineEdit * target_block_edit_{nullptr};
  QDoubleSpinBox * timeout_spin_{nullptr};
  QCheckBox * enable_debug_check_{nullptr};
  QCheckBox * refine_grasped_use_target_check_{nullptr};
  QPushButton * scene_discovery_button_{nullptr};
  QPushButton * refine_block_button_{nullptr};
  QPushButton * set_task_move_button_{nullptr};
  QPushButton * refine_grasped_button_{nullptr};
  QLabel * status_label_{nullptr};
  bool busy_{false};
};

}  // namespace concrete_block_rviz_plugins
