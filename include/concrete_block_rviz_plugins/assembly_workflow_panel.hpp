#pragma once

#include <memory>

#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QStringList>

#include "concrete_block_world_model_interfaces/srv/run_pose_estimation.hpp"
#include "lsrl_behavior_tree/action/execute_bt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

namespace concrete_block_rviz_plugins
{

class AssemblyWorkflowPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AssemblyWorkflowPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  using ExecuteBT = lsrl_behavior_tree::action::ExecuteBT;
  using GoalHandleExecuteBT = rclcpp_action::ClientGoalHandle<ExecuteBT>;
  using RunPoseEstimation = concrete_block_world_model_interfaces::srv::RunPoseEstimation;

  void buildUi();
  void startSession();
  void sendCommand(const std::string & command);
  void refreshSceneEstimate();
  void setExpectedCommand(const QString & command);
  void updateButtonEnablement();
  void setStatus(const QString & text, bool error = false);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ExecuteBT>::SharedPtr execute_client_;
  rclcpp::Client<RunPoseEstimation>::SharedPtr run_pose_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr refinement_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr residual_indicator_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr residual_indicator_threshold_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr residual_translation_error_sub_;
  QPushButton * start_button_{nullptr};
  QPushButton * acquire_button_{nullptr};
  QPushButton * pick_button_{nullptr};
  QPushButton * hover_button_{nullptr};
  QPushButton * measure_button_{nullptr};
  QPushButton * correct_button_{nullptr};
  QPushButton * refresh_scene_button_{nullptr};
  QPushButton * place_button_{nullptr};
  QLabel * state_label_{nullptr};
  QLabel * refinement_label_{nullptr};
  QLabel * residual_light_{nullptr};
  QLabel * residual_light_label_{nullptr};
  QLabel * execution_label_{nullptr};
  QLabel * status_label_{nullptr};
  QString expected_command_;
  QStringList expected_commands_;
  bool session_active_{false};
  bool refresh_in_flight_{false};
  double residual_indicator_threshold_m_{0.10};
  bool residual_indicator_measured_{false};
  bool residual_within_indicator_threshold_{false};
  double residual_translation_error_m_{0.0};

  void setResidualLight(bool within_threshold, bool measured);
};

}  // namespace concrete_block_rviz_plugins
