#pragma once

// Qt first: cpplint reads an extension-less header as a C system header, and
// those have to come before the C++ standard ones.
#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QTimer>
#include <QVBoxLayout>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "concrete_block_assembly_interfaces/srv/plan_control.hpp"
#include "concrete_block_rviz_plugins/cbs_run_state.hpp"
#include "concrete_block_world_model_interfaces/srv/clear_world_model.hpp"
#include "concrete_block_world_model_interfaces/srv/run_pose_estimation.hpp"
#include "lsrl_behavior_tree/action/execute_bt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace concrete_block_rviz_plugins
{

/// One panel for a CBS run: status, setup, start, and the operator steps.
/**
 * Replaces the PlanControl -> BehaviorTree -> AssemblyWorkflow sequence, whose
 * order the operator had to know. The state logic is in CBSRunState; this class
 * is the ROS plumbing and the widgets.
 */
class CBSRunPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CBSRunPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  using ExecuteBT = lsrl_behavior_tree::action::ExecuteBT;
  using GoalHandleExecuteBT = rclcpp_action::ClientGoalHandle<ExecuteBT>;
  using PlanControl = concrete_block_assembly_interfaces::srv::PlanControl;
  using RunPoseEstimation = concrete_block_world_model_interfaces::srv::RunPoseEstimation;
  using ClearWorldModel = concrete_block_world_model_interfaces::srv::ClearWorldModel;

  // Pickup and placement refinement report the same three things on their own topics.
  struct Residual
  {
    bool measured{false};
    bool within{false};
    double error_m{0.0};
    QString text;
  };

  void buildUi();
  void loadCatalog(const std::string & catalog_file);
  void startRun();
  void sendGoal();
  void stopRun();
  void retryBlock();
  void sendCommand(const std::string & command);
  void sendPlanCommand(
    const std::string & command, const std::string & plan_name,
    const std::string & plans_file = "");
  void runSceneDiscovery();
  void clearWorldModel();
  void onOperatorState(const QString & latched);
  void refresh();
  void setBusy(bool busy);
  void setStatus(const QString & text, bool error = false);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ExecuteBT>::SharedPtr execute_client_;
  rclcpp::Client<PlanControl>::SharedPtr plan_client_;
  rclcpp::Client<RunPoseEstimation>::SharedPtr run_pose_client_;
  rclcpp::Client<ClearWorldModel>::SharedPtr clear_world_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> residual_subs_;

  QLabel * run_label_{nullptr};
  QLabel * phase_label_{nullptr};
  QPushButton * stop_button_{nullptr};
  QPushButton * retry_button_{nullptr};
  QLabel * deadman_banner_{nullptr};
  QLineEdit * plans_file_edit_{nullptr};
  QComboBox * plan_combo_{nullptr};
  QPushButton * reload_button_{nullptr};
  QPushButton * activate_button_{nullptr};
  QPushButton * clear_plan_button_{nullptr};
  QPushButton * scene_discovery_button_{nullptr};
  QPushButton * clear_world_button_{nullptr};
  QGroupBox * tree_box_{nullptr};
  QVBoxLayout * tree_layout_{nullptr};
  QCheckBox * approve_every_move_check_{nullptr};
  QPushButton * start_button_{nullptr};
  QGroupBox * step_box_{nullptr};
  QPushButton * continue_button_{nullptr};
  QPushButton * measure_button_{nullptr};
  QPushButton * correct_button_{nullptr};
  QLabel * residual_light_{nullptr};
  QLabel * residual_label_{nullptr};
  QLabel * refinement_label_{nullptr};
  QLabel * status_label_{nullptr};
  QTimer * connect_timer_{nullptr};

  std::vector<std::pair<QRadioButton *, QString>> tree_radios_;
  CBSRunState state_;
  Residual pickup_;
  Residual placement_;
  std::string active_tree_path_;
  QString last_state_text_;
  bool command_in_flight_{false};
  bool retry_pending_{false};
  bool busy_{false};
};

}  // namespace concrete_block_rviz_plugins
