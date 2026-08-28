// Copyright [2025] AIT Austrian Institute of Technology GmbH

// The behavior-tree runner panel, ported out of `behavior_tree_panel` in
// `src/legacy/timber_crane_rviz_plugins/`.  That tree carries `COLCON_IGNORE`,
// so the panel `concrete_block_behavior_tree/rviz/cbs.rviz` asks for was never
// built and RViz dropped it silently -- which is what made
// `cbs_move_empty_pzs100.xml` unreachable from the operator UI.
//
// Three things changed in the port, and nothing else:
//
//   * the behavior-tree package is read from `BEHAVIOR_TREE_PANEL_BT_PACKAGE`
//     and defaults to `concrete_block_behavior_tree` rather than being fixed at
//     `epsilon_crane_behavior_tree`.  `cbs_wall_assembly_pzs100.launch.py`
//     has been exporting that variable all along; the legacy panel never read
//     it;
//   * a catalog named by `BEHAVIOR_TREE_PANEL_BT_CATALOG` is loaded at startup,
//     so the tree list is populated without walking a file dialog first.  The
//     dialog stays, for a catalog that is not the launch's;
//   * a missing package is logged and disables the panel instead of throwing
//     out of `onInitialize()`, which took RViz down with it.
//
// The action name, the goal fields and the feedback rendering are the legacy
// ones unchanged: this panel talks to `lsrl_behavior_tree`'s `bt_action_server`
// on `/execute_bt` and to nothing else.

#ifndef CONCRETE_BLOCK_RVIZ_PLUGINS__BEHAVIOR_TREE_PANEL_HPP_
#define CONCRETE_BLOCK_RVIZ_PLUGINS__BEHAVIOR_TREE_PANEL_HPP_

#include <QSettings>
#include <QString>

#include <map>
#include <memory>
#include <string>

#include "lsrl_behavior_tree/action/execute_bt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"

// The uic output is a build artifact in the binary directory, so it is included
// by the .cpp alone and the member below is held through a forward declaration.
// This header is installed; an installed header that included a generated one
// would not compile anywhere but here.
namespace Ui
{
class BehaviorTreePanelForm;
}  // namespace Ui

namespace concrete_block_rviz_plugins
{

using ExecuteBT = lsrl_behavior_tree::action::ExecuteBT;

class BehaviorTreePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit BehaviorTreePanel(QWidget * parent = nullptr);
  ~BehaviorTreePanel() override;

  void onInitialize() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  using GoalHandleExecuteBT = rclcpp_action::ClientGoalHandle<ExecuteBT>;

  void timer_cb();
  void send_bt(const std::string & behaviortree);

  /// Fill the combo box from a `bt_panel_catalog.yaml`. Returns entries loaded.
  std::size_t load_catalog(const QString & file_name);

  void goal_response_callback(GoalHandleExecuteBT::SharedPtr goal_handle);
  void feedback_callback(
    GoalHandleExecuteBT::SharedPtr,
    const std::shared_ptr<const ExecuteBT::Feedback> feedback);
  void result_callback(const GoalHandleExecuteBT::WrappedResult & result);

  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool is_bt_running_ = false;
  /// False when the behavior-tree package could not be found: every button that
  /// would send a path built from it stays disabled rather than sending "".
  bool bt_package_found_ = false;
  /// False when the fixed "Move empty" tree does not exist in that package.
  bool move_empty_available_ = false;

  std::string behaviortree_pkg_share_;
  std::string move_empty_path_;

  rclcpp::CallbackGroup::SharedPtr bt_cb_group_;
  rclcpp_action::Client<ExecuteBT>::SharedPtr client_bt_;
  std::shared_future<GoalHandleExecuteBT::SharedPtr> goal_handle_future_;

  std::map<QString, QString> bt_map_;

  std::shared_ptr<Ui::BehaviorTreePanelForm> ui_;

  QSettings settings_;
  QString lastUsedFolder_;
};

}  // namespace concrete_block_rviz_plugins

#endif  // CONCRETE_BLOCK_RVIZ_PLUGINS__BEHAVIOR_TREE_PANEL_HPP_
