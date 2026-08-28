// Copyright [2025] AIT Austrian Institute of Technology GmbH

#include "concrete_block_rviz_plugins/behavior_tree_panel.hpp"

#include <QFileDialog>
#include <QtWidgets>

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display_context.hpp"
#include "yaml-cpp/yaml.h"

#include "ui_behavior_tree_panel.h"  // NOLINT(build/include_subdir)

namespace concrete_block_rviz_plugins
{

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;  // NOLINT(build/namespaces)

namespace
{

/// The launch profile's choice of behavior-tree package, or the CBS default.
/**
 * `cbs_wall_assembly_pzs100.launch.py` exports this together with
 * `BEHAVIOR_TREE_PANEL_BT_CATALOG`.  The default is the CBS package rather than
 * `epsilon_crane_behavior_tree`, because this panel now ships in the CBS tree.
 */
std::string bt_package_from_environment()
{
  const char * from_env = std::getenv("BEHAVIOR_TREE_PANEL_BT_PACKAGE");
  if (from_env != nullptr && *from_env != '\0') {
    return std::string(from_env);
  }
  return std::string("concrete_block_behavior_tree");
}

std::string environment_or(const char * name, const std::string & fallback)
{
  const char * from_env = std::getenv(name);
  if (from_env != nullptr && *from_env != '\0') {
    return std::string(from_env);
  }
  return fallback;
}

}  // namespace

BehaviorTreePanel::BehaviorTreePanel(QWidget * parent)
: Panel(parent),
  ui_(std::make_shared<Ui::BehaviorTreePanelForm>())
{
  ui_->setupUi(this);

  lastUsedFolder_ = settings_.value("LastUsedFolderBT").toString();

  connect(
    ui_->pushButton_move, &QPushButton::clicked, [this]() {
      send_bt(move_empty_path_);
    });

  connect(
    ui_->toolButton_loadFile, &QPushButton::clicked, [this]() {
      const QString fileName = QFileDialog::getOpenFileName(
        nullptr, "Open BT catalog", lastUsedFolder_, "YAML Files (*.yaml)");
      if (fileName.isEmpty()) {
        return;
      }
      settings_.setValue("LastUsedFolderBT", QFileInfo(fileName).path());
      load_catalog(fileName);
    });

  connect(
    ui_->pushButton_start, &QPushButton::clicked, [this]() {
      const auto selectedBtName = ui_->comboBox_BTs->currentText();
      const auto entry = bt_map_.find(selectedBtName);
      if (entry == bt_map_.end()) {
        RCLCPP_WARN_STREAM(
          node_->get_logger(),
          "No behavior tree in the catalog named '" << selectedBtName.toStdString() << "'");
        return;
      }
      send_bt(entry->second.toStdString());
    });

  connect(
    ui_->pushButton_cancel, &QPushButton::clicked, [this]() {
      client_bt_->async_cancel_all_goals();
      RCLCPP_INFO(node_->get_logger(), "BT goal cancel requested");
    });
}

BehaviorTreePanel::~BehaviorTreePanel() = default;

void BehaviorTreePanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  const std::string bt_package = bt_package_from_environment();
  try {
    behaviortree_pkg_share_ = ament_index_cpp::get_package_share_directory(bt_package);
    bt_package_found_ = true;
    RCLCPP_INFO(
      node_->get_logger(), "Behavior trees from package '%s' at %s",
      bt_package.c_str(), behaviortree_pkg_share_.c_str());
  } catch (const std::exception & e) {
    // Not a throw. The legacy panel threw here, and pluginlib carried the
    // exception out through RViz's panel construction: a workspace without that
    // one package lost the whole RViz session rather than one panel.
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "Behavior-tree package '" << bt_package << "' not found: " << e.what() <<
        ". Set BEHAVIOR_TREE_PANEL_BT_PACKAGE. The panel stays disabled.");
  }

  if (bt_package_found_) {
    move_empty_path_ = behaviortree_pkg_share_ +
      environment_or("BEHAVIOR_TREE_PANEL_BT_MOVE_EMPTY", "/behavior_trees/move_empty.xml");
    // The button is fixed to one tree, and which one is a profile's choice. The
    // CBS package has no `move_empty.xml` -- it has `cbs_move_empty_pzs100.xml`
    // -- so without the check above this button would send a path that does not
    // exist and the failure would surface as an aborted goal.
    move_empty_available_ = std::filesystem::exists(move_empty_path_);
    if (!move_empty_available_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "No move-empty tree at %s; that button stays disabled. Set "
        "BEHAVIOR_TREE_PANEL_BT_MOVE_EMPTY to a path inside the package share.",
        move_empty_path_.c_str());
    }
  }

  // The catalog the launch profile named, loaded without a file dialog. The
  // dialog remains for any other catalog.
  const std::string catalog = environment_or("BEHAVIOR_TREE_PANEL_BT_CATALOG", "");
  if (!catalog.empty()) {
    const std::size_t loaded = load_catalog(QString::fromStdString(catalog));
    RCLCPP_INFO(
      node_->get_logger(), "Loaded %zu behavior tree(s) from %s", loaded, catalog.c_str());
  }

  timer_ = node_->create_wall_timer(10ms, std::bind(&BehaviorTreePanel::timer_cb, this));

  bt_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_bt_ = rclcpp_action::create_client<ExecuteBT>(node_, "/execute_bt", bt_cb_group_);
}

std::size_t BehaviorTreePanel::load_catalog(const QString & file_name)
{
  ui_->comboBox_BTs->clear();
  ui_->comboBox_BTs->setEnabled(false);
  bt_map_.clear();

  try {
    YAML::Node config = YAML::LoadFile(file_name.toStdString());

    // `package:` names the share the relative `path:` entries hang off. It is
    // optional; without it the panel's own behavior-tree package is used.
    std::string catalog_pkg_share = behaviortree_pkg_share_;
    if (config["package"]) {
      const auto pkg_name = config["package"].as<std::string>();
      try {
        catalog_pkg_share = ament_index_cpp::get_package_share_directory(pkg_name);
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(
          node_->get_logger(),
          "bt_panel_catalog: could not find package '" << pkg_name << "': " << e.what());
      }
    }

    for (const auto & bt : config["behavior_trees"]) {
      const QString name = QString::fromStdString(bt["name"].as<std::string>());
      const QString path =
        QString::fromStdString(catalog_pkg_share + bt["path"].as<std::string>());
      bt_map_[name] = path;
      ui_->comboBox_BTs->addItem(name);
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "Could not read BT catalog " << file_name.toStdString() << ": " << e.what());
    return 0;
  }

  if (ui_->comboBox_BTs->count() > 0) {
    ui_->comboBox_BTs->setEnabled(true);
  }
  return bt_map_.size();
}

void BehaviorTreePanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void BehaviorTreePanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

void BehaviorTreePanel::timer_cb()
{
  const bool idle = !is_bt_running_;
  ui_->pushButton_move->setEnabled(idle && bt_package_found_ && move_empty_available_);
  ui_->pushButton_start->setEnabled(idle && !bt_map_.empty());
  ui_->pushButton_cancel->setEnabled(is_bt_running_);
  ui_->cb_autonomy_mode->setEnabled(idle);
}

void BehaviorTreePanel::send_bt(const std::string & behaviortree)
{
  if (is_bt_running_) {return;}

  if (!client_bt_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node_->get_logger(), "BT action server not available after waiting");
    ui_->btStatus->setText(QString("No /execute_bt server"));
    return;
  }

  auto goal_msg = ExecuteBT::Goal();
  goal_msg.behavior_tree_path = behaviortree;
  goal_msg.autonomy_mode = ui_->cb_autonomy_mode->isChecked();

  RCLCPP_INFO(node_->get_logger(), "Sending %s to the BT action server", behaviortree.c_str());
  auto send_goal_options = rclcpp_action::Client<ExecuteBT>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&BehaviorTreePanel::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&BehaviorTreePanel::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&BehaviorTreePanel::result_callback, this, _1);
  goal_handle_future_ = client_bt_->async_send_goal(goal_msg, send_goal_options);
}

void BehaviorTreePanel::goal_response_callback(GoalHandleExecuteBT::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    ui_->btStatus->setText(QString("Rejected"));
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
  is_bt_running_ = true;
}

void BehaviorTreePanel::feedback_callback(
  GoalHandleExecuteBT::SharedPtr,
  const std::shared_ptr<const ExecuteBT::Feedback> feedback)
{
  if (feedback->last_change.current_status == "IDLE") {
    ui_->btStatus->setText(QString(""));
    return;
  }
  ui_->btStatus->setText(
    QString::fromStdString(
      feedback->last_change.node_name + ": " + feedback->last_change.current_status));
}

void BehaviorTreePanel::result_callback(const GoalHandleExecuteBT::WrappedResult & result)
{
  is_bt_running_ = false;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Goal was reached");
      ui_->btStatus->setText(QString("Success"));
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(node_->get_logger(), "Goal was aborted");
      ui_->btStatus->setText(QString("Aborted"));
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "Goal was canceled");
      ui_->btStatus->setText(QString("Canceled"));
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      ui_->btStatus->setText(QString("Unknown error"));
      break;
  }
}

}  // namespace concrete_block_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  concrete_block_rviz_plugins::BehaviorTreePanel, rviz_common::Panel)
