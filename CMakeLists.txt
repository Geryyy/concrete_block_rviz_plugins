cmake_minimum_required(VERSION 3.8)
project(concrete_block_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(concrete_block_assembly_interfaces REQUIRED)
find_package(concrete_block_world_model_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lsrl_behavior_tree REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

add_definitions(-DQT_NO_KEYWORDS)

set(${PROJECT_NAME}_HDRS
  include/concrete_block_rviz_plugins/perception_control_panel.hpp
  include/concrete_block_rviz_plugins/plan_control_panel.hpp
  include/concrete_block_rviz_plugins/assembly_workflow_panel.hpp
  include/concrete_block_rviz_plugins/behavior_tree_panel.hpp
)

set(${PROJECT_NAME}_SRCS
  src/perception_control_panel.cpp
  src/plan_control_panel.cpp
  src/assembly_workflow_panel.cpp
  src/behavior_tree_panel.cpp
)

# The behavior-tree panel is the only one built from a Qt Designer file. uic
# writes ui_behavior_tree_panel.h into the binary directory; only
# src/behavior_tree_panel.cpp includes it, which is why the panel header holds
# its Ui:: member through a forward declaration -- the installed header must not
# reach into the build tree.
set(${PROJECT_NAME}_UIS
  qt/behavior_tree_panel.ui
)

set(${PROJECT_NAME}_DEPS
  concrete_block_assembly_interfaces
  concrete_block_world_model_interfaces
  ament_index_cpp
  lsrl_behavior_tree
  pluginlib
  rclcpp
  rclcpp_action
  rviz_common
)

qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})

add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_HDRS}
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_UIS_H}
  ${${PROJECT_NAME}_MOCS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Where qt5_wrap_ui put ui_behavior_tree_panel.h. PRIVATE: no consumer of this
# library ever sees it.
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_DEPS}
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  yaml-cpp
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES package.xml rviz_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ${${PROJECT_NAME}_DEPS}
)

ament_package()
