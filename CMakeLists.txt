cmake_minimum_required(VERSION 3.8)
project(concrete_block_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(concrete_block_world_model_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

add_definitions(-DQT_NO_KEYWORDS)

set(${PROJECT_NAME}_HDRS
  include/concrete_block_rviz_plugins/perception_control_panel.hpp
)

set(${PROJECT_NAME}_SRCS
  src/perception_control_panel.cpp
)

qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})

add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_HDRS}
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_MOCS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  concrete_block_world_model_interfaces
  pluginlib
  rclcpp
  rviz_common
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
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
  concrete_block_world_model_interfaces
  pluginlib
  rclcpp
  rviz_common
)

ament_package()
