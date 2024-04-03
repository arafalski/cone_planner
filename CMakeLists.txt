cmake_minimum_required(VERSION 3.14)
project(cone_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/cone_planner/cone_planner.hpp
  include/cone_planner/visibility_control.hpp
  src/cone_planner.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/cone_planner/cone_planner_node.hpp
  src/cone_planner_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "cone_planner::ConePlannerNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_cone_planner
    test/test_cone_planner.cpp
  )
  ament_target_dependencies(test_cone_planner ${TEST_SOURCES})
  target_link_libraries(test_cone_planner ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
