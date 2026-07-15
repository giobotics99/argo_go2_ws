cmake_minimum_required(VERSION 3.8)
project(go2_patrol_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(go2_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  plansys2_msgs
  plansys2_executor
  nav2_msgs
  geometry_msgs
  go2_interfaces
  yaml-cpp
)

add_executable(navigate_action_node src/navigate_action_node.cpp)
ament_target_dependencies(navigate_action_node ${dependencies})

add_executable(inspect_action_node src/inspect_action_node.cpp)
ament_target_dependencies(inspect_action_node ${dependencies})

install(DIRECTORY launch pddl config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  navigate_action_node
  inspect_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gemma_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
