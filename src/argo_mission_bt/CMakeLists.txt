cmake_minimum_required(VERSION 3.8)
project(argo_mission_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(go2_interfaces REQUIRED)
find_package(unitree_go REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  rclcpp ament_index_cpp behaviortree_cpp_v3
  plansys2_msgs plansys2_domain_expert plansys2_problem_expert plansys2_planner plansys2_executor
  plansys2_pddl_parser go2_interfaces unitree_go geometry_msgs std_msgs)

include_directories(include)

add_executable(mission_node
  src/mission_node.cpp
  src/context.cpp
  src/conditions.cpp
  src/plansys2_nodes.cpp
  src/gemma_nodes.cpp
  src/interaction_nodes.cpp)
ament_target_dependencies(mission_node ${dependencies})
target_link_libraries(mission_node yaml-cpp)

install(TARGETS mission_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees include DESTINATION share/${PROJECT_NAME})

ament_package()
