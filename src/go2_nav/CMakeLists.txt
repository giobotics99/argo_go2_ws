cmake_minimum_required(VERSION 3.8)
project(go2_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install configuration directories
install(DIRECTORY launch params rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as programs to lib/${PROJECT_NAME}
# This allows them to be found by 'ros2 run' and in launch files
install(PROGRAMS scripts/pointcloud_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
