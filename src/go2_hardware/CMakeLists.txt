cmake_minimum_required(VERSION 3.8)
project(go2_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Nodo voce: legge /supervisor/explanation con lo speaker del Go2 (Piper + WebRTC, librerie in venv-voice)
install(PROGRAMS
  scripts/argo_voice_node.py
  scripts/go2_speaker_worker.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
