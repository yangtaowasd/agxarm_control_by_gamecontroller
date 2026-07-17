cmake_minimum_required(VERSION 3.8)
project(armbycontroller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(keyboard
  src/keyboard.cpp
)

ament_target_dependencies(keyboard
  rclcpp
  std_msgs
)

install(TARGETS
  keyboard
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  armbycontroller/keyboard_controller.py
  armbycontroller/pose_controller.py
  armbycontroller/terminal_teleop.py
  armbycontroller/hand_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # package.xml declares Apache-2.0, but this compact source package does not
  # carry separate LICENSE/CONTRIBUTING files. Keep functional/style linters
  # enabled while skipping the file-presence copyright linter.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(arm_control_test
    test/test_arm_control.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()
