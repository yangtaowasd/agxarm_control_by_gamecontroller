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
  armbycontroller/ros/keyboard_controller_node.py
  armbycontroller/pose_controller.py
  armbycontroller/terminal_teleop.py
  armbycontroller/hand_controller.py
  armbycontroller/motion_link_bridge.py
<<<<<<< HEAD
  armbycontroller/backend_api.py
  armbycontroller/hardware_probe.py
=======
  armbycontroller/ros/momentum_observer_node.py
  armbycontroller/ros/experiment_recorder_node.py
>>>>>>> feature/cartesian-impedance-step-by-step
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY
<<<<<<< HEAD
  docs
=======
  config
>>>>>>> feature/cartesian-impedance-step-by-step
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  agx_arm_urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  PROJECT_STUDY_GUIDE_ZH_EN.md
  DESTINATION share/${PROJECT_NAME}
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
  ament_add_pytest_test(cartesian_impedance_formula_test
    test/test_cartesian_impedance.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(momentum_observer_test
    test/test_momentum_observer.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(cartesian_admittance_test
    test/test_cartesian_admittance.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(controller_interface_test
    test/test_control_interface.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(experiment_framework_test
    test/test_experiment.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(hardware_connection_test
    test/test_hardware_connection.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()
