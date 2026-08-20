cmake_minimum_required(VERSION 3.8)
project(agxarm_control_by_gamecontroller)

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
find_package(X11 REQUIRED)

# The ROS package follows the repository name; keep the established Python
# import namespace independent from the ROS package identity.
ament_python_install_package(armbycontroller)

add_executable(keyboard
  src/keyboard.cpp
)

ament_target_dependencies(keyboard
  rclcpp
  std_msgs
)
target_link_libraries(keyboard
  X11::X11
)

install(TARGETS
  keyboard
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  armbycontroller/ros/main.py
  armbycontroller/pose_controller.py
  armbycontroller/terminal_teleop.py
  armbycontroller/hand_controller.py
  armbycontroller/motion_link_bridge.py
  armbycontroller/ros/momentum_observer_node.py
  armbycontroller/ros/experiment_recorder_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/setup_can.sh
  scripts/select_input_device.sh
  scripts/start_nero.sh
  scripts/start_piper_l.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(DIRECTORY
  agx_arm_urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  PROJECT_STUDY_GUIDE_ZH_EN.md
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  docs/ROS_NODE_AND_ARCHITECTURE_ZH_EN.md
  DESTINATION share/${PROJECT_NAME}/docs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # package.xml declares Apache-2.0, but this compact source package does not
  # carry separate LICENSE/CONTRIBUTING files. Keep functional/style linters
  # enabled while skipping the file-presence copyright linter.
  set(ament_cmake_copyright_FOUND TRUE)
  # Register style linters with explicit source paths. This prevents in-source
  # colcon build/install directories from being mistaken for project source.
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_lint_cmake
    ament_cmake_pep257
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cpplint(src)
  ament_flake8(armbycontroller test launch)
  ament_lint_cmake(CMakeLists.txt)
  ament_pep257(armbycontroller test launch)
  ament_uncrustify(src)
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
  ament_add_pytest_test(observer_friction_test
    test/test_observer_friction.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(cartesian_admittance_test
    test/test_cartesian_admittance.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(cartesian_common_test
    test/test_cartesian_common.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(controller_interface_test
    test/test_control_interface.py
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(hybrid_control_test
    test/test_hybrid_control.py
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
