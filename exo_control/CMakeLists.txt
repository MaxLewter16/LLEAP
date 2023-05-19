cmake_minimum_required(VERSION 3.8)
project(exo_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros2_control REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)

# install c++ executables
include_directories(include)
add_executable(hw_interface src/exo_hw_interface.cpp)
add_executable(ros2_control src/ros2_control_node.cpp)
ament_target_dependencies(ros2_control rclcpp controller_manager)

install(TARGETS
  hw_interface
  ros2_control
  DESTINATION lib/${PORJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/bluetoothesp32.py
  ${PROJECT_NAME}/bluetoothscanner.py
  ${PROJECT_NAME}/serialesp32.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
