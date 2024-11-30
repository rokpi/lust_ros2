cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

# Add compiler warnings for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED) # Automatically handles dependencies
find_package(gazebo_ros REQUIRED) # If you're using Gazebo
find_package(robot_state_publisher REQUIRED) # Publishes robot state
find_package(controller_manager REQUIRED) # Manages controllers (e.g., diff_drive_controller)
find_package(robot_localization REQUIRED) # For odometry fusion

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install URDF and config files if they exist
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Enable testing if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Skips linter checks for copyrights if needed
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_package()

