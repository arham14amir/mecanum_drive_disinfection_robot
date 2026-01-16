cmake_minimum_required(VERSION 3.8)
project(robot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# 2. Add library
add_library(${PROJECT_NAME} SHARED
  src/robot_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

# 3. Link Dependencies (The FIX: Use ament_target_dependencies)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# 4. Export Plugin
pluginlib_export_plugin_description_file(hardware_interface robot_hardware_interface.xml)

# 5. Install Rules
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES robot_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# 6. Export for other packages
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()