cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(communicate_2025 REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

# Add TinyMPC subdirectory
add_subdirectory(tinympc)

# Create planner library
add_library(${PROJECT_NAME} SHARED
  planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  tinympcstatic
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  auto_aim_interfaces
)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  planner.hpp
  DESTINATION include/${PROJECT_NAME}
)

# Build planner_node as a shared library for component loading
add_library(planner_node SHARED planner_node.cpp)
target_link_libraries(planner_node ${PROJECT_NAME})
ament_target_dependencies(planner_node 
  rclcpp 
  rclcpp_components
  auto_aim_interfaces 
  communicate_2025
)

# Register the component
rclcpp_components_register_node(planner_node
  PLUGIN "armor::PlannerNode"
  EXECUTABLE planner_node_exe
)

# Install node library
install(TARGETS planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} planner_node)
ament_export_dependencies(rclcpp rclcpp_components auto_aim_interfaces communicate_2025 Eigen3)

ament_package()
