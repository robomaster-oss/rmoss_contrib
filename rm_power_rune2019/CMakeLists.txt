cmake_minimum_required(VERSION 3.5)
project(rm_power_rune2019)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(rm_common REQUIRED)
find_package(rm_task REQUIRED)
find_package(rm_projectile_motion REQUIRED)
find_package(OpenCV REQUIRED)

# Include
include_directories(include)

# Create rm_power_rune2019 lib
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})
ament_target_dependencies(${PROJECT_NAME}
    rclcpp 
    rmoss_interfaces
    rm_common
    rm_task
    rm_projectile_motion
    OpenCV
)

# Create executable nodes
add_executable(task_power_rune nodes/task_power_rune_node.cpp)
target_link_libraries(task_power_rune ${PROJECT_NAME})


# Install launch res directories
install(DIRECTORY launch res
  DESTINATION share/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install executable nodes
install(TARGETS task_power_rune 
  DESTINATION lib/${PROJECT_NAME}
)

# Install executable scripts
install(PROGRAMS 
        scripts/client.py
    DESTINATION lib/${PROJECT_NAME})
    
#export rm_auto_aim dependency
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rmoss_interfaces)
ament_export_dependencies(rm_common)
ament_export_dependencies(rm_task)
ament_export_dependencies(rm_projectile_motion)
ament_export_dependencies(OpenCV)

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()