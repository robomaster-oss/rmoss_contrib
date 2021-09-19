cmake_minimum_required(VERSION 3.5)
project(rm_auto_aim)

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
find_package(rm_util REQUIRED)
find_package(rm_cam REQUIRED)
find_package(rm_projectile_motion REQUIRED)
find_package(OpenCV REQUIRED)

# Include
include_directories(include)

# Create rm_auto_aim lib
add_library(${PROJECT_NAME} SHARED
  src/auto_aim_algo/armor_detector.cpp
  src/auto_aim_algo/simple_auto_aim_algo.cpp
  src/simple_auto_aim_node.cpp
)
set(dependencies
  rclcpp 
  rmoss_interfaces
  rm_util
  rm_cam
  rm_projectile_motion
  OpenCV
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Create executable nodes
add_executable(simple_auto_aim src/simple_auto_aim_main.cpp)
target_link_libraries(simple_auto_aim ${PROJECT_NAME})

add_executable(armor_detector_test_main src/armor_detector_test_main.cpp)
target_link_libraries(armor_detector_test_main ${PROJECT_NAME})

# Install launch directories
install(DIRECTORY launch config resource
  DESTINATION share/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include
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
install(TARGETS simple_auto_aim
  DESTINATION lib/${PROJECT_NAME}
)

# Install executable scripts
install(PROGRAMS 
        scripts/client.py
    DESTINATION lib/${PROJECT_NAME})


#export rm_auto_aim dependency
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()