cmake_minimum_required(VERSION 3.5)

project(rm_auto_aim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_tool REQUIRED)
find_package(rm_task REQUIRED)
find_package(rm_msgs REQUIRED)
find_package(rm_projectile_motion REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# rm_auto_aim lib
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})

ament_target_dependencies(${PROJECT_NAME}
 rclcpp 
 rm_tool
 rm_task
 rm_msgs
 rm_projectile_motion
 OpenCV
)

#task_show_auto_aim
add_executable(task_auto_aim nodes/task_auto_aim_node.cpp)
target_link_libraries(task_auto_aim ${PROJECT_NAME})

#test_armor_detector
add_executable(test_armor_detector nodes/test_armor_detector.cpp)
target_link_libraries(test_armor_detector ${PROJECT_NAME})


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
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install executables
install(TARGETS task_auto_aim test_armor_detector
  DESTINATION lib/${PROJECT_NAME}
)

#export include
ament_export_include_directories(include)
#export libraries
ament_export_libraries(${PROJECT_NAME})
#export dependency
ament_export_dependencies(rclcpp)
ament_export_dependencies(rm_tool)
ament_export_dependencies(rm_task)
ament_export_dependencies(rm_msgs)
ament_export_dependencies(rm_projectile_motion)
ament_export_dependencies(OpenCV)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()