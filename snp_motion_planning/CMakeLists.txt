cmake_minimum_required(VERSION 3.5)
project(snp_motion_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
 include
)

set(ROS2_DEPS
    rclcpp
    snp_msgs
    std_srvs
    tesseract_monitoring
    tf2_eigen)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
foreach(dep ${ROS2_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

find_package(descartes_light REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_collision REQUIRED COMPONENTS vhacd)
find_package(tesseract_task_composer REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(LAPACK REQUIRED) # Requried for ikfast
find_package(yaml-cpp REQUIRED)

# Libraries
add_library(
  ${PROJECT_NAME}_tasks SHARED
  src/plugins/tasks/constant_tcp_speed_time_parameterization_profile.cpp
  src/plugins/tasks/constant_tcp_speed_time_parameterization_task.cpp
  src/plugins/tasks/kinematic_limits_check_profile.cpp
  src/plugins/tasks/kinematic_limits_check_task.cpp
  src/plugins/tasks/tcp_speed_limiter_profile.cpp
  src/plugins/tasks/tcp_speed_limiter_task.cpp)
target_link_libraries(
  ${PROJECT_NAME}_tasks
  tesseract::tesseract_common
  tesseract::tesseract_command_language
  tesseract::tesseract_task_composer
  tesseract::tesseract_kinematics_kdl
  tesseract::tesseract_time_parameterization_core
  tesseract::tesseract_environment
  tesseract::tesseract_motion_planners_core
  yaml-cpp)

# Planning server
add_executable(${PROJECT_NAME}_node src/planning_server.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${ROS2_DEPS})
target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_tasks
  tesseract::tesseract_common
  tesseract::tesseract_collision_vhacd_convex_decomposition
  tesseract::tesseract_command_language
  tesseract::tesseract_task_composer
  tesseract::tesseract_task_composer_planning_nodes
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_simple
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_descartes
  tesseract::tesseract_time_parameterization_isp
  tesseract::tesseract_kinematics_kdl
  yaml-cpp)

# Install the executable(s)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/motion_client.py
  ${PROJECT_NAME}/velocity_plot.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${ROS2_DEPS})
ament_export_targets(${PROJECT_NAME}_tasks HAS_LIBRARY_TARGET)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

install(
  TARGETS ${PROJECT_NAME}_tasks
  EXPORT ${PROJECT_NAME}_tasks
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include

)
ament_package()
