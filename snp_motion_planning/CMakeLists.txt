cmake_minimum_required(VERSION 3.5)
project(snp_motion_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ROS2_DEPS
    rclcpp
    snp_msgs
    std_srvs
    tesseract_monitoring
    tf2_eigen)

find_package(ament_cmake REQUIRED)
foreach(dep ${ROS2_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

find_package(descartes_light REQUIRED)
find_package(
  tesseract_planning REQUIRED
  COMPONENTS command_language
             task_composer
             motion_planners
             time_parameterization)
find_package(
  tesseract REQUIRED
  COMPONENTS kinematics
             collision_vhacd
             kinematics_kdl
             kinematics_ikfast)
find_package(LAPACK REQUIRED) # Requried for ikfast
find_package(yaml-cpp REQUIRED)

# Libraries
add_library(${PROJECT_NAME}_tasks SHARED src/plugins/tasks/tcp_speed_limiter_profile.cpp
                                         src/plugins/tasks/tcp_speed_limiter_task.cpp)
target_link_libraries(
  ${PROJECT_NAME}_tasks
  tesseract::common
  tesseract::command_language
  tesseract::task_composer
  tesseract::kinematics_kdl
  tesseract::time_parameterization
  tesseract::environment
  tesseract::motion_planners
  trajopt::trajopt_ifopt
  tesseract::motion_planners_trajopt_ifopt
  trajopt::trajopt_sqp
  trajopt::trajopt
  yaml-cpp)

# Planning server
add_executable(${PROJECT_NAME}_node src/planning_server.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${ROS2_DEPS})
target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_tasks
  tesseract::common
  tesseract::collision_vhacd_convex_decomposition
  tesseract::command_language
  tesseract::task_composer
  tesseract::task_composer_planning_nodes
  tesseract::motion_planners
  tesseract::motion_planners_simple
  tesseract::motion_planners_ompl
  tesseract::motion_planners_trajopt
  tesseract::motion_planners_trajopt_ifopt
  tesseract::motion_planners_descartes
  tesseract::time_parameterization_isp
  tesseract::kinematics_kdl
  trajopt::trajopt_ifopt
  yaml-cpp)

# Plugin Library
add_library(${PROJECT_NAME}_plugins SHARED src/plugins/kinematics/motoman_hc10_ikfast_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_plugins PUBLIC ${LAPACK_LIBRARIES} tesseract::common tesseract::kinematics_ikfast)
target_include_directories(${PROJECT_NAME}_plugins SYSTEM PUBLIC ${LAPACK_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Install the executable(s)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

# Install the library(ies)
install(TARGETS ${PROJECT_NAME}_plugins ${PROJECT_NAME}_tasks EXPORT ${PROJECT_NAME}-targets DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${ROS2_DEPS})
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_package()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)
