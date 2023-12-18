cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
project(gp4_lego VERSION 1.0.0 LANGUAGES CXX)

# /* -------------------------------------------------------------------------- */
# /*                              Build Properties                              */
# /* -------------------------------------------------------------------------- */
add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

# /* -------------------------------------------------------------------------- */
# /*                                Find Package                                */
# /* -------------------------------------------------------------------------- */
find_package(Eigen3 3.3.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  message_generation
  yk_api
)

# /* -------------------------------------------------------------------------- */
# /*                               Create Library                               */
# /* -------------------------------------------------------------------------- */
file(GLOB src_files src/*.cpp
  src/Utils/*.cpp)

add_library(${PROJECT_NAME} SHARED ${src_files})

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  jsoncpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1)

add_service_files(
  FILES
  SetPose.srv
  GetPose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  yk_api
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


# /* -------------------------------------------------------------------------- */
# /*                             Testing Executables                            */
# /* -------------------------------------------------------------------------- */
add_executable(${PROJECT_NAME}_bringup
  src/ros_nodes/controller_node.cpp
)
target_link_libraries(${PROJECT_NAME}_bringup
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_task_planning_cartesian
  src/ros_nodes/lego_task_planning_cartesian_node.cpp
)

add_dependencies(${PROJECT_NAME}_task_planning_cartesian
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_task_planning_cartesian
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_integrate_task_planning
  src/ros_nodes/lego_integrate_task_planning.cpp
)

add_dependencies(${PROJECT_NAME}_integrate_task_planning
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_integrate_task_planning
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_sim_task_planning
  src/ros_nodes/lego_sim.cpp
)

add_dependencies(${PROJECT_NAME}_sim_task_planning
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_sim_task_planning
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
