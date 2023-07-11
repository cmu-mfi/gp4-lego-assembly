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
find_package(OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  message_generation
)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

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
  MakeMove.srv
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
)

# /* -------------------------------------------------------------------------- */
# /*                             Testing Executables                            */
# /* -------------------------------------------------------------------------- */

add_executable(
  service_sample_node src/ros_nodes/service_sample_node.cpp
)


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
target_link_libraries(${PROJECT_NAME}_task_planning_cartesian
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_task_planning_cartesian_sim
  src/ros_nodes/lego_task_planning_cartesian_sim_node.cpp
)
target_link_libraries(${PROJECT_NAME}_task_planning_cartesian_sim
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)


target_link_libraries(service_sample_node
  ${catkin_LIBRARIES}
)



install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



