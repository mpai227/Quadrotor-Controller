cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_controller)

find_package(Armadillo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  parameter_utils
  geometry_utils
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
add_definitions(${catkin_DEFINITIONS})

add_executable(${PROJECT_NAME}
  src/QuadrotorController.cpp
  src/quadrotor_controller.cpp
)

add_dependencies(${PROJECT_NAME}
  quadrotor_msgs_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
