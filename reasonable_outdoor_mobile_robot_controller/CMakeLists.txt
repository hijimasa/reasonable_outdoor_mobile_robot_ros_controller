cmake_minimum_required(VERSION 2.8.3)
project(reasonable_outdoor_mobile_robot_controller)

add_compile_options(--std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
)

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/reasonable_robot_controller.cpp
  src/reasonable_robot_interface.cpp
  src/reasonable_robot_arduino_communicator.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

