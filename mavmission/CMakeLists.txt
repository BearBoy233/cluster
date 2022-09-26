cmake_minimum_required(VERSION 3.10.2)
project(mav_mission)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation

  mavcomm_msgs
)

add_message_files(
  FILES
  PositionCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs

  mavcomm_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES mav_mission
  CATKIN_DEPENDS 
  message_generation message_runtime roscpp rospy std_msgs
  mavcomm_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## include 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(mission
  src/com_define.cpp
  src/mission.cpp
  src/task_part.cpp
  src/formation_part.cpp 
)
add_dependencies(mission
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mission
  ${catkin_LIBRARIES}
)

## cpp
## Declare a cpp executable
add_executable(uav_mission 
  src/mission_node.cpp
)
add_dependencies(uav_mission 
  mission
)
#${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(uav_mission 
  mission
  ${catkin_LIBRARIES}
)

