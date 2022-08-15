cmake_minimum_required(VERSION 2.8.3)
project(px4_offb)

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

catkin_package(
  INCLUDE_DIRS 
# include
# LIBRARIES px4_offb
  CATKIN_DEPENDS 
  message_generation message_runtime roscpp rospy std_msgs
  mavcomm_msgs
#  DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(uav_ctrl src/uav_ctrl.cpp)
add_dependencies(uav_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_ctrl ${catkin_LIBRARIES})

add_executable(vision_pose src/vision_pose.cpp)
add_dependencies(vision_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_pose ${catkin_LIBRARIES})

add_executable(odom2camerapose src/odom2camerapose.cpp)
add_dependencies(odom2camerapose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom2camerapose ${catkin_LIBRARIES})
