# cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.10.2)
project(mavcomm_msgs)

find_package(catkin REQUIRED COMPONENTS geographic_msgs geometry_msgs message_generation sensor_msgs std_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES

  Mavlink.msg
  Heartbeat.msg
  Console.msg
  Console_monitor.msg
  ChangeState.msg
  local_pos_enu.msg
  global_pos_int.msg  
  get_param.msg

  kcf_set_target.msg
  kcf_target_pos.msg



  mission_info.msg
  mission_back_info.msg
  mission_set.msg
  
  formation_back_info.msg
  formation_set.msg
  formation_set_info.msg

)

add_service_files(
  DIRECTORY srv
  FILES
)

# add_action_files(
#   DIRECTORY action
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(DEPENDENCIES geographic_msgs geometry_msgs sensor_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geographic_msgs geometry_msgs message_runtime sensor_msgs std_msgs
)

# install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")
