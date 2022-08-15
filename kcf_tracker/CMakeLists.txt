cmake_minimum_required(VERSION 2.8.3)
project(kcf_tracker)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED) # support opencv 3/4

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  OpenCV
  message_generation

  mavcomm_msgs
)

add_message_files(
  FILES

  track_target.msg
  set_track_mission.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES kcf_tracker 
  CATKIN_DEPENDS roscpp std_msgs message_runtime message_generation
#  DEPENDS system_lib
)

include_directories(include/kcf_tracker include/lib ${catkin_INCLUDE_DIRS} ) 
FILE(GLOB_RECURSE libfiles "src/lib/*.cpp")

add_executable(${PROJECT_NAME}_node src/kcf_tracker_node.cpp ${libfiles} )
add_dependencies(${PROJECT_NAME}_node kcf_tracker_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )
