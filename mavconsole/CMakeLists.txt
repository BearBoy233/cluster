cmake_minimum_required(VERSION 3.10.2)
project(mavconsole)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

# message_generation
  nav_msgs
  geometry_msgs

  serial
  tf2_eigen
  
  mavcomm_msgs
)

#generate_messages(
#    DEPENDENCIES
#    std_msgs
#   nav_msgs
#    geometry_msgs
    
#    mavcomm_msgs
#   # Or other packages containing msgs
#)


catkin_package(
   INCLUDE_DIRS # include
#  LIBRARIES mavconsole
   CATKIN_DEPENDS
	roscpp 
	message_runtime
	geometry_msgs
	serial
	std_msgs

  mavcomm_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mavconsole  mavconsole.cpp)
add_dependencies(mavconsole ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
# ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(mavconsole  ${catkin_LIBRARIES} )
