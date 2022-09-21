cmake_minimum_required(VERSION 3.10.2)
project(mavcomm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

# message_generation
  nav_msgs
  geometry_msgs
  mavros_msgs

  serial
  tf2_eigen
  
  mavcomm_msgs
)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  nav_msgs
#  geometry_msgs
#  mavros_msgs
    
#  mavcomm_msgs
#)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES mavcomm
#  DEPENDS system_lib
  CATKIN_DEPENDS
	roscpp 
	message_runtime
	geometry_msgs
	serial
	std_msgs
	mavros_msgs

  mavcomm_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mavcomm  src/mavcomm.cpp)
add_dependencies(mavcomm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavcomm  ${catkin_LIBRARIES} )

# TODO UDP Test 
add_executable(mavcomm_udp_test  src/mavcomm_udp_test.cpp)
add_dependencies(mavcomm_udp_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mavcomm_udp_test  ${catkin_LIBRARIES} )

# plugins px4_bridge
add_executable(px4_bridge  src/plugins/px4_bridge.cpp)
add_dependencies(px4_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(px4_bridge  ${catkin_LIBRARIES} )

# plugins uart_test
add_executable(uart_test  src/plugins/uart_test.cpp)
add_dependencies(uart_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uart_test  ${catkin_LIBRARIES} )
