cmake_minimum_required(VERSION 2.8.3)
project(robot_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  trajectory_msgs
	control_msgs
	std_srvs
  message_generation
  actionlib
	pilz_control
	prbt_hardware_support
)

add_message_files(
  FILES
  planned_trajectory_msg.msg
  geometry_trajectory_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
  robot_bridge
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)


include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

##################################################################################################

# Bridge

add_executable(prbt_on_mpo_Bridge_Node 
	src/prbt_on_mpo/prbt_on_mpo_bridge_Node.cpp 
	src/prbt_on_mpo/prbt_on_mpo_bridge.cpp)
target_link_libraries(prbt_on_mpo_Bridge_Node ${catkin_LIBRARIES})
add_dependencies(prbt_on_mpo_Bridge_Node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(mpo_500_Bridge_Node 
	src/mpo_500_bridge/mpo_500_bridge_Node.cpp 
	src/mpo_500_bridge/mpo_500_bridge.cpp)
target_link_libraries(mpo_500_Bridge_Node ${catkin_LIBRARIES})
add_dependencies(mpo_500_Bridge_Node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(prbt_Bridge_Node 
	src/prbt_bridge/prbt_bridge_Node.cpp 
	src/prbt_bridge/prbt_bridge.cpp)
target_link_libraries(prbt_Bridge_Node ${catkin_LIBRARIES})
add_dependencies(prbt_Bridge_Node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

##################################################################################################

