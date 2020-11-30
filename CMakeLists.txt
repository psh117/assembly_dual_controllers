cmake_minimum_required(VERSION 2.8.3)
project(assembly_dual_controllers)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE RELEASE)

add_definitions(-DUSE_REAL_ROBOT_INTERFACE)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")


find_package(RBDL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  controller_interface
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  ##pluginlibcf
  realtime_tools
  roscpp
  rospy
  roslib
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
  assembly_msgs
  tf
  tf_conversions
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
# find_package(Franka 0.5.0 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  dynamic_reconfigure
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  # pluginlib
  realtime_tools
  roscpp
  rospy
  roslib
  assembly_msgs
  DEPENDS Franka
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIR}
)

add_library(assembly_controller_common_lib
  src/servers/joint_trajectory_action_server.cpp
  src/servers/assemble_approach_action_server.cpp
  src/servers/assemble_insert_action_server.cpp
  src/servers/assemble_spiral_action_server.cpp
  src/servers/assemble_verify_action_server.cpp
  src/servers/idle_control_server.cpp
  src/servers/assemble_move_action_server.cpp
  # src/servers/assemble_press_action_server.cpp
  # src/servers/assemble_side_chair_action_server.cpp
  src/servers/assemble_rotation_action_server.cpp
  src/servers/assemble_triple_recovery_action_server.cpp
  src/servers/assemble_approach_bolt_action_server.cpp
  src/servers/assemble_dual_spiral_action_server.cpp
  src/servers/assemble_dual_approach_action_server.cpp
  src/servers/assemble_probe_edge_action_server.cpp
  src/servers/assemble_triple_move_action_server.cpp
#  src/servers/assemble_dual_side_chair_recovery_action_server.cpp
  src/servers/assemble_retreat_bolt_action_server.cpp
  src/servers/task_space_move_action_server.cpp
  src/model/franka_model_updater.cpp
  src/utils/control/peg_in_hole_base.cpp
  src/utils/control/criteria.cpp
  src/utils/dyros_math.cpp
  src/servers/assemble_approach_hip_action_server.cpp
  src/servers/assemble_kitting_action_server.cpp
  src/servers/assemble_back_forth_action_server.cpp
  src/servers/assemble_bolting_ready_action_server.cpp
  src/model/franka_panda_model.cpp

)

add_library(${PROJECT_NAME}
  src/assembly_single_controller.cpp
  src/assembly_dual_controller.cpp
  src/assembly_triple_controller.cpp
 )


add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(assembly_controller_common_lib PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}

  ${RBDL_LIBRARY}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  ${RBDL_LIBRARY}
  assembly_controller_common_lib
)




#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
