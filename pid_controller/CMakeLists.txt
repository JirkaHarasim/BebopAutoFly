cmake_minimum_required(VERSION 2.8.3)
project(pid_controller)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)

include_directories(SYSTEM "msg_gen/cpp/include")

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  trajectory_msgs
  actionlib_msgs
  std_msgs
  message_generation
	tf
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
  FILES
  MultiDofFollowJointTrajectoryFeedback.msg
  MultiDofFollowJointTrajectoryGoal.msg
  MultiDofFollowJointTrajectoryResult.msg
  MultiDofFollowJointTrajectoryActionFeedback.msg
  MultiDofFollowJointTrajectoryActionGoal.msg
  MultiDofFollowJointTrajectoryActionResult.msg
  MultiDofFollowJointTrajectoryAction.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  actionlib_msgs
  action_controller
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
  CATKIN_DEPENDS message_runtime
  moveit_core
)

include_directories(include)


#add_executable(pid_manager src/pidManager.cpp)
add_executable(pid_controller src/pidController.cpp)

#target_link_libraries(pid_manager ${catkin_LIBRARIES})
target_link_libraries(pid_controller ${catkin_LIBRARIES})

#add_dependencies(pid_manager pidmanager_gencpp)
add_dependencies(pid_controller pidcontroller_gencpp)





