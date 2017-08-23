cmake_minimum_required(VERSION 2.8.3)
project(moveit_multi_dof_plans)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	nav_msgs
	std_msgs
	geometry_msgs
	moveit_msgs
	roscpp
	rospy
	tf
	message_generation
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools)

find_package(Eigen3 REQUIRED)

add_service_files(DIRECTORY srv
    FILES
    GetRobotTrajectoryFromPath.srv
    TransformTrajectory.srv
    InverseTransformTrajectory.srv
#    NOINSTALL
    )

  generate_messages(DEPENDENCIES std_msgs geometry_msgs moveit_msgs nav_msgs)
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    moveit_msgs
    roscpp)

include_directories(
	${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR})

add_executable(frodopathy src/frodopath.cpp)
add_executable(robottrajectory src/robottrajectoryfrompathservice.cpp)
add_executable(trajectorytransformer src/trajectorytransformer.cpp)
add_executable(backtracker src/backtracker.cpp)
add_executable(boomerang src/boomerang.cpp)
add_executable(backandforth src/backandforth.cpp)
add_executable(square src/square.cpp)
add_executable(rotation src/rotation.cpp)

target_link_libraries(frodopathy ${catkin_LIBRARIES})
target_link_libraries(robottrajectory ${catkin_LIBRARIES})
target_link_libraries(trajectorytransformer ${catkin_LIBRARIES})
target_link_libraries(backtracker ${catkin_LIBRARIES})
target_link_libraries(boomerang ${catkin_LIBRARIES})
target_link_libraries(backandforth ${catkin_LIBRARIES})
target_link_libraries(square ${catkin_LIBRARIES})
target_link_libraries(rotation ${catkin_LIBRARIES})

add_dependencies(frodopathy frodopathy_gencpp)
add_dependencies(backtracker backtracker_gencpp)
add_dependencies(boomerang boomerang_gencpp)
add_dependencies(backandforth backandforth_gencpp)
add_dependencies(square square_gencpp)
add_dependencies(rotation rotation_gencpp)

add_dependencies(robottrajectory robottrajectory_gencpp)
add_dependencies(trajectorytransformer trajectorytransformer_gencpp)
add_dependencies(robottrajectory moveit_multi_dof_plans_generate_messages_cpp)
add_dependencies(trajectorytransformer moveit_multi_dof_plans_generate_messages_cpp)

