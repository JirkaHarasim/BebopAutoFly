#!/bin/bash
roslaunch bebop_gazebo plan_and_execute_dso.launch &
sleep 5
rosrun dynamic_reconfigure dynparam set /move_group/trajectory_execution execution_duration_monitoring false
