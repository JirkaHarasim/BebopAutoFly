#!/bin/bash
roslaunch bebop_gazebo plan_and_execute.launch use_ground_truth_for_tf:=false &
sleep 5
rosrun dynamic_reconfigure dynparam set /move_group/trajectory_execution execution_duration_monitoring false
