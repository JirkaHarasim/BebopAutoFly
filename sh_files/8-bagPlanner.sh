#!/bin/bash
roslaunch bebop_model plan_and_execute_bag.launch &
sleep 5
rosrun dynamic_reconfigure dynparam set /move_group/trajectory_execution execution_duration_monitoring false
