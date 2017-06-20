# BebopAutoFly
ROS launch files / settings to fly bebop autonomously in simulator and in real environment.

Prerequisities:
   - testing repository on branch odometryModifications https://github.com/fairf4x/testing.git

   - bebop\_autonomy repository https://github.com/AutonomyLab/bebop\_autonomy.git with imported bebop urdf from BebopAutoFly repo (replace the bebop_description/urdf file)

   - gazebo and rviz

   - moveit! working and integrated with rviz https://github.com/ros-planning/moveit.git

   - dso compiled and working https://github.com/JakobEngel/dso.git, dso\_ros on address https://github.com/fairf4x/dso_ros.git, branch pcl.
     There is a bug in dso interface to ros, which causes the call of the method 'publishCamPose' to have corrupt data and makes it effectively unusable. I used a  workaround to fix that, since I don't have the rights for the dso repository. 
Recompile the dso with the publishCamPose method edited to this (also don't forget to update all implementations of this interface): 
virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib, Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4>* transformation) {}

update the FullSystem.cpp call of this method with this call:
Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4> transformation = frame->camToWorld.matrix3x4();
	    Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4>* toPass = &transformation;
            ow->publishCamPose(frame, &Hcalib, toPass);

   - Autonomous-Flight-ROS (containing the controller and used as a starting point for integration of all pieces together)
https://github.com/AlessioTonioni/Autonomous-Flight-ROS.git. I have an updated controller for the bebop, which is modified to use parameters for setup instead of inbuild constants in this project and I will publish it later. In general, this page uses the same project as a starting point and contains good setup instructions: https://www.wilselby.com/research/ros-integration/3d-mapping-navigation/
It is using GPS to navigate, but the setup instructions are very useful.

   - yos\_cmd\_vel\_mux subproject of this project https://github.com/yujinrobot/yujin_ocs.git which is used to prioritize the automatic controller over the gamepad I am using. Only the yos\_cmd\_vel\_mux subproject is needed, feel free to delete everything else. Use parameter files in the "cmd vel mux params" directory.



To run the simulated drone, execute the scripts in the sh_files directory in order 1-simulator, 2-controller, 3-planner.
To run the bag file recorded with previous flight, execute the scripts 5-realBag, 2-controller, 6-realPlanner.
To fly with the actual bebop, execute 4-real, 2-controller, 6-realPlanner.

