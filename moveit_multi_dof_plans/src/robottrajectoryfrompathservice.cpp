#include "ros/ros.h"
#include "testing/GetRobotTrajectoryFromPath.h"

class RobotTrajectoryFromPath
{
public:
  bool getRobotTrajectoryFromPath(testing::GetRobotTrajectoryFromPath::Request& req,
                                  testing::GetRobotTrajectoryFromPath::Response& res)
  {
    moveit_msgs::RobotTrajectory robotTrajectoryMsg;
    trajectory_msgs::MultiDOFJointTrajectory multi;

    ROS_INFO("Received call with %d poses.", (int)req.path.poses.size());

    multi.header.frame_id = req.path.header.frame_id;
    multi.joint_names = req.joint_names;

    if (req.path.poses.size() <= 0)
    {
	ROS_ERROR("Trajectory from path service recieved invalid input.");
	return false;
    }

    firstMessageTime = req.path.poses[0].header.stamp;

    for(int i = 0; i < req.path.poses.size(); i++)
    {
	multi.points.push_back(getMultiFromPoseStamped(req.path.poses[i]));
    }

    robotTrajectoryMsg.multi_dof_joint_trajectory = multi;
    res.trajectory = robotTrajectoryMsg;

    return true;
  }

private:
  ros::Time firstMessageTime;

  trajectory_msgs::MultiDOFJointTrajectoryPoint getMultiFromPoseStamped(geometry_msgs::PoseStamped pose)
  {
     trajectory_msgs::MultiDOFJointTrajectoryPoint toReturn;
     toReturn.time_from_start = pose.header.stamp - firstMessageTime;

     geometry_msgs::Transform transform;
     transform.translation.x = pose.pose.position.x;
     transform.translation.y = pose.pose.position.y;
     transform.translation.z = pose.pose.position.z;
     transform.rotation = pose.pose.orientation;

     toReturn.transforms.push_back(transform);

     return toReturn;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_trajectory_from_path_server");
  ros::NodeHandle n;

  RobotTrajectoryFromPath server;
  ros::ServiceServer ss = n.advertiseService("get_robot_trajectory_from_path", &RobotTrajectoryFromPath::getRobotTrajectoryFromPath, &server);

  ros::spin();

  return 0;
}
