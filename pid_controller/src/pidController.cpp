#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>
#include "std_msgs/Bool.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <action_controller/MultiDofFollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseStamped.h"
#include "pidManager.cpp"

class PidController{
private:
	typedef actionlib::ActionServer<action_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
	typedef ActionServer::GoalHandle GoalHandle;
public:
	PidController(ros::NodeHandle &node) :
		node_(node),
		pidManager_(node),
		action_server_(node_, "multi_dof_joint_trajectory_action",
				boost::bind(&PidController::goalCB, this, _1),
				boost::bind(&PidController::cancelCB, this, _1),
				false),
				has_active_goal_(false)
{
	created=false;

	action_server_.start();

	std::string ns;
	node.param<std::string>("pid_controller_namespace", ns, "pid_controller");
	ns = ns + "/";

	node.param<std::string>("pid_enable_topic_name", enableTopicName, ns + "pid_enable");

	node.param<std::string>("pid_x_desired_topic_name", xDesiredTopicName, ns + "x_desired");
	node.param<std::string>("pid_y_desired_topic_name", yDesiredTopicName, ns + "y_desired");
	node.param<std::string>("pid_z_desired_topic_name", zDesiredTopicName, ns + "z_desired");
	node.param<std::string>("pid_z_desired_angle_topic_name", zDesiredAngleTopicName, ns + "z_desired_angle");

        pidEnable = node_.advertise<std_msgs::Bool>(enableTopicName, 1);
        xDesiredPublisher = node_.advertise<std_msgs::Float64>(xDesiredTopicName, 1);
        yDesiredPublisher = node_.advertise<std_msgs::Float64>(yDesiredTopicName, 1);
        zDesiredPublisher = node_.advertise<std_msgs::Float64>(zDesiredTopicName, 1);
        zDesiredAnglePublisher = node_.advertise<std_msgs::Float64>(zDesiredAngleTopicName, 1);
	
	pidManager_.initPublishersAndSubscribers(node);

	ROS_INFO_STREAM("Node ready!");


}

	~PidController()
{
	if (has_active_goal_)
	{
	    cancelCB(active_goal_);
	}
}

private:
	ros::NodeHandle node_;
	ActionServer action_server_;
	PidManager pidManager_;

	geometry_msgs::Transform_<std::allocator<void> > lastPosition;
	pthread_t trajectoryExecutor;
	bool created;
	tf::TransformListener listener;

	ros::Publisher pidEnable;
	ros::Publisher xDesiredPublisher;
	ros::Publisher yDesiredPublisher;
	ros::Publisher zDesiredPublisher;
	ros::Publisher zDesiredAnglePublisher;

	std::string enableTopicName;
	std::string xDesiredTopicName;
	std::string yDesiredTopicName;
	std::string zDesiredTopicName;
	std::string zDesiredAngleTopicName;

	double desiredX;
	double desiredY;
	double desiredZ;
	double desiredYaw;

	bool has_active_goal_;
	GoalHandle active_goal_;
	trajectory_msgs::MultiDOFJointTrajectory_<std::allocator<void> > toExecute;

	double calculateRotationConstant(double rotation) 
	{ 
		ROS_INFO_STREAM("Rotation constant setting to " << rotation );
		return rotation;
	}

	double calculateMovementConstant(double move) 
	{ 
		ROS_INFO_STREAM("Move constant setting to " << move );
		return move;
	}

	void cancelCB(GoalHandle gh){
		if (active_goal_ == gh)
		{
			// Stops the controller.
			if(created){
				ROS_INFO_STREAM("Stop thread");
				pthread_cancel(trajectoryExecutor);
				created=false;
			}
			std_msgs::Bool toPublish;
			toPublish.data = false;
			pidEnable.publish(toPublish);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}
	}

	void goalCB(GoalHandle gh){
		if (has_active_goal_)
		{
			// Stops the controller.
			if(created){
				pthread_cancel(trajectoryExecutor);
				created=false;
			}
			std_msgs::Bool toPublish;
			toPublish.data = false;
			pidEnable.publish(toPublish);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}

		gh.setAccepted();
		active_goal_ = gh;
		has_active_goal_ = true;
		toExecute = gh.getGoal()->trajectory;

		if(pthread_create(&trajectoryExecutor, NULL, threadWrapper, this)==0){
			created=true;
			ROS_INFO_STREAM("Thread for trajectory execution created");
		} else {
			ROS_INFO_STREAM("Thread creation failed!");
		}

	}

	static void* threadWrapper(void* arg) {
		PidController* mySelf = (PidController*)arg;
		mySelf->executeTrajectory();
		return NULL;
	}

	void executeTrajectory()
        {
	    ROS_INFO("----------->Executing trajectory with %d points on joint %s.", (int)toExecute.points.size(), toExecute.joint_names[0].c_str());

	    ros::Rate loop_rate(10);
	    geometry_msgs::PoseStamped pose_in;
	    if(toExecute.joint_names[0]=="Base" && toExecute.points.size()>0)
	    {
		std_msgs::Bool toPublish;
		toPublish.data = true;
		pidEnable.publish(toPublish);

		int lastIndex =toExecute.points.size()-1 ;
		for(int k = 0; k <= lastIndex; k++)
		{
		    geometry_msgs::Transform_<std::allocator<void> > transform = toExecute.points[k].transforms[0];

		    pose_in.header.frame_id = "odom";

		    pose_in.pose.position.x = transform.translation.x;
		    pose_in.pose.position.y = transform.translation.y;
		    pose_in.pose.position.z = transform.translation.z;

		    pose_in.pose.orientation = transform.rotation;

    		    if (k == lastIndex)
		    {
			ROS_INFO("----------->Executing last point (hover).");
		    }
		
		    // inits the desired positions    
		    while(!publishActualDesiredState(pose_in, k))
		    {
			loop_rate.sleep();
			pidManager_.publish();
		    }

		    // waits until close to the desired pose
		    while(!isAllCloseToZero(desiredX, desiredY, desiredZ, desiredYaw) && k != lastIndex)
		    {
			publishActualDesiredState(pose_in, k);
			loop_rate.sleep();
			pidManager_.publish();
		    }

		    // hovers over the last pose
		    int counter = 0;
		    while(k == lastIndex && counter++ < 1000)
		    {
			publishActualDesiredState(pose_in, k);
			pidManager_.publish();
			loop_rate.sleep();
		    }
		}
	    }
	    ROS_INFO("----------->Trajectory executed.");

	    std_msgs::Bool toPublish;
	    toPublish.data = false;
	    pidEnable.publish(toPublish);

	    active_goal_.setSucceeded();
	    has_active_goal_=false;
	    created=false;
	}

	bool isAllCloseToZero(double desX, double desY, double desZ, double desYaw)
	{
	    if(!isValueCloseToZero(desX, 0.1))
		return false;

	    if(!isValueCloseToZero(desY, 0.1))
		return false;

	    if(!isValueCloseToZero(desZ, 0.4))
		return false;

	    //if(!isValueCloseToZero(desYaw))
		//return false;

	    return true;
	}

	bool isValueCloseToZero(double value, double limit)
	{
	    if (value > 0 && value > limit)
		return false;

	    if (value < 0 && value < -limit)
		return false;

	    return true;
	}

	bool publishActualDesiredState(geometry_msgs::PoseStamped &pose_in, int k)
	{
	    pose_in.header.stamp = ros::Time::now() - ros::Duration(0.30);
	    geometry_msgs::PoseStamped pose_out;
	    try
	    {
		listener.transformPose("base_link", pose_in, pose_out);
	    }
	    catch (tf::TransformException ex)
	    {
  		ROS_ERROR("%s",ex.what());
		return false;
	    }

	    tf::Quaternion rotation(pose_out.pose.orientation.x, pose_out.pose.orientation.y, pose_out.pose.orientation.z, pose_out.pose.orientation.w);
	    double roll, pitch;
	    tf::Matrix3x3 m(rotation);
	    m.getRPY(roll, pitch, desiredYaw);

	    desiredX = pose_out.pose.position.x;
	    desiredY = pose_out.pose.position.y;
	    desiredZ = pose_out.pose.position.z;

	    ROS_INFO("----------->Executing point with index %d \n odom [%f, %f, %f] \n base_link [%f, %f, %f] with %f rads.", (int)k, 
		pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z,
		desiredX, desiredY, desiredZ, desiredYaw);

	    xDesiredPublisher.publish(desiredX);
	    yDesiredPublisher.publish(desiredY);
	    zDesiredPublisher.publish(desiredZ);
	    zDesiredAnglePublisher.publish(desiredYaw);

	    return true;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pid_controller");
	ros::NodeHandle node;

	PidController control(node);

	ros::spin();

	return 0;
}
