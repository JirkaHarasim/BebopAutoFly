#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "tf/transform_datatypes.h"
#include <string>

class PidManager
{
    public:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
    { 
	tf::Quaternion rotation(odom->pose.pose.orientation.x,
				odom->pose.pose.orientation.y,
				odom->pose.pose.orientation.z,
				odom->pose.pose.orientation.w);
        double roll, pitch, yaw;
	tf::Matrix3x3 m(rotation);
	m.getRPY(roll, pitch, yaw);

	xOdomPublisher.publish(odom->pose.pose.position.x);
	yOdomPublisher.publish(odom->pose.pose.position.y);
	zOdomPublisher.publish(odom->pose.pose.position.z);
	zOdomAnglePublisher.publish(yaw);

	ROS_INFO("Actual pose and angle was [%f, %f, %f] with %f rads.", odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z, yaw);
    }

    void xCallback(const std_msgs::Float64::ConstPtr& xCommand)
    { 
	commandToSend.linear.x = xCommand->data;
    }

    void yCallback(const std_msgs::Float64::ConstPtr& yCommand)
    { 
	commandToSend.linear.y = yCommand->data;
    }

    void zCallback(const std_msgs::Float64::ConstPtr& zCommand)
    { 
	commandToSend.linear.z = zCommand->data;
    }

    void zAngleCallback(const std_msgs::Float64::ConstPtr& zAngleCommand)
    { 
	commandToSend.angular.z = zAngleCommand->data;
    }

    void initPublishersAndSubscribers(ros::NodeHandle n)
    {
	initSubscribers(n);
        initPublishers(n);
    }

    PidManager(ros::NodeHandle node) 
    {
	std::string ns;
	node.param<std::string>("pid_manager_namespace", ns, "pid_manager");
	ns = ns + "/";

	node.param<std::string>("pid_odom_subscribe_topic", odomSubscribeTopic, ns + "odom");
	node.param<std::string>("pid_x_twist_subscribe_topic", xTwistSubscribeTopic, ns + "x_twist");
	node.param<std::string>("pid_y_twist_subscribe_topic", yTwistSubscribeTopic, ns + "y_twist");
	node.param<std::string>("pid_z_twist_subscribe_topic", zTwistSubscribeTopic, ns + "z_twist");
	node.param<std::string>("pid_z_twist_angle_subscribe_topic", zTwistAngleSubscribeTopic, ns + "z_angle_twist");

	node.param<std::string>("pid_twist_publish_topic", twistPublishTopic, ns + "cmd_vel");
	node.param<std::string>("pid_x_odom_publish_topic", xOdomPublishTopic, ns + "x_odom");
	node.param<std::string>("pid_y_odom_publish_topic", yOdomPublishTopic, ns + "y_odom");
	node.param<std::string>("pid_z_odom_publish_topic", zOdomPublishTopic, ns + "z_odom");
	node.param<std::string>("pid_z_odom_angle_publish_topic", zOdomAnglePublishTopic, ns + "z_angle_odom");

	commandToSend.linear.x = 0;
	commandToSend.linear.y = 0;
	commandToSend.linear.z = 0;

	commandToSend.angular.x = 0;
	commandToSend.angular.y = 0;
	commandToSend.angular.z = 0;
    }

    void publish()
    {
	ROS_INFO("Publishing [%f, %f, %f, %f].", commandToSend.linear.x, commandToSend.linear.y, commandToSend.linear.z, commandToSend.angular.z);
	twistPublisher.publish(commandToSend);
    }

    geometry_msgs::Twist getLastState()
    {
	return commandToSend;
    }

    private:
    ros::Publisher twistPublisher;
    ros::Publisher xOdomPublisher;
    ros::Publisher yOdomPublisher;
    ros::Publisher zOdomPublisher;
    ros::Publisher zOdomAnglePublisher;

    ros::Subscriber odomSubscriber;
    ros::Subscriber xTwistSubscriber;
    ros::Subscriber yTwistSubscriber;
    ros::Subscriber zTwistSubscriber;
    ros::Subscriber zTwistAngleSubscriber;

    geometry_msgs::Twist commandToSend;

    std::string odomSubscribeTopic;
    std::string xTwistSubscribeTopic;
    std::string yTwistSubscribeTopic;
    std::string zTwistSubscribeTopic;
    std::string zTwistAngleSubscribeTopic;

    std::string twistPublishTopic;
    std::string xOdomPublishTopic;
    std::string yOdomPublishTopic;
    std::string zOdomPublishTopic;
    std::string zOdomAnglePublishTopic;

    void initSubscribers(ros::NodeHandle n)
    {
        odomSubscriber = n.subscribe(odomSubscribeTopic, 5, &PidManager::odomCallback, this);
        xTwistSubscriber = n.subscribe(xTwistSubscribeTopic, 1, &PidManager::xCallback, this);
        yTwistSubscriber = n.subscribe(yTwistSubscribeTopic, 1, &PidManager::yCallback, this);
        zTwistSubscriber = n.subscribe(zTwistSubscribeTopic, 1, &PidManager::zCallback, this);
        zTwistAngleSubscriber = n.subscribe(zTwistAngleSubscribeTopic, 1, &PidManager::zAngleCallback, this);
    }

    void initPublishers(ros::NodeHandle n)
    {
        twistPublisher = n.advertise<geometry_msgs::Twist>(twistPublishTopic, 5); 
        xOdomPublisher = n.advertise<std_msgs::Float64>(xOdomPublishTopic, 1);
        yOdomPublisher = n.advertise<std_msgs::Float64>(yOdomPublishTopic, 1);
        zOdomPublisher = n.advertise<std_msgs::Float64>(zOdomPublishTopic, 1);
        zOdomAnglePublisher = n.advertise<std_msgs::Float64>(zOdomAnglePublishTopic, 1);
    }
};
