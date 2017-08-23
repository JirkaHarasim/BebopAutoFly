#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>

class CmdVelRepeater
{
    public:
    void callback(const geometry_msgs::Twist::ConstPtr& twist)
    { 
	count++;
	lastTime = ros::Time::now();

	lastMessage.linear = twist->linear;
	lastMessage.angular = twist->angular;
    }

    void initPublisherAndSubscriber(ros::NodeHandle n, std::string sourceTopicName, std::string outputTopicName)
    {
	initSubscriber(n, sourceTopicName);
        initPublisher(n, outputTopicName);
    }

    CmdVelRepeater() 
    {
	count = 0;
    }

    void publishIfValid()
    {
	if (!lastMessageIsEmpty() or ( lastTime + ros::Duration(1) > ros::Time::now() ))
	{
	    cmdVelPublisher.publish(lastMessage);
	}
    }

    private:
    ros::Publisher cmdVelPublisher;
    ros::Subscriber cmdVelSubscriber;

    geometry_msgs::Twist lastMessage;
    int count;
    ros::Time lastTime;

    void initSubscriber(ros::NodeHandle n, std::string sourceTopicName)
    {
        cmdVelSubscriber = n.subscribe(sourceTopicName, 50, &CmdVelRepeater::callback, this);
    }

    void initPublisher(ros::NodeHandle n, std::string outputTopicName)
    {
        cmdVelPublisher = n.advertise<geometry_msgs::Twist>(outputTopicName, 50); 
    }

    bool lastMessageIsEmpty()
    {
	if (lastMessage.linear.x != 0)
	    return false;

	if (lastMessage.linear.y != 0)
	    return false;

	if (lastMessage.linear.z != 0)
	    return false;

	if (lastMessage.angular.x != 0)
	    return false;

	if (lastMessage.angular.y != 0)
	    return false;

	if (lastMessage.angular.z != 0)
	    return false;

	return true;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "cmdVelRepeater");
    ros::NodeHandle n;

    ROS_INFO("Have %i arguments passed.", argc);

    std::string sourceTopicName;
    std::string outputTopicName;
    bool argumentsPassed = false;

    if (argc == 3)
    {
    	sourceTopicName = argv[1];
	outputTopicName = argv[2];
	argumentsPassed = true;
    } 

    CmdVelRepeater repeater;

    if(argumentsPassed)
    {
        ROS_INFO("Got params: %s, %s", sourceTopicName.c_str(), outputTopicName.c_str());
	repeater.initPublisherAndSubscriber(n, sourceTopicName, outputTopicName);
    }
    else
    {
        ROS_INFO("Got no params, using /cmd_vel_to_repeat and /repeater/cmd_vel.");
	repeater.initPublisherAndSubscriber(n, "/cmd_vel_to_repeat", "/repeater/cmd_vel");
    }

    ros::Rate loop_rate(20);

    while(ros::ok())
    {
        repeater.publishIfValid();
        ros::spinOnce();
        loop_rate.sleep();    
    }

    return 0;
}
