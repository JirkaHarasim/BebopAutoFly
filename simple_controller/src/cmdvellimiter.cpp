#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "cmdlimiter.cpp"
#include "cmdcompensator.cpp"
#include "cmdbooster.cpp"
#include <string>

class CmdVelLimiter
{
    public:
    void callback(const geometry_msgs::Twist::ConstPtr& twist)
    { 
        newMessage = true;
	actualMessage.linear = twist->linear;
	actualMessage.angular = twist->angular;
    }

    void initPublisherAndSubscriber(ros::NodeHandle n, std::string sourceTopicName, std::string outputTopicName)
    {
	initSubscriber(n, sourceTopicName);
        initPublisher(n, outputTopicName);
    }

    CmdVelLimiter(double movLim, double rotLim) 
	: limiter_(movLim, rotLim)
    {
	newMessage = false;
    }

    void publishIfNew()
    {
	if (newMessage)
	{
	    actualMessage = limiter_.getLimitedCommand(actualMessage);

	    cmdVelPublisher.publish(actualMessage);
            newMessage = false;
	}
    }

    private:
    ros::Publisher cmdVelPublisher;
    ros::Subscriber cmdVelSubscriber;

    CmdLimiter limiter_;

    geometry_msgs::Twist actualMessage;
    bool newMessage;

    void initSubscriber(ros::NodeHandle n, std::string sourceTopicName)
    {
        cmdVelSubscriber = n.subscribe(sourceTopicName, 50, &CmdVelLimiter::callback, this);
    }

    void initPublisher(ros::NodeHandle n, std::string outputTopicName)
    {
        cmdVelPublisher = n.advertise<geometry_msgs::Twist>(outputTopicName, 50); 
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "cmdVelLimiter");
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

    double movementLimit;
    double rotationLimit;
    n.param<double>("cmd_vel_limit", movementLimit, 0.3);
    n.param<double>("cmd_vel_rot_limit", rotationLimit, 0.3);

    CmdVelLimiter limiter(movementLimit, rotationLimit);

    if(argumentsPassed)
    {
        ROS_INFO("Got params: %s, %s", sourceTopicName.c_str(), outputTopicName.c_str());
	limiter.initPublisherAndSubscriber(n, sourceTopicName, outputTopicName);
    }
    else
    {
        ROS_INFO("Got no params, using /cmd_vel_to_limit and /limit/cmd_vel.");
	limiter.initPublisherAndSubscriber(n, "/cmd_vel_to_limit", "/limit/cmd_vel");
    }

    ros::Rate loop_rate(20);

    while(ros::ok())
    {
        limiter.publishIfNew();
        ros::spinOnce();
        loop_rate.sleep();    
    }

    return 0;
}
