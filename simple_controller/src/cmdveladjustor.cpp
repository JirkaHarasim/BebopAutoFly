#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "cmdlimiter.cpp"
#include "cmdcompensator.cpp"
#include "cmdbooster.cpp"
#include <string>

class CmdVelAdjustor
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

    CmdVelAdjustor(double compX, double compY, double compZ, double compRot, double movLim, double rotLim) 
	: limiter_(movLim, rotLim),
	  compensator_(compX, compY, compZ, compRot),
	  booster_(compX, compY, movLim)
    {
	newMessage = false;
    }

    void publishIfNew()
    {
	if (newMessage)
	{
	    actualMessage = limiter_.getLimitedCommand(actualMessage);

	    
	    booster_.resolveBoosting(actualMessage);
	
	    if (booster_.isBoostingX())
	    {
		actualMessage.linear.x = booster_.getBoostX();
	    }
	    else
	    {
		actualMessage.linear.x = compensator_.compensateX(actualMessage.linear.x);
	    }

	    if (booster_.isBoostingY())
	    {
		actualMessage.linear.y = booster_.getBoostY();
	    }
	    else
	    {
		actualMessage.linear.y = compensator_.compensateY(actualMessage.linear.y);
	    }


	    actualMessage.linear.z = compensator_.compensateZ(actualMessage.linear.z);
	    actualMessage.angular.z = compensator_.compensateRot(actualMessage.angular.z);

	    cmdVelPublisher.publish(actualMessage);
            newMessage = false;
	}
    }

    private:
    ros::Publisher cmdVelPublisher;
    ros::Subscriber cmdVelSubscriber;

    CmdLimiter limiter_;
    CmdCompensator compensator_;
    CmdBooster booster_;

    geometry_msgs::Twist actualMessage;
    bool newMessage;

    void initSubscriber(ros::NodeHandle n, std::string sourceTopicName)
    {
        cmdVelSubscriber = n.subscribe(sourceTopicName, 50, &CmdVelAdjustor::callback, this);
    }

    void initPublisher(ros::NodeHandle n, std::string outputTopicName)
    {
        cmdVelPublisher = n.advertise<geometry_msgs::Twist>(outputTopicName, 50); 
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "cmdVelAdjustor");
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

    double compensationX;
    double compensationY;
    double compensationZ;
    double compensationRot;
    n.param<double>("compensation_x", compensationX, 0.0);
    n.param<double>("compensation_y", compensationY, 0.0);
    n.param<double>("compensation_z", compensationZ, 0.0);
    n.param<double>("compensation_rot", compensationRot, 0.0);

    double movementLimit;
    double rotationLimit;
    n.param<double>("cmd_vel_limit", movementLimit, 0.3);
    n.param<double>("cmd_vel_rot_limit", rotationLimit, 0.3);

    CmdVelAdjustor adjustor(compensationX, compensationY, compensationZ, compensationRot, movementLimit, rotationLimit);

    if(argumentsPassed)
    {
        ROS_INFO("Got params: %s, %s", sourceTopicName.c_str(), outputTopicName.c_str());
	adjustor.initPublisherAndSubscriber(n, sourceTopicName, outputTopicName);
    }
    else
    {
        ROS_INFO("Got no params, using /cmd_vel_to_compensate and /compensator/cmd_vel.");
	adjustor.initPublisherAndSubscriber(n, "/cmd_vel_to_compensate", "/compensator/cmd_vel");
    }

    ros::Rate loop_rate(20);

    while(ros::ok())
    {
        adjustor.publishIfNew();
        ros::spinOnce();
        loop_rate.sleep();    
    }

    return 0;
}
