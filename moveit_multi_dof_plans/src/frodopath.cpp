#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include <string>

class Frodopathy
{
    public:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
    { 
	ROS_INFO("I heard: [%s]", odom->header.frame_id.c_str());

        if (count != 0 and areTooSimilar(odom->pose.pose, path.poses[path.poses.size()-1].pose))
            return;

	path.header.seq = ++count;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "odom";

	geometry_msgs::PoseStamped poses;
	poses.header = odom->header;
	poses.pose = odom->pose.pose;

	path.poses.push_back(poses);

	newMessage = true;
    }

    void initPublisherAndSubscriber(ros::NodeHandle n)
    {
	initSubscriber(n, "/bebop/odom");
        initPublisher(n, "/bebop/frodopathy");
    }

    void initPublisherAndSubscriber(ros::NodeHandle n, std::string sourceTopicName, std::string outputTopicName)
    {
	initSubscriber(n, sourceTopicName);
        initPublisher(n, outputTopicName);
    }

    Frodopathy(double limit) 
    { 
        newMessage = false;
	count = 0;
        treshold = limit;
    }

    void publishIfNew()
    {
	if (newMessage)
	{
	    frodopathyPublisher.publish(path);
	    newMessage = false;
	}
    }

    private:
    ros::Publisher frodopathyPublisher;
    ros::Subscriber odometrySubscriber;

    nav_msgs::Path path;
    bool newMessage;
    int count;
    double treshold;

    void initSubscriber(ros::NodeHandle n, std::string sourceTopicName)
    {
        odometrySubscriber = n.subscribe(sourceTopicName, 50, &Frodopathy::odomCallback, this);
    }

    void initPublisher(ros::NodeHandle n, std::string outputTopicName)
    {
        frodopathyPublisher = n.advertise<nav_msgs::Path>(outputTopicName, 50); 
    }

    bool areTooSimilar(geometry_msgs::Pose first, geometry_msgs::Pose second)
    {
        if (std::abs(first.position.x - second.position.x) > treshold)
            return false;

        if (std::abs(first.position.y - second.position.y) > treshold)
            return false;

        if (std::abs(first.position.z - second.position.z) > treshold)
            return false;

        return true;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "frodopath");
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

    double frodoLimit;
    n.param<double>("frodo_limit", frodoLimit, 0.05);

    Frodopathy frodo(frodoLimit);

    if(argumentsPassed)
    {
        ROS_INFO("Got params: %s, %s", sourceTopicName.c_str(), outputTopicName.c_str());
	frodo.initPublisherAndSubscriber(n, sourceTopicName, outputTopicName);
    }
    else
    {
        ROS_INFO("Got no params, using /bebop/odom and /bebop/frodopathy.");
	frodo.initPublisherAndSubscriber(n);
    }

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        frodo.publishIfNew();
        ros::spinOnce();
        loop_rate.sleep();    
    }

    return 0;
}
