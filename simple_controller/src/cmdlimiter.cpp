#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>

class CmdLimiter
{
    public:
    geometry_msgs::Twist getLimitedCommand(geometry_msgs::Twist cmd)
    { 
	geometry_msgs::Twist toReturn;
	toReturn.linear = cmd.linear;
	toReturn.angular = cmd.angular;

	if (toReturn.linear.x > movementLimit)
	    toReturn.linear.x = movementLimit;

	if (toReturn.linear.x < -movementLimit)
	    toReturn.linear.x = -movementLimit;

	if (toReturn.linear.y > movementLimit)
	    toReturn.linear.y = movementLimit;

	if (toReturn.linear.y < -movementLimit)
	    toReturn.linear.y = -movementLimit;

	if (toReturn.angular.z > rotationLimit)
	    toReturn.angular.z = rotationLimit;

	if (toReturn.angular.z < -rotationLimit)
	    toReturn.angular.z = -rotationLimit;

	return toReturn;
    }

    CmdLimiter(double moveLim, double rotLim) 
    {
	movementLimit = std::min( 1.0, std::abs(moveLim) );
	rotationLimit = std::min( 1.0, std::abs(rotLim) );
    }

    private:
    double movementLimit;
    double rotationLimit;
};
