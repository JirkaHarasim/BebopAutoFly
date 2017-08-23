#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>

class CmdCompensator
{
    public:
    double compensateX(double x)
    {
	if (shouldCompensate())
	    return makeValid(x + compensationX);
	
	return x;
    }

    double compensateY(double y)
    {
	if (shouldCompensate())
	    return makeValid(y + compensationY);

	return y;
    }

    double compensateZ(double z)
    {
	if (shouldCompensate())
	    return makeValid(z + compensationZ);

	return z;
    }

    double compensateRot(double rot)
    {
	if (shouldCompensate())
	    return makeValid(rot + compensationRot);

	return rot;
    }

    CmdCompensator(double compX, double compY, double compZ, double compRot) 
    {
	compensationX = compX;
	compensationY = compY;
	compensationZ = compZ;
	compensationRot = compRot;

	milisecondsNotCompensating = 66;
    }

    private:
    double compensationX;
    double compensationY;
    double compensationZ;
    double compensationRot;

    int milisecondsNotCompensating;

    double makeValid(double in)
    {
	if (in > 1)
	    return 1;

	if (in < -1)
	    return -1;

	return in;
    }

    bool shouldCompensate()
    {
	ros::Time stamp = ros::Time::now();
	int val = (long)(( stamp.toSec()-(long)stamp.toSec() ) * 100);
	
	if (val > milisecondsNotCompensating)
	    return true;

	return false;
    }
};
