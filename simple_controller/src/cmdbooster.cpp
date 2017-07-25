#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>

class CmdBooster
{
    struct boostInfo
    {
	bool isBoosting;
	int duration;
	int durationLimit;
	double boostValue;
	double compensation;

	double last;
	double actual;

	std::string boostName;

	boostInfo(double comp, std::string name)
	{
	    isBoosting = false;
	    boostValue = 0;
	    duration = 0;
	    durationLimit = 0;

	    last = 0;
	    actual = 0;

	    compensation = comp;
	    boostName = name;
	}
    };

    public:
    void resolveBoosting(geometry_msgs::Twist actualMessage)
    {
	boostX_.last = boostX_.actual;
	boostX_.actual = actualMessage.linear.x;

	boostY_.last = boostY_.actual;
	boostY_.actual = actualMessage.linear.y;

	resolveBoost(&boostX_);
	resolveBoost(&boostY_);
    }

    bool isBoostingX()
    {
	return boostX_.isBoosting;
    }

    bool isBoostingY()
    {
	return boostY_.isBoosting;
    }

    double getBoostX()
    {
	return makeValid(boostX_.boostValue);
    }

    double getBoostY()
    {
	return makeValid(boostY_.boostValue);
    }

    CmdBooster(double compX, double compY, double lim) 
	: boostX_(compX, "X"),
	  boostY_(compY, "Y")
    {
	limit = lim;
    }

    private:
    geometry_msgs::Twist lastMessage;
    double limit;

    boostInfo boostX_;
    boostInfo boostY_;

    void resolveBoost(boostInfo* boost)
    {
	double diff = std::abs(boost->actual - boost->last);

	if (diff > limit * 0.6)
	{
	    if (std::abs(boost->last) >= std::abs(boost->actual) || diff >= limit * 1.5)
	    {
		handleOppositeDirection(boost, diff);
	    }

	    if (!boost->isBoosting)
	    {
		boost->isBoosting = true;  
		boost->boostValue = 1.5*boost->actual + 3*boost->compensation*std::abs(boost->actual);
		boost->durationLimit = 8 + (long) (30*boost->compensation*modifSgn(boost->actual));
		ROS_INFO("Boosting %s with %f for %d cycles. Actual is %f and last was %f.", boost->boostName.c_str(), boost->boostValue, boost->durationLimit, boost->actual, boost->last);
	    }
	}

	if (boost->isBoosting && ++boost->duration > boost->durationLimit)
	{
	    boost->isBoosting = false;
	    boost->boostValue = 0;
	    boost->duration = 0;
	    boost->durationLimit = 0;
	    ROS_INFO("End of boosting %s.", boost->boostName.c_str());
	}
    }

    void handleOppositeDirection(boostInfo* boost, double diff)
    {
	boost->isBoosting = true;
	boost->boostValue = -3*boost->last + 2*boost->compensation*std::abs(boost->last);

	if (boost->duration > 0)
	{
	    boost->durationLimit = boost->duration + (long) (-30*boost->duration*boost->compensation*modifSgn(boost->last) / boost->durationLimit);
	    boost->duration = 0;
	    ROS_INFO("Opposite boosting %s with %f for %d cycles. Actual is %f and last was %f.", boost->boostName.c_str(), boost->boostValue, boost->durationLimit, boost->actual, boost->last);
	}
	else
	{
	    if (diff < limit * 1.5)
	    {
		boost->durationLimit = 8 + (long) (-30*boost->compensation*modifSgn(boost->last));
		ROS_INFO("Opposite boosting %s with %f for %d cycles. Actual is %f and last was %f.", boost->boostName.c_str(), boost->boostValue, boost->durationLimit, boost->actual, boost->last);
	    }
	    else
	    {
		boost->durationLimit = 16 + (long) (-30*boost->compensation*modifSgn(boost->last));
		ROS_INFO("Counter-boosting %s with %f for %d cycles. Actual is %f and last was %f.", boost->boostName.c_str(), boost->boostValue, boost->durationLimit, boost->actual, boost->last);
	    }
	}
    }

    double modifSgn(double in)
    {
	return (in < 0) ? -1 : 1;
    }

    double makeValid(double in)
    {
	if (in > 1)
	    return 1;

	if (in < -1)
	    return -1;

	return in;
    }
};
