#include "BlockIEKF.hpp"
#include <px4_posix.h>

BlockIEKF::BlockIEKF() :
	_nh(), // node handle
	_sub_test(_nh.subscribe("test", 1000, &BlockIEKF::callback, this)),
	_pub_test(_nh.advertise<float>("test", 1000))
{
}

void BlockIEKF::update()
{
	_pub_test.publish(1.0f);
	PX4_INFO("publish");
}

void BlockIEKF::callback(const float *msg)
{
	PX4_INFO("callback");
}
