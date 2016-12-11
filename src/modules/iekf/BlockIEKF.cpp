#include "BlockIEKF.hpp"

BlockIEKF::BlockIEKF() :
	_nh(), // node handle
	_sub_test(_nh.subscribe("sensor_combined", 1, &BlockIEKF::callback, this))
	//_pub_test(_nh.advertise<vehicle_attitude_s>(ORB_ID(vehicle_attitude), 0))
{
}

void BlockIEKF::update()
{
	//_pub_test.publish(1.0f);
	//ROS_INFO("publish %d", X::q_nb_3);
}

void BlockIEKF::callback(const sensor_combined_s *msg)
{
	ROS_INFO("callback %10.4f", double(msg->accelerometer_m_s2[0]));
}
