#include "ros/ros.hpp"

class BlockIEKF
{
public:
	BlockIEKF();
	void update();
	bool ok() { return _nh.ok(); }
	void callback(const float *msg);
private:
	ros::NodeHandle _nh;
	ros::Subscriber _sub_test;
	ros::Publisher _pub_test;
};
