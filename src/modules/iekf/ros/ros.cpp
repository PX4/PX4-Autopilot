#include "ros.hpp"
#include "px4_posix.h"
#include "drivers/drv_hrt.h"

namespace ros
{

Node *_node;

void Node::spin()
{
}

void spin()
{
	if (_node != NULL) {
		_node->spin();
	}
}

void init(int argc, char **argv, const std::string &node_name)
{
}

Rate::Rate(float frequency):
	_frequency(frequency),
	_wake_timestamp(hrt_absolute_time())
{
}

void Rate::sleep()
{
	int32_t dt = _wake_timestamp - hrt_absolute_time();

	if (dt > 0) {
		usleep(dt);
	}

	_wake_timestamp = hrt_absolute_time() + 1.0e6f / _frequency;
}

Subscriber::Subscriber(const std::string &topic, size_t queue_size)
//_topic(topic),
//_queue_size(queue_size)
{
}

Subscriber::~Subscriber()
{
}

Publisher::Publisher(const std::string &name, size_t queue_size)
//_name(name),
//_queue_size(queue_size)
{
}

Publisher::~Publisher()
{
}

bool NodeHandle::ok()
{
	return true;
}

void NodeHandle::param(std::string name, float val, std::string topic)
{
}

}
