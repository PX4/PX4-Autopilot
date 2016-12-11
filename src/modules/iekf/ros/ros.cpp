#include "ros.hpp"
#include "px4_posix.h"
#include "drivers/drv_hrt.h"

namespace ros
{

// The node for this process
Node _node;

void init(int argc, char **argv, const std::string &node_name)
{
}

void spin()
{
	_node.spin();
}

void Node::spin()
{
	Subscriber *tail = _subListHead;

	while (tail != NULL) {
		tail->callback();
		tail = tail->next;
	}
}

void Node::addSubscriber(Subscriber *sub)
{
	if (_subListHead == NULL) {
		_subListHead = sub;
	}

	Subscriber *tail = _subListHead;

	while (tail->next != NULL) {
		tail = tail->next;
	}

	tail->next = sub;
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

Subscriber::Subscriber(const std::string &topic, size_t queue_size, CallbackInterface *cb) :
	next(NULL),
	_callbackPtr(cb)
//_topic(topic),
//_queue_size(queue_size)
{
}

Subscriber::~Subscriber()
{
}

void Subscriber::callback()
{
	_callbackPtr->callback();
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
