#include "ros.hpp"
#include "px4_posix.h"
#include "drivers/drv_hrt.h"

namespace ros
{

// The node for this process
Node *_node;

void init(int argc, char **argv, const char *node_name)
{
	_node = new Node();
}

void spin()
{
	_node->spin();
}

Node::Node()
{
	//_hmap.put("vehicle_attitude", ORB_ID(vehicle_attitude));
	//_hmap.put("sensor_combined", ORB_ID(sensor_combined));
	//_hmap.put("vision_position_estimate", ORB_ID(vision_position_estimate));
	//_hmap.put("att_pos_mocap", ORB_ID(att_pos_mocap));
	//_hmap.put("airspeed", ORB_ID(airspeed));
	//_hmap.put("parameter_update", ORB_ID(parameter_update));
	//_hmap.put("vehicle_global_position", ORB_ID(vehicle_global_position));
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

bool Node::getTopicMeta(const char *topic, const struct orb_metadata *meta)
{
	//return _hmap.get(topic, meta);
	return true;
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

Subscriber::Subscriber() :
	next(NULL),
	_callbackPtr(NULL)
{
}

Subscriber::Subscriber(Node *node, CallbackInterface *cb) :
	next(NULL),
	_callbackPtr(cb)
{
	node->addSubscriber(this);
}


Subscriber::~Subscriber()
{
}

void Subscriber::callback()
{
	_callbackPtr->callback();
}

Publisher::Publisher(const char *name, size_t queue_size)
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

void NodeHandle::param(const char *name, float val, const char *topic)
{
}

}
