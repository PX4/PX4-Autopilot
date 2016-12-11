#include <px4_posix.h>

///
// ROS to uORB API translator
//

#define ROS_INFO PX4_INFO
#define ROS_WARN PX4_WARN

namespace ros
{

// forward declarations
class Node;
class Rate;
class Subscriber;
class Publishder;
class Callback;
class CallbackInterface;

// node for this process declared in ros.cpp
extern Node _node;

/***
 * Check if any callbacks are ready to fire and call them
 * for this process
 */
void spin();

/**
 * Doesn't actually do anything currently
 */
void init(int argc, char **argv, const char *node_name);

/**
 * There is one node per process and it's job is to run communication
 * with the rest of the system via uORB
 */
class Node
{
public:
	void spin();
	void addSubscriber(Subscriber *sub);
private:
	Subscriber *_subListHead;
};

/**
 * This sleeps until a deadline to meet a
 * desired frequency
 */
class Rate
{
public:
	Rate(float frequency);
	void sleep();
private:
	float _frequency;
	uint64_t _wake_timestamp;
};

/**
 * This class represents one subscribe and contains the required
 * hooks for the callback
 */
class Subscriber
{
public:
	Subscriber(const char *topic, size_t queue_size, CallbackInterface *cb);
	virtual ~Subscriber();
	void callback();
	Subscriber *next;
private:
	CallbackInterface *_callbackPtr;
	//const char * _topic;
	//size_t _queue_size;
};

/**
 * A publisher.
 */
class Publisher
{
public:
	Publisher(const char *name, size_t queue_size);
	virtual ~Publisher();

	template <class T>
	void publish(const T &msg)
	{
	}

private:
	//const char * _name;
	//size_t _queue_size;;
};

/**
 * The abstract interface to the callback.
 */
class CallbackInterface
{
public:
	virtual void callback() = 0;
};

/**
 * The callback implementation. It contains
 * an object and a function pointre to call a member
 * function.
 */
template <class MsgType, class ObjType>
class CallbackImpl : public CallbackInterface
{
public:
	CallbackImpl(void (ObjType::*callbackFuncPtr)(const MsgType *msg), ObjType *obj) :
		_callbackFuncPtr(callbackFuncPtr), _objPtr(obj), _msg()
	{
	}

	virtual void callback()
	{
		(_objPtr->*_callbackFuncPtr)(&_msg);
	}
private:
	void (ObjType::*_callbackFuncPtr)(const MsgType *msg);
	ObjType *_objPtr;
	MsgType _msg;
};

/**
 * This defines the standard ROS node handle that
 * ROS users expect.
 */
class NodeHandle
{
public:

	bool ok();

	void param(const char *name, float val, const char *topic);

	template <class MsgType, class ObjType>
	Subscriber subscribe(const char *topic, size_t queue_size,
			     void (ObjType::*cb)(const MsgType *msg), ObjType *obj)
	{
		Subscriber sub(topic, queue_size, new CallbackImpl<MsgType, ObjType>(cb, obj));
		_node.addSubscriber(&sub);
		return sub;
	}

	template <class T>
	Publisher advertise(const char *name, size_t queue_size)
	{
		return Publisher(name, queue_size);
	}

};

}
