#include <string>

///
// ROS to uORB API translator
//

namespace ros
{

class Node;
class Subscriber;
class Publishder;
class Callback;
class CallbackInterface;

extern Node _node;

void spin();
void init(int argc, char **argv, const char *node_name);

class Node
{
public:
	void spin();
	void addSubscriber(Subscriber *sub);
private:
	Subscriber *_subListHead;
};

class Rate
{
public:
	Rate(float frequency);
	void sleep();
private:
	float _frequency;
	uint64_t _wake_timestamp;
};

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

class CallbackInterface
{
public:
	virtual void callback() = 0;
};

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

class NodeHandle
{
public:

	bool ok();

	void param(const char *name, float val, const char *topic);

	template <class MsgType, class ObjType>
	Subscriber subscribe(const char *topic, size_t queue_size,
			     void (ObjType::*cb)(const MsgType *msg), ObjType *obj)
	{
		CallbackInterface *callback = new CallbackImpl<MsgType, ObjType>(cb, obj);
		Subscriber sub(topic, queue_size, callback);
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
