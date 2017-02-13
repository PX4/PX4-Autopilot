#pragma once

#include <px4_posix.h>
#include "uORB/Subscription.hpp"
#include "uORB/Publication.hpp"
#include "HashMap/hashmap/HashMap.h"
#include "containers/List.hpp"

///
// ROS to uORB API translator
//

#define ROS_INFO PX4_INFO
#define ROS_WARN PX4_WARN
#define ROS_ERROR PX4_ERR
#define ROS_DEBUG PX4_DEBUG

typedef const void *const_void_ptr;

namespace ros
{

// forward declarations
class Node;
class NodeHandle;
class Rate;
class Subscriber;
class Publisher;
class Callback;
class CallbackInterface;

class Time
{
public:
	// TODO ROS uses sec (uint32_t) and nsec (uint32_t)
	Time();
	Time(uint64_t nsec);
	static Time fromNSec(uint64_t nsec);
	uint64_t toNSec();
	static Time now();
private:
	uint64_t _nsec;
};

// node for this process declared in ros.cpp
extern Node *_node;
static const size_t _tableSize = 128;
extern HashMap<const char *, const struct orb_metadata *, _tableSize> _topicDict;
extern HashMap<const char *, param_t, _tableSize> _paramDict;

/***
 * Check if any callbacks are ready to fire and call them
 * for this process
 */
void spin();

/**
 * Initializes node interface
 */
void init(int argc, char **argv, const char *node_name);

/**
 * Shutdown node interface
 */
void shutdown();

/**
 * There is one node per process and it's job is to run communication
 * with the rest of the system via uORB
 */
class Node
{
public:
	Node();
	~Node();
	void spin();
	void addSubscriber(Subscriber *sub);
	bool getTopicMeta(const char *topic, const struct orb_metadata **meta);
	void registerHandle(NodeHandle *nh);
private:
	List<CallbackInterface *> _callbackList;
	List<NodeHandle *> _nodeHandleList;
	// disallow copy/ assignment
	Node(const Node &other);
	Node &operator=(const Node &other);
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
	Subscriber();
	Subscriber(const Subscriber &sub);
	Subscriber(CallbackInterface *cb);
	Subscriber &operator=(const Subscriber &sub);
	void callback();
	CallbackInterface *_callbackPtr;
	~Subscriber();
	int getHandle();
};

/**
 * A publisher.
 */
class Publisher
{
public:
	Publisher();
	Publisher(uORB::PublicationTiny *pub);
	Publisher(const Publisher &pub);
	Publisher &operator=(const Publisher &pub);
	template <class T>
	void publish(const T &msg)
	{
		if (_pub != NULL) {
			_pub->update(const_void_ptr(&msg));

		} else {
			ROS_INFO("publication is NULL");
		};
	}
	uORB::PublicationTiny *_pub;
	~Publisher();
};

/**
 * The abstract interface to the callback.
 */
class CallbackInterface : public ListNode<CallbackInterface *>
{
public:
	virtual void callback() = 0;
	CallbackInterface() :
		ListNode()
	{
	}
	virtual ~CallbackInterface()
	{
	}
	virtual int getHandle() = 0;
private:
	CallbackInterface(const CallbackInterface &other);
	const CallbackInterface &operator=(const CallbackInterface &other);
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
	CallbackImpl(const struct orb_metadata *meta,
		     unsigned interval,
		     int instance,
		     void (ObjType::*callbackFuncPtr)(const MsgType *msg),
		     ObjType *obj) :
		_sub(meta, interval, instance),
		_callbackFuncPtr(callbackFuncPtr),
		_objPtr(obj)
	{
	}

	int getHandle()
	{
		return _sub.getHandle();
	}

	virtual void callback()
	{
		if (_sub.getHandle() < 0) {
			ROS_INFO("subscription handle not valid");
			return;
		}

		if (_sub.updated()) {
			MsgType msg;
			_sub.update(&msg);
			(_objPtr->*_callbackFuncPtr)(&msg);
		}
	}
private:
	CallbackImpl(const CallbackImpl &other);
	const CallbackImpl &operator=(const CallbackImpl &other);
	uORB::SubscriptionTiny _sub;
	void (ObjType::*_callbackFuncPtr)(const MsgType *msg);
	ObjType *_objPtr;
};

/**
 * This defines the standard ROS node handle that
 * ROS users expect.
 *
 * Only difference is interval and instance and meta passed
 * instead of topic name.
 */
class NodeHandle : public ListNode<NodeHandle *>
{
public:

	NodeHandle() :
		ListNode(),
		_ok(true)
	{
		_node->registerHandle(this);
	}

	inline bool ok()
	{
		return _ok;
	}

	inline void shutdown()
	{
		ROS_INFO("node handle shutdown %p\n", this);
		_ok = false;
	}

	template<class T>
	param_t getPX4ParamHandle(const char *key)
	{
		param_t handle = PARAM_INVALID;

		// check if key is in dict
		if (!_paramDict.get(key, handle)) {
			// if not in dict try to find it
			handle = param_find(key);

			// if we can't find it, warn
			if (handle == PARAM_INVALID) {
				ROS_WARN("error finding param: %s\n", key);

			} else {
				// if we find it, add it to the dict
				_paramDict.put(key, handle);
			}
		}

		return handle;
	}

	template<class T>
	bool getParam(const char *key, T &val)
	{
		// default to zero
		val = 0;
		param_t handle = getPX4ParamHandle<T>(key);

		if (handle == PARAM_INVALID) {
			return false;
		}

		param_get(handle, &val);
		return true;
	}

	template<class T>
	void setParam(const char *key, const T &val)
	{
		param_t handle = getPX4ParamHandle<T>(key);

		if (handle != PARAM_INVALID) {
			param_set(handle, &val);
		}
	}

	void deleteParam(const char *key);

	template<class T>
	void param(const char *key, T &val, const T &defaultValue)
	{
		if (!getParam(key, val)) {
			val = defaultValue;
		}
	}

	template <class MsgType, class ObjType>
	Subscriber subscribe(const char *topic, size_t queue_size,
			     void (ObjType::*cb)(const MsgType *msg), ObjType *obj,
			     unsigned interval = 0, int instance = 0)
	{
		(void)queue_size; // unused
		const struct orb_metadata *meta = NULL;

		if (_node  == NULL) {
			ROS_ERROR("node not initialized");
			return Subscriber();
		}

		if (!_node->getTopicMeta(topic, &meta)) {
			ROS_ERROR("error, topic not found %s\n", topic);
			return Subscriber();
		}

		CallbackInterface *cb_impl = new CallbackImpl<MsgType, ObjType>(
			meta, interval, instance, cb, obj);
		return Subscriber(cb_impl);
	}

	template <class T>
	Publisher advertise(const char *topic, size_t queue_size, int priority = -1)
	{
		(void)queue_size; // unused
		const struct orb_metadata *meta = NULL;

		if (_node  == NULL) {
			ROS_ERROR("node not initialized");
			return Publisher();
		}

		if (!_node->getTopicMeta(topic, &meta)) {
			ROS_ERROR("error, topic not found %s\n", topic);
			return Publisher();
		}

		return Publisher(new uORB::PublicationTiny(meta, priority));
	}

private:
	volatile bool _ok;
};

}
