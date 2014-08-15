#pragma once

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <systemlib/visibility.h>
#include <systemlib/err.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

namespace uORB2 {

/**
 * Object metadata.
 */
struct orb_metadata {
	const char *o_name;			/**< unique object name */
	const size_t o_size;		/**< object size */
	const size_t queue_size;	/**< queue size */
};

typedef const struct orb_metadata *orb_id_t;

/**
 * Generates a pointer to the uORB metadata structure for
 * a given topic.
 *
 * The topic must have been declared previously in scope
 * with ORB_DECLARE().
 *
 * @param _name		The name of the topic.
 */
#define ORB_ID(_name)		&__orb_##_name

#define ORB_DECLARE(_name)		extern "C" const struct orb_metadata __orb_##_name __EXPORT
#define ORB_DECLARE_OPTIONAL(_name)	extern "C" const struct orb_metadata __orb_##_name __EXPORT __attribute__((weak))

/**
 * Define (instantiate) the uORB metadata for a topic.
 *
 * The uORB metadata is used to help ensure that updates and
 * copies are accessing the right data.
 *
 * Note that there must be no more than one instance of this macro
 * for each topic.
 *
 * @param _name		The name of the topic.
 * @param _struct	The structure the topic provides.
 */
#define ORB_DEFINE(_name, _struct, _queue_size)			\
	const struct orb_metadata __orb_##_name = {	\
		#_name,					\
		sizeof(_struct),		\
		_queue_size				\
	};

/**
 * Thread-safe storage class for topic data.
 */
class Topic {
public:
	Topic* next = nullptr;

	Topic(orb_id_t orb_id) :
		_orb_id(orb_id),
		_buffer(new char[orb_id->o_size * orb_id->queue_size])
	{
	}

	~Topic()
	{
		if (_buffer) {
			delete _buffer;
		}
	}

	/**
	 * Get topic data to buffer. Return generation number, 0 means no data.
	 */
	uint64_t get(void* buffer, uint64_t generation_have, uint64_t generation_get) {
		irqstate_t flags = irqsave();

		if (generation_have >= _generation) {
			/* caller already has current generation, nothing to do */
			return 0;
		}

		if (generation_get) {
			/* minimum available generation */
			uint64_t g_min = _generation - _orb_id->queue_size + 1;
			generation_get = generation_get < g_min ? g_min : generation_get;

		} else {
			/* 0 generation in argument means get latest generation */
			generation_get = _generation;
		}

		memcpy(buffer, get_buffer_ptr(generation_get), _orb_id->o_size);
		irqrestore(flags);
		//warnx("Topic::get, gen %uul", generation_get);
		return generation_get;
	}

	/**
	 * Publish data from buffer to topic. Buffer size
	 */
	void put(void* buffer)
	{
		irqstate_t flags = irqsave();
		_generation++;
		memcpy(get_buffer_ptr(_generation), buffer, _orb_id->o_size);
		//warnx("Topic::put, gen %llu", _generation);
		irqrestore(flags);
	}

	orb_id_t get_orb_id()
	{
		return _orb_id;
	}

private:
	orb_id_t	_orb_id;
	char*		_buffer = nullptr;
	uint64_t	_generation = 0;

	inline void* get_buffer_ptr(uint64_t generation) {
		return _buffer + (generation % _orb_id->queue_size) * _orb_id->o_size;
	}
};


/**
 * Subscription object. Subscriber must create subscription to receive data.
 */
class Subscription {
public:
	Subscription(Topic& topic) :
		_topic(topic)
	{
	}

	/**
	 * Get last update on topic.
	 *
	 * @return true if got update and it was copied to buffer
	 */
	bool update(void* buffer)
	{
		uint64_t res = _topic.get(buffer, _generation, 0);

		if (res) {
			_generation = res;
			return true;

		} else {
			return false;
		}
	}

	/**
	 * Get next update on topic. May skip updates in case of topic buffer overflow.
	 *
	 * @return true if got update and it was copied to buffer
	 */
	bool next(void* buffer)
	{
		uint64_t res = _topic.get(buffer, _generation, _generation + 1);

		if (res != 0) {
			_generation = res;
			return true;

		} else {
			return false;
		}
	}

private:
	Topic&		_topic;
	uint64_t	_generation = 0;
};

/**
 * Publication object. Publisher must create publication to publish data.
 */
class Publication {
public:
	Publication(Topic& topic) :
		_topic(topic)
	{
	}

	void publish(void* buffer)
	{
		_topic.put(buffer);
	}

private:
	Topic&		_topic;
};


/**
 * Poll set of subscriptions.
 * @return number of updated subscription, 0 on timeout.
 */
int uORB_poll(Subscription* subs, bool* updated, ssize_t n, unsigned timeout);


/**
 * Topics manager.
 */
class ORBMaster {
public:
	/**
	 * Get topic by ID. If topic doesn't exists, create it.
	 */
	Topic& get_topic(const orb_id_t orb_id) {
		irqstate_t flags = irqsave();

		for (Topic* t = _topics_head; t != nullptr; t = t->next) {
			if (t->get_orb_id() == orb_id) {
				/* Topic found in list */
				irqrestore(flags);

				warnx("ORBMaster: topic found: %s", t->get_orb_id()->o_name);
				return *t;
			}
		}

		/* Topic not found, create it */
		Topic* t = new Topic(orb_id);
		if (_topics_tail) {
			_topics_tail->next = t;

		} else {
			_topics_head = t;
			_topics_tail = t;
		}

		irqrestore(flags);

		warnx("ORBMaster: new topic created: %s", t->get_orb_id()->o_name);
		return *t;
	}

private:
	Topic*		_topics_head = nullptr;
	Topic*		_topics_tail = nullptr;
};

}
