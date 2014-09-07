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
#define ORB_DEFINE(_name, _struct)			\
	const struct orb_metadata __orb_##_name = {	\
		#_name,					\
		sizeof(_struct)		\
	};

/**
 * Thread-safe storage class for topic data.
 */
class Topic {
public:
	Topic* next = nullptr;

	Topic(orb_id_t orb_id) :
		_orb_id(orb_id),
		_buffer(new char[orb_id->o_size])
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
	uint64_t get(void* buffer, uint64_t generation_have) {
		irqstate_t flags = irqsave();
		if (generation_have >= _generation) {
			/* caller already has current generation, nothing to do */
			return 0;
		}
		uint64_t g = _generation;
		memcpy(buffer, _buffer, _orb_id->o_size);
		irqrestore(flags);
		return g;
	}

	/**
	 * Publish data from buffer to topic. Buffer size
	 */
	void put(void* buffer)
	{
		irqstate_t flags = irqsave();
		_generation++;
		memcpy(_buffer, buffer, _orb_id->o_size);
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
		uint64_t res = _topic.get(buffer, _generation);

		if (res) {
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
