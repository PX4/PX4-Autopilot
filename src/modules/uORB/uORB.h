#pragma once

#include "notification_listener.h"
#include <systemlib/visibility.h>
#include <systemlib/err.h>

#ifdef __cplusplus

namespace uORB {

#define ORB_DECLARE(name, type)	extern ::uORB::TopicAlloc<type> __orb_##name
#define ORB_DEFINE(name, type)	::uORB::TopicAlloc<type> __orb_##name

/**
 * Topic storage. Implements thread safe access, wait()/broadcast() functionality, adding external NotificationListeners.
 */
class __EXPORT Topic : public NotificationListener {
protected:
    /**
     * Basic constructor.
     * Shouldn't be called directly, but only by "allocator" child class.
     */
    Topic(const size_t size, void *buffer);

public:
    /**
     * Get data from topic if generation is different from one passed in argument.
     * On successful update generation passed in argument updated.
     *
     * @return true if new data was copied to buffer.
     */
    bool get(void *buffer, uint64_t &generation);

    /**
     * Put new data on topic, increases generation of data and notifies all listeners.
     */
    void put(const void *buffer);

    /**
     * Get current generation of data.
     */
    uint64_t get_generation();

    /**
     * Wait for new data (i.e. with generation different from one passed in argument).
     *
     * @return true if new data available, false on timeout.
     */
    bool wait(uint64_t &generation, unsigned timeout);

    /**
     * Add notification listener. listener.notify() will be called on each update of the topic.
     */
    void add_listener(NotificationListener *lsn);

    /**
     * Remove notification listener.
     */
    void remove_listener(NotificationListener *lsn);

private:
    static constexpr unsigned _max_listeners_per_topic = 32;

    const size_t    _size;
    void *          _buffer = nullptr;
    uint64_t        _generation = 0;
    NotificationListener *	_listeners[_max_listeners_per_topic];
    unsigned		_listeners_num = 0;
};


template <class T>
class __EXPORT TopicAlloc : public Topic {
public:
    TopicAlloc() :
        Topic(sizeof(_buf), &_buf)
    {}

private:
    T	_buf;
};


/**
* Subscription object. Subscriber must create subscription to receive data.
*/
class __EXPORT Subscription {
public:
    /**
    * Subscription constructor.
    */
    Subscription(Topic &topic) :
        _topic(topic)
    {}

    inline Topic &get_topic() {
        return _topic;
    }

    /**
    * Check for last update on topic.
    *
    * @return true if got update and it was copied to buffer
    */
    inline bool update(void *buffer) {
        return _topic.get(buffer, _generation);
    }

    /**
    * Copy last update on topic.
    *
    * @return true if this topic was ever published and data was copied to buffer
    */
    inline bool copy(void *buffer) {
        _generation = 0;
        return _topic.get(buffer, _generation);
    }

    /**
     * Check for updates on topic. Don't clear 'updated' flag.
     *
     * @return true if any updates available
     */
    inline bool check() {
        return _topic.get_generation() != _generation;
    }

    /**
     * Wait blocking for update for specified timeout.
     *
     * @return true if update happened or false if not
     */
    inline bool wait(unsigned timeout) {
        return _topic.wait(_generation, timeout);
    }

private:
    Topic &                 _topic;
    uint64_t                _generation = 0;
};


/**
* Publication object. Publisher must create publication to publish data.
*/
class __EXPORT Publication {
public:
    /**
    * Publication constructor..
    */
    Publication(Topic &topic) :
        _topic(topic)
    {}

    /**
    * Publish update.
    *
    * @return true if update was actually published.
    */
    inline void publish(void *buffer) {
        _topic.put(buffer);
    }

private:
    Topic &     _topic;
};


class __EXPORT SubscriptionsSet : private NotificationListener {
public:
    ~SubscriptionsSet() {
    	for (unsigned i = 0; i < _subscriptions_num; i++) {
    		_subscriptions[i]->get_topic().remove_listener(this);
        };
    }

    void add(Subscription *sub) {
    	if (_subscriptions_num < _max_subscriptions_in_set) {
    		_subscriptions[_subscriptions_num++] = sub;
            sub->get_topic().add_listener(this);

    	} else {
    		warnx("too many subscriptions in set");
    	}
    }

    bool check() {
    	for (unsigned i = 0; i < _subscriptions_num; i++) {
    		if (_subscriptions[i]->check()) {
    			return true;
    		}
    	}
    	return false;
    }

    bool wait(unsigned timeout) {
        return NotificationListener::wait_cond([&](){ return check(); }, timeout);
    }

    template <typename P>
    void get_data(void *buf, P proc) {
    	for (unsigned i = 0; i < _subscriptions_num; i++) {
    		Subscription *sub = _subscriptions[i];
			if (sub->update(buf)) {
			   proc(i, sub);
			}
    	}
    }

private:
    static constexpr unsigned _max_subscriptions_in_set = 32;

    Subscription *	_subscriptions[_max_subscriptions_in_set];
    unsigned		_subscriptions_num = 0;
};

}

#endif
