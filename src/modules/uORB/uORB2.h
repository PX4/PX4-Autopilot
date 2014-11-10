#pragma once

#include "uORB.h"
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
#include <pthread.h>
#include <drivers/drv_hrt.h>

namespace uORB {

__EXPORT extern pthread_cond_t topics_cv;

#define ORB2_ID(_name)		*reinterpret_cast<uORB::Topic*>(&__orb_##_name)

/**
 * Topic.
 */
class __EXPORT Topic {
protected:
    Topic(const size_t size, void *buffer) :
    	_size(size),
    	_buffer(buffer)
    {}

public:
    bool get(void *buffer, uint64_t &generation) const {
		irqstate_t flags = irqsave();
		bool res;
        if (generation == _generation) {
            /* caller already has current generation, nothing to do */
            res = false;
        } else {
            if (buffer) {
                ::memcpy(buffer, _buffer, _size);
            }
            generation = _generation;
            res = true;
        }
		irqrestore(flags);
		return res;
    }

    void put(const void *buffer) {
		irqstate_t flags = irqsave();
        ::memcpy(_buffer, buffer, _size);
        ++_generation;
		irqrestore(flags);
		pthread_cond_broadcast(&_cv);
		pthread_cond_broadcast(&topics_cv);
    }

    uint64_t get_generation() {
		irqstate_t flags = irqsave();
        uint64_t g = _generation;
		irqrestore(flags);
		return g;
    }

    pthread_cond_t* get_cond_var() {
    	return &_cv;
    }

private:
    const size_t    _size;
    void *          _buffer;
    uint64_t        _generation = 0;
    pthread_cond_t	_cv;
};

template <class T>
class TopicAlloc : public Topic {
public:
    TopicAlloc() :
        Topic(sizeof(_buf), &_buf)
    {}

private:
    T	_buf;
};

template <typename P>
int topics_poll(P f_check, unsigned timeout) {
	struct timespec time_to_wait;
	clock_gettime(0, &time_to_wait);
	time_to_wait.tv_sec += timeout / 1000000;
	time_to_wait.tv_nsec += (timeout % 1000000) * 1000;
    pthread_mutex_t mtx;
	pthread_mutex_init(&mtx, NULL);
    int ret = 0;
    while (true) {
        ret += f_check();
        if (ret) {
        	break;
        }
        pthread_mutex_lock(&mtx);
        int res = pthread_cond_timedwait(&topics_cv, &mtx, &time_to_wait);
        pthread_mutex_unlock(&mtx);
        if (res) {
        	break;	// Timeout
        }
    }
    pthread_mutex_destroy(&mtx);
    return ret;
}

/**
 * Subscription object. Subscriber must create subscription to receive data.
 */
class __EXPORT Subscription {
public:
	Subscription(Topic& topic) :
		_topic(topic)
	{}

	Subscription(Topic& topic, int interval) :
		_topic(topic)
	{}

	/**
	 * Get last update on topic.
	 *
	 * @return true if got update and it was copied to buffer
	 */
	bool update(void* buffer) {
		if (_topic.get(buffer, _generation)) {
			_next_update = hrt_absolute_time() + _interval;
			return true;

		} else {
			return false;
		}
	}

	/**
	 * Copy last revision of topic to buffer (even if no update since previous check).
	 *
	 * @return true if any data was ever published and copied to buffer
	 */
	bool copy(void* buffer) {
		_generation = 0;
		if (_topic.get(buffer, _generation)) {
			_next_update = hrt_absolute_time() + _interval;
			return true;

		} else {
			return false;
		}
	}

	/**
	 * Check if topic has updates.
	 *
	 * @return true if updates available
	 */
	bool check() {
		if (_interval > 0) {
			uint64_t now = hrt_absolute_time();
			if (now < _next_update) {
				return false;
			}
		}
		return _topic.get_generation() != _generation;
	}

	/**
	 * Wait blocking for update for specified timeout (in us).
	 *
	 * @return true if update happened or false if not
	 */
	bool wait(unsigned timeout) {
		if (_interval > 0) {
			uint64_t now = hrt_absolute_time();
			if (now < _next_update) {
				usleep(_next_update - now);
			}
		}
		struct timespec time_to_wait;
		clock_gettime(0, &time_to_wait);
		time_to_wait.tv_sec += timeout / 1000000;
		time_to_wait.tv_nsec += (timeout % 1000000) * 1000;

	    while (true) {
	    	if (check()) {
	    		return true;
	    	}
	        pthread_mutex_t mtx;
	    	pthread_mutex_init(&mtx, NULL);
	        pthread_mutex_lock(&mtx);
	        int res = pthread_cond_timedwait(_topic.get_cond_var(), &mtx, &time_to_wait);
	        pthread_mutex_unlock(&mtx);
	        if (res) {
	        	break;	// Timeout
	        }
	    }
    	return false;
	}

	void set_interval(uint64_t interval) {
		_interval = interval;
	}

private:
	Topic&		_topic;
	uint64_t	_generation = 0;
	uint64_t	_interval = 0;
	uint64_t	_next_update = 0;
};

/**
 * Publication object. Publisher must create publication to publish data.
 */
class __EXPORT Publication {
public:
	Publication(Topic& topic) :
		_topic(topic)
	{}

	/**
	 * Publish data on topic
	 */
	void publish(const void* buffer) {
		_topic.put(buffer);
	}

private:
	Topic&		_topic;
};

int subscriptions_poll(Subscription **subs, bool *updated, unsigned n, unsigned timeout);

}
