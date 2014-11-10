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
#include <pthread.h>

namespace uORB2 {

extern pthread_cond_t topics_cv;

/**
 * Topic.
 */
class Topic {
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

/**
 * Generates a pointer to the uORB2 topic.
 *
 * The topic must have been declared previously in scope
 * with ORB_DECLARE().
 *
 * @param _name		The name of the topic.
 */
#define ORB_ID(_name)		__orb_##_name

#define ORB_DECLARE(_name)		extern "C" Topic &__orb_##_name __EXPORT
#define ORB_DECLARE_OPTIONAL(_name)	extern "C" Topic &__orb_##_name __EXPORT __attribute__((weak))

/**
 * Define (instantiate) the uORB2 topic.
 *
 * Note that there must be no more than one instance of this macro
 * for each topic.
 *
 * @param _name		The name of the topic.
 * @param _struct	The structure the topic provides.
 */
#define ORB_DEFINE(_name, _struct)		TopicAlloc<_struct> __orb_##_name_alloc; Topic &__orb_##_name = __orb_##_name_alloc;


template <typename P>
int topics_poll(P f_check, unsigned timeout) {
	struct timespec time_to_wait;
	clock_gettime(0, &time_to_wait);
	time_to_wait.tv_sec += timeout / 1000000;
	time_to_wait.tv_nsec += (timeout % 1000000) * 1000;
    int ret = 0;
    while (true) {
        ret += f_check();
        if (ret) {
            return ret;
        }
        pthread_mutex_t mtx;
    	pthread_mutex_init(&mtx, NULL);
        pthread_mutex_lock(&mtx);
        int res = pthread_cond_timedwait(&topics_cv, &mtx, &time_to_wait);
        pthread_mutex_unlock(&mtx);
        if (res) {
        	break;	// Timeout
        }
    }
    return 0;
}

/**
 * Subscription object. Subscriber must create subscription to receive data.
 */
class Subscription {
public:
	Subscription(Topic& topic) :
		_topic(topic)
	{}

	/**
	 * Get last update on topic.
	 *
	 * @return true if got update and it was copied to buffer
	 */
	bool update(void* buffer) {
		return _topic.get(buffer, _generation);
	}

	/**
	 * Check if topic has updates.
	 *
	 * @return true if updates available
	 */
	bool check() {
	    return _topic.get_generation() != _generation;
	}

	/**
	 * Wait blocking for update for specified timeout (in us).
	 *
	 * @return true if update happened or false if not
	 */
	bool wait(unsigned timeout) {
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
	{}

	/**
	 * Publish data on topic
	 */
	void publish(void* buffer) {
		_topic.put(buffer);
	}

private:
	Topic&		_topic;
};

int subscriptions_poll(Subscription **subs, bool *updated, unsigned n, unsigned timeout) {
    auto f_check = [&]()->unsigned{
        unsigned ret = 0;
        for (unsigned i = 0; i < n; i++) {
            updated[i] = subs[i]->check();
            if (updated[i]) {
                ret++;
            }
        }
        return ret;
    };

    return topics_poll(f_check, timeout);
}

}
