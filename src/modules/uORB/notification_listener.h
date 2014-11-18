#pragma once

#include <systemlib/visibility.h>
#include <pthread.h>
#include <nuttx/clock.h>

namespace uORB {

class __EXPORT NotificationListener {
public:
	NotificationListener() {
		pthread_mutex_init(&_mtx, nullptr);
		pthread_cond_init(&_cv, nullptr);
	}

	~NotificationListener() {
		pthread_mutex_destroy(&_mtx);
		pthread_cond_destroy(&_cv);
	}

	inline void broadcast() {
		pthread_cond_broadcast(&_cv);
    }

    template <typename FUNC>
    bool wait_cond(FUNC cond, unsigned timeout) {
    	bool ret = true;
        pthread_mutex_lock(&_mtx);
        if (!cond()) {
        	timespec tend;
        	clock_gettime(0, &tend);
        	tend.tv_sec += timeout / 1000000;
        	tend.tv_nsec += (timeout % 1000000) * 1000;

        	ret = (0 == pthread_cond_timedwait(&_cv, &_mtx, &tend));	// return false in case of timeout/error
        }
        pthread_mutex_unlock(&_mtx);
        return ret;
    }

protected:
    inline void lock() {
        pthread_mutex_lock(&_mtx);
    }

    inline void unlock() {
        pthread_mutex_unlock(&_mtx);
    }

    pthread_mutex_t	_mtx;
    pthread_cond_t	_cv;
};

}
