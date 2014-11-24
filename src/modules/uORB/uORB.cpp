#include "uORB.h"
//#include <errno.h>
//#include <drivers/drv_hrt.h>
//#include <systemlib/systemlib.h>
//#include <poll.h>

/*
 * uORB server 'main'.
 */
extern "C" { __EXPORT int uorb_main(int argc, char *argv[]); }

namespace uORB {

Topic::Topic(const size_t size, void *buffer) :
    _size(size),
    _buffer(buffer)
{}

bool Topic::get(void *buffer, uint64_t &generation) {
    bool ret = false;
    lock();
    if (generation != _generation) {
        if (buffer) {
            ::memcpy(buffer, _buffer, _size);
        }
        generation = _generation;
        ret = true;
    }
    unlock();
    return ret;
}

void Topic::put(const void *buffer) {
    lock();
    ::memcpy(_buffer, buffer, _size);
    ++_generation;
    unlock();
    broadcast();   // Notify self
    for (unsigned i = 0; i < _listeners_num; ++i) {
    	_listeners[i]->broadcast();
    };
}

bool Topic::wait(uint64_t &generation, unsigned timeout) {
    return NotificationListener::wait_cond([&](){
        return generation != _generation;
    }, timeout);
}

void Topic::add_listener(NotificationListener *lsn) {
    lock();
    if (_listeners_num < _max_listeners_per_topic) {
    	_listeners[_listeners_num++] = lsn;

    } else {
		warnx("too many listeners on topic");
    }
    unlock();
}

void Topic::remove_listener(NotificationListener *lsn) {
    lock();
    for (unsigned i = 0; i < _listeners_num; ++i) {
    	if (_listeners[i] == lsn) {
    		--_listeners_num;
            for (unsigned j = i; j < _listeners_num; ++j) {
                _listeners[j] = _listeners[j+1];
            }
    		break;
    	}
    };
    unlock();
}


int start();
int stop();
int test();
int test_pub();
int test_sub();
int test_pub_thread(int argc, char *argv[]);
int test_sub_thread(int argc, char *argv[]);

struct uorb2_test_topic_s {
	uint64_t	timestamp;
	int			val;
};

TopicAlloc<uorb2_test_topic_s> UORB2_TEST_TOPIC;

int start() {
	pthread_cond_init(&topics_cv, NULL);
	warnx("started");
	return 0;
}

int stop() {
	pthread_cond_destroy(&topics_cv);
	warnx("stopped");
	return 0;
}

int test() {
	Publication pub(UORB2_TEST_TOPIC);

	struct uorb2_test_topic_s t;

	uint64_t t_start = hrt_absolute_time();
	for (unsigned i = 0; i < 1000000; i++) {
		t.val = i;
		t.timestamp = hrt_absolute_time();
		pub.publish(&t);
	}
	uint64_t t_end = hrt_absolute_time();
	warnx("pub: data=%i t=%llu", t.val, t_end - t_start);

	Subscription sub(UORB2_TEST_TOPIC);

	struct uorb2_test_topic_s t1;
	for (int i = 0; i < 10; i++) {
		bool res = sub.update(&t1);
		warnx("sub next: updated=%i, data=%i", (int)res, t1.val);
	}

	return 0;
}

int test_pub() {
	int test_pub_task = task_spawn_cmd("uorb2_test_pub",
				       SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 2000,
				       test_pub_thread,
				       NULL);
	if (test_pub_task) {
		warnx("test_pub started");
	}
	return !test_pub_task;
}

int test_pub_thread(int argc, char *argv[]) {
	Publication pub(UORB2_TEST_TOPIC);

	struct uorb2_test_topic_s t;

	t.val = 0;
	while (true) {
		++t.val;
		warnx("pub: data=%i", t.val);
		t.timestamp = hrt_absolute_time();
		pub.publish(&t);
		sleep(1);
	}
	return 0;
}

int test_sub() {
	int test_sub_task = task_spawn_cmd("uorb2_test_sub",
				       SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 2000,
				       test_sub_thread,
				       NULL);
	if (test_sub_task) {
		warnx("test_sub started");
	}
	return !test_sub_task;
}

int test_sub_thread(int argc, char *argv[]) {
	Subscription sub(UORB2_TEST_TOPIC);

	struct uorb2_test_topic_s t;

	while (true) {
		bool updated = sub.wait(2000000);
		if (updated) {
			sub.update(&t);
			int latency = hrt_absolute_time() - t.timestamp;
			warnx("sub: data=%i latency=%i", t.val, latency);
		} else {
			warnx("sub: timeout");
		}
	}
	return 0;
}

}

int uorb_main(int argc, char *argv[]) {
	if (!strcmp(argv[1], "start")) {
		return uORB::start();
	} else if (!strcmp(argv[1], "stop")) {
		return uORB::stop();
	} else if (!strcmp(argv[1], "test")) {
		return uORB::test();
	} else if (!strcmp(argv[1], "test_pub")) {
		return uORB::test_pub();
	} else if (!strcmp(argv[1], "test_sub")) {
		return uORB::test_sub();
	}
	return 0;
}
