#include <errno.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <systemlib/systemlib.h>

#include "uORB2.h"

/*
 * uORB server 'main'.
 */
extern "C" { __EXPORT int uorb_main(int argc, char *argv[]); }

namespace uORB {

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

int start();
int stop();
int test();
int test_pub();
int test_sub();
int test_pub_thread(int argc, char *argv[]);
int test_sub_thread(int argc, char *argv[]);

pthread_cond_t topics_cv;

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
			uint64_t latency = hrt_absolute_time() - t.timestamp;
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

/*
 * C wrappers, old uORB interface compatibility.
 */

int orb_subscribe(orb_id_t topic) {
	uORB::Subscription *sub = new uORB::Subscription(*reinterpret_cast<uORB::Topic*>(topic));
	return (int)sub;
}

int orb_unsubscribe(int handle) {
	delete (uORB::Subscription *)handle;
	return 0;
}

orb_advert_t orb_advertise(orb_id_t topic, const void *data) {
	uORB::Publication *pub = new uORB::Publication(*reinterpret_cast<uORB::Topic*>(topic));
	pub->publish(data);
	return (orb_advert_t)pub;
}

int orb_publish(orb_id_t topic, orb_advert_t handle, const void *data) {
	reinterpret_cast<uORB::Publication*>(handle)->publish(data);
	return 0;
}

int orb_copy(orb_id_t topic, int handle, void *buffer) {
	uint64_t gen = 0;
	return !reinterpret_cast<uORB::Topic*>(topic)->get(buffer, gen);
}

int orb_check(int handle, bool *updated) {
	*updated = reinterpret_cast<uORB::Subscription*>(handle)->check();
	return 0;
}

int orb_stat(int handle, uint64_t *time) {
	// TODO
	return 0;
}

int	orb_set_interval(int handle, unsigned interval) {
	return 0;
}
