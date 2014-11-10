#include <errno.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <systemlib/systemlib.h>

#include "uORB2.h"

/*
 * uORB server 'main'.
 */
extern "C" { __EXPORT int uorb2_main(int argc, char *argv[]); }

namespace uORB2 {

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

ORB_DECLARE(uorb2_test_topic);
ORB_DEFINE(uorb2_test_topic, struct uorb2_test_topic_s);


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
	Publication pub(ORB_ID(uorb2_test_topic));

	struct uorb2_test_topic_s t;

	uint64_t t_start = hrt_absolute_time();
	for (unsigned i = 0; i < 1000000; i++) {
		t.val = i;
		t.timestamp = hrt_absolute_time();
		pub.publish(&t);
	}
	uint64_t t_end = hrt_absolute_time();
	warnx("pub: data=%i t=%llu", t.val, t_end - t_start);

	Subscription sub(ORB_ID(uorb2_test_topic));

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
	Publication pub(ORB_ID(uorb2_test_topic));

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
	Subscription sub(ORB_ID(uorb2_test_topic));

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

int uorb2_main(int argc, char *argv[]) {
	if (!strcmp(argv[1], "start")) {
		return uORB2::start();
	} else if (!strcmp(argv[1], "stop")) {
		return uORB2::stop();
	} else if (!strcmp(argv[1], "test")) {
		return uORB2::test();
	} else if (!strcmp(argv[1], "test_pub")) {
		return uORB2::test_pub();
	} else if (!strcmp(argv[1], "test_sub")) {
		return uORB2::test_sub();
	}
	return 0;
}
