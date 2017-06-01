#include <errno.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "uORB2.hpp"

/*
 * uORB server 'main'.
 */
extern "C" { __EXPORT int uorb2_main(int argc, char *argv[]); }

namespace uORB2 {

ORBMaster	*g_dev;

struct uorb2_test_topic_s {

	int	val;
};

ORB_DECLARE(uorb2_test_topic);
ORB_DEFINE(uorb2_test_topic, struct uorb2_test_topic_s, 5);

int test();

int test()
{
	Publication pub(g_dev->get_topic(ORB_ID(uorb2_test_topic)));

	struct uorb2_test_topic_s t;

	uint64_t t_start = hrt_absolute_time();
	for (unsigned i = 0; i < 1000000; i++) {
		t.val = i;
		pub.publish(&t);
	}
	uint64_t t_end = hrt_absolute_time();
	warnx("pub: data=%i t=%llu", t.val, t_end - t_start);

	Subscription sub(g_dev->get_topic(ORB_ID(uorb2_test_topic)));

	struct uorb2_test_topic_s t1;
	for (int i = 0; i < 10; i++) {
		bool res = sub.update(&t1);
		warnx("sub next: updated=%i, data=%i", (int)res, t1.val);
	}

	return 0;
}

}

int
uorb2_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 *
	 * XXX it would be nice to have a wrapper for this...
	 */
	if (!strcmp(argv[1], "start")) {

		if (uORB2::g_dev != nullptr) {
			warnx("already loaded");
			/* user wanted to start uorb2, its already running, no error */
			return 0;
		}

		/* create the driver */
		uORB2::g_dev = new uORB2::ORBMaster();

		if (uORB2::g_dev == nullptr) {
			warnx("driver alloc failed");
			return -ENOMEM;
		}

		warnx("ready");
		return OK;
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		return uORB2::test();
	}
}
