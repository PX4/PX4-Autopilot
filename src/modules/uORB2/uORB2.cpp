#include <errno.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "uORB2.h"

/*
 * uORB server 'main'.
 */
extern "C" { __EXPORT int uorb2_main(int argc, char *argv[]); }

namespace uORB2 {

struct uorb2_test_topic_s {
	int	val;
};

ORB_DECLARE(uorb2_test_topic);
ORB_DEFINE(uorb2_test_topic, struct uorb2_test_topic_s);

int test();

int test()
{
	Publication pub(ORB_ID(uorb2_test_topic));

	struct uorb2_test_topic_s t;

	uint64_t t_start = hrt_absolute_time();
	for (unsigned i = 0; i < 1000000; i++) {
		t.val = i;
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

}

int
uorb2_main(int argc, char *argv[])
{
	/*
	 * Test uORB2 topics.
	 */
	if (!strcmp(argv[1], "test")) {
		return uORB2::test();
	}
}
