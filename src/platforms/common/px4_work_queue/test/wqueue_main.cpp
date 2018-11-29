
#include "wqueue_test.h"
#include "wqueue_scheduled_test.h"

#include <px4_log.h>
#include <px4_middleware.h>
#include <px4_app.h>
#include <stdio.h>

int PX4_MAIN(int argc, char **argv)
{
	px4::init(argc, argv, "wqueue_test");

	PX4_INFO("wqueue test 1");
	WQueueTest wq1;
	wq1.main();

	PX4_INFO("wqueue test 2 (scheduled)");
	WQueueScheduledTest wq2;
	wq2.main();

	PX4_INFO("wqueue test complete, exiting");

	return 0;
}
