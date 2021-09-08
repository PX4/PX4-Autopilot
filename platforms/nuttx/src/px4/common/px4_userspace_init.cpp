#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <uORB/uORB.h>
#include <sys/boardctl.h>

extern void cdcacm_init(void);

extern "C" void px4_userspace_init(void)
{
	hrt_init();

	param_init();

	px4::WorkQueueManagerStart();

	uorb_start();

#if defined(CONFIG_SYSTEM_CDCACM)
	cdcacm_init();
#endif
}
