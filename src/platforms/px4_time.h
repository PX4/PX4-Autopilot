#pragma once

#include <sys/types.h>
#include <time.h>

#if defined(__PX4_LINUX) || defined(__PX4_NUTTX)

#define px4_clock_gettime clock_gettime
#define px4_clock_settime clock_settime

#elif defined(__PX4_QURT)

#include <sys/timespec.h>

__BEGIN_DECLS

#if 0
#if !defined(__cplusplus)
struct timespec
{
	time_t tv_sec;
	long tv_nsec;
};
#endif
#endif
int px4_clock_gettime(clockid_t clk_id, struct timespec *tp);
int px4_clock_settime(clockid_t clk_id, struct timespec *tp);

__EXPORT unsigned int sleep(unsigned int sec);

__END_DECLS
#endif
