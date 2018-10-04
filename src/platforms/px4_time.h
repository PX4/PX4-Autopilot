#pragma once

#include <sys/types.h>
#include <time.h>

#if defined(__PX4_APPLE_LEGACY)

__BEGIN_DECLS

#define clockid_t unsigned

int px4_clock_gettime(clockid_t clk_id, struct timespec *tp);
int px4_clock_settime(clockid_t clk_id, struct timespec *tp);

__EXPORT unsigned int sleep(unsigned int sec);

__END_DECLS

#elif defined(__PX4_QURT)

#include <sys/timespec.h>

__BEGIN_DECLS

int px4_clock_gettime(clockid_t clk_id, struct timespec *tp);
int px4_clock_settime(clockid_t clk_id, struct timespec *tp);

__EXPORT unsigned int sleep(unsigned int sec);

__END_DECLS

#else

#define px4_clock_gettime clock_gettime
#define px4_clock_settime clock_settime

#define px4_usleep system_usleep

#endif
