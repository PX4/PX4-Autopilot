#include <sys/time.h>
#include <inttypes.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>

hrt_abstime hrt_absolute_time()
{
	struct timeval te;
	gettimeofday(&te, NULL); // get current time 取得当前时间
	hrt_abstime us = static_cast<uint64_t>(te.tv_sec) * 1e6 + te.tv_usec; // caculate us 换算成微秒
	return us;
}

hrt_abstime hrt_elapsed_time(const volatile hrt_abstime *then)
{
	// not thread safe
	return hrt_absolute_time() - *then;
}
