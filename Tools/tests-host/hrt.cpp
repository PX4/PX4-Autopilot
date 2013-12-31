#include <sys/time.h>
#include <inttypes.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>

uint64_t hrt_absolute_time()
{
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    unsigned long long us = static_cast<uint64_t>(te.tv_sec) * 1e6 + te.tv_usec; // caculate us
    printf("us: %lld\n", us);
    return us;
}