#include <px4_defines.h>

#include <stdio.h>
#include <unistd.h>

#include <DevMgr.hpp>

#include <drivers/drv_rgbled.h>

#include "navio_rgbled.h"

using namespace DriverFramework;

int do_test();

int do_test()
{
	DevHandle h;
    RGBLED *g_dev = nullptr;

    if (Framework::initialize() < 0) {
        printf("Framework init failed\n");
        return -1;
    }

    g_dev = new RGBLED("navio_rgbled test");
    g_dev->start();

    DevMgr::getHandle(RGBLED0_DEVICE_PATH, h);
    if (!h.isValid()) {
        printf("No RGB LED at " RGBLED0_DEVICE_PATH);
        return -1;
    }

    printf("off\n");
    h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_OFF);
    sleep(2);

    printf("red\n");
    h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_RED);
    sleep(2);

    printf("yellow\n");
    h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_YELLOW);
    sleep(2);

    printf("purple\n");
    h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_PURPLE);
    sleep(2);

    printf("green\n");
    h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_GREEN);
    sleep(2);

    printf("blue\n");
    h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_BLUE);
    sleep(2);

    printf("blue blink slow\n");
    h.ioctl(RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_BLINK_SLOW);
    sleep(10);

    printf("green blink normal\n");
    h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_GREEN);
    h.ioctl(RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_BLINK_NORMAL);
    sleep(10);

    printf("red blink fast\n");
    h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_RED);
    h.ioctl(RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_BLINK_FAST);
    sleep(10);

    printf("blue breathe (bogus)\n");
    h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_BLUE);
    h.ioctl(RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_BREATHE);
    sleep(10);

	return 0;
}
