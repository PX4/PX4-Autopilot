#include "led_hw.h"
#include <stddef.h>
#include <drivers/drv_hrt.h>

static orb_advert_t led_control_pub = NULL;
static struct led_control_s led_control;

void sl_led_init(void)
{
	led_control.led_mask = 0xff;
	led_control.mode = LED_CONTROL_MODE_OFF;
	led_control.priority = 0;
	led_control.timestamp = hrt_absolute_time();
	led_control_pub = orb_advertise(ORB_ID(led_control), &led_control);
}

void sl_led_deinit(void)
{
	led_control.led_mask = 0xff;
	led_control.mode = LED_CONTROL_MODE_DISABLED;
	led_control.priority = LED_CONTROL_MAX_PRIORITY;
	led_control.timestamp = hrt_absolute_time();
	led_control_pub = orb_advertise(ORB_ID(led_control), &led_control);
	orb_unadvertise(led_control_pub);
}

void sl_rgbled_set_color_and_mode(uint8_t color, uint8_t mode, uint8_t blinks, uint8_t prio)
{
	led_control.mode = mode;
	led_control.color = color;
	led_control.num_blinks = blinks;
	led_control.priority = prio;
	led_control.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(led_control), led_control_pub, &led_control);
}
