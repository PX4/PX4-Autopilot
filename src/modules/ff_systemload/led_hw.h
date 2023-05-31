#ifndef __FF_SL_LED_HW_H
#define __FF_SL_LED_HW_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include <uORB/topics/led_control.h>

void sl_led_init(void);
void sl_led_deinit(void);
void sl_rgbled_set_color_and_mode(uint8_t color, uint8_t mode, uint8_t blinks, uint8_t prio);

#ifdef __cplusplus
}
#endif

#endif
