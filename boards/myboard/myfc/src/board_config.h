/**
 * @file board_config.h
 *
 * myboardmyfc internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

#ifndef __ASSEMBLY__

#include <px4_platform_common/board_common.h>

/* LEDs */
// LED1 - GPIO 25 - Green
#define GPIO_LED1       PX4_MAKE_GPIO_OUTPUT_CLEAR(25) // Take a look at rpi_common micro_hal.h
#define GPIO_LED_GREEN   GPIO_LED1

#define BOARD_OVERLOAD_LED     LED_GREEN

/* High-resolution timer */
#define HRT_TIMER 1
#define HRT_TIMER_CHANNEL 1

#endif /* __ASSEMBLY__ */

__END_DECLS
