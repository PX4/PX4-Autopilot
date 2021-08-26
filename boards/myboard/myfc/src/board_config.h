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

/* LEDs */
// LED1 - GPIO 25 - Green
#define GPIO_LED1       PX4_MAKE_GPIO_OUTPUT_CLEAR(25) // Take a look at rpi_common micro_hal.h
#define GPIO_LED_BLUE   GPIO_LED1

#define BOARD_OVERLOAD_LED     LED_BLUE

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3)	// Change this later based on the adc channels actually used

#define ADC_BATTERY_VOLTAGE_CHANNEL  1			// Corresponding GPIO 27. Used in init.c for disabling GPIO_IE
#define ADC_BATTERY_CURRENT_CHANNEL  2			// Corresponding GPIO 28. Used in init.c for disabling GPIO_IE
#define ADC_RC_RSSI_CHANNEL          0

/* Define Battery 1 Voltage Divider and A per V. */
#define BOARD_BATTERY1_V_DIV         (13.653333333f)
#define BOARD_BATTERY1_A_PER_V       (36.367515152f)

/* High-resolution timer */
#define HRT_TIMER 1
#define HRT_TIMER_CHANNEL 1
#define HRT_PPM_CHANNEL 1	// Number really doesn't matter for this board
#define GPIO_PPM_IN		(16 | GPIO_FUN(RP2040_GPIO_FUNC_SIO))
#define RC_SERIAL_PORT               "/dev/ttyS0"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

/* This board provides a DMA pool and APIs */			// Needs to be figured out
#define BOARD_DMA_ALLOC_POOL_SIZE 2048

/* USB
 *
 *  VBUS detection is on 29  ADC_DPM0 and PTE8
 */
#define GPIO_USB_VBUS_VALID     (24 | GPIO_FUN(RP2040_GPIO_FUNC_SIO))    // Used in usb.c

/*
 * By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */

#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_USB_VBUS_VALID))

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: rp2040_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void rp2040_spiinitialize(void);


/****************************************************************************************************
 * Name: rp2040_usbinitialize
 *
 * Description:
 *   Called to configure USB IO.
 *
 ****************************************************************************************************/

extern void rp2040_usbinitialize(void);

// extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
