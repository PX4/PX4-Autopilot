/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * Corvon 743V2 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Configuration ************************************************************************************/

#define BOARD_HAS_NBAT_V        2 /* Dual battery voltage */
#define BOARD_HAS_NBAT_I        2 /* Dual battery current */

/* CORVON 743V2 GPIOs ****************************************************************/

/* LEDs - Blue / Red / Green (active LOW) */
#define GPIO_nLED_BLUE    /* PE4 */ (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#define GPIO_nLED_RED     /* PE5 */ (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)
#define GPIO_nLED_GREEN   /* PE6 */ (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN6)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_STATE_LED  LED_BLUE

/* I2C Buses */
#define PX4_I2C_BUS_EXPANSION   1   /* I2C1 - external (compass / airspeed / 4-pin I2C connector) */
#define PX4_I2C_BUS_ONBOARD     2   /* I2C2 - onboard BMP581 + IST8310 */
#define PX4_I2C_BUS_LED         PX4_I2C_BUS_ONBOARD

/* SPI Buses */
#define PX4_SPI_BUS_SENSORS     3   /* SPI3 - ICM-42688P + BMI088 */

/* Onboard IMU chip selects (SPI3) */
#define GPIO_SPI3_CS1_ICM42688  /* PA15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN15)
#define GPIO_SPI3_CS2_BMI088_G  /* PD5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN5)
#define GPIO_SPI3_CS3_BMI088_A  /* PD4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN4)

/* IMU Data-Ready (DRDY) interrupts */
#define GPIO_DRDY_ICM42688      /* PD10 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN10)
#define GPIO_DRDY_BMI088_G      /* PD11 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN11)
#define GPIO_DRDY_BMI088_A      /* PD6  */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN6)

/****************************************************************************************************
 * ADC Channels
 ****************************************************************************************************/

#define ADC_CHANNELS_COUNT      6

/* GPIO configs for the ADC inputs - consumed by PX4_GPIO_INIT_LIST.
 * Legacy pinmap macro names (no "_0" suffix) because CONFIG_STM32H7_USE_LEGACY_PINMAP
 * defaults to y in NuttX. */
#define PX4_ADC_GPIO  \
	/* PC0 */  GPIO_ADC123_INP10, \
	/* PC1 */  GPIO_ADC123_INP11, \
	/* PC4 */  GPIO_ADC12_INP4,   \
	/* PC5 */  GPIO_ADC12_INP8,   \
	/* PA4 */  GPIO_ADC12_INP18,  \
	/* PA6 */  GPIO_ADC12_INP3

/* Battery 1 (main via PM connector + B+ pad) */
#define ADC_BATTERY1_VOLTAGE_CHANNEL    10    /* PC0 - ADC123_INP10 */
#define ADC_BATTERY1_CURRENT_CHANNEL    11    /* PC1 - ADC123_INP11 */

/* Battery 2 (backup via B2V / B2I pads) */
#define ADC_BATTERY2_VOLTAGE_CHANNEL    4     /* PC4 - ADC12_INP4 */
#define ADC_BATTERY2_CURRENT_CHANNEL    8     /* PC5 - ADC12_INP8 */

/* Airspeed (analog) */
#define ADC_AIRSPEED_IN_CHANNEL         18    /* PA4 - ADC12_INP18 */

/* RSSI (analog) */
#define ADC_RSSI_IN_CHANNEL             3     /* PA6 - ADC12_INP3 */

#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL) | \
	 (1 << ADC_BATTERY2_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY2_CURRENT_CHANNEL) | \
	 (1 << ADC_AIRSPEED_IN_CHANNEL)      | \
	 (1 << ADC_RSSI_IN_CHANNEL))

/* Battery voltage divider R82=20K (top) / R62=1K (bottom), V_IN = V_ADC * 21.2 */
#define BOARD_BATTERY1_V_DIV         (21.2f)
#define BOARD_BATTERY2_V_DIV         (21.2f)

/* Battery current amps per volt - calibrate per PM sensor */
#define BOARD_BATTERY1_A_PER_V       (40.0f)
#define BOARD_BATTERY2_A_PER_V       (40.0f)

/****************************************************************************************************
 * PWM / Motor outputs
 ****************************************************************************************************/

#define DIRECT_PWM_OUTPUT_CHANNELS  12

/* Number of io_timers used (TIM1, TIM3, TIM4, TIM12) - must match
 * timer_config.cpp. Default MAX_IO_TIMERS is 2 without this override. */
#define BOARD_NUM_IO_TIMERS         4

/* Motor 1..10 on TIM1/TIM3/TIM4 (DShot-capable), Servo 11/12 on TIM12 (PWM only) */
/* Actual timer, channel, and pin mapping is in src/timer_config.cpp */

/****************************************************************************************************
 * HRT (High Resolution Timer) on TIM2.
 *
 * TIM2 was originally reserved for the WS2812 LED strip, but the NeoPixel
 * driver is disabled in default.px4board (CONFIG_DRIVERS_LIGHTS_NEOPIXEL=n),
 * which frees TIM2 for HRT. STM32H743 has only two 32-bit timers (TIM2 and
 * TIM5); using TIM5 for HRT was observed to cause wq:I2C2 schedule
 * starvation on this board (BMP581 / IST8310 work queue would not fire
 * after the first few transfers). The exact mechanism is not pinned down
 * (likely an interaction between the timer interrupt vector / priority and
 * the I2C2 work queue), but TIM2 reproduces the v1 known-working
 * configuration. If NeoPixel is re-enabled later and TIM2 is needed for it,
 * NeoPixel should move to a different timer (e.g., a free 16-bit one)
 * rather than reverting HRT to TIM5.
 ****************************************************************************************************/

#define HRT_TIMER                   2
#define HRT_TIMER_CHANNEL           1

/* Tone alarm (buzzer) on PA7 / TIM14_CH1 (AF9) */
#define TONE_ALARM_TIMER            14
#define TONE_ALARM_CHANNEL          1
#define GPIO_BUZZER_1               /* PA7 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN7)
#define GPIO_TONE_ALARM_IDLE        GPIO_BUZZER_1
#define GPIO_TONE_ALARM             GPIO_TIM14_CH1OUT_1

/* DMA allocator pool for SPI/UART coherent buffers on H7 */
#define BOARD_DMA_ALLOC_POOL_SIZE   5120

/* Store parameters in internal flash (sector 15 = last 128 KB of 2 MB flash -
 * matches the bootloader's APP_RESERVATION_SIZE carving). */
#define FLASH_BASED_PARAMS

/****************************************************************************************************
 * High-level features
 ****************************************************************************************************/

/* USB OTG_FS
 *
 * V2 has no physical VBUS sense wire, but several places reference
 * GPIO_OTGFS_VBUS unconditionally:
 *   - bootloader common main.c (skipped via BOARD_USB_VBUS_SENSE_DISABLED)
 *   - platforms/nuttx/src/px4/common/board_ctrl.c (board_read_VBUS_state())
 *   - platforms/nuttx/src/px4/common/cdc_acm_check.cpp gates sercon_main()
 *     on board_read_VBUS_state() returning PX4_OK
 *
 * So GPIO_OTGFS_VBUS must read HIGH for the application's CDC/ACM auto-start
 * logic to ever fire and register /dev/ttyACM0.  We point it at PE3 (an
 * unused pin per the resource doc) configured as INPUT with internal
 * pull-up, which makes board_read_VBUS_state() always report "USB present"
 * and unconditionally lets cdcacm_autostart bring up CDC/ACM. The actual
 * USB power-source mode is bus-powered (CONFIG_USBDEV_BUSPOWERED=y in
 * defconfig); this fake-VBUS approach only spoofs the cable-detect logic.
 */
#define BOARD_USB_VBUS_SENSE_DISABLED     1
#define GPIO_OTGFS_VBUS      /* PE3 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN3)

/* SDMMC1 - 4-bit SD card logging */
#define BOARD_HAS_ON_RESET                1
#define BOARD_ENABLE_CONSOLE_BUFFER       1

/* SD card slot parameters (consumed by src/sdio.c) */
#define SDIO_SLOTNO                       0  /* Only one slot */
#define SDIO_MINOR                        0

/****************************************************************************************************
 * User switches (SW1-SW4) as generic GPIO pads (NOT safety switch - SW1 is a custom pad)
 ****************************************************************************************************/

#define GPIO_SW1             /* PD0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN0)
#define GPIO_SW2             /* PA8 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)
#define GPIO_SW3             /* PD1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN1)
#define GPIO_SW4             /* PE2 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN2)

/****************************************************************************************************
 * CAN (FDCAN1) - single interface, statically configured via
 * CONFIG_BOARD_UAVCAN_INTERFACES="1" in default.px4board.  Do NOT define
 * UAVCAN_NUM_IFACES_RUNTIME here (that forces us to implement
 * board_get_can_interfaces() at the board level).
 ****************************************************************************************************/

/****************************************************************************************************
 * Power supply
 ****************************************************************************************************/

#define BOARD_NUMBER_BRICKS                 2
/* No hardware "brick valid" sense on V2 - battery is assumed always valid
 * whenever the board is powered (voltage presence is inferred from the ADC
 * reading itself in the battery_status module). */
#define BOARD_ADC_BRICK1_VALID              (true)
#define BOARD_ADC_BRICK2_VALID              (true)

/****************************************************************************************************
 * Serial connection table
 ****************************************************************************************************/

/*                  UART              Device Path   Purpose                */
#define BOARD_NUMBER_DIGITAL_BRICKS   0
#define BOARD_HAS_DUAL_BATTERY        1

/****************************************************************************************************
 * GPIO auto-init list - consumed by px4_gpio_init() in stm32_boardinitialize()
 ****************************************************************************************************/

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,        \
		GPIO_CAN1_TX,        \
		GPIO_CAN1_RX,        \
	}

/****************************************************************************************************
 * Public Function Prototypes
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

__BEGIN_DECLS

extern void stm32_spiinitialize(void);
extern void board_peripheral_reset(int ms);
extern void board_on_reset(int status);
extern void stm32_usbinitialize(void);

/* Initialise SDIO-based MMC/SD card support (defined in src/sdio.c) */
int stm32_sdio_initialize(void);

#include <px4_platform_common/board_common.h>

__END_DECLS

#endif /* __ASSEMBLY__ */
