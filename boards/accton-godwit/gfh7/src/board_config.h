/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdbool.h>
#include <stdint.h>
#include <stm32_gpio.h>

/* Status LEDs, active low. */
#define GPIO_nLED_GREEN  /* PC13 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
#define GPIO_nLED_BLUE   /* PE10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN10)

#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED             LED_GREEN
#define BOARD_ARMED_STATE_LED          LED_BLUE

/* ADC1 inputs. */
#define PX4_ADC_GPIO \
	GPIO_ADC123_INP10, /* PC0: voltage */ \
	GPIO_ADC12_INP9,   /* PB0: current */ \
	GPIO_ADC12_INP5    /* PB1: RSSI */

#define ADC_BATTERY_VOLTAGE_CHANNEL  10
#define ADC_BATTERY_CURRENT_CHANNEL  9
#define ADC_RSSI_IN_CHANNEL          5
#define ADC_CHANNELS ((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
		      (1 << ADC_BATTERY_CURRENT_CHANNEL) | \
		      (1 << ADC_RSSI_IN_CHANNEL))

/* 160 kOhm / 10 kOhm divider. Current scaling remains intentionally unset. */
#define BOARD_BATTERY1_V_DIV 17.0f

/* Eight motor outputs: TIM1 CH1-4 and TIM8 CH1-4. */
#define DIRECT_PWM_OUTPUT_CHANNELS 8
#define BOARD_NUM_IO_TIMERS        2

/* TIM3 is free from TIM2 NeoPixel and TIM4 IMU clock duties. */
#define HRT_TIMER         3
#define HRT_TIMER_CHANNEL 1

/* PE12 drives the external buzzer transistor. GPIO mode supports active buzzers. */
#define GPIO_TONE_ALARM_IDLE /* PE12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN12)
#define GPIO_TONE_ALARM_GPIO GPIO_TONE_ALARM_IDLE

/* Board controls. */
#define GPIO_HEATER       /* PE3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)
#define GPIO_12V_EN       /* PC12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN12)
#define PAYLOAD_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_12V_EN, (on_true))
#define GPIO_HW_VER0      /* PC14 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN14)
#define GPIO_HW_VER1      /* PC15 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN15)
#define GPIO_PHY_RST_SAFE /* PD15 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTD|GPIO_PIN15)

/* No physical USB VBUS sense; PA9 is an unconnected test point. */
#define BOARD_USB_VBUS_SENSE_DISABLED 1
#define GPIO_OTGFS_VBUS /* PA9 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)
#define BOARD_ADC_USB_CONNECTED (1)
#define BOARD_ADC_USB_VALID     BOARD_ADC_USB_CONNECTED

/* SDMMC2, card detect is active low. */
#define SDIO_SLOTNO 0
#define SDIO_MINOR  0
#define GPIO_SDMMC2_NCD /* PA15 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTA|GPIO_PIN15)

/* Up to eight WS2812/SK6812-compatible LEDs on TIM2 CH3 / PB10. */
#if defined(USE_S_RGB_LED_DMA)
# define BOARD_HAS_N_S_RGB_LED 8
# define BOARD_MAX_LEDS        BOARD_HAS_N_S_RGB_LED
# define S_RGB_LED_DMA         DMAMAP_DMA12_TIM2CH3_0
# define S_RGB_LED_TIMER       2
# define S_RGB_LED_CHANNEL     3
# define S_RGB_LED_TIM_GPIO    GPIO_TIM2_CH3OUT_2
#endif

#define BOARD_DMA_ALLOC_POOL_SIZE 5120
#define BOARD_HAS_ON_RESET        1
#define BOARD_ENABLE_CONSOLE_BUFFER
#define FLASH_BASED_PARAMS

#define BOARD_ADC_BRICK_VALID (true)
#define RC_SERIAL_PORT "/dev/ttyS3"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

#define PX4_GPIO_INIT_LIST { \
			PX4_ADC_GPIO, \
			GPIO_CAN1_TX, GPIO_CAN1_RX, \
			GPIO_HEATER, GPIO_12V_EN, GPIO_TONE_ALARM_IDLE, \
			GPIO_HW_VER0, GPIO_HW_VER1, GPIO_PHY_RST_SAFE \
	}

__BEGIN_DECLS

int stm32_sdio_initialize(void);
extern void stm32_spiinitialize(void);
extern void stm32_usbinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

__END_DECLS
