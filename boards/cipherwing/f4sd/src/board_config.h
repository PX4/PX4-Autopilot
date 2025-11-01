/****************************************************************************
 *
 *   Copyright (c) 2018, 2014 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/
/**
 * @file board_config.h
 * @brief CIPHERWING F4SD board (6 PWM, ICM20689, SD kept, PB8/PB9 = I2C1, PPM disabled)
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/* ===== LEDs ===== */
// power - green
// LED1 - PB5 - blue
#define GPIO_LED1           (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)
#define GPIO_LED_BLUE       GPIO_LED1
#define BOARD_OVERLOAD_LED  LED_BLUE

#define FLASH_BASED_PARAMS

/* ===== ADC channels =====
 * CH12: battery voltage
 * CH11: battery current
 * CH0 : RSSI
 */
#define ADC_CHANNELS ((1 << 0) | (1 << 11) | (1 << 12))
#define ADC_BATTERY_VOLTAGE_CHANNEL  12
#define ADC_BATTERY_CURRENT_CHANNEL  11
#define ADC_RC_RSSI_CHANNEL           0

/* ===== PWM outputs (6 total) =====
 * No UART conflicts; no TIM1/5/12 usage.
 *
 * M1 PB0 TIM3_CH3
 * M2 PB1 TIM3_CH4
 * M3 PC8 TIM8_CH3
 * M4 PC9 TIM8_CH4
 * M5 PC6 TIM8_CH1   (uses pin that could be UART6_TX; ensure UART6 unused)
 * M6 PB7 TIM4_CH2
 */
#define _MK_GPIO_INPUT(def)  (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))
#define _MK_GPIO_OUTPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR))

#define GPIO_GPIO0_OUTPUT  _MK_GPIO_OUTPUT(GPIO_TIM3_CH3OUT)  /* PB0 */
#define GPIO_GPIO1_OUTPUT  _MK_GPIO_OUTPUT(GPIO_TIM3_CH4OUT)  /* PB1 */
#define GPIO_GPIO2_OUTPUT  _MK_GPIO_OUTPUT(GPIO_TIM8_CH3OUT)  /* PC8 */
#define GPIO_GPIO3_OUTPUT  _MK_GPIO_OUTPUT(GPIO_TIM8_CH4OUT)  /* PC9 */
#define GPIO_GPIO4_OUTPUT  _MK_GPIO_OUTPUT(GPIO_TIM8_CH1OUT)  /* PC6 */
#define GPIO_GPIO5_OUTPUT  _MK_GPIO_OUTPUT(GPIO_TIM4_CH2OUT)  /* PB7 */

#define DIRECT_PWM_OUTPUT_CHANNELS 6

/* ===== USB OTG FS =====
 * PA9 OTG_FS_VBUS sensing
 */
#define GPIO_OTGFS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTC|GPIO_PIN5)

/* ===== High-resolution timer / PPM =====
 * Keep HRT on TIM4 CH1 as stock.
 * Disable PPM entirely so PB8/PB9 are free for I2C1:
 *  - Do NOT define HRT_PPM_CHANNEL (let it be absent)
 *  - Ensure GPIO_PPM_IN is undefined
 */
#define HRT_TIMER          4   // TIM4
#define HRT_TIMER_CHANNEL  1   // CH1
/* PPM disabled: */
#undef GPIO_PPM_IN
/* (intentionally no #define HRT_PPM_CHANNEL here) */

#define RC_SERIAL_PORT "/dev/ttyS0"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

/* ===== IMU: ICM20689 over SPI1 (instead of MPU6000) ===== */
#define PX4_SPIDEV_MPU       PX4_SPIDEV_EXT0
#define GPIO_SPI1_MPU_CS     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
#define DRV_IMU_DEVTYPE_MPU  DRV_IMU_DEVTYPE_ICM20689
#define BOARD_HAS_SENSOR_ICM20689 1

/* ===== DMA + Console buffer (stock) ===== */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120
#define BOARD_HAS_ON_RESET 1
#define BOARD_ENABLE_CONSOLE_BUFFER
#define BOARD_CONSOLE_BUFFER_SIZE (1024*3)

__BEGIN_DECLS

extern void stm32_spiinitialize(void);
extern void stm32_usbinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

__END_DECLS

