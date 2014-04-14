/*
 * @brief LPC11xx basic chip inclusion file
 *
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#ifndef __CHIP_H_
#define __CHIP_H_

#include "lpc_types.h"
#include "sys_config.h"
#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CORE_M0
#error CORE_M0 is not defined for the LPC11xx architecture
#error CORE_M0 should be defined as part of your compiler define list
#endif

#if !defined(ENABLE_UNTESTED_CODE)
#if defined(CHIP_LPC110X)
#warning The LCP110X code has not been tested with a platform. This code should \
build without errors but may not work correctly for the device. To disable this \
#warning message, define ENABLE_UNTESTED_CODE.
#endif
#if defined(CHIP_LPC11XXLV)
#warning The LPC11XXLV code has not been tested with a platform. This code should \
build without errors but may not work correctly for the device. To disable this \
#warning message, define ENABLE_UNTESTED_CODE.
#endif
#if defined(CHIP_LPC11AXX)
#warning The LPC11AXX code has not been tested with a platform. This code should \
build without errors but may not work correctly for the device. To disable this \
#warning message, define ENABLE_UNTESTED_CODE.
#endif
#if defined(CHIP_LPC11EXX)
#warning The LPC11EXX code has not been tested with a platform. This code should \
build without errors but may not work correctly for the device. To disable this \
warning message, define ENABLE_UNTESTED_CODE.
#endif
#endif

#if !defined(CHIP_LPC110X) && !defined(CHIP_LPC11XXLV) && !defined(CHIP_LPC11AXX) && \
	!defined(CHIP_LPC11CXX) && !defined(CHIP_LPC11EXX) && !defined(CHIP_LPC11UXX) && \
	!defined(CHIP_LPC1125)
#error CHIP_LPC110x/CHIP_LPC11XXLV/CHIP_LPC11AXX/CHIP_LPC11CXX/CHIP_LPC11EXX/CHIP_LPC11UXX/CHIP_LPC1125 is not defined!
#endif

/* Peripheral mapping per device
   Peripheral					Device(s)
   ----------------------------	-------------------------------------------------------------------------------------------------------------
   I2C(40000000)								CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX	CHIP_LPC1125
   WDT(40004000)				CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX	CHIP_LPC1125
   UART0(40008000)				CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX													CHIP_LPC1125
   UART1(40020000)																												CHIP_LPC1125
   UART2(40024000)																												CHIP_LPC1125
   USART/SMARTCARD(40008000)													CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   TIMER0_16(4000C000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX	CHIP_LPC1125
   TIMER1_16(40010000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX	CHIP_LPC1125
   TIMER0_32(40014000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX	CHIP_LPC1125
   TIMER1_32(40018000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX	CHIP_LPC1125
   ADC(4001C000)				CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX	CHIP_LPC1125
   DAC(40024000)																								CHIP_LPC11AXX
   ACMP(40028000)																								CHIP_LPC11AXX
   PMU(40038000)				CHIP_LPC110x/					CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX					CHIP_LPC1125
   FLASH_CTRL(4003C000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX									CHIP_LPC1125
   FLASH_EEPROM(4003C000)																		CHIP_LPC11EXX/	CHIP_LPC11AXX
   SPI0(40040000)				CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX
   SSP0(40040000)																CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX	CHIP_LPC1125
   IOCONF(40044000)				CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX	CHIP_LPC1125
   SYSCON(40048000)				CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX	CHIP_LPC1125
   GPIOINTS(4004C000)															CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   USB(40080000)																CHIP_LPC11UXX
   CCAN(40050000)												CHIP_LPC11CXX
   SPI1(40058000)												CHIP_LPC11CXX
   SSP1(40058000)																CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX	CHIP_LPC1125
   GPIO_GRP0_INT(4005C000)														CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   GPIO_GRP1_INT(40060000)														CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   GPIO_PORT(50000000)															CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   GPIO_PIO0(50000000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX													CHIP_LPC1125
   GPIO_PIO1(50010000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX													CHIP_LPC1125
   GPIO_PIO2(50020000)							CHIP_LPC11XXLV/	CHIP_LPC11CXX													CHIP_LPC1125
   GPIO_PIO3(50030000)							CHIP_LPC11XXLV/	CHIP_LPC11CXX													CHIP_LPC1125
 */

/** @defgroup PERIPH_11XX_BASE CHIP: LPC11xx Peripheral addresses and register set declarations
 * @ingroup CHIP_11XX_Drivers
 * @{
 */

#define LPC_I2C_BASE              0x40000000
#define LPC_WWDT_BASE             0x40004000
#define LPC_USART_BASE            0x40008000
#define LPC_TIMER16_0_BASE        0x4000C000
#define LPC_TIMER16_1_BASE        0x40010000
#define LPC_TIMER32_0_BASE        0x40014000
#define LPC_TIMER32_1_BASE        0x40018000
#define LPC_ADC_BASE              0x4001C000
#define LPC_DAC_BASE              0x40024000
#define LPC_ACMP_BASE             0x40028000
#define LPC_PMU_BASE              0x40038000
#define LPC_FLASH_BASE            0x4003C000
#define LPC_SSP0_BASE             0x40040000
#define LPC_IOCON_BASE            0x40044000
#define LPC_SYSCTL_BASE           0x40048000
#define LPC_USB0_BASE             0x40080000
#define LPC_CAN0_BASE             0x40050000
#define LPC_SSP1_BASE             0x40058000
#if defined(CHIP_LPC1125)
#define LPC_USART1_BASE           0x40020000
#define LPC_USART2_BASE           0x40024000
#endif
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
#define LPC_GPIO_PIN_INT_BASE     0x4004C000
#define LPC_GPIO_GROUP_INT0_BASE  0x4005C000
#define LPC_GPIO_GROUP_INT1_BASE  0x40060000
#define LPC_GPIO_PORT_BASE        0x50000000
#else
#define LPC_GPIO_PORT0_BASE       0x50000000
#define LPC_GPIO_PORT1_BASE       0x50010000
#if defined(CHIP_LPC11XXLV) || defined(CHIP_LPC11CXX) || defined(CHIP_LPC1125)
#define LPC_GPIO_PORT2_BASE       0x50020000
#define LPC_GPIO_PORT3_BASE       0x50030000
#endif /* defined(CHIP_LPC11XXLV) || defined(CHIP_LPC11CXX) || defined(CHIP_LPC1125) */
#endif /* defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX) */
#define IAP_ENTRY_LOCATION        0X1FFF1FF1
#define LPC_ROM_API_BASE_LOC      0x1FFF1FF8

#if !defined(CHIP_LPC110x)
#define LPC_I2C                   ((LPC_I2C_T              *) LPC_I2C_BASE)
#endif

#define LPC_WWDT                  ((LPC_WWDT_T             *) LPC_WWDT_BASE)
#define LPC_USART                 ((LPC_USART_T            *) LPC_USART_BASE)
#define LPC_TIMER16_0             ((LPC_TIMER_T            *) LPC_TIMER16_0_BASE)
#define LPC_TIMER16_1             ((LPC_TIMER_T            *) LPC_TIMER16_1_BASE)
#define LPC_TIMER32_0             ((LPC_TIMER_T            *) LPC_TIMER32_0_BASE)
#define LPC_TIMER32_1             ((LPC_TIMER_T            *) LPC_TIMER32_1_BASE)
#define LPC_ADC                   ((LPC_ADC_T              *) LPC_ADC_BASE)

#if defined(CHIP_LPC1125)
#define LPC_USART0                LPC_USART
#define LPC_USART1                ((LPC_USART_T            *) LPC_USART1_BASE)
#define LPC_USART2                ((LPC_USART_T            *) LPC_USART2_BASE)
#endif

#if defined(CHIP_LPC11AXX)
#define LPC_DAC                   ((LPC_DAC_T              *) LPC_DAC_BASE)
#define LPC_CMP                   ((LPC_CMP_T              *) LPC_ACMP_BASE)
#endif

#define LPC_PMU                   ((LPC_PMU_T              *) LPC_PMU_BASE)
#define LPC_FMC                   ((LPC_FMC_T              *) LPC_FLASH_BASE)
#define LPC_SSP0                  ((LPC_SSP_T              *) LPC_SSP0_BASE)
#define LPC_IOCON                 ((LPC_IOCON_T            *) LPC_IOCON_BASE)
#define LPC_SYSCTL                ((LPC_SYSCTL_T           *) LPC_SYSCTL_BASE)
#if defined(CHIP_LPC11CXX) || defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX) || defined(CHIP_LPC1125)
#define LPC_SSP1                  ((LPC_SSP_T              *) LPC_SSP1_BASE)
#endif
#define LPC_USB                   ((LPC_USB_T              *) LPC_USB0_BASE)

#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
#define LPC_PININT                ((LPC_PIN_INT_T          *) LPC_GPIO_PIN_INT_BASE)
#define LPC_GPIOGROUP             ((LPC_GPIOGROUPINT_T     *) LPC_GPIO_GROUP_INT0_BASE)
#define LPC_GPIO                  ((LPC_GPIO_T             *) LPC_GPIO_PORT_BASE)
#else
#define LPC_GPIO                  ((LPC_GPIO_T             *) LPC_GPIO_PORT0_BASE)
#endif

#define LPC_ROM_API               (*((LPC_ROM_API_T        * *) LPC_ROM_API_BASE_LOC))


/**
 * @}
 */

/** @ingroup CHIP_11XX_DRIVER_OPTIONS
 */

/**
 * @brief	System oscillator rate
 * This value is defined externally to the chip layer and contains
 * the value in Hz for the external oscillator for the board. If using the
 * internal oscillator, this rate can be 0.
 */
extern const uint32_t OscRateIn;

/**
 * @}
 */

/** @ingroup CHIP_11XX_DRIVER_OPTIONS
 */

/**
 * @brief	Clock rate on the CLKIN pin
 * This value is defined externally to the chip layer and contains
 * the value in Hz for the CLKIN pin for the board. If this pin isn't used,
 * this rate can be 0.
 */
extern const uint32_t ExtRateIn;

/**
 * @}
 */

#include "pmu_11xx.h"
#include "fmc_11xx.h"
#include "sysctl_11xx.h"
#include "clock_11xx.h"
#include "iocon_11xx.h"
#include "timer_11xx.h"
#include "uart_11xx.h"
#include "wwdt_11xx.h"
#include "ssp_11xx.h"
#include "romapi_11xx.h"

#if !defined(CHIP_LPC1125)
/* All LPC1xx devices except the LPC1125 */
#include "adc_11xx.h"

#else
/* LPC1125 has different IP than other LPC11xx devices */
#include "adc_1125.h"
#endif

/* Different GPIO/GPIOGROUP/PININT blocks for parts with similar numbers */
#if defined(CHIP_LPC11CXX) || defined(CHIP_LPC110X) || defined(CHIP_LPC11XXLV) || defined(CHIP_LPC1125)
#include "gpio_11xx_2.h"
#else
#include "gpio_11xx_1.h"
#include "gpiogroup_11xx.h"
#include "pinint_11xx.h"
#endif

/* Family specific drivers */
#if defined(CHIP_LPC11AXX)
#include "acmp_11xx.h"
#include "dac_11xx.h"
#endif
#if !defined(CHIP_LPC110X)
#include "i2c_11xx.h"
#endif
#if defined(CHIP_LPC11CXX)
#include "ccand_11xx.h"
#endif
#if defined(CHIP_LPC11UXX)
#include "usbd_11xx.h"
#endif

/** @defgroup SUPPORT_11XX_FUNC CHIP: LPC11xx support functions
 * @ingroup CHIP_11XX_Drivers
 * @{
 */

/**
 * @brief	Current system clock rate, mainly used for sysTick
 */
extern uint32_t SystemCoreClock;

/**
 * @brief	Update system core clock rate, should be called if the
 *			system has a clock rate change
 * @return	None
 */
void SystemCoreClockUpdate(void);

/**
 * @brief	Set up and initialize hardware prior to call to main()
 * @return	None
 * @note	Chip_SystemInit() is called prior to the application and sets up
 * system clocking prior to the application starting.
 */
void Chip_SystemInit(void);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __CHIP_H_ */
