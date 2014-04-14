/*
 * @brief Basic CMSIS include file for LPC11CXX
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
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
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#ifndef __CMSIS_11CXX_H_
#define __CMSIS_11CXX_H_

#include "lpc_types.h"
#include "sys_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup CMSIS_11CXX CHIP: LPC11Cxx CMSIS include file
 * @ingroup CHIP_11XX_CMSIS_Drivers
 * Applies to LPC1111, LPC1112, LPC1113, LPC1114, LPC11D14, LPC1115,
 * LPC11C12, LPC11C13, LPC11C22, and LPC11C34 devices.
 * @{
 */

#if defined(__ARMCC_VERSION)
// Kill warning "#pragma push with no matching #pragma pop"
  #pragma diag_suppress 2525
  #pragma push
  #pragma anon_unions
#elif defined(__CWCC__)
  #pragma push
  #pragma cpp_extensions on
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
//  #pragma push // FIXME not usable for IAR
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

#if !defined(CHIP_LPC11CXX)
#error Incorrect or missing device variant (CHIP_LPC11AXX)
#endif

/** @defgroup CMSIS_11CXX_IRQ CHIP_LPC11CXX: LPC11CXX/LPC111X peripheral interrupt numbers
 * @{
 */

typedef enum LPC11CXX_IRQn {
	NonMaskableInt_IRQn           = -14,	/*!< 2 Non Maskable Interrupt                           */
	HardFault_IRQn                = -13,	/*!< 3 Cortex-M0 Hard Fault Interrupt                   */
	SVCall_IRQn                   = -5,		/*!< 11 Cortex-M0 SV Call Interrupt                     */
	PendSV_IRQn                   = -2,		/*!< 14 Cortex-M0 Pend SV Interrupt                     */
	SysTick_IRQn                  = -1,		/*!< 15 Cortex-M0 System Tick Interrupt                 */

	PIO0_0_IRQn                   = 0,		/*!< GPIO port 0, pin 0 Interrupt                      */
	PIO0_1_IRQn                   = 1,		/*!< GPIO port 0, pin 1 Interrupt                      */
	PIO0_2_IRQn                   = 2,		/*!< GPIO port 0, pin 2 Interrupt                      */
	PIO0_3_IRQn                   = 3,		/*!< GPIO port 0, pin 3 Interrupt                      */
	PIO0_4_IRQn                   = 4,		/*!< GPIO port 0, pin 4 Interrupt                      */
	PIO0_5_IRQn                   = 5,		/*!< GPIO port 0, pin 5 Interrupt                      */
	PIO0_6_IRQn                   = 6,		/*!< GPIO port 0, pin 6 Interrupt                      */
	PIO0_7_IRQn                   = 7,		/*!< GPIO port 0, pin 7 Interrupt                      */
	PIO0_8_IRQn                   = 8,		/*!< GPIO port 0, pin 8 Interrupt                      */
	PIO0_9_IRQn                   = 9,		/*!< GPIO port 0, pin 9 Interrupt                      */
	PIO0_10_IRQn                  = 10,		/*!< GPIO port 0, pin 10 Interrupt                     */
	PIO0_11_IRQn                  = 11,		/*!< GPIO port 0, pin 11 Interrupt                     */
	PIO1_0_IRQn                   = 12,		/*!< GPIO port 1, pin 0 Interrupt                      */
	CAN_IRQn                      = 13,		/*!< CAN Interrupt                                    */
	SSP1_IRQn                     = 14,		/*!< SSP1 Interrupt                                   */
	I2C0_IRQn                     = 15,		/*!< I2C Interrupt                                    */
	TIMER_16_0_IRQn               = 16,		/*!< 16-bit Timer0 Interrupt                          */
	TIMER_16_1_IRQn               = 17,		/*!< 16-bit Timer1 Interrupt                          */
	TIMER_32_0_IRQn               = 18,		/*!< 32-bit Timer0 Interrupt                          */
	TIMER_32_1_IRQn               = 19,		/*!< 32-bit Timer1 Interrupt                          */
	SSP0_IRQn                     = 20,		/*!< SSP0 Interrupt                                   */
	UART0_IRQn                    = 21,		/*!< UART Interrupt                                   */
	Reserved22_IRQn               = 22,
	Reserved23_IRQn               = 23,
	ADC_IRQn                      = 24,		/*!< A/D Converter Interrupt                          */
	WDT_IRQn                      = 25,		/*!< Watchdog timer Interrupt                         */
	BOD_IRQn                      = 26,		/*!< Brown Out Detect(BOD) Interrupt                  */
	Reserved27_IRQn               = 27,
	EINT3_IRQn                    = 28,		/*!< External Interrupt 3 Interrupt                   */
	EINT2_IRQn                    = 29,		/*!< External Interrupt 2 Interrupt                   */
	EINT1_IRQn                    = 30,		/*!< External Interrupt 1 Interrupt                   */
	EINT0_IRQn                    = 31,		/*!< External Interrupt 0 Interrupt                   */
} LPC11CXX_IRQn_Type;

/**
 * @}
 */

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/** @defgroup CMSIS_11CXX_COMMON CHIP: LPC11Cxx Cortex CMSIS definitions
 * @{
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __MPU_PRESENT             0			/*!< MPU present or not                               */
#define __NVIC_PRIO_BITS          2			/*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0			/*!< Set to 1 if different SysTick Config is used     */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __CMSIS_11CXX_H_ */
