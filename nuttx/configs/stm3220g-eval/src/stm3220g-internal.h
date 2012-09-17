/****************************************************************************************************
 * configs/stm3220g_eval/src/stm3220g_internal.h
 * arch/arm/src/board/stm3220g_internal.n
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ****************************************************************************************************/

#ifndef __CONFIGS_STM3220G_EVAL_SRC_STM3220G_INTERNAL_H
#define __CONFIGS_STM3220G_EVAL_SRC_STM3220G_INTERNAL_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/
/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#endif

/* You can use either CAN1 or CAN2, but you can't use both because they share the same transceiver */

#if defined(CONFIG_STM32_CAN1) && defined(CONFIG_STM32_CAN2)
#  warning "The STM3250G-EVAL will only support one of CAN1 and CAN2"
#endif

/* You can't use CAN1 with FSMC:
 *
 *   PD0   = FSMC_D2 & CAN1_RX   
 *   PD1   = FSMC_D3 & CAN1_TX  
 */

#if defined(CONFIG_STM32_CAN1) && defined(CONFIG_STM32_FSMC)
#  warning "The STM3250G-EVAL will only support one of CAN1 and FSMC"
#endif

/* The USB OTG HS ULPI bus is shared with CAN2 bus:
 *
 *   PB13  = ULPI_D6 & CAN2_TX
 *   PB5   = ULPI_D7 & CAN2_RX
 */

#if defined(CONFIG_STM32_CAN2) && defined(CONFIG_STM32_OTGHS)
#  warning "The STM3250G-EVAL will only support one of CAN2 and USB OTG HS"
#endif

/* STM3220G-EVAL GPIOs ******************************************************************************/
/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN6)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN8)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN9)
#define GPIO_LED4       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN7)

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON   BUTTON_WAKEUP
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  (BUTTON_USER - BUTTON_WAKEUP + 1)

#define GPIO_BTN_WAKEUP (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)
#define GPIO_BTN_TAMPER (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)
#define GPIO_BTN_USER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN15)

/* PWM
 *
 * The STM3220G-Eval has no real on-board PWM devices, but the board can be
 * configured to output a pulse train using TIM4, TIM1, or TIM8 (see board.h).
 * Let's figure out which the user has configured.
 */

#ifdef CONFIG_PWM
#  if defined(CONFIG_STM32_TIM1_PWM)
#    define STM3220G_EVAL_PWMTIMER 1
#  elif defined(CONFIG_STM32_TIM4_PWM)
#    define STM3220G_EVAL_PWMTIMER 4
#  elif defined(CONFIG_STM32_TIM8_PWM)
#    define STM3220G_EVAL_PWMTIMER 8
#  endif
#endif

/* USB OTG FS 
 *
 * PA9  VBUS_FS
 * PH5  OTG_FS_PowerSwitchOn
 * PF11 OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)
#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_OUTPUT_SET|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTH|GPIO_PIN5)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTF|GPIO_PIN11)
#else
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTF|GPIO_PIN11)
#endif

/* The STM3220G-EVAL has two STMPE811QTR I/O expanders on board both connected
 * to the STM32 via I2C1.  They share a common interrupt line: PI2.
 * 
 * STMPE811 U24, I2C address 0x41 (7-bit)
 * ------ ---- ---------------- --------------------------------------------
 * STPE11 PIN  BOARD SIGNAL     BOARD CONNECTION
 * ------ ---- ---------------- --------------------------------------------
 *   Y-        TouchScreen_Y-   LCD Connector XL
 *   X-        TouchScreen_X-   LCD Connector XR
 *   Y+        TouchScreen_Y+   LCD Connector XD
 *   X+        TouchScreen_X+   LCD Connector XU
 *   IN3       EXP_IO9
 *   IN2       EXP_IO10
 *   IN1       EXP_IO11
 *   IN0       EXP_IO12
 * 
 * STMPE811 U29, I2C address 0x44 (7-bit)
 * ------ ---- ---------------- --------------------------------------------
 * STPE11 PIN  BOARD SIGNAL     BOARD CONNECTION
 * ------ ---- ---------------- --------------------------------------------
 *   Y-        EXP_IO1
 *   X-        EXP_IO2
 *   Y+        EXP_IO3
 *   X+        EXP_IO4
 *   IN3       EXP_IO5
 *   IN2       EXP_IO6
 *   IN1       EXP_IO7
 *   IN0       EXP_IO8
 */

#define STMPE811_ADDR1    0x41
#define STMPE811_ADDR2    0x44

#define GPIO_IO_EXPANDER (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTI|GPIO_PIN2)

/* GPIO settings that will be altered when external memory is selected:
 *
 * ----- ------- -------- ------------ ------- ------------ -------- ----------- -------- -------------
 * PB7:  FSMC NL PD0-1:   FSMC D2-D3   PE0:    FSMC NBL0    PF0-5:   FSMC A0-A5  PG0-5:   FSMC A10-A15
 *               PD3:     FSMC CLK     PE1:    FSMC BLN1    PF6:     FSMC NIORD  PG6-7:   FSMC INT2-3
 *               PD4:     FSMC NOE     PE2:    FSMC A23     PF7:     FSMC NREG   PG9:     FSMC NCE3
 *               PD5:     FSMC NWE     PE3-6:  FSMC A19-A22 PF8:     FSMC NIOWR  PG9-10:  FSMC NE2-3
 *               PD6:     FSMC NWAIT   PE7-15: FSMC D4-D12  PF9:     FSMC CD     PG10:    FSMC NCE4 (1)
 *               PD7:     FSMC NE1                          PF10:    FSMC INTR   PG11:    FSMC NCE4 (2)
 *               PD7:     FSMC NCE2                         PF12-15: FSMC A6-A9  PG12:    FSMC NE4
 *               PD8-10:  FSMC D13-D15                                           PG13-14: FSMC A24-A25
 *               PD11-13: FSMC_A16-A18
 *               PD14-15: FSMC D0-D1
 */

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
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the STM3220G-EVAL board.
 *
 ****************************************************************************************************/

void weak_function stm32_spiinitialize(void);

/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in inialization to setup USB-related GPIO pins for
 *   the STM3220G-EVAL board.
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host functionality. This function will
 *   start a thread that will monitor for device connection/disconnection events.
 *
 ****************************************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
int stm32_usbhost_initialize(void);
#endif

/****************************************************************************************************
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for external memory usage
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemgpios(const uint32_t *gpios, int ngpios);
#endif

/****************************************************************************************************
 * Name: stm32_extmemaddr
 *
 * Description:
 *   Initialize adress line GPIOs for external memory access
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemaddr(int naddrs);
#endif

/****************************************************************************************************
 * Name: stm32_extmemdata
 *
 * Description:
 *   Initialize data line GPIOs for external memory access
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemdata(int ndata);
#endif

/************************************************************************************
 * Name: stm32_enablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_enablefsmc(void);
#endif

/************************************************************************************
 * Name: stm32_disablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_disablefsmc(void);
#endif

/************************************************************************************
 * Name: stm32_selectsram
 *
 * Description:
 *   Initialize to access external SRAM.  SRAM will be visible at the FSMC Bank 
 *   NOR/SRAM2 base address (0x64000000)
 *
 *   General transaction rules.  The requested AHB transaction data size can be 8-,
 *   16- or 32-bit wide whereas the SRAM has a fixed 16-bit data width. Some simple
 *   transaction rules must be followed:
 *
 *   Case 1: AHB transaction width and SRAM data width are equal
 *     There is no issue in this case.
 *   Case 2: AHB transaction size is greater than the memory size
 *     In this case, the FSMC splits the AHB transaction into smaller consecutive
 *     memory accesses in order to meet the external data width.
 *   Case 3: AHB transaction size is smaller than the memory size.
 *     SRAM supports the byte select feature.
 *     a) FSMC allows write transactions accessing the right data through its
 *        byte lanes (NBL[1:0])
 *     b) Read transactions are allowed (the controller reads the entire memory
 *        word and uses the needed byte only). The NBL[1:0] are always kept low
 *        during read transactions.
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_selectsram(void);
#endif

/************************************************************************************
 * Name: stm32_deselectsram
 *
 * Description:
 *   Disable SRAM
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_deselectsram(void);
#endif

/************************************************************************************
 * Name: stm32_selectlcd
 *
 * Description:
 *   Initialize to the LCD
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_selectlcd(void);
#endif

/************************************************************************************
 * Name: stm32_deselectlcd
 *
 * Description:
 *   Disable the LCD
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_deselectlcd(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_STM3220G_EVAL_SRC_STM3220G_INTERNAL_H */

