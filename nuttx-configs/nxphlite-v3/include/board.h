/************************************************************************************
 * configs/nxphlit-v3/include/board.h
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *   		  Jordan MacIntyre
 *   		  David Sidrane <david_s5@nscdg.com>
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
 ************************************************************************************/

#ifndef __CONFIG_NXPHLITE_V3_INCLUDE_BOARD_H
#define __CONFIG_NXPHLITE_V3_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/*
 *  The NXPHlite-v3 is populated with a MK66FN2M0VLQ18 has 2 MiB of FLASH and
 *  256 KiB of SRAM.
 */
/* Clocking *************************************************************************/
/* The NXPHlite-v3 uses a 16MHz external Oscillator.  The Kinetis MCU startup from an
 * internal digitally-controlled oscillator (DCO). Nuttx will enable the main external
 * oscillator (EXTAL0/XTAL0). The external oscillator/resonator can range from
 * 32.768 KHz up to 50 MHz. The default external source for the MCG oscillator inputs
 * EXTAL.
 *
 * Y1 a High-frequency, low-power Xtal
 */
#define BOARD_EXTAL_LP		 1
#define BOARD_EXTAL_FREQ     16000000       /* 16MHz Oscillator Y1 */
#define BOARD_XTAL32_FREQ    32768          /* 32KHz RTC Oscillator */

/* PLL Configuration.  Either the external clock or crystal frequency is used to
 * select the PRDIV value. Only reference clock frequencies are supported that will
 * produce a KINETIS_MCG_PLL_REF_MIN >= PLLIN <=KINETIS_MCG_PLL_REF_MAX
 * reference clock to the PLL.
 *
 *   PLL Input frequency:   PLLIN  = REFCLK / PRDIV = 16 MHz  / 2 = 8Mhz MHz
 *   PLL Output frequency:  PLLOUT = PLLIN  * VDIV  = 8 MHz  * 42 = 336 MHz
 *   MCG Frequency:         PLLOUT = 168 Mhz = 336 MHz / KINETIS_MCG_PLL_INTERNAL_DIVBY
 *
 * PRDIV register value is the divider minus KINETIS_MCG_C5_PRDIV_BASE.
 * VDIV  register value is offset by KINETIS_MCG_C6_VDIV_BASE.
 */

#define BOARD_PRDIV          2              /* PLL External Reference Divider */
#define BOARD_VDIV           42             /* PLL VCO Divider (frequency multiplier) */

/* Define additional MCG_C2 Setting */

#define BOARD_MCG_C2_FCFTRIM 0              /* Do not enable FCFTRIM */
#define BOARD_MCG_C2_LOCRE0  MCG_C2_LOCRE0  /* Enable reset on loss of clock */

#define BOARD_PLLIN_FREQ     (BOARD_EXTAL_FREQ / BOARD_PRDIV)
#define BOARD_PLLOUT_FREQ    (BOARD_PLLIN_FREQ * BOARD_VDIV)
#define BOARD_MCG_FREQ       (BOARD_PLLOUT_FREQ/KINETIS_MCG_PLL_INTERNAL_DIVBY)

/* SIM CLKDIV1 dividers */

#define BOARD_OUTDIV1        1              /* Core        = MCG,    168 MHz */
#define BOARD_OUTDIV2        3              /* Bus         = MCG / 3, 56 MHz */
#define BOARD_OUTDIV3        3              /* FlexBus     = MCG / 3, 56 MHz */
#define BOARD_OUTDIV4        6              /* Flash clock = MCG / 6, 28 MHz */

#define BOARD_CORECLK_FREQ  (BOARD_MCG_FREQ / BOARD_OUTDIV1)
#define BOARD_BUS_FREQ      (BOARD_MCG_FREQ / BOARD_OUTDIV2)
#define BOARD_FLEXBUS_FREQ  (BOARD_MCG_FREQ / BOARD_OUTDIV3)
#define BOARD_FLASHCLK_FREQ (BOARD_MCG_FREQ / BOARD_OUTDIV4)

/* SDHC clocking ********************************************************************/

/* SDCLK configurations corresponding to various modes of operation.   Formula is:
 *
 *   SDCLK  frequency = (base clock) / (prescaler * divisor)
 *
 * The SDHC module is always configure configured so that the core clock is the base
 * clock.  Possible values for presscaler and divisor are:
 *
 *   SDCLKFS: {2, 4, 8, 16, 32, 63, 128, 256}
 *   DVS:     {1..16}
 */

/* Identification mode:  Optimal 400KHz, Actual 168Mhz / (32 * 14) = 375 KHz */

#define BOARD_SDHC_IDMODE_PRESCALER    SDHC_SYSCTL_SDCLKFS_DIV32
#define BOARD_SDHC_IDMODE_DIVISOR      SDHC_SYSCTL_DVS_DIV(14)

/* MMC normal mode: Optimal 20MHz, Actual 168Mhz / (2 * 5) = 16.8 MHz */

#define BOARD_SDHC_MMCMODE_PRESCALER   SDHC_SYSCTL_SDCLKFS_DIV2
#define BOARD_SDHC_MMCMODE_DIVISOR     SDHC_SYSCTL_DVS_DIV(5)

/* SD normal mode (1-bit): Optimal 20MHz, Actual 168Mhz / (2 * 5) = 16.8 MHz */

#define BOARD_SDHC_SD1MODE_PRESCALER   SDHC_SYSCTL_SDCLKFS_DIV2
#define BOARD_SDHC_SD1MODE_DIVISOR     SDHC_SYSCTL_DVS_DIV(5)

/* SD normal mode (4-bit): Optimal 25MHz, Actual Actual 168Mhz / (2 * 4) = 21 MHz (with DMA)
 * SD normal mode (4-bit): Optimal 20MHz, Actual 168Mhz / (2 * 5) = 16.8 MHz (no DMA)
 */

#ifdef CONFIG_KINETIS_SDHC_DMA
#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV2
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(4)
#else
#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV2
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(5)
#endif

/* LED definitions ******************************************************************/
/* The NXPHlite-v3 has a separate Red, Green and Blue LEDs driven by the K66 as
 * follows:
 *
 *   LED    K66
 *   ------ -------------------------------------------------------
 *   RED    FB_CS0_b/ UART2_CTS_b / ADC0_SE5b / SPI0_SCK / FTM3_CH1/ PTD1
 *   GREEN  FTM2_FLT0/ CMP0_IN3/ FB_AD6 / I2S0_RX_BCLK/ FTM3_CH5/ ADC1_SE5b/ PTC9
 *   BLUE   CMP0_IN2/ FB_AD7 / I2S0_MCLK/ FTM3_CH4/ ADC1_SE4b/ PTC8
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_R       0
#define BOARD_LED_G       1
#define BOARD_LED_B       2
#define BOARD_NLEDS       3

/* LED bits for use with board_userled_all() */

#define BOARD_LED_R_BIT   (1 << BOARD_LED_R)
#define BOARD_LED_G_BIT   (1 << BOARD_LED_G)
#define BOARD_LED_B_BIT   (1 << BOARD_LED_B)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the NXPHlite-v3.  The following definitions describe how NuttX controls
 * the LEDs:
 *
 *   SYMBOL                Meaning                      LED state
 *                                                      RED   GREEN  BLUE
 *   -------------------  ----------------------------  ----------------- */
#define LED_STARTED       1 /* NuttX has been started    OFF   OFF    OFF */
#define LED_HEAPALLOCATE  2 /* Heap has been allocated   OFF   OFF    ON  */
#define LED_IRQSENABLED   0 /* Interrupts enabled        OFF   OFF    ON  */
#define LED_STACKCREATED  3 /* Idle stack created        OFF   ON     OFF */
#define LED_INIRQ         0 /* In an interrupt          (no change)       */
#define LED_SIGNAL        0 /* In a signal handler      (no change)       */
#define LED_ASSERTION     0 /* An assertion failed      (no change)       */
#define LED_PANIC         4 /* The system has crashed    FLASH OFF    OFF */
#undef  LED_IDLE            /* K66 is in sleep mode     (Not used)        */

/* Alternative pin resolution *******************************************************/
/* If there are alternative configurations for various pins in the
 * kinetis_k66pinmux.h header file, those alternative pins will be labeled with a
 * suffix like _1, _2, etc.  The logic in this file must select the correct pin
 * configuration for the board by defining a pin configuration (with no suffix) that
 * maps to the correct alternative.
 */

/* CAN
 *
 */
#define PIN_CAN0_RX       PIN_CAN0_RX_2
#define PIN_CAN0_TX       PIN_CAN0_TX_2

/* 12C
 *
 */

/* I2C0 MPL3115A2 Pressure Sensor */

#define PIN_I2C0_SCL     PIN_I2C0_SCL_4   /* PTE24 P_SCL */
#define PIN_I2C0_SDA     PIN_I2C0_SDA_4   /* PTE25 P_SDA */

/* I2C1 NFC Connector */

#define PIN_I2C1_SCL     PIN_I2C1_SCL_1   /* PTC10 NFC_SCL P2-2 */
#define PIN_I2C1_SDA     PIN_I2C1_SDA_1   /* PTC11 NFC_SDA P2-3 */


/* PWM
 *
 */

/* PWM Channels */

#define GPIO_FTM3_CH0OUT PIN_FTM3_CH0_2  /* PTE5  PWM1  P4-34 */
#define GPIO_FTM3_CH1OUT PIN_FTM3_CH1_2  /* PTE6  PWM2  P4-31 */
#define GPIO_FTM3_CH2OUT PIN_FTM3_CH2_2  /* PTE7  PWM3  P4-28 */
#define GPIO_FTM3_CH3OUT PIN_FTM3_CH3_2  /* PTE8  PWM4  P4-25 */
#define GPIO_FTM3_CH4OUT PIN_FTM3_CH4_2  /* PTE9  PWM5  P4-22 */
#define GPIO_FTM3_CH5OUT PIN_FTM3_CH5_2  /* PTE10 PWM6  P4-29 */
#define GPIO_FTM3_CH6OUT PIN_FTM3_CH6_2  /* PTE11 PWM7  P4-26 */
#define GPIO_FTM3_CH7OUT PIN_FTM3_CH7_2  /* PTE12 PWM8  P4-13 */

#define GPIO_FTM0_CH4OUT PIN_FTM0_CH4_3  /* PTD4  PWM0  P4-46 */
#define GPIO_FTM0_CH5OUT PIN_FTM0_CH5_3  /* PTD5  PWM10 P4-43 */
#define GPIO_FTM0_CH6OUT PIN_FTM0_CH6_2  /* PTD6  PWM11 P4-40 */
#define GPIO_FTM0_CH7OUT PIN_FTM0_CH7_2  /* PTD7  PWM12 P4-37 */

#define GPIO_FTM0_CH3OUT PIN_FTM0_CH3_1  /* PTA6  PWM13 P4-7 */
#define GPIO_FTM0_CH2OUT PIN_FTM0_CH2_2  /* PTC3  PWM14 P4-10 */

//todo:This is a Guess on timer utilisation
#define GPIO_FTM3_CH0IN  PIN_FTM3_CH0_2  /* PTE5  PWM1  P4-34 */
#define GPIO_FTM3_CH1IN  PIN_FTM3_CH1_2  /* PTE6  PWM2  P4-31 */
#define GPIO_FTM3_CH2IN  PIN_FTM3_CH2_2  /* PTE7  PWM3  P4-28 */
#define GPIO_FTM3_CH3IN  PIN_FTM3_CH3_2  /* PTE8  PWM4  P4-25 */
#define GPIO_FTM3_CH4IN  PIN_FTM3_CH4_2  /* PTE9  PWM5  P4-22 */
#define GPIO_FTM3_CH5IN  PIN_FTM3_CH5_2  /* PTE10 PWM6  P4-29 */
#define GPIO_FTM3_CH6IN  PIN_FTM3_CH6_2  /* PTE11 PWM7  P4-26 */
#define GPIO_FTM3_CH7IN  PIN_FTM3_CH7_2  /* PTE12 PWM8  P4-13 */

#define GPIO_FTM0_CH4IN  PIN_FTM0_CH4_3  /* PTD4  PWM0  P4-46 */
#define GPIO_FTM0_CH5IN  PIN_FTM0_CH5_3  /* PTD5  PWM10 P4-43 */
#define GPIO_FTM0_CH6IN  PIN_FTM0_CH6_2  /* PTD6  PWM11 P4-40 */
#define GPIO_FTM0_CH7IN  PIN_FTM0_CH7_2  /* PTD7  PWM12 P4-37 */

#define GPIO_FTM0_CH3IN  PIN_FTM0_CH3_1  /* PTA6  PWM13 P4-7 */
#define GPIO_FTM0_CH2IN  PIN_FTM0_CH2_2  /* PTC3  PWM14 P4-10 */

/* SPI
 *
 */

/* SPI0 SD Card */

#define PIN_SPI0_PCS0    PIN_SPI0_PCS0_2  /* PTC4 SPI_CS  SD1-2 */
#define PIN_SPI0_SCK     PIN_SPI0_SCK_2   /* PTC5 SPI_CLK SD1-5 */
#define PIN_SPI0_OUT     PIN_SPI0_SOUT_2  /* PTC6 SPI_OUT SD1-3 */
#define PIN_SPI0_SIN     PIN_SPI0_SIN_2   /* PTC7 SPI_IN  SD1-5 */

/* SPI1 FXOS8700CQ Accelerometer */

#define PIN_SPI1_PCS0    PIN_SPI1_PCS0_1  /* PTB10 A_CS   */
#define PIN_SPI1_SCK     PIN_SPI1_SCK_1   /* PTB11 A_SCLK */
#define PIN_SPI1_OUT     PIN_SPI1_SOUT_1  /* PTB16 A_MOSI */
#define PIN_SPI1_SIN     PIN_SPI1_SIN_1   /* PTB17 A_MISO */

/* SPI2 FXAS21002CQ Gyroscope */

#define PIN_SPI2_PCS0    PIN_SPI2_PCS0_1  /* PTB20 GM_CS   */
#define PIN_SPI2_SCK     PIN_SPI2_SCK_1   /* PTB21 GM_SCLK */
#define PIN_SPI2_OUT     PIN_SPI2_SOUT_1  /* PTB22 GM_MOSI */
#define PIN_SPI2_SIN     PIN_SPI2_SIN_1   /* PTB23 GM_MISO */

/* UART
 *
 * NuttX Will use UART4 as the Console
 */

#define PIN_UART0_RX      PIN_UART0_RX_4   /* PTD6 P4-40 PWM11 */
#define PIN_UART0_TX      PIN_UART0_TX_4   /* PTD7 P4-37 PWM12 */

#define PIN_UART1_RX      PIN_UART1_RX_2  /* PTE1 UART P14-3 */
#define PIN_UART1_TX      PIN_UART1_TX_2  /* PTE0 UART P14-2 */

/* No Alternative pins for UART2
 * PD2 BL P1-5
 * PD3 BL P1-4
 */

#define PIN_UART3_RX      PIN_UART3_RX_2 /* PTC16 GPS P3-3 */
#define PIN_UART3_TX      PIN_UART3_TX_2 /* PTC17 GPS P3-2 */

#define PIN_UART4_RX      PIN_UART4_RX_1  /* PTC14 UART P10-3 */
#define PIN_UART4_TX      PIN_UART4_TX_1  /* PTC15 UART P10-2 */

/* UART5 is not connected on V1
 */

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: kinetis_boardinitialize
 *
 * Description:
 *   All kinetis architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void kinetis_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_NXPHLITE_V23_INCLUDE_BOARD_H */
