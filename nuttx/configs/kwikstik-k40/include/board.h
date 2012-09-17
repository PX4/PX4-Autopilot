/************************************************************************************
 * configs/kwikstik-k40/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The Kwikstik-K40 has a 4MHz crystal on board */

#undef  BOARD_EXTCLOCK                      /* Crystal */
#define BOARD_EXTAL_FREQ     4000000        /* 4MHz crystal frequency (REFCLK) */
#define BOARD_XTAL32_FREQ    32768          /* 32KHz RTC Oscillator */

/* PLL Configuration.  NOTE: Only even frequency crystals are supported that will
 * produce a 2MHz reference clock to the PLL.
 *
 *   PLL Input frequency:   PLLIN  = REFCLK/PRDIV = 4MHz/2 = 2MHz
 *   PLL Output frequency:  PLLOUT = PLLIN*VDIV   = 2Mhz*48 = 96MHz
 *   MCG Frequency:         PLLOUT = 96MHz
 */

#define BOARD_PRDIV          2              /* PLL External Reference Divider */
#define BOARD_VDIV           48             /* PLL VCO Divider (frequency multiplier) */

#define BOARD_PLLIN_FREQ     (BOARD_EXTAL_FREQ / BOARD_PRDIV)
#define BOARD_PLLOUT_FREQ    (BOARD_PLLIN_FREQ * BOARD_VDIV)
#define BOARD_MCG_FREQ       BOARD_PLLOUT_FREQ

/* SIM CLKDIV1 dividers */

#define BOARD_OUTDIV1        1              /* Core        = MCG, 96MHz */
#define BOARD_OUTDIV2        2              /* Bus         = MCG/2, 48MHz */
#define BOARD_OUTDIV3        2              /* FlexBus     = MCG/2, 48MHz */
#define BOARD_OUTDIV4        4              /* Flash clock = MCG/4, 24MHz */

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
 * clock.
 */

/* Identification mode:  400KHz = 96MHz / ( 16 * 15) */

#define BOARD_SDHC_IDMODE_PRESCALER   SDHC_SYSCTL_SDCLKFS_DIV16
#define BOARD_SDHC_IDMODE_DIVISOR     SDHC_SYSCTL_DVS_DIV(15)

/* MMC normal mode: 16MHz  = 96MHz / (2 * 3) */

#define BOARD_SDHC_MMCMODE_PRESCALER  SDHC_SYSCTL_SDCLKFS_DIV2
#define BOARD_SDHC_MMCMODE_DIVISOR    SDHC_SYSCTL_DVS_DIV(3)

/* SD normal mode (1-bit): 16MHz  = 96MHz / (2 * 3) */

#define BOARD_SDHC_SD1MODE_PRESCALER  SDHC_SYSCTL_SDCLKFS_DIV2
#define BOARD_SDHC_SD1MODE_DIVISOR    SDHC_SYSCTL_DVS_DIV(3)

/* SD normal mode (4-bit): 24MHz  = 96MHz / (2 * 2) (with DMA)
 * SD normal mode (4-bit): 16MHz  = 96MHz / (2 * 3) (no DMA)
 */

#ifdef CONFIG_SDIO_DMA
#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV2
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(2)
#else
//#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV2
//#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(3)
#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV16
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(15)
#endif

/* LED definitions ******************************************************************/
/* The KwikStik-K40 board has no MCU driven, GPIO-based LEDs */

#define LED_STARTED       0
#define LED_HEAPALLOCATE  1
#define LED_IRQSENABLED   2
#define LED_STACKCREATED  3
#define LED_INIRQ         4
#define LED_SIGNAL        5
#define LED_ASSERTION     6
#define LED_PANIC         7

/* Button definitions ***************************************************************/
/* The KwikStik-K40 board has no standard GPIO contact buttons */

/* Alternative pin resolution *******************************************************/
/* If there are alternative configurations for various pins in the
 * kinetis_k40pinmux.h header file, those alternative pins will be labeled with a
 * suffix like _1, _2, etc.  The logic in this file must select the correct pin
 * configuration for the board by defining a pin configuration (with no suffix) that
 * maps to the correct alternative.
 */

/* On-Board Connections
 *
 * ------------------- -------------------------- -------- -------------------
 * FEATURE             CONNECTION                 PORT/PIN PIN FUNCTION
 * ------------------- -------------------------- -------- -------------------
 * Audio Jack Output   Audio Amp On               PTE28    PTE28
 *                     Audio Output               DAC1_OUT DAC1_OUT
 *                     Volume Up                  PTD10    PTD10
 *                     Volume Down                PTD11    PTD11
 * Buzzer              Audio Out                  PTA8     FTM1_CH0
 * Microphone          Microphone input           PTA7     ADC0_SE10
 * SD Card Slot        SD Clock                   PTE2     SDHC0_DCLK
 *                     SD Command                 PTE3     SDHC0_CMD
 *                     SD Data0                   PTD12    SDHC0_D4
 *                     SD Data1                   PTD13    SDHC0_D5
 *                     SD Data2                   PTD14    SDHC0_D6
 *                     SD Data3                   PTD15    SDHC0_D7
 *                     SD Card Detect             PTE27    PTE27
 *                     SD Card On                 PTE6     PTE6
 * Infrared Port       IR Transmit                PTE4     IR_TX
 *                     IR Receive                 PTA13    CMP2_IN0
 * Touch Pads          E1 / Touch                 PTB0     TSI0_CH0
 *                     E2 / Touch                 PTA4     TSI0_CH5
 *                     E3 / Touch                 PTA24    PTA24
 *                     E4 / Touch                 PTA25    PTA25
 *                     E5 / Touch                 PTA26    PTA26
 *                     E6 / Touch                 PTA27    PTA27
 */

#define PIN_FTM1_CH0   PIN_FTM1_CH0_1

/* Connections via the General Purpose Tower Plug-in (TWRPI) Socket
 * ------------------- -------------------------- -------- -------------------
 * FEATURE             CONNECTION                 PORT/PIN PIN FUNCTION
 * ------------------- -------------------------- -------- -------------------
 * General Purpose     TWRPI AN0 (J8 Pin 8)       ?        ADC0_DP0/ADC1_DP3
 * TWRPI Socket        TWRPI AN1 (J8 Pin 9)       ?        ADC0_DM0/ADC1_DM3
 *                     TWRPI AN2 (J8 Pin 12)      ?        ADC1_DP0/ADC0_DP3
 *                     TWRPI ID0 (J8 Pin 17)      ?        ADC0_DP1
 *                     TWRPI ID1 (J8 Pin 18)      ?        ADC0_DM1
 *                     TWRPI I2C SCL (J9 Pin 3)   PTC10    I2C1_SCL
 *                     TWRPI I2C SDA (J9 Pin 4)   PTC11    I2C1_SDA
 *                     TWRPI SPI MISO (J9 Pin 9)  PTB23    SPI2_SIN
 *                     TWRPI SPI MOSI (J9 Pin 10) PTB22    SPI2_SOUT
 *                     TWRPI SPI SS (J9 Pin 11)   PTB20    SPI2_PCS0
 *                     TWRPI SPI CLK (J9 Pin 12)  PTB21    SPI2_SCK
 *                     TWRPI GPIO0 (J9 Pin 15)    PTC12    PTC12
 *                     TWRPI GPIO1 (J9 Pin 16)    PTB9     PTB9
 *                     TWRPI GPIO2 (J9 Pin 17)    PTB10    PTB10
 *                     TWRPI GPIO3 (J9 Pin 18)    PTC5     PTC5
 *                     TWRPI GPIO4 (J9 Pin 19)    PTA5     PTA5
 */

#define PIN_I2C1_SCL   PIN_I2C1_SCL_1
#define PIN_I2C1_SDA   PIN_I2C1_SDA_1
#define PIN_SPI2_SIN   PIN_SPI2_SIN_1
#define PIN_SPI2_SOUT  PIN_SPI2_SOUT_1
#define PIN_SPI2_PCS0  PIN_SPI2_PCS0_1
#define PIN_SPI2_SCK   PIN_SPI2_SCK_1

/* Connections via the Tower Primary Connector Side A
 * --- -------------------- --------------------------------
 * PIN NAME                 USAGE
 * --- -------------------- --------------------------------
 * A9  GPIO9 / CTS1         PTE10/UART_CTS
 * A43 RXD1                 PTE9/UART_RX
 * A44 TXD1                 PTE8/UART_TX
 * A63 RSTOUT_b             PTA9/FTM1_CH1
 */

#define PIN_UART5_CTS   PIN_UART5_CTS_2
#define PIN_FTM1_CH1    PIN_FTM1_CH1_1

/* Connections via the Tower Primary Connector Side B
 * --- -------------------- --------------------------------
 * PIN NAME                 USAGE
 * --- -------------------- --------------------------------
 * B21 GPIO1 / RTS1         PTE7/UART_RTS
 * B37 PWM7                 PTA8/FTM1_CH0
 * B38 PWM6                 PTA9/FTM1_CH1
 * B41 CANRX0               PTE25/CAN1_RX
 * B42 CANTX0               PTE24/CAN1_TX
 * B44 SPI0_MISO            PTA17/SPI0_SIN
 * B45 SPI0_MOSI            PTA16/SPI0_SOUT
 * B46 SPI0_CS0_b           PTA14/SPI0_PCS0
 * B48 SPI0_CLK             PTA15/SPI0_SCK
 * B50 SCL1                 PTE1/I2C1_SCL
 * B51 SDA1                 PTE0/I2C1_SDA
 * B52 GPIO5 / SD_CARD_DET  PTA16
 */

#define PIN_UART3_RTS   PIN_UART3_RTS_3
#define PIN_CAN1_RX     PIN_CAN1_RX_2
#define PIN_CAN1_TX     PIN_CAN1_TX_2
#define PIN_SPI0_SIN    PIN_SPI0_SIN_1
#define PIN_SPI0_SOUT   PIN_SPI0_SOUT_1
#define PIN_SPI0_SCK    PIN_SPI0_SCK_1
#define PIN_SPI0_PCS0   PIN_SPI0_PCS0_1
#define PIN_I2C1_SCL    PIN_I2C1_SCL_2
#define PIN_I2C1_SDA    PIN_I2C1_SDA_2

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
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
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void kinetis_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
