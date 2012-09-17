/************************************************************************************
 * configs/kwikstik-k40/src/kwikstik-internal.h
 * arch/arm/src/board/kwikstik-internal.h
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

#ifndef __CONFIGS_KWIKSTK_K40_SRC_KWIKSTIK_INTERNAL_H
#define __CONFIGS_KWIKSTK_K40_SRC_KWIKSTIK_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* How many SPI modules does this chip support? The LM3S6918 supports 2 SPI
 * modules (others may support more -- in such case, the following must be
 * expanded).
 */

#if KINETIS_NSPI < 1
#  undef CONFIG_KINETIS_SPI1
#  undef CONFIG_KINETIS_SPI2
#elif KINETIS_NSPI < 2
#  undef CONFIG_KINETIS_SPI2
#endif

/* KwikStik-K40 GPIOs ***************************************************************/
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

#define GPIO_SD_CARDDETECT (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTE | PIN27)
#define GPIO_SD_CARDON     (GPIO_HIGHDRIVE | GPIO_OUTPUT_ZER0 | PIN_PORTE | PIN6)

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

/* Connections via the Tower Primary Connector Side A
 * --- -------------------- --------------------------------
 * PIN NAME                 USAGE
 * --- -------------------- --------------------------------
 * A9  GPIO9 / CTS1         PTE10/UART_CTS
 * A43 RXD1                 PTE9/UART_RX
 * A44 TXD1                 PTE8/UART_TX
 * A63 RSTOUT_b             PTA9/FTM1_CH1
 */

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

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_ledinit
 *
 * Description:
 *   Initialize LED GPIOs so that LEDs can be controlled.
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
extern void up_ledinit(void);
#endif

/************************************************************************************
 * Name: kinetis_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the KwikStik-K40 board.
 *
 ************************************************************************************/

extern void weak_function kinetis_spiinitialize(void);

/************************************************************************************
 * Name: kinetis_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the KwikStik-K40 board.
 *
 ************************************************************************************/

extern void weak_function kinetis_usbinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_KWIKSTK_K40_SRC_KWIKSTIK_INTERNAL_H */

