/************************************************************************************
 * configs/twr-k60n512/src/twrk60-internal.h
 * arch/arm/src/board/twrk60-internal.h
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

#ifndef __CONFIGS_TWR_K60N512_SRC_TWRK60_INTERNAL_H
#define __CONFIGS_TWR_K60N512_SRC_TWRK60_INTERNAL_H

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

/* TWR-K60N512 GPIOs ****************************************************************/
/* On-Board Connections
 * -------------------- ------------------------- -------- -------------------
 * FEATURE              CONNECTION                PORT/PIN PIN FUNCTION
 * -------------------- ------------------------- -------- -------------------
 * OSJTAG USB-to-serial OSJTAG Bridge RX Data     PTE9     UART5_RX
 * Bridge               OSJTAG Bridge TX Data     PTE8     UART5_TX
 * SD Card Slot         SD Clock                  PTE2     SDHC0_DCLK
 *                      SD Command                PTE3     SDHC0_CMD
 *                      SD Data0                  PTE1     SDHC0_D0
 *                      SD Data1                  PTE0     SDHC0_D1
 *                      SD Data2                  PTE5     SDHC0_D2
 *                      SD Data3                  PTE4     SDHC0_D3
 *                      SD Card Detect            PTE28    PTE28
 *                      SD Write Protect          PTE27    PTE27
 * Infrared Port        IR Transmit               PTD7     CMT_IRO
 *                      IR Receive                PTC6     CMP0_IN0
 * Pushbuttons          SW1 (IRQ0)                PTA19    PTA19
 *                      SW2 (IRQ1)                PTE26    PTE26
 *                      SW3 (RESET)               RESET_b  RESET_b
 * Touch Pads           E1 / Touch                PTA4     TSI0_CH5
 *                      E2 / Touch                PTB3     TSI0_CH8
 *                      E3 / Touch                PTB2     TSI0_CH7
 *                      E4 / Touch                PTB16    TSI0_CH9
 * LEDs                 E1 / Orange LED           PTA11    PTA11
 *                      E2 / Yellow LED           PTA28    PTA28
 *                      E3 / Green LED            PTA29    PTA29
 *                      E4 / Blue LED             PTA10    PTA10
 * Potentiometer        Potentiometer (R71)       ?        ADC1_DM1
 * Accelerometer        I2C SDA                   PTD9     I2C0_SDA
 *                      I2C SCL                   PTD8     I2C0_SCL
 *                      IRQ                       PTD10    PTD10
 * Touch Pad / Segment  Electrode 0 (J3 Pin 3)    PTB0     TSI0_CH0
 * LCD TWRPI Socket     Electrode 1 (J3 Pin 5)    PTB1     TSI0_CH6
 *                      Electrode 2 (J3 Pin 7)    PTB2     TSI0_CH7
 *                      Electrode 3 (J3 Pin 8)    PTB3     TSI0_CH8
 *                      Electrode 4 (J3 Pin 9)    PTC0     TSI0_CH13
 *                      Electrode 5 (J3 Pin 10)   PTC1     TSI0_CH14
 *                      Electrode 6 (J3 Pin 11)   PTC2     TSI0_CH15
 *                      Electrode 7 (J3 Pin 12)   PTA4     TSI0_CH5
 *                      Electrode 8 (J3 Pin 13)   PTB16    TSI0_CH9
 *                      Electrode 9 (J3 Pin 14)   PTB17    TSI0_CH10
 *                      Electrode 10 (J3 Pin 15)  PTB18    TSI0_CH11
 *                      Electrode 11 (J3 Pin 16)  PTB19    TSI0_CH12
 *                      TWRPI ID0 (J3 Pin 17)     ?        ADC1_DP1
 *                      TWRPI ID1 (J3 Pin 18)     ?        ADC1_SE16
 */
 
#define GPIO_SD_CARDDETECT (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTE | PIN28)
#define GPIO_SD_WRPROTECT  (GPIO_PULLUP | PIN_PORTE | PIN27)

#define GPIO_SW1           (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTA | PIN19)
#define GPIO_SW2           (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTE | PIN26)
 
#define GPIO_LED1          (GPIO_LOWDRIVE | GPIO_OUTPUT_ZER0 | PIN_PORTA | PIN11)
#define GPIO_LED2          (GPIO_LOWDRIVE | GPIO_OUTPUT_ZER0 | PIN_PORTA | PIN28)
#define GPIO_LED3          (GPIO_LOWDRIVE | GPIO_OUTPUT_ZER0 | PIN_PORTA | PIN29)
#define GPIO_LED4          (GPIO_LOWDRIVE | GPIO_OUTPUT_ZER0 | PIN_PORTA | PIN10)

/* Connections via the General Purpose Tower Plug-in (TWRPI) Socket
 * -------------------- ------------------------- -------- -------------------
 * FEATURE             CONNECTION                 PORT/PIN PIN FUNCTION
 * -------------------- ------------------------- -------- -------------------
 * General Purpose      TWRPI AN0 (J4 Pin 8)       ?        ADC0_DP0/ADC1_DP3
 * TWRPI Socket         TWRPI AN1 (J4 Pin 9)       ?        ADC0_DM0/ADC1_DM3
 *                      TWRPI AN2 (J4 Pin 12)      ?        ADC1_DP0/ADC0_DP3
 *                      TWRPI ID0 (J4 Pin 17)      ?        ADC0_DP1
 *                      TWRPI ID1 (J4 Pin 18)      ?        ADC0_DM1
 *                      TWRPI I2C SCL (J5 Pin 3)   PTD8     I2C0_SCL
 *                      TWRPI I2C SDA (J5 Pin 4)   PTD9     I2C0_SDA
 *                      TWRPI SPI MISO (J5 Pin 9)  PTD14    SPI2_SIN
 *                      TWRPI SPI MOSI (J5 Pin 10) PTD13    SPI2_SOUT
 *                      TWRPI SPI SS (J5 Pin 11)   PTD15    SPI2_PCS0
 *                      TWRPI SPI CLK (J5 Pin 12)  PTD12    SPI2_SCK
 *                      TWRPI GPIO0 (J5 Pin 15)    PTD10    PTD10
 *                      TWRPI GPIO1 (J5 Pin 16)    PTB8     PTB8
 *                      TWRPI GPIO2 (J5 Pin 17)    PTB9     PTB9
 *                      TWRPI GPIO3 (J5 Pin 18)    PTA19    PTA19
 *                      TWRPI GPIO4 (J5 Pin 19)    PTE26    PTE26
 */

/* Connections via the Tower Primary Connector Side A
 * --- -------------------- --------------------------------
 * PIN NAME                 USAGE
 * --- -------------------- --------------------------------
 * A7  SCL0                 PTD8
 * A8  SDA0                 PTD9
 * A9  GPIO9 / CTS1         PTC19
 * A10 GPIO8 / SDHC_D2      PTE5
 * A11 GPIO7 / SD_WP_DET    PTE27
 * A13 ETH_MDC              PTB1
 * A14 ETH_MDIO             PTB0
 * A16 ETH_RXDV             PTA14
 * A19 ETH_RXD1             PTA12
 * A20 ETH_RXD0             PTA13
 * A21 SSI_MCLK             PTE6
 * A22 SSI_BCLK             PTE12
 * A23 SSI_FS               PTE11
 * A24 SSI_RXD              PTE7
 * A25 SSI_TXD              PTE10
 * A27 AN3                  PGA0_DP/ADC0_DP0/ADC1_DP3
 * A28 AN2                  PGA0_DM/ADC0_DM0/ADC1_DM3
 * A29 AN1                  PGA1_DP/ADC1_DP0/ADC0_DP3
 * A30 AN0                  PGA1_DM/ADC1_DM0/ADC0_DM3
 * A33 TMR1                 PTA9
 * A34 TMR0                 PTA8
 * A35 GPIO6                PTB9
 * A37 PWM3                 PTA6
 * A38 PWM2                 PTC3
 * A39 PWM1                 PTC2
 * A40 PWM0                 PTC1
 * A41 RXD0                 PTE25
 * A42 TXD0                 PTE24
 * A43 RXD1                 PTC16
 * A44 TXD1                 PTC17
 * A64 CLKOUT0              PTC3
 * A66 EBI_AD14             PTC0
 * A67 EBI_AD13             PTC1
 * A68 EBI_AD12             PTC2
 * A69 EBI_AD11             PTC4
 * A70 EBI_AD10             PTC5
 * A71 EBI_AD9              PTC6
 * A71 EBI_R/W_b            PTC11
 * A72 EBI_AD8              PTC7
 * A73 EBI_AD7              PTC8
 * A74 EBI_AD6              PTC9
 * A75 EBI_AD5              PTC10
 * A76 EBI_AD4              PTD2
 * A77 EBI_AD3              PTD3
 * A78 EBI_AD2              PTD4
 * A79 EBI_AD1              PTD5
 * A80 EBI_AD0              PTD6
 */

/* Connections via the Tower Primary Connector Side B
 * --- -------------------- --------------------------------
 * PIN NAME                 USAGE
 * --- -------------------- --------------------------------
 * B7  SDHC_CLK / SPI1_CLK  PTE2
 * B9  SDHC_D3 / SPI1_CS0_b PTE4
 * B10 SDHC_CMD / SPI1_MOSI PTE1
 * B11 SDHC_D0 / SPI1_MISO  PTE3
 * B13 ETH_RXER             PTA5
 * B15 ETH_TXEN             PTA15
 * B19 ETH_TXD1             PTA17
 * B20 ETH_TXD0             PTA16
 * B21 GPIO1 / RTS1         PTC18
 * B22 GPIO2 / SDHC_D1      PTE0
 * B23 GPIO3                PTE28
 * B24 CLKIN0               PTA18
 * B25 CLKOUT1              PTE26
 * B27 AN7                  PTB7
 * B28 AN6                  PTB6
 * B29 AN5                  PTB5
 * B30 AN4                  PTB4
 * B34 TMR2                 PTD6
 * B35 GPIO4                PTB8
 * B37 PWM7                 PTA2
 * B38 PWM6                 PTA1
 * B39 PWM5                 PTD5
 * B40 PWM4                 PTA7
 * B41 CANRX0               PTE25
 * B42 CANTX0               PTE24
 * B44 SPI0_MISO            PTD14
 * B45 SPI0_MOSI            PTD13
 * B46 SPI0_CS0_b           PTD11
 * B47 SPI0_CS1_b           PTD15
 * B48 SPI0_CLK             PTD12
 * B50 SCL1                 PTD8
 * B51 SDA1                 PTD9
 * B52 GPIO5 / SD_CARD_DET  PTE28
 * B55 IRQ_H                PTA24
 * B56 IRQ_G                PTA24
 * B57 IRQ_F                PTA25
 * B58 IRQ_E                PTA25
 * B59 IRQ_D                PTA26
 * B60 IRQ_C                PTA26
 * B61 IRQ_B                PTA27
 * B62 IRQ_A                PTA27
 * B63 EBI_ALE / EBI_CS1_b  PTD0
 * B64 EBI_CS0_b            PTD1
 * B66 EBI_AD15             PTB18
 * B67 EBI_AD16             PTB17
 * B68 EBI_AD17             PTB16
 * B69 EBI_AD18             PTB11
 * B70 EBI_AD19             PTB10
 * B72 EBI_OE_b             PTB19
 * B73 EBI_D7               PTB20
 * B74 EBI_D6               PTB21
 * B75 EBI_D5               PTB22
 * B76 EBI_D4               PTB23
 * B77 EBI_D3               PTC12
 * B78 EBI_D2               PTC13
 * B79 EBI_D1               PTC14
 * B80 EBI_D0               PTC15
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
 *   Called to configure SPI chip select GPIO pins for the TWR-K60N512 board.
 *
 ************************************************************************************/

extern void weak_function kinetis_spiinitialize(void);

/************************************************************************************
 * Name: kinetis_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the TWR-K60N512 board.
 *
 ************************************************************************************/

extern void weak_function kinetis_usbinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_TWR_K60N512_SRC_TWRK60_INTERNAL_H */

