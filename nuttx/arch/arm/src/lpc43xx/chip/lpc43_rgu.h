/****************************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_rgu.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_RGU_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_RGU_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Register Offsets *********************************************************************************/

#define LPC43_RGU_CTRL0_OFFSET           0x100  /* Reset control register 0 */
#define LPC43_RGU_CTRL1_OFFSET           0x104  /* Reset control register 1 */
#define LPC43_RGU_STATUS0_OFFSET         0x110  /* Reset status register 0 */
#define LPC43_RGU_STATUS1_OFFSET         0x114  /* Reset status register 1 */
#define LPC43_RGU_STATUS2_OFFSET         0x118  /* Reset status register 2 */
#define LPC43_RGU_STATUS3_OFFSET         0x11c  /* Reset status register 3 */
#define LPC43_RGU_ACTIVE0_OFFSET         0x150  /* Reset active status register */
#define LPC43_RGU_ACTIVE1_OFFSET         0x154  /* Reset active status register */

/* External Status Register Indices */

#define RGU_CORE_RST                     0
#define RGU_PERIPH_RST                   1
#define RGU_MASTER_RST                   2
#define RGU_WWDT_RST                     4
#define RGU_CREG_RST                     5
#define RGU_BUS_RST                      8
#define RGU_SCU_RST                      9
#define RGU_M4_RST                       13
#define RGU_LCD_RST                      16
#define RGU_USB0_RST                     17
#define RGU_USB1_RST                     18
#define RGU_DMA_RST                      19
#define RGU_SDIO_RST                     20
#define RGU_EMC_RST                      21
#define RGU_ETHERNET_RST                 22
#define RGU_FLASHA_RST                   25
#define RGU_EEPROM_RST                   27
#define RGU_GPIO_RST                     28
#define RGU_FLASHB_RST                   29
#define RGU_TIMER0_RST                   32
#define RGU_TIMER1_RST                   33
#define RGU_TIMER2_RST                   34
#define RGU_TIMER3_RST                   35
#define RGU_RITIMER_RST                  36
#define RGU_SCT_RST                      37
#define RGU_MCPWM_RST                    38
#define RGU_QEI_RST                      39
#define RGU_ADC0_RST                     40
#define RGU_ADC1_RST                     41
#define RGU_DAC_RST                      42
#define RGU_USART0_RST                   44
#define RGU_UART1_RST                    45
#define RGU_USART2_RST                   46
#define RGU_USART3_RST                   47
#define RGU_I2C0_RST                     48
#define RGU_I2C1_RST                     49
#define RGU_SSP0_RST                     50
#define RGU_SSP1_RST                     51
#define RGU_I2S_RST                      52
#define RGU_SPIFI_RST                    53
#define RGU_CAN1_RST                     54
#define RGU_CAN0_RST                     55
#define RGU_M0APP_RST                    56
#define RGU_SGPIO_RST                    57
#define RGU_SPI_RST                      58

/* External Status Registers */

#define LPC43_RGU_EXTSTAT_OFFSET(n)      (0x0400 + ((n) << 2))  /* Reset external status register n=0..63 */
#define LPC43_RGU_EXTSTAT0_OFFSET        0x400  /* Reset external status register 0 for CORE_RST */
#define LPC43_RGU_EXTSTAT1_OFFSET        0x404  /* Reset external status register 1 for PERIPH_RST */
#define LPC43_RGU_EXTSTAT2_OFFSET        0x408  /* Reset external status register 2 for MASTER_RST */
#define LPC43_RGU_EXTSTAT4_OFFSET        0x410  /* Reset external status register 4 for WWDT_RST */
#define LPC43_RGU_EXTSTAT5_OFFSET        0x414  /* Reset external status register 5 for CREG_RST */
#define LPC43_RGU_EXTSTAT8_OFFSET        0x420  /* Reset external status register 8 for BUS_RST */
#define LPC43_RGU_EXTSTAT9_OFFSET        0x424  /* Reset external status register 9 for SCU_RST */
#define LPC43_RGU_EXTSTAT13_OFFSET       0x434  /* Reset external status register 13 for M4_RST */
#define LPC43_RGU_EXTSTAT16_OFFSET       0x440  /* Reset external status register 16 for LCD_RST */
#define LPC43_RGU_EXTSTAT17_OFFSET       0x444  /* Reset external status register 17 for USB0_RST */
#define LPC43_RGU_EXTSTAT18_OFFSET       0x448  /* Reset external status register 18 for USB1_RST */
#define LPC43_RGU_EXTSTAT19_OFFSET       0x44c  /* Reset external status register 19 for DMA_RST */
#define LPC43_RGU_EXTSTAT20_OFFSET       0x450  /* Reset external status register 20 for SDIO_RST */
#define LPC43_RGU_EXTSTAT21_OFFSET       0x454  /* Reset external status register 21 for EMC_RST */
#define LPC43_RGU_EXTSTAT22_OFFSET       0x458  /* Reset external status register 22 for ETHERNET_RST */
#define LPC43_RGU_EXTSTAT25_OFFSET       0x464  /* Reset external status register 25 for FLASHA_RST */
#define LPC43_RGU_EXTSTAT27_OFFSET       0x46c  /* Reset external status register 27 for EEPROM_RST */
#define LPC43_RGU_EXTSTAT28_OFFSET       0x470  /* Reset external status register 28 for GPIO_RST */
#define LPC43_RGU_EXTSTAT29_OFFSET       0x474  /* Reset external status register 29 for FLASHB_RST */
#define LPC43_RGU_EXTSTAT32_OFFSET       0x480  /* Reset external status register 32 for TIMER0_RST */
#define LPC43_RGU_EXTSTAT33_OFFSET       0x484  /* Reset external status register 33 for TIMER1_RST */
#define LPC43_RGU_EXTSTAT34_OFFSET       0x488  /* Reset external status register 34 for TIMER2_RST */
#define LPC43_RGU_EXTSTAT35_OFFSET       0x48c  /* Reset external status register 35 for TIMER3_RST */
#define LPC43_RGU_EXTSTAT36_OFFSET       0x490  /* Reset external status register 36 for RITIMER_RST */
#define LPC43_RGU_EXTSTAT37_OFFSET       0x494  /* Reset external status register 37 for SCT_RST */
#define LPC43_RGU_EXTSTAT38_OFFSET       0x498  /* Reset external status register 38 for MCPWM_RST */
#define LPC43_RGU_EXTSTAT39_OFFSET       0x49c  /* Reset external status register 39 for QEI_RST */
#define LPC43_RGU_EXTSTAT40_OFFSET       0x4a0  /* Reset external status register 40 for ADC0_RST */
#define LPC43_RGU_EXTSTAT41_OFFSET       0x4a4  /* Reset external status register 41 for ADC1_RST */
#define LPC43_RGU_EXTSTAT42_OFFSET       0x4a8  /* Reset external status register 42 for DAC_RST */
#define LPC43_RGU_EXTSTAT44_OFFSET       0x4b0  /* Reset external status register 44 for USART0_RST */
#define LPC43_RGU_EXTSTAT45_OFFSET       0x4b4  /* Reset external status register 45 for UART1_RST */
#define LPC43_RGU_EXTSTAT46_OFFSET       0x4b8  /* Reset external status register 46 for USART2_RST */
#define LPC43_RGU_EXTSTAT47_OFFSET       0x4bc  /* Reset external status register 47 for USART3_RST */
#define LPC43_RGU_EXTSTAT48_OFFSET       0x4c0  /* Reset external status register 48 for I2C0_RST */
#define LPC43_RGU_EXTSTAT49_OFFSET       0x4c4  /* Reset external status register 49 for I2C1_RST */
#define LPC43_RGU_EXTSTAT50_OFFSET       0x4c8  /* Reset external status register 50 for SSP0_RST */
#define LPC43_RGU_EXTSTAT51_OFFSET       0x4cc  /* Reset external status register 51 for SSP1_RST */
#define LPC43_RGU_EXTSTAT52_OFFSET       0x4d0  /* Reset external status register 52 for I2S_RST */
#define LPC43_RGU_EXTSTAT53_OFFSET       0x4d4  /* Reset external status register 53 for SPIFI_RST */
#define LPC43_RGU_EXTSTAT54_OFFSET       0x4d8  /* Reset external status register 54 for CAN1_RST */
#define LPC43_RGU_EXTSTAT55_OFFSET       0x4dc  /* Reset external status register 55 for CAN0_RST */
#define LPC43_RGU_EXTSTAT56_OFFSET       0x4e0  /* Reset external status register 56 for M0APP_RST */
#define LPC43_RGU_EXTSTAT57_OFFSET       0x4e4  /* Reset external status register 57 for SGPIO_RST */
#define LPC43_RGU_EXTSTAT58_OFFSET       0x4e8  /* Reset external status register 58 for SPI_RST */

/* Register Addresses *******************************************************************************/

#define LPC43_RGU_CTRL0                  (LPC43_RGU_BASE+LPC43_RGU_CTRL0_OFFSET)
#define LPC43_RGU_CTRL1                  (LPC43_RGU_BASE+LPC43_RGU_CTRL1_OFFSET)
#define LPC43_RGU_STATUS0                (LPC43_RGU_BASE+LPC43_RGU_STATUS0_OFFSET)
#define LPC43_RGU_STATUS1                (LPC43_RGU_BASE+LPC43_RGU_STATUS1_OFFSET)
#define LPC43_RGU_STATUS2                (LPC43_RGU_BASE+LPC43_RGU_STATUS2_OFFSET)
#define LPC43_RGU_STATUS3                (LPC43_RGU_BASE+LPC43_RGU_STATUS3_OFFSET)
#define LPC43_RGU_ACTIVE0                (LPC43_RGU_BASE+LPC43_RGU_ACTIVE0_OFFSET)
#define LPC43_RGU_ACTIVE1                (LPC43_RGU_BASE+LPC43_RGU_ACTIVE1_OFFSET)

/* External Status Registers */

#define LPC43_RGU_EXTSTAT(n)             (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT_OFFSET(n))
#define LPC43_RGU_EXTSTAT0               (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT0_OFFSET)
#define LPC43_RGU_EXTSTAT1               (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT1_OFFSET)
#define LPC43_RGU_EXTSTAT2               (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT2_OFFSET)
#define LPC43_RGU_EXTSTAT4               (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT4_OFFSET)
#define LPC43_RGU_EXTSTAT5               (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT5_OFFSET)
#define LPC43_RGU_EXTSTAT8               (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT8_OFFSET)
#define LPC43_RGU_EXTSTAT9               (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT9_OFFSET)
#define LPC43_RGU_EXTSTAT13              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT13_OFFSET)
#define LPC43_RGU_EXTSTAT16              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT16_OFFSET)
#define LPC43_RGU_EXTSTAT17              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT17_OFFSET)
#define LPC43_RGU_EXTSTAT18              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT18_OFFSET)
#define LPC43_RGU_EXTSTAT19              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT19_OFFSET)
#define LPC43_RGU_EXTSTAT20              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT20_OFFSET)
#define LPC43_RGU_EXTSTAT21              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT21_OFFSET)
#define LPC43_RGU_EXTSTAT22              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT22_OFFSET)
#define LPC43_RGU_EXTSTAT25              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT25_OFFSET)
#define LPC43_RGU_EXTSTAT27              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT27_OFFSET)
#define LPC43_RGU_EXTSTAT28              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT28_OFFSET)
#define LPC43_RGU_EXTSTAT29              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT29_OFFSET)
#define LPC43_RGU_EXTSTAT32              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT32_OFFSET)
#define LPC43_RGU_EXTSTAT33              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT33_OFFSET)
#define LPC43_RGU_EXTSTAT34              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT34_OFFSET)
#define LPC43_RGU_EXTSTAT35              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT35_OFFSET)
#define LPC43_RGU_EXTSTAT36              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT36_OFFSET)
#define LPC43_RGU_EXTSTAT37              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT37_OFFSET)
#define LPC43_RGU_EXTSTAT38              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT38_OFFSET)
#define LPC43_RGU_EXTSTAT39              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT39_OFFSET)
#define LPC43_RGU_EXTSTAT40              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT40_OFFSET)
#define LPC43_RGU_EXTSTAT41              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT41_OFFSET)
#define LPC43_RGU_EXTSTAT42              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT42_OFFSET)
#define LPC43_RGU_EXTSTAT44              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT44_OFFSET)
#define LPC43_RGU_EXTSTAT45              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT45_OFFSET)
#define LPC43_RGU_EXTSTAT46              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT46_OFFSET)
#define LPC43_RGU_EXTSTAT47              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT47_OFFSET)
#define LPC43_RGU_EXTSTAT48              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT48_OFFSET)
#define LPC43_RGU_EXTSTAT49              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT49_OFFSET)
#define LPC43_RGU_EXTSTAT50              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT50_OFFSET)
#define LPC43_RGU_EXTSTAT51              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT51_OFFSET)
#define LPC43_RGU_EXTSTAT52              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT52_OFFSET)
#define LPC43_RGU_EXTSTAT53              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT53_OFFSET)
#define LPC43_RGU_EXTSTAT54              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT54_OFFSET)
#define LPC43_RGU_EXTSTAT55              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT55_OFFSET)
#define LPC43_RGU_EXTSTAT56              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT56_OFFSET)
#define LPC43_RGU_EXTSTAT57              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT57_OFFSET)
#define LPC43_RGU_EXTSTAT58              (LPC43_RGU_BASE+LPC43_RGU_EXTSTAT58_OFFSET)

/* Alternative naming */

#define LPC43_RGU_EXTSTAT_CORE_RST       LPC43_RGU_EXTSTAT0
#define LPC43_RGU_EXTSTAT_PERIPH_RST     LPC43_RGU_EXTSTAT1
#define LPC43_RGU_EXTSTAT_MASTER_RST     LPC43_RGU_EXTSTAT2
#define LPC43_RGU_EXTSTAT_WWDT_RST       LPC43_RGU_EXTSTAT4
#define LPC43_RGU_EXTSTAT_CREG_RST       LPC43_RGU_EXTSTAT5
#define LPC43_RGU_EXTSTAT_BUS_RST        LPC43_RGU_EXTSTAT8
#define LPC43_RGU_EXTSTAT_SCU_RST        LPC43_RGU_EXTSTAT9
#define LPC43_RGU_EXTSTAT_M4_RST         LPC43_RGU_EXTSTAT13
#define LPC43_RGU_EXTSTAT_LCD_RST        LPC43_RGU_EXTSTAT16
#define LPC43_RGU_EXTSTAT_USB0_RST       LPC43_RGU_EXTSTAT17
#define LPC43_RGU_EXTSTAT_USB1_RST       LPC43_RGU_EXTSTAT18
#define LPC43_RGU_EXTSTAT_DMA_RST        LPC43_RGU_EXTSTAT19
#define LPC43_RGU_EXTSTAT_SDIO_RST       LPC43_RGU_EXTSTAT20
#define LPC43_RGU_EXTSTAT_EMC_RST        LPC43_RGU_EXTSTAT21
#define LPC43_RGU_EXTSTAT_ETHERNET_RST   LPC43_RGU_EXTSTAT22
#define LPC43_RGU_EXTSTAT_FLASHA_RST     LPC43_RGU_EXTSTAT25
#define LPC43_RGU_EXTSTAT_EEPROM_RST     LPC43_RGU_EXTSTAT27
#define LPC43_RGU_EXTSTAT_GPIO_RST       LPC43_RGU_EXTSTAT28
#define LPC43_RGU_EXTSTAT_FLASHB_RST     LPC43_RGU_EXTSTAT29
#define LPC43_RGU_EXTSTAT_TIMER0_RST     LPC43_RGU_EXTSTAT32
#define LPC43_RGU_EXTSTAT_TIMER1_RST     LPC43_RGU_EXTSTAT33
#define LPC43_RGU_EXTSTAT_TIMER2_RST     LPC43_RGU_EXTSTAT34
#define LPC43_RGU_EXTSTAT_TIMER3_RST     LPC43_RGU_EXTSTAT35
#define LPC43_RGU_EXTSTAT_RITIMER_RST    LPC43_RGU_EXTSTAT36
#define LPC43_RGU_EXTSTAT_SCT_RST        LPC43_RGU_EXTSTAT37
#define LPC43_RGU_EXTSTAT_MCPWM_RST      LPC43_RGU_EXTSTAT38
#define LPC43_RGU_EXTSTAT_QEI_RST        LPC43_RGU_EXTSTAT39
#define LPC43_RGU_EXTSTAT_ADC0_RST       LPC43_RGU_EXTSTAT40
#define LPC43_RGU_EXTSTAT_ADC1_RST       LPC43_RGU_EXTSTAT41
#define LPC43_RGU_EXTSTAT_DAC_RST        LPC43_RGU_EXTSTAT42
#define LPC43_RGU_EXTSTAT_USART0_RST     LPC43_RGU_EXTSTAT44
#define LPC43_RGU_EXTSTAT_UART1_RST      LPC43_RGU_EXTSTAT45
#define LPC43_RGU_EXTSTAT_USART2_RST     LPC43_RGU_EXTSTAT46
#define LPC43_RGU_EXTSTAT_USART3_RST     LPC43_RGU_EXTSTAT47
#define LPC43_RGU_EXTSTAT_I2C0_RST       LPC43_RGU_EXTSTAT48
#define LPC43_RGU_EXTSTAT_I2C1_RST       LPC43_RGU_EXTSTAT49
#define LPC43_RGU_EXTSTAT_SSP0_RST       LPC43_RGU_EXTSTAT50
#define LPC43_RGU_EXTSTAT_SSP1_RST       LPC43_RGU_EXTSTAT51
#define LPC43_RGU_EXTSTAT_I2S_RST        LPC43_RGU_EXTSTAT52
#define LPC43_RGU_EXTSTAT_SPIFI_RST      LPC43_RGU_EXTSTAT53
#define LPC43_RGU_EXTSTAT_CAN1_RST       LPC43_RGU_EXTSTAT54
#define LPC43_RGU_EXTSTAT_CAN0_RST       LPC43_RGU_EXTSTAT55
#define LPC43_RGU_EXTSTAT_M0APP_RST      LPC43_RGU_EXTSTAT56
#define LPC43_RGU_EXTSTAT_SGPIO_RST      LPC43_RGU_EXTSTAT57
#define LPC43_RGU_EXTSTAT_SPI_RST        LPC43_RGU_EXTSTAT58

/* Register Bit Definitions *************************************************************************/

/* Reset control register 0 */

#define RGU_CTRL0_CORE_RST               (1 << 0)
#define RGU_CTRL0_PERIPH_RST             (1 << 1)
#define RGU_CTRL0_MASTER_RST             (1 << 2)
                                                   /* Bit 3:  Reserved */
#define RGU_CTRL0_WWDT_RST               (1 << 4)  /* Writing a one to this bit has no effect */
#define RGU_CTRL0_CREG_RST               (1 << 5)  /* Writing a one to this bit has no effect */
                                                   /* Bits 6-7:  Reserved */
#define RGU_CTRL0_BUS_RST                (1 << 8)
#define RGU_CTRL0_SCU_RST                (1 << 9)
                                                   /* Bits 10-12:  Reserved */
#define RGU_CTRL0_M4_RST                 (1 << 13)
                                                   /* Bits 14-15:  Reserved */
#define RGU_CTRL0_LCD_RST                (1 << 16)
#define RGU_CTRL0_USB0_RST               (1 << 17)
#define RGU_CTRL0_USB1_RST               (1 << 18)
#define RGU_CTRL0_DMA_RST                (1 << 19)
#define RGU_CTRL0_SDIO_RST               (1 << 20)
#define RGU_CTRL0_EMC_RST                (1 << 21)
#define RGU_CTRL0_ETHERNET_RST           (1 << 22)
                                                   /* Bits 23-24:  Reserved */
#define RGU_CTRL0_FLASHA_RST             (1 << 25)
                                                   /* Bit 26:  Reserved */
#define RGU_CTRL0_EEPROM_RST             (1 << 27)
#define RGU_CTRL0_GPIO_RST               (1 << 28)
#define RGU_CTRL0_FLASHB_RST             (1 << 29)
                                                   /* Bits 30-31:  Reserved */
/* Reset control register 1 */

#define RGU_CTRL1_TIMER0_RST             (1 << 0)
#define RGU_CTRL1_TIMER1_RST             (1 << 1)
#define RGU_CTRL1_TIMER2_RST             (1 << 2)
#define RGU_CTRL1_TIMER3_RST             (1 << 3)
#define RGU_CTRL1_RITIMER_RST            (1 << 4)
#define RGU_CTRL1_SCT_RST                (1 << 5)
#define RGU_CTRL1_MCPWM_RST              (1 << 6)
#define RGU_CTRL1_QEI_RST                (1 << 7)
#define RGU_CTRL1_ADC0_RST               (1 << 8)
#define RGU_CTRL1_ADC1_RST               (1 << 9)
#define RGU_CTRL1_DAC_RST                (1 << 10)
                                                    /* Bit 11:  Reserved */
#define RGU_CTRL1_USART0_RST             (1 << 12)
#define RGU_CTRL1_UART1_RST              (1 << 13)
#define RGU_CTRL1_USART2_RST             (1 << 14)
#define RGU_CTRL1_USART3_RST             (1 << 15)
#define RGU_CTRL1_I2C0_RST               (1 << 16)
#define RGU_CTRL1_I2C1_RST               (1 << 17)
#define RGU_CTRL1_SSP0_RST               (1 << 18)
#define RGU_CTRL1_SSP1_RST               (1 << 19)
#define RGU_CTRL1_I2S_RST                (1 << 20)
#define RGU_CTRL1_SPIFI_RST              (1 << 21)
#define RGU_CTRL1_CAN1_RST               (1 << 22)
#define RGU_CTRL1_CAN0_RST               (1 << 23)
#define RGU_CTRL1_M0APP_RST              (1 << 24)
#define RGU_CTRL1_SGPIO_RST              (1 << 25)
#define RGU_CTRL1_SPI_RST                (1 << 26)
                                                    /* Bits 27-31:  Reserved */
/* Reset status register 0 */

#define RGU_RST_NONE                     0          /* No reset activated */
#define RGU_RST_HW                       1          /* Reset output activated by input to the reset generator */
#define RGU_RST_SW                       3          /* Reset output activated by software write to RESET_CTRL register */

#define RGU_STATUS0_CORE_RST_SHIFT       (0)        /* Bits 0-1: Status of the CORE_RST reset generator output */
#define RGU_STATUS0_CORE_RST_MASK        (3 << RGU_STATUS0_CORE_RST_SHIFT)
#  define RGU_STATUS0_CORE_RST_NONE      (0 << RGU_STATUS0_CORE_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS0_CORE_RST_HW        (1 << RGU_STATUS0_CORE_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS0_CORE_RST_SW        (3 << RGU_STATUS0_CORE_RST_SHIFT) /* Activated by software */
#define RGU_STATUS0_PERIPH_RST_SHIFT     (2)        /* Bits 2-3: Status of the PERIPH_RST reset generator output */
#define RGU_STATUS0_PERIPH_RST_MASK      (3 << RGU_STATUS0_PERIPH_RST_SHIFT)
#  define RGU_STATUS0_PERIPH_RST_NONE    (0 << RGU_STATUS0_PERIPH_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS0_PERIPH_RST_HW      (1 << RGU_STATUS0_PERIPH_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS0_PERIPH_RST_SW      (3 << RGU_STATUS0_PERIPH_RST_SHIFT) /* Activated by software */
#define RGU_STATUS0_MASTER_RST_SHIFT     (4)        /* Bits 4-5: Status of the MASTER_RST reset generator output */
#define RGU_STATUS0_MASTER_RST_MASK      (3 << RGU_STATUS0_MASTER_RST_SHIFT)
#  define RGU_STATUS0_MASTER_RST_NONE    (0 << RGU_STATUS0_MASTER_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS0_MASTER_RST_HW      (1 << RGU_STATUS0_MASTER_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS0_MASTER_RST_SW      (3 << RGU_STATUS0_MASTER_RST_SHIFT) /* Activated by software */
                                                    /* Bits 6-7:  Reserved */
#define RGU_STATUS0_WWDT_RST_SHIFT       (8)        /* Bits 8-9: Status of the WWDT_RST reset generator output */
#define RGU_STATUS0_WWDT_RST_MASK        (3 << RGU_STATUS0_WWDT_RST_SHIFT)
#  define RGU_STATUS0_WWDT_RST_NONE      (0 << RGU_STATUS0_WWDT_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS0_WWDT_RST_HW        (1 << RGU_STATUS0_WWDT_RST_SHIFT) /* Activated by reset generator */
#define RGU_STATUS0_CREG_RST_SHIFT       (10)       /* Bits 10-11: Status of the CREG_RST reset generator output */
#define RGU_STATUS0_CREG_RST_MASK        (3 << RGU_STATUS0_CREG_RST_SHIFT)
#  define RGU_STATUS0_CREG_RST_NONE      (0 << RGU_STATUS0_CREG_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS0_CREG_RST_HW        (1 << RGU_STATUS0_CREG_RST_SHIFT) /* Activated by reset generator */
                                                    /* Bits 12-15:  Reserved */
#define RGU_STATUS0_BUS_RST_SHIFT        (16)       /* Bits 16-17: Status of the BUS_RST reset generator output */
#define RGU_STATUS0_BUS_RST_MASK         (3 << RGU_STATUS0_BUS_RST_SHIFT)
#  define RGU_STATUS0_BUS_RST_NONE       (0 << RGU_STATUS0_BUS_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS0_BUS_RST_HW         (1 << RGU_STATUS0_BUS_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS0_BUS_RST_SW         (3 << RGU_STATUS0_BUS_RST_SHIFT) /* Activated by software */
#define RGU_STATUS0_SCU_RST_SHIFT        (18)       /* Bits 18-19: Status of the SCU_RST reset generator output */
#define RGU_STATUS0_SCU_RST_MASK         (3 << RGU_STATUS0_SCU_RST_SHIFT)
#  define RGU_STATUS0_SCU_RST_NONE       (0 << RGU_STATUS0_SCU_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS0_SCU_RST_HW         (1 << RGU_STATUS0_SCU_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS0_SCU_RST_SW         (3 << RGU_STATUS0_SCU_RST_SHIFT) /* Activated by software */
                                                    /* Bits 20-25:  Reserved */
#define RGU_STATUS0_M4_RST_SHIFT         (26)       /* Bits 26-27: Status of the M4_RST reset generator output */
#define RGU_STATUS0_M4_RST_MASK          (3 << RGU_STATUS0_M4_RST_SHIFT)
#  define RGU_STATUS0_M4_RST_NONE        (0 << RGU_STATUS0_M4_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS0_M4_RST_HW          (1 << RGU_STATUS0_M4_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS0_M4_RST_SW          (3 << RGU_STATUS0_M4_RST_SHIFT) /* Activated by software */
                                                    /* Bits 29-31:  Reserved */
/* Reset status register 1 */

#define RGU_STATUS1_LCD_RST_SHIFT        (0)        /* Bits 0-1: Status of the LCD_RST reset generator output */
#define RGU_STATUS1_LCD_RST_MASK         (3 << RGU_STATUS1_LCD_RST_SHIFT)
#  define RGU_STATUS1_LCD_RST_NONE       (0 << RGU_STATUS1_LCD_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS1_LCD_RST_HW         (1 << RGU_STATUS1_LCD_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS1_LCD_RST_SW         (3 << RGU_STATUS1_LCD_RST_SHIFT) /* Activated by software */
#define RGU_STATUS1_USB0_RST_SHIFT       (2)        /* Bits 2-3: Status of the USB0_RST reset generator output */
#define RGU_STATUS1_USB0_RST_MASK        (3 << RGU_STATUS1_USB0_RST_SHIFT)
#  define RGU_STATUS1_USB0_RST_NONE      (0 << RGU_STATUS1_USB0_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS1_USB0_RST_HW        (1 << RGU_STATUS1_USB0_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS1_USB0_RST_SW        (3 << RGU_STATUS1_USB0_RST_SHIFT) /* Activated by software */
#define RGU_STATUS1_USB1_RST_SHIFT       (4)        /* Bits 4-5: Status of the USB1_RST reset generator output */
#define RGU_STATUS1_USB1_RST_MASK        (3 << RGU_STATUS1_USB1_RST_SHIFT)
#  define RGU_STATUS1_USB1_RST_NONE      (0 << RGU_STATUS1_USB1_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS1_USB1_RST_HW        (1 << RGU_STATUS1_USB1_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS1_USB1_RST_SW        (3 << RGU_STATUS1_USB1_RST_SHIFT) /* Activated by software */
#define RGU_STATUS1_DMA_RST_SHIFT        (6)        /* Bits 6-7: Status of the DMA_RST reset generator output */
#define RGU_STATUS1_DMA_RST_MASK         (3 << RGU_STATUS1_DMA_RST_SHIFT)
#  define RGU_STATUS1_DMA_RST_NONE       (0 << RGU_STATUS1_DMA_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS1_DMA_RST_HW         (1 << RGU_STATUS1_DMA_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS1_DMA_RST_SW         (3 << RGU_STATUS1_DMA_RST_SHIFT) /* Activated by software */
#define RGU_STATUS1_SDIO_RST_SHIFT       (8)        /* Bits 8-9: Status of the SDIO_RST reset generator output */
#define RGU_STATUS1_SDIO_RST_MASK        (3 << RGU_STATUS1_SDIO_RST_SHIFT)
#  define RGU_STATUS1_SDIO_RST_NONE      (0 << RGU_STATUS1_SDIO_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS1_SDIO_RST_HW        (1 << RGU_STATUS1_SDIO_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS1_SDIO_RST_SW        (3 << RGU_STATUS1_SDIO_RST_SHIFT) /* Activated by software */
#define RGU_STATUS1_EMC_RST_SHIFT        (10)       /* Bits 10-11: Status of the EMC_RST reset generator output */
#define RGU_STATUS1_EMC_RST_MASK         (3 << RGU_STATUS1_EMC_RST_SHIFT)
#  define RGU_STATUS1_EMC_RST_NONE       (0 << RGU_STATUS1_EMC_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS1_EMC_RST_HW         (1 << RGU_STATUS1_EMC_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS1_EMC_RST_SW         (3 << RGU_STATUS1_EMC_RST_SHIFT) /* Activated by software */
#define RGU_STATUS1_ETHERNET_RST_SHIFT   (12)       /* Bits 12-13: Status of the ETHERNET_RST reset generator output */
#define RGU_STATUS1_ETHERNET_RST_MASK    (3 << RGU_STATUS1_ETHERNET_RST_SHIFT)
#  define RGU_STATUS1_ETHERNET_RST_NONE  (0 << RGU_STATUS1_ETHERNET_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS1_ETHERNET_RST_HW    (1 << RGU_STATUS1_ETHERNET_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS1_ETHERNET_RST_SW    (3 << RGU_STATUS1_ETHERNET_RST_SHIFT) /* Activated by software */
#define RGU_STATUS1_FLASHA_RST_SHIFT     (18)       /* Bits 18-19: Status of the FLASHA_RST reset generator output */
#define RGU_STATUS1_FLASHA_RST_MASK      (3 << RGU_STATUS1_FLASHA_RST_SHIFT)
#  define RGU_STATUS1_FLASHA_RST_NONE    (0 << RGU_STATUS1_FLASHA_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS1_FLASHA_RST_HW      (1 << RGU_STATUS1_FLASHA_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS1_FLASHA_RST_SW      (3 << RGU_STATUS1_FLASHA_RST_SHIFT) /* Activated by software */
#define RGU_STATUS1_EEPROM_RST_SHIFT     (22)       /* Bits 22-23: Status of the EEPROM_RST reset generator output */
#define RGU_STATUS1_EEPROM_RST_MASK      (3 << RGU_STATUS1_EEPROM_RST_SHIFT)
#  define RGU_STATUS1_EEPROM_RST_NONE    (0 << RGU_STATUS1_EEPROM_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS1_EEPROM_RST_HW      (1 << RGU_STATUS1_EEPROM_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS1_EEPROM_RST_SW      (3 << RGU_STATUS1_EEPROM_RST_SHIFT) /* Activated by software */
#define RGU_STATUS1_GPIO_RST_SHIFT       (24)       /* Bits 24-25: Status of the GPIO_RST reset generator output */
#define RGU_STATUS1_GPIO_RST_MASK        (3 << RGU_STATUS1_GPIO_RST_SHIFT)
#  define RGU_STATUS1_GPIO_RST_NONE      (0 << RGU_STATUS1_GPIO_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS1_GPIO_RST_HW        (1 << RGU_STATUS1_GPIO_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS1_GPIO_RST_SW        (3 << RGU_STATUS1_GPIO_RST_SHIFT) /* Activated by software */
#define RGU_STATUS1_FLASHB_RST_SHIFT     (26)       /* Bits 26-27: Status of the FLASHB_RST reset generator output */
#define RGU_STATUS1_FLASHB_RST_MASK      (3 << RGU_STATUS1_FLASHB_RST_SHIFT)
#  define RGU_STATUS1_FLASHB_RST_NONE    (0 << RGU_STATUS1_FLASHB_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS1_FLASHB_RST_HW      (1 << RGU_STATUS1_FLASHB_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS1_FLASHB_RST_SW      (3 << RGU_STATUS1_FLASHB_RST_SHIFT) /* Activated by software */
                                                    /* Bits 28-31:  Reserved */
/* Reset status register 2 */

#define RGU_STATUS2_TIMER0_RST_SHIFT     (0)        /* Bits 0-1: 1:0  Status of the TIMER0_RST reset generator output */
#define RGU_STATUS2_TIMER0_RST_MASK      (3 << RGU_STATUS2_TIMER0_RST_SHIFT)
#  define RGU_STATUS2_TIMER0_RST_NONE    (0 << RGU_STATUS2_TIMER0_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_TIMER0_RST_HW      (1 << RGU_STATUS2_TIMER0_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_TIMER0_RST_SW      (3 << RGU_STATUS2_TIMER0_RST_SHIFT) /* Activated by software */
#define RGU_STATUS2_TIMER1_RST_SHIFT     (2)        /* Bits 2-3: 3:2  Status of the TIMER1_RST reset generator output */
#define RGU_STATUS2_TIMER1_RST_MASK      (3 << RGU_STATUS2_TIMER1_RST_SHIFT)
#  define RGU_STATUS2_TIMER1_RST_NONE    (0 << RGU_STATUS2_TIMER1_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_TIMER1_RST_HW      (1 << RGU_STATUS2_TIMER1_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_TIMER1_RST_SW      (3 << RGU_STATUS2_TIMER1_RST_SHIFT) /* Activated by software */
#define RGU_STATUS2_TIMER2_RST_SHIFT     (4)        /* Bits 4-5: 5:4  Status of the TIMER2_RST reset generator output */
#define RGU_STATUS2_TIMER2_RST_MASK      (3 << RGU_STATUS2_TIMER2_RST_SHIFT)
#  define RGU_STATUS2_TIMER2_RST_NONE    (0 << RGU_STATUS2_TIMER2_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_TIMER2_RST_HW      (1 << RGU_STATUS2_TIMER2_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_TIMER2_RST_SW      (3 << RGU_STATUS2_TIMER2_RST_SHIFT) /* Activated by software */
#define RGU_STATUS2_TIMER3_RST_SHIFT     (6)        /* Bits 6-7: 7:6  Status of the TIMER3_RST reset generator output */
#define RGU_STATUS2_TIMER3_RST_MASK      (3 << RGU_STATUS2_TIMER3_RST_SHIFT)
#  define RGU_STATUS2_TIMER3_RST_NONE    (0 << RGU_STATUS2_TIMER3_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_TIMER3_RST_HW      (1 << RGU_STATUS2_TIMER3_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_TIMER3_RST_SW      (3 << RGU_STATUS2_TIMER3_RST_SHIFT) /* Activated by software */
#define RGU_STATUS2_RITIMER_RST_SHIFT    (8)        /* Bits 8-9: 9:8  Status of the RITIMER_RST reset generator output */
#define RGU_STATUS2_RITIMER_RST_MASK     (3 << RGU_STATUS2_RITIMER_RST_SHIFT)
#  define RGU_STATUS2_RITIMER_RST_NONE   (0 << RGU_STATUS2_RITIMER_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_RITIMER_RST_HW     (1 << RGU_STATUS2_RITIMER_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_RITIMER_RST_SW     (3 << RGU_STATUS2_RITIMER_RST_SHIFT) /* Activated by software */
#define RGU_STATUS2_SCT_RST_SHIFT        (10)       /* Bits 10-11: 11:10  Status of the SCT_RST reset generator output */
#define RGU_STATUS2_SCT_RST_MASK         (3 << RGU_STATUS2_SCT_RST_SHIFT)
#  define RGU_STATUS2_SCT_RST_NONE       (0 << RGU_STATUS2_SCT_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_SCT_RST_HW         (1 << RGU_STATUS2_SCT_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_SCT_RST_SW         (3 << RGU_STATUS2_SCT_RST_SHIFT) /* Activated by software */
#define RGU_STATUS2_MCPWM_RST_SHIFT      (12)       /* Bits 12-13: 13:12  Status of the MOTOCONPWM_RST reset generator output */
#define RGU_STATUS2_MCPWM_RST_MASK       (3 << RGU_STATUS2_MCPWM_RST_SHIFT)
#  define RGU_STATUS2_MCPWM_RST_NONE     (0 << RGU_STATUS2_MCPWM_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_MCPWM_RST_HW       (1 << RGU_STATUS2_MCPWM_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_MCPWM_RST_SW       (3 << RGU_STATUS2_MCPWM_RST_SHIFT) /* Activated by software */
#define RGU_STATUS2_QEI_RST_SHIFT        (14)       /* Bits 14-15: 15:14  Status of the QEI_RST reset generator output */
#define RGU_STATUS2_QEI_RST_MASK         (3 << RGU_STATUS2_QEI_RST_SHIFT)
#  define RGU_STATUS2_QEI_RST_NONE       (0 << RGU_STATUS2_QEI_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_QEI_RST_HW         (1 << RGU_STATUS2_QEI_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_QEI_RST_SW         (3 << RGU_STATUS2_QEI_RST_SHIFT) /* Activated by software */
#define RGU_STATUS2_ADC0_RST_SHIFT       (16)       /* Bits 16-17: 17:16  Status of the ADC0_RST reset generator output */
#define RGU_STATUS2_ADC0_RST_MASK        (3 << RGU_STATUS2_ADC0_RST_SHIFT)
#  define RGU_STATUS2_ADC0_RST_NONE      (0 << RGU_STATUS2_ADC0_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_ADC0_RST_HW        (1 << RGU_STATUS2_ADC0_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_ADC0_RST_SW        (3 << RGU_STATUS2_ADC0_RST_SHIFT) /* Activated by software */
#define RGU_STATUS2_ADC1_RST_SHIFT       (18)       /* Bits 18-19: 19:18  Status of the ADC1_RST reset generator output */
#define RGU_STATUS2_ADC1_RST_MASK        (3 << RGU_STATUS2_ADC1_RST_SHIFT)
#  define RGU_STATUS2_ADC1_RST_NONE      (0 << RGU_STATUS2_ADC1_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_ADC1_RST_HW        (1 << RGU_STATUS2_ADC1_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_ADC1_RST_SW        (3 << RGU_STATUS2_ADC1_RST_SHIFT) /* Activated by software */
#define RGU_STATUS2_DAC_RST_SHIFT        (20)       /* Bits 20-21: 21:20  Status of the DAC_RST reset generator output */
#define RGU_STATUS2_DAC_RST_MASK         (3 << RGU_STATUS2_DAC_RST_SHIFT)
#  define RGU_STATUS2_DAC_RST_NONE       (0 << RGU_STATUS2_DAC_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_DAC_RST_HW         (1 << RGU_STATUS2_DAC_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_DAC_RST_SW         (3 << RGU_STATUS2_DAC_RST_SHIFT) /* Activated by software */
                                                   /* Bits 22-23:  Reserved */
#define RGU_STATUS2_USART0_RST_SHIFT     (24)       /* Bits 24-24: 25:24  Status of the USART0_RST reset generator output */
#define RGU_STATUS2_USART0_RST_MASK      (3 << RGU_STATUS2_USART0_RST_SHIFT)
#  define RGU_STATUS2_USART0_RST_NONE    (0 << RGU_STATUS2_USART0_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_USART0_RST_HW      (1 << RGU_STATUS2_USART0_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_USART0_RST_SW      (3 << RGU_STATUS2_USART0_RST_SHIFT) /* Activated by software */
#define RGU_STATUS2_UART1_RST_SHIFT      (26)       /* Bits 26-27: 27:26  Status of the UART1_RST reset generator output */
#define RGU_STATUS2_UART1_RST_MASK       (3 << RGU_STATUS2_UART1_RST_SHIFT)
#  define RGU_STATUS2_UART1_RST_NONE     (0 << RGU_STATUS2_UART1_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_UART1_RST_HW       (1 << RGU_STATUS2_UART1_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_UART1_RST_SW       (3 << RGU_STATUS2_UART1_RST_SHIFT) /* Activated by software */
#define RGU_STATUS2_USART2_RST_SHIFT     (28)       /* Bits 28-29: 29:28  Status of the USART2_RST reset generator output */
#define RGU_STATUS2_USART2_RST_MASK      (3 << RGU_STATUS2_USART2_RST_SHIFT)
#  define RGU_STATUS2_USART2_RST_NONE    (0 << RGU_STATUS2_USART2_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_USART2_RST_HW      (1 << RGU_STATUS2_USART2_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_USART2_RST_SW      (3 << RGU_STATUS2_USART2_RST_SHIFT) /* Activated by software */
#define RGU_STATUS2_USART3_RST_SHIFT     (30)       /* Bits 30-31: 31:30  Status of the USART3_RST reset generator output */
#define RGU_STATUS2_USART3_RST_MASK      (3 << RGU_STATUS2_USART3_RST_SHIFT)
#  define RGU_STATUS2_USART3_RST_NONE    (0 << RGU_STATUS2_USART3_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS2_USART3_RST_HW      (1 << RGU_STATUS2_USART3_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS2_USART3_RST_SW      (3 << RGU_STATUS2_USART3_RST_SHIFT) /* Activated by software */

/* Reset status register 3 */

#define RGU_STATUS3_I2C0_RST_SHIFT       (0)        /* Bits 0-1: 1:0  Status of the I2C0_RST reset generator output */
#define RGU_STATUS3_I2C0_RST_MASK        (3 << RGU_STATUS3_I2C0_RST_SHIFT)
#  define RGU_STATUS3_I2C0_RST_NONE      (0 << RGU_STATUS3_I2C0_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS3_I2C0_RST_HW        (1 << RGU_STATUS3_I2C0_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS3_I2C0_RST_SW        (3 << RGU_STATUS3_I2C0_RST_SHIFT) /* Activated by software */
#define RGU_STATUS3_I2C1_RST_SHIFT       (2)        /* Bits 2-3: 3:2  Status of the I2C1_RST reset generator output */
#define RGU_STATUS3_I2C1_RST_MASK        (3 << RGU_STATUS3_I2C1_RST_SHIFT)
#  define RGU_STATUS3_I2C1_RST_NONE      (0 << RGU_STATUS3_I2C1_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS3_I2C1_RST_HW        (1 << RGU_STATUS3_I2C1_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS3_I2C1_RST_SW        (3 << RGU_STATUS3_I2C1_RST_SHIFT) /* Activated by software */
#define RGU_STATUS3_SSP0_RST_SHIFT       (4)        /* Bits 4-5: 5:4  Status of the SSP0_RST reset generator output */
#define RGU_STATUS3_SSP0_RST_MASK        (3 << RGU_STATUS3_SSP0_RST_SHIFT)
#  define RGU_STATUS3_SSP0_RST_NONE      (0 << RGU_STATUS3_SSP0_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS3_SSP0_RST_HW        (1 << RGU_STATUS3_SSP0_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS3_SSP0_RST_SW        (3 << RGU_STATUS3_SSP0_RST_SHIFT) /* Activated by software */
#define RGU_STATUS3_SSP1_RST_SHIFT       (6)        /* Bits 6-7: 7:6  Status of the SSP1_RST reset generator output */
#define RGU_STATUS3_SSP1_RST_MASK        (3 << RGU_STATUS3_SSP1_RST_SHIFT)
#  define RGU_STATUS3_SSP1_RST_NONE      (0 << RGU_STATUS3_SSP1_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS3_SSP1_RST_HW        (1 << RGU_STATUS3_SSP1_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS3_SSP1_RST_SW        (3 << RGU_STATUS3_SSP1_RST_SHIFT) /* Activated by software */
#define RGU_STATUS3_I2S_RST_SHIFT        (8)        /* Bits 8-9: 9:8  Status of the I2S_RST reset generator output */
#define RGU_STATUS3_I2S_RST_MASK         (3 << RGU_STATUS3_I2S_RST_SHIFT)
#  define RGU_STATUS3_I2S_RST_NONE       (0 << RGU_STATUS3_I2S_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS3_I2S_RST_HW         (1 << RGU_STATUS3_I2S_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS3_I2S_RST_SW         (3 << RGU_STATUS3_I2S_RST_SHIFT) /* Activated by software */
#define RGU_STATUS3_SPIFI_RST_SHIFT      (10)       /* Bits 10-11: 11:10  Status of the SPIFI_RST reset generator output */
#define RGU_STATUS3_SPIFI_RST_MASK       (3 << RGU_STATUS3_SPIFI_RST_SHIFT)
#  define RGU_STATUS3_SPIFI_RST_NONE     (0 << RGU_STATUS3_SPIFI_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS3_SPIFI_RST_HW       (1 << RGU_STATUS3_SPIFI_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS3_SPIFI_RST_SW       (3 << RGU_STATUS3_SPIFI_RST_SHIFT) /* Activated by software */
#define RGU_STATUS3_CAN1_RST_SHIFT       (12)       /* Bits 12-13: 13:12  Status of the CAN1_RST reset generator output */
#define RGU_STATUS3_CAN1_RST_MASK        (3 << RGU_STATUS3_CAN1_RST_SHIFT)
#  define RGU_STATUS3_CAN1_RST_NONE      (0 << RGU_STATUS3_CAN1_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS3_CAN1_RST_HW        (1 << RGU_STATUS3_CAN1_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS3_CAN1_RST_SW        (3 << RGU_STATUS3_CAN1_RST_SHIFT) /* Activated by software */
#define RGU_STATUS3_CAN0_RST_SHIFT       (14)       /* Bits 14-15: 15:14  Status of the CAN0_RST reset generator output */
#define RGU_STATUS3_CAN0_RST_MASK        (3 << RGU_STATUS3_CAN0_RST_SHIFT)
#  define RGU_STATUS3_CAN0_RST_NONE      (0 << RGU_STATUS3_CAN0_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS3_CAN0_RST_HW        (1 << RGU_STATUS3_CAN0_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS3_CAN0_RST_SW        (3 << RGU_STATUS3_CAN0_RST_SHIFT) /* Activated by software */
#define RGU_STATUS3_M0APP_RST_SHIFT      (16)       /* Bits 16-17: 17:16  Status of the M0APP_RST reset generator output */
#define RGU_STATUS3_M0APP_RST_MASK       (3 << RGU_STATUS3_M0APP_RST_SHIFT)
#  define RGU_STATUS3_M0APP_RST_NONE     (0 << RGU_STATUS3_M0APP_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS3_M0APP_RST_HW       (1 << RGU_STATUS3_M0APP_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS3_M0APP_RST_SW       (3 << RGU_STATUS3_M0APP_RST_SHIFT) /* Activated by software */
#define RGU_STATUS3_SGPIO_RST_SHIFT      (18)       /* Bits 18-19: 19:18  Status of the SGPIO_RST reset generator output */
#define RGU_STATUS3_SGPIO_RST_MASK       (3 << RGU_STATUS3_SGPIO_RST_SHIFT)
#  define RGU_STATUS3_SGPIO_RST_NONE     (0 << RGU_STATUS3_SGPIO_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS3_SGPIO_RST_HW       (1 << RGU_STATUS3_SGPIO_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS3_SGPIO_RST_SW       (3 << RGU_STATUS3_SGPIO_RST_SHIFT) /* Activated by software */
#define RGU_STATUS3_SPI_RST_SHIFT        (20)       /* Bits 20-21: 21:20  Status of the SPI_RST reset generator output */
#define RGU_STATUS3_SPI_RST_MASK         (3 << RGU_STATUS3_SPI_RST_SHIFT)
#  define RGU_STATUS3_SPI_RST_NONE       (0 << RGU_STATUS3_SPI_RST_SHIFT) /* No reset activated */
#  define RGU_STATUS3_SPI_RST_HW         (1 << RGU_STATUS3_SPI_RST_SHIFT) /* Activated by reset generator */
#  define RGU_STATUS3_SPI_RST_SW         (3 << RGU_STATUS3_SPI_RST_SHIFT) /* Activated by software */
                                                    /* Bits 22-31:  Reserved */
/* Reset active status register */

#define RGU_ACTIVE0_CORE_RST             (1 << 0)
#define RGU_ACTIVE0_PERIPH_RST           (1 << 1)
#define RGU_ACTIVE0_MASTER_RST           (1 << 2)
                                                   /* Bit 3:  Reserved */
#define RGU_ACTIVE0_WWDT_RST             (1 << 4)
#define RGU_ACTIVE0_CREG_RST             (1 << 5)
                                                   /* Bits 6-7:  Reserved */
#define RGU_ACTIVE0_BUS_RST              (1 << 8)
#define RGU_ACTIVE0_SCU_RST              (1 << 9)
                                                   /* Bits 10-12:  Reserved */
#define RGU_ACTIVE0_M4_RST               (1 << 13)
                                                   /* Bits 14-15:  Reserved */
#define RGU_ACTIVE0_LCD_RST              (1 << 16)
#define RGU_ACTIVE0_USB0_RST             (1 << 17)
#define RGU_ACTIVE0_USB1_RST             (1 << 18)
#define RGU_ACTIVE0_DMA_RST              (1 << 19)
#define RGU_ACTIVE0_SDIO_RST             (1 << 20)
#define RGU_ACTIVE0_EMC_RST              (1 << 21)
#define RGU_ACTIVE0_ETHERNET_RST         (1 << 22)
                                                   /* Bits 23-24:  Reserved */
#define RGU_ACTIVE0_FLASHA_RST           (1 << 25)
                                                   /* Bit 26:  Reserved */
#define RGU_ACTIVE0_EEPROM_RST           (1 << 27)
#define RGU_ACTIVE0_GPIO_RST             (1 << 28)
#define RGU_ACTIVE0_FLASHB_RST           (1 << 29)
                                                   /* Bits 30-31:  Reserved */
/* Reset active status register */

#define RGU_ACTIVE1_TIMER0_RST           (1 << 0)
#define RGU_ACTIVE1_TIMER1_RST           (1 << 1)
#define RGU_ACTIVE1_TIMER2_RST           (1 << 2)
#define RGU_ACTIVE1_TIMER3_RST           (1 << 3)
#define RGU_ACTIVE1_RITIMER_RST          (1 << 4)
#define RGU_ACTIVE1_SCT_RST              (1 << 5)
#define RGU_ACTIVE1_MCPWM_RST            (1 << 6)
#define RGU_ACTIVE1_QEI_RST              (1 << 7)
#define RGU_ACTIVE1_ADC0_RST             (1 << 8)
#define RGU_ACTIVE1_ADC1_RST             (1 << 9)
#define RGU_ACTIVE1_DAC_RST              (1 << 10)
                                                    /* Bit 11:  Reserved */
#define RGU_ACTIVE1_USART0_RST           (1 << 12)
#define RGU_ACTIVE1_UART1_RST            (1 << 13)
#define RGU_ACTIVE1_USART2_RST           (1 << 14)
#define RGU_ACTIVE1_USART3_RST           (1 << 15)
#define RGU_ACTIVE1_I2C0_RST             (1 << 16)
#define RGU_ACTIVE1_I2C1_RST             (1 << 17)
#define RGU_ACTIVE1_SSP0_RST             (1 << 18)
#define RGU_ACTIVE1_SSP1_RST             (1 << 19)
#define RGU_ACTIVE1_I2S_RST              (1 << 20)
#define RGU_ACTIVE1_SPIFI_RST            (1 << 21)
#define RGU_ACTIVE1_CAN1_RST             (1 << 22)
#define RGU_ACTIVE1_CAN0_RST             (1 << 23)
#define RGU_ACTIVE1_M0APP_RST            (1 << 24)
#define RGU_ACTIVE1_SGPIO_RST            (1 << 25)
#define RGU_ACTIVE1_SPI_RST              (1 << 26)
                                                    /* Bits 27-31:  Reserved */
/* Reset external status register 0 for CORE_RST */

#define RGU_EXTSTAT_CORE_EXTRESET        (1 << 0)   /* Bit 0: Reset activated by external reset from reset pin */
                                                    /* Bits 1-3: Reserved */
#define RGU_EXTSTAT_CORE_BODRESET        (1 << 4)   /* Bit 4: Reset activated by BOD reset */
#define RGU_EXTSTAT_CORE_WWDTRESET       (1 << 5)   /* Bit 5: Reset activated by WWDT time-out */
                                                    /* Bits 6-31: Reserved */
/* Reset external status register 1 for PERIPH_RST */
                                                    /* Bit 0: Reserved */
#define RGU_EXTSTAT_PERIPH_CORERESET     (1 << 1)   /* Bit 1: Reset activated by CORE_RST output */
                                                    /* Bits 2-31: Reserved */
/* Reset external status register 2 for MASTER_RST */
                                                    /* Bits 0-1: Reserved */
#define RGU_EXTSTAT_MASTER_PERIPHRESET   (1 << 2)   /* Bit 2: Reset activated by PERIPHERAL_RST output */
                                                    /* Bits 2-31: Reserved */
/* Reset external status register 4 for WWDT_RST */
                                                    /* Bit 0: Reserved */
#define RGU_EXTSTAT_WWDT_CORERESET       (1 << 1)   /* Bit 1: Reset activated by CORE_RST output */
                                                    /* Bits 2-31: Reserved */
/* Reset external status register 5 for CREG_RST */
                                                    /* Bit 0: Reserved */
#define RGU_EXTSTAT_CREG_CORERESET       (1 << 1)   /* Bit 1: Reset activated by CORE_RST output */
                                                    /* Bits 2-31: Reserved */
/* Reset external status registers for PERIPHERAL_RESET */
                                                    /* Bits 0-1: Reserved */
#define RGU_EXTSTAT_PERIPH_RESET         (1 << 2)   /* Bit 2: Reset activated by PERIPHERAL_RST output */
                                                    /* Bits 2-31: Reserved */
/* Reset external status registers for MASTER_RESET */
                                                    /* Bits 0-2: Reserved */
#define RGU_EXTSTAT_MASTER_RESET         (1 << 3)   /* Bit 3: Reset activated by MASTER_RST output */
                                                    /* Bits 2-31: Reserved */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_RGU_H */
