/****************************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_ccu.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_CCU_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_CCU_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Register Offsets *********************************************************************************/

#define LPC43_CCU1_PM_OFFSET                0x0000 /* CCU1 power mode register */
#define LPC43_CCU1_BASE_STAT_OFFSET         0x0004 /* CCU1 base clock status register */
#define LPC43_CCU1_APB3_BUS_CFG_OFFSET      0x0100  /* CLK_APB3_BUS clock configuration register */
#define LPC43_CCU1_APB3_BUS_STAT_OFFSET     0x0104  /* CLK_APB3_BUS clock status register */
#define LPC43_CCU1_APB3_I2C1_CFG_OFFSET     0x0108  /* CLK_APB3_I2C1 configuration register */
#define LPC43_CCU1_APB3_I2C1_STAT_OFFSET    0x010c  /* CLK_APB3_I2C1 status register */
#define LPC43_CCU1_APB3_DAC_CFG_OFFSET      0x0110  /* CLK_APB3_DAC configuration register */
#define LPC43_CCU1_APB3_DAC_STAT_OFFSET     0x0114  /* CLK_APB3_DAC status register */
#define LPC43_CCU1_APB3_ADC0_CFG_OFFSET     0x0118  /* CLK_APB3_ADC0 configuration register */
#define LPC43_CCU1_APB3_ADC0_STAT_OFFSET    0x011c  /* CLK_APB3_ADC0 status register */
#define LPC43_CCU1_APB3_ADC1_CFG_OFFSET     0x0120  /* CLK_APB3_ADC1 configuration register */
#define LPC43_CCU1_APB3_ADC1_STAT_OFFSET    0x0124  /* CLK_APB3_ADC1 status register */
#define LPC43_CCU1_APB3_CAN0_CFG_OFFSET     0x0128  /* CLK_APB3_CAN0 configuration register */
#define LPC43_CCU1_APB3_CAN0_STAT_OFFSET    0x012c  /* CLK_APB3_CAN0 status register */
#define LPC43_CCU1_APB1_BUS_CFG_OFFSET      0x0200  /* CLK_APB1_BUS configuration register */
#define LPC43_CCU1_APB1_BUS_STAT_OFFSET     0x0204  /* CLK_APB1_BUS status register */
#define LPC43_CCU1_APB1_MCPWM_CFG_OFFSET    0x0208  /* CLK_APB1_MOTOCON configuration register */
#define LPC43_CCU1_APB1_MCPWM_STAT_OFFSET   0x020c  /* CLK_APB1_MOTOCON status register */
#define LPC43_CCU1_APB1_I2C0_CFG_OFFSET     0x0210  /* CLK_APB1_I2C0 configuration register */
#define LPC43_CCU1_APB1_I2C0_STAT_OFFSET    0x0214  /* CLK_APB1_I2C0 status register */
#define LPC43_CCU1_APB1_I2S_CFG_OFFSET      0x0218  /* CLK_APB1_I2S configuration register */
#define LPC43_CCU1_APB1_I2S_STAT_OFFSET     0x021c  /* CLK_APB1_I2S status register */
#define LPC43_CCU1_APB1_CAN1_CFG_OFFSET     0x0220  /* CLK_APB3_CAN1 configuration register */
#define LPC43_CCU1_APB1_CAN1_STAT_OFFSET    0x0224  /* CLK_APB3_CAN1 status register */
#define LPC43_CCU1_SPIFI_CFG_OFFSET         0x0300  /* CLK_SPIFI configuration register */
#define LPC43_CCU1_SPIFI_STAT_OFFSET        0x0304  /* CLK_SPIFI status register */
#define LPC43_CCU1_M4_BUS_CFG_OFFSET        0x0400  /* CLK_M4_BUS configuration register */
#define LPC43_CCU1_M4_BUS_STAT_OFFSET       0x0404  /* CLK_M4_BUS status register */
#define LPC43_CCU1_M4_SPIFI_CFG_OFFSET      0x0408  /* CLK_M4_SPIFI configuration register */
#define LPC43_CCU1_M4_SPIFI_STAT_OFFSET     0x040c  /* CLK_M4_SPIFI status register */
#define LPC43_CCU1_M4_GPIO_CFG_OFFSET       0x0410  /* CLK_M4_GPIO configuration register */
#define LPC43_CCU1_M4_GPIO_STAT_OFFSET      0x0414  /* CLK_M4_GPIO status register */
#define LPC43_CCU1_M4_LCD_CFG_OFFSET        0x0418  /* CLK_M4_LCD configuration register */
#define LPC43_CCU1_M4_LCD_STAT_OFFSET       0x041c  /* CLK_M4_LCD status register */
#define LPC43_CCU1_M4_ETHERNET_CFG_OFFSET   0x0420  /* CLK_M4_ETHERNET configuration register */
#define LPC43_CCU1_M4_ETHERNET_STAT_OFFSET  0x0424  /* CLK_M4_ETHERNET status register */
#define LPC43_CCU1_M4_USB0_CFG_OFFSET       0x0428  /* CLK_M4_USB0 configuration register */
#define LPC43_CCU1_M4_USB0_STAT_OFFSET      0x042c  /* CLK_M4_USB0 status register */
#define LPC43_CCU1_M4_EMC_CFG_OFFSET        0x0430  /* CLK_M4_EMC configuration register */
#define LPC43_CCU1_M4_EMC_STAT_OFFSET       0x0434  /* CLK_M4_EMC status register */
#define LPC43_CCU1_M4_SDIO_CFG_OFFSET       0x0438  /* CLK_M4_SDIO configuration register */
#define LPC43_CCU1_M4_SDIO_STAT_OFFSET      0x043c  /* CLK_M4_SDIO status register */
#define LPC43_CCU1_M4_DMA_CFG_OFFSET        0x0440  /* CLK_M4_DMA configuration register */
#define LPC43_CCU1_M4_DMA_STAT_OFFSET       0x0444  /* CLK_M4_DMA status register */
#define LPC43_CCU1_M4_M4CORE_CFG_OFFSET     0x0448  /* CLK_M4_M4CORE configuration register */
#define LPC43_CCU1_M4_M4CORE_STAT_OFFSET    0x044c  /* CLK_M4_M4CORE status register */
#define LPC43_CCU1_M4_SCT_CFG_OFFSET        0x0468  /* CLK_M4_SCT configuration register */
#define LPC43_CCU1_M4_SCT_STAT_OFFSET       0x046c  /* CLK_M4_SCT status register */
#define LPC43_CCU1_M4_USB1_CFG_OFFSET       0x0470  /* CLK_M4_USB1 configuration register */
#define LPC43_CCU1_M4_USB1_STAT_OFFSET      0x0474  /* CLK_M4_USB1 status register */
#define LPC43_CCU1_M4_EMCDIV_CFG_OFFSET     0x0478  /* CLK_M4_EMCDIV configuration register */
#define LPC43_CCU1_M4_EMCDIV_STAT_OFFSET    0x047c  /* CLK_M4_EMCDIV status register */
#define LPC43_CCU1_M4_FLASHA_CFG_OFFSET     0x0480  /* CLK_M4_FLASHA configuration register */
#define LPC43_CCU1_M4_FLASHA_STAT_OFFSET    0x0484  /* CLK_M4_FLASHA status register */
#define LPC43_CCU1_M4_FLASHB_CFG_OFFSET     0x0488  /* CLK_M4_FLASHB configuration register */
#define LPC43_CCU1_M4_FLASHB_STAT_OFFSET    0x048c  /* CLK_M4_FLASHB status register */
#define LPC43_CCU1_M4_M0APP_CFG_OFFSET      0x0490  /* CLK_M4_M0_CFG configuration register */
#define LPC43_CCU1_M4_M0APP_STAT_OFFSET     0x0494  /* CLK_M4_M0_STAT status register */
#define LPC43_CCU1_M4_VADC_CFG_OFFSET       0x0498  /* CLK_M4_VADC_CFG configuration register */
#define LPC43_CCU1_M4_VADC_STAT_OFFSET      0x049c  /* CLK_M4_VADC_STAT configuration register */
#define LPC43_CCU1_M4_EEPROM_CFG_OFFSET     0x04a0  /* CLK_M4_EEPROM configuration register */
#define LPC43_CCU1_M4_EEPROM_STAT_OFFSET    0x04a4  /* CLK_M4_EEPROM status register */
#define LPC43_CCU1_M4_WWDT_CFG_OFFSET       0x0500  /* CLK_M4_WWDT configuration register */
#define LPC43_CCU1_M4_WWDT_STAT_OFFSET      0x0504  /* CLK_M4_WWDT status register */
#define LPC43_CCU1_M4_USART0_CFG_OFFSET     0x0508  /* CLK_M4_USART0 configuration register */
#define LPC43_CCU1_M4_USART0_STAT_OFFSET    0x050c  /* CLK_M4_USART0 status register */
#define LPC43_CCU1_M4_UART1_CFG_OFFSET      0x0510  /* CLK_M4_UART1 configuration register */
#define LPC43_CCU1_M4_UART1_STAT_OFFSET     0x0514  /* CLK_M4_UART1 status register */
#define LPC43_CCU1_M4_SSP0_CFG_OFFSET       0x0518  /* CLK_M4_SSP0 configuration register */
#define LPC43_CCU1_M4_SSP0_STAT_OFFSET      0x051c  /* CLK_M4_SSP0 status register */
#define LPC43_CCU1_M4_TIMER0_CFG_OFFSET     0x0520  /* CLK_M4_TIMER0 configuration register */
#define LPC43_CCU1_M4_TIMER0_STAT_OFFSET    0x0524  /* CLK_M4_TIMER0 status register */
#define LPC43_CCU1_M4_TIMER1_CFG_OFFSET     0x0528  /* CLK_M4_TIMER1 configuration register */
#define LPC43_CCU1_M4_TIMER1_STAT_OFFSET    0x052c  /* CLK_M4_TIMER1 status register */
#define LPC43_CCU1_M4_SCU_CFG_OFFSET        0x0530  /* CLK_M4_SCU configuration register */
#define LPC43_CCU1_M4_SCU_STAT_OFFSET       0x0534  /* CLK_M4_SCU status register */
#define LPC43_CCU1_M4_CREG_CFG_OFFSET       0x0538  /* CLK_M4_CREG configuration register */
#define LPC43_CCU1_M4_CREG_STAT_OFFSET      0x053c  /* CLK_M4_CREG status register */
#define LPC43_CCU1_M4_RITIMER_CFG_OFFSET    0x0600  /* CLK_M4_RITIMER configuration register */
#define LPC43_CCU1_M4_RITIMER_STAT_OFFSET   0x0604  /* CLK_M4_RITIMER status register */
#define LPC43_CCU1_M4_USART2_CFG_OFFSET     0x0608  /* CLK_M4_USART2 configuration register */
#define LPC43_CCU1_M4_USART2_STAT_OFFSET    0x060c  /* CLK_M4_USART2 status register */
#define LPC43_CCU1_M4_USART3_CFG_OFFSET     0x0610  /* CLK_M4_USART3 configuration register */
#define LPC43_CCU1_M4_USART3_STAT_OFFSET    0x0614  /* CLK_M4_USART3 status register */
#define LPC43_CCU1_M4_TIMER2_CFG_OFFSET     0x0618  /* CLK_M4_TIMER2 configuration register */
#define LPC43_CCU1_M4_TIMER2_STAT_OFFSET    0x061c  /* CLK_M4_TIMER2 status register */
#define LPC43_CCU1_M4_TIMER3_CFG_OFFSET     0x0620  /* CLK_M4_TIMER3 configuration register */
#define LPC43_CCU1_M4_TIMER3_STAT_OFFSET    0x0624  /* CLK_M4_TIMER3 status register */
#define LPC43_CCU1_M4_SSP1_CFG_OFFSET       0x0628  /* CLK_M4_SSP1 configuration register */
#define LPC43_CCU1_M4_SSP1_STAT_OFFSET      0x062c  /* CLK_M4_SSP1 status register */
#define LPC43_CCU1_M4_QEI_CFG_OFFSET        0x0630  /* CLK_M4_QEI configuration register */
#define LPC43_CCU1_M4_QEI_STAT_OFFSET       0x0634  /* CLK_M4_QEI status register */
#define LPC43_CCU1_PERIPH_BUS_CFG_OFFSET    0x0700  /* CLK_PERIPH_BUS configuration register */
#define LPC43_CCU1_PERIPH_BUS_STAT_OFFSET   0x0704  /* CLK_PERIPH_BUS status register */
#define LPC43_CCU1_PERIPH_CORE_CFG_OFFSET   0x0710  /* CLK_PERIPH_CORE configuration register */
#define LPC43_CCU1_PERIPH_CORE_STAT_OFFSET  0x0714  /* CLK_PERIPH_CORE status register */
#define LPC43_CCU1_PERIPH_SGPIO_CFG_OFFSET  0x0718  /* CLK_PERIPH_SGPIO configuration register */
#define LPC43_CCU1_PERIPH_SGPIO_STAT_OFFSET 0x071c  /* CLK_PERIPH_SGPIO status register */
#define LPC43_CCU1_USB0_CFG_OFFSET          0x0800  /* CLK_USB0 configuration register */
#define LPC43_CCU1_USB0_STAT_OFFSET         0x0804  /* CLK_USB0 status register */
#define LPC43_CCU1_USB1_CFG_OFFSET          0x0900  /* CLK_USB1 configuration register */
#define LPC43_CCU1_USB1_STAT_OFFSET         0x0904  /* CLK_USB1 status register */
#define LPC43_CCU1_SPI_CFG_OFFSET           0x0a00  /* CLK_SPI configuration register */
#define LPC43_CCU1_SPI_STAT_OFFSET          0x0a04  /* CLK_SPI status register */
#define LPC43_CCU1_VADC_CFG_OFFSET          0x0b00  /* CLK_VADC configuration register */
#define LPC43_CCU1_VADC_STAT_OFFSET         0x0b04  /* CLK_VADC status register */

#define LPC43_CCU2_PM_OFFSET                0x0000 /* CCU2 power mode register */
#define LPC43_CCU2_BASE_STAT_OFFSET         0x0004 /* CCU2 base clocks status register */
#define LPC43_CCU2_APLL_CFG_OFFSET          0x0100  /* CLK_APLL configuration register */
#define LPC43_CCU2_APLL_STAT_OFFSET         0x0104  /* CLK_APLL status register */
#define LPC43_CCU2_APB2_USART3_CFG_OFFSET   0x0200  /* CLK_APB2_USART3 configuration register */
#define LPC43_CCU2_APB2_USART3_STAT_OFFSET  0x0204  /* CLK_APB2_USART3 status register */
#define LPC43_CCU2_APB2_USART2_CFG_OFFSET   0x0300  /* CLK_APB2_USART2 configuration register */
#define LPC43_CCU2_APB2_USART2_STAT_OFFSET  0x0304  /* CLK_APB2_USART2 status register */
#define LPC43_CCU2_APB0_UART1_CFG_OFFSET    0x0400  /* CLK_APB0_UART1 configuration register */
#define LPC43_CCU2_APB0_UART1_STAT_OFFSET   0x0404  /* CLK_APB0_UART1 status register */
#define LPC43_CCU2_APB0_USART0_CFG_OFFSET   0x0500  /* CLK_APB0_USART0 configuration register */
#define LPC43_CCU2_APB0_USART0_STAT_OFFSET  0x0504  /* CLK_APB0_USART0 status register */
#define LPC43_CCU2_APB2_SSP1_CFG_OFFSET     0x0600  /* CLK_APB2_SSP1 configuration register */
#define LPC43_CCU2_APB2_SSP1_STAT_OFFSET    0x0604  /* CLK_APB2_SSP1 status register */
#define LPC43_CCU2_APB0_SSP0_CFG_OFFSET     0x0700  /* CLK_APB0_SSP0 configuration register */
#define LPC43_CCU2_APB0_SSP0_STAT_OFFSET    0x0704  /* CLK_APB0_SSP0 status register */
#define LPC43_CCU2_SDIO_CFG_OFFSET          0x0800  /* CLK_SDIO configuration register (for SD/MMC) */
#define LPC43_CCU2_SDIO_STAT_OFFSET         0x0804  /* CLK_SDIO status register (for SD/MMC) */

/* Register Addresses *******************************************************************************/

#define LPC43_CCU1_PM                       (LPC43_CCU1_BASE+LPC43_CCU1_PM_OFFSET)
#define LPC43_CCU1_BASE_STAT                (LPC43_CCU1_BASE+LPC43_CCU1_BASE_STAT_OFFSET)
#define LPC43_CCU1_APB3_BUS_CFG             (LPC43_CCU1_BASE+LPC43_CCU1_APB3_BUS_CFG_OFFSET)
#define LPC43_CCU1_APB3_BUS_STAT            (LPC43_CCU1_BASE+LPC43_CCU1_APB3_BUS_STAT_OFFSET)
#define LPC43_CCU1_APB3_I2C1_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_APB3_I2C1_CFG_OFFSET)
#define LPC43_CCU1_APB3_I2C1_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_APB3_I2C1_STAT_OFFSET)
#define LPC43_CCU1_APB3_DAC_CFG             (LPC43_CCU1_BASE+LPC43_CCU1_APB3_DAC_CFG_OFFSET)
#define LPC43_CCU1_APB3_DAC_STAT            (LPC43_CCU1_BASE+LPC43_CCU1_APB3_DAC_STAT_OFFSET)
#define LPC43_CCU1_APB3_ADC0_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_APB3_ADC0_CFG_OFFSET)
#define LPC43_CCU1_APB3_ADC0_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_APB3_ADC0_STAT_OFFSET)
#define LPC43_CCU1_APB3_ADC1_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_APB3_ADC1_CFG_OFFSET)
#define LPC43_CCU1_APB3_ADC1_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_APB3_ADC1_STAT_OFFSET)
#define LPC43_CCU1_APB3_CAN0_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_APB3_CAN0_CFG_OFFSET)
#define LPC43_CCU1_APB3_CAN0_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_APB3_CAN0_STAT_OFFSET)
#define LPC43_CCU1_APB1_BUS_CFG             (LPC43_CCU1_BASE+LPC43_CCU1_APB1_BUS_CFG_OFFSET)
#define LPC43_CCU1_APB1_BUS_STAT            (LPC43_CCU1_BASE+LPC43_CCU1_APB1_BUS_STAT_OFFSET)
#define LPC43_CCU1_APB1_MCPWM_CFG           (LPC43_CCU1_BASE+LPC43_CCU1_APB1_MCPWM_CFG_OFFSET)
#define LPC43_CCU1_APB1_MCPWM_STAT          (LPC43_CCU1_BASE+LPC43_CCU1_APB1_MCPWM_STAT_OFFSET)
#define LPC43_CCU1_APB1_I2C0_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_APB1_I2C0_CFG_OFFSET)
#define LPC43_CCU1_APB1_I2C0_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_APB1_I2C0_STAT_OFFSET)
#define LPC43_CCU1_APB1_I2S_CFG             (LPC43_CCU1_BASE+LPC43_CCU1_APB1_I2S_CFG_OFFSET)
#define LPC43_CCU1_APB1_I2S_STAT            (LPC43_CCU1_BASE+LPC43_CCU1_APB1_I2S_STAT_OFFSET)
#define LPC43_CCU1_APB1_CAN1_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_APB1_CAN1_CFG_OFFSET)
#define LPC43_CCU1_APB1_CAN1_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_APB1_CAN1_STAT_OFFSET)
#define LPC43_CCU1_SPIFI_CFG                (LPC43_CCU1_BASE+LPC43_CCU1_SPIFI_CFG_OFFSET)
#define LPC43_CCU1_SPIFI_STAT               (LPC43_CCU1_BASE+LPC43_CCU1_SPIFI_STAT_OFFSET)
#define LPC43_CCU1_M4_BUS_CFG               (LPC43_CCU1_BASE+LPC43_CCU1_M4_BUS_CFG_OFFSET)
#define LPC43_CCU1_M4_BUS_STAT              (LPC43_CCU1_BASE+LPC43_CCU1_M4_BUS_STAT_OFFSET)
#define LPC43_CCU1_M4_SPIFI_CFG             (LPC43_CCU1_BASE+LPC43_CCU1_M4_SPIFI_CFG_OFFSET)
#define LPC43_CCU1_M4_SPIFI_STAT            (LPC43_CCU1_BASE+LPC43_CCU1_M4_SPIFI_STAT_OFFSET)
#define LPC43_CCU1_M4_GPIO_CFG              (LPC43_CCU1_BASE+LPC43_CCU1_M4_GPIO_CFG_OFFSET)
#define LPC43_CCU1_M4_GPIO_STAT             (LPC43_CCU1_BASE+LPC43_CCU1_M4_GPIO_STAT_OFFSET)
#define LPC43_CCU1_M4_LCD_CFG               (LPC43_CCU1_BASE+LPC43_CCU1_M4_LCD_CFG_OFFSET)
#define LPC43_CCU1_M4_LCD_STAT              (LPC43_CCU1_BASE+LPC43_CCU1_M4_LCD_STAT_OFFSET)
#define LPC43_CCU1_M4_ETHERNET_CFG          (LPC43_CCU1_BASE+LPC43_CCU1_M4_ETHERNET_CFG_OFFSET)
#define LPC43_CCU1_M4_ETHERNET_STAT         (LPC43_CCU1_BASE+LPC43_CCU1_M4_ETHERNET_STAT_OFFSET)
#define LPC43_CCU1_M4_USB0_CFG              (LPC43_CCU1_BASE+LPC43_CCU1_M4_USB0_CFG_OFFSET)
#define LPC43_CCU1_M4_USB0_STAT             (LPC43_CCU1_BASE+LPC43_CCU1_M4_USB0_STAT_OFFSET)
#define LPC43_CCU1_M4_EMC_CFG               (LPC43_CCU1_BASE+LPC43_CCU1_M4_EMC_CFG_OFFSET)
#define LPC43_CCU1_M4_EMC_STAT              (LPC43_CCU1_BASE+LPC43_CCU1_M4_EMC_STAT_OFFSET)
#define LPC43_CCU1_M4_SDIO_CFG              (LPC43_CCU1_BASE+LPC43_CCU1_M4_SDIO_CFG_OFFSET)
#define LPC43_CCU1_M4_SDIO_STAT             (LPC43_CCU1_BASE+LPC43_CCU1_M4_SDIO_STAT_OFFSET)
#define LPC43_CCU1_M4_DMA_CFG               (LPC43_CCU1_BASE+LPC43_CCU1_M4_DMA_CFG_OFFSET)
#define LPC43_CCU1_M4_DMA_STAT              (LPC43_CCU1_BASE+LPC43_CCU1_M4_DMA_STAT_OFFSET)
#define LPC43_CCU1_M4_M4CORE_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_M4_M4CORE_CFG_OFFSET)
#define LPC43_CCU1_M4_M4CORE_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_M4_M4CORE_STAT_OFFSET)
#define LPC43_CCU1_M4_SCT_CFG               (LPC43_CCU1_BASE+LPC43_CCU1_M4_SCT_CFG_OFFSET)
#define LPC43_CCU1_M4_SCT_STAT              (LPC43_CCU1_BASE+LPC43_CCU1_M4_SCT_STAT_OFFSET)
#define LPC43_CCU1_M4_USB1_CFG              (LPC43_CCU1_BASE+LPC43_CCU1_M4_USB1_CFG_OFFSET)
#define LPC43_CCU1_M4_USB1_STAT             (LPC43_CCU1_BASE+LPC43_CCU1_M4_USB1_STAT_OFFSET)
#define LPC43_CCU1_M4_EMCDIV_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_M4_EMCDIV_CFG_OFFSET)
#define LPC43_CCU1_M4_EMCDIV_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_M4_EMCDIV_STAT_OFFSET)
#define LPC43_CCU1_M4_FLASHA_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_M4_FLASHA_CFG_OFFSET)
#define LPC43_CCU1_M4_FLASHA_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_M4_FLASHA_STAT_OFFSET)
#define LPC43_CCU1_M4_FLASHB_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_M4_FLASHB_CFG_OFFSET)
#define LPC43_CCU1_M4_FLASHB_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_M4_FLASHB_STAT_OFFSET)
#define LPC43_CCU1_M4_M0APP_CFG             (LPC43_CCU1_BASE+LPC43_CCU1_M4_M0APP_CFG_OFFSET)
#define LPC43_CCU1_M4_M0APP_STAT            (LPC43_CCU1_BASE+LPC43_CCU1_M4_M0APP_STAT_OFFSET)
#define LPC43_CCU1_M4_VADC_CFG              (LPC43_CCU1_BASE+LPC43_CCU1_M4_VADC_CFG_OFFSET)
#define LPC43_CCU1_M4_VADC_STAT             (LPC43_CCU1_BASE+LPC43_CCU1_M4_VADC_STAT_OFFSET)
#define LPC43_CCU1_M4_EEPROM_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_M4_EEPROM_CFG_OFFSET)
#define LPC43_CCU1_M4_EEPROM_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_M4_EEPROM_STAT_OFFSET)
#define LPC43_CCU1_M4_WWDT_CFG              (LPC43_CCU1_BASE+LPC43_CCU1_M4_WWDT_CFG_OFFSET)
#define LPC43_CCU1_M4_WWDT_STAT             (LPC43_CCU1_BASE+LPC43_CCU1_M4_WWDT_STAT_OFFSET)
#define LPC43_CCU1_M4_USART0_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_M4_USART0_CFG_OFFSET)
#define LPC43_CCU1_M4_USART0_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_M4_USART0_STAT_OFFSET)
#define LPC43_CCU1_M4_UART1_CFG             (LPC43_CCU1_BASE+LPC43_CCU1_M4_UART1_CFG_OFFSET)
#define LPC43_CCU1_M4_UART1_STAT            (LPC43_CCU1_BASE+LPC43_CCU1_M4_UART1_STAT_OFFSET)
#define LPC43_CCU1_M4_SSP0_CFG              (LPC43_CCU1_BASE+LPC43_CCU1_M4_SSP0_CFG_OFFSET)
#define LPC43_CCU1_M4_SSP0_STAT             (LPC43_CCU1_BASE+LPC43_CCU1_M4_SSP0_STAT_OFFSET)
#define LPC43_CCU1_M4_TIMER0_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_M4_TIMER0_CFG_OFFSET)
#define LPC43_CCU1_M4_TIMER0_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_M4_TIMER0_STAT_OFFSET)
#define LPC43_CCU1_M4_TIMER1_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_M4_TIMER1_CFG_OFFSET)
#define LPC43_CCU1_M4_TIMER1_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_M4_TIMER1_STAT_OFFSET)
#define LPC43_CCU1_M4_SCU_CFG               (LPC43_CCU1_BASE+LPC43_CCU1_M4_SCU_CFG_OFFSET)
#define LPC43_CCU1_M4_SCU_STAT              (LPC43_CCU1_BASE+LPC43_CCU1_M4_SCU_STAT_OFFSET)
#define LPC43_CCU1_M4_CREG_CFG              (LPC43_CCU1_BASE+LPC43_CCU1_M4_CREG_CFG_OFFSET)
#define LPC43_CCU1_M4_CREG_STAT             (LPC43_CCU1_BASE+LPC43_CCU1_M4_CREG_STAT_OFFSET)
#define LPC43_CCU1_M4_RITIMER_CFG           (LPC43_CCU1_BASE+LPC43_CCU1_M4_RITIMER_CFG_OFFSET)
#define LPC43_CCU1_M4_RITIMER_STAT          (LPC43_CCU1_BASE+LPC43_CCU1_M4_RITIMER_STAT_OFFSET)
#define LPC43_CCU1_M4_USART2_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_M4_USART2_CFG_OFFSET)
#define LPC43_CCU1_M4_USART2_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_M4_USART2_STAT_OFFSET)
#define LPC43_CCU1_M4_USART3_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_M4_USART3_CFG_OFFSET)
#define LPC43_CCU1_M4_USART3_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_M4_USART3_STAT_OFFSET)
#define LPC43_CCU1_M4_TIMER2_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_M4_TIMER2_CFG_OFFSET)
#define LPC43_CCU1_M4_TIMER2_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_M4_TIMER2_STAT_OFFSET)
#define LPC43_CCU1_M4_TIMER3_CFG            (LPC43_CCU1_BASE+LPC43_CCU1_M4_TIMER3_CFG_OFFSET)
#define LPC43_CCU1_M4_TIMER3_STAT           (LPC43_CCU1_BASE+LPC43_CCU1_M4_TIMER3_STAT_OFFSET)
#define LPC43_CCU1_M4_SSP1_CFG              (LPC43_CCU1_BASE+LPC43_CCU1_M4_SSP1_CFG_OFFSET)
#define LPC43_CCU1_M4_SSP1_STAT             (LPC43_CCU1_BASE+LPC43_CCU1_M4_SSP1_STAT_OFFSET)
#define LPC43_CCU1_M4_QEI_CFG               (LPC43_CCU1_BASE+LPC43_CCU1_M4_QEI_CFG_OFFSET)
#define LPC43_CCU1_M4_QEI_STAT              (LPC43_CCU1_BASE+LPC43_CCU1_M4_QEI_STAT_OFFSET)
#define LPC43_CCU1_PERIPH_BUS_CFG           (LPC43_CCU1_BASE+LPC43_CCU1_PERIPH_BUS_CFG_OFFSET)
#define LPC43_CCU1_PERIPH_BUS_STAT          (LPC43_CCU1_BASE+LPC43_CCU1_PERIPH_BUS_STAT_OFFSET)
#define LPC43_CCU1_PERIPH_CORE_CFG          (LPC43_CCU1_BASE+LPC43_CCU1_PERIPH_CORE_CFG_OFFSET)
#define LPC43_CCU1_PERIPH_CORE_STAT         (LPC43_CCU1_BASE+LPC43_CCU1_PERIPH_CORE_STAT_OFFSET)
#define LPC43_CCU1_PERIPH_SGPIO_CFG         (LPC43_CCU1_BASE+LPC43_CCU1_PERIPH_SGPIO_CFG_OFFSET)
#define LPC43_CCU1_PERIPH_SGPIO_STAT        (LPC43_CCU1_BASE+LPC43_CCU1_PERIPH_SGPIO_STAT_OFFSET)
#define LPC43_CCU1_USB0_CFG                 (LPC43_CCU1_BASE+LPC43_CCU1_USB0_CFG_OFFSET)
#define LPC43_CCU1_USB0_STAT                (LPC43_CCU1_BASE+LPC43_CCU1_USB0_STAT_OFFSET)
#define LPC43_CCU1_USB1_CFG                 (LPC43_CCU1_BASE+LPC43_CCU1_USB1_CFG_OFFSET)
#define LPC43_CCU1_USB1_STAT                (LPC43_CCU1_BASE+LPC43_CCU1_USB1_STAT_OFFSET)
#define LPC43_CCU1_SPI_CFG                  (LPC43_CCU1_BASE+LPC43_CCU1_SPI_CFG_OFFSET)
#define LPC43_CCU1_SPI_STAT                 (LPC43_CCU1_BASE+LPC43_CCU1_SPI_STAT_OFFSET)
#define LPC43_CCU1_VADC_CFG                 (LPC43_CCU1_BASE+LPC43_CCU1_VADC_CFG_OFFSET)
#define LPC43_CCU1_VADC_STAT                (LPC43_CCU1_BASE+LPC43_CCU1_VADC_STAT_OFFSET)

#define LPC43_CCU2_PM                       (LPC43_CCU2_BASE+LPC43_CCU2_PM_OFFSET)
#define LPC43_CCU2_BASE_STAT                (LPC43_CCU2_BASE+LPC43_CCU2_BASE_STAT_OFFSET)
#define LPC43_CCU2_APLL_CFG                 (LPC43_CCU2_BASE+LPC43_CCU2_APLL_CFG_OFFSET)
#define LPC43_CCU2_APLL_STAT                (LPC43_CCU2_BASE+LPC43_CCU2_APLL_STAT_OFFSET)
#define LPC43_CCU2_APB2_USART3_CFG          (LPC43_CCU2_BASE+LPC43_CCU2_APB2_USART3_CFG_OFFSET)
#define LPC43_CCU2_APB2_USART3_STAT         (LPC43_CCU2_BASE+LPC43_CCU2_APB2_USART3_STAT_OFFSET)
#define LPC43_CCU2_APB2_USART2_CFG          (LPC43_CCU2_BASE+LPC43_CCU2_APB2_USART2_CFG_OFFSET)
#define LPC43_CCU2_APB2_USART2_STAT         (LPC43_CCU2_BASE+LPC43_CCU2_APB2_USART2_STAT_OFFSET)
#define LPC43_CCU2_APB0_UART1_CFG           (LPC43_CCU2_BASE+LPC43_CCU2_APB0_UART1_CFG_OFFSET)
#define LPC43_CCU2_APB0_UART1_STAT          (LPC43_CCU2_BASE+LPC43_CCU2_APB0_UART1_STAT_OFFSET)
#define LPC43_CCU2_APB0_USART0_CFG          (LPC43_CCU2_BASE+LPC43_CCU2_APB0_USART0_CFG_OFFSET)
#define LPC43_CCU2_APB0_USART0_STAT         (LPC43_CCU2_BASE+LPC43_CCU2_APB0_USART0_STAT_OFFSET)
#define LPC43_CCU2_APB2_SSP1_CFG            (LPC43_CCU2_BASE+LPC43_CCU2_APB2_SSP1_CFG_OFFSET)
#define LPC43_CCU2_APB2_SSP1_STAT           (LPC43_CCU2_BASE+LPC43_CCU2_APB2_SSP1_STAT_OFFSET)
#define LPC43_CCU2_APB0_SSP0_CFG            (LPC43_CCU2_BASE+LPC43_CCU2_APB0_SSP0_CFG_OFFSET)
#define LPC43_CCU2_APB0_SSP0_STAT           (LPC43_CCU2_BASE+LPC43_CCU2_APB0_SSP0_STAT_OFFSET)
#define LPC43_CCU2_SDIO_CFG                 (LPC43_CCU2_BASE+LPC43_CCU2_SDIO_CFG_OFFSET)
#define LPC43_CCU2_SDIO_STAT                (LPC43_CCU2_BASE+LPC43_CCU2_SDIO_STAT_OFFSET)

/* Register Bit Definitions *************************************************************************/

/* CCU1/2 Power Mode Register */

#define CCU_PM_PD                           (1 << 0)  /* Bit 0: Initiate power-down mode */
                                                      /* Bits 1-31:  Reserved */
/* CCU1 Base Clock Status Register */

#define CCU1_BASE_STAT_AB3                  (1 << 0)  /* Bit 0:  Base clock indicator for BASE_APB3_CLK */
#define CCU1_BASE_STAT_APB1                 (1 << 1)  /* Bit 1:  Base clock indicator for BASE_APB1_CLK */
#define CCU1_BASE_STAT_SPIFI                (1 << 2)  /* Bit 2:  Base clock indicator for BASE_SPIFI_CLK */
#define CCU1_BASE_STAT_M4                   (1 << 3)  /* Bit 3:  Base clock indicator for BASE_M4_CLK */
                                                      /* Bits 4-5:  Reserved */
#define CCU1_BASE_STAT_PERIPH               (1 << 6)  /* Bit 6:  Base clock indicator for BASE_PERIPH_CLK */
#define CCU1_BASE_STAT_USB0                 (1 << 7)  /* Bit 7:  Base clock indicator for BASE_USB0_CLK */
#define CCU1_BASE_STAT_USB1                 (1 << 8)  /* Bit 8:  Base clock indicator for BASE_USB1_CLK */
#define CCU1_BASE_STAT_SPI                  (1 << 9)  /* Bit 9:  Base clock indicator for BASE_SPI_CLK */
                                                      /* Bits 10-31:  Reserved */
/* CCU2 Base Clock Status Register */
                                                      /* Bit 0:  Reserved */
#define CCU2_BASE_STAT_USART3               (1 << 1)  /* Bit 1:  Base clock indicator for BASE_USART3_CLK */
#define CCU2_BASE_STAT_USART2               (1 << 2)  /* Bit 2:  Base clock indicator for BASE_USART2_CLK */
#define CCU2_BASE_STAT_UART1                (1 << 3)  /* Bit 3:  Base clock indicator for BASE_UART1_CLK */
#define CCU2_BASE_STAT_USART0               (1 << 4)  /* Bit 4:  Base clock indicator for BASE_USART0_CLK */
#define CCU2_BASE_STAT_SSP1                 (1 << 5)  /* Bit 5:  Base clock indicator for BASE_SSP1_CLK */
#define CCU2_BASE_STAT_SSP0                 (1 << 6)  /* Bit 6:  Base clock indicator for BASE_SSP0_CLK */
                                                      /* Bits 7-31:  Reserved */
/* CCU1/2 Branch Clock Configuration/Status Registers */

#define CCU_CLK_CFG_RUN                     (1 << 0)  /* Bit 0:  Run enable */
#define CCU_CLK_CFG_AUTO                    (1 << 1)  /* Bit 1:  Auto (AHB disable mechanism) enable */
#define CCU_CLK_CFG_WAKEUP                  (1 << 2)  /* Bit 2:  Wake-up mechanism enable */
                                                      /* Bits 3-31:  Reserved */
/* CCU1/2 Branch Clock Status Registers */

#define CCU_CLK_STAT_RUN                    (1 << 0)  /* Bit 0:  Run enable status */
#define CCU_CLK_STAT_AUTO                   (1 << 1)  /* Bit 1:  Auto (AHB disable mechanism) enable status */
#define CCU_CLK_STAT_WAKEUP                 (1 << 2)  /* Bit 2:  Wake-up mechanism enable status */
                                                      /* Bits 3-31:  Reserved */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_CCU_H */
