/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_cgu.h
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   - UM10314 LPC3130/31 User manual Rev. 1.01 — 9 September 2009
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_CGU_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_CGU_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* CGU register base address offset into the APB0 domain ****************************************/

/* APB0 offsets to Clock Switch Box (CSB) and CGU Configuration (CFG) register groups */

#define LPC31_APB0_GCU_CSB_OFFSET      (LPC31_APB0_GCU_OFFSET)
#define LPC31_APB0_GCU_CFG_OFFSET      (LPC31_APB0_GCU_OFFSET+0x0c00)

/* Virtual and physical base address of the CGU block and CSB and CFG register groups */

#define LPC31_CGU_VBASE                (LPC31_APB0_VADDR+LPC31_APB0_CGU_OFFSET)
#define LPC31_CGU_PBASE                (LPC31_APB0_PADDR+LPC31_APB0_CGU_OFFSET)

#define LPC31_CGU_CSB_VBASE            (LPC31_APB0_VADDR+LPC31_APB0_GCU_CSB_OFFSET)
#define LPC31_CGU_CSB_PBASE            (LPC31_APB0_PADDR+LPC31_APB0_GCU_CSB_OFFSET)

#define LPC31_CGU_CFG_VBASE            (LPC31_APB0_VADDR+LPC31_APB0_GCU_CFG_OFFSET)
#define LPC31_CGU_CFG_PBASE            (LPC31_APB0_PADDR+LPC31_APB0_GCU_CFG_OFFSET)

/* CGU register offsets *************************************************************************/
/* CGU clock switchbox register offsets (with respect to the CGU CSB register base) *************/
/* Switch configuration registers (SCR) for base clocks */

#define LPC31_CGU_SCR_OFFSET(n)        (0x000+((n)<<2))
#define LPC31_CGU_SCR0_OFFSET          0x000 /* SYS base */
#define LPC31_CGU_SCR1_OFFSET          0x004 /* AHB0_APB0 base */
#define LPC31_CGU_SCR2_OFFSET          0x008 /* AHB0_APB1 base */
#define LPC31_CGU_SCR3_OFFSET          0x00c /* AHB0_APB2 base */
#define LPC31_CGU_SCR4_OFFSET          0x010 /* AHB0_APB3 base */
#define LPC31_CGU_SCR5_OFFSET          0x014 /* PCM base */
#define LPC31_CGU_SCR6_OFFSET          0x018 /* UART base */
#define LPC31_CGU_SCR7_OFFSET          0x01c /* CLK1024FS base */
#define LPC31_CGU_SCR8_OFFSET          0x020 /* I2SRX_BCK0 base */
#define LPC31_CGU_SCR9_OFFSET          0x024 /* I2SRX_BCK1 base */
#define LPC31_CGU_SCR10_OFFSET         0x028 /* SPI_CLK base */
#define LPC31_CGU_SCR11_OFFSET         0x02c /* SYSCLK_O base */

/* Frequency select (FS) registers 1 for base clocks */

#define LPC31_CGU_FS1_OFFSET(n)        (0x030+((n)<<2))
#define LPC31_CGU_FS1_0_OFFSET         0x030 /* SYS base */
#define LPC31_CGU_FS1_1_OFFSET         0x034 /* AHB0_APB0 base */
#define LPC31_CGU_FS1_2_OFFSET         0x038 /* AHB0_APB1 base */
#define LPC31_CGU_FS1_3_OFFSET         0x03c /* AHB0_APB2 base */
#define LPC31_CGU_FS1_4_OFFSET         0x040 /* AHB0_APB3 base */
#define LPC31_CGU_FS1_5_OFFSET         0x044 /* PCM base */
#define LPC31_CGU_FS1_6_OFFSET         0x048 /* UART base */
#define LPC31_CGU_FS1_7_OFFSET         0x04c /* CLK1024FS base */
#define LPC31_CGU_FS1_8_OFFSET         0x050 /* I2SRX_BCK0 base */
#define LPC31_CGU_FS1_9_OFFSET         0x054 /* I2SRX_BCK1 base */
#define LPC31_CGU_FS1_10_OFFSET        0x058 /* SPI_CLK base */
#define LPC31_CGU_FS1_11_OFFSET        0x05c /* SYSCLK_O base */

/* Frequency select (FS) registers 2 for base clocks */

#define LPC31_CGU_FS2_OFFSET(n)        (0x060+((n)<<2))
#define LPC31_CGU_FS2_0_OFFSET         0x060 /* SYS base */
#define LPC31_CGU_FS2_1_OFFSET         0x064 /* AHB0_APB0 base */
#define LPC31_CGU_FS2_2_OFFSET         0x068 /* AHB0_APB1 base */
#define LPC31_CGU_FS2_3_OFFSET         0x06c /* AHB0_APB2 base */
#define LPC31_CGU_FS2_4_OFFSET         0x070 /* AHB0_APB3 base */
#define LPC31_CGU_FS2_5_OFFSET         0x074 /* PCM base */
#define LPC31_CGU_FS2_6_OFFSET         0x078 /* UART base */
#define LPC31_CGU_FS2_7_OFFSET         0x07c /* CLK1024FS base */
#define LPC31_CGU_FS2_8_OFFSET         0x080 /* I2SRX_BCK0 base */
#define LPC31_CGU_FS2_9_OFFSET         0x084 /* I2SRX_BCK1 base */
#define LPC31_CGU_FS2_10_OFFSET        0x088 /* SPI_CLK base */
#define LPC31_CGU_FS2_11_OFFSET        0x08c /* SYSCLK_O base */

/* Switch status registers (SSR) for base clocks */

#define LPC31_CGU_SSR_OFFSET(n)        (0x090+((n)<<2))
#define LPC31_CGU_SSR0_OFFSET          0x090 /* SYS base */
#define LPC31_CGU_SSR1_OFFSET          0x094 /* AHB0_APB0 base */
#define LPC31_CGU_SSR2_OFFSET          0x098 /* AHB0_APB1 base */
#define LPC31_CGU_SSR3_OFFSET          0x09c /* AHB0_APB2 base */
#define LPC31_CGU_SSR4_OFFSET          0x0a0 /* AHB0_APB3 base */
#define LPC31_CGU_SSR5_OFFSET          0x0a4 /* PCM base */
#define LPC31_CGU_SSR6_OFFSET          0x0a8 /* UART base */
#define LPC31_CGU_SSR7_OFFSET          0x0ac /* CLK1024FS base */
#define LPC31_CGU_SSR8_OFFSET          0x0b0 /* I2SRX_BCK0 base */
#define LPC31_CGU_SSR9_OFFSET          0x0b4 /* I2SRX_BCK1 base */
#define LPC31_CGU_SSR10_OFFSET         0x0b8 /* SPI_CLK base */
#define LPC31_CGU_SSR11_OFFSET         0x0bc /* SYSCLK_O base */

/* Power control registers (PCR), spreading stage */

#define LPC31_CGU_PCR_OFFSET(n)        (0x0c0+((n)<<2))
#define LPC31_CGU_PCR0_OFFSET          0x0c0 /* APB0_CLK */
#define LPC31_CGU_PCR1_OFFSET          0x0c4 /* APB1_CLK */
#define LPC31_CGU_PCR2_OFFSET          0x0c8 /* APB2_CLK */
#define LPC31_CGU_PCR3_OFFSET          0x0cc /* APB3_CLK */
#define LPC31_CGU_PCR4_OFFSET          0x0d0 /* APB4_CLK */
#define LPC31_CGU_PCR5_OFFSET          0x0d4 /* AHB_TO_INTC_CLK */
#define LPC31_CGU_PCR6_OFFSET          0x0d8 /* AHB0_CLK */
#define LPC31_CGU_PCR7_OFFSET          0x0dc /* EBI_CLK */
#define LPC31_CGU_PCR8_OFFSET          0x0e0 /* DMA_PCLK */
#define LPC31_CGU_PCR9_OFFSET          0x0e4 /* DMA_CLK_GATED */
#define LPC31_CGU_PCR10_OFFSET         0x0e8 /* NANDFLASH_S0_CLK */
#define LPC31_CGU_PCR11_OFFSET         0x0ec /* NANDFLASH_ECC_CLK */
#define LPC31_CGU_PCR12_OFFSET         0x0f0 /* Reserved */
#define LPC31_CGU_PCR13_OFFSET         0x0f4 /* NANDFLASH_NAND_CLK */
#define LPC31_CGU_PCR14_OFFSET         0x0f8 /* NANDFLASH_PCLK */
#define LPC31_CGU_PCR15_OFFSET         0x0fc /* CLOCK_OUT */
#define LPC31_CGU_PCR16_OFFSET         0x100 /* ARM926_CORE_CLK */
#define LPC31_CGU_PCR17_OFFSET         0x104 /* ARM926_BUSIF_CLK */
#define LPC31_CGU_PCR18_OFFSET         0x108 /* ARM926_RETIME_CLK */
#define LPC31_CGU_PCR19_OFFSET         0x10c /* SD_MMC_HCLK */
#define LPC31_CGU_PCR20_OFFSET         0x110 /* SD_MMC_CCLK_IN */
#define LPC31_CGU_PCR21_OFFSET         0x114 /* USB_OTG_AHB_CLK */
#define LPC31_CGU_PCR22_OFFSET         0x118 /* ISRAM0_CLK */
#define LPC31_CGU_PCR23_OFFSET         0x11c /* RED_CTL_RSCLK */
#define LPC31_CGU_PCR24_OFFSET         0x120 /* ISRAM1_CLK (LPC313x only) */
#define LPC31_CGU_PCR25_OFFSET         0x124 /* ISROM_CLK */
#define LPC31_CGU_PCR26_OFFSET         0x128 /* MPMC_CFG_CLK */
#define LPC31_CGU_PCR27_OFFSET         0x12c /* MPMC_CFG_CLK2 */
#define LPC31_CGU_PCR28_OFFSET         0x130 /* MPMC_CFG_CLK3 */
#define LPC31_CGU_PCR29_OFFSET         0x134 /* INTC_CLK */
#define LPC31_CGU_PCR30_OFFSET         0x138 /* AHB_TO_APB0_PCLK */
#define LPC31_CGU_PCR31_OFFSET         0x13c /* EVENT_ROUTER_PCLK */
#define LPC31_CGU_PCR32_OFFSET         0x140 /* ADC_PCLK */
#define LPC31_CGU_PCR33_OFFSET         0x144 /* ADC_CLK */
#define LPC31_CGU_PCR34_OFFSET         0x148 /* WDOG_PCLK */
#define LPC31_CGU_PCR35_OFFSET         0x14c /* IOCONF_PCLK */
#define LPC31_CGU_PCR36_OFFSET         0x150 /* CGU_PCLK */
#define LPC31_CGU_PCR37_OFFSET         0x154 /* SYSCREG_PCLK */
#define LPC31_CGU_PCR38_OFFSET         0x158 /* Reserved */
#define LPC31_CGU_PCR39_OFFSET         0x15c /* RNG_PCLK */
#define LPC31_CGU_PCR40_OFFSET         0x160 /* AHB_TO_APB1_PCLK */
#define LPC31_CGU_PCR41_OFFSET         0x164 /* TIMER0_PCLK */
#define LPC31_CGU_PCR42_OFFSET         0x168 /* TIMER1_PCLK */
#define LPC31_CGU_PCR43_OFFSET         0x16c /* TIMER2_PCLK */
#define LPC31_CGU_PCR44_OFFSET         0x170 /* TIMER3_PCLK */
#define LPC31_CGU_PCR45_OFFSET         0x174 /* PWM_PCLK */
#define LPC31_CGU_PCR46_OFFSET         0x178 /* PWM_PCLK_REGS */
#define LPC31_CGU_PCR47_OFFSET         0x17c /* PWM_CLK */
#define LPC31_CGU_PCR48_OFFSET         0x180 /* I2C0_PCLK */
#define LPC31_CGU_PCR49_OFFSET         0x184 /* I2C1_PCLK */
#define LPC31_CGU_PCR50_OFFSET         0x188 /* AHB_TO_APB2_PCLK */
#define LPC31_CGU_PCR51_OFFSET         0x18c /* PCM_PCLK */
#define LPC31_CGU_PCR52_OFFSET         0x190 /* PCM_APB_PCLK */
#define LPC31_CGU_PCR53_OFFSET         0x194 /* UART_APB_CLK */
#define LPC31_CGU_PCR54_OFFSET         0x198 /* LCD_PCLK */
#define LPC31_CGU_PCR55_OFFSET         0x19c /* LCD_CLK */
#define LPC31_CGU_PCR56_OFFSET         0x1a0 /* SPI_PCLK */
#define LPC31_CGU_PCR57_OFFSET         0x1a4 /* SPI_PCLK_GATED */
#define LPC31_CGU_PCR58_OFFSET         0x1a8 /* AHB_TO_APB3_PCLK */
#define LPC31_CGU_PCR59_OFFSET         0x1ac /* I2S_CFG_PCLK */
#define LPC31_CGU_PCR60_OFFSET         0x1b0 /* EDGE_DET_PCLK */
#define LPC31_CGU_PCR61_OFFSET         0x1b4 /* I2STX_FIFO_0_PCLK */
#define LPC31_CGU_PCR62_OFFSET         0x1b8 /* I2STX_IF_0_PCLK */
#define LPC31_CGU_PCR63_OFFSET         0x1bc /* I2STX_FIFO_1_PCLK */
#define LPC31_CGU_PCR64_OFFSET         0x1c0 /* I2STX_IF_1_PCLK */
#define LPC31_CGU_PCR65_OFFSET         0x1c4 /* I2SRX_FIFO_0_PCLK */
#define LPC31_CGU_PCR66_OFFSET         0x1c8 /* I2SRX_IF_0_PCLK */
#define LPC31_CGU_PCR67_OFFSET         0x1cc /* I2SRX_FIFO_1_PCLK */
#define LPC31_CGU_PCR68_OFFSET         0x1d0 /* I2SRX_IF_1_PCLK */
#define LPC31_CGU_PCR69_OFFSET         0x1d4 /* Reserved */
#define LPC31_CGU_PCR70_OFFSET         0x1d8 /* Reserved */
#define LPC31_CGU_PCR71_OFFSET         0x1dc /* PCM_CLK_IP */
#define LPC31_CGU_PCR72_OFFSET         0x1e0 /* UART_U_CLK */
#define LPC31_CGU_PCR73_OFFSET         0x1e4 /* I2S_EDGE_DETECT_CLK */
#define LPC31_CGU_PCR74_OFFSET         0x1e8 /* I2STX_BCK0_N */
#define LPC31_CGU_PCR75_OFFSET         0x1ec /* I2STX_WS0 */
#define LPC31_CGU_PCR76_OFFSET         0x1f0 /* I2STX_CLK0 */
#define LPC31_CGU_PCR77_OFFSET         0x1f4 /* I2STX_BCK1_N */
#define LPC31_CGU_PCR78_OFFSET         0x1f8 /* I2STX_WS1 */
#define LPC31_CGU_PCR79_OFFSET         0x1fc /* CLK_256FS */
#define LPC31_CGU_PCR80_OFFSET         0x200 /* I2SRX_BCK0_N */
#define LPC31_CGU_PCR81_OFFSET         0x204 /* I2SRX_WS0 */
#define LPC31_CGU_PCR82_OFFSET         0x208 /* I2SRX_BCK1_N */
#define LPC31_CGU_PCR83_OFFSET         0x20c /* I2SRX_WS1 */
#define LPC31_CGU_PCR84_OFFSET         0x210 /* Reserved */
#define LPC31_CGU_PCR85_OFFSET         0x214 /* Reserved */
#define LPC31_CGU_PCR86_OFFSET         0x218 /* Reserved */
#define LPC31_CGU_PCR87_OFFSET         0x21c /* I2SRX_BCK0 */
#define LPC31_CGU_PCR88_OFFSET         0x220 /* I2SRX_BCK1 */
#define LPC31_CGU_PCR89_OFFSET         0x224 /* SPI_CLK */
#define LPC31_CGU_PCR90_OFFSET         0x228 /* SPI_CLK_GATED */
#define LPC31_CGU_PCR91_OFFSET         0x22c /* SYSCLK_O */

/* Power status registers (PSR), spreading stage */

#define LPC31_CGU_PSR_OFFSET(n)        (0x230+((n)<<2))
#define LPC31_CGU_PSR0_OFFSET          0x230 /* PB0_CLK */
#define LPC31_CGU_PSR1_OFFSET          0x234 /* PB1_CLK */
#define LPC31_CGU_PSR2_OFFSET          0x238 /* PB2_CLK */
#define LPC31_CGU_PSR3_OFFSET          0x23c /* PB3_CLK */
#define LPC31_CGU_PSR4_OFFSET          0x240 /* PB4_CLK */
#define LPC31_CGU_PSR5_OFFSET          0x244 /* HB_TO_INTC_CLK */
#define LPC31_CGU_PSR6_OFFSET          0x248 /* HB0_CLK */
#define LPC31_CGU_PSR7_OFFSET          0x24c /* EBI_CLK */
#define LPC31_CGU_PSR8_OFFSET          0x250 /* DMA_PCLK */
#define LPC31_CGU_PSR9_OFFSET          0x254 /* DMA_CLK_GATED */
#define LPC31_CGU_PSR10_OFFSET         0x258 /* NANDFLASH_S0_CLK */
#define LPC31_CGU_PSR11_OFFSET         0x25c /* NANDFLASH_ECC_CLK */
#define LPC31_CGU_PSR12_OFFSET         0x260 /* Reserved */
#define LPC31_CGU_PSR13_OFFSET         0x264 /* NANDFLASH_NAND_CLK */
#define LPC31_CGU_PSR14_OFFSET         0x268 /* NANDFLASH_PCLK */
#define LPC31_CGU_PSR15_OFFSET         0x26c /* CLOCK_OUT */
#define LPC31_CGU_PSR16_OFFSET         0x270 /* RM926_CORE_CLK */
#define LPC31_CGU_PSR17_OFFSET         0x274 /* RM926_BUSIF_CLK */
#define LPC31_CGU_PSR18_OFFSET         0x278 /* RM926_RETIME_CLK */
#define LPC31_CGU_PSR19_OFFSET         0x27c /* SD_MMC_HCLK */
#define LPC31_CGU_PSR20_OFFSET         0x280 /* SD_MMC_CCLK_IN */
#define LPC31_CGU_PSR21_OFFSET         0x284 /* USB_OTG_AHB_CLK */
#define LPC31_CGU_PSR22_OFFSET         0x288 /* ISRAM0_CLK */
#define LPC31_CGU_PSR23_OFFSET         0x28c /* RED_CTL_RSCLK */
#define LPC31_CGU_PSR24_OFFSET         0x290 /* ISRAM1_CLK */
#define LPC31_CGU_PSR25_OFFSET         0x294 /* ISROM_CLK */
#define LPC31_CGU_PSR26_OFFSET         0x298 /* MPMC_CFG_CLK */
#define LPC31_CGU_PSR27_OFFSET         0x29c /* MPMC_CFG_CLK2 */
#define LPC31_CGU_PSR28_OFFSET         0x2a0 /* MPMC_CFG_CLK3 */
#define LPC31_CGU_PSR29_OFFSET         0x2a4 /* INTC_CLK */
#define LPC31_CGU_PSR30_OFFSET         0x2a8 /* HB_TO_APB0_PCLK */
#define LPC31_CGU_PSR31_OFFSET         0x2ac /* EVENT_ROUTER_PCLK */
#define LPC31_CGU_PSR32_OFFSET         0x2b0 /* DC_PCLK */
#define LPC31_CGU_PSR33_OFFSET         0x2b4 /* DC_CLK */
#define LPC31_CGU_PSR34_OFFSET         0x2b8 /* WDOG_PCLK */
#define LPC31_CGU_PSR35_OFFSET         0x2bc /* IOCONF_PCLK */
#define LPC31_CGU_PSR36_OFFSET         0x2c0 /* CGU_PCLK */
#define LPC31_CGU_PSR37_OFFSET         0x2c4 /* SYSCREG_PCLK */
#define LPC31_CGU_PSR38_OFFSET         0x2c8 /* Reserved */
#define LPC31_CGU_PSR39_OFFSET         0x2cc /* RNG_PCLK */
#define LPC31_CGU_PSR40_OFFSET         0x2d0 /* HB_TO_APB1_PCLK */
#define LPC31_CGU_PSR41_OFFSET         0x2d4 /* TIMER0_PCLK */
#define LPC31_CGU_PSR42_OFFSET         0x2d8 /* TIMER1_PCLK */
#define LPC31_CGU_PSR43_OFFSET         0x2dc /* TIMER2_PCLK */
#define LPC31_CGU_PSR44_OFFSET         0x2e0 /* TIMER3_PCLK */
#define LPC31_CGU_PSR45_OFFSET         0x2e4 /* PWM_PCLK */
#define LPC31_CGU_PSR46_OFFSET         0x2e8 /* PWM_PCLK_REGS */
#define LPC31_CGU_PSR47_OFFSET         0x2ec /* PWM_CLK */
#define LPC31_CGU_PSR48_OFFSET         0x2f0 /* I2C0_PCLK */
#define LPC31_CGU_PSR49_OFFSET         0x2f4 /* I2C1_PCLK */
#define LPC31_CGU_PSR50_OFFSET         0x2f8 /* HB_TO_APB2_PCLK */
#define LPC31_CGU_PSR51_OFFSET         0x2fc /* PCM_PCLK */
#define LPC31_CGU_PSR52_OFFSET         0x300 /* PCM_APB_PCLK */
#define LPC31_CGU_PSR53_OFFSET         0x304 /* UART_APB_CLK */
#define LPC31_CGU_PSR54_OFFSET         0x308 /* LCD_PCLK */
#define LPC31_CGU_PSR55_OFFSET         0x30c /* LCD_CLK */
#define LPC31_CGU_PSR56_OFFSET         0x310 /* SPI_PCLK */
#define LPC31_CGU_PSR57_OFFSET         0x314 /* SPI_PCLK_GATED */
#define LPC31_CGU_PSR58_OFFSET         0x318 /* HB_TO_APB3_PCLK */
#define LPC31_CGU_PSR59_OFFSET         0x31c /* I2S_CFG_PCLK */
#define LPC31_CGU_PSR60_OFFSET         0x320 /* EDGE_DET_PCLK */
#define LPC31_CGU_PSR61_OFFSET         0x324 /* I2STX_FIFO_0_PCLK */
#define LPC31_CGU_PSR62_OFFSET         0x328 /* I2STX_IF_0_PCLK */
#define LPC31_CGU_PSR63_OFFSET         0x32c /* I2STX_FIFO_1_PCLK */
#define LPC31_CGU_PSR64_OFFSET         0x330 /* I2STX_IF_1_PCLK */
#define LPC31_CGU_PSR65_OFFSET         0x334 /* I2SRX_FIFO_0_PCLK */
#define LPC31_CGU_PSR66_OFFSET         0x338 /* I2SRX_IF_0_PCLK */
#define LPC31_CGU_PSR67_OFFSET         0x33c /* I2SRX_FIFO_1_PCLK */
#define LPC31_CGU_PSR68_OFFSET         0x340 /* I2SRX_IF_1_PCLK */
#define LPC31_CGU_PSR69_OFFSET         0x344 /* Reserved */
#define LPC31_CGU_PSR70_OFFSET         0x348 /* Reserved */
#define LPC31_CGU_PSR71_OFFSET         0x34c /* PCM_CLK_IP */
#define LPC31_CGU_PSR72_OFFSET         0x350 /* UART_U_CLK */
#define LPC31_CGU_PSR73_OFFSET         0x354 /* I2S_EDGE_DETECT_CLK */
#define LPC31_CGU_PSR74_OFFSET         0x358 /* I2STX_BCK0_N */
#define LPC31_CGU_PSR75_OFFSET         0x35c /* I2STX_WS0 */
#define LPC31_CGU_PSR76_OFFSET         0x360 /* I2STX_CLK0 */
#define LPC31_CGU_PSR77_OFFSET         0x364 /* I2STX_BCK1_N */
#define LPC31_CGU_PSR78_OFFSET         0x368 /* I2STX_WS1 */
#define LPC31_CGU_PSR79_OFFSET         0x36c /* CLK_256FS */
#define LPC31_CGU_PSR80_OFFSET         0x370 /* I2SRX_BCK0_N */
#define LPC31_CGU_PSR81_OFFSET         0x374 /* I2SRX_WS0 */
#define LPC31_CGU_PSR82_OFFSET         0x378 /* I2SRX_BCK1_N */
#define LPC31_CGU_PSR83_OFFSET         0x37c /* I2SRX_WS1 */
#define LPC31_CGU_PSR84_OFFSET         0x380 /* Reserved */
#define LPC31_CGU_PSR85_OFFSET         0x384 /* Reserved */
#define LPC31_CGU_PSR86_OFFSET         0x388 /* Reserved */
#define LPC31_CGU_PSR87_OFFSET         0x38c /* I2SRX_BCK0 */
#define LPC31_CGU_PSR88_OFFSET         0x390 /* I2SRX_BCK1 */
#define LPC31_CGU_PSR89_OFFSET         0x394 /* SPI_CLK */
#define LPC31_CGU_PSR90_OFFSET         0x398 /* SPI_CLK_GATED */
#define LPC31_CGU_PSR91_OFFSET         0x39c /* SYSCLK_O */

/* Enable select registers (ESR), spreading stage */

#define LPC31_CGU_ESR_OFFSET(n)        (0x3a0+((n)<<2))
#define LPC31_CGU_ESR0_OFFSET          0x3a0 /* APB0_CLK */
#define LPC31_CGU_ESR1_OFFSET          0x3a4 /* APB1_CLK */
#define LPC31_CGU_ESR2_OFFSET          0x3A8 /* APB2_CLK */
#define LPC31_CGU_ESR3_OFFSET          0x3ac /* APB3_CLK */
#define LPC31_CGU_ESR4_OFFSET          0x3b0 /* APB4_CLK */
#define LPC31_CGU_ESR5_OFFSET          0x3b4 /* AHB_TO_INTC_CLK */
#define LPC31_CGU_ESR6_OFFSET          0x3b8 /* AHB0_CLK */
#define LPC31_CGU_ESR7_OFFSET          0x3bc /* EBI_CLK */
#define LPC31_CGU_ESR8_OFFSET          0x3c0 /* DMA_PCLK */
#define LPC31_CGU_ESR9_OFFSET          0x3c4 /* DMA_CLK_GATED */
#define LPC31_CGU_ESR10_OFFSET         0x3c8 /* NANDFLASH_S0_CLK */
#define LPC31_CGU_ESR11_OFFSET         0x3cc /* NANDFLASH_ECC_CLK */
#define LPC31_CGU_ESR12_OFFSET         0x3d0 /* Reserved */
#define LPC31_CGU_ESR13_OFFSET         0x3d4 /* NANDFLASH_NAND_CLK */
#define LPC31_CGU_ESR14_OFFSET         0x3d8 /* NANDFLASH_PCLK */
#define LPC31_CGU_ESR15_OFFSET         0x3dc /* CLOCK_OUT */
#define LPC31_CGU_ESR16_OFFSET         0x3e0 /* ARM926_CORE_CLK */
#define LPC31_CGU_ESR17_OFFSET         0x3e4 /* ARM926_BUSIF_CLK */
#define LPC31_CGU_ESR18_OFFSET         0x3e8 /* ARM926_RETIME_CLK */
#define LPC31_CGU_ESR19_OFFSET         0x3ec /* SD_MMC_HCLK */
#define LPC31_CGU_ESR20_OFFSET         0x3f0 /* SD_MMC_CCLK_IN */
#define LPC31_CGU_ESR21_OFFSET         0x3f4 /* USB_OTG_AHB_CLK */
#define LPC31_CGU_ESR22_OFFSET         0x3f8 /* ISRAM0_CLK */
#define LPC31_CGU_ESR23_OFFSET         0x3fc /* RED_CTL_RSCLK */
#define LPC31_CGU_ESR24_OFFSET         0x400 /* ISRAM1_CLK */
#define LPC31_CGU_ESR25_OFFSET         0x404 /* ISROM_CLK */
#define LPC31_CGU_ESR26_OFFSET         0x408 /* MPMC_CFG_CLK */
#define LPC31_CGU_ESR27_OFFSET         0x40c /* MPMC_CFG_CLK2 */
#define LPC31_CGU_ESR28_OFFSET         0x410 /* MPMC_CFG_CLK3 */
#define LPC31_CGU_ESR29_OFFSET         0x414 /* INTC_CLK */
#define LPC31_CGU_ESR30_OFFSET         0x418 /* AHB_TO_APB0_PCLK */
#define LPC31_CGU_ESR31_OFFSET         0x41c /* EVENT_ROUTER_PCLK */
#define LPC31_CGU_ESR32_OFFSET         0x420 /* ADC_PCLK */
#define LPC31_CGU_ESR33_OFFSET         0x424 /* ADC_CLK */
#define LPC31_CGU_ESR34_OFFSET         0x428 /* WDOG_PCLK */
#define LPC31_CGU_ESR35_OFFSET         0x42c /* IOCONF_PCLK */
#define LPC31_CGU_ESR36_OFFSET         0x430 /* CGU_PCLK */
#define LPC31_CGU_ESR37_OFFSET         0x434 /* SYSCREG_PCLK */
#define LPC31_CGU_ESR38_OFFSET         0x438 /* Reserved */
#define LPC31_CGU_ESR39_OFFSET         0x43c /* RNG_PCLK */
#define LPC31_CGU_ESR40_OFFSET         0x440 /* AHB_TO_APB1_PCLK */
#define LPC31_CGU_ESR41_OFFSET         0x444 /* TIMER0_PCLK */
#define LPC31_CGU_ESR42_OFFSET         0x448 /* TIMER1_PCLK */
#define LPC31_CGU_ESR43_OFFSET         0x44c /* TIMER2_PCLK */
#define LPC31_CGU_ESR44_OFFSET         0x450 /* TIMER3_PCLK */
#define LPC31_CGU_ESR45_OFFSET         0x454 /* PWM_PCLK */
#define LPC31_CGU_ESR46_OFFSET         0x458 /* PWM_PCLK_REGS */
#define LPC31_CGU_ESR47_OFFSET         0x45c /* PWM_CLK */
#define LPC31_CGU_ESR48_OFFSET         0x460 /* I2C0_PCLK */
#define LPC31_CGU_ESR49_OFFSET         0x464 /* I2C1_PCLK */
#define LPC31_CGU_ESR50_OFFSET         0x468 /* AHB_TO_APB2_PCLK */
#define LPC31_CGU_ESR51_OFFSET         0x46c /* PCM_PCLK */
#define LPC31_CGU_ESR52_OFFSET         0x470 /* PCM_APB_PCLK */
#define LPC31_CGU_ESR53_OFFSET         0x474 /* UART_APB_CLK */
#define LPC31_CGU_ESR54_OFFSET         0x478 /* LCD_PCLK */
#define LPC31_CGU_ESR55_OFFSET         0x47c /* LCD_CLK */
#define LPC31_CGU_ESR56_OFFSET         0x480 /* SPI_PCLK */
#define LPC31_CGU_ESR57_OFFSET         0x484 /* SPI_PCLK_GATED */
#define LPC31_CGU_ESR58_OFFSET         0x488 /* AHB_TO_APB3_PCLK */
#define LPC31_CGU_ESR59_OFFSET         0x48c /* I2S_CFG_PCLK */
#define LPC31_CGU_ESR60_OFFSET         0x490 /* EDGE_DET_PCLK */
#define LPC31_CGU_ESR61_OFFSET         0x494 /* I2STX_FIFO_0_PCLK */
#define LPC31_CGU_ESR62_OFFSET         0x498 /* I2STX_IF_0_PCLK */
#define LPC31_CGU_ESR63_OFFSET         0x49c /* I2STX_FIFO_1_PCLK */
#define LPC31_CGU_ESR64_OFFSET         0x4a0 /* I2STX_IF_1_PCLK */
#define LPC31_CGU_ESR65_OFFSET         0x4a4 /* I2SRX_FIFO_0_PCLK */
#define LPC31_CGU_ESR66_OFFSET         0x4a8 /* I2SRX_IF_0_PCLK */
#define LPC31_CGU_ESR67_OFFSET         0x4ac /* I2SRX_FIFO_1_PCLK */
#define LPC31_CGU_ESR68_OFFSET         0x4b0 /* I2SRX_IF_1_PCLK */
#define LPC31_CGU_ESR69_OFFSET         0x4b4 /* Reserved */
#define LPC31_CGU_ESR70_OFFSET         0x4b8 /* Reserved */
#define LPC31_CGU_ESR71_OFFSET         0x4bc /* PCM_CLK_IP */
#define LPC31_CGU_ESR72_OFFSET         0x4c0 /* UART_U_CLK */
#define LPC31_CGU_ESR73_OFFSET         0x4c4 /* I2S_EDGE_DETECT_CLK */
#define LPC31_CGU_ESR74_OFFSET         0x4c8 /* R_I2STX_BCK0_N */
#define LPC31_CGU_ESR75_OFFSET         0x4cc /* I2STX_WS0 */
#define LPC31_CGU_ESR76_OFFSET         0x4d0 /* I2STX_CLK0 */
#define LPC31_CGU_ESR77_OFFSET         0x4d4 /* I2STX_IF_BCK1_N */
#define LPC31_CGU_ESR78_OFFSET         0x4d8 /* I2STX_WS1 */
#define LPC31_CGU_ESR79_OFFSET         0x4dc /* CLK_256FS */
#define LPC31_CGU_ESR80_OFFSET         0x4e0 /* I2SRX_BCK0_N */
#define LPC31_CGU_ESR81_OFFSET         0x4e4 /* I2SRX_WS0 */
#define LPC31_CGU_ESR82_OFFSET         0x4e8 /* I2SRX_BCK1_N */
#define LPC31_CGU_ESR83_OFFSET         0x4ec /* I2SRX_WS1 */
#define LPC31_CGU_ESR84_OFFSET         0x4f0 /* Reserved */
#define LPC31_CGU_ESR85_OFFSET         0x4f4 /* Reserved */
#define LPC31_CGU_ESR86_OFFSET         0x4f8 /* Reserved */
#define LPC31_CGU_ESR87_OFFSET         0x4fc /* SPI_CLK */
#define LPC31_CGU_ESR88_OFFSET         0x500 /* SPI_CLK_GATED */

/* Base control registers (BCR) for SYS base */

#define LPC31_CGU_BCR_OFFSET(n)        (0x504+((n)<<2))
#define LPC31_CGU_BCR0_OFFSET          0x504 /* SYS base */
#define LPC31_CGU_BCR1_OFFSET          0x508 /* AHB0_APB0 base */
#define LPC31_CGU_BCR2_OFFSET          0x50c /* AHB0_APB1 base */
#define LPC31_CGU_BCR3_OFFSET          0x510 /* AHB0_APB2 base */
#define LPC31_CGU_BCR7_OFFSET          0x514 /* CLK1024FS base */

/* Fractional divider configuration (FDC) registers */

#define LPC31_CGU_FDC_OFFSET(n)        (0x518+((n)<<2))
#define LPC31_CGU_FDC0_OFFSET          0x518 /* Fractional Divider 0 (SYS base) */
#define LPC31_CGU_FDC1_OFFSET          0x51c /* Fractional Divider 1 (SYS base) */
#define LPC31_CGU_FDC2_OFFSET          0x520 /* Fractional Divider 2 (SYS base) */
#define LPC31_CGU_FDC3_OFFSET          0x524 /* Fractional Divider 3 (SYS base) */
#define LPC31_CGU_FDC4_OFFSET          0x528 /* Fractional Divider 4 (SYS base) */
#define LPC31_CGU_FDC5_OFFSET          0x52c /* Fractional Divider 5 (SYS base) */
#define LPC31_CGU_FDC6_OFFSET          0x530 /* Fractional Divider 6 (SYS base) */
#define LPC31_CGU_FDC7_OFFSET          0x534 /* Fractional Divider 7 (AHB0_APB0 base) */
#define LPC31_CGU_FDC8_OFFSET          0x538 /* Fractional Divider 8 (AHB0_APB0 base) */
#define LPC31_CGU_FDC9_OFFSET          0x53c /* Fractional Divider 9 (AHB0_APB1 base) */
#define LPC31_CGU_FDC10_OFFSET         0x540 /* Fractional Divider 10 (AHB0_APB1 base) */
#define LPC31_CGU_FDC11_OFFSET         0x544 /* Fractional Divider 11 (AHB0_APB2 base) */
#define LPC31_CGU_FDC12_OFFSET         0x548 /* Fractional Divider 12 (AHB0_APB2 base) */
#define LPC31_CGU_FDC13_OFFSET         0x54c /* Fractional Divider 13 (AHB0_APB2 base) */
#define LPC31_CGU_FDC14_OFFSET         0x550 /* Fractional Divider 14 (AHB0_APB3 base) */
#define LPC31_CGU_FDC15_OFFSET         0x554 /* Fractional Divider 15 (PCM base) */
#define LPC31_CGU_FDC16_OFFSET         0x558 /* Fractional Divider 16 (UART base) */
#define LPC31_CGU_FDC17_OFFSET         0x55c /* Fractional Divider 17 (CLK1024FS base) */
#define LPC31_CGU_FDC18_OFFSET         0x560 /* Fractional Divider 18 (CLK1024FS base) */
#define LPC31_CGU_FDC19_OFFSET         0x564 /* Fractional Divider 19 (CLK1024FS base) */
#define LPC31_CGU_FDC20_OFFSET         0x568 /* Fractional Divider 20 (CLK1024FS base) */
#define LPC31_CGU_FDC21_OFFSET         0x56c /* Fractional Divider 21 (CLK1024FS base) */
#define LPC31_CGU_FDC22_OFFSET         0x570 /* Fractional Divider 22 (CLK1024FS base) */
#define LPC31_CGU_FDC23_OFFSET         0x574 /* Fractional Divider 23 (SPI_CLK base) */

/* Dynamic fractional divider configuration (DYNFDC) registers (SYS base only) */

#define LPC31_CGU_DYNFDC_OFFSET(n)     (0x578+((n)<<2))
#define LPC31_CGU_DYNFDC0_OFFSET       0x578 /* Fractional Divider 0 (SYS base) */
#define LPC31_CGU_DYNFDC1_OFFSET       0x57c /* Fractional Divider 1 (SYS base) */
#define LPC31_CGU_DYNFDC2_OFFSET       0x580 /* Fractional Divider 2 (SYS base) */
#define LPC31_CGU_DYNFDC3_OFFSET       0x584 /* Fractional Divider 3 (SYS base) */
#define LPC31_CGU_DYNFDC4_OFFSET       0x588 /* Fractional Divider 4 (SYS base) */
#define LPC31_CGU_DYNFDC5_OFFSET       0x58c /* Fractional Divider 5 (SYS base) */
#define LPC31_CGU_DYNFDC6_OFFSET       0x590 /* Fractional Divider 6 (SYS base) */

/* Dynamic fractional divider selection (DYNSEL) registers (SYS base only) */

#define LPC31_CGU_DYNSEL_OFFSET(n)     (0x594+((n)<<2))
#define LPC31_CGU_DYNSEL0_OFFSET       0x594 /* Fractional Divider 0 (SYS base) */
#define LPC31_CGU_DYNSEL1_OFFSET       0x598 /* Fractional Divider 1 (SYS base) */
#define LPC31_CGU_DYNSEL2_OFFSET       0x59c /* Fractional Divider 2 (SYS base) */
#define LPC31_CGU_DYNSEL3_OFFSET       0x5a0 /* Fractional Divider 3 (SYS base) */
#define LPC31_CGU_DYNSEL4_OFFSET       0x5a4 /* Fractional Divider 4 (SYS base) */
#define LPC31_CGU_DYNSEL5_OFFSET       0x5a8 /* Fractional Divider 5 (SYS base) */
#define LPC31_CGU_DYNSEL6_OFFSET       0x5ac /* Fractional Divider 6 (SYS base) */

/* CGU configuration register offsets (with respect to the CGU CFG register base) ***************/
/* Power and oscillator control */

#define LPC31_CGU_POWERMODE_OFFSET     0x000 /* Power mode register */
#define LPC31_CGU_WDBARK_OFFSET        0x004 /* Watch dog bark register */
#define LPC31_CGU_FFASTON_OFFSET       0x008 /* Activate fast oscillator register */
#define LPC31_CGU_FFASTBYP_OFFSET      0x00c /* Bypass comparator register fast oscillator reset */

/* Reset control */

#define LPC31_CGU_SOFTRST_OFFSET(n)    (0x010+((n)<<2))
#define LPC31_CGU_APB0RST_OFFSET       0x010 /* Reset AHB part of AHB_TO_APB0 bridge */
#define LPC31_CGU_AHB2APB0RST_OFFSET   0x014 /* Reset APB part of AHB_TO_APB0 bridge */
#define LPC31_CGU_APB1RST_OFFSET       0x018 /* Reset AHB part of AHB_TO_APB1 bridge */
#define LPC31_CGU_AHB2PB1RST_OFFSET    0x01c /* Reset APB part of AHB_TO_APB1 bridge */
#define LPC31_CGU_APB2RST_OFFSET       0x020 /* Reset AHB part of AHB_TO_APB2 bridge */
#define LPC31_CGU_AHB2APB2RST_OFFSET   0x024 /* Reset APB part of AHB_TO_APB2 bridge */
#define LPC31_CGU_APB3RST_OFFSET       0x028 /* Reset AHB part of AHB_TO_APB3 bridge */
#define LPC31_CGU_AHB2APB3RST_OFFSET   0x02c /* Reset APB part of AHB_TO_APB3 bridge */
#define LPC31_CGU_APB4RST_OFFSET       0x030 /* Reset AHB_TO_APB4 bridge */
#define LPC31_CGU_AHB2INTCRST_OFFSET   0x034 /* Reset AHB_TO_INTC */
#define LPC31_CGU_AHB0RST_OFFSET       0x038 /* Reset AHB0 */
#define LPC31_CGU_EBIRST_OFFSET        0x03c /* Reset EBI */
#define LPC31_CGU_PCMAPBRST_OFFSET     0x040 /* Reset APB domain of PCM */
#define LPC31_CGU_PCMCLKIPRST_OFFSET   0x044 /* Reset synchronous clk_ip domain of PCM */
#define LPC31_CGU_PCMRSTASYNC_OFFSET   0x048 /* Reset asynchronous clk_ip domain of PCM */
#define LPC31_CGU_TIMER0RST_OFFSET     0x04c /* Reset Timer0 */
#define LPC31_CGU_TIMER1RST_OFFSET     0x050 /* Reset Timer1 */
#define LPC31_CGU_TIMER2RST_OFFSET     0x054 /* Reset Timer2 */
#define LPC31_CGU_TIMER3RST_OFFSET     0x058 /* Reset Timer3 */
#define LPC31_CGU_ADCPRST_OFFSET       0x05c /* Reset controller of 10 bit ADC Interface */
#define LPC31_CGU_ADCRST_OFFSET        0x060 /* Reset A/D converter of ADC Interface */
#define LPC31_CGU_PWMRST_OFFSET        0x064 /* Reset PWM */
#define LPC31_CGU_UARTRST_OFFSET       0x068 /* Reset UART/IrDA */
#define LPC31_CGU_I2C0RST_OFFSET       0x06c /* Reset I2C0 */
#define LPC31_CGU_I2C1RST_OFFSET       0x070 /* Reset I2C1 */
#define LPC31_CGU_I2SCFGRST_OFFSET     0x074 /* Reset I2S_Config */
#define LPC31_CGU_I2SNSOFRST_OFFSET    0x078 /* Reset NSOF counter of I2S_CONFIG */
#define LPC31_CGU_EDGEDETRST_OFFSET    0x07c /* Reset Edge_det */
#define LPC31_CGU_I2STXFF0RST_OFFSET   0x080 /* Reset I2STX_FIFO_0 */
#define LPC31_CGU_I2STXIF0RST_OFFSET   0x084 /* Reset I2STX_IF_0 */
#define LPC31_CGU_I2STXFF1RST_OFFSET   0x088 /* Reset I2STX_FIFO_1 */
#define LPC31_CGU_I2STXIF1RST_OFFSET   0x08c /* Reset I2STX_IF_1 */
#define LPC31_CGU_I2SRXFF0RST_OFFSET   0x090 /* Reset I2SRX_FIFO_0 */
#define LPC31_CGU_I2SRXIF0RST_OFFSET   0x094 /* Reset I2SRX_IF_0 */
#define LPC31_CGU_I2SRXFF1RST_OFFSET   0x098 /* Reset I2SRX_FIFO_1 */
#define LPC31_CGU_I2SRXIF1RST_OFFSET   0x09c /* Reset I2SRX_IF_1 */
                                               /* 0x0a0 to 0x0b0: Reserved */
#define LPC31_CGU_LCDRST_OFFSET        0x0b4 /* Reset LCD Interface */
#define LPC31_CGU_SPIRSTAPB_OFFSET     0x0b8 /* Reset apb_clk domain of SPI */
#define LPC31_CGU_SPIRSTIP_OFFSET      0x0bc /* Reset ip_clk domain of SPI */
#define LPC31_CGU_DMARST_OFFSET        0x0c0 /* Reset DMA */
#define LPC31_CGU_NANDECCRST_OFFSET    0x0c4 /* Reset Nandflash Controller ECC clock */
                                               /* 0x0c8: Reserved */
#define LPC31_CGU_NANDCTRLRST_OFFSET   0x0cc /* Reset of Nandflash Controller */
#define LPC31_CGU_RNGRST_OFFSET        0x0d0 /* Reset of RNG */
#define LPC31_CGU_SDMMCRST_OFFSET      0x0d4 /* Reset MCI (on AHB clock) */
#define LPC31_CGU_SDMMCRSTCKIN_OFFSET  0x0d8 /* Reset MCI synchronous (on IP clock) */
#define LPC31_CGU_USBOTGAHBRST_OFFSET  0x0dc /* Reset USB_OTG */
#define LPC31_CGU_REDCTLRST_OFFSET     0x0e0 /* Reset Redundancy Controller */
#define LPC31_CGU_AHBMPMCHRST_OFFSET   0x0e4 /* Reset MPMC */
#define LPC31_CGU_AHBMPMCRFRST_OFFSET  0x0e8 /* Reset refresh generator used for MPMC */
#define LPC31_CGU_INTCRST_OFFSET       0x0ec /* Reset Interrupt Controller */

/* HP PLL controls */

#define LPC313x_CGU_HP0PLL_OFFSET        0x0f0 /* Base offset to HP0 PLL registers */
#define LPC313x_CGU_HP1PLL_OFFSET        0x128 /* Base offset to HP1 PLL registers */
#define CGU_HP0PLL                       0     /* HP0 PLL selector */
#define CGU_HP1PLL                       1     /* HP1 PLL selector */
#define LPC313x_CGU_HPPLL_OFFSET(n)      ((n) ? LPC313x_CGU_HP1PLL_OFFSET : LPC313x_CGU_HP0PLL_OFFSET)

#define LPC31_CGU_HPFINSEL_OFFSET      0x000 /* Register for selecting input to high HPPLL0/1 */
#define LPC31_CGU_HPMDEC_OFFSET        0x004 /* M-divider register of HP0/1 PLL */
#define LPC31_CGU_HPNDEC_OFFSET        0x008 /* N-divider register of HP0/1 PLL */
#define LPC31_CGU_HPPDEC_OFFSET        0x00c /* P-divider register of HP0/1 PLL */
#define LPC31_CGU_HPMODE_OFFSET        0x010 /* Mode register of HP0/1 PLL */
#define LPC31_CGU_HPSTATUS_OFFSET      0x014 /* Status register of HP0/1 PLL */
#define LPC31_CGU_HPACK_OFFSET         0x018 /* Ratio change acknowledge register of HP0/1 PLL */
#define LPC31_CGU_HPREQ_OFFSET         0x01c /* Ratio change request register of HP0/1 PLL */
#define LPC31_CGU_HPINSELR_OFFSET      0x020 /* Bandwidth selection register of HP0/1 PLL */
#define LPC31_CGU_HPINSELI_OFFSET      0x024 /* Bandwidth selection register of HP0/1 PLL */
#define LPC31_CGU_HPINSELP_OFFSET      0x028 /* Bandwidth selection register of HP0/1 PLL */
#define LPC31_CGU_HPSELR_OFFSET        0x02c /* Bandwidth selection register of HP0/1 PLL */
#define LPC31_CGU_HPSELI_OFFSET        0x030 /* Bandwidth selection register of HP0/1 PLL */
#define LPC31_CGU_HPSELP_OFFSET        0x034 /* Bandwidth selection register of HP0/1 PLL */

/* HP0 PLL control (audio PLL) */

#define LPC31_CGU_HP0FINSEL_OFFSET     0x0f0 /* Register for selecting input to high HPPLL0 */
#define LPC31_CGU_HP0MDEC_OFFSET       0x0f4 /* M-divider register of HP0 PLL */
#define LPC31_CGU_HP0NDEC_OFFSET       0x0f8 /* N-divider register of HP0 PLL */
#define LPC31_CGU_HP0PDEC_OFFSET       0x0fc /* P-divider register of HP0 PLL */
#define LPC31_CGU_HP0MODE_OFFSET       0x100 /* Mode register of HP0 PLL */
#define LPC31_CGU_HP0STATUS_OFFSET     0x104 /* Status register of HP0 PLL */
#define LPC31_CGU_HP0ACK_OFFSET        0x108 /* Ratio change acknowledge register of HP0 PLL */
#define LPC31_CGU_HP0REQ_OFFSET        0x10c /* Ratio change request register of HP0 PLL */
#define LPC31_CGU_HP0INSELR_OFFSET     0x110 /* Bandwidth selection register of HP0 PLL */
#define LPC31_CGU_HP0INSELI_OFFSET     0x114 /* Bandwidth selection register of HP0 PLL */
#define LPC31_CGU_HP0INSELP_OFFSET     0x118 /* Bandwidth selection register of HP0 PLL */
#define LPC31_CGU_HP0SELR_OFFSET       0x11c /* Bandwidth selection register of HP0 PLL */
#define LPC31_CGU_HP0SELI_OFFSET       0x120 /* Bandwidth selection register of HP0 PLL */
#define LPC31_CGU_HP0SELP_OFFSET       0x124 /* Bandwidth selection register of HP0 PLL */

/* HP1 PLL control (system PLL) */

#define LPC31_CGU_HP1FINSEL_OFFSET     0x128 /* Register for selecting input to high HP1 PLL */
#define LPC31_CGU_HP1MDEC_OFFSET       0x12c /* M-divider register of HP1 PLL */
#define LPC31_CGU_HP1NDEC_OFFSET       0x130 /* N-divider register of HP1 PLL */
#define LPC31_CGU_HP1PDEC_OFFSET       0x134 /* P-divider register of HP1 PLL */
#define LPC31_CGU_HP1MODE_OFFSET       0x138 /* Mode register of HP1 PLL */
#define LPC31_CGU_HP1STATUS_OFFSET     0x13c /* Status register of HP1 PLL */
#define LPC31_CGU_HP1ACK_OFFSET        0x140 /* Ratio change acknowledge register of HP1 PLL */
#define LPC31_CGU_HP1REQ_OFFSET        0x144 /* Ratio change request register of HP1 PLL */
#define LPC31_CGU_HP1INSELR_OFFSET     0x148 /* Bandwidth selection register of HP1 PLL */
#define LPC31_CGU_HP1INSELI_OFFSET     0x14c /* Bandwidth selection register of HP1 PLL */
#define LPC31_CGU_HP1INSELP_OFFSET     0x150 /* Bandwidth selection register of HP1 PLL */
#define LPC31_CGU_HP1SELR_OFFSET       0x154 /* Bandwidth selection register of HP1 PLL */
#define LPC31_CGU_HP1SELI_OFFSET       0x158 /* Bandwidth selection register of HP1 PLL */
#define LPC31_CGU_HP1SELP_OFFSET       0x15c /* Bandwidth selection register of HP1 PLL */

/* CGU register (virtual) addresses *************************************************************/
/* CGU clock switchbox (virtual) register addresses *********************************************/
/* Switch configuration registers (SCR) for base clocks */

#define LPC31_CGU_SCR(n)               (LPC31_CGU_CSB_VBASE+LPC31_CGU_SCR_OFFSET(n))
#define LPC31_CGU_SCR0                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SCR0_OFFSET)
#define LPC31_CGU_SCR1                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SCR1_OFFSET)
#define LPC31_CGU_SCR2                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SCR2_OFFSET)
#define LPC31_CGU_SCR3                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SCR3_OFFSET)
#define LPC31_CGU_SCR4                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SCR4_OFFSET)
#define LPC31_CGU_SCR5                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SCR5_OFFSET)
#define LPC31_CGU_SCR6                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SCR6_OFFSET)
#define LPC31_CGU_SCR7                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SCR7_OFFSET)
#define LPC31_CGU_SCR8                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SCR8_OFFSET)
#define LPC31_CGU_SCR9                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SCR9_OFFSET)
#define LPC31_CGU_SCR10                (LPC31_CGU_CSB_VBASE+LPC31_CGU_SCR10_OFFSET)
#define LPC31_CGU_SCR11                (LPC31_CGU_CSB_VBASE+LPC31_CGU_SCR11_OFFSET)

/* Frequency select (FS) registers 1 for base clocks */

#define LPC31_CGU_FS1(n)               (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS1_OFFSET(n))
#define LPC31_CGU_FS1_0                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS1_0_OFFSET)
#define LPC31_CGU_FS1_1                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS1_1_OFFSET)
#define LPC31_CGU_FS1_2                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS1_2_OFFSET)
#define LPC31_CGU_FS1_3                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS1_3_OFFSET)
#define LPC31_CGU_FS1_4                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS1_4_OFFSET)
#define LPC31_CGU_FS1_5                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS1_5_OFFSET)
#define LPC31_CGU_FS1_6                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS1_6_OFFSET)
#define LPC31_CGU_FS1_7                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS1_7_OFFSET)
#define LPC31_CGU_FS1_8                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS1_8_OFFSET)
#define LPC31_CGU_FS1_9                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS1_9_OFFSET)
#define LPC31_CGU_FS1_10               (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS1_10_OFFSET)
#define LPC31_CGU_FS1_11               (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS1_11_OFFSET)

/* Frequency select (FS) registers 2 for base clocks */

#define LPC31_CGU_FS2(n)               (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS2_OFFSET(n))
#define LPC31_CGU_FS2_0                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS2_0_OFFSET)
#define LPC31_CGU_FS2_1                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS2_1_OFFSET)
#define LPC31_CGU_FS2_2                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS2_2_OFFSET)
#define LPC31_CGU_FS2_3                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS2_3_OFFSET)
#define LPC31_CGU_FS2_4                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS2_4_OFFSET)
#define LPC31_CGU_FS2_5                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS2_5_OFFSET)
#define LPC31_CGU_FS2_6                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS2_6_OFFSET)
#define LPC31_CGU_FS2_7                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS2_7_OFFSET)
#define LPC31_CGU_FS2_8                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS2_8_OFFSET)
#define LPC31_CGU_FS2_9                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS2_9_OFFSET)
#define LPC31_CGU_FS2_10               (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS2_10_OFFSET)
#define LPC31_CGU_FS2_11               (LPC31_CGU_CSB_VBASE+LPC31_CGU_FS2_11_OFFSET)

/* Switch status registers (SSR) for base clocks */

#define LPC31_CGU_SSR(n)               (LPC31_CGU_CSB_VBASE+LPC31_CGU_SSR_OFFSET(n))
#define LPC31_CGU_SSR0                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SSR0_OFFSET)
#define LPC31_CGU_SSR1                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SSR1_OFFSET)
#define LPC31_CGU_SSR2                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SSR2_OFFSET)
#define LPC31_CGU_SSR3                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SSR3_OFFSET)
#define LPC31_CGU_SSR4                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SSR4_OFFSET)
#define LPC31_CGU_SSR5                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SSR5_OFFSET)
#define LPC31_CGU_SSR6                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SSR6_OFFSET)
#define LPC31_CGU_SSR7                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SSR7_OFFSET)
#define LPC31_CGU_SSR8                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SSR8_OFFSET)
#define LPC31_CGU_SSR9                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_SSR9_OFFSET)
#define LPC31_CGU_SSR10                (LPC31_CGU_CSB_VBASE+LPC31_CGU_SSR10_OFFSET)
#define LPC31_CGU_SSR11                (LPC31_CGU_CSB_VBASE+LPC31_CGU_SSR11_OFFSET)

/* Power control registers (PCR), spreading stage */

#define LPC31_CGU_PCR(n)               (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR_OFFSET(n))
#define LPC31_CGU_PCR0                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR0_OFFSET)
#define LPC31_CGU_PCR1                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR1_OFFSET)
#define LPC31_CGU_PCR2                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR2_OFFSET)
#define LPC31_CGU_PCR3                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR3_OFFSET)
#define LPC31_CGU_PCR4                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR4_OFFSET)
#define LPC31_CGU_PCR5                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR5_OFFSET)
#define LPC31_CGU_PCR6                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR6_OFFSET)
#define LPC31_CGU_PCR7                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR7_OFFSET)
#define LPC31_CGU_PCR8                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR8_OFFSET)
#define LPC31_CGU_PCR9                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR9_OFFSET)
#define LPC31_CGU_PCR10                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR10_OFFSET)
#define LPC31_CGU_PCR11                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR11_OFFSET)
#define LPC31_CGU_PCR12                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR12_OFFSET)
#define LPC31_CGU_PCR13                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR13_OFFSET)
#define LPC31_CGU_PCR14                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR14_OFFSET)
#define LPC31_CGU_PCR15                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR15_OFFSET)
#define LPC31_CGU_PCR16                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR16_OFFSET)
#define LPC31_CGU_PCR17                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR17_OFFSET)
#define LPC31_CGU_PCR18                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR18_OFFSET)
#define LPC31_CGU_PCR19                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR19_OFFSET)
#define LPC31_CGU_PCR20                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR20_OFFSET)
#define LPC31_CGU_PCR21                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR21_OFFSET)
#define LPC31_CGU_PCR22                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR22_OFFSET)
#define LPC31_CGU_PCR23                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR23_OFFSET)
#define LPC31_CGU_PCR24                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR24_OFFSET)
#define LPC31_CGU_PCR25                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR25_OFFSET)
#define LPC31_CGU_PCR26                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR26_OFFSET)
#define LPC31_CGU_PCR27                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR27_OFFSET)
#define LPC31_CGU_PCR28                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR28_OFFSET)
#define LPC31_CGU_PCR29                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR29_OFFSET)
#define LPC31_CGU_PCR30                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR30_OFFSET)
#define LPC31_CGU_PCR31                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR31_OFFSET)
#define LPC31_CGU_PCR32                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR32_OFFSET)
#define LPC31_CGU_PCR33                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR33_OFFSET)
#define LPC31_CGU_PCR34                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR34_OFFSET)
#define LPC31_CGU_PCR35                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR35_OFFSET)
#define LPC31_CGU_PCR36                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR36_OFFSET)
#define LPC31_CGU_PCR37                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR37_OFFSET)
#define LPC31_CGU_PCR38                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR38_OFFSET)
#define LPC31_CGU_PCR39                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR39_OFFSET)
#define LPC31_CGU_PCR40                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR40_OFFSET)
#define LPC31_CGU_PCR41                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR41_OFFSET)
#define LPC31_CGU_PCR42                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR42_OFFSET)
#define LPC31_CGU_PCR43                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR43_OFFSET)
#define LPC31_CGU_PCR44                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR44_OFFSET)
#define LPC31_CGU_PCR45                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR45_OFFSET)
#define LPC31_CGU_PCR46                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR46_OFFSET)
#define LPC31_CGU_PCR47                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR47_OFFSET)
#define LPC31_CGU_PCR48                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR48_OFFSET)
#define LPC31_CGU_PCR49                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR49_OFFSET)
#define LPC31_CGU_PCR50                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR50_OFFSET)
#define LPC31_CGU_PCR51                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR51_OFFSET)
#define LPC31_CGU_PCR52                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR52_OFFSET)
#define LPC31_CGU_PCR53                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR53_OFFSET)
#define LPC31_CGU_PCR54                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR54_OFFSET)
#define LPC31_CGU_PCR55                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR55_OFFSET)
#define LPC31_CGU_PCR56                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR56_OFFSET)
#define LPC31_CGU_PCR57                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR57_OFFSET)
#define LPC31_CGU_PCR58                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR58_OFFSET)
#define LPC31_CGU_PCR59                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR59_OFFSET)
#define LPC31_CGU_PCR60                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR60_OFFSET)
#define LPC31_CGU_PCR61                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR61_OFFSET)
#define LPC31_CGU_PCR62                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR62_OFFSET)
#define LPC31_CGU_PCR63                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR63_OFFSET)
#define LPC31_CGU_PCR64                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR64_OFFSET)
#define LPC31_CGU_PCR65                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR65_OFFSET)
#define LPC31_CGU_PCR66                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR66_OFFSET)
#define LPC31_CGU_PCR67                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR67_OFFSET)
#define LPC31_CGU_PCR68                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR68_OFFSET)
#define LPC31_CGU_PCR69                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR69_OFFSET)
#define LPC31_CGU_PCR70                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR70_OFFSET)
#define LPC31_CGU_PCR71                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR71_OFFSET)
#define LPC31_CGU_PCR72                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR72_OFFSET)
#define LPC31_CGU_PCR73                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR73_OFFSET)
#define LPC31_CGU_PCR74                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR74_OFFSET)
#define LPC31_CGU_PCR75                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR75_OFFSET)
#define LPC31_CGU_PCR76                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR76_OFFSET)
#define LPC31_CGU_PCR77                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR77_OFFSET)
#define LPC31_CGU_PCR78                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR78_OFFSET)
#define LPC31_CGU_PCR79                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR79_OFFSET)
#define LPC31_CGU_PCR80                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR80_OFFSET)
#define LPC31_CGU_PCR81                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR81_OFFSET)
#define LPC31_CGU_PCR82                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR82_OFFSET)
#define LPC31_CGU_PCR83                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR83_OFFSET)
#define LPC31_CGU_PCR84                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR84_OFFSET)
#define LPC31_CGU_PCR85                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR85_OFFSET)
#define LPC31_CGU_PCR86                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR86_OFFSET)
#define LPC31_CGU_PCR87                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR87_OFFSET)
#define LPC31_CGU_PCR88                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR88_OFFSET)
#define LPC31_CGU_PCR89                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR89_OFFSET)
#define LPC31_CGU_PCR90                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR90_OFFSET)
#define LPC31_CGU_PCR91                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PCR91_OFFSET)

/* Power status registers (PSR), spreading stage */

#define LPC31_CGU_PSR(n)               (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR_OFFSET(n))
#define LPC31_CGU_PSR0                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR0_OFFSET)
#define LPC31_CGU_PSR1                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR1_OFFSET)
#define LPC31_CGU_PSR2                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR2_OFFSET)
#define LPC31_CGU_PSR3                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR3_OFFSET)
#define LPC31_CGU_PSR4                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR4_OFFSET)
#define LPC31_CGU_PSR5                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR5_OFFSET)
#define LPC31_CGU_PSR6                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR6_OFFSET)
#define LPC31_CGU_PSR7                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR7_OFFSET)
#define LPC31_CGU_PSR8                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR8_OFFSET)
#define LPC31_CGU_PSR9                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR9_OFFSET)
#define LPC31_CGU_PSR10                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR10_OFFSET)
#define LPC31_CGU_PSR11                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR11_OFFSET)
#define LPC31_CGU_PSR12                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR12_OFFSET)
#define LPC31_CGU_PSR13                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR13_OFFSET)
#define LPC31_CGU_PSR14                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR14_OFFSET)
#define LPC31_CGU_PSR15                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR15_OFFSET)
#define LPC31_CGU_PSR16                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR16_OFFSET)
#define LPC31_CGU_PSR17                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR17_OFFSET)
#define LPC31_CGU_PSR18                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR18_OFFSET)
#define LPC31_CGU_PSR19                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR19_OFFSET)
#define LPC31_CGU_PSR20                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR20_OFFSET)
#define LPC31_CGU_PSR21                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR21_OFFSET)
#define LPC31_CGU_PSR22                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR22_OFFSET)
#define LPC31_CGU_PSR23                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR23_OFFSET)
#define LPC31_CGU_PSR24                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR24_OFFSET)
#define LPC31_CGU_PSR25                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR25_OFFSET)
#define LPC31_CGU_PSR26                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR26_OFFSET)
#define LPC31_CGU_PSR27                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR27_OFFSET)
#define LPC31_CGU_PSR28                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR28_OFFSET)
#define LPC31_CGU_PSR29                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR29_OFFSET)
#define LPC31_CGU_PSR30                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR30_OFFSET)
#define LPC31_CGU_PSR31                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR31_OFFSET)
#define LPC31_CGU_PSR32                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR32_OFFSET)
#define LPC31_CGU_PSR33                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR33_OFFSET)
#define LPC31_CGU_PSR34                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR34_OFFSET)
#define LPC31_CGU_PSR35                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR35_OFFSET)
#define LPC31_CGU_PSR36                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR36_OFFSET)
#define LPC31_CGU_PSR37                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR37_OFFSET)
#define LPC31_CGU_PSR38                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR38_OFFSET)
#define LPC31_CGU_PSR39                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR39_OFFSET)
#define LPC31_CGU_PSR40                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR40_OFFSET)
#define LPC31_CGU_PSR41                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR41_OFFSET)
#define LPC31_CGU_PSR42                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR42_OFFSET)
#define LPC31_CGU_PSR43                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR43_OFFSET)
#define LPC31_CGU_PSR44                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR44_OFFSET)
#define LPC31_CGU_PSR45                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR45_OFFSET)
#define LPC31_CGU_PSR46                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR46_OFFSET)
#define LPC31_CGU_PSR47                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR47_OFFSET)
#define LPC31_CGU_PSR48                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR48_OFFSET)
#define LPC31_CGU_PSR49                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR49_OFFSET)
#define LPC31_CGU_PSR50                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR50_OFFSET)
#define LPC31_CGU_PSR51                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR51_OFFSET)
#define LPC31_CGU_PSR52                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR52_OFFSET)
#define LPC31_CGU_PSR53                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR53_OFFSET)
#define LPC31_CGU_PSR54                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR54_OFFSET)
#define LPC31_CGU_PSR55                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR55_OFFSET)
#define LPC31_CGU_PSR56                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR56_OFFSET)
#define LPC31_CGU_PSR57                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR57_OFFSET)
#define LPC31_CGU_PSR58                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR58_OFFSET)
#define LPC31_CGU_PSR59                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR59_OFFSET)
#define LPC31_CGU_PSR60                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR60_OFFSET)
#define LPC31_CGU_PSR61                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR61_OFFSET)
#define LPC31_CGU_PSR62                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR62_OFFSET)
#define LPC31_CGU_PSR63                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR63_OFFSET)
#define LPC31_CGU_PSR64                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR64_OFFSET)
#define LPC31_CGU_PSR65                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR65_OFFSET)
#define LPC31_CGU_PSR66                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR66_OFFSET)
#define LPC31_CGU_PSR67                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR67_OFFSET)
#define LPC31_CGU_PSR68                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR68_OFFSET)
#define LPC31_CGU_PSR69                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR69_OFFSET)
#define LPC31_CGU_PSR70                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR70_OFFSET)
#define LPC31_CGU_PSR71                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR71_OFFSET)
#define LPC31_CGU_PSR72                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR72_OFFSET)
#define LPC31_CGU_PSR73                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR73_OFFSET)
#define LPC31_CGU_PSR74                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR74_OFFSET)
#define LPC31_CGU_PSR75                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR75_OFFSET)
#define LPC31_CGU_PSR76                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR76_OFFSET)
#define LPC31_CGU_PSR77                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR77_OFFSET)
#define LPC31_CGU_PSR78                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR78_OFFSET)
#define LPC31_CGU_PSR79                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR79_OFFSET)
#define LPC31_CGU_PSR80                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR80_OFFSET)
#define LPC31_CGU_PSR81                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR81_OFFSET)
#define LPC31_CGU_PSR82                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR82_OFFSET)
#define LPC31_CGU_PSR83                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR83_OFFSET)
#define LPC31_CGU_PSR84                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR84_OFFSET)
#define LPC31_CGU_PSR85                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR85_OFFSET)
#define LPC31_CGU_PSR86                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR86_OFFSET)
#define LPC31_CGU_PSR87                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR87_OFFSET)
#define LPC31_CGU_PSR88                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR88_OFFSET)
#define LPC31_CGU_PSR89                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR89_OFFSET)
#define LPC31_CGU_PSR90                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR90_OFFSET)
#define LPC31_CGU_PSR91                (LPC31_CGU_CSB_VBASE+LPC31_CGU_PSR91_OFFSET)

/* Enable select registers (ESR), spreading stage */

#define LPC31_CGU_ESR(n)               (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR_OFFSET(n))
#define LPC31_CGU_ESR0                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR0_OFFSET)
#define LPC31_CGU_ESR1                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR1_OFFSET)
#define LPC31_CGU_ESR2                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR2_OFFSET)
#define LPC31_CGU_ESR3                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR3_OFFSET)
#define LPC31_CGU_ESR4                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR4_OFFSET)
#define LPC31_CGU_ESR5                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR5_OFFSET)
#define LPC31_CGU_ESR6                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR6_OFFSET)
#define LPC31_CGU_ESR7                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR7_OFFSET)
#define LPC31_CGU_ESR8                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR8_OFFSET)
#define LPC31_CGU_ESR9                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR9_OFFSET)
#define LPC31_CGU_ESR10                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR10_OFFSET)
#define LPC31_CGU_ESR11                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR11_OFFSET)
#define LPC31_CGU_ESR12                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR12_OFFSET)
#define LPC31_CGU_ESR13                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR13_OFFSET)
#define LPC31_CGU_ESR14                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR14_OFFSET)
#define LPC31_CGU_ESR15                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR15_OFFSET)
#define LPC31_CGU_ESR16                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR16_OFFSET)
#define LPC31_CGU_ESR17                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR17_OFFSET)
#define LPC31_CGU_ESR18                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR18_OFFSET)
#define LPC31_CGU_ESR19                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR19_OFFSET)
#define LPC31_CGU_ESR20                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR20_OFFSET)
#define LPC31_CGU_ESR21                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR21_OFFSET)
#define LPC31_CGU_ESR22                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR22_OFFSET)
#define LPC31_CGU_ESR23                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR23_OFFSET)
#define LPC31_CGU_ESR24                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR24_OFFSET)
#define LPC31_CGU_ESR25                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR25_OFFSET)
#define LPC31_CGU_ESR26                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR26_OFFSET)
#define LPC31_CGU_ESR27                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR27_OFFSET)
#define LPC31_CGU_ESR28                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR28_OFFSET)
#define LPC31_CGU_ESR29                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR29_OFFSET)
#define LPC31_CGU_ESR30                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR30_OFFSET)
#define LPC31_CGU_ESR31                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR31_OFFSET)
#define LPC31_CGU_ESR32                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR32_OFFSET)
#define LPC31_CGU_ESR33                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR33_OFFSET)
#define LPC31_CGU_ESR34                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR34_OFFSET)
#define LPC31_CGU_ESR35                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR35_OFFSET)
#define LPC31_CGU_ESR36                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR36_OFFSET)
#define LPC31_CGU_ESR37                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR37_OFFSET)
#define LPC31_CGU_ESR38                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR38_OFFSET)
#define LPC31_CGU_ESR39                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR39_OFFSET)
#define LPC31_CGU_ESR40                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR40_OFFSET)
#define LPC31_CGU_ESR41                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR41_OFFSET)
#define LPC31_CGU_ESR42                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR42_OFFSET)
#define LPC31_CGU_ESR43                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR43_OFFSET)
#define LPC31_CGU_ESR44                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR44_OFFSET)
#define LPC31_CGU_ESR45                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR45_OFFSET)
#define LPC31_CGU_ESR46                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR46_OFFSET)
#define LPC31_CGU_ESR47                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR47_OFFSET)
#define LPC31_CGU_ESR48                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR48_OFFSET)
#define LPC31_CGU_ESR49                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR49_OFFSET)
#define LPC31_CGU_ESR50                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR50_OFFSET)
#define LPC31_CGU_ESR51                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR51_OFFSET)
#define LPC31_CGU_ESR52                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR52_OFFSET)
#define LPC31_CGU_ESR53                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR53_OFFSET)
#define LPC31_CGU_ESR54                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR54_OFFSET)
#define LPC31_CGU_ESR55                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR55_OFFSET)
#define LPC31_CGU_ESR56                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR56_OFFSET)
#define LPC31_CGU_ESR57                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR57_OFFSET)
#define LPC31_CGU_ESR58                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR58_OFFSET)
#define LPC31_CGU_ESR59                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR59_OFFSET)
#define LPC31_CGU_ESR60                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR60_OFFSET)
#define LPC31_CGU_ESR61                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR61_OFFSET)
#define LPC31_CGU_ESR62                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR62_OFFSET)
#define LPC31_CGU_ESR63                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR63_OFFSET)
#define LPC31_CGU_ESR64                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR64_OFFSET)
#define LPC31_CGU_ESR65                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR65_OFFSET)
#define LPC31_CGU_ESR66                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR66_OFFSET)
#define LPC31_CGU_ESR67                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR67_OFFSET)
#define LPC31_CGU_ESR68                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR68_OFFSET)
#define LPC31_CGU_ESR69                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR69_OFFSET)
#define LPC31_CGU_ESR70                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR70_OFFSET)
#define LPC31_CGU_ESR71                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR71_OFFSET)
#define LPC31_CGU_ESR72                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR72_OFFSET)
#define LPC31_CGU_ESR73                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR73_OFFSET)
#define LPC31_CGU_ESR74                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR74_OFFSET)
#define LPC31_CGU_ESR75                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR75_OFFSET)
#define LPC31_CGU_ESR76                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR76_OFFSET)
#define LPC31_CGU_ESR77                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR77_OFFSET)
#define LPC31_CGU_ESR78                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR78_OFFSET)
#define LPC31_CGU_ESR79                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR79_OFFSET)
#define LPC31_CGU_ESR80                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR80_OFFSET)
#define LPC31_CGU_ESR81                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR81_OFFSET)
#define LPC31_CGU_ESR82                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR82_OFFSET)
#define LPC31_CGU_ESR83                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR83_OFFSET)
#define LPC31_CGU_ESR84                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR84_OFFSET)
#define LPC31_CGU_ESR85                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR85_OFFSET)
#define LPC31_CGU_ESR86                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR86_OFFSET)
#define LPC31_CGU_ESR87                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR87_OFFSET)
#define LPC31_CGU_ESR88                (LPC31_CGU_CSB_VBASE+LPC31_CGU_ESR88_OFFSET)

/* Base control registers (BCR) for SYS base */

#define LPC31_CGU_BCR(n)               (LPC31_CGU_CSB_VBASE+LPC31_CGU_BCR_OFFSET(n))
#define LPC31_CGU_BCR0                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_BCR0_OFFSET)
#define LPC31_CGU_BCR1                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_BCR1_OFFSET)
#define LPC31_CGU_BCR2                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_BCR2_OFFSET)
#define LPC31_CGU_BCR3                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_BCR3_OFFSET)
#define LPC31_CGU_BCR7                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_BCR7_OFFSET)

/* Fractional divider configuration (FDC) registers */

#define LPC31_CGU_FDC(n)               (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC_OFFSET(n))
#define LPC31_CGU_FDC0                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC0_OFFSET)
#define LPC31_CGU_FDC1                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC1_OFFSET)
#define LPC31_CGU_FDC2                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC2_OFFSET)
#define LPC31_CGU_FDC3                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC3_OFFSET)
#define LPC31_CGU_FDC4                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC4_OFFSET)
#define LPC31_CGU_FDC5                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC5_OFFSET)
#define LPC31_CGU_FDC6                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC6_OFFSET)
#define LPC31_CGU_FDC7                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC7_OFFSET)
#define LPC31_CGU_FDC8                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC8_OFFSET)
#define LPC31_CGU_FDC9                 (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC9_OFFSET)
#define LPC31_CGU_FDC10                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC10_OFFSET)
#define LPC31_CGU_FDC11                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC11_OFFSET)
#define LPC31_CGU_FDC12                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC12_OFFSET)
#define LPC31_CGU_FDC13                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC13_OFFSET)
#define LPC31_CGU_FDC14                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC14_OFFSET)
#define LPC31_CGU_FDC15                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC15_OFFSET)
#define LPC31_CGU_FDC16                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC16_OFFSET)
#define LPC31_CGU_FDC17                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC17_OFFSET)
#define LPC31_CGU_FDC18                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC18_OFFSET)
#define LPC31_CGU_FDC19                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC19_OFFSET)
#define LPC31_CGU_FDC20                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC20_OFFSET)
#define LPC31_CGU_FDC21                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC21_OFFSET)
#define LPC31_CGU_FDC22                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC22_OFFSET)
#define LPC31_CGU_FDC23                (LPC31_CGU_CSB_VBASE+LPC31_CGU_FDC23_OFFSET)

/* Dynamic fractional divider configuration (DYNFDC) registers (SYS base only) */

#define LPC31_CGU_DYNFDC(n)            (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNFDC_OFFSET(n))
#define LPC31_CGU_DYNFDC0              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNFDC0_OFFSET)
#define LPC31_CGU_DYNFDC1              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNFDC1_OFFSET)
#define LPC31_CGU_DYNFDC2              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNFDC2_OFFSET)
#define LPC31_CGU_DYNFDC3              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNFDC3_OFFSET)
#define LPC31_CGU_DYNFDC4              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNFDC4_OFFSET)
#define LPC31_CGU_DYNFDC5              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNFDC5_OFFSET)
#define LPC31_CGU_DYNFDC6              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNFDC6_OFFSET)

/* Dynamic fractional divider selection (DYNSEL) registers (SYS base only) */

#define LPC31_CGU_DYNSEL(n)            (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNSEL_OFFSET(n))
#define LPC31_CGU_DYNSEL0              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNSEL0_OFFSET)
#define LPC31_CGU_DYNSEL1              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNSEL1_OFFSET)
#define LPC31_CGU_DYNSEL2              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNSEL2_OFFSET)
#define LPC31_CGU_DYNSEL3              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNSEL3_OFFSET)
#define LPC31_CGU_DYNSEL4              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNSEL4_OFFSET)
#define LPC31_CGU_DYNSEL5              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNSEL5_OFFSET)
#define LPC31_CGU_DYNSEL6              (LPC31_CGU_CSB_VBASE+LPC31_CGU_DYNSEL6_OFFSET)

/* CGU configuration (virtural) register address ************************************************/
/* Power and oscillator control */

#define LPC31_CGU_POWERMODE            (LPC31_CGU_CFG_VBASE+LPC31_CGU_POWERMODE_OFFSET)
#define LPC31_CGU_WDBARK               (LPC31_CGU_CFG_VBASE+LPC31_CGU_WDBARK_OFFSET)
#define LPC31_CGU_FFASTON              (LPC31_CGU_CFG_VBASE+LPC31_CGU_FFASTON_OFFSET)
#define LPC31_CGU_FFASTBYP             (LPC31_CGU_CFG_VBASE+LPC31_CGU_FFASTBYP_OFFSET)

/* Reset control */

#define LPC31_CGU_SOFTRST(n)           (LPC31_CGU_CFG_VBASE+LPC31_CGU_SOFTRST_OFFSET(n))
#define LPC31_CGU_APB0RST              (LPC31_CGU_CFG_VBASE+LPC31_CGU_APB0RST_OFFSET)
#define LPC31_CGU_AHB2APB0RST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_AHB2APB0RST_OFFSET)
#define LPC31_CGU_APB1RST              (LPC31_CGU_CFG_VBASE+LPC31_CGU_APB1RST_OFFSET)
#define LPC31_CGU_AHB2PB1RST           (LPC31_CGU_CFG_VBASE+LPC31_CGU_AHB2PB1RST_OFFSET)
#define LPC31_CGU_APB2RST              (LPC31_CGU_CFG_VBASE+LPC31_CGU_APB2RST_OFFSET)
#define LPC31_CGU_AHB2APB2RST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_AHB2APB2RST_OFFSET)
#define LPC31_CGU_APB3RST              (LPC31_CGU_CFG_VBASE+LPC31_CGU_APB3RST_OFFSET)
#define LPC31_CGU_AHB2APB3RST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_AHB2APB3RST_OFFSET)
#define LPC31_CGU_APB4RST              (LPC31_CGU_CFG_VBASE+LPC31_CGU_APB4RST_OFFSET)
#define LPC31_CGU_AHB2INTCRST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_AHB2INTCRST_OFFSET)
#define LPC31_CGU_AHB0RST              (LPC31_CGU_CFG_VBASE+LPC31_CGU_AHB0RST_OFFSET)
#define LPC31_CGU_EBIRST               (LPC31_CGU_CFG_VBASE+LPC31_CGU_EBIRST_OFFSET)
#define LPC31_CGU_PCMAPBRST            (LPC31_CGU_CFG_VBASE+LPC31_CGU_PCMAPBRST_OFFSET)
#define LPC31_CGU_PCMCLKIPRST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_PCMCLKIPRST_OFFSET)
#define LPC31_CGU_PCMRSTASYNC          (LPC31_CGU_CFG_VBASE+LPC31_CGU_PCMRSTASYNC_OFFSET)
#define LPC31_CGU_TIMER0RST            (LPC31_CGU_CFG_VBASE+LPC31_CGU_TIMER0RST_OFFSET)
#define LPC31_CGU_TIMER1RST            (LPC31_CGU_CFG_VBASE+LPC31_CGU_TIMER1RST_OFFSET)
#define LPC31_CGU_TIMER2RST            (LPC31_CGU_CFG_VBASE+LPC31_CGU_TIMER2RST_OFFSET)
#define LPC31_CGU_TIMER3RST            (LPC31_CGU_CFG_VBASE+LPC31_CGU_TIMER3RST_OFFSET)
#define LPC31_CGU_ADCPRST              (LPC31_CGU_CFG_VBASE+LPC31_CGU_ADCPRST_OFFSET)
#define LPC31_CGU_ADCRST               (LPC31_CGU_CFG_VBASE+LPC31_CGU_ADCRST_OFFSET)
#define LPC31_CGU_PWMRST               (LPC31_CGU_CFG_VBASE+LPC31_CGU_PWMRST_OFFSET)
#define LPC31_CGU_UARTRST              (LPC31_CGU_CFG_VBASE+LPC31_CGU_UARTRST_OFFSET)
#define LPC31_CGU_I2C0RST              (LPC31_CGU_CFG_VBASE+LPC31_CGU_I2C0RST_OFFSET)
#define LPC31_CGU_I2C1RST              (LPC31_CGU_CFG_VBASE+LPC31_CGU_I2C1RST_OFFSET)
#define LPC31_CGU_I2SCFGRST            (LPC31_CGU_CFG_VBASE+LPC31_CGU_I2SCFGRST_OFFSET)
#define LPC31_CGU_I2SNSOFRST           (LPC31_CGU_CFG_VBASE+LPC31_CGU_I2SNSOFRST_OFFSET)
#define LPC31_CGU_EDGEDETRST           (LPC31_CGU_CFG_VBASE+LPC31_CGU_EDGEDETRST_OFFSET)
#define LPC31_CGU_I2STXFF0RST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_I2STXFF0RST_OFFSET)
#define LPC31_CGU_I2STXIF0RST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_I2STXIF0RST_OFFSET)
#define LPC31_CGU_I2STXFF1RST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_I2STXFF1RST_OFFSET)
#define LPC31_CGU_I2STXIF1RST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_I2STXIF1RST_OFFSET)
#define LPC31_CGU_I2SRXFF0RST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_I2SRXFF0RST_OFFSET)
#define LPC31_CGU_I2SRXIF0RST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_I2SRXIF0RST_OFFSET)
#define LPC31_CGU_I2SRXFF1RST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_I2SRXFF1RST_OFFSET)
#define LPC31_CGU_I2SRXIF1RST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_I2SRXIF1RST_OFFSET)
#define LPC31_CGU_LCDRST               (LPC31_CGU_CFG_VBASE+LPC31_CGU_LCDRST_OFFSET)
#define LPC31_CGU_SPIRSTAPB            (LPC31_CGU_CFG_VBASE+LPC31_CGU_SPIRSTAPB_OFFSET)
#define LPC31_CGU_SPIRSTIP             (LPC31_CGU_CFG_VBASE+LPC31_CGU_SPIRSTIP_OFFSET)
#define LPC31_CGU_DMARST               (LPC31_CGU_CFG_VBASE+LPC31_CGU_DMARST_OFFSET)
#define LPC31_CGU_NANDECCRST           (LPC31_CGU_CFG_VBASE+LPC31_CGU_NANDECCRST_OFFSET)
#define LPC31_CGU_NANDCTRLRST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_NANDCTRLRST_OFFSET)
#define LPC31_CGU_SDMMCRST             (LPC31_CGU_CFG_VBASE+LPC31_CGU_SDMMCRST_OFFSET)
#define LPC31_CGU_SDMMCRSTCKIN         (LPC31_CGU_CFG_VBASE+LPC31_CGU_SDMMCRSTCKIN_OFFSET)
#define LPC31_CGU_USBOTGAHBRST         (LPC31_CGU_CFG_VBASE+LPC31_CGU_USBOTGAHBRST_OFFSET)
#define LPC31_CGU_REDCTLRST            (LPC31_CGU_CFG_VBASE+LPC31_CGU_REDCTLRST_OFFSET)
#define LPC31_CGU_AHBMPMCHRST          (LPC31_CGU_CFG_VBASE+LPC31_CGU_AHBMPMCHRST_OFFSET)
#define LPC31_CGU_AHBMPMCRFRST         (LPC31_CGU_CFG_VBASE+LPC31_CGU_AHBMPMCRFRST_OFFSET)
#define LPC31_CGU_INTCRST              (LPC31_CGU_CFG_VBASE+LPC31_CGU_INTCRST_OFFSET)

/* HP PLL controls */

#define LPC313x_CGU_HP0PLL               (LPC31_CGU_CFG_VBASE+LPC313x_CGU_HP0PLL_OFFSET)
#define LPC313x_CGU_HP1PLL               (LPC31_CGU_CFG_VBASE+LPC313x_CGU_HP1PLL_OFFSET)
#define LPC313x_CGU_HPPLL(n)             ((n) ? LPC313x_CGU_HP1PLL : LPC313x_CGU_HP0PLL)

/* HPO PLL control (audio PLL) */

#define LPC31_CGU_HP0FINSEL            (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0FINSEL_OFFSET)
#define LPC31_CGU_HP0MDEC              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0MDEC_OFFSET)
#define LPC31_CGU_HP0NDEC              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0NDEC_OFFSET)
#define LPC31_CGU_HP0PDEC              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0PDEC_OFFSET)
#define LPC31_CGU_HP0MODE              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0MODE_OFFSET)
#define LPC31_CGU_HP0STATUS            (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0STATUS_OFFSET)
#define LPC31_CGU_HP0ACK               (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0ACK_OFFSET)
#define LPC31_CGU_HP0REQ               (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0REQ_OFFSET)
#define LPC31_CGU_HP0INSELR            (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0INSELR_OFFSET)
#define LPC31_CGU_HP0INSELI            (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0INSELI_OFFSET)
#define LPC31_CGU_HP0INSELP            (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0INSELP_OFFSET)
#define LPC31_CGU_HP0SELR              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0SELR_OFFSET)
#define LPC31_CGU_HP0SELI              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0SELI_OFFSET)
#define LPC31_CGU_HP0SELP              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP0SELP_OFFSET)

/* HP1 PLL control (system PLL) */

#define LPC31_CGU_HP1FINSEL            (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1FINSEL_OFFSET)
#define LPC31_CGU_HP1MDEC              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1MDEC_OFFSET)
#define LPC31_CGU_HP1NDEC              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1NDEC_OFFSET)
#define LPC31_CGU_HP1PDEC              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1PDEC_OFFSET)
#define LPC31_CGU_HP1MODE              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1MODE_OFFSET)
#define LPC31_CGU_HP1STATUS            (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1STATUS_OFFSET)
#define LPC31_CGU_HP1ACK               (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1ACK_OFFSET)
#define LPC31_CGU_HP1REQ               (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1REQ_OFFSET)
#define LPC31_CGU_HP1INSELR            (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1INSELR_OFFSET)
#define LPC31_CGU_HP1INSELI            (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1INSELI_OFFSET)
#define LPC31_CGU_HP1INSELP            (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1INSELP_OFFSET)
#define LPC31_CGU_HP1SELR              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1SELR_OFFSET)
#define LPC31_CGU_HP1SELI              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1SELI_OFFSET)
#define LPC31_CGU_HP1SELP              (LPC31_CGU_CFG_VBASE+LPC31_CGU_HP1SELP_OFFSET)

/* CGU register bit definitions *****************************************************************/

/* Frequency inputs */

#define CGU_FREQIN_FFAST                 0 /* ffast 12 MHz */
#define CGU_FREQIN_I2SRXBCK0             1 /* I2SRX_BCK0 */
#define CGU_FREQIN_I2SRXWS0              2 /* I2SRX_WS0 */
#define CGU_FREQIN_I2SRXBCK1             3 /* I2SRX_BCK1 */
#define CGU_FREQIN_I2SRXWS1              4 /* I2SRX_WS1 */
#define CGU_FREQIN_HPPLL0                5 /* HPPLL0 (Audio/I2S PLL) */
#define CGU_FREQIN_HPPLL1                6 /* HPPLL1 (System PLL) */
#define CGU_NFREQIN                      7 

/* CGU clock switchbox register bit definitions *************************************************/

/* Switch configuration register SCR0 to SCR11, addresses 0x13004000 to 0x1300402c */

#define CGU_SCR_STOP                     (1 << 3)  /* Bit 3:  Forces switch in disable mode */
#define CGU_SCR_RESET                    (1 << 2)  /* Bit 2:  Asynchronous reset of both switches */
#define CGU_SCR_ENF2                     (1 << 1)  /* Bit 1:  Enable side #2 of switch */
#define CGU_SCR_ENF1                     (1 << 0)  /* Bit 0:  Enable side #1 of switch */

/* Frequency select register 1 FS1_0 to FS1_11, addresses 0x13004030 to 0x1300405c,
 * Frequency Select register 2 FS2_0 to FS2_11, addresses 0x13004060 to 0x1300408c
 */

#define CGU_FS_SHIFT                     (0)      /* Bits 0-2: Selects input frequency for either side of frequency switch */
#define CGU_FS_MASK                      (7 << CGU_FS_SHIFT)
#  define CGU_FS_FFAST                   (CGU_FREQIN_FFAST     << CGU_FS_SHIFT) /* ffast 12 MHz */
#  define CGU_FS_I2SRXBCK0               (CGU_FREQIN_I2SRXBCK0 << CGU_FS_SHIFT) /* I2SRX_BCK0 */
#  define CGU_FS_I2SRXWS0                (CGU_FREQIN_I2SRXWS0  << CGU_FS_SHIFT) /* I2SRX_WS0 */
#  define CGU_FS_I2SRXBCK1               (CGU_FREQIN_I2SRXBCK1 << CGU_FS_SHIFT) /* I2SRX_BCK1 */
#  define CGU_FS_I2SRXWS1                (CGU_FREQIN_I2SRXWS1  << CGU_FS_SHIFT) /* I2SRX_WS1 */
#  define CGU_FS_HPPLL0                  (CGU_FREQIN_HPPLL0    << CGU_FS_SHIFT) /* HPPLL0 (Audio/I2S PLL) */
#  define CGU_FS_HPPLL1                  (CGU_FREQIN_HPPLL1    << CGU_FS_SHIFT) /* HPPLL1 (System PLL) */

/* Switch Status register SSR0 to SSR11, addresses 0x13004090 to 0x1300 40bc */

#define CGU_SSR_FS_SHIFT                 (2)       /* Bits 2-4: Feedback of current frequency selection */
#define CGU_SSR_FS_MASK                  (7 << CGU_SSR_FS_SHIFT)
#  define CGU_SSR_FFAST                  (CGU_FREQIN_FFAST     << CGU_SSR_FS_SHIFT) /* ffast 12 MHz */
#  define CGU_SSR_I2SRXBCK0              (CGU_FREQIN_I2SRXBCK0 << CGU_SSR_FS_SHIFT) /* I2SRX_BCK0 */
#  define CGU_SSR_I2SRXWS0               (CGU_FREQIN_I2SRXWS0  << CGU_SSR_FS_SHIFT) /* I2SRX_WS0 */
#  define CGU_SSR_I2SRXBCK1              (CGU_FREQIN_I2SRXBCK1 << CGU_SSR_FS_SHIFT) /* I2SRX_BCK1 */
#  define CGU_SSR_I2SRXWS1               (CGU_FREQIN_I2SRXWS1  << CGU_SSR_FS_SHIFT) /* I2SRX_WS1 */
#  define CGU_SSR_HPPLL0                 (CGU_FREQIN_HPPLL0    << CGU_SSR_FS_SHIFT) /* HPPLL0 (Audio/I2S PLL) */
#  define CGU_SSR_HPPLL1                 (CGU_FREQIN_HPPLL1    << CGU_SSR_FS_SHIFT) /* HPPLL1 (System PLL) */
#define CGU_SSR_FS2STAT                  (1 << 1)  /* Bit 1:  Enable side #2 of the frequency switch */
#define CGU_SSR_FS1STAT                  (1 << 0)  /* Bit 0:  Enable side #1 of the frequency switch */

/* Power Control register PCR0 to PCR91, addresses 0x130040c0 to 0x1300422c */

#define CGU_PCR_ENOUTEN                  (1 << 4)  /* Bit 4:  Enable clock preview signal */
#define CGU_PCR_EXTENEN                  (1 << 3)  /* Bit 3:  Enable external enabling */
#define CGU_PCR_WAKEEN                   (1 << 2)  /* Bit 2:  Enable central wakeup */
#define CGU_PCR_AUTO                     (1 << 1)  /* Bit 1:  Enable wakeup and external enable */
#define CGU_PCR_RUN                      (1 << 0)  /* Bit 0:  Enable clocks */

/* Power Status register PSR0 to PSR91, addresses 0x13004230 to 0x1300439c */

#define CGU_PSR_WAKEUP                   (1 << 1)  /* Bit 1:  Clock wakeup condition */
#define CGU_PSR_ACTIVE                   (1 << 0)  /* Bit 0:  Indicates clock is active */

/* Enable Select register ESR0 to ESR88, addresses 0x130043a0 to 0x13004500 */
/* The ESR_SEL varies according to the selected clock */

#define CGU_ESR_ESRSEL_SHIFT             (1)       /* Bits 1-n: Common shift value */
#define CGU_ESR0_29_ESRSEL_SHIFT         (1)       /* Bits 1-3: Selection of fractional dividers */
#define CGU_ESR0_29_ESRSEL_MASK          (7 << CGU_ESR0_29_ESRSEL_SHIFT)
#  define CGU_ESR0_29_ESRSEL_FDC0        (0 << CGU_ESR0_29_ESRSEL_SHIFT) /* Selects FDC0 */
#  define CGU_ESR0_29_ESRSEL_FDC1        (1 << CGU_ESR0_29_ESRSEL_SHIFT) /* Selects FDC1 */
#  define CGU_ESR0_29_ESRSEL_FDC2        (2 << CGU_ESR0_29_ESRSEL_SHIFT) /* Selects FDC2 */
#  define CGU_ESR0_29_ESRSEL_FDC3        (3 << CGU_ESR0_29_ESRSEL_SHIFT) /* Selects FDC3 */
#  define CGU_ESR0_29_ESRSEL_FDC4        (4 << CGU_ESR0_29_ESRSEL_SHIFT) /* Selects FDC4 */
#  define CGU_ESR0_29_ESRSEL_FDC5        (5 << CGU_ESR0_29_ESRSEL_SHIFT) /* Selects FDC5 */
#  define CGU_ESR0_29_ESRSEL_FDC6        (6 << CGU_ESR0_29_ESRSEL_SHIFT) /* Selects FDC6 */

#define CGU_ESR30_39_ESRSEL_FDC7         (0)       /* Bit 1=0 selects FDC7 */
#define CGU_ESR30_39_ESRSEL_FDC8         (1 << 1)  /* Bit 1=1 selects FDC8 */

#define CGU_ESR40_49_ESRSEL_FDC9         (0)       /* Bit 1=0 selects FDC9 */
#define CGU_ESR40_49_ESRSEL_FDC10        (1 << 1)  /* Bit 1=1 selects FDC10 */

#define CGU_ESR50_57_ESRSEL_SHIFT        (1)       /* Bits 1-3: Selection of fractional dividers */
#define CGU_ESR50_57_ESRSEL_MASK         (3 << CGU_ESR50_57_ESRSEL_SHIFT)
#  define CGU_ESR50_57_ESRSEL_FDC11      (0 << CGU_ESR50_57_ESRSEL_SHIFT) /* Selects FDC11 */
#  define CGU_ESR50_57_ESRSEL_FDC12      (1 << CGU_ESR50_57_ESRSEL_SHIFT) /* Selects FDC12 */
#  define CGU_ESR50_57_ESRSEL_FDC13      (2 << CGU_ESR50_57_ESRSEL_SHIFT) /* Selects FDC13 */

#define CGU_ESR58_70_ESRSEL_FDC14        (0)       /* Only option */
#define CGU_ESR71_ESRSEL_FDC15           (0)       /* Only option */
#define CGU_ESR72_ESRSEL_FDC16           (0)       /* Only option */

#define CGU_ESR73_86_ESRSEL_SHIFT        (1)       /* Bits 1-3: Selection of fractional dividers */
#define CGU_ESR73_86_ESRSEL_MASK         (7 << CGU_ESR73_86_ESRSEL_SHIFT)
#  define CGU_ESR73_86_ESRSEL_FDC17      (0 << CGU_ESR73_86_ESRSEL_SHIFT) /* Selects FDC17 */
#  define CGU_ESR73_86_ESRSEL_FDC18      (1 << CGU_ESR73_86_ESRSEL_SHIFT) /* Selects FDC18 */
#  define CGU_ESR73_86_ESRSEL_FDC19      (2 << CGU_ESR73_86_ESRSEL_SHIFT) /* Selects FDC19 */
#  define CGU_ESR73_86_ESRSEL_FDC20      (3 << CGU_ESR73_86_ESRSEL_SHIFT) /* Selects FDC20 */
#  define CGU_ESR73_86_ESRSEL_FDC21      (4 << CGU_ESR73_86_ESRSEL_SHIFT) /* Selects FDC21 */
#  define CGU_ESR73_86_ESRSEL_FDC22      (5 << CGU_ESR73_86_ESRSEL_SHIFT) /* Selects FDC22 */

#define CGU_ESR87_88_ESRSEL_FDC23        (0)       /* Only option */

#define CGU_ESR_ESREN                    (1 << 0)  /* Bit 0:  Enable from FD selected by ESRSEL */

/* Base control registers 0 BCR0 to BCR7, addresses 0x13004504 to 0x13004514 */

#define CGU_BCR_FDRUN                    (1 << 0)  /* Bit 0: Enable fractional dividers */

/* Fractional divider register 0 to 23 FDC0 to FDC23 (except FDC17) addresses 0x13004518 to 0x13004574 */

#define CGU_FDC_MSUB_SHIFT               (11)      /* Bits 11-18: Modulo subtraction value */
#define CGU_FDC_MSUB_MASK                (0x000000ff << CGU_FDC_MSUB_SHIFT)
#define CGU_FDC_MSUB_EXTEND              (0xffffff00)
#define CGU_FDC_MADD_SHIFT               (3)       /* Bits 3-10: Modulo addition value */
#define CGU_FDC_MADD_MASK                (0x000000ff << CGU_FDC_MADD_SHIFT)
#define CGU_FDC_STRETCH                  (1 << 2)  /* Bit 2:  Enables the stretching option */
#define CGU_FDC_RESET                    (1 << 1)  /* Bit 1:  Reset fractional divider */
#define CGU_FDC_RUN                      (1 << 0)  /* Bit 0:  Enable fractional divider */

#define CGU_FDC17_MSUB_SHIFT             (16)      /* Bits 16-28: Modulo subtraction value */
#define CGU_FDC17_MSUB_MASK              (0x00001fff << CGU_FDC17_MSUB_SHIFT)
#define CGU_FDC17_MSUB_EXTEND            (0xffffe000)
#define CGU_FDC17_MADD_SHIFT             (3)       /* Bits 3-15: Modulo addition value */
#define CGU_FDC17_MADD_MASK              (0x00001fff << CGU_FDC17_MADD_SHIFT)
#define CGU_FDC17_STRETCH                (1 << 2)  /* Bit 2:  Enables the stretching option */
#define CGU_FDC17_RESET                  (1 << 1)  /* Bit 1:  Reset fractional divider */
#define CGU_FDC17_RUN                    (1 << 0)  /* Bit 0:  Enable fractional divider */

#define CGU_FDC_FIELDWIDTH               8         /* MSUB and MADD fields are 8-bits in width */
#define CGU_FDC17_FIELDWIDTH             13        /* Exept for FDC17 which is 13-bits in width */

/* Dynamic Fractional Divider registers DYNFDC0 to DYNFDC6, addresses 0x13004578 to 0x13004590 */

#define CGU_DYNFDC_STOPAUTORST           (1 << 19) /* Bit 19: Disable auto reset of fractional divider */
#define CGU_DYNFDC_MSUB_SHIFT            (11)      /* Bits 11-18: Modulo subtraction value */
#define CGU_DYNFDC_MSUB_MASK             (255 << CGU_DYNFDC_MSUB_SHIFT)
#define CGU_DYNFDC_MADD_SHIFT            (3)       /* Bits 3-10: Modulo addition value */
#define CGU_DYNFDC_MADD_MASK             (255 << CGU_DYNFDC_MADD_SHIFT)
#define CGU_DYNFDC_STRETCH               (1 << 2)  /* Bit 2:  Enables the stretching option */
#define CGU_DYNFDC_ALLOW                 (1 << 1)  /* Bit 1:  Enable dynamic fractional divider */
#define CGU_DYNFDC_RUN                   (1 << 0)  /* Bit 0:  Enable the fractional divider during low speeds */

/* Dynamic Fractional Divider Selection register DYNSEL0 to DYNSEL6, addresses 0x13004594 to 0x130045ac */

#define CGU_DYNSEL_MPMCREFRESHREQ        (1 << 8)  /* Bit 8:  Ext SDRAM refresh can enable high speed */
#define CGU_DYNSEL_ECCRAMBUSY            (1 << 7)  /* Bit 7:  Hispeed mode during NAND ECC */
#define CGU_DYNSEL_USBOTGMSTTRANS        (1 << 6)  /* Bit 6:  USB OTG transfers can enable high speed */
#define CGU_DYNSEL_ARM926LPDREADY        (1 << 5)  /* Bit 5:  ARM926 data transfers can enable high-speed */
#define CGU_DYNSEL_ARM926LPDTRANS        (1 << 4)  /* Bit 4:  ARM926 data transfers can enable high-speed */
#define CGU_DYNSEL_ARM926LPIREADY        (1 << 3)  /* Bit 3:  ARM926 instr last transfers can enable high-speed */
#define CGU_DYNSEL_ARM926LPITRANS        (1 << 2)  /* Bit 2:  ARM926 instr transfers can enable high-speed */
#define CGU_DYNSEL_DMAREADY              (1 << 1)  /* Bit 1:  dma last transfers can enable high-speed */
#define CGU_DYNSEL_DMATRANS              (1 << 0)  /* Bit 0:  dma transfers can enable high-speed */
#define CGU_DYNSEL_ALLBITS               (0x1ff)

/* CGU configuration register bit definitions ***************************************************/
/* Power and oscillator control registers */
/* Powermode register POWERMODE, address 0x13004c00 */

#define CGU_POWERMODE_SHIFT              (0)      /* Bits 0-1: Powermode */
#define CGU_POWERMODE_MASK               (3 << bb)
#define CGU_POWERMODE_NORMAL             (1 << bb) /* Normal operational mode */
#define CGU_POWERMODE_WAKEUP             (3 << bb) /* Wakeup clocks disabled until wakeup event */

/* Watchdog Bark register WDBARK, address 0x13004c04 */

#define CGU_WDBARK_RESET                 (1 << 0)  /* Bit 0:  Set when a watchdog reset has occurred */

/* Fast Oscillator activate register FFASTON, 0x13004c08 */

#define CGU_FFASTON_ACTIVATE             (1 << 0)  /* Bit 0:  Activate fast oscillator */

/* Fast Oscillator Bypass comparator register FFASTBYP, 0x13004c0c */

#define CGU_FFASTBYP_TESTMODE            (1 << 0)  /* Bit 0: Oscillator test mode */

/* Reset control registers */

#define CGU_SOFTRESET                    (1 << 0)  /* Bit 0: in all of the reset control registers */

/* APB0_RESETN_SOFT register, address 0x13004c10 */

#define CGU_APB0RST_RESET                (1 << 0)  /* Bit 0: Reserved */

/* AHB_TO_APB0_PNRES_SOFT register, address 0x13004c14 */

#define CGU_AHB2APB0RST_RESET            (1 << 0)  /* Bit 0:  Reserved */

/* APB1_RESETN_SOFT register, address 0x13004c18 */

#define CGU_APB1RST_RESET                (1 << 0)  /* Bit 0:  Reset for AHB part of AHB_TO_APB1 bridge */

/* AHB_TO_APB1_PNRES_SOFT register, address 0x13004c1c */

#define CGU_AHB2PB1RST_RESET             (1 << 0)  /* Bit 0:  Reset for APB part of AHB_TO_APB1 bridge */

/* APB2_RESETN_SOFT register, address 0x13004c20 */

#define CGU_APB2RST_RESET                (1 << 0)  /* Bit 0:  Reset for AHB part of AHB_TO_APB2 bridge */

/* AHB_TO_APB2_PNRES_SOFT register, address 0x13004c24 */

#define CGU_AHB2APB2RST_RESET            (1 << 0)  /* Bit 0:  Reset for APB part of AHB_TO_APB2 bridge */

/* APB3_RESETN_SOFT register, address 0x13004c28 */

#define CGU_APB3RST_RESET                (1 << 0)  /* Bit 0:  Reset for AHB part of AHB_TO_APB3 bridge */

/* AHB_TO_APB3_PNRES_SOFT register, address 0x13004c2c */

#define CGU_AHB2APB3RST_RESET            (1 << 0)  /* Bit 0:  Reset for APB part of AHB_TO_APB3 bridge */

/* APB4_RESETN_SOFT register, address 0x13004c30 */

#define CGU_APB4RST_RESET                (1 << 0)  /* Bit 0:  Reset for AHB part of AHB_TO_APB4 bridge */

/* AHB_TO_INTC_RESETN_SOFT register, address 0x13004c34 */

#define CGU_AHB2INTCRST_RESET            (1 << 0)  /* Bit 0:  Reset for AHB_TO_INTC */

/* AHB0_RESETN_SOFT register, address 0x13004c38 */

#define CGU_AHB0RST_RESET                (1 << 0)  /* Bit 0:  Reserved */

/* EBI_RESETN_SOFT register, address 0x13004c2c */

#define CGU_EBIRST_RESET                 (1 << 0)  /* Bit 0:  Reset for EBI */

/* PCM_PNRES_SOFT UNIT register, address 0x13004c40 */

#define CGU_PCMAPBRST_RESET              (1 << 0)  /* Bit 0:  Reset for APB domain of PCM */

/* PCM_RESET_N_SOFT register, address 0x13004c44 */

#define CGU_PCMCLKIPRST_RESET            (1 << 0)  /* Bit 0:  Reset for synchronous clk_ip domain of PCM */

/* PCM_RESET_ASYNC_N_SOFT register, address 0x13004c48 */

#define CGU_PCMRSTASYNC_RESET            (1 << 0)  /* Bit 0:  Reset for asynchronous clk_ip domain of PCM */

/* TIMER0_PNRES_SOFT register, address 0x13004c4c */

#define CGU_TIMER0RST_RESET              (1 << 0)  /* Bit 0:  Reset for Timer0 */

/* TIMER1_PNRES_SOFT register, address 0x13004c50 */

#define CGU_TIMER1RST_RESET              (1 << 0)  /* Bit 0:  Reset for Timer1 */

/* TIMER2_PNRES_SOFT register, address 0x13004c54 */

#define CGU_TIMER2RST_RESET              (1 << 0)  /* Bit 0:  Reset for Timer2 */

/* TIMER3_PNRES_SOFT register, address 0x13004c58 */

#define CGU_TIMER3RST_RESET              (1 << 0)  /* Bit 0:  Reset for Timer3 */

/* ADC_PRESETN_SOFT register, address 0x13004c5c */

#define CGU_ADCPRST_RESET                (1 << 0)  /* Bit 0:  Reset for controller of 10 bit ADC Interface */

/* ADC_RESETN_ADC10BITS_SOFT register, address 0x1300 4c60 */

#define CGU_ADCRST_RESET                 (1 << 0)  /* Bit 0:  Reset for A/D converter of ADC Interface */

/* PWM_RESET_AN_SOFT register, address 0x13004c64 */

#define CGU_PWMRST_RESET                 (1 << 0)  /* Bit 0:  Reset for PWM */

/* UART_SYS_RST_AN_SOFT register, address 0x13004c68 */

#define CGU_UARTRST_RESET                (1 << 0)  /* Bit 0:  Reset for UART/IrDA */

/* I2C0_PNRES_SOFT register, address 0x13004c6c */

#define CGU_I2C0RST_RESET                (1 << 0)  /* Bit 0:  Reset for I2C0 */

/* I2C1_PNRES_SOFT register, address 0x13004c70 */

#define CGU_I2C1RST_RESET                (1 << 0)  /* Bit 0:  Reset for I2C1 */

/* I2S_CFG_RST_N_SOFT register, address 0x13004c74 */

#define CGU_I2SCFGRST_RESET              (1 << 0)  /* Bit 0:  Reset for I2S_Config */

/* I2S_NSOF_RST_N_SOFT register, address 0x13004c78 */

#define CGU_I2SNSOFRST_RESET             (1 << 0)  /* Bit 0:  Reset for NSOF counter of I2S_CONFIG */

/* EDGE_DET_RST_N_SOFT register, address 0x13004c7c */

#define CGU_EDGEDETRST_RESET             (1 << 0)  /* Bit 0:  Reset for Edge_det */

/* I2STX_FIFO_0_RST_N_SOFT register, address 0x13004c80 */

#define CGU_I2STXFF0RST_RESET            (1 << 0)  /* Bit 0:  Reset for I2STX_FIFO_0 */

/* I2STX_IF_0_RST_N_SOFT register, address 0x13004c84 */

#define CGU_I2STXIF0RST_RESET            (1 << 0)  /* Bit 0:  Reset for I2STX_IF_0 */

/* I2STX_FIFO_1_RST_N_SOFT register, address 0x13004c88 */

#define CGU_I2STXFF1RST_RESET            (1 << 0)  /* Bit 0:  Reset for I2STX_FIFO_1 */

/* I2STX_IF_1_RST_N_SOFT register, address 0x13004c8c */

#define CGU_I2STXIF1RST_RESET            (1 << 0)  /* Bit 0:  Reset for I2STX_IF_1 */

/* I2SRX_FIFO_0_RST_N_SOFT register, address 0x13004c90 */

#define CGU_I2SRXFF0RST_RESET            (1 << 0)  /* Bit 0:  Reset for I2SRX_FIFO_0 */

/* I2SRX_IF_0_RST_N_SOFT register, address 0x13004c94 */

#define CGU_I2SRXIF0RST_RESET            (1 << 0)  /* Bit 0:  Reset for I2SRX_IF_0 */

/* I2SRX_FIFO_1_RST_N_SOFT register, address 0x13004c98 */

#define CGU_I2SRXFF1RST_RESET            (1 << 0)  /* Bit 0:  Reset for I2SRX_FIFO_1 */

/* I2SRX_IF_1_RST_N_SOFT register, address 0x13004c9c */

#define CGU_I2SRXIF1RST_RESET            (1 << 0)  /* Bit 0:  Reset for I2SRX_IF_1 */

/* LCD_PNRES_SOFT register, address 0x13004cB4 */

#define CGU_LCDRST_RESET                 (1 << 0)  /* Bit 0:  Reset for LCD Interface */

/* SPI_PNRES_APB_SOFT register, address 0x13004cb8 */

#define CGU_SPIRSTAPB_RESET              (1 << 0)  /* Bit 0:  Reset register for apb_clk domain of SPI */

/* SPI_PNRES_IP_SOFT register, address 0x13004cbc */

#define CGU_SPIRSTIP_RESET               (1 << 0)  /* Bit 0:  Reset for ip_clk domain of SPI */

/* DMA_PNRES_SOFT register, address 0x13004cc0 */

#define CGU_DMARST_RESET                 (1 << 0)  /* Bit 0:  Reset for DMA */

/* NANDFLASH_CTRL_ECC_RESET_N_SOFT register, address 0x13004cc4 */

#define CGU_NANDECCRST_RESET             (1 << 0)  /* Bit 0:  Reset for ECC clock domain of Nandflash Controller */

/* NANDFLASH_CTRL_AES_RESET_N_SOFT register, address 0x13004cc8 (LPC3154 only) */

#define CGU_NANDAECRST_RESET             (1 << 0)  /* Bit 0:  Reset for AES clock domain of
Nandflash Controller */

/* NANDFLASH_CTRL_NAND_RESET_N_SOFT register, address 0x13004ccc */

#define CGU_NANDCTRLRST_RESET            (1 << 0)  /* Bit 0:  Reset for Nandflash Controller */

/* SD_MMC_PNRES_SOFT register, address 0x13004cd4 */

#define CGU_SDMMCRST_RESET               (1 << 0)  /* Bit 0:  Reset for MCI synchronous with AHB clock */

/* SD_MMC_NRES_CCLK_IN_SOFT register, address 0x1300 4cd8 */

#define CGU_SDMMCRSTCKIN_RESET           (1 << 0)  /* Bit 0:  Reset register for MCI synchronous with IP clock */

/* USB_OTG_AHB_RST_N_SOFT, address 0x13004cdc */

#define CGU_USBOTGAHBRST_RESET           (1 << 0)  /* Bit 0:  Reset for USB_OTG */

/* RED_CTL_RESET_N_SOFT, address 0x13004ce0 */

#define CGU_REDCTLRST_RESET              (1 << 0)  /* Bit 0:  Reset for Redundancy Controller */

/* AHB_MPMC_HRESETN_SOFT, address 0x13004ce4 */

#define CGU_AHBMPMCHRST_RESET            (1 << 0)  /* Bit 0:  Reset for MPMC */

/* AHB_MPMC_REFRESH_RESETN_SOFT, address 0x13004ce8 */

#define CGU_AHBMPMCRFRST_RESET           (1 << 0)  /* Bit 0:  Reset for refresh generator used for MPMC */

/* INTC_RESETN_SOFT, address 0x13004cec */

#define CGU_INTCRST_RESET                (1 << 0)  /* Bit 0:  Reset for Interrupt Controller */

/* PLL control registers */
/* HP0 Frequency Input Select register HP0_FIN_SELECT, address 0x13004cf0,
 * HP1 Frequency Input Select register HP1_FIN_SELECT, address 0x13004d28
 */

#define CGU_HPFINSEL_SHIFT               (0)       /* Bits 0-3: Select input to high HPPLL0 */
#define CGU_HPFINSEL_MASK                (15 << CGU_HPFINSEL_SHIFT)
#  define CGU_HPFINSEL_FFAST             (CGU_FREQIN_FFAST     << CGU_HPFINSEL_SHIFT) /* ffast (12 Mhz) */
#  define CGU_HPFINSEL_I2SRXBCK0         (CGU_FREQIN_I2SRXBCK0 << CGU_HPFINSEL_SHIFT) /* I2SRX_BCK0 */
#  define CGU_HPFINSEL_I2SRXWS0          (CGU_FREQIN_I2SRXWS0  << CGU_HPFINSEL_SHIFT) /* I2SRX_WS0 */
#  define CGU_HPFINSEL_I2SRXBCK1         (CGU_FREQIN_I2SRXBCK1 << CGU_HPFINSEL_SHIFT) /* I2SRX_BCK1 */
#  define CGU_HPFINSEL_I2SRXWS1          (CGU_FREQIN_I2SRXWS1  << CGU_HPFINSEL_SHIFT) /* I2SRX_WS1 */
#  define CGU_HPFINSEL_HP0FOUT           (CGU_FREQIN_HPPLL0    << CGU_HPFINSEL_SHIFT) /* HP0_FOUT */
#  define CGU_HPFINSEL_HP1FOUT           (CGU_FREQIN_HPPLL1    << CGU_HPFINSEL_SHIFT) /* HP1_FOUT */

/* HP0 M-divider register HP0_MDEC, address 0x13004cF4,
 * HP1 M-divider register HP1_MDEC, address 0x13004d2C
 */

#define CGU_HPMDEC_SHIFT                 (0)       /* Bits 0-16: Decoded divider ratio for M-divider */
#define CGU_HPMDEC_MASK                  (0x1ffff << CGU_HPMDEC_SHIFT)

/* HP0 N-divider register HP0_NDEC, address 0x13004cf8,
 * HP1 N-divider register HP1_NDEC, address 0x13004D30
 */

#define CGU_HPNDEC_SHIFT                 (0)       /* Bits 0-9: Decoded divider ratio for N-divider */
#define CGU_HPNDEC_MASK                  (0x3ff << CGU_HPNDEC_SHIFT)

/* HP0 P-divider register HP0_PDEC, address 0x13004cfc.
 * HP1 P-divider register HP1_PDEC, address 0x13004D34
 */

#define CGU_HPPDEC_SHIFT                 (0)       /* Bits 0-6: Decoded divider ratio for P-divider */
#define CGU_HPPDEC_MASK                  (0x7F << CGU_HPPDEC_SHIFT)

/* HP0 Mode register HP0_MODE, address 0x13004d00,
 * HP1 Mode register HP1_MODE, address 0x13004d38
 */

#define CGU_HPMODE_BYPASS                (1 << 8)  /* Bit 8:  Bypass mode */
#define CGU_HPMODE_LIMUPOFF              (1 << 7)  /* Bit 7:  Up limiter */
#define CGU_HPMODE_BANDSEL               (1 << 6)  /* Bit 6:  Bandwidth adjustment pin */
#define CGU_HPMODE_FRM                   (1 << 5)  /* Bit 5:  Free Running Mode */
#define CGU_HPMODE_DIRECTI               (1 << 4)  /* Bit 4:  Normal operation with DIRECTO */
#define CGU_HPMODE_DIRECTO               (1 << 3)  /* Bit 3:  Normal operation with DIRECTI */
#define CGU_HPMODE_PD                    (1 << 2)  /* Bit 2:  Power down mode */
#define CGU_HPMODE_SKEWEN                (1 << 1)  /* Bit 1:  Skew mode */
#define CGU_HPMODE_CLKEN                 (1 << 0)  /* Bit 0:  Enable mode */

/* HP0 Status register HP0_STATUS, address 0x13004d04,
 * HP1 Status register HP1_STATUS, address 0x13004d3c
 */

#define CGU_HPSTATUS_FR                  (1 << 1)  /* Bit 1:  Free running detector */
#define CGU_HPSTATUS_LOCK                (1 << 0)  /* Bit 0:  Lock detector */

/* HP0 Acknowledge register HP0_ACK, address 0x13004d08,
 * HP1 Acknowledge register HP1_ACK, address 0x13004d40
 */

#define CGU_HPACK_P                      (1 << 2)  /* Bit 2:  Post-divider ratio change acknowledge */
#define CGU_HPACK_N                      (1 << 1)  /* Bit 1:  Pre-divider ratio change acknowledge */
#define CGU_HPACK_M                      (1 << 0)  /* Bit 0:  Feedback divider ratio change acknowledge */

/* HP0 request register HP0_REQ, address 0x13004d0c,
 * HP1 Request register HP1_REQ, address 0x13004d44
 */

#define CGU_HPREQ_P                      (1 << 2)  /* Bit 2:  Post-divider ratio change request */
#define CGU_HPREQ_N                      (1 << 1)  /* Bit 1:  Pre-divider ratio change request */
#define CGU_HPREQ_M                      (1 << 0)  /* Bit 0:  Feedback divider ratio change request */

/* HP0 Bandwith Selection register HP0_INSELR, address 0x13004d10,
 * HP1 bandwith Selection register HP1_INSELR, address 0x13004d48
 */
 
#define CGU_HPINSELR_SHIFT               (0)      /* Bits 0-3: Pins to select the bandwidth */
#define CGU_HPINSELR_MASK                (15 << CGU_HPINSELR_SHIFT)

/* HP0 Bandwith Selection register HP0_INSELI, address 0x13004d14,
 * HP1 bandwith Selection register HP1_INSELI, address 0x13004d4c
 */

#define CGU_HPINSELI_SHIFT               (0)      /* Bits 0-5: Bandwidth selection register of HP0/1 PLL */
#define CGU_HPINSELI_MASK                (63 << CGU_HPINSELI_SHIFT)


/* HP0 Bandwith Selection register HP0_INSELP, address 0x13004d18,
 * HP1 bandwith Selection register HP1_INSELP, address 0x13004d50
 */

#define CGU_HPINSELP_SHIFT               (0)      /* Bits 0-4: Bandwidth selection register of HP0/1 PLL */
#define CGU_HPINSELP_MASK                (31 << CGU_HPINSELP_SHIFT)

/* HP0 Bandwith Selection register HP0_SELR, address 0x13004d1c,
 * HP1 bandwith Selection register HP1_SELR, address 0x13004d54
 */

#define CGU_HPSELR_SHIFT                 (0)      /* Bits 0-3: Bandwidth selection register of HP0/1 PLL */
#define CGU_HPSELR_MASK                  (15 << CGU_HPSELR_SHIFT)

/* HP0 Bandwith Selection register HP0_SELI, address 0x13004d20
 * HP1 bandwith Selection register HP1_SELI, address 0x13004d58
 */

#define CGU_HPSELI_SHIFT                 (0)      /* Bits 0-5: Bandwidth selection register of HP0/1 PLL */
#define CGU_HPSELI_MASK                  (63 << CGU_HPSELI_SHIFT)

/* HP0 Bandwith Selection register HP0_SELP, address 0x13004d24,
 * HP1 bandwith Selection register HP1_SELP, address 0x13004d5c
 */

#define CGU_HPSELP_SHIFT                 (0)      /* Bits 0-4: Bandwidth selection register of HP0/1 PLL */
#define CGU_HPIELP_MASK                  (31 << CGU_HPSELP_SHIFT)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Inline Functions
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_CGU_H */
