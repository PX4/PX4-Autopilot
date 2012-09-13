/************************************************************************************
 * arch/hc/src/m9s12/chip.h
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_HC_SRC_M9S12_CHIP_H
#define __ARCH_HC_SRC_M9S12_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Memory Map.
 *
 * At reset:
 *   0x0000–0x03ff: register space
 *   0x0000–0x1fff: 7K RAM (1K RAM hidden behind register space)
 */

#define HCS12_REG_BASE     0x0000 /* 0x0000-0x03ff: Mapped Register base address */
#define HCS12_EEPROM_BASE  0x0800 /* 0x0800:        Mapped EEPROM base address */
#define HCS12_SRAM_BASE    0x2000 /* 0x2000-0x3fff: Mapped SRAM base address */
#define HCS12_FFLASH1_BASE 0x4000 /* 0x4000-0x7fff: 16Kb Fixed FLASH EEPROM */
#define HCS12_PPAGE_BASE   0x8000 /* 0x8000-0xbfff: 16Kb Page window */
#define HCS12_FFLASH2_BASE 0xc000 /* 0xc000-0xffff: 16Kb Fixed FLASH EEPROM */

/* Device Register Map Overview (all relative to HCS12_REG_BASE) */

#define HCS12_CORE1_BASE   0x0000 /* 0x0000–0x0017: Ports A, B, E, Modes, Inits (MMC, INT, MEBI) */
                                  /* 0x0018–0x0019: Reserved */
#define HCS12_DEVID_BASE   0x001a /* 0x001a-0x001b: Device ID register (PARTID) */
#define HCS12_CORE2_BASE   0x001c /* 0x001c–0x001f: MEMSIZ, IRQ, HPRIO (INT, MMC) */
#define HCS12_CORE3_BASE   0x0020 /* 0x0020-0x002f: DBG */
#define HCS12_CORE4_BASE   0x0030 /* 0x0030–0x0033: PPAGE, Port K (MEBI, MMC) */
#define HCS12_CRG_BASE     0x0034 /* 0x0034–0x003f: Clock and Reset Generator (PLL, RTI, COP) */
#define HCS12_TIM_BASE     0x0040 /* 0x0040–0x006f: Standard Timer 16-bit 4 channels (TIM) */
                                  /* 0x0070–0x007f: Reserved */
#define HCS12_ATD_BASE     0x0080 /* 0x0080–0x009f: Analog-to-Digital Converter 10-bit, 8-channel (ATD) */
                                  /* 0x00a0–0x00c7: Reserved */
#define HCS12_SCI0_BASE    0x00c8 /* 0x00c8–0x00cf: Serial Communications Interface 0 (SCI0) */
#define HCS12_SCI1_BASE    0x00d0 /* 0x00d0–0x00d7: Serial Communications Interface 1 (SCI1) */o
#define HCS12_SPI_BASE     0x00d8 /* 0x00d8–0x00df: Serial Peripheral Interface (SPI) */
#define HCS12_IIC_BASE     0x00e0 /* 0x00e0–0x00e7: Inter IC Bus (IIC) */
                                  /* 0x00e8–0x00ff: Reserved */
#define HCS12_FLASH_BASE   0x0100 /* 0x0100–0x010f: FLASH Control Register */
                                  /* 0x0110–0x011f: Reserved */
#define HCS12_EPHY_BASE    0x0120 /* 0x0120–0x0123: Ethernet Physical Interface (EPHY) */
                                  /* 0x0124–0x013f: Reserved */
#define HCS12_EMAC_BASE    0x0140 /* 0x0140–0x016f: Ethernet Media Access Controller (EMAC) */
                                  /* 0x0170–0x023f: Reserved */
#define HCS12_PIM_BASE     0x0240 /* 0x0240–0x026f: Port Integration Module (PIM) */
                                  /* 0x0270–0x03ff: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_HC_SRC_M9S12_CHIP_H */
