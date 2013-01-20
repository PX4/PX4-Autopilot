/****************************************************************************************************
 * arch/arm/src/stm32/chip/stm32_eth.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_ETH_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_ETH_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if STM32_NETHERNET > 0

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Register Offsets *********************************************************************************/
/* MAC Registers */

#define STM32_ETH_MACCR_OFFSET       0x0000 /* Ethernet MAC configuration register */
#define STM32_ETH_MACFFR_OFFSET      0x0004 /* Ethernet MAC frame filter register */
#define STM32_ETH_MACHTHR_OFFSET     0x0008 /* Ethernet MAC hash table high register */
#define STM32_ETH_MACHTLR_OFFSET     0x000c /* Ethernet MAC hash table low register */
#define STM32_ETH_MACMIIAR_OFFSET    0x0010 /* Ethernet MAC MII address register */
#define STM32_ETH_MACMIIDR_OFFSET    0x0014 /* Ethernet MAC MII data register */
#define STM32_ETH_MACFCR_OFFSET      0x0018 /* Ethernet MAC flow control register */
#define STM32_ETH_MACVLANTR_OFFSET   0x001c /* Ethernet MAC VLAN tag register */
#define STM32_ETH_MACRWUFFR_OFFSET   0x0028 /* Ethernet MAC remote wakeup frame filter reg */
#define STM32_ETH_MACPMTCSR_OFFSET   0x002c /* Ethernet MAC PMT control and status register */
#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define STM32_ETH_MACDBGR_OFFSET   0x0034 /* Ethernet MAC debug register */
#endif
#define STM32_ETH_MACSR_OFFSET       0x0038 /* Ethernet MAC interrupt status register */
#define STM32_ETH_MACIMR_OFFSET      0x003c /* Ethernet MAC interrupt mask register */
#define STM32_ETH_MACA0HR_OFFSET     0x0040 /* Ethernet MAC address 0 high register */
#define STM32_ETH_MACA0LR_OFFSET     0x0044 /* Ethernet MAC address 0 low register */
#define STM32_ETH_MACA1HR_OFFSET     0x0048 /* Ethernet MAC address 1 high register */
#define STM32_ETH_MACA1LR_OFFSET     0x004c /* Ethernet MAC address1 low register */
#define STM32_ETH_MACA2HR_OFFSET     0x0050 /* Ethernet MAC address 2 high register */
#define STM32_ETH_MACA2LR_OFFSET     0x0054 /* Ethernet MAC address 2 low register */
#define STM32_ETH_MACA3HR_OFFSET     0x0058 /* Ethernet MAC address 3 high register */
#define STM32_ETH_MACA3LR_OFFSET     0x005c /* Ethernet MAC address 3 low register */

/* MMC Registers */

#define STM32_ETH_MMCCR_OFFSET       0x0100 /* Ethernet MMC control register */
#define STM32_ETH_MMCRIR_OFFSET      0x0104 /* Ethernet MMC receive interrupt register */
#define STM32_ETH_MMCTIR_OFFSET      0x0108 /* Ethernet MMC transmit interrupt register */
#define STM32_ETH_MMCRIMR_OFFSET     0x010c /* Ethernet MMC receive interrupt mask register */
#define STM32_ETH_MMCTIMR_OFFSET     0x0110 /* Ethernet MMC transmit interrupt mask register */
#define STM32_ETH_MMCTGFSCCR_OFFSET  0x014c /* Ethernet MMC transmitted good frames counter register (single collision) */
#define STM32_ETH_MMCTGFMSCCR_OFFSET 0x0150 /* Ethernet MMC transmitted good frames counter register (multiple-collision) */
#define STM32_ETH_MMCTGFCR_OFFSET    0x0168 /* Ethernet MMC transmitted good frames counter register */
#define STM32_ETH_MMCRFCECR_OFFSET   0x0194 /* Ethernet MMC received frames with CRC error counter register */
#define STM32_ETH_MMCRFAECR_OFFSET   0x0198 /* Ethernet MMC received frames with alignment error counter */
#define STM32_ETH_MMCRGUFCR_OFFSET   0x01c4 /* MMC received good unicast frames counter register */

/* IEEE 1588 time stamp registers */

#define STM32_ETH_PTPTSCR_OFFSET     0x0700 /* Ethernet PTP time stamp control register */
#define STM32_ETH_PTPSSIR_OFFSET     0x0704 /* Ethernet PTP subsecond increment register */
#define STM32_ETH_PTPTSHR_OFFSET     0x0708 /* Ethernet PTP time stamp high register */
#define STM32_ETH_PTPTSLR_OFFSET     0x070c /* Ethernet PTP time stamp low register */
#define STM32_ETH_PTPTSHUR_OFFSET    0x0710 /* Ethernet PTP time stamp high update register */
#define STM32_ETH_PTPTSLUR_OFFSET    0x0714 /* Ethernet PTP time stamp low update register */
#define STM32_ETH_PTPTSAR_OFFSET     0x0718 /* Ethernet PTP time stamp addend register */
#define STM32_ETH_PTPTTHR_OFFSET     0x071c /* Ethernet PTP target time high register */
#define STM32_ETH_PTPTTLR_OFFSET     0x0720 /* Ethernet PTP target time low register */
#define STM32_ETH_PTPTSSR_OFFSET     0x0728 /* Ethernet PTP time stamp status register */
#define STM32_ETH_PTPPPSCR_OFFSET    0x072c /* Ethernet PTP PPS control register */

/* DMA Registers */

#define STM32_ETH_DMABMR_OFFSET      0x1000 /* Ethernet DMA bus mode register */
#define STM32_ETH_DMATPDR_OFFSET     0x1004 /* Ethernet DMA transmit poll demand register */
#define STM32_ETH_DMARPDR_OFFSET     0x1008 /* Ethernet DMA receive poll demand register */
#define STM32_ETH_DMARDLAR_OFFSET    0x100c /* Ethernet DMA receive descriptor list address register */
#define STM32_ETH_DMATDLAR_OFFSET    0x1010 /* Ethernet DMA transmit descriptor list address register */
#define STM32_ETH_DMASR_OFFSET       0x1014 /* Ethernet DMA status register */
#define STM32_ETH_DMAOMR_OFFSET      0x1018 /* Ethernet DMA operation mode register */
#define STM32_ETH_DMAIER_OFFSET      0x101c /* Ethernet DMA interrupt enable register */
#define STM32_ETH_DMAMFBOC_OFFSET    0x1020 /* Ethernet DMA missed frame and buffer overflow counter register */
#define STM32_ETH_DMARSWTR_OFFSET    0x1024 /* Ethernet DMA receive status watchdog timer register */
#define STM32_ETH_DMACHTDR_OFFSET    0x1048 /* Ethernet DMA current host transmit descriptor register */
#define STM32_ETH_DMACHRDR_OFFSET    0x104c /* Ethernet DMA current host receive descriptor register */
#define STM32_ETH_DMACHTBAR_OFFSET   0x1050 /* Ethernet DMA current host transmit buffer address register */
#define STM32_ETH_DMACHRBAR_OFFSET   0x1054 /* Ethernet DMA current host receive buffer address register */

/* Register Base Addresses **************************************************************************/
/* MAC Registers */

#define STM32_ETH_MACCR              (STM32_ETHERNET_BASE+STM32_ETH_MACCR_OFFSET)
#define STM32_ETH_MACFFR             (STM32_ETHERNET_BASE+STM32_ETH_MACFFR_OFFSET)
#define STM32_ETH_MACHTHR            (STM32_ETHERNET_BASE+STM32_ETH_MACHTHR_OFFSET)
#define STM32_ETH_MACHTLR            (STM32_ETHERNET_BASE+STM32_ETH_MACHTLR_OFFSET)
#define STM32_ETH_MACMIIAR           (STM32_ETHERNET_BASE+STM32_ETH_MACMIIAR_OFFSET)
#define STM32_ETH_MACMIIDR           (STM32_ETHERNET_BASE+STM32_ETH_MACMIIDR_OFFSET)
#define STM32_ETH_MACFCR             (STM32_ETHERNET_BASE+STM32_ETH_MACFCR_OFFSET)
#define STM32_ETH_MACVLANTR          (STM32_ETHERNET_BASE+STM32_ETH_MACVLANTR_OFFSET)
#define STM32_ETH_MACRWUFFR          (STM32_ETHERNET_BASE+STM32_ETH_MACRWUFFR_OFFSET)
#define STM32_ETH_MACPMTCSR          (STM32_ETHERNET_BASE+STM32_ETH_MACPMTCSR_OFFSET)
#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define STM32_ETH_MACDBGR          (STM32_ETHERNET_BASE+STM32_ETH_MACDBGR_OFFSET)
#endif
#define STM32_ETH_MACSR              (STM32_ETHERNET_BASE+STM32_ETH_MACSR_OFFSET)
#define STM32_ETH_MACIMR             (STM32_ETHERNET_BASE+STM32_ETH_MACIMR_OFFSET)
#define STM32_ETH_MACA0HR            (STM32_ETHERNET_BASE+STM32_ETH_MACA0HR_OFFSET)
#define STM32_ETH_MACA0LR            (STM32_ETHERNET_BASE+STM32_ETH_MACA0LR_OFFSET)
#define STM32_ETH_MACA1HR            (STM32_ETHERNET_BASE+STM32_ETH_MACA1HR_OFFSET)
#define STM32_ETH_MACA1LR            (STM32_ETHERNET_BASE+STM32_ETH_MACA1LR_OFFSET)
#define STM32_ETH_MACA2HR            (STM32_ETHERNET_BASE+STM32_ETH_MACA2HR_OFFSET)
#define STM32_ETH_MACA2LR            (STM32_ETHERNET_BASE+STM32_ETH_MACA2LR_OFFSET)
#define STM32_ETH_MACA3HR            (STM32_ETHERNET_BASE+STM32_ETH_MACA3HR_OFFSET)
#define STM32_ETH_MACA3LR            (STM32_ETHERNET_BASE+STM32_ETH_MACA3LR_OFFSET)

/* MMC Registers */

#define STM32_ETH_MMCC               (STM32_ETHERNET_BASE+STM32_ETH_MMCCR_OFFSET)
#define STM32_ETH_MMCRIR             (STM32_ETHERNET_BASE+STM32_ETH_MMCRIR_OFFSET)
#define STM32_ETH_MMCTIR             (STM32_ETHERNET_BASE+STM32_ETH_MMCTIR_OFFSET)
#define STM32_ETH_MMCRIMR            (STM32_ETHERNET_BASE+STM32_ETH_MMCRIMR_OFFSET)
#define STM32_ETH_MMCTIMR            (STM32_ETHERNET_BASE+STM32_ETH_MMCTIMR_OFFSET)
#define STM32_ETH_MMCTGFSCCR         (STM32_ETHERNET_BASE+STM32_ETH_MMCTGFSCCR_OFFSET)
#define STM32_ETH_MMCTGFMSCCR        (STM32_ETHERNET_BASE+STM32_ETH_MMCTGFMSCCR_OFFSET)
#define STM32_ETH_MMCTGFCR           (STM32_ETHERNET_BASE+STM32_ETH_MMCTGFCR_OFFSET)
#define STM32_ETH_MMCRFCECR          (STM32_ETHERNET_BASE+STM32_ETH_MMCRFCECR_OFFSET)
#define STM32_ETH_MMCRFAECR          (STM32_ETHERNET_BASE+STM32_ETH_MMCRFAECR_OFFSET)
#define STM32_ETH_MMCRGUFCR          (STM32_ETHERNET_BASE+STM32_ETH_MMCRGUFCR_OFFSET)

/* IEEE 1588 time stamp registers */

#define STM32_ETH_PTPTSCR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTSCR_OFFSET)
#define STM32_ETH_PTPSSIR            (STM32_ETHERNET_BASE+STM32_ETH_PTPSSIR_OFFSET)
#define STM32_ETH_PTPTSHR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTSHR_OFFSET)
#define STM32_ETH_PTPTSLR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTSLR_OFFSET)
#define STM32_ETH_PTPTSHUR           (STM32_ETHERNET_BASE+STM32_ETH_PTPTSHUR_OFFSET)
#define STM32_ETH_PTPTSLUR           (STM32_ETHERNET_BASE+STM32_ETH_PTPTSLUR_OFFSET)
#define STM32_ETH_PTPTSAR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTSAR_OFFSET)
#define STM32_ETH_PTPTTHR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTTHR_OFFSET)
#define STM32_ETH_PTPTTLR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTTLR_OFFSET)
#define STM32_ETH_PTPTSSR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTSSR_OFFSET)
#define STM32_ETH_PTPPPSCR           (STM32_ETHERNET_BASE+STM32_ETH_PTPPPSCR_OFFSET)

/* DMA Registers */

#define STM32_ETH_DMABMR             (STM32_ETHERNET_BASE+STM32_ETH_DMABMR_OFFSET)
#define STM32_ETH_DMATPDR            (STM32_ETHERNET_BASE+STM32_ETH_DMATPDR_OFFSET)
#define STM32_ETH_DMARPDR            (STM32_ETHERNET_BASE+STM32_ETH_DMARPDR_OFFSET)
#define STM32_ETH_DMARDLAR           (STM32_ETHERNET_BASE+STM32_ETH_DMARDLAR_OFFSET)
#define STM32_ETH_DMATDLAR           (STM32_ETHERNET_BASE+STM32_ETH_DMATDLAR_OFFSET)
#define STM32_ETH_DMASR              (STM32_ETHERNET_BASE+STM32_ETH_DMASR_OFFSET)
#define STM32_ETH_DMAOMR             (STM32_ETHERNET_BASE+STM32_ETH_DMAOMR_OFFSET)
#define STM32_ETH_DMAIER             (STM32_ETHERNET_BASE+STM32_ETH_DMAIER_OFFSET)
#define STM32_ETH_DMAMFBOC           (STM32_ETHERNET_BASE+STM32_ETH_DMAMFBOC_OFFSET)
#define STM32_ETH_DMARSWTR           (STM32_ETHERNET_BASE+STM32_ETH_DMARSWTR_OFFSET)
#define STM32_ETH_DMACHTDR           (STM32_ETHERNET_BASE+STM32_ETH_DMACHTDR_OFFSET)
#define STM32_ETH_DMACHRDR           (STM32_ETHERNET_BASE+STM32_ETH_DMACHRDR_OFFSET)
#define STM32_ETH_DMACHTBAR          (STM32_ETHERNET_BASE+STM32_ETH_DMACHTBAR_OFFSET)
#define STM32_ETH_DMACHRBAR          (STM32_ETHERNET_BASE+STM32_ETH_DMACHRBAR_OFFSET)

/* Register Bit-Field Definitions *******************************************************************/
/* MAC Registers */

/* Ethernet MAC configuration register */

#define ETH_MACCR_RE                 (1 << 2)  /* Bit 2:  Receiver enable */
#define ETH_MACCR_TE                 (1 << 3)  /* Bit 3:  Transmitter enable */
#define ETH_MACCR_DC                 (1 << 4)  /* Bit 4:  Deferral check */
#define ETH_MACCR_BL_SHIFT           (5)       /* Bits 5-6: Back-off limit */
#define ETH_MACCR_BL_MASK            (3 << ETH_MACCR_BL_SHIFT)
#  define ETH_MACCR_BL_10            (0 << ETH_MACCR_BL_SHIFT) /* 00: k = min (n, 10) */
#  define ETH_MACCR_BL_8             (1 << ETH_MACCR_BL_SHIFT) /* 01: k = min (n, 8) */
#  define ETH_MACCR_BL_4             (2 << ETH_MACCR_BL_SHIFT) /* 10: k = min (n, 4) */
#  define ETH_MACCR_BL_1             (3 << ETH_MACCR_BL_SHIFT) /* 11: k = min (n, 1) */
#define ETH_MACCR_APCS               (1 << 7)  /* Bit 7:  Automatic pad/CRC stripping */
#define ETH_MACCR_RD                 (1 << 9)  /* Bit 9:  Retry disable */
#define ETH_MACCR_IPCO               (1 << 10) /* Bit 10: IPv4 checksum offload */
#define ETH_MACCR_DM                 (1 << 11) /* Bit 11: Duplex mode */
#define ETH_MACCR_LM                 (1 << 12) /* Bit 12: Loopback mode */
#define ETH_MACCR_ROD                (1 << 13) /* Bit 13: Receive own disable */
#define ETH_MACCR_FES                (1 << 14) /* Bit 14: Fast Ethernet speed */
#define ETH_MACCR_CSD                (1 << 16) /* Bit 16: Carrier sense disable */
#define ETH_MACCR_IFG_SHIFT          (17)      /* Bits 17-19: Interframe gap */
#define ETH_MACCR_IFG_MASK           (7 << ETH_MACCR_IFG_SHIFT)
#  define ETH_MACCR_IFG(n)           ((12-((n) >> 3)) << ETH_MACCR_IFG_SHIFT) /* n bit times, n=40,48,..96 */
#define ETH_MACCR_JD                 (1 << 22) /* Bit 22: Jabber disable */
#define ETH_MACCR_WD                 (1 << 23) /* Bit 23: Watchdog disable */
#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define ETH_MACCR_CSTF             (1 << 25) /* Bits 25: CRC stripping for Type frames */
#endif

/* Ethernet MAC frame filter register */

#define ETH_MACFFR_PM                (1 << 0)  /* Bit 0: Promiscuous mode */
#define ETH_MACFFR_HU                (1 << 1)  /* Bit 1: Hash unicast */
#define ETH_MACFFR_HM                (1 << 2)  /* Bit 2: Hash multicast */
#define ETH_MACFFR_DAIF              (1 << 3)  /* Bit 3: Destination address inverse filtering */
#define ETH_MACFFR_PAM               (1 << 4)  /* Bit 4: Pass all multicast */
#define ETH_MACFFR_BFD               (1 << 5)  /* Bit 5: Broadcast frames disable */
#define ETH_MACFFR_PCF_SHIFT         (6)       /* Bits 6-7: Pass control frames */
#define ETH_MACFFR_PCF_MASK          (3 << ETH_MACFFR_PCF_SHIFT)
#  define ETH_MACFFR_PCF_NONE        (0 << ETH_MACFFR_PCF_SHIFT) /* Prevents all control frames */
#  define ETH_MACFFR_PCF_PAUSE       (1 << ETH_MACFFR_PCF_SHIFT) /* Prevents all except Pause control frames */
#  define ETH_MACFFR_PCF_ALL         (2 << ETH_MACFFR_PCF_SHIFT) /* Forwards all control frames */
#  define ETH_MACFFR_PCF_FILTER      (3 << ETH_MACFFR_PCF_SHIFT) /* Forwards all that pass address filter */
#define ETH_MACFFR_SAIF              (1 << 8)  /* Bit 8: Source address inverse filtering */
#define ETH_MACFFR_SAF               (1 << 9)  /* Bit 9: Source address filter */
#define ETH_MACFFR_HPF               (1 << 10) /* Bit 10: Hash or perfect filter */
#define ETH_MACFFR_RA                (1 << 31) /* Bit 31: Receive all */

/* Ethernet MAC hash table high/low registers (32-bit values) */

/* Ethernet MAC MII address register */

#define ETH_MACMIIAR_MB              (1 << 0)  /* Bit 0: MII busy */
#define ETH_MACMIIAR_MW              (1 << 1)  /* Bit 1: MII write */
#define ETH_MACMIIAR_CR_SHIFT        (2)       /* Bits 2-4: Clock range */
#define ETH_MACMIIAR_CR_MASK         (7 << ETH_MACMIIAR_CR_SHIFT)
#if 0 /* Per the reference manual */
#  define ETH_MACMIIAR_CR_60_100     (0 << ETH_MACMIIAR_CR_SHIFT) /* 000 60-100  MHzHCLK/42 */
#  define ETH_MACMIIAR_CR_100_168    (1 << ETH_MACMIIAR_CR_SHIFT) /* 001 100-168 MHz HCLK/62 */
#  define ETH_MACMIIAR_CR_20_35      (2 << ETH_MACMIIAR_CR_SHIFT) /* 010 20-35   MHz HCLK/16 */
#  define ETH_MACMIIAR_CR_35_60      (3 << ETH_MACMIIAR_CR_SHIFT) /* 011 35-60   MHz HCLK/26 */
#else /* Per the driver example */
#  define ETH_MACMIIAR_CR_60_100     (0 << ETH_MACMIIAR_CR_SHIFT) /* 000 60-100  MHz HCLK/42 */
#  define ETH_MACMIIAR_CR_100_150    (1 << ETH_MACMIIAR_CR_SHIFT) /* 001 100-150 MHz HCLK/62 */
#  define ETH_MACMIIAR_CR_20_35      (2 << ETH_MACMIIAR_CR_SHIFT) /* 010 20-35   MHz HCLK/16 */
#  define ETH_MACMIIAR_CR_35_60      (3 << ETH_MACMIIAR_CR_SHIFT) /* 011 35-60   MHz HCLK/26 */
#  define ETH_MACMIIAR_CR_150_168    (4 << ETH_MACMIIAR_CR_SHIFT) /* 100 150-168 MHz HCLK/102 */
#endif
#define ETH_MACMIIAR_MR_SHIFT        (6)       /* Bits 6-10: MII register */
#define ETH_MACMIIAR_MR_MASK         (31 << ETH_MACMIIAR_MR_SHIFT)
#define ETH_MACMIIAR_PA_SHIFT        (11)      /* Bits 11-15: PHY address */
#define ETH_MACMIIAR_PA_MASK         (31 << ETH_MACMIIAR_PA_SHIFT)

/* Ethernet MAC MII data register */

#define ETH_MACMIIDR_MASK            (0xffff)

/* Ethernet MAC flow control register */

#define ETH_MACFCR_FCB_BPA           (1 << 0)  /* Bit 0: Flow control busy/back pressure activate */
#define ETH_MACFCR_TFCE              (1 << 1)  /* Bit 1: Transmit flow control enable */
#define ETH_MACFCR_RFCE              (1 << 2)  /* Bit 2: Receive flow control enable */
#define ETH_MACFCR_UPFD              (1 << 3)  /* Bit 3: Unicast pause frame detect */
#define ETH_MACFCR_PLT_SHIFT         (4)       /* Bits 4-5: Pause low threshold */
#define ETH_MACFCR_PLT_MASK          (3 << ETH_MACFCR_PLT_SHIFT)
#  define ETH_MACFCR_PLT_M4          (0 << ETH_MACFCR_PLT_SHIFT) /* 00 Pause - 4 slot times */
#  define ETH_MACFCR_PLT_M28         (1 << ETH_MACFCR_PLT_SHIFT) /* 01 Pause - 28 slot times */
#  define ETH_MACFCR_PLT_M144        (2 << ETH_MACFCR_PLT_SHIFT) /* 10 Pause - 144 slot times */
#  define ETH_MACFCR_PLT_M256        (3 << ETH_MACFCR_PLT_SHIFT) /* 11 Pause -s 256 slot times */
#define ETH_MACFCR_ZQPD              (1 << 7)  /* Bit 7: Zero-quanta pause disable */
#define ETH_MACFCR_PT_SHIFT          (16)      /* Bits 16-31: Pause time */
#define ETH_MACFCR_PT_MASK           (0xffff << ETH_MACFCR_PT_SHIFT)

/* Ethernet MAC VLAN tag register */

#define ETH_MACVLANTR_VLANTI_SHIFT   (0)       /* Bits 0-15: VLAN tag identifier (for receive frames) */
#define ETH_MACVLANTR_VLANTI_MASK    (0xffff << ETH_MACVLANTR_VLANTI_SHIFT)
#define ETH_MACVLANTR_VLANTC         (1 << 16) /* Bit 16: 12-bit VLAN tag comparison */

/* Ethernet MAC remote wakeup frame filter reg.  Provides 32-bit access to remote
 * remote wake-up filters.
 */

/* Ethernet MAC PMT control and status register */

#define ETH_MACPMTCSR_PD             (1 << 0)  /* Bit 0: Power down */
#define ETH_MACPMTCSR_MPE            (1 << 1)  /* Bit 1: Magic Packet enable */
#define ETH_MACPMTCSR_WFE            (1 << 2)  /* Bit 2: Wakeup frame enable */
#define ETH_MACPMTCSR_MPR            (1 << 5)  /* Bit 5: Magic packet received */
#define ETH_MACPMTCSR_WFR            (1 << 6)  /* Bit 6: Wakeup frame received */
#define ETH_MACPMTCSR_GU             (1 << 9)  /* Bit 9: Global unicast */

/* Ethernet MAC debug register */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)

#define ETH_MACDBGR_MMRPEA           (1 << 0)  /* Bit 0: MAC MII receive protocol engine active */
#define ETH_MACDBGR_MSFRWCS_SHIFT    (1)       /* Bits 1-2: MAC small FIFO read / write controllers status */
#define ETH_MACDBGR_MSFRWCS_MASK     (3 << ETH_MACDBGR_MSFRWCS_SHIFT)
#define ETH_MACDBGR_RFWRA            (1 << 4)  /* Bit 4: Rx FIFO write controller active */
#define ETH_MACDBGR_RFRCS_SHIFT      (5)       /* Bits 5-6: Rx FIFO read controller status */
#define ETH_MACDBGR_RFRCS_MASK       (3 << ETH_MACDBGR_RFRCS_SHIFT)
#  define ETH_MACDBGR_RFRCS_IDLE     (0 << ETH_MACDBGR_RFRCS_SHIFT) /* 00: IDLE state */
#  define ETH_MACDBGR_RFRCS_RFRAME   (1 << ETH_MACDBGR_RFRCS_SHIFT) /* 01: Reading frame data */
#  define ETH_MACDBGR_RFRCS_RSTATUS  (2 << ETH_MACDBGR_RFRCS_SHIFT) /* 10: Reading frame status (or time-stamp) */
#  define ETH_MACDBGR_RFRCS_FLUSHING (3 << ETH_MACDBGR_RFRCS_SHIFT) /* 11: Flushing the frame data and status */
#define ETH_MACDBGR_RFFL_SHIFT       (8)       /* Bits 8-9: Rx FIFO fill level */
#define ETH_MACDBGR_RFFL_MASK        (3 << ETH_MACDBGR_RFFL_SHIFT)
#  define ETH_MACDBGR_RFFL_EMPTY     (0 << ETH_MACDBGR_RFFL_SHIFT) /* 00: RxFIFO empty */
#  define ETH_MACDBGR_RFFL_DEACT     (1 << ETH_MACDBGR_RFFL_SHIFT) /* 01: RxFIFO fill-level below flow-control de-activate threshold */
#  define ETH_MACDBGR_RFFL_ACTIV     (2 << ETH_MACDBGR_RFFL_SHIFT) /* 10: RxFIFO fill-level above flow-control activate threshold */
#  define ETH_MACDBGR_RFFL_FULL      (3 << ETH_MACDBGR_RFFL_SHIFT) /* 11: RxFIFO full */
#define ETH_MACDBGR_MMTEA            (1 << 16) /* Bit 16: MAC MII transmit engine active */
#define ETH_MACDBGR_MTFCS_SHIFT      (17)      /* Bits 17-18: MAC transmit frame controller status */
#define ETH_MACDBGR_MTFCS_MASK       (3 << ETH_MACDBGR_MTFCS_SHIFT)
#  define ETH_MACDBGR_MTFCS_IDLE     (0 << ETH_MACDBGR_MTFCS_SHIFT) /* 00: Idle */
#  define ETH_MACDBGR_MTFCS_WAITING  (1 << ETH_MACDBGR_MTFCS_SHIFT) /* 01: Waiting for Status of previous frame or IFG/backoff period to be over */
#  define ETH_MACDBGR_MTFCS_PAUSE    (2 << ETH_MACDBGR_MTFCS_SHIFT) /* 10: Generating and transmitting a Pause control frame */
#  define ETH_MACDBGR_MTFCS_FRAME    (3 << ETH_MACDBGR_MTFCS_SHIFT) /* 11: Transferring input frame for transmission */
#define ETH_MACDBGR_MTP              (1 << 19) /* Bit 19: MAC transmitter in pause */
#define ETH_MACDBGR_TFRS_SHIFT       (20)      /* Bits 20-21: Tx FIFO read status */
#define ETH_MACDBGR_TFRS_MASK        (3 << ETH_MACDBGR_TFRS_SHIFT)
#  define ETH_MACDBGR_TFRS_IDLE      (0 << ETH_MACDBGR_TFRS_SHIFT) /* 00: Idle state */
#  define ETH_MACDBGR_TFRS_READ      (1 << ETH_MACDBGR_TFRS_SHIFT) /* 01: Read state */
#  define ETH_MACDBGR_TFRS_WAITING   (2 << ETH_MACDBGR_TFRS_SHIFT) /* 10: Waiting for TxStatus from MAC transmitter */
#  define ETH_MACDBGR_TFRS_WRITING   (3 << ETH_MACDBGR_TFRS_SHIFT) /* 11: Writing the received TxStatus or flushing the TxFIFO */
#define ETH_MACDBGR_TFWA             (1 << 22) /* Bit 22: Tx FIFO write active */
#define ETH_MACDBGR_TFNE             (1 << 24) /* Bit 24: Tx FIFO not empty */
#define ETH_MACDBGR_TFF              (1 << 25) /* Bit 25: Tx FIFO full */

#endif

/* Ethernet MAC interrupt status register */

#define ETH_MACSR_PMTS               (1 << 3)  /* Bit 3: PMT status */
#define ETH_MACSR_MMCS               (1 << 4)  /* Bit 4: MMC status */
#define ETH_MACSR_MMCRS              (1 << 5)  /* Bit 5: MMC receive status */
#define ETH_MACSR_MMCTS              (1 << 6)  /* Bit 6: MMC transmit status */
#define ETH_MACSR_TSTS               (1 << 9)  /* Bit 9: Time stamp trigger status */

/* Ethernet MAC interrupt mask register */

#define ETH_MACIMR_PMTIM             (1 << 3)  /* Bit 3: PMT interrupt mask */
#define ETH_MACIMR_TSTIM             (1 << 9)  /* Bit 9: Time stamp trigger interrupt mask */
#define ETH_MACIMR_ALLINTS           (ETH_MACIMR_PMTIM|ETH_MACIMR_TSTIM)

/* Ethernet MAC address 0 high register */

#define ETH_MACA0HR_MACA0H_SHIFT     (0)       /* Bits 0-15: MAC address0 high [47:32] */
#define ETH_MACA0HR_MACA0H_MASK      (0xffff << ETH_MACA0HR_MACA0H_SHIFT)
#define ETH_MACA0HR_MO               (1 << 31) /* Bit 31:Always  */

/* Ethernet MAC address 0 low register (MAC address0 low [31:0]) */

/* Ethernet MAC address 1 high register */

#define ETH_MACA1HR_MACA1H_SHIFT     (0)       /* Bits 0-15: MAC address1 high [47:32] */
#define ETH_MACA1HR_MACA1H_MASK      (0xffff << ETH_MACA1HR_MACA1H_SHIFT)
#define ETH_MACA1HR_MBC_SHIFT        (24)      /* Bits 24-29: Mask byte control */
#define ETH_MACA1HR_MBC_MASK         (0x3f << ETH_MACA1HR_MBC_SHIFT)
#  define ETH_MACA1HR_MBC_40_47      (0x20 << ETH_MACA1HR_MBC_SHIFT) /* Bit 29: ETH_MACA1HR [8-15] */
#  define ETH_MACA1HR_MBC_32_39      (0x10 << ETH_MACA1HR_MBC_SHIFT) /* Bit 28: ETH_MACA1HR [0-7] */
#  define ETH_MACA1HR_MBC_24_31      (0x08 << ETH_MACA1HR_MBC_SHIFT) /* Bit 27: ETH_MACA1LR [24-31] */
#  define ETH_MACA1HR_MBC_16_23      (0x04 << ETH_MACA1HR_MBC_SHIFT) /* Bit 26: ETH_MACA1LR [16-23] */
#  define ETH_MACA1HR_MBC_8_15       (0x02 << ETH_MACA1HR_MBC_SHIFT) /* Bit 25: ETH_MACA1LR [8-15] */
#  define ETH_MACA1HR_MBC_0_7        (0x01 << ETH_MACA1HR_MBC_SHIFT) /* Bit 24: ETH_MACA1LR [0-7] */
#define ETH_MACA1HR_SA               (1 << 30) /* Bit 30: Source address */
#define ETH_MACA1HR_AE               (1 << 31) /* Bit 31: Address enable */

/* Ethernet MAC address1 low register (MAC address1 low [31:0]) */

/* Ethernet MAC address 2 high register */

#define ETH_MACA2HR_MACA2H_SHIFT     (0)       /* Bits 0-15: MAC address2 high [47:32] */
#define ETH_MACA2HR_MACA2H_MASK      (0xffff << ETH_MACA2HR_MACA2H_SHIFT)
#define ETH_MACA2HR_MBC_SHIFT        (24)      /* Bits 24-29: Mask byte control */
#define ETH_MACA2HR_MBC_MASK         (0x3f << ETH_MACA2HR_MBC_SHIFT)
#  define ETH_MACA2HR_MBC_40_47      (0x20 << ETH_MACA2HR_MBC_SHIFT) /* Bit 29: ETH_MACA2HR [8-15] */
#  define ETH_MACA2HR_MBC_32_39      (0x10 << ETH_MACA2HR_MBC_SHIFT) /* Bit 28: ETH_MACA2HR [0-7] */
#  define ETH_MACA2HR_MBC_24_31      (0x08 << ETH_MACA2HR_MBC_SHIFT) /* Bit 27: ETH_MACA2LR [24-31] */
#  define ETH_MACA2HR_MBC_16_23      (0x04 << ETH_MACA2HR_MBC_SHIFT) /* Bit 26: ETH_MACA2LR [16-23] */
#  define ETH_MACA2HR_MBC_8_15       (0x02 << ETH_MACA2HR_MBC_SHIFT) /* Bit 25: ETH_MACA2LR [8-15] */
#  define ETH_MACA2HR_MBC_0_7        (0x01 << ETH_MACA2HR_MBC_SHIFT) /* Bit 24: ETH_MACA2LR [0-7] */
#define ETH_MACA2HR_SA               (1 << 30) /* Bit 30: Source address */
#define ETH_MACA2HR_AE               (1 << 31) /* Bit 31: Address enable */

/* Ethernet MAC address 2 low register (MAC address2 low [31:0]) */

/* Ethernet MAC address 3 high register */

#define ETH_MACA3HR_MACA3H_SHIFT     (0)       /* Bits 0-15: MAC address3 high [47:32] */
#define ETH_MACA3HR_MACA3H_MASK      (0xffff << ETH_MACA3HR_MACA3H_SHIFT)
#define ETH_MACA3HR_MBC_SHIFT        (24)      /* Bits 24-29: Mask byte control */
#define ETH_MACA3HR_MBC_MASK         (0x3f << ETH_MACA3HR_MBC_SHIFT)
#  define ETH_MACA3HR_MBC_40_47      (0x20 << ETH_MACA3HR_MBC_SHIFT) /* Bit 29: ETH_MACA3HR [8-15] */
#  define ETH_MACA3HR_MBC_32_39      (0x10 << ETH_MACA3HR_MBC_SHIFT) /* Bit 28: ETH_MACA3HR [0-7] */
#  define ETH_MACA3HR_MBC_24_31      (0x08 << ETH_MACA3HR_MBC_SHIFT) /* Bit 27: ETH_MACA3LR [24-31] */
#  define ETH_MACA3HR_MBC_16_23      (0x04 << ETH_MACA3HR_MBC_SHIFT) /* Bit 26: ETH_MACA3LR [16-23] */
#  define ETH_MACA3HR_MBC_8_15       (0x02 << ETH_MACA3HR_MBC_SHIFT) /* Bit 25: ETH_MACA3LR [8-15] */
#  define ETH_MACA3HR_MBC_0_7        (0x01 << ETH_MACA3HR_MBC_SHIFT) /* Bit 24: ETH_MACA3LR [0-7] */
#define ETH_MACA3HR_SA               (1 << 30) /* Bit 30: Source address */
#define ETH_MACA3HR_AE               (1 << 31) /* Bit 31: Address enable */

/* Ethernet MAC address 3 low register (MAC address3 low [31:0]) */

/* MMC Registers */

/* Ethernet MMC control register */

#define ETH_MMCCR_CR                 (1 << 0)  /* Bit 0: Counter reset */
#define ETH_MMCCR_CSR                (1 << 1)  /* Bit 1: Counter stop rollover */
#define ETH_MMCCR_ROR                (1 << 2)  /* Bit 2: Reset on read */
#define ETH_MMCCR_MCF                (1 << 3)  /* Bit 3: MMC counter freeze */
#define ETH_MMCCR_MCP                (1 << 4)  /* Bit 4: MMC counter preset */
#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define ETH_MMCCR_MCFHP            (1 << 5)  /* Bit 5: MMC counter Full-Half preset */
#endif

/* Ethernet MMC receive interrupt and interrupt mask registers */

#define ETH_MMCRI_RFCE               (1 << 5)  /* Bit 5: Received frame CRC error */
#define ETH_MMCRI_RFAE               (1 << 6)  /* Bit 6: Received frames alignment error */
#define ETH_MMCRI_RGUF               (1 << 17) /* Bit 17: Received good unicast frames */

/* Ethernet MMC transmit interrupt and interrupt mask register */

#define ETH_MMCTI_TGFSC              (1 << 14)  /* Bit 14: Transmitted good frames single collision */
#define ETH_MMCTI_TGFMSC             (1 << 15)  /* Bit 15: Transmitted good frames more single collision */
#define ETH_MMCTI_TGF                (1 << 21)  /* Bit 21: Transmitted good frames */

/* 32-bit counters:
 *
 * Ethernet MMC transmitted good frames counter register (single collision)
 * Ethernet MMC transmitted good frames counter register (multiple-collision)
 * Ethernet MMC transmitted good frames counter register
 * Ethernet MMC received frames with CRC error counter register
 * Ethernet MMC received frames with alignment error counter
 * MMC received good unicast frames counter register
 */

/* IEEE 1588 time stamp registers */

/* Ethernet PTP time stamp control register */

#define ETH_PTPTSCR_TSE              (1 << 0)  /* Bit 0:  Time stamp enable */
#define ETH_PTPTSCR_TSFCU            (1 << 1)  /* Bit 1:  Time stamp fine or coarse update */
#define ETH_PTPTSCR_TSSTI            (1 << 2)  /* Bit 2:  Time stamp system time initialize */
#define ETH_PTPTSCR_TSSTU            (1 << 3)  /* Bit 3:  Time stamp system time update */
#define ETH_PTPTSCR_TSITE            (1 << 4)  /* Bit 4:  Time stamp interrupt trigger enable */
#define ETH_PTPTSCR_TSARU            (1 << 5)  /* Bit 5:  Time stamp addend register update */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#define ETH_PTPTSCR_TSSARFE          (1 << 8)  /* Bit 8:  Time stamp snapshot for all received frames enable */
#define ETH_PTPTSCR_TSSSR            (1 << 9)  /* Bit 9:  Time stamp subsecond rollover: digital or binary rollover control */
#define ETH_PTPTSCR_TSPTPPSV2E       (1 << 10) /* Bit 10: Time stamp PTP packet snooping for version2 format enable */
#define ETH_PTPTSCR_TSSPTPOEFE       (1 << 11) /* Bit 11: Time stamp snapshot for PTP over ethernet frames enable */
#define ETH_PTPTSCR_TSSIPV6FE        (1 << 12) /* Bit 12: Time stamp snapshot for IPv6 frames enable */
#define ETH_PTPTSCR_TSSIPV4FE        (1 << 13) /* Bit 13: Time stamp snapshot for IPv4 frames enable */
#define ETH_PTPTSCR_TSSEME           (1 << 14) /* Bit 14: Time stamp snapshot for event message enable */
#define ETH_PTPTSCR_TSSMRME          (1 << 15) /* Bit 15: Time stamp snapshot for message relevant to master enable */
#define ETH_PTPTSCR_TSCNT_SHIFT      (16)      /* Bits 16-17: Time stamp clock node type */
#define ETH_PTPTSCR_TSCNT_MASK       (3 << ETH_PTPTSCR_TSCNT_SHIFT)
#  define ETH_PTPTSCR_TSCNT_ORDINARY (0 << ETH_PTPTSCR_TSCNT_SHIFT) /* 00: Ordinary clock */
#  define ETH_PTPTSCR_TSCNT_BOUNDARY (1 << ETH_PTPTSCR_TSCNT_SHIFT) /* 01: Boundary clock */
#  define ETH_PTPTSCR_TSCNT_E2E      (2 << ETH_PTPTSCR_TSCNT_SHIFT) /* 10: End-to-end transparent clock */
#  define ETH_PTPTSCR_TSCNT_P2P      (3 << ETH_PTPTSCR_TSCNT_SHIFT) /* 11: Peer-to-peer transparent clock */
#define ETH_PTPTSCR_TSPFFMAE         (1 << 18) /* Bit 18: Time stamp PTP frame filtering MAC address enable */
#endif

/* Ethernet PTP subsecond increment register */

#define ETH_PTPSSIR_MASK             (0xff)

/* Ethernet PTP time stamp high register (32-bit) */

/* Ethernet PTP time stamp low register */

#define ETH_PTPTSLR_STPNS            (1 << 31)    /* Bit 31: System time positive or negative sign */
#define ETH_PTPTSLR_MASK             (0x7fffffff) /* Bits 0-30: System time subseconds */

/* Ethernet PTP time stamp high update register (32-bit) */

/* Ethernet PTP time stamp low update register */

#define ETH_PTPTSLU_TSUPNS           (1 << 31)    /* Bit 31: System time positive or negative sign */
#define ETH_PTPTSLU_MASK             (0x7fffffff) /* Bits 0-30: Time stamp update subsecond */

/* Ethernet PTP time stamp addend register (32-bit) */
/* Ethernet PTP target time high register (32-bit) */
/* Ethernet PTP target time low register (32-bit) */

/* Ethernet PTP time stamp status register */

#define ETH_PTPTSSR_TSSO             (1 << 0)  /* Bit 0: Time stamp second overflow */
#define ETH_PTPTSSR_TSTTR            (1 << 1)  /* Bit 1: Time stamp target time reached */

/* Ethernet PTP PPS control register */

#define ETH_PTPPPSCR_PPSFREQ_SHIFT   (0)       /* Bits 0-3: PPS frequency selection */
#define ETH_PTPPPSCR_PPSFREQ_MASK    (15 << ETH_PTPPPSCR_PPSFREQ_SHIFT)
#  define ETH_PTPPPSCR_PPSFREQ_1HZ   (0 << ETH_PTPPPSCR_PPSFREQ_SHIFT)  /* 1 Hz with pulse width of 125/100 ms for binary/digital rollover */
#  define ETH_PTPPPSCR_PPSFREQ_2HZ   (1 << ETH_PTPPPSCR_PPSFREQ_SHIFT)  /* 2 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_4HZ   (2 << ETH_PTPPPSCR_PPSFREQ_SHIFT)  /* 4 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_8HZ   (3 << ETH_PTPPPSCR_PPSFREQ_SHIFT)  /* 8 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_16HZ  (4 << ETH_PTPPPSCR_PPSFREQ_SHIFT)  /* 16 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_32HZ  (5 << ETH_PTPPPSCR_PPSFREQ_SHIFT)  /* 32 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_64HZ  (6 << ETH_PTPPPSCR_PPSFREQ_SHIFT)  /* 64 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_128HZ (7 << ETH_PTPPPSCR_PPSFREQ_SHIFT)  /* 128 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_256HZ (8 << ETH_PTPPPSCR_PPSFREQ_SHIFT)  /* 256 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_512HZ (9 << ETH_PTPPPSCR_PPSFREQ_SHIFT)  /* 512 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_1KHZ  (10 << ETH_PTPPPSCR_PPSFREQ_SHIFT) /* 1024 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_2KHZ  (11 << ETH_PTPPPSCR_PPSFREQ_SHIFT) /* 2048 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_4KHZ  (12 << ETH_PTPPPSCR_PPSFREQ_SHIFT) /* 4096 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_8KHZ  (13 << ETH_PTPPPSCR_PPSFREQ_SHIFT) /* 8192 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_16KHZ (14 << ETH_PTPPPSCR_PPSFREQ_SHIFT) /* 16384 Hz with 50% duty cycle */
#  define ETH_PTPPPSCR_PPSFREQ_32KHZ (15 << ETH_PTPPPSCR_PPSFREQ_SHIFT) /* 32768 Hz with 50% duty cycle */

/* DMA Registers */

/* Ethernet DMA bus mode register */

#define ETH_DMABMR_SR                (1 << 0)  /* Bit 0: Software reset */
#define ETH_DMABMR_DA                (1 << 1)  /* Bit 1: DMA Arbitration */
#define ETH_DMABMR_DSL_SHIFT         (2)       /* Bits 2-6: Descriptor skip length */
#define ETH_DMABMR_DSL_MASK          (31 << ETH_DMABMR_DSL_SHIFT)
#  define ETH_DMABMR_DSL(n)          ((n) << ETH_DMABMR_DSL_SHIFT)
#define ETH_DMABMR_EDFE              (1 << 7)  /* Bit 7: Enhanced descriptor format enable */
#define ETH_DMABMR_PBL_SHIFT         (8)       /* Bits 8-13: Programmable burst length */
#  define ETH_DMABMR_PBL(n)          ((n) << ETH_DMABMR_PBL_SHIFT) /* n=1, 2, 4, 8, 16, 32 */
#define ETH_DMABMR_PBL_MASK          (0x3f << ETH_DMABMR_PBL_SHIFT)
#define ETH_DMABMR_RTPR_SHIFT        (14)      /* Bits 14-15: Rx Tx priority ratio */
#define ETH_DMABMR_RTPR_MASK         (3 << ETH_DMABMR_RTPR_SHIFT)
#  define ETH_DMABMR_RTPR_1TO1       (0 << ETH_DMABMR_RTPR_SHIFT) /* 00: 1:1 */
#  define ETH_DMABMR_RTPR_2TO1       (1 << ETH_DMABMR_RTPR_SHIFT) /* 01: 2:1 */
#  define ETH_DMABMR_RTPR_3TO1       (2 << ETH_DMABMR_RTPR_SHIFT) /* 10: 3:1 */
#  define ETH_DMABMR_RTPR_4TO1       (3 << ETH_DMABMR_RTPR_SHIFT) /* 11: 4:1 */
#define ETH_DMABMR_FB                (1 << 16) /* Bit 16: Fixed burst */
#define ETH_DMABMR_RDP_SHIFT         (17)      /* Bits 17-22: Rx DMA PBL */
#define ETH_DMABMR_RDP_MASK          (0x3f << ETH_DMABMR_RDP_SHIFT)
#  define ETH_DMABMR_RDP(n)          ((n) << ETH_DMABMR_RDP_SHIFT) /* n=1, 2, 4, 8, 16, 32 */
#define ETH_DMABMR_USP               (1 << 23) /* Bit 23: Use separate PBL */
#define ETH_DMABMR_FPM               (1 << 24) /* Bit 24: 4xPBL mode */
#define ETH_DMABMR_AAB               (1 << 25) /* Bit 25: Address-aligned beats */
#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define ETH_DMABMR_MB              (1 << 26) /* Bit 26: Mixed burst */
#endif

/* Ethernet DMA transmit poll demand register (32-bit) */
/* Ethernet DMA receive poll demand register (32-bit) */
/* Ethernet DMA receive descriptor list address register (32-bit address) */
/* Ethernet DMA transmit descriptor list address register (32-bit address) */

/* Interrupt bit definitions common between the DMA status register (DMASR) and
 * the DMA interrupt enable register (DMAIER).
 */

#define ETH_DMAINT_TI                (1 << 0)  /* Bit 0:  Transmit interrupt */
#define ETH_DMAINT_TPSI              (1 << 1)  /* Bit 1:  Transmit process stopped interrupt */
#define ETH_DMAINT_TBUI              (1 << 2)  /* Bit 2:  Transmit buffer unavailable interrupt */
#define ETH_DMAINT_TJTI              (1 << 3)  /* Bit 3:  Transmit jabber timeout interrupt */
#define ETH_DMAINT_ROI               (1 << 4)  /* Bit 4:  Overflow interrupt */
#define ETH_DMAINT_TUI               (1 << 5)  /* Bit 5:  Underflow interrupt */
#define ETH_DMAINT_RI                (1 << 6)  /* Bit 6:  Receive interrupt */
#define ETH_DMAINT_RBUI              (1 << 7)  /* Bit 7:  Receive buffer unavailable interrupt */
#define ETH_DMAINT_RPSI              (1 << 8)  /* Bit 8:  Receive process stopped interrupt */
#define ETH_DMAINT_RWTI              (1 << 9)  /* Bit 9:  Receive watchdog timeout interrupt */
#define ETH_DMAINT_ETI               (1 << 10) /* Bit 10: Early transmit interrupt */
#define ETH_DMAINT_FBEI              (1 << 13) /* Bit 13: Fatal bus error interrupt */
#define ETH_DMAINT_ERI               (1 << 14) /* Bit 14: Early receive interrupt */
#define ETH_DMAINT_AIS               (1 << 15) /* Bit 15: Abnormal interrupt summary */
#define ETH_DMAINT_NIS               (1 << 16) /* Bit 16: Normal interrupt summary */

/* Ethernet DMA status register (in addition to the interrupt bits above */

#define ETH_DMASR_RPS_SHIFT          (17)      /* Bits 17-19: Receive process state */
#define ETH_DMASR_RPS_MASK           (7 << ETH_DMASR_RPS_SHIFT)
#  define ETH_DMASR_RPS_STOPPED      (0 << ETH_DMASR_RPS_SHIFT) /* 000: Stopped: Reset or Stop Receive Command issued */
#  define ETH_DMASR_RPS_RXDESC       (1 << ETH_DMASR_RPS_SHIFT) /* 001: Running: Fetching receive transfer descriptor */
#  define ETH_DMASR_RPS_WAITING      (3 << ETH_DMASR_RPS_SHIFT) /* 011: Running: Waiting for receive packet */
#  define ETH_DMASR_RPS_SUSPENDED    (4 << ETH_DMASR_RPS_SHIFT) /* 100: Suspended: Receive descriptor unavailable */
#  define ETH_DMASR_RPS_CLOSING      (5 << ETH_DMASR_RPS_SHIFT) /* 101: Running: Closing receive descriptor */
#  define ETH_DMASR_RPS_TRANSFER     (6 << ETH_DMASR_RPS_SHIFT) /* 111: Running: Transferring the receive data to memory */
#define ETH_DMASR_TPS_SHIFT          (20)      /* Bits 20-22: Transmit process state */
#define ETH_DMASR_TPS_MASK           (7 << ETH_DMASR_TPS_SHIFT)
#  define ETH_DMASR_TPS_STOPPED      (0 << ETH_DMASR_TPS_SHIFT) /* 000: Stopped; Reset or Stop Transmit Command issued */
#  define ETH_DMASR_TPS_TXDESC       (1 << ETH_DMASR_TPS_SHIFT) /* 001: Running; Fetching transmit transfer descriptor */
#  define ETH_DMASR_TPS_WAITING      (2 << ETH_DMASR_TPS_SHIFT) /* 010: Running; Waiting for status */
#  define ETH_DMASR_TPS_TRANSFER     (3 << ETH_DMASR_TPS_SHIFT) /* 011: Running; Reading data and queuing to transmit (TxFIFO) */
#  define ETH_DMASR_TPS_SUSPENDED    (6 << ETH_DMASR_TPS_SHIFT) /* 110: Suspended; Transmit descriptor unavailable or buffer underflow */
#  define ETH_DMASR_TPS_CLOSING      (7 << ETH_DMASR_TPS_SHIFT) /* 111: Running; Closing transmit descriptor */
#define ETH_DMASR_EBS_SHIFT          (23)      /* Bits 23-25: Error bits status */
#define ETH_DMASR_EBS_MASK           (7 << ETH_DMASR_EBS_SHIFT)
#define ETH_DMASR_EBS_TXDMS          (1 << ETH_DMASR_EBS_SHIFT) /* Bit 23 1 Error during data transfer by TxDMA */
#define ETH_DMASR_EBS_READ           (2 << ETH_DMASR_EBS_SHIFT) /* Bit 24 1 Error during read transfer */
#define ETH_DMASR_EBS_DESC           (4 << ETH_DMASR_EBS_SHIFT) /* Bit 25 1 Error during descriptor access */
#define ETH_DMASR_MMCS               (1 << 27) /* Bit 27: MMC status */
#define ETH_DMASR_PMTS               (1 << 28) /* Bit 28: PMT status */
#define ETH_DMASR_TSTS               (1 << 29) /* Bit 29: Time stamp trigger status */

/* Ethernet DMA operation mode register */

#define ETH_DMAOMR_SR                (1 << 1)  /* Bit 1:  Start/stop receive */
#define ETH_DMAOMR_OSF               (1 << 2)  /* Bit 2:  Operate on second frame */
#define ETH_DMAOMR_RTC_SHIFT         (3)       /* Bits 3-4: Receive threshold control */
#define ETH_DMAOMR_RTC_MASK          (3 << ETH_DMAOMR_RTC_SHIFT)
#  define ETH_DMAOMR_RTC_64          (0 << ETH_DMAOMR_RTC_SHIFT)
#  define ETH_DMAOMR_RTC_32          (1 << ETH_DMAOMR_RTC_SHIFT)
#  define ETH_DMAOMR_RTC_96          (2 << ETH_DMAOMR_RTC_SHIFT)
#  define ETH_DMAOMR_RTC_128         (3 << ETH_DMAOMR_RTC_SHIFT)
#define ETH_DMAOMR_FUGF              (1 << 6)  /* Bit 6:  Forward undersized good frames */
#define ETH_DMAOMR_FEF               (1 << 7)  /* Bit 7:  Forward error frames */
#define ETH_DMAOMR_ST                (1 << 13) /* Bit 13: Start/stop transmission */
#define ETH_DMAOMR_TTC_SHIFT         (14)      /* Bits 14-16: Transmit threshold control */
#define ETH_DMAOMR_TTC_MASK          (7 << ETH_DMAOMR_TTC_SHIFT)
#  define ETH_DMAOMR_TTC_64          (0 << ETH_DMAOMR_TTC_SHIFT)
#  define ETH_DMAOMR_TTC_128         (1 << ETH_DMAOMR_TTC_SHIFT)
#  define ETH_DMAOMR_TTC_192         (2 << ETH_DMAOMR_TTC_SHIFT)
#  define ETH_DMAOMR_TTC_256         (3 << ETH_DMAOMR_TTC_SHIFT)
#  define ETH_DMAOMR_TTC_40          (4 << ETH_DMAOMR_TTC_SHIFT)
#  define ETH_DMAOMR_TTC_32          (5 << ETH_DMAOMR_TTC_SHIFT)
#  define ETH_DMAOMR_TTC_24          (6 << ETH_DMAOMR_TTC_SHIFT)
#  define ETH_DMAOMR_TTC_16          (7 << ETH_DMAOMR_TTC_SHIFT)
#define ETH_DMAOMR_FTF               (1 << 20) /* Bit 20: Flush transmit FIFO */
#define ETH_DMAOMR_TSF               (1 << 21) /* Bit 21: Transmit store and forward */
#define ETH_DMAOMR_DFRF              (1 << 24) /* Bit 24: Disable flushing of received frames */
#define ETH_DMAOMR_RSF               (1 << 25) /* Bit 25: Receive store and forward */
#define ETH_DMAOMR_DTCEFD            (1 << 26) /* Bit 26: Dropping of TCP/IP checksum error frames disable */

/* Ethernet DMA missed frame and buffer overflow counter register */

#define ETH_DMAMFBOC_MFC_SHIFT       (0)       /* Bits 0-15: Missed frames by the controller */
#define ETH_DMAMFBOC_MFC_MASK        (0xffff << ETH_DMAMFBOC_MFC_SHIFT)
#define ETH_DMAMFBOC_OMFC            (1 << 16) /* Bit 16: Overflow bit for missed frame counter */
#define ETH_DMAMFBOC_MFA_SHIFT       (17)      /* Bits 17-27: Missed frames by the application */
#define ETH_DMAMFBOC_MFA_MASK        (0x7ff << ETH_DMAMFBOC_MFA_SHIFT)
#define ETH_DMAMFBOC_OFOC            (1 << 28) /* Bit 28: Overflow bit for FIFO overflow counter */

/* Ethernet DMA receive status watchdog timer register */

#define ETH_DMARSWTR_MASK            (0xff)

/* Ethernet DMA current host transmit descriptor register (32-bit address) */
/* Ethernet DMA current host receive descriptor register (32-bit address) */
/* Ethernet DMA current host transmit buffer address register (32-bit address) */
/* Ethernet DMA current host receive buffer address register (32-bit address) */

/* DMA Descriptors **********************************************************************************/
/* TDES0: Transmit descriptor Word0 */

#define ETH_TDES0_DB                 (1 << 0)  /* Bit 0:  Deferred bit */
#define ETH_TDES0_UF                 (1 << 1)  /* Bit 1:  Underflow error */
#define ETH_TDES0_ED                 (1 << 2)  /* Bit 2:  Excessive deferral */
#define ETH_TDES0_CC_SHIFT           (3)       /* Bits 3-6: Collision count */
#define ETH_TDES0_CC_MASK            (15 << ETH_TDES0_CC_SHIFT)
#define ETH_TDES0_VF                 (1 << 7)  /* Bit 7:  VLAN frame */
#define ETH_TDES0_EC                 (1 << 8)  /* Bit 8:  Excessive collision */
#define ETH_TDES0_LCO                (1 << 9)  /* Bit 9:  Late collision */
#define ETH_TDES0_NC                 (1 << 10) /* Bit 10: No carrier */
#define ETH_TDES0_LCA                (1 << 11) /* Bit 11: Loss of carrier */
#define ETH_TDES0_IPE                (1 << 12) /* Bit 12: IP payload error */
#define ETH_TDES0_FF                 (1 << 13) /* Bit 13: Frame flushed */
#define ETH_TDES0_JT                 (1 << 14) /* Bit 14: Jabber timeout */
#define ETH_TDES0_ES                 (1 << 15) /* Bit 15: Error summary */
#define ETH_TDES0_IHE                (1 << 16) /* Bit 16: IP header error */
#define ETH_TDES0_TTSS               (1 << 17) /* Bit 17: Transmit time stamp status */
#define ETH_TDES0_TCH                (1 << 20) /* Bit 20: Second address chained */
#define ETH_TDES0_TER                (1 << 21) /* Bit 21: Transmit end of ring */
#define ETH_TDES0_CIC_SHIFT          (22)      /* Bits 22-23: Checksum insertion control */
#define ETH_TDES0_CIC_MASK           (3 << ETH_TDES0_CIC_SHIFT)
#  define ETH_TDES0_CIC_DISABLED     (0 << ETH_TDES0_CIC_SHIFT) /* Checksum disabled */
#  define ETH_TDES0_CIC_IH           (1 << ETH_TDES0_CIC_SHIFT) /* IP header checksum enabled */
#  define ETH_TDES0_CIC_IHPL         (2 << ETH_TDES0_CIC_SHIFT) /* IP header and payload checksum enabled */
#  define ETH_TDES0_CIC_ALL          (3 << ETH_TDES0_CIC_SHIFT) /* IP Header, payload, and pseudo-header checksum enabled */
#define ETH_TDES0_TTSE               (1 << 25) /* Bit 25: Transmit time stamp enable */
#define ETH_TDES0_DP                 (1 << 26) /* Bit 26: Disable pad */
#define ETH_TDES0_DC                 (1 << 27) /* Bit 27: Disable CRC */
#define ETH_TDES0_FS                 (1 << 28) /* Bit 28: First segment */
#define ETH_TDES0_LS                 (1 << 29) /* Bit 29: Last segment */
#define ETH_TDES0_IC                 (1 << 30) /* Bit 30: Interrupt on completion */
#define ETH_TDES0_OWN                (1 << 31) /* Bit 31: Own bit */

/* TDES1: Transmit descriptor Word1 */

#define ETH_TDES1_TBS1_SHIFT         (0)  /* Bits 0-12: Transmit buffer 1 size */
#define ETH_TDES1_TBS1_MASK          (0x1fff << ETH_TDES1_TBS1_SHIFT)
#define ETH_TDES1_TBS2_SHIFT         (16)  /* Bits 16-28: Transmit buffer 2 size */
#define ETH_TDES1_TBS2_MASK          (0x1fff << ETH_TDES1_TBS2_SHIFT)

/* TDES2: Transmit descriptor Word2 (32-bit address) */
/* TDES3: Transmit descriptor Word3 (32-bit address) */
/* TDES6: Transmit descriptor Word6 (32-bit time stamp) */
/* TDES7: Transmit descriptor Word7 (32-bit time stamp) */

/* RDES0: Receive descriptor Word0 */

#define ETH_RDES0_PCE                (1 << 0)  /* Bit 0:  Payload checksum error */
#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define ETH_RDES0_ESA              (1 << 0)  /* Bit 0:  Extended status available */
#endif
#define ETH_RDES0_CE                 (1 << 1)  /* Bit 1:  CRC error */
#define ETH_RDES0_DBE                (1 << 2)  /* Bit 2:  Dribble bit error */
#define ETH_RDES0_RE                 (1 << 3)  /* Bit 3:  Receive error */
#define ETH_RDES0_RWT                (1 << 4)  /* Bit 4:  Receive watchdog timeout */
#define ETH_RDES0_FT                 (1 << 5)  /* Bit 5:  Frame type */
#define ETH_RDES0_LCO                (1 << 6)  /* Bit 6:  Late collision */
#define ETH_RDES0_TSV                (1 << 7)  /* Bit 7:  Time stamp valid */
#define ETH_RDES0_IPHCE              (1 << 7)  /* Bit 7:  IPv header checksum error */
#define ETH_RDES0_LS                 (1 << 8)  /* Bit 8:  Last descriptor */
#define ETH_RDES0_FS                 (1 << 9)  /* Bit 9:  First descriptor */
#define ETH_RDES0_VLAN               (1 << 10) /* Bit 10: VLAN tag */
#define ETH_RDES0_OE                 (1 << 11) /* Bit 11: Overflow error */
#define ETH_RDES0_LE                 (1 << 12) /* Bit 12: Length error */
#define ETH_RDES0_SAF                (1 << 13) /* Bit 13: Source address filter fail */
#define ETH_RDES0_DE                 (1 << 14) /* Bit 14: Descriptor error */
#define ETH_RDES0_ES                 (1 << 15) /* Bit 15: Error summary */
#define ETH_RDES0_FL_SHIFT           (16)      /* Bits 16-29: Frame length */
#define ETH_RDES0_FL_MASK            (0x3fff << ETH_RDES0_FL_SHIFT)
#define ETH_RDES0_AFM                (1 << 30) /* Bit 30: Destination address filter fail */
#define ETH_RDES0_OWN                (1 << 31) /* Bit 31: Own bit */

/* RDES1: Receive descriptor Word1 */

#define ETH_RDES1_RBS1_SHIFT         (0)       /* Bits 0-12: Receive buffer 1 size */
#define ETH_RDES1_RBS1_MASK          (0x1fff << ETH_RDES1_RBS1_SHIFT)
                                               /* Bit 13: Reserved */
#define ETH_RDES1_RCH                (1 << 14) /* Bit 14: Second address chained */
#define ETH_RDES1_RER                (1 << 15) /* Bit 15: Receive end of ring */
#define ETH_RDES1_RBS2_SHIFT         (16)      /* Bits 16-28: Receive buffer 2 size */
#define ETH_RDES1_RBS2_MASK          (0x1fff << ETH_RDES1_RBS2_SHIFT)
#define ETH_RDES1_DIC                (1 << 31) /* Bit 31: Disable interrupt on completion */

/* RDES2: Receive descriptor Word2 (32-bit address) */
/* RDES3: Receive descriptor Word3 (32-bit address) */

/* RDES4: Receive descriptor Word4 */

#define ETH_RDES4_IPPT_SHIFT         (0)       /* Bits 0-2: IP payload type */
#define ETH_RDES4_IPPT_MASK          (7 << ETH_RDES4_IPPT_SHIFT)
# define ETH_RDES4_IPPT_UDP          (1 << ETH_RDES4_IPPT_SHIFT) /* UDP payload in IP datagram */
# define ETH_RDES4_IPPT_TCP          (2 << ETH_RDES4_IPPT_SHIFT) /* TCP payload in IP datagram */ 
# define ETH_RDES4_IPPT_ICMP         (3 << ETH_RDES4_IPPT_SHIFT) /* ICMP payload in IP datagram */
#define ETH_RDES4_IPHE               (1 << 3)  /* Bit 3:  IP header error */
#define ETH_RDES4_IPPE               (1 << 4)  /* Bit 4:  IP payload error */
#define ETH_RDES4_IPCB               (1 << 5)  /* Bit 5:  IP checksum bypassed */
#define ETH_RDES4_IPV4PR             (1 << 6)  /* Bit 6:  IPv4 packet received */
#define ETH_RDES4_IPV6PR             (1 << 7)  /* Bit 7:  IPv6 packet received */
#define ETH_RDES4_PMT_SHIFT          (8)       /* Bits 8-11: PTP message type */
#define ETH_RDES4_PMT_MASK           (15 << ETH_RDES4_PMT_SHIFT)
#  define ETH_RDES4_PMT_NONE         (0 << ETH_RDES4_PMT_SHIFT) /* No PTP message received */
#  define ETH_RDES4_PMT_SYNC         (1 << ETH_RDES4_PMT_SHIFT) /* SYNC (all clock types) */
#  define ETH_RDES4_PMT_FOLLOWUP     (2 << ETH_RDES4_PMT_SHIFT) /* Follow_Up (all clock types) */
#  define ETH_RDES4_PMT_DELAYREQ     (3 << ETH_RDES4_PMT_SHIFT) /* Delay_Req (all clock types) */
#  define ETH_RDES4_PMT_DELAYRESP    (4 << ETH_RDES4_PMT_SHIFT) /* Delay_Resp (all clock types) */
#  define ETH_RDES4_PMT_PDELREQAM    (5 << ETH_RDES4_PMT_SHIFT) /* Pdelay_Req (in peer-to-peer
                                                                 * transparent clock) or Announce (in
                                                                 * ordinary or boundary clock) */
#  define ETH_RDES4_PMT_PDELREQMM    (6 << ETH_RDES4_PMT_SHIFT) /* Pdelay_Resp (in peer-to-peer
                                                                 * transparent clock) or Management (in
                                                                 * ordinary or boundary clock) */
#  define ETH_RDES4_PMT_PDELREQFUS   (7 << ETH_RDES4_PMT_SHIFT) /* Pdelay_Resp_Follow_Up (in
                                                                 * peer-to-peer transparent clock) or
                                                                 * Signaling (for ordinary or boundary
                                                                 * clock) */
#define ETH_RDES4_PFT                (1 << 12) /* Bit 12: PTP frame type */
#define ETH_RDES4_PV                 (1 << 13) /* Bit 13: PTP version */

/* RDES5: Receive descriptor Word5 - Reserved */
/* RDES6: Receive descriptor Word6 (32-bit time stamp) */
/* RDES7: Receive descriptor Word7 (32-bit time stamp) */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/* Ethernet TX DMA Descriptor */ 

struct eth_txdesc_s
{
  /* Normal DMA descriptor words */

  volatile uint32_t tdes0;   /* Status */
  volatile uint32_t tdes1;   /* Control and buffer1/2 lengths */
  volatile uint32_t tdes2;   /* Buffer1 address pointer */
  volatile uint32_t tdes3;   /* Buffer2 or next descriptor address pointer */

  /* Enhanced DMA descriptor words with time stamp */

#ifdef CONFIG_STM32_ETH_ENHANCEDDESC
  volatile uint32_t tdes4;   /* Reserved */
  volatile uint32_t tdes5;   /* Reserved */
  volatile uint32_t tdes6;   /* Time Stamp Low value for transmit and receive */
  volatile uint32_t tdes7;   /* Time Stamp High value for transmit and receive */
#endif
};

/* Ethernet RX DMA Descriptor */ 

struct eth_rxdesc_s
{
  volatile uint32_t rdes0;   /* Status */
  volatile uint32_t rdes1;   /* Control and buffer1/2 lengths */
  volatile uint32_t rdes2;   /* Buffer1 address pointer */
  volatile uint32_t rdes3;   /* Buffer2 or next descriptor address pointer */

  /* Enhanced DMA descriptor words with time stamp and PTP support */

#ifdef CONFIG_STM32_ETH_ENHANCEDDESC
  volatile uint32_t rdes4;   /* Extended status for PTP receive descriptor */
  volatile uint32_t rdes5;   /* Reserved */
  volatile uint32_t rdes6;   /* Time Stamp Low value for transmit and receive */
  volatile uint32_t rdes7;   /* Time Stamp High value for transmit and receive */
#endif
};

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ASSEMBLY__ */
#endif /* STM32_NETHERNET > 0 */
#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_ETH_H */

