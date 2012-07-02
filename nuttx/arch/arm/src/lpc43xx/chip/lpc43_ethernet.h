/****************************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_ethernet.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_ETHERNET_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_ETHERNET_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Register Offsets *********************************************************************************/
/* MAC Registers */

#define LPC43_ETH_MACCFG_OFFSET      0x0000 /* MAC configuration register */
#define LPC43_ETH_MACFFLT_OFFSET     0x0004 /* MAC frame filter register */
#define LPC43_ETH_MACHTHI_OFFSET     0x0008 /* MAC hash table high register */
#define LPC43_ETH_MACHTLO_OFFSET     0x000c /* MAC hash table low register */
#define LPC43_ETH_MACMIIA_OFFSET     0x0010 /* MAC MII address register */
#define LPC43_ETH_MACMIID_OFFSET     0x0014 /* MAC MII data register */
#define LPC43_ETH_MACFC_OFFSET       0x0018 /* MAC flow control register */
#define LPC43_ETH_MACVLANT_OFFSET    0x001c /* MAC VLAN tag register */
#define LPC43_ETH_MACDBG_OFFSET      0x0024 /* MAC debug register */
#define LPC43_ETH_MACRWFFLT_OFFSET   0x0028 /* MAC remote wakeup frame filter reg */
#define LPC43_ETH_MACPMTCS_OFFSET    0x002c /* MAC PMT control and status register */
#define LPC43_ETH_MACINTR_OFFSET     0x0038 /* MAC interrupt status register */
#define LPC43_ETH_MACIM_OFFSET       0x003c /* MAC interrupt mask register */
#define LPC43_ETH_MACA0HI_OFFSET     0x0040 /* MAC address 0 high register */
#define LPC43_ETH_MACA0LO_OFFSET     0x0044 /* MAC address 0 low register */

/* IEEE 1588 time stamp registers */

#define LPC43_ETH_TSCTRL_OFFSET      0x0700 /* Time stamp control register */
#define LPC43_ETH_SSINCR_OFFSET      0x0704 /* Sub-second increment register */
#define LPC43_ETH_SECONDS_OFFSET     0x0708 /* System time seconds register */
#define LPC43_ETH_NANOSEC_OFFSET     0x070c /* System time nanoseconds register */
#define LPC43_ETH_SECUPD_OFFSET      0x0710 /* System time seconds update register */
#define LPC43_ETH_NSECUPD_OFFSET     0x0714 /* System time nanoseconds update register */
#define LPC43_ETH_ADDEND_OFFSET      0x0718 /* Time stamp addend register */
#define LPC43_ETH_TGTSEC_OFFSET      0x071c /* Target time seconds register */
#define LPC43_ETH_TGTNSEC_OFFSET     0x0720 /* Target time nanoseconds register */
#define LPC43_ETH_HIGHWORD_OFFSET    0x0724 /* System time higher word seconds register */
#define LPC43_ETH_TSSTAT_OFFSET      0x0728 /* Time stamp status register */

/* DMA Registers */

#define LPC43_ETH_DMABMODE_OFFSET    0x1000 /* DMA bus mode register */
#define LPC43_ETH_DMATXPD_OFFSET     0x1004 /* DMA transmit poll demand register */
#define LPC43_ETH_DMARXPD_OFFSET     0x1008 /* DMA receive poll demand register */
#define LPC43_ETH_DMARXDLA_OFFSET    0x100c /* DMA receive descriptor list address register */
#define LPC43_ETH_DMATXDLA_OFFSET    0x1010 /* DMA transmit descriptor list address register */
#define LPC43_ETH_DMASTAT_OFFSET     0x1014 /* DMA status register */
#define LPC43_ETH_DMAOPMODE_OFFSET   0x1018 /* DMA operation mode register */
#define LPC43_ETH_DMAINTEN_OFFSET    0x101c /* DMA interrupt enable register */
#define LPC43_ETH_DMAMFBO_OFFSET     0x1020 /* DMA missed frame and buffer overflow counter register */
#define LPC43_ETH_DMARXWDT_OFFSET    0x1024 /* DMA receive status watchdog timer register */
#define LPC43_ETH_DMACHTXD_OFFSET    0x1048 /* DMA current host transmit descriptor register */
#define LPC43_ETH_DMACHRXD_OFFSET    0x104c /* DMA current host receive descriptor register */
#define LPC43_ETH_DMACHTXBUF_OFFSET  0x1050 /* DMA current host transmit buffer address register */
#define LPC43_ETH_DMACHRXBUF_OFFSET  0x1054 /* DMA current host receive buffer address register */

/* Register Base Addresses **************************************************************************/
/* MAC Registers */

#define LPC43_ETH_MACCR              (LPC43_ETHERNET_BASE+LPC43_ETH_MACCFG_OFFSET)
#define LPC43_ETH_MACFFLT            (LPC43_ETHERNET_BASE+LPC43_ETH_MACFFLT_OFFSET)
#define LPC43_ETH_MACHTHI            (LPC43_ETHERNET_BASE+LPC43_ETH_MACHTHI_OFFSET)
#define LPC43_ETH_MACHTLO            (LPC43_ETHERNET_BASE+LPC43_ETH_MACHTLO_OFFSET)
#define LPC43_ETH_MACMIIA            (LPC43_ETHERNET_BASE+LPC43_ETH_MACMIIA_OFFSET)
#define LPC43_ETH_MACMIID            (LPC43_ETHERNET_BASE+LPC43_ETH_MACMIID_OFFSET)
#define LPC43_ETH_MACFC              (LPC43_ETHERNET_BASE+LPC43_ETH_MACFC_OFFSET)
#define LPC43_ETH_MACVLANT           (LPC43_ETHERNET_BASE+LPC43_ETH_MACVLANT_OFFSET)
#define LPC43_ETH_MACDBG             (LPC43_ETHERNET_BASE+LPC43_ETH_MACDBG_OFFSET)
#define LPC43_ETH_MACRWFFLT          (LPC43_ETHERNET_BASE+LPC43_ETH_MACRWFFLT_OFFSET)
#define LPC43_ETH_MACPMTCS           (LPC43_ETHERNET_BASE+LPC43_ETH_MACPMTCS_OFFSET)
#define LPC43_ETH_MACSR              (LPC43_ETHERNET_BASE+LPC43_ETH_MACINTR_OFFSET)
#define LPC43_ETH_MACIM              (LPC43_ETHERNET_BASE+LPC43_ETH_MACIM_OFFSET)
#define LPC43_ETH_MACA0HI            (LPC43_ETHERNET_BASE+LPC43_ETH_MACA0HI_OFFSET)
#define LPC43_ETH_MACA0LO            (LPC43_ETHERNET_BASE+LPC43_ETH_MACA0LO_OFFSET)

/* IEEE 1588 time stamp registers */

#define LPC43_ETH_TSCTRL             (LPC43_ETHERNET_BASE+LPC43_ETH_TSCTRL_OFFSET)
#define LPC43_ETH_SSINCR             (LPC43_ETHERNET_BASE+LPC43_ETH_SSINCR_OFFSET)
#define LPC43_ETH_SECONDS            (LPC43_ETHERNET_BASE+LPC43_ETH_SECONDS_OFFSET)
#define LPC43_ETH_NANOSEC            (LPC43_ETHERNET_BASE+LPC43_ETH_NANOSEC_OFFSET)
#define LPC43_ETH_SECUPD             (LPC43_ETHERNET_BASE+LPC43_ETH_SECUPD_OFFSET)
#define LPC43_ETH_NSECUPD            (LPC43_ETHERNET_BASE+LPC43_ETH_NSECUPD_OFFSET)
#define LPC43_ETH_ADDEND             (LPC43_ETHERNET_BASE+LPC43_ETH_ADDEND_OFFSET)
#define LPC43_ETH_TGTSEC             (LPC43_ETHERNET_BASE+LPC43_ETH_TGTSEC_OFFSET)
#define LPC43_ETH_TGTNSEC            (LPC43_ETHERNET_BASE+LPC43_ETH_TGTNSEC_OFFSET)
#define LPC43_ETH_HIGHWORD           (LPC43_ETHERNET_BASE+LPC43_ETH_HIGHWORD_OFFSET)
#define LPC43_ETH_TSSTAT             (LPC43_ETHERNET_BASE+LPC43_ETH_TSSTAT_OFFSET)

/* DMA Registers */

#define LPC43_ETH_DMABMODE           (LPC43_ETHERNET_BASE+LPC43_ETH_DMABMODE_OFFSET)
#define LPC43_ETH_DMATXPD            (LPC43_ETHERNET_BASE+LPC43_ETH_DMATXPD_OFFSET)
#define LPC43_ETH_DMARXPD            (LPC43_ETHERNET_BASE+LPC43_ETH_DMARXPD_OFFSET)
#define LPC43_ETH_DMARXDLA           (LPC43_ETHERNET_BASE+LPC43_ETH_DMARXDLA_OFFSET)
#define LPC43_ETH_DMATXDLA           (LPC43_ETHERNET_BASE+LPC43_ETH_DMATXDLA_OFFSET)
#define LPC43_ETH_DMASTAT            (LPC43_ETHERNET_BASE+LPC43_ETH_DMASTAT_OFFSET)
#define LPC43_ETH_DMAOPMODE          (LPC43_ETHERNET_BASE+LPC43_ETH_DMAOPMODE_OFFSET)
#define LPC43_ETH_DMAINTEN           (LPC43_ETHERNET_BASE+LPC43_ETH_DMAINTEN_OFFSET)
#define LPC43_ETH_DMAMFBO            (LPC43_ETHERNET_BASE+LPC43_ETH_DMAMFBO_OFFSET)
#define LPC43_ETH_DMARXWDT           (LPC43_ETHERNET_BASE+LPC43_ETH_DMARXWDT_OFFSET)
#define LPC43_ETH_DMACHTXD           (LPC43_ETHERNET_BASE+LPC43_ETH_DMACHTXD_OFFSET)
#define LPC43_ETH_DMACHRXD           (LPC43_ETHERNET_BASE+LPC43_ETH_DMACHRXD_OFFSET)
#define LPC43_ETH_DMACHTXBUF         (LPC43_ETHERNET_BASE+LPC43_ETH_DMACHTXBUF_OFFSET)
#define LPC43_ETH_DMACHRXBUF         (LPC43_ETHERNET_BASE+LPC43_ETH_DMACHRXBUF_OFFSET)

/* Register Bit-Field Definitions *******************************************************************/
/* MAC Registers */

/* MAC configuration register */
                                               /* Bits 0-1:  Reserved */
#define ETH_MACCFG_RE                (1 << 2)  /* Bit 2:  Receiver enable */
#define ETH_MACCFG_TE                (1 << 3)  /* Bit 3:  Transmitter enable */
#define ETH_MACCFG_DF                (1 << 4)  /* Bit 4:  Deferral check */
#define ETH_MACCFG_BL_SHIFT          (5)       /* Bits 5-6: Back-off limit */
#define ETH_MACCFG_BL_MASK           (3 << ETH_MACCFG_BL_SHIFT)
#  define ETH_MACCFG_BL_10           (0 << ETH_MACCFG_BL_SHIFT) /* 00: k = min (n, 10) */
#  define ETH_MACCFG_BL_8            (1 << ETH_MACCFG_BL_SHIFT) /* 01: k = min (n, 8) */
#  define ETH_MACCFG_BL_4            (2 << ETH_MACCFG_BL_SHIFT) /* 10: k = min (n, 4) */
#  define ETH_MACCFG_BL_1            (3 << ETH_MACCFG_BL_SHIFT) /* 11: k = min (n, 1) */
#define ETH_MACCFG_ACS               (1 << 7)  /* Bit 7:  Automatic pad/CRC stripping */
#define ETH_MACCFG_LUD               (1 << 8)  /* Bit 8:  Link up/down */
#define ETH_MACCFG_RD                (1 << 9)  /* Bit 9:  Disable Retry */
                                               /* Bit 10: Reserved */
#define ETH_MACCFG_DM                (1 << 11) /* Bit 11: Duplex mode */
#define ETH_MACCFG_LM                (1 << 12) /* Bit 12: Loopback mode */
#define ETH_MACCFG_DO                (1 << 13) /* Bit 13: Disable receive own */
#define ETH_MACCFG_FES               (1 << 14) /* Bit 14: Fast Ethernet speed */
#define ETH_MACCFG_PS                (1 << 15) /* Bit 15: Port select */
#define ETH_MACCFG_DCRS              (1 << 16) /* Bit 16: Disable carrier sense during transmission */
#define ETH_MACCFG_IFG_SHIFT         (17)      /* Bits 17-19: Interframe gap */
#define ETH_MACCFG_IFG_MASK          (7 << ETH_MACCFG_IFG_SHIFT)
#  define ETH_MACCFG_IFG(n)          ((12-((n) >> 3)) << ETH_MACCFG_IFG_SHIFT) /* n bit times, n=40,48,..96 */
#define ETH_MACCFG_JE                (1 << 20) /* Bit 20: Jumbo frame enable */
                                               /* Bit 21: Reserved */
#define ETH_MACCFG_JD                (1 << 22) /* Bit 22: Jabber disable */
#define ETH_MACCFG_WD                (1 << 23) /* Bit 23: Watchdog disable */
#define ETH_MACCFG_CSTF              (1 << 25) /* Bits 25: CRC stripping for Type frames */
                                               /* Bots 24-31: Reserved */
/* MAC frame filter register */

#define ETH_MACFFLT_PR               (1 << 0)  /* Bit 0: Promiscuous mode */
                                               /* Bits 1-2: Reserved */
#define ETH_MACFFLT_DAIF             (1 << 3)  /* Bit 3: Destination address inverse filtering */
#define ETH_MACFFLT_PM               (1 << 4)  /* Bit 4: Pass all multicast */
#define ETH_MACFFLT_DBF              (1 << 5)  /* Bit 5: Disable Broadcast Frames */
#define ETH_MACFFLT_PCF_SHIFT        (6)       /* Bits 6-7: Pass control frames */
#define ETH_MACFFLT_PCF_MASK         (3 << ETH_MACFFLT_PCF_SHIFT)
#  define ETH_MACFFLT_PCF_NONE       (0 << ETH_MACFFLT_PCF_SHIFT) /* Prevents all control frames */
#  define ETH_MACFFLT_PCF_PAUSE      (1 << ETH_MACFFLT_PCF_SHIFT) /* Prevents all except Pause control frames */
#  define ETH_MACFFLT_PCF_ALL        (2 << ETH_MACFFLT_PCF_SHIFT) /* Forwards all control frames */
#  define ETH_MACFFLT_PCF_FILTER     (3 << ETH_MACFFLT_PCF_SHIFT) /* Forwards all that pass address filter */
#define ETH_MACFFLT_SAIF             (1 << 8)  /* Bit 8: Source address inverse filtering */
#define ETH_MACFFLT_SAF              (1 << 9)  /* Bit 9: Source address filter */
                                               /* Bits 10-30: Reserved */
#define ETH_MACFFLT_RA               (1 << 31) /* Bit 31: Receive all */

/* MAC hash table high/low register (32-bit values) */

/* MAC MII address register */

#define ETH_MACMIIA_GB               (1 << 0)  /* Bit 0: MII busy */
#define ETH_MACMIIA_WR               (1 << 1)  /* Bit 1: MII write */
#define ETH_MACMIIA_CR_SHIFT         (2)       /* Bits 2-5: Clock range */
#define ETH_MACMIIA_CR_MASK          (15 << ETH_MACMIIA_CR_SHIFT)
#  define ETH_MACMIIA_CR_60_100      (0 << ETH_MACMIIA_CR_SHIFT)  /* 60-100  MHz CLK_M4_ETHERNET/42 */
#  define ETH_MACMIIA_CR_100_150     (1 << ETH_MACMIIA_CR_SHIFT)  /* 100-150 MHz CLK_M4_ETHERNET/62 */
#  define ETH_MACMIIA_CR_20_35       (2 << ETH_MACMIIA_CR_SHIFT)  /* 20-35   MHz CLK_M4_ETHERNET/16 */
#  define ETH_MACMIIA_CR_35_60       (3 << ETH_MACMIIA_CR_SHIFT)  /* 35-60   MHz CLK_M4_ETHERNET/26 */
#  define ETH_MACMIIA_CR_150_168     (4 << ETH_MACMIIA_CR_SHIFT)  /* 150-168 MHz CLK_M4_ETHERNET/102 */
#  define ETH_MACMIIA_CR_150_168     (5 << ETH_MACMIIA_CR_SHIFT)  /* 250 - 300 MHz CLK_M4_ETHERNET/124 */
#  define ETH_MACMIIA_CR_DIV42       (8 << ETH_MACMIIA_CR_SHIFT)  /* 60-100  MHz CLK_M4_ETHERNET/42 */
#  define ETH_MACMIIA_CR_DIV62       (9 << ETH_MACMIIA_CR_SHIFT)  /* 100-150 MHz CLK_M4_ETHERNET/62 */
#  define ETH_MACMIIA_CR_DIV16       (10 << ETH_MACMIIA_CR_SHIFT) /* 20-35   MHz CLK_M4_ETHERNET/16 */
#  define ETH_MACMIIA_CR_DIV26       (11 << ETH_MACMIIA_CR_SHIFT) /* 35-60   MHz CLK_M4_ETHERNET/26 */
#  define ETH_MACMIIA_CR_DIV102      (12 << ETH_MACMIIA_CR_SHIFT) /* 150-168 MHz CLK_M4_ETHERNET/102 */
#  define ETH_MACMIIA_CR_DIV124      (13 << ETH_MACMIIA_CR_SHIFT) /* 250 - 300 MHz CLK_M4_ETHERNET/124 */
#  define ETH_MACMIIA_CR_DIV42_2     (14 << ETH_MACMIIA_CR_SHIFT) /* 60-100  MHz CLK_M4_ETHERNET/42 */
#  define ETH_MACMIIA_CR_DIV62_2     (15 << ETH_MACMIIA_CR_SHIFT) /* 100-150 MHz CLK_M4_ETHERNET/62 */
#define ETH_MACMIIA_MR_SHIFT         (6)       /* Bits 6-10: MII register */
#define ETH_MACMIIA_MR_MASK          (31 << ETH_MACMIIA_MR_SHIFT)
#define ETH_MACMIIA_PA_SHIFT         (11)      /* Bits 11-15: PHY address */
#define ETH_MACMIIA_PA_MASK          (31 << ETH_MACMIIA_PA_SHIFT)
                                               /* Bits 16-31: Reserved */
/* MAC MII data register */

#define ETH_MACMIID_MASK             (0xffff)

/* MAC flow control register */

#define ETH_MACFC_FCB                (1 << 0)  /* Bit 0: Flow control busy/back pressure activate */
#define ETH_MACFC_TFE                (1 << 1)  /* Bit 1: Transmit flow control enable */
#define ETH_MACFC_RFE                (1 << 2)  /* Bit 2: Receive flow control enable */
#define ETH_MACFC_UP                 (1 << 3)  /* Bit 3: Unicast pause frame detect */
#define ETH_MACFC_PLT_SHIFT          (4)       /* Bits 4-5: Pause low threshold */
#define ETH_MACFC_PLT_MASK           (3 << ETH_MACFC_PLT_SHIFT)
#  define ETH_MACFC_PLT(n)           ((n) << ETH_MACFC_PLT_SHIFT)
                                               /* Bit 6:  Reserved */
#define ETH_MACFC_DZPQ               (1 << 7)  /* Bit 7:  Disable Zero-Quanta Pause */
                                               /* Bits 8-15: Reserved */
#define ETH_MACFC_PT_SHIFT           (16)      /* Bits 16-31: Pause time */
#define ETH_MACFC_PT_MASK            (0xffff << ETH_MACFC_PT_SHIFT)

/* MAC VLAN tag register */

#define ETH_MACVLANT_VL_SHIFT        (0)       /* Bits 0-15: VLAN tag identifier (for receive frames) */
#define ETH_MACVLANT_VL_MAS K        (0xffff << ETH_MACVLANT_VLANTI_SHIFT)
#define ETH_MACVLANT_ETV             (1 << 16) /* Bit 16: 12-bit VLAN tag comparison */

/* MAC debug register */

#define ETH_MACDBG_RXACTIVE          (1 << 0)  /* Bit 0: MAC MII receive protocol engine active */
#define ETH_MACDBG_FS0_SHIFT         (1)       /* Bits 1-2: MAC small FIFO read / write controllers status */
#define ETH_MACDBG_FS0_MASK          (3 << ETH_MACDBG_FS0_SHIFT)
#define ETH_MACDBG_RFS1              (1 << 4)  /* Bit 4: Rx FIFO write controller active */
#define ETH_MACDBG_RFS_SHIFT         (5)       /* Bits 5-6: Rx FIFO read controller status */
#define ETH_MACDBG_RFS_MASK          (3 << ETH_MACDBG_RFS_SHIFT)
#  define ETH_MACDBG_RFS_IDLE        (0 << ETH_MACDBG_RFS_SHIFT) /* 00: IDLE state */
#  define ETH_MACDBG_RFS_RFRAME      (1 << ETH_MACDBG_RFS_SHIFT) /* 01: Reading frame data */
#  define ETH_MACDBG_RFS_RSTATUS     (2 << ETH_MACDBG_RFS_SHIFT) /* 10: Reading frame status (or time-stamp) */
#  define ETH_MACDBG_RFS_FLUSHING    (3 << ETH_MACDBG_RFS_SHIFT) /* 11: Flushing the frame data and status */
                                               /* Bit 7:  Reserved */
#define ETH_MACDBG_RFFL_SHIFT        (8)       /* Bits 8-9: Rx FIFO fill level */
#define ETH_MACDBG_RFFL_MASK         (3 << ETH_MACDBG_RFFL_SHIFT)
#  define ETH_MACDBG_RFFL_EMPTY      (0 << ETH_MACDBG_RFFL_SHIFT) /* 00: RxFIFO empty */
#  define ETH_MACDBG_RFFL_DEACT      (1 << ETH_MACDBG_RFFL_SHIFT) /* 01: RxFIFO fill-level below flow-control de-activate threshold */
#  define ETH_MACDBG_RFFL_ACTIV      (2 << ETH_MACDBG_RFFL_SHIFT) /* 10: RxFIFO fill-level above flow-control activate threshold */
#  define ETH_MACDBG_RFFL_FULL       (3 << ETH_MACDBG_RFFL_SHIFT) /* 11: RxFIFO full */
                                              /* Bits 10-15: Reserved */
#define ETH_MACDBG_TXACTIVE          (1 << 16) /* Bit 16: MAC MII transmit engine active */
#define ETH_MACDBG_TXSTAT_SHIFT      (17)      /* Bits 17-18: State of the MAC transmit frame controller module */
#define ETH_MACDBG_TXSTAT_MASK       (3 << ETH_MACDBG_TXSTAT_SHIFT)
#  define ETH_MACDBG_TXSTAT_IDLE     (0 << ETH_MACDBG_TXSTAT_SHIFT) /* 00: Idle */
#  define ETH_MACDBG_TXSTAT_WAITING  (1 << ETH_MACDBG_TXSTAT_SHIFT) /* 01: Waiting for Status of previous frame or IFG/backoff period to be over */
#  define ETH_MACDBG_TXSTAT_PAUSE    (2 << ETH_MACDBG_TXSTAT_SHIFT) /* 10: Generating and transmitting a Pause control frame */
#  define ETH_MACDBG_TXSTAT_FRAME    (3 << ETH_MACDBG_TXSTAT_SHIFT) /* 11: Transferring input frame for transmission */
#define ETH_MACDBG_PAUSE             (1 << 19) /* Bit 19: MAC transmitter in pause */
#define ETH_MACDBG_TFRS_SHIFT        (20)      /* Bits 20-21: State of the TxFIFO read Controller */
#define ETH_MACDBG_TFRS_MASK         (3 << ETH_MACDBG_TFRS_SHIFT)
#  define ETH_MACDBG_TFRS_IDLE       (0 << ETH_MACDBG_TFRS_SHIFT) /* 00: Idle state */
#  define ETH_MACDBG_TFRS_READ       (1 << ETH_MACDBG_TFRS_SHIFT) /* 01: Read state */
#  define ETH_MACDBG_TFRS_WAITING    (2 << ETH_MACDBG_TFRS_SHIFT) /* 10: Waiting for TxStatus from MAC transmitter */
#  define ETH_MACDBG_TFRS_WRITING    (3 << ETH_MACDBG_TFRS_SHIFT) /* 11: Writing the received TxStatus or flushing the TxFIFO */
#define ETH_MACDBG_TFS1              (1 << 22) /* Bit 22: Tx FIFO write active */
                                               /* Bit 23: Reserved */
#define ETH_MACDBG_TFNE              (1 << 24) /* Bit 24: Tx FIFO not empty */
#define ETH_MACDBG_TFF               (1 << 25) /* Bit 25: Tx FIFO full */
                                               /* Bits 26-31: Reserved */

/* MAC remote wakeup frame filter reg.  Provides 32-bit access to remote remote wake-up filters. */

/* MAC PMT control and status register */

#define ETH_MACPMTCS_PD              (1 << 0)  /* Bit 0: Power down */
#define ETH_MACPMTCS_MPE             (1 << 1)  /* Bit 1: Magic Packet enable */
#define ETH_MACPMTCS_WFE             (1 << 2)  /* Bit 2: Wakeup frame enable */
                                               /* Bits 3-4: Reserved */
#define ETH_MACPMTCS_MPR             (1 << 5)  /* Bit 5: Magic packet received */
#define ETH_MACPMTCS_WFR             (1 << 6)  /* Bit 6: Wakeup frame received */
#define ETH_MACPMTCS_GU              (1 << 9)  /* Bit 9: Global unicast */
                                               /* Bits 10-30: Reserved */
#define ETH_MACPMTCS_WFFRPR          (1 << 31) /* Bit 31: Wake-up Frame Filter Register Pointer Reset */

/* MAC interrupt status register */
                                               /* Bits 0-2: Reserved */
#define ETH_MACINTR_PMT              (1 << 3)  /* Bit 3: PMT status */
                                               /* Bits 4-8: Reserved */
#define ETH_MACINTR_TS               (1 << 9)  /* Bit 9: Time stamp trigger status */
                                               /* Bits 10-31: Reserved */
/* MAC interrupt mask register */
                                               /* Bits 0-2: Reserved */
#define ETH_MACIM_PMTIM              (1 << 3)  /* Bit 3: PMT interrupt mask */
                                               /* Bits 4-8: Reserved */
#define ETH_MACIM_TSIM               (1 << 9)  /* Bit 9: Time stamp interrupt mask */
                                               /* Bits 10-31: Reserved */
#define ETH_MACIM_ALLINTS            (ETH_MACIM_PMTIM|ETH_MACIM_TSTIM)

/* MAC address 0 high register */

#define ETH_MACA0HI_MACA0H_SHIFT     (0)       /* Bits 0-15: MAC address0 high [47:32] */
#define ETH_MACA0HI_MACA0H_MASK      (0xffff << ETH_MACA0HI_MACA0H_SHIFT)
                                               /* Bits 16-30: Reserved */
#define ETH_MACA0HI_MO               (1 << 31) /* Bit 31: Always 1 */

/* MAC address 0 low register (MAC address0 low [31:0]) */

/* Time stamp control register */

#define ETH_TSCTRL_TSENA             (1 << 0)  /* Bit 0:  Time stamp enable */
#define ETH_TSCTRL_TSCFUPDT          (1 << 1)  /* Bit 1:  Time stamp fine or coarse update */
#define ETH_TSCTRL_TSINIT            (1 << 2)  /* Bit 2:  Time stamp initialize */
#define ETH_TSCTRL_TSUPDT            (1 << 3)  /* Bit 3:  Time stamp up */
#define ETH_TSCTRL_TSTRIG            (1 << 4)  /* Bit 4:  Time stamp interrupt trigger enable */
#define ETH_TSCTRL_TSADDREG          (1 << 5)  /* Bit 5:  Addend reg update */
                                               /* Bits 6-7: Reserved */
#define ETH_TSCTRL_TSENALL           (1 << 8)  /* Bit 8:  Enable time stamp for all frames */
#define ETH_TSCTRL_TSCTRLSSR         (1 << 9)  /* Bit 9:  Time stamp digital or binary rollover control */
#define ETH_TSCTRL_TSVER2ENA         (1 << 10) /* Bit 10: Enable PTP packet snooping for version 2 format */
#define ETH_TSCTRL_TSIPENA           (1 << 11) /* Bit 11: Enable time stamp snapshot for ptp over ethernet frames */
#define ETH_TSCTRL_TSIPV6ENA         (1 << 12) /* Bit 12: Enable time stamp snapshot for IPv6 frames */
#define ETH_TSCTRL_TSIPV4ENA         (1 << 13) /* Bit 13: Enable time stamp snapshot for IPv4 frames */
#define ETH_TSCTRL_TSEVNTENA         (1 << 14) /* Bit 14: Enable time stamp snapshot for event messages */
#define ETH_TSCTRL_TSMSTRENA         (1 << 15) /* Bit 15: Enable snapshot for messages relevant to master */
#define ETH_TSCTRL_TSCNT_SHIFT       (16)      /* Bits 16-17: Time stamp clock node type */
#define ETH_TSCTRL_TSCNT_MASK        (3 << ETH_TSCTRL_TSCNT_SHIFT)
#  define ETH_TSCTRL_TSCNT_ORDINARY  (0 << ETH_TSCTRL_TSCNT_SHIFT) /* 00: Ordinary clock */
#  define ETH_TSCTRL_TSCNT_BOUNDARY  (1 << ETH_TSCTRL_TSCNT_SHIFT) /* 01: Boundary clock */
#  define ETH_TSCTRL_TSCNT_E2E       (2 << ETH_TSCTRL_TSCNT_SHIFT) /* 10: End-to-end transparent clock */
#  define ETH_TSCTRL_TSCNT_P2P       (3 << ETH_TSCTRL_TSCNT_SHIFT) /* 11: Peer-to-peer transparent clock */
#define ETH_TSCTRL_TSENMACADDR       (1 << 18) /* Bit 18: Enable MAC address for PTP frame filtering */
                                               /* Bits 19-31: Reserved */
/* Sub-second increment register */

#define ETH_SSINCR_MASK              (0xff)    /* Bits 0-7: Sub-second increment value */
                                               /* Bits 8-31: Reserved */
/* System time seconds register (32-bit) */

/* System time nanoseconds register */

#define ETH_NANOSEC_MASK             (0x7fffffff) /* Bits 0-30: Time stamp sub seconds */
#define ETH_NANOSEC_PSNT             (1 << 31)    /* Bit 31: Positive or negative time */

/* System time seconds update register (32-bit) */

/* System time nanoseconds update register */

#define ETH_NSECUPD_MASK             (0x7fffffff) /* Bits 0-30: Time stamp sub seconds */
#define ETH_NSECUPD_ADDSUB           (1 << 31)    /* Bit 31: Add or subtract time */

/* Time stamp addend register  (32-bit) */
/* Target time seconds register (32-bit) */
/* Target time nanoseconds register (32-bit) */

#define ETH_TGTNSEC_MASK             (0x7fffffff) /* Bits 0-30: Target time stamp low */
                                                  /* Bit 31: Reserved */
/* System time higher words seconds register */

#define ETH_HIGHWORD_MASK            (0x0000ffff) /* Bits 0-15:Time stamp higher word */
#define ETH_HIGHWORD_ADDSUB          (1 << 31)    /* Bit 31: Add or subtract time */

/* Time stamp status register */

#define ETH_TSSTAT_TSSOVF            (1 << 0)  /* Bit 0: Time stamp second overflow */
#define ETH_TSSTAT_TSTARGT           (1 << 1)  /* Bit 1: Time stamp target time reached */
                                               /* Bits 2-31: Reserved */
/* DMA Registers */

/* DMA bus mode register */

#define ETH_DMABMODE_SWR              (1 << 0)  /* Bit 0: Software reset */
#define ETH_DMABMODE_DA               (1 << 1)  /* Bit 1: DMA arbitration scheme */
#define ETH_DMABMODE_DSL_SHIFT        (2)       /* Bits 2-6: Descriptor skip length */
#define ETH_DMABMODE_DSL_MASK         (31 << ETH_DMABMODE_DSL_SHIFT)
#  define ETH_DMABMODE_DSL(n)         ((n) << ETH_DMABMODE_DSL_SHIFT)
#define ETH_DMABMODE_ATDS             (1 << 7)  /* Bit 7: Alternate descriptor size */
#define ETH_DMABMODE_PBL_SHIFT        (8)       /* Bits 8-13: Programmable burst length */
#define ETH_DMABMODE_PBL_MASK         (0x3f << ETH_DMABMODE_PBL_SHIFT)
#  define ETH_DMABMODE_PBL(n)         ((n) << ETH_DMABMODE_PBL_SHIFT) /* n=1, 2, 4, 8, 16, 32 */
#define ETH_DMABMODE_PR_SHIFT         (14)      /* Bits 14-15: Rx-to-Tx priority ratio */
#define ETH_DMABMODE_PR_MASK          (3 << ETH_DMABMODE_PR_SHIFT)
#  define ETH_DMABMODE_PR_1TO1        (0 << ETH_DMABMODE_PR_SHIFT) /* 00: 1-to-1 */
#  define ETH_DMABMODE_PR_2TO1        (1 << ETH_DMABMODE_PR_SHIFT) /* 01: 2-to-1 */
#  define ETH_DMABMODE_PR_3TO1        (2 << ETH_DMABMODE_PR_SHIFT) /* 10: 3-to-1 */
#  define ETH_DMABMODE_PR_4TO1        (3 << ETH_DMABMODE_PR_SHIFT) /* 11: 4-to-1 */
#define ETH_DMABMODE_FB               (1 << 16) /* Bit 16: Fixed burst */
#define ETH_DMABMODE_RPBL_SHIFT       (17)      /* Bits 17-22: RxDMA PBL */
#define ETH_DMABMODE_RPBL_MASK        (0x3f << ETH_DMABMODE_RPBL_SHIFT)
#  define ETH_DMABMODE_RPBL(n)        ((n) << ETH_DMABMODE_RPBL_SHIFT) /* n=1, 2, 4, 8, 16, 32 */
#define ETH_DMABMODE_USP              (1 << 23) /* Bit 23: Use separate PBL */
#define ETH_DMABMODE_PBL8X            (1 << 24) /* Bit 24: 8 x PBL mode */
#define ETH_DMABMODE_AAL              (1 << 25) /* Bit 25: Address-aligned beats */
#define ETH_DMABMODE_MB               (1 << 26) /* Bit 26: Mixed burst */
#define ETH_DMABMODE_TXPR             (1 << 27) /* Bit 27: Tx DMA has higher priority than Rx DMA */
                                                /* Bits 28-31: Reserved */
/* DMA transmit poll demand register (32-bit) */
/* DMA receive poll demand register (32-bit) */
/* DMA receive descriptor list address register (32-bit address) */
/* DMA transmit descriptor list address register (32-bit address) */

/* Interrupt bit definitions common between the DMA status register (DMASTAT) and
 * the DMA interrupt enable register (DMAINTEN).
 */

#define ETH_DMAINT_TI                (1 << 0)  /* Bit 0:  Transmit interrupt */
#define ETH_DMAINT_TPS               (1 << 1)  /* Bit 1:  Transmit process stopped */
#define ETH_DMAINT_TU                (1 << 2)  /* Bit 2:  Transmit buffer unavailable */
#define ETH_DMAINT_TJT               (1 << 3)  /* Bit 3:  Transmit jabber timeout */
#define ETH_DMAINT_OVF               (1 << 4)  /* Bit 4:  Receive overflow */
#define ETH_DMAINT_UNF               (1 << 5)  /* Bit 5:  Transmit underflow */
#define ETH_DMAINT_RI                (1 << 6)  /* Bit 6:  Receive interrupt */
#define ETH_DMAINT_RU                (1 << 7)  /* Bit 7:  Receive buffer unavailable */
#define ETH_DMAINT_RPS               (1 << 8)  /* Bit 8:  Receive process stopped */
#define ETH_DMAINT_RWT               (1 << 9)  /* Bit 9:  Receive watchdog timeout */
#define ETH_DMAINT_ETI               (1 << 10) /* Bit 10: Early transmit interrupt */
                                               /* Bits 11-12: Reserved */
#define ETH_DMAINT_FBI               (1 << 13) /* Bit 13: Fatal bus error interrupt */
#define ETH_DMAINT_ERI               (1 << 14) /* Bit 14: Early receive interrupt */
#define ETH_DMAINT_AIS               (1 << 15) /* Bit 15: Abnormal interrupt summary */
#define ETH_DMAINT_NIS               (1 << 16) /* Bit 16: Normal interrupt summary */
                                               /* Bits 17-31: Reserved */

/* DMA operation mode register */
                                               /* Bit 0:  Reserved */
#define ETH_DMAOPMODE_SR             (1 << 1)  /* Bit 1:  Start/stop receive */
#define ETH_DMAOPMODE_OSF            (1 << 2)  /* Bit 2:  Operate on second frame */
#define ETH_DMAOPMODE_RTC_SHIFT      (3)       /* Bits 3-4: Receive threshold control */
#define ETH_DMAOPMODE_RTC_MASK       (3 << ETH_DMAOPMODE_RTC_SHIFT)
#  define ETH_DMAOPMODE_RTC_64       (0 << ETH_DMAOPMODE_RTC_SHIFT)
#  define ETH_DMAOPMODE_RTC_32       (1 << ETH_DMAOPMODE_RTC_SHIFT)
#  define ETH_DMAOPMODE_RTC_96       (2 << ETH_DMAOPMODE_RTC_SHIFT)
#  define ETH_DMAOPMODE_RTC_128      (3 << ETH_DMAOPMODE_RTC_SHIFT)
                                               /* Bit 5:  Reserved */
#define ETH_DMAOPMODE_FUF            (1 << 6)  /* Bit 6:  Forward undersized good frames */
#define ETH_DMAOPMODE_FEF            (1 << 7)  /* Bit 7:  Forward error frames */
                                               /* Bits 8-12: Reserved */
#define ETH_DMAOPMODE_ST             (1 << 13) /* Bit 13: Start/stop transmission */
#define ETH_DMAOPMODE_TTC_SHIFT      (14)      /* Bits 14-16: Transmit threshold control */
#define ETH_DMAOPMODE_TTC_MASK       (7 << ETH_DMAOPMODE_TTC_SHIFT)
#  define ETH_DMAOPMODE_TTC_64       (0 << ETH_DMAOPMODE_TTC_SHIFT)
#  define ETH_DMAOPMODE_TTC_128      (1 << ETH_DMAOPMODE_TTC_SHIFT)
#  define ETH_DMAOPMODE_TTC_192      (2 << ETH_DMAOPMODE_TTC_SHIFT)
#  define ETH_DMAOPMODE_TTC_256      (3 << ETH_DMAOPMODE_TTC_SHIFT)
#  define ETH_DMAOPMODE_TTC_40       (4 << ETH_DMAOPMODE_TTC_SHIFT)
#  define ETH_DMAOPMODE_TTC_32       (5 << ETH_DMAOPMODE_TTC_SHIFT)
#  define ETH_DMAOPMODE_TTC_24       (6 << ETH_DMAOPMODE_TTC_SHIFT)
#  define ETH_DMAOPMODE_TTC_16       (7 << ETH_DMAOPMODE_TTC_SHIFT)
                                               /* Bits 17-19: Reserved */
#define ETH_DMAOPMODE_FTF            (1 << 20) /* Bit 20: Flush transmit FIFO */
                                               /* Bits 21-23: Reserved */
#define ETH_DMAOPMODE_DFF            (1 << 24) /* Bit 24: Disable flushing of received frames */
                                               /* Bits 25-31: Reserved */

/* DMA missed frame and buffer overflow counter register */

#define ETH_DMAMFBO_FMC_SHIFT       (0)       /* Bits 0-15: Number of frames missed */
#define ETH_DMAMFBO_FMC_MASK        (0xffff << ETH_DMAMFBO_FMC_SHIFT)
#define ETH_DMAMFBO_OC              (1 << 16) /* Bit 16: Overflow bit for missed frame counter */
#define ETH_DMAMFBO_FMA_SHIFT       (17)      /* Bits 17-27: Number of frames missed by the application */
#define ETH_DMAMFBO_FMA_MASK        (0x7ff << ETH_DMAMFBO_FMA_SHIFT)
#define ETH_DMAMFBO_OF              (1 << 28) /* Bit 28: Overflow bit for FIFO overflow counter */
                                              /* Bits 29-31: Reserved */
/* DMA receive status watchdog timer register */

#define ETH_DMARXWDT_MASK            (0xff)   /* Bits 9-6: RI watchdog timeout */
                                              /* Bits 8-31: Reserved */
/* DMA current host transmit descriptor register (32-bit address) */
/* DMA current host receive descriptor register (32-bit address) */
/* DMA current host transmit buffer address register (32-bit address) */
/* DMA current host receive buffer address register (32-bit address) */

/* DMA Descriptors **********************************************************************************/
/* TDES0: Transmit descriptor Word0 */

#define ETH_TDES0_DB                 (1 << 0)  /* Bit 0:  Deferred bit */
#define ETH_TDES0_UF                 (1 << 1)  /* Bit 1:  Underflow error */
#define ETH_TDES0_ED                 (1 << 2)  /* Bit 2:  Excessive deferral */
#define ETH_TDES0_CC_SHIFT           (3)       /* Bits 3-6: Collision count */
#define ETH_TDES0_CC_MASK            (15 << ETH_TDES0_CC_SHIFT)
#define ETH_TDES0_VF                 (1 << 7)  /* Bit 7:  VLAN frame */
#define ETH_TDES0_EC                 (1 << 8)  /* Bit 8:  Excessive collision */
#define ETH_TDES0_LC                 (1 << 9)  /* Bit 9:  Late collision */
#define ETH_TDES0_NC                 (1 << 10) /* Bit 10: No carrier */
#define ETH_TDES0_LC                 (1 << 11) /* Bit 11: Loss of carrier */
#define ETH_TDES0_IPE                (1 << 12) /* Bit 12: IP payload error */
#define ETH_TDES0_FF                 (1 << 13) /* Bit 13: Frame flushed */
#define ETH_TDES0_JT                 (1 << 14) /* Bit 14: Jabber timeout */
#define ETH_TDES0_ES                 (1 << 15) /* Bit 15: Error summary */
#define ETH_TDES0_IHE                (1 << 16) /* Bit 16: IP header error */
#define ETH_TDES0_TTSS               (1 << 17) /* Bit 17: Transmit time stamp status */
                                               /* Bits 18-19: Reserved */
#define ETH_TDES0_TCH                (1 << 20) /* Bit 20: Second address chained */
#define ETH_TDES0_TER                (1 << 21) /* Bit 21: Transmit end of ring */
                                               /* Bits 22-24: Reserved */
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

#define ETH_RDES0_ESA                (1 << 0)  /* Bit 0:  Extended status available */
#define ETH_RDES0_CE                 (1 << 1)  /* Bit 1:  CRC error */
#define ETH_RDES0_DE                 (1 << 2)  /* Bit 2:  Dribble bit error */
#define ETH_RDES0_RE                 (1 << 3)  /* Bit 3:  Receive error */
#define ETH_RDES0_RWT                (1 << 4)  /* Bit 4:  Receive watchdog timeout */
#define ETH_RDES0_FT                 (1 << 5)  /* Bit 5:  Frame type */
#define ETH_RDES0_LC                 (1 << 6)  /* Bit 6:  Late collision */
#define ETH_RDES0_TSA                (1 << 7)  /* Bit 7:  Time stamp available */
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
                                               /* Bit 29-31: Reserved */
/* RDES2: Receive descriptor Word2 (32-bit address) */
/* RDES3: Receive descriptor Word3 (32-bit address) */

/* RDES4: Receive descriptor Word4 */

                                               /* Bits 0-5: Reserved */
#define ETH_RDES4_IPV4               (1 << 6)  /* Bit 6:  IPv4 packet received */
#define ETH_RDES4_IPV6               (1 << 7)  /* Bit 7:  IPv6 packet received */
#define ETH_RDES4_MT_SHIFT           (8)       /* Bits 8-11: Message type */
#define ETH_RDES4_MT_MASK            (15 << ETH_RDES4_MT_SHIFT)
#  define ETH_RDES4_MT_NONE          (0 << ETH_RDES4_MT_SHIFT)  /* No PTP message received */
#  define ETH_RDES4_MT_SYNC          (1 << ETH_RDES4_MT_SHIFT)  /* SYNC (all clock types) */
#  define ETH_RDES4_MT_FOLLOWUP      (2 << ETH_RDES4_MT_SHIFT)  /* Follow_Up (all clock types) */
#  define ETH_RDES4_MT_DELAYREQ      (3 << ETH_RDES4_MT_SHIFT)  /* Delay_Req (all clock types) */
#  define ETH_RDES4_MT_DELAYRESP     (4 << ETH_RDES4_MT_SHIFT)  /* Delay_Resp (all clock types) */
#  define ETH_RDES4_MT_PDELREQAM     (5 << ETH_RDES4_MT_SHIFT)  /* Pdelay_Req (in peer-to-peer
                                                                 * transparent clock) */
#  define ETH_RDES4_MT_PDELREQMM     (6 << ETH_RDES4_MT_SHIFT)  /* Pdelay_Resp (in peer-to-peer
                                                                 * transparent clock) */
#  define ETH_RDES4_MT_PDELREQFUS    (7 << ETH_RDES4_MT_SHIFT)  /* Pdelay_Resp_Follow_Up (in
                                                                 * peer-to-peer transparent clock) */
#  define ETH_RDES4_MT_PDELREQFUS    (8 << ETH_RDES4_MT_SHIFT)  /* Announce */
#  define ETH_RDES4_MT_PDELREQFUS    (9 << ETH_RDES4_MT_SHIFT)  /* Management */
#  define ETH_RDES4_MT_PDELREQFUS    (10 << ETH_RDES4_MT_SHIFT) /* Signaling */
#  define ETH_RDES4_MT_PDELREQFUS    (15 << ETH_RDES4_MT_SHIFT) /* PTP packet with Reserved message type */
#define ETH_RDES4_PTPTYPE            (1 << 12) /* Bit 12: PTP frame type */
#define ETH_RDES4_PTPVERSION         (1 << 13) /* Bit 13: PTP version */
                                               /* Bits 14-31:  Reserved */

/* RDES5: Receive descriptor Word5 - Reserved */
/* RDES6: Receive descriptor Word6 (32-bit time stamp) */
/* RDES7: Receive descriptor Word7 (32-bit time stamp) */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/* Ethernet TX DMA Descriptor.  Descriptor size can be 4 DWORDS (16 bytes) or 8 DWORDS (32 bytes)
 * depending on the setting of the ATDS bit in the DMA Bus Mode register.
 */ 

struct eth_txdesc_s
{
  /* Normal DMA descriptor words */

  volatile uint32_t tdes0;   /* Status */
  volatile uint32_t tdes1;   /* Control and buffer1/2 lengths */
  volatile uint32_t tdes2;   /* Buffer1 address pointer */
  volatile uint32_t tdes3;   /* Buffer2 or next descriptor address pointer */

  /* Alternate DMA descriptor with time stamp */

#ifdef CONFIG_LPC43_ETH_ALTDESC
  volatile uint32_t tdes4;   /* Reserved */
  volatile uint32_t tdes5;   /* Reserved */
  volatile uint32_t tdes6;   /* Time Stamp Low value for transmit and receive */
  volatile uint32_t tdes7;   /* Time Stamp High value for transmit and receive */
#endif
};

/* Ethernet RX DMA Descriptor.  Descriptor size can be 4 DWORDS (16 bytes) or 8 DWORDS (32 bytes)
 * depending on the setting of the ATDS bit in the DMA Bus Mode register.
 */ 

struct eth_rxdesc_s
{
  volatile uint32_t rdes0;   /* Status */
  volatile uint32_t rdes1;   /* Control and buffer1/2 lengths */
  volatile uint32_t rdes2;   /* Buffer1 address pointer */
  volatile uint32_t rdes3;   /* Buffer2 or next descriptor address pointer */

  /* Alternate DMA descriptor with time stamp and PTP support */

#ifdef CONFIG_LPC43_ETH_ALTDESC
  volatile uint32_t rdes4;   /* Extended status for PTP receive descriptor */
  volatile uint32_t rdes5;   /* Reserved */
  volatile uint32_t rdes6;   /* Time Stamp Low value for transmit and receive */
  volatile uint32_t rdes7;   /* Time Stamp High value for transmit and receive */
#endif
};

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#endif /* __ASSEMBLY__ */
#endif /* LPC43_NETHERNET > 0 */
#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_ETHERNET_H */

