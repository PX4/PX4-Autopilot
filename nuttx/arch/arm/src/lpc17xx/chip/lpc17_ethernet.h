/************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc17_ethernet.h
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_ETHERNET_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_ETHERNET_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* MAC registers */

#define LPC17_ETH_MAC1_OFFSET       0x0000 /* MAC configuration register 1 */
#define LPC17_ETH_MAC2_OFFSET       0x0004 /* MAC configuration register 2 */
#define LPC17_ETH_IPGT_OFFSET       0x0008 /* Back-to-Back Inter-Packet-Gap register */
#define LPC17_ETH_IPGR_OFFSET       0x000c /* Non Back-to-Back Inter-Packet-Gap register */
#define LPC17_ETH_CLRT_OFFSET       0x0010 /* Collision window / Retry register */
#define LPC17_ETH_MAXF_OFFSET       0x0014 /* Maximum Frame register */
#define LPC17_ETH_SUPP_OFFSET       0x0018 /* PHY Support register */
#define LPC17_ETH_TEST_OFFSET       0x001c /* Test register */
#define LPC17_ETH_MCFG_OFFSET       0x0020 /* MII Mgmt Configuration register */
#define LPC17_ETH_MCMD_OFFSET       0x0024 /* MII Mgmt Command register */
#define LPC17_ETH_MADR_OFFSET       0x0028 /* MII Mgmt Address register */
#define LPC17_ETH_MWTD_OFFSET       0x002c /* MII Mgmt Write Data register */
#define LPC17_ETH_MRDD_OFFSET       0x0030 /* MII Mgmt Read Data register */
#define LPC17_ETH_MIND_OFFSET       0x0034 /* MII Mgmt Indicators register */
#define LPC17_ETH_SA0_OFFSET        0x0040 /* Station Address 0 register */
#define LPC17_ETH_SA1_OFFSET        0x0044 /* Station Address 1 register */
#define LPC17_ETH_SA2_OFFSET        0x0048 /* Station Address 2 register */

/* Control registers */

#define LPC17_ETH_CMD_OFFSET        0x0100 /* Command register */
#define LPC17_ETH_STAT_OFFSET       0x0104 /* Status register */
#define LPC17_ETH_RXDESC_OFFSET     0x0108 /* Receive descriptor base address register */
#define LPC17_ETH_RXSTAT_OFFSET     0x010c /* Receive status base address register */
#define LPC17_ETH_RXDESCNO_OFFSET   0x0110 /* Receive number of descriptors register */
#define LPC17_ETH_RXPRODIDX_OFFSET  0x0114 /* Receive produce index register */
#define LPC17_ETH_RXCONSIDX_OFFSET  0x0118 /* Receive consume index register */
#define LPC17_ETH_TXDESC_OFFSET     0x011c /* Transmit descriptor base address register */
#define LPC17_ETH_TXSTAT_OFFSET     0x0120 /* Transmit status base address register */
#define LPC17_ETH_TXDESCRNO_OFFSET  0x0124 /* Transmit number of descriptors register */
#define LPC17_ETH_TXPRODIDX_OFFSET  0x0128 /* Transmit produce index register */
#define LPC17_ETH_TXCONSIDX_OFFSET  0x012c /* Transmit consume index register */
#define LPC17_ETH_TSV0_OFFSET       0x0158 /* Transmit status vector 0 register */
#define LPC17_ETH_TSV1_OFFSET       0x015c /* Transmit status vector 1 register */
#define LPC17_ETH_RSV_OFFSET        0x0160 /* Receive status vector register */
#define LPC17_ETH_FCCNTR_OFFSET     0x0170 /* Flow control counter register */
#define LPC17_ETH_FCSTAT_OFFSET     0x0174 /* Flow control status register */

/* Rx filter registers */

#define LPC17_ETH_RXFLCTRL_OFFSET   0x0200 /* Receive filter control register */
#define LPC17_ETH_RXFLWOLST_OFFSET  0x0204 /* Receive filter WoL status register */
#define LPC17_ETH_RXFLWOLCLR_OFFSET 0x0208 /* Receive filter WoL clear register */
#define LPC17_ETH_HASHFLL_OFFSET    0x0210 /* Hash filter table LSBs register */
#define LPC17_ETH_HASHFLH_OFFSET    0x0214 /* Hash filter table MSBs register */

/* Module control registers */

#define LPC17_ETH_INTST_OFFSET      0x0fe0 /* Interrupt status register */
#define LPC17_ETH_INTEN_OFFSET      0x0fe4 /* Interrupt enable register */
#define LPC17_ETH_INTCLR_OFFSET     0x0fe8 /* Interrupt clear register */
#define LPC17_ETH_INTSET_OFFSET     0x0fec /* Interrupt set register */
#define LPC17_ETH_PWRDOWN_OFFSET    0x0ff4 /* Power-down register */

/* Register addresses ***************************************************************/
/* MAC registers */

#define LPC17_ETH_MAC1              (LPC17_ETH_BASE+LPC17_ETH_MAC1_OFFSET)
#define LPC17_ETH_MAC2              (LPC17_ETH_BASE+LPC17_ETH_MAC2_OFFSET)
#define LPC17_ETH_IPGT              (LPC17_ETH_BASE+LPC17_ETH_IPGT_OFFSET)
#define LPC17_ETH_IPGR              (LPC17_ETH_BASE+LPC17_ETH_IPGR_OFFSET)
#define LPC17_ETH_CLRT              (LPC17_ETH_BASE+LPC17_ETH_CLRT_OFFSET)
#define LPC17_ETH_MAXF              (LPC17_ETH_BASE+LPC17_ETH_MAXF_OFFSET)
#define LPC17_ETH_SUPP              (LPC17_ETH_BASE+LPC17_ETH_SUPP_OFFSET)
#define LPC17_ETH_TEST              (LPC17_ETH_BASE+LPC17_ETH_TEST_OFFSET)
#define LPC17_ETH_MCFG              (LPC17_ETH_BASE+LPC17_ETH_MCFG_OFFSET)
#define LPC17_ETH_MCMD              (LPC17_ETH_BASE+LPC17_ETH_MCMD_OFFSET)
#define LPC17_ETH_MADR              (LPC17_ETH_BASE+LPC17_ETH_MADR_OFFSET)
#define LPC17_ETH_MWTD              (LPC17_ETH_BASE+LPC17_ETH_MWTD_OFFSET)
#define LPC17_ETH_MRDD              (LPC17_ETH_BASE+LPC17_ETH_MRDD_OFFSET)
#define LPC17_ETH_MIND              (LPC17_ETH_BASE+LPC17_ETH_MIND_OFFSET)
#define LPC17_ETH_SA0               (LPC17_ETH_BASE+LPC17_ETH_SA0_OFFSET)
#define LPC17_ETH_SA1               (LPC17_ETH_BASE+LPC17_ETH_SA1_OFFSET)
#define LPC17_ETH_SA2               (LPC17_ETH_BASE+LPC17_ETH_SA2_OFFSET)

/* Control registers */

#define LPC17_ETH_CMD               (LPC17_ETH_BASE+LPC17_ETH_CMD_OFFSET)
#define LPC17_ETH_STAT              (LPC17_ETH_BASE+LPC17_ETH_STAT_OFFSET)
#define LPC17_ETH_RXDESC            (LPC17_ETH_BASE+LPC17_ETH_RXDESC_OFFSET)
#define LPC17_ETH_RXSTAT            (LPC17_ETH_BASE+LPC17_ETH_RXSTAT_OFFSET)
#define LPC17_ETH_RXDESCNO          (LPC17_ETH_BASE+LPC17_ETH_RXDESCNO_OFFSET)
#define LPC17_ETH_RXPRODIDX         (LPC17_ETH_BASE+LPC17_ETH_RXPRODIDX_OFFSET)
#define LPC17_ETH_RXCONSIDX         (LPC17_ETH_BASE+LPC17_ETH_RXCONSIDX_OFFSET)
#define LPC17_ETH_TXDESC            (LPC17_ETH_BASE+LPC17_ETH_TXDESC_OFFSET)
#define LPC17_ETH_TXSTAT            (LPC17_ETH_BASE+LPC17_ETH_TXSTAT_OFFSET)
#define LPC17_ETH_TXDESCRNO         (LPC17_ETH_BASE+LPC17_ETH_TXDESCRNO_OFFSET)
#define LPC17_ETH_TXPRODIDX         (LPC17_ETH_BASE+LPC17_ETH_TXPRODIDX_OFFSET)
#define LPC17_ETH_TXCONSIDX         (LPC17_ETH_BASE+LPC17_ETH_TXCONSIDX_OFFSET)
#define LPC17_ETH_TSV0              (LPC17_ETH_BASE+LPC17_ETH_TSV0_OFFSET)
#define LPC17_ETH_TSV1              (LPC17_ETH_BASE+LPC17_ETH_TSV1_OFFSET)
#define LPC17_ETH_RSV               (LPC17_ETH_BASE+LPC17_ETH_RSV_OFFSET)
#define LPC17_ETH_FCCNTR            (LPC17_ETH_BASE+LPC17_ETH_FCCNTR_OFFSET)
#define LPC17_ETH_FCSTAT            (LPC17_ETH_BASE+LPC17_ETH_FCSTAT_OFFSET)

/* Rx filter registers */

#define LPC17_ETH_RXFLCTRL          (LPC17_ETH_BASE+LPC17_ETH_RXFLCTRL_OFFSET)
#define LPC17_ETH_RXFLWOLST         (LPC17_ETH_BASE+LPC17_ETH_RXFLWOLST_OFFSET)
#define LPC17_ETH_RXFLWOLCLR        (LPC17_ETH_BASE+LPC17_ETH_RXFLWOLCLR_OFFSET)
#define LPC17_ETH_HASHFLL           (LPC17_ETH_BASE+LPC17_ETH_HASHFLL_OFFSET)
#define LPC17_ETH_HASHFLH           (LPC17_ETH_BASE+LPC17_ETH_HASHFLH_OFFSET)

/* Module control registers */

#define LPC17_ETH_INTST             (LPC17_ETH_BASE+LPC17_ETH_INTST_OFFSET)
#define LPC17_ETH_INTEN             (LPC17_ETH_BASE+LPC17_ETH_INTEN_OFFSET)
#define LPC17_ETH_INTCLR            (LPC17_ETH_BASE+LPC17_ETH_INTCLR_OFFSET)
#define LPC17_ETH_INTSET            (LPC17_ETH_BASE+LPC17_ETH_INTSET_OFFSET)
#define LPC17_ETH_PWRDOWN           (LPC17_ETH_BASE+LPC17_ETH_PWRDOWN_OFFSET)

/* Register bit definitions *********************************************************/
/* MAC registers */
/* MAC configuration register 1 (MAC1) */

#define ETH_MAC1_RE                 (1 << 0)  /* Bit 0:  Receive enable */
#define ETH_MAC1_PARF               (1 << 1)  /* Bit 1:  Passall all receive frames */
#define ETH_MAC1_RFC                (1 << 2)  /* Bit 2:  RX flow control */
#define ETH_MAC1_TFC                (1 << 3)  /* Bit 3:  TX flow control */
#define ETH_MAC1_LPBK               (1 << 4)  /* Bit 4:  Loopback */
                                              /* Bits 5-7: Reserved */
#define ETH_MAC1_TXRST              (1 << 8)  /* Bit 8:  Reset TX */
#define ETH_MAC1_MCSTXRST           (1 << 9)  /* Bit 9:  Reset MCS/TX */
#define ETH_MAC1_RXRST              (1 << 10) /* Bit 10: Reset RX */
#define ETH_MAC1_MCSRXRST           (1 << 11) /* Bit 11: Reset MCS/RX */
                                              /* Bits 12-13: Reserved */
#define ETH_MAC1_SIMRST             (1 << 14) /* Bit 14: Simulation reset */
#define ETH_MAC1_SOFTRST            (1 << 15) /* Bit 15: Soft reset */
                                              /* Bits 16-31: Reserved */
/* MAC configuration register 2 (MAC2) */

#define ETH_MAC2_FD                 (1 << 0)  /* Bit 0:  Full duplex */
#define ETH_MAC2_FLC                (1 << 1)  /* Bit 1:  Frame length checking */
#define ETH_MAC2_HFE                (1 << 2)  /* Bit 2:  Huge frame enable */
#define ETH_MAC2_DCRC               (1 << 3)  /* Bit 3:  Delayed CRC */
#define ETH_MAC2_CRCEN              (1 << 4)  /* Bit 4:  CRC enable */
#define ETH_MAC2_PADCRCEN           (1 << 5)  /* Bit 5:  Pad/CRC enable */
#define ETH_MAC2_VLANPADEN          (1 << 6)  /* Bit 6:  VLAN pad enable */
#define ETH_MAC2_AUTOPADEN          (1 << 7)  /* Bit 7:  Auto detect pad enable */
#define ETH_MAC2_PPE                (1 << 8)  /* Bit 8:  Pure preamble enforcement */
#define ETH_MAC2_LPE                (1 << 9)  /* Bit 9:  Long preamble enforcement */
                                              /* Bits 10-11: Reserved */
#define ETH_MAC2_NBKOFF             (1 << 12) /* Bit 12: No backoff */
#define ETH_MAC2_BPNBKOFF           (1 << 13) /* Bit 13: Back pressure/no backoff */
#define ETH_MAC2_EXDEF              (1 << 14) /* Bit 14: Excess defer */
                                              /* Bits 15-31: Reserved */
/* Back-to-Back Inter-Packet-Gap register (IPGT) */

#define ETH_IPGT_SHIFT              (0)       /* Bits 0-6 */
#define ETH_IPGT_MASK               (0x7f << ETH_IPGT_SHIFT)
                                              /* Bits 7-31: Reserved */
/* Non Back-to-Back Inter-Packet-Gap register (IPGR) */

#define ETH_IPGR_GAP2_SHIFT         (0)       /* Bits 0-6: Gap part 2 */
#define ETH_IPGR_GAP2_MASK          (0x7f << ETH_IPGR_GAP2_SHIFT)
                                              /* Bit 7: Reserved */
#define ETH_IPGR_GAP1_SHIFT         (8)       /* Bits 8-18: Gap part 1 */
#define ETH_IPGR_GAP1_MASK          (0x7f << ETH_IPGR_GAP2_SHIFT)
                                              /* Bits 15-31: Reserved */
/* Collision window / Retry register (CLRT) */

#define ETH_CLRT_RMAX_SHIFT         (0)       /* Bits 0-3: Retransmission maximum */
#define ETH_CLRT_RMAX_MASK          (15 << ETH_CLRT_RMAX_SHIFT)
                                              /* Bits 4-7: Reserved */
#define ETH_CLRT_COLWIN_SHIFT       (8)       /* Bits 8-13: Collision window */
#define ETH_CLRT_COLWIN_MASK        (0x3f << ETH_CLRT_COLWIN_SHIFT)
                                              /* Bits 14-31: Reserved */
/* Maximum Frame register (MAXF) */

#define ETH_MAXF_SHIFT              (0)       /* Bits 0-15 */
#define ETH_MAXF_MASK               (0xffff << ETH_MAXF_SHIFT)
                                              /* Bits 16-31: Reserved */
/* PHY Support register (SUPP) */
                                              /* Bits 0-7: Reserved */
#define ETH_SUPP_SPEED              (1 << 8)  /* Bit 8:  0=10Bps 1=100Bps */
                                              /* Bits 9-31: Reserved */
/* Test register (TEST) */

#define ETH_TEST_SPQ                (1 << 0)  /* Bit 0:  Shortcut pause quanta */
#define ETH_TEST_TP                 (1 << 1)  /* Bit 1:  Test pause */
#define ETH_TEST_TBP                (1 << 2)  /* Bit 2:  Test packpressure */
                                              /* Bits 3-31: Reserved */
/* MII Mgmt Configuration register (MCFG) */

#define ETH_MCFG_SCANINC            (1 << 0)  /* Bit 0:  Scan increment */
#define ETH_MCFG_SUPPRE             (1 << 1)  /* Bit 1:  Suppress preamble */
#define ETH_MCFG_CLKSEL_SHIFT       (2)       /* Bits 2-5: Clock select */
#define ETH_MCFG_CLKSEL_MASK        (15 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV4      (0 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV6      (2 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV8      (3 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV10     (4 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV14     (5 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV20     (6 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV28     (7 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV36     (8 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV40     (9 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV44     (10 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV48     (11 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV52     (12 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV56     (13 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV60     (14 << ETH_MCFG_CLKSEL_SHIFT)
#  define ETH_MCFG_CLKSEL_DIV64     (15 << ETH_MCFG_CLKSEL_SHIFT)
                                              /* Bits 6-14: Reserved */
#define ETH_MCFG_MIIRST             (1 << 15) /* Bit 15: Reset MII mgmt */
                                              /* Bits 16-31: Reserved */
/* MII Mgmt Command register (MCMD) */

#define ETH_MCMD_READ               (1 << 0)  /* Bit 0:  Single read cycle */
#define ETH_MCMD_SCAN               (1 << 1)  /* Bit 1:  Continuous read cycles */
                                              /* Bits 2-31: Reserved */
#define ETH_MCMD_WRITE              (0)

/* MII Mgmt Address register (MADR) */

#define ETH_MADR_REGADDR_SHIFT      (0)       /* Bits 0-4: Register address */
#define ETH_MADR_REGADDR_MASK       (31 << ETH_MADR_REGADDR_SHIFT)
                                              /* Bits 7-5: Reserved */
#define ETH_MADR_PHYADDR_SHIFT      (8)       /* Bits 8-12: PHY address */
#define ETH_MADR_PHYADDR_MASK       (31 << ETH_MADR_PHYADDR_SHIFT)
                                              /* Bits 13-31: Reserved */
/* MII Mgmt Write Data register (MWTD) */

#define ETH_MWTD_SHIFT              (0)       /* Bits 0-15 */
#define ETH_MWTD_MASK               (0xffff << ETH_MWTD_SHIFT)
                                              /* Bits 16-31: Reserved */
/* MII Mgmt Read Data register (MRDD) */

#define ETH_MRDD_SHIFT              (0)       /* Bits 0-15 */
#define ETH_MRDD_MASK               (0xffff << ETH_MRDD_SHIFT)
                                              /* Bits 16-31: Reserved */
/* MII Mgmt Indicators register (MIND) */

#define ETH_MIND_BUSY               (1 << 0)  /* Bit 0:  Busy */
#define ETH_MIND_SCANNING           (1 << 1)  /* Bit 1:  Scanning */
#define ETH_MIND_NVALID             (1 << 2)  /* Bit 2:  Not valid */
#define ETH_MIND_MIIFAIL            (1 << 3)  /* Bit 3:  MII link fail */
                                              /* Bits 4-31: Reserved */
/* Station Address 0 register (SA0) */

#define ETH_SA0_OCTET2_SHIFT        (0)       /* Bits 0-7: Station address 2nd octet */
#define ETH_SA0_OCTET2_MASK         (0xff << ETH_SA0_OCTET2_SHIFT)
#define ETH_SA0_OCTET1_SHIFT        (8)       /* Bits 8-15: Station address 1st octet */
#define ETH_SA0_OCTET1_MASK         (0xff << ETH_SA0_OCTET1_SHIFT)
                                              /* Bits 16-31: Reserved */
/* Station Address 1 register (SA1) */

#define ETH_SA1_OCTET4_SHIFT        (0)       /* Bits 0-7: Station address 4th octet */
#define ETH_SA1_OCTET4_MASK         (0xff << ETH_SA0_OCTET4_SHIFT)
#define ETH_SA1_OCTET3_SHIFT        (8)       /* Bits 8-15: Station address 3rd octet */
#define ETH_SA1_OCTET3_MASK         (0xff << ETH_SA0_OCTET3_SHIFT)
                                              /* Bits 16-31: Reserved */
/* Station Address 2 register (SA2) */

#define ETH_SA2_OCTET6_SHIFT        (0)       /* Bits 0-7: Station address 5th octet */
#define ETH_SA2_OCTET6_MASK         (0xff << ETH_SA0_OCTET6_SHIFT)
#define ETH_SA2_OCTET5_SHIFT        (8)       /* Bits 8-15: Station address 6th octet */
#define ETH_SA2_OCTET5_MASK         (0xff << ETH_SA0_OCTET5_SHIFT)
                                              /* Bits 16-31: Reserved */
/* Control registers */
/* Command register (CMD) */

#define ETH_CMD_RXEN                (1 << 0)  /* Bit 0:  Receive enable */
#define ETH_CMD_TXEN                (1 << 1)  /* Bit 1:  Transmit enable */
                                              /* Bit 2:  Reserved */
#define ETH_CMD_REGRST              (1 << 3)  /* Bit 3:  Reset host registers */
#define ETH_CMD_TXRST               (1 << 4)  /* Bit 4:  Reset transmit datapath */
#define ETH_CMD_RXRST               (1 << 5)  /* Bit 5:  Reset receive datapath */
#define ETH_CMD_PRFRAME             (1 << 6)  /* Bit 6:  Pass run frame */
#define ETH_CMD_PRFILTER            (1 << 7)  /* Bit 7:  Pass RX filter */
#define ETH_CMD_TXFC                (1 << 8)  /* Bit 8:  TX flow control */
#define ETH_CMD_RMII                (1 << 9)  /* Bit 9:  RMII mode */
#define ETH_CMD_FD                  (1 << 10) /* Bit 10: Full duplex */
                                              /* Bits 11-31: Reserved */
/* Status register */

#define ETH_STAT_RX                 (1 << 0)  /* Bit 0:  RX status */
#define ETH_STAT_TX                 (1 << 1)  /* Bit 1:  TX status */
                                              /* Bits 2-31: Reserved */
/* Receive descriptor base address register (RXDESC)
 *
 * The receive descriptor base address is a byte address aligned to a word
 * boundary i.e. LSB 1:0 are fixed to 00. The register contains the lowest
 * address in the array of descriptors.
 */

/* Receive status base address register (RXSTAT)
 *
 * The receive status base address is a byte address aligned to a double word
 * boundary i.e. LSB 2:0 are fixed to 000.
 */

/* Receive number of descriptors register (RXDESCNO) */

#define ETH_RXDESCNO_SHIFT          (0)       /* Bits 0-15 */
#define ETH_RXDESCNO_MASK           (0xffff << ETH_RXDESCNO_SHIFT)
                                              /* Bits 16-31: Reserved */
/* Receive produce index register (RXPRODIDX) */

#define ETH_RXPRODIDX_SHIFT         (0)       /* Bits 0-15 */
#define ETH_RXPRODIDX_MASK          (0xffff << ETH_RXPRODIDX_SHIFT)
                                              /* Bits 16-31: Reserved */
/* Receive consume index register (RXCONSIDX) */

#define ETH_RXCONSIDX_SHIFT         (0)       /* Bits 0-15 */
#define ETH_RXCONSIDX_MASK          (0xffff << ETH_RXPRODIDX_SHIFT)
                                              /* Bits 16-31: Reserved */
/* Transmit descriptor base address register (TXDESC)
 *
 *  The transmit descriptor base address is a byte address aligned to a word
 *  boundary i.e. LSB 1:0 are fixed to 00. The register contains the lowest
 * address in the array of descriptors.
 */

/* Transmit status base address register (TXSTAT)
 *
 * The transmit status base address is a byte address aligned to a word
 * boundary i.e. LSB1:0 are fixed to 00. The register contains the lowest
 * address in the array of statuses.
 */

/* Transmit number of descriptors register (TXDESCRNO) */

#define ETH_TXDESCRNO_SHIFT         (0)       /* Bits 0-15 */
#define ETH_TXDESCRNO_MASK          (0xffff << ETH_TXDESCRNO_SHIFT)
                                              /* Bits 16-31: Reserved */
/* Transmit produce index register (TXPRODIDX) */

#define ETH_TXPRODIDX_SHIFT         (0)       /* Bits 0-15 */
#define ETH_TXPRODIDX_MASK          (0xffff << ETH_TXPRODIDX_SHIFT)
                                              /* Bits 16-31: Reserved */
/* Transmit consume index register (TXCONSIDX) */

#define ETH_TXCONSIDX_SHIFT         (0)       /* Bits 0-15 */
#define ETH_TXCONSIDX_MASK          (0xffff << ETH_TXPRODIDX_SHIFT)
                                              /* Bits 16-31: Reserved */
/* Transmit status vector 0 register (TSV0) */

#define ETH_TSV0_CRCERR             (1 << 0)  /* Bit 0:  CRC error */
#define ETH_TSV0_LENCHKERR          (1 << 1)  /* Bit 1:  Length check error */
#define ETH_TSV0_LENOOR             (1 << 2)  /* Bit 2:  Length out of range */
#define ETH_TSV0_DONE               (1 << 3)  /* Bit 3:  Done */
#define ETH_TSV0_MCAST              (1 << 4)  /* Bit 4:  Multicast */
#define ETH_TSV0_BCAST              (1 << 5)  /* Bit 5:  Broadcast */
#define ETH_TSV0_PKTDEFER           (1 << 6)  /* Bit 6:  Packet Defer */
#define ETH_TSV0_EXCDEFER           (1 << 7)  /* Bit 7:  Excessive Defer */
#define ETH_TSV0_EXCCOL             (1 << 8)  /* Bit 8:  Excessive Collision */
#define ETH_TSV0_LATECOL            (1 << 9)  /* Bit 9:  Late Collision */
#define ETH_TSV0_GIANT              (1 << 10) /* Bit 10: Giant */
#define ETH_TSV0_UNDRUN             (1 << 11) /* Bit 11: Underrun */
#define ETH_TSV0_TOTBYTES_SHIFT     (12)      /* Bits 12-27:Total bytes */
#define ETH_TSV0_TOTBYTES_MASK      (0xffff << ETH_TSV0_TOTBYTES_SHIFT)
#define ETH_TSV0_CTLFRAME           (1 << 28) /* Bit 28: Control frame */
#define ETH_TSV0_PAUSE              (1 << 29) /* Bit 29: Pause */
#define ETH_TSV0_BP                 (1 << 30) /* Bit 30: Backpressure */
#define ETH_TSV0_VLAN               (1 << 31) /* Bit 31: VLAN */

/* Transmit status vector 1 register (TSV1) */

#define ETH_TSV1_TXCNT_SHIFT        (0)       /* Bits 0-15: Transmit byte count */
#define ETH_TSV1_TXCNT_MASK         (0xffff << ETH_TSV1_TXCNT_SHIFT)
#define ETH_TSV1_COLCNT_SHIFT       (16)      /* Bits 16-19: Transmit collision count */
#define ETH_TSV1_COLCNT_MASK        (15 << ETH_TSV1_COLCNT_SHIFT)
                                              /* Bits 20-31: Reserved */
/* Receive status vector register (RSV) */

#define ETH_RSV_RXCNT_SHIFT         (0)       /* Bits 0-15: Received byte count */
#define ETH_RSV_RXCNT_MASK          (0xffff << ETH_RSV_RXCNT_SHIFT)
#define ETH_RSV_PKTPI               (1 << 16) /* Bit 16: Packet previously ignored */
#define ETH_RSV_RXEPS               (1 << 17) /* Bit 17: RXDV event previously seen */
#define ETH_RSV_CEPS                (1 << 18) /* Bit 18: Carrier event previously seen */
#define ETH_RSV_RXCV                (1 << 19) /* Bit 19: Receive code violation */
#define ETH_RSV_CRCERR              (1 << 20) /* Bit 20: CRC error */
#define ETH_RSV_LENCHKERR           (1 << 21) /* Bit 21: Length check error */
#define ETH_RSV_LENOOR              (1 << 22) /* Bit 22: Length out of range */
#define ETH_RSV_RXOK                (1 << 23) /* Bit 23: Receive OK */
#define ETH_RSV_MCAST               (1 << 24) /* Bit 24: Multicast */
#define ETH_RSV_BCAST               (1 << 25) /* Bit 25: Broadcast */
#define ETH_RSV_DRIBNIB             (1 << 26) /* Bit 26: Dribble Nibble */
#define ETH_RSV_CTLFRAME            (1 << 27) /* Bit 27: Control frame */
#define ETH_RSV_PAUSE               (1 << 28) /* Bit 28: Pause */
#define ETH_RSV_UNSUPOP             (1 << 29) /* Bit 29: Unsupported Opcode */
#define ETH_RSV_VLAN                (1 << 30) /* Bit 30: VLAN */
                                              /* Bit 31: Reserved */
/* Flow control counter register (FCCNTR) */

#define ETH_FCCNTR_MCOUNT_SHIFT     (0)       /* Bits 0-15: Mirror count */
#define ETH_FCCNTR_MCOUNT_MASK      (0xffff << ETH_FCCNTR_MCOUNT_SHIFT)
#define ETH_FCCNTR_PTMR_SHIFT       (16)      /* Bits 16-31: Pause timer */
#define ETH_FCCNTR_PTMR_MASK        (0xffff << ETH_FCCNTR_PTMR_SHIFT)

/* Flow control status register (FCSTAT) */

#define ETH_FCSTAT_MCOUNT_SHIFT     (0)       /* Bits 0-15: Current mirror count */
#define ETH_FCSTAT_MCOUNT_MASK      (0xffff << ETH_FCSTAT_MCOUNT_SHIFT)
                                              /* Bits 16-31: Reserved */
/* Rx filter registers */
/* Receive filter control register (RXFLCTRL) */

#define ETH_RXFLCTRL_UCASTEN        (1 << 0)  /* Bit 0:  Accept all unicast frames */
#define ETH_RXFLCTRL_BCASTEN        (1 << 1)  /* Bit 1:  Accept all broadcast frames */
#define ETH_RXFLCTRL_MCASTEN        (1 << 2)  /* Bit 2:  Accept all multicast frames */
#define ETH_RXFLCTRL_UCASTHASHEN    (1 << 3)  /* Bit 3:  Accept hashed unicast */
#define ETH_RXFLCTRL_MCASTHASHEN    (1 << 4)  /* Bit 4:  Accect hashed multicast */
#define ETH_RXFLCTRL_PERFEN         (1 << 5)  /* Bit 5:  Accept perfect dest match */
                                              /* Bits 6-11: Reserved */
#define ETH_RXFLCTRL_MPKTEN         (1 << 12) /* Bit 12: Magic pkt filter WoL int */
#define ETH_RXFLCTRL_RXFILEN        (1 << 13) /* Bit 13: Perfect match WoL interrupt */
                                              /* Bits 14-31: Reserved */
/* Receive filter WoL status register (RXFLWOLST) AND
 * Receive filter WoL clear register (RXFLWOLCLR)
 */

#define ETH_RXFLWOL_UCAST           (1 << 0)  /* Bit 0:  Unicast frame WoL */
#define ETH_RXFLWOL_BCAST           (1 << 1)  /* Bit 1:  Broadcast frame WoL */
#define ETH_RXFLWOL_MCAST           (1 << 2)  /* Bit 2:  Multicast frame WoL */
#define ETH_RXFLWOL_UCASTHASH       (1 << 3)  /* Bit 3:  Unicast hash filter WoL */
#define ETH_RXFLWOL_MCASTHASH       (1 << 4)  /* Bit 4:  Multiicast hash filter WoL */
#define ETH_RXFLWOL_PERF            (1 << 5)  /* Bit 5:  Perfect addr match WoL  */
                                              /* Bit 6:  Reserved */
#define ETH_RXFLWOL_RXFIL           (1 << 7)  /* Bit 7:  Receive filter WoL */
#define ETH_RXFLWOL_MPKT            (1 << 8)  /* Bit 8:  Magic pkt filter WoL */
                                              /* Bits 9-31: Reserved */
/* Hash filter table LSBs register (HASHFLL) AND Hash filter table MSBs register
* (HASHFLH) Are registers containing a 32-bit value with no bitfield.
 */

/* Module control registers */
/* Interrupt status register (INTST), Interrupt enable register (INTEN), Interrupt
 * clear register (INTCLR), and Interrupt set register (INTSET) common bit field
 * definition:
 */

#define ETH_INT_RXOVR               (1 << 0)  /* Bit 0:  RX overrun interrupt */
#define ETH_INT_RXERR               (1 << 1)  /* Bit 1:  RX error interrupt */
#define ETH_INT_RXFIN               (1 << 2)  /* Bit 2:  RX finished interrupt */
#define ETH_INT_RXDONE              (1 << 3)  /* Bit 3:  RX done interrupt */
#define ETH_INT_TXUNR               (1 << 4)  /* Bit 4:  TX underrun interrupt */
#define ETH_INT_TXERR               (1 << 5)  /* Bit 5:  TX error interrupt */
#define ETH_INT_TXFIN               (1 << 6)  /* Bit 6:  TX finished interrupt */
#define ETH_INT_TXDONE              (1 << 7)  /* Bit 7:  TX done interrupt */
                                              /* Bits 8-11: Reserved */
#define ETH_INT_SOFT                (1 << 12) /* Bit 12: Soft interrupt */
#define ETH_INT_WKUP                (1 << 13) /* Bit 13: Wakeup interrupt */
                                              /* Bits 14-31: Reserved */
/* Power-down register */
                                              /* Bits 0-30: Reserved */
#define ETH_PWRDOWN_MACAHB          (1 << 31) /* Power down MAC/AHB */

/* Descriptors Offsets **************************************************************/

/* Tx descriptor offsets */

#define LPC17_TXDESC_PACKET         0x00      /* Base address of the Tx data buffer */
#define LPC17_TXDESC_CONTROL        0x04      /* Control Information */
#define LPC17_TXDESC_SIZE           0x08      /* Size in bytes of one Tx descriptor */

/* Tx status offsets */

#define LPC17_TXSTAT_INFO           0x00      /* Transmit status return flags */
#define LPC17_TXSTAT_SIZE           0x04      /* Size in bytes of one Tx status */

/* Rx descriptor offsets */

#define LPC17_RXDESC_PACKET         0x00      /* Base address of the Rx data buffer */
#define LPC17_RXDESC_CONTROL        0x04      /* Control Information */
#define LPC17_RXDESC_SIZE           0x08      /* Size in bytes of one Rx descriptor */

/* Rx status offsets */

#define LPC17_RXSTAT_INFO           0x00      /* Receive status return flags */
#define LPC17_RXSTAT_HASHCRC        0x04      /* Dest and source hash CRC */
#define LPC17_RXSTAT_SIZE           0x08      /* Size in bytes of one Rx status */

/* Descriptor Bit Definitions *******************************************************/

/* Tx descriptor bit definitions */

#define TXDESC_CONTROL_SIZE_SHIFT   (0)       /* Bits 0-10: Size of data buffer */
#define TXDESC_CONTROL_SIZE_MASK    (0x7ff << RXDESC_CONTROL_SIZE_SHIFT)

#define TXDESC_CONTROL_OVERRIDE     (1 << 26  /* Bit 26: Per-frame override */
#define TXDESC_CONTROL_HUGE         (1 << 27) /* Bit 27: Enable huge frame size */
#define TXDESC_CONTROL_PAD          (1 << 28) /* Bit 28: Pad short frames */
#define TXDESC_CONTROL_CRC          (1 << 29) /* Bit 29: Append CRC */
#define TXDESC_CONTROL_LAST         (1 << 30) /* Bit 30: Last descriptor of a fragment */
#define TXDESC_CONTROL_INT          (1 << 31) /* Bit 31: Generate TxDone interrupt */

/* Tx status bit definitions */

#define TXSTAT_INFO_COLCNT_SHIFT    (21)      /* Bits 21-24: Number of collisions */
#define TXSTAT_INFO_COLCNT_MASK     (15 << TXSTAT_INFO_COLCNT_SHIFT)
#define TXSTAT_INFO_DEFER           (1 << 25) /* Bit 25: Packet deffered */
#define TXSTAT_INFO_EXCESSDEFER     (1 << 26) /* Bit 26: Excessive packet defferals */
#define TXSTAT_INFO_EXCESSCOL       (1 << 27) /* Bit 27: Excessive packet collisions */
#define TXSTAT_INFO_LATECOL         (1 << 28) /* Bit 28: Out of window collision */
#define TXSTAT_INFO_UNDERRUN        (1 << 29) /* Bit 29: Tx underrun */
#define TXSTAT_INFO_NODESC          (1 << 30) /* Bit 29: No Tx descriptor available */
#define TXSTAT_INFO_ERROR           (1 << 31) /* Bit 31: OR of other error conditions */

/* Rx descriptor bit definitions */

#define RXDESC_CONTROL_SIZE_SHIFT   (0)       /* Bits 0-10: Size of data buffer */
#define RXDESC_CONTROL_SIZE_MASK    (0x7ff << RXDESC_CONTROL_SIZE_SHIFT)
#define RXDESC_CONTROL_INT          (1 << 31) /* Bit 31: Generate RxDone interrupt */

/* Rx status bit definitions */

#define RXSTAT_SAHASHCRC_SHIFT      (0)       /* Bits 0-8: Hash CRC calculated from the source address */
#define RXSTAT_SAHASHCRC_MASK       (0x1ff << RXSTAT_SAHASHCRC_SHIFT)
#define RXSTAT_DAHASHCRC_SHIFT      (16)      /* Bits 16-24: Hash CRC calculated from the dest address */
#define RXSTAT_DAHASHCRC_MASK       (0x1ff << RXSTAT_DAHASHCRC_SHIFT)

#define RXSTAT_INFO_RXSIZE_SHIFT    (0)       /* Bits 0-10: Size of actual data transferred */
#define RXSTAT_INFO_RXSIZE_MASK     (0x7ff << RXSTAT_INFO_RXSIZE_SHIFT)
#define RXSTAT_INFO_CONTROL         (1 << 18) /* Bit 18: This is a control frame */
#define RXSTAT_INFO_VLAN            (1 << 19) /* Bit 19: This is a VLAN frame */
#define RXSTAT_INFO_FAILFILTER      (1 << 20) /* Bit 20: Frame failed Rx filter */
#define RXSTAT_INFO_MULTICAST       (1 << 21) /* Bit 21: This is a multicast frame */
#define RXSTAT_INFO_BROADCAST       (1 << 22) /* Bit 22: This is a broadcast frame */
#define RXSTAT_INFO_CRCERROR        (1 << 23) /* Bit 23: Received frame had a CRC error */
#define RXSTAT_INFO_SYMBOLERROR     (1 << 24) /* Bit 24: PHY reported bit error */
#define RXSTAT_INFO_LENGTHERROR     (1 << 25) /* Bit 25: Invalid frame length */
#define RXSTAT_INFO_RANGEERROR      (1 << 26) /* Bit 26: Exceeds maximum packet size */
#define RXSTAT_INFO_ALIGNERROR      (1 << 27) /* Bit 27: Alignment error */
#define RXSTAT_INFO_OVERRUN         (1 << 28) /* Bit 28: Receive overrun error */
#define RXSTAT_INFO_NODESC          (1 << 29) /* Bit 29: No Rx descriptor available */
#define RXSTAT_INFO_LASTFLAG        (1 << 30) /* Bit 30: Last fragment of a frame */
#define RXSTAT_INFO_ERROR           (1 << 31) /* Bit 31: OR of other error conditions */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_ETHERNET_H */
