/************************************************************************************
 * arch/hc/src/m9s12/m9s12_emac.h
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

#ifndef __ARCH_ARM_HC_SRC_M9S12_M9S12_EMAC_H
#define __ARCH_ARM_HC_SRC_M9S12_M9S12_EMAC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define HCS12_EMAC_NETCTL_OFFSET       0x0000 /* Network Control (8-bit) */
#define HCS12_EMAC_RXCTS_OFFSET        0x0003 /* Receive Control and Status (8-bit) */
#define HCS12_EMAC_TXCTS_OFFSET        0x0004 /* Transmit Control and Status (8-bit) */
#define HCS12_EMAC_ETCTL_OFFSET        0x0005 /* Ethertype Control (8-bit) */
#define HCS12_EMAC_ETYPE_OFFSET        0x0006 /* Programmable Ethertype (16-bit) */
#define HCS12_EMAC_PTIME_OFFSET        0x0008 /* PAUSE Timer Value and Counter (16-bit) */
#define HCS12_EMAC_IEVENT_OFFSET       0x000a /* Interrupt Event (16-bit) */
#define HCS12_EMAC_IMASK_OFFSET        0x000c /* Interrupt Mask (16-bit) */
#define HCS12_EMAC_SWRST_OFFSET        0x000e /* Software Reset (8-bit) */
#define HCS12_EMAC_MPADR_OFFSET        0x0010 /* MII Management PHY Address (8-bit) */
#define HCS12_EMAC_MRADR_OFFSET        0x0011 /* MII Management Register Address (8-bit) */
#define HCS12_EMAC_MWDATA_OFFSET       0x0012 /* MII Management Write Data (16-bit) */
#define HCS12_EMAC_MRDATA_OFFSET       0x0014 /* MII Management Read Data (16-bit) */
#define HCS12_EMAC_MCMST_OFFSET        0x0016 /* MII Management Command and Status (8-bit) */
#define HCS12_EMAC_BUFCFG_OFFSET       0x0018 /* Ethernet Buffer Configuration (16-bit) */
#define HCS12_EMAC_RXAEFP_OFFSET       0x001a /* Receive A End-of-Frame Pointer (16-bit) */
#define HCS12_EMAC_RXBEFP_OFFSET       0x001c /* Receive B End-of-Frame Pointer (16-bit) */
#define HCS12_EMAC_TXEFP_OFFSET        0x001e /* Transmit End-of-Frame Pointer (16-bit) */
#define HCS12_EMAC_MCHASH48_OFFSET     0x0020 /* Multicast Hash Table 48-63 (16-bit) */
#define HCS12_EMAC_MCHASH32_OFFSET     0x0022 /* Multicast Hash Table 32-47 (16-bit) */
#define HCS12_EMAC_MCHASH16_OFFSET     0x0024 /* Multicast Hash Table 16-31 (16-bit) */
#define HCS12_EMAC_MCHASH0_OFFSET      0x0026 /* Multicast Hash Table 0:15 (16-bit) */
#define HCS12_EMAC_MACAD32_OFFSET      0x0028 /* MAC Unicast AAddress 32-47 (16-bit) */
#define HCS12_EMAC_MACAD16_OFFSET      0x002a /* MAC Unicast AAddress 16-31 (16-bit) */
#define HCS12_EMAC_MACAD0_OFFSET       0x002c /* MAC Unicast AAddress 0-15 (16-bit) */
#define HCS12_EMAC_EMISC _OFFSET       0x002e /* Miscellaneous (16-bit) */

/* Register Addresses ***************************************************************/

#define HCS12_EMAC_NETCTL              (HCS12_EMAC_BASE+HCS12_EMAC_NETCTL_OFFSET)
#define HCS12_EMAC_RXCTS               (HCS12_EMAC_BASE+HCS12_EMAC_RXCTS_OFFSET)
#define HCS12_EMAC_TXCTS               (HCS12_EMAC_BASE+HCS12_EMAC_TXCTS_OFFSET)
#define HCS12_EMAC_ETCTL               (HCS12_EMAC_BASE+HCS12_EMAC_ETCTL_OFFSET)
#define HCS12_EMAC_ETYPE               (HCS12_EMAC_BASE+HCS12_EMAC_ETYPE_OFFSET)
#define HCS12_EMAC_PTIME               (HCS12_EMAC_BASE+HCS12_EMAC_PTIME_OFFSET)
#define HCS12_EMAC_IEVENT              (HCS12_EMAC_BASE+HCS12_EMAC_IEVENT_OFFSET)
#define HCS12_EMAC_IMASK               (HCS12_EMAC_BASE+HCS12_EMAC_IMASK_OFFSET)
#define HCS12_EMAC_SWRST               (HCS12_EMAC_BASE+HCS12_EMAC_SWRST_OFFSET)
#define HCS12_EMAC_MPADR               (HCS12_EMAC_BASE+HCS12_EMAC_MPADR_OFFSET)
#define HCS12_EMAC_MRADR               (HCS12_EMAC_BASE+HCS12_EMAC_MRADR_OFFSET)
#define HCS12_EMAC_MWDATA              (HCS12_EMAC_BASE+HCS12_EMAC_MWDATA_OFFSET)
#define HCS12_EMAC_MRDATA              (HCS12_EMAC_BASE+HCS12_EMAC_MRDATA_OFFSET)
#define HCS12_EMAC_MCMST               (HCS12_EMAC_BASE+HCS12_EMAC_MCMST_OFFSET)
#define HCS12_EMAC_BUFCFG              (HCS12_EMAC_BASE+HCS12_EMAC_BUFCFG_OFFSET)
#define HCS12_EMAC_RXAEFP              (HCS12_EMAC_BASE+HCS12_EMAC_RXAEFP_OFFSET)
#define HCS12_EMAC_RXBEFP              (HCS12_EMAC_BASE+HCS12_EMAC_RXBEFP_OFFSET)
#define HCS12_EMAC_TXEFP               (HCS12_EMAC_BASE+HCS12_EMAC_TXEFP_OFFSET)
#define HCS12_EMAC_MCHASH48            (HCS12_EMAC_BASE+HCS12_EMAC_MCHASH48_OFFSET)
#define HCS12_EMAC_MCHASH32            (HCS12_EMAC_BASE+HCS12_EMAC_MCHASH32_OFFSET)
#define HCS12_EMAC_MCHASH16            (HCS12_EMAC_BASE+HCS12_EMAC_MCHASH16_OFFSET)
#define HCS12_EMAC_MCHASH0             (HCS12_EMAC_BASE+HCS12_EMAC_MCHASH0_OFFSET)
#define HCS12_EMAC_MACAD32             (HCS12_EMAC_BASE+HCS12_EMAC_MACAD32_OFFSET)
#define HCS12_EMAC_MACAD16             (HCS12_EMAC_BASE+HCS12_EMAC_MACAD16_OFFSET)
#define HCS12_EMAC_MACAD0              (HCS12_EMAC_BASE+HCS12_EMAC_MACAD0_OFFSET)
#define HCS12_EMAC_EMISC               (HCS12_EMAC_BASE+HCS12_EMAC_EMISC_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* Network Control (8-bit) */

#define EMAC_NETCTL_FDX                (1 << 1)  /* Bit 1: Full Duplex */
#define EMAC_NETCTL_MLB                (1 << 2)  /* Bit 2: MAC Loopback */
#define EMAC_NETCTL_EXTPHY             (1 << 3)  /* Bit 3: External PHY */
#define EMAC_NETCTL_ESWAI              (1 << 4)  /* Bit 4: EMAC Disabled during Wait Mode */
#define EMAC_NETCTL_EMACE              (1 << 7)  /* Bit 7: EMAC Enable */

/* Receive Control and Status (8-bit) */

#define EMAC_RXCTS_BCREJ               (1 << 0)  /* Bit 0: Broadcast Reject */
#define EMAC_RXCTS_CONMC               (1 << 1)  /* Bit 1: Conditional Multicast */
#define EMAC_RXCTS_PROM                (1 << 2)  /* Bit 2: Promiscuous Mode */
#define EMAC_RXCTS_RFCE                (1 << 4)  /* Bit 4: Reception Flow Control Enable */
#define EMAC_RXCTS_RXACT               (1 << 7)  /* Bit 7: Receiver Active Status */

/* Transmit Control and Status (8-bit) */

#define EMAC_TXCTS_TCMD_SHIFT          (0)       /* Bits 0-1: Transmit Command */
#define EMAC_TXCTS_TCMD_MASK           (3)
#  define EMAC_TXCTS_TCMD_START        (1)       /* Transmit buffer frame */
#  define EMAC_TXCTS_TCMD_PAUSE        (2)       /* Transmit PAUSE frame (full-duplex mode only) */
#  define EMAC_TXCTS_TCMD_ABORT        (3)       /* Abort transmission */
#define EMAC_TXCTS_SSB                 (1 << 3)  /* Bit 3: Single Slot Backoff */
#define EMAC_TXCTS_PTRC                (1 << 4)  /* Bit 4: PAUSE Timer Register Control */
#define EMAC_TXCTS_CSLF                (1 << 5)  /* Bit 5: Carrier Sense Lost Flag */
#define EMAC_TXCTS_TXACT               (1 << 7)  /* Bit 7: Transmitter Active Status */

/* Ethertype Control (8-bit) */

#define EMAC_ETCTL_FIEEE               (1 << 0)  /* Bit 0: IEEE802.3 Length Field Ethertype */
#define EMAC_ETCTL_FIPV4               (1 << 1)  /* Bit 1: Internet Protocol Version 4 (IPv4) Ethertype */
#define EMAC_ETCTL_FARP                (1 << 2)  /* Bit 2: Address Resolution Protocol (ARP) Ethertype */
#define EMAC_ETCTL_FIPV6               (1 << 3)  /* Bit 3: Internet Protocol Version 6 (IPv6) Ethertype */
#define EMAC_ETCTL_FEMW                (1 << 4)  /* Bit 4: Emware Ethertype */
#define EMAC_ETCTL_FPET                (1 << 7)  /* Bit 7: Programmable Ethertype */

/* Programmable Ethertype (16-bit) -- 16-bit Ethernet type data */
/* PAUSE Timer Value and Counter (16-bit) -- 16-bit PAUSER timer value */

/* Interrupt Event (16-bit) */
/* Interrupt Mask (16-bit) */

#define EMAC_INT_TXCI                  (1 << 1)  /* Bit 1:  Frame Transmission Complete Interrupt */
#define EMAC_INT_ECI                   (1 << 4)  /* Bit 4:  Excessive Collision Interrupt */
#define EMAC_INT_LCI                   (1 << 5)  /* Bit 5:  Late Collision Interrupt */
#define EMAC_INT_MMCI                  (1 << 7)  /* Bit 7:  MII Management Transfer Complete Interrupt */
#define EMAC_INT_RXBCI                 (1 << 8)  /* Bit 8:  Valid Frame Reception to Receive Buffer B Complete Interrupt */
#define EMAC_INT_RXACI                 (1 << 9)  /* Bit 9:  Valid Frame Reception to Receive Buffer A Complete Interrupt */
#define EMAC_INT_RXBOI                 (1 << 10) /* Bit 10: Receive Buffer B Overrun Interrupt */
#define EMAC_INT_RXAOI                 (1 << 11) /* Bit 11: Receive Buffer A Overrun Interrupt */
#define EMAC_INT_RXEI                  (1 << 12) /* Bit 12: Receive Error Interrupt */
#define EMAC_INT_BREI                  (1 << 13) /* Bit 13: Babbling Receive Error Interrupt */
#define EMAC_INT_RFCI                  (1 << 15) /* Bit 15: Receive Flow Control Interrupt */

/* Software Reset (8-bit) */

#define EMAC_SWRST_MACRST              (1 << 7)  /* Bit 7: EMAC is reset */

/* MII Management PHY Address (8-bit) */

#define EMAC_MPADR_MASK                (0x1f)

/* MII Management Register Address (8-bit) */

#define EMAC_MRADR_MASK                (0x1f)

/* MII Management Write Data (16-bit) -- 16-bit write data */
/* MII Management Read Data (16-bit) -- 16-bit read data */

/* MII Management Command and Status (8-bit) */

#define EMAC_MCMST_MDCSEL_SHIFT        (0)       /* Bits 0-3: Management Clock Rate Sel */
#define EMAC_MCMST_MDCSEL_MASK         (15)      /*           MDC frequency = Bus clock frequency / (2 * MDCSEL) */
#define EMAC_MCMST_NOPRE               (1 << 4)  /* Bit 4:  No Preamble */
#define EMAC_MCMST_BUSY                (1 << 5)  /* Bit 5:  Operation in Progress */
#define EMAC_MCMST_OP_SHIFT            (6)       /* Bits 6-7: Operation Code */
#define EMAC_MCMST_OP_MASK             (3 << EMAC_MCMST_OP_SHIFT)
#  define EMAC_MCMST_OP_IGNORE         (0 << EMAC_MCMST_OP_SHIFT)
#  define EMAC_MCMST_OP_WRITE          (1 << EMAC_MCMST_OP_SHIFT)
#  define EMAC_MCMST_OP_READ           (2 << EMAC_MCMST_OP_SHIFT)

/* Ethernet Buffer Configuration (16-bit) */

#define EMAC_BUFCFG_MAXFL_SHIFT        (0)       /* Bits 0-10 Receive Maximum Frame Length */
#define EMAC_BUFCFG_MAXFL_MASK         (0x07ff)
#define EMAC_BUFCFG_BUFMAP_SHIFT       (12)      /* Bits 12-14: Buffer Size and Starting Address Mapping */
#define EMAC_BUFCFG_BUFMAP_SHIFT       (7 << EMAC_BUFCFG_BUFMAP_SHIFT)
#  define EMAC_BUFCFG_BUFMAP_128       (0 << EMAC_BUFCFG_BUFMAP_SHIFT) /* Rx/Tx = 128 bytes */
#  define EMAC_BUFCFG_BUFMAP_256       (1 << EMAC_BUFCFG_BUFMAP_SHIFT) /* Rx/Tx = 256 bytes */
#  define EMAC_BUFCFG_BUFMAP_512       (2 << EMAC_BUFCFG_BUFMAP_SHIFT) /* Rx/Tx = 512 bytes */
#  define EMAC_BUFCFG_BUFMAP_1024      (3 << EMAC_BUFCFG_BUFMAP_SHIFT) /* Rx/Tx = 1Kb */
#  define EMAC_BUFCFG_BUFMAP_1536      (4 << EMAC_BUFCFG_BUFMAP_SHIFT) /* Rx/Tx = 1.5Kb */

/* Receive A End-of-Frame Pointer (16-bit) */

#define EMAC_RXAEFP_MASK               (0x07ff)

/* Receive B End-of-Frame Pointer (16-bit) */

#define EMAC_RXBEFP_MASK               (0x07ff)

/* Transmit End-of-Frame Pointer (16-bit) */

#define EMAC_TXEFP_MASK                (0x07ff)

/* Multicast Hash Table 48-63 (16-bit) -- 16-bits of MAC address */
/* Multicast Hash Table 32-47 (16-bit) -- 16-bits of MAC address */
/* Multicast Hash Table 16-31 (16-bit) -- 16-bits of MAC address */
/* Multicast Hash Table  0:15 (16-bit) -- 16-bits of MAC address */

/* MAC Unicast Address 32-47 (16-bit) -- 16-bits of address */
/* MAC Unicast AAddress 16-31 (16-bit) -- 16-bits of address */
/* MAC Unicast AAddress 0-15 (16-bit) -- 16-bits of address */

/* Miscellaneous (16-bit) */

#define EMAC_EMISC_SHIFT               (0)       /* Bits 0-10: Misc data */
#define EMAC_EMISC_MASK                (0x07ff)
#define EMAC_EMISC_INDEX_SHIFT         (13)      /* Bits 13-15: Miscellaneous Index */
#define EMAC_EMISC_INDEX_MASK          (7 << EMAC_EMISC_INDEX_SHIFT)
#  define EMAC_EMISC_INDEX_TXBYT       (3 << EMAC_EMISC_INDEX_SHIFT) /* Transmit Frame Byte Counter */
#  define EMAC_EMISC_INDEX_BSLOT       (4 << EMAC_EMISC_INDEX_SHIFT) /* Backoff Slot Time Counter */
#  define EMAC_EMISC_INDEX_RETX        (5 << EMAC_EMISC_INDEX_SHIFT) /* Retransmission Counter */
#  define EMAC_EMISC_INDEX_RANDOM      (6 << EMAC_EMISC_INDEX_SHIFT) /* Backoff Random Number */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_M9S12_M9S12_EMAC_H */
