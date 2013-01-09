/************************************************************************************
 * arch/arm/src/lm/chip/lm_ethernet.h
 *
 *   Copyright (C) 2009-2010, 2012-2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LM_CHIP_LM_ETHERNET_H
#define __ARCH_ARM_SRC_LM_CHIP_LM_ETHERNET_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/mii.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Ethernet Controller Register Offsets *********************************************/

/* Ethernet MAC Register Offsets */

#define LM_MAC_RIS_OFFSET     0x000 /* Ethernet MAC Raw Interrupt Status */
#define LM_MAC_IACK_OFFSET    0x000 /* Ethernet MAC Acknowledge */
#define LM_MAC_IM_OFFSET      0x004 /* Ethernet MAC Interrupt Mask */
#define LM_MAC_RCTL_OFFSET    0x008 /* Ethernet MAC Receive Control */
#define LM_MAC_TCTL_OFFSET    0x00c /* Ethernet MAC Transmit Control */
#define LM_MAC_DATA_OFFSET    0x010 /* Ethernet MAC Data */
#define LM_MAC_IA0_OFFSET     0x014 /* Ethernet MAC Individual Address 0 */
#define LM_MAC_IA1_OFFSET     0x018 /* Ethernet MAC Individual Address 1 */
#define LM_MAC_THR_OFFSET     0x01c /* Ethernet MAC Threshold */
#define LM_MAC_MCTL_OFFSET    0x020 /* Ethernet MAC Management Control */
#define LM_MAC_MDV_OFFSET     0x024 /* Ethernet MAC Management Divider */
#define LM_MAC_MTXD_OFFSET    0x02c /* Ethernet MAC Management Transmit Data */
#define LM_MAC_MRXD_OFFSET    0x030 /* Ethernet MAC Management Receive Data */
#define LM_MAC_NP_OFFSET      0x034 /* Ethernet MAC Number of Packets */
#define LM_MAC_TR_OFFSET      0x038 /* Ethernet MAC Transmission Request */
#ifdef LM_ETHTS
#  define LM_MAC_TS_OFFSET    0x03c /* Ethernet MAC Time Stamp Configuration */
#endif

/* MII Management Register Offsets (see include/nuttx/net/mii.h) */

/* Ethernet Controller Register Addresses *******************************************/

#define LM_MAC_RIS            (LM_ETHCON_BASE + LM_MAC_RIS_OFFSET)
#define LM_MAC_IACK           (LM_ETHCON_BASE + LM_MAC_IACK_OFFSET)
#define LM_MAC_IM             (LM_ETHCON_BASE + LM_MAC_IM_OFFSET)
#define LM_MAC_RCTL           (LM_ETHCON_BASE + LM_MAC_RCTL_OFFSET)
#define LM_MAC_TCTL           (LM_ETHCON_BASE + LM_MAC_TCTL_OFFSET)
#define LM_MAC_DATA           (LM_ETHCON_BASE + LM_MAC_DATA_OFFSET)
#define LM_MAC_IA0            (LM_ETHCON_BASE + LM_MAC_IA0_OFFSET)
#define LM_MAC_IA1            (LM_ETHCON_BASE + LM_MAC_IA1_OFFSET)
#define LM_MAC_THR            (LM_ETHCON_BASE + LM_MAC_THR_OFFSET)
#define LM_MAC_MCTL           (LM_ETHCON_BASE + LM_MAC_MCTL_OFFSET)
#define LM_MAC_MDV            (LM_ETHCON_BASE + LM_MAC_MDV_OFFSET)
#define LM_MAC_MTXD           (LM_ETHCON_BASE + LM_MAC_MTXD_OFFSET)
#define LM_MAC_MRXD           (LM_ETHCON_BASE + LM_MAC_MRXD_OFFSET)
#define LM_MAC_NP             (LM_ETHCON_BASE + LM_MAC_NP_OFFSET)
#define LM_MAC_TR             (LM_ETHCON_BASE + LM_MAC_TR_OFFSET)
#ifdef LM_ETHTS
#  define LM_MAC_TS           (LM_ETHCON_BASE + LM_MAC_TS_OFFSET)
#endif

/* Memory Mapped MII Management Registers */

#define MAC_MII_MCR           (LM_ETHCON_BASE + MII_MCR)
#define MAC_MII_MSR           (LM_ETHCON_BASE + MII_MSR)
#define MAC_MII_PHYID1        (LM_ETHCON_BASE + MII_PHYID1)
#define MAC_MII_PHYID2        (LM_ETHCON_BASE + MII_PHYID2)
#define MAC_MII_ADVERTISE     (LM_ETHCON_BASE + MII_ADVERTISE)
#define MAC_MII_LPA           (LM_ETHCON_BASE + MII_LPA)
#define MAC_MII_EXPANSION     (LM_ETHCON_BASE + MII_EXPANSION)
#define MAC_MII_VSPECIFIC     (LM_ETHCON_BASE + MII_LM_VSPECIFIC)
#define MAC_MII_INTCS         (LM_ETHCON_BASE + MII_LM_INTCS)
#define MAC_MII_DIAGNOSTIC    (LM_ETHCON_BASE + MII_LM_DIAGNOSTIC)
#define MAC_MII_XCVRCONTROL   (LM_ETHCON_BASE + MII_LM_XCVRCONTROL)
#define MAC_MII_LEDCONFIG     (LM_ETHCON_BASE + MII_LM_LEDCONFIG)
#define MAC_MII_MDICONTROL    (LM_ETHCON_BASE + MII_LM_MDICONTROL)

/* Ethernet Controller Register Bit Definitions *************************************/

/* Ethernet MAC Raw Interrupt Status/Acknowledge (MACRIS/MACIACK), offset 0x000 */

#define MAC_RIS_RXINT         (1 << 0)  /* Bit 0:  Packet Received */
#define MAC_RIS_TXER          (1 << 1)  /* Bit 1:  Transmit Error */
#define MAC_RIS_TXEMP         (1 << 2)  /* Bit 2:  Transmit FIFO Empty */
#define MAC_RIS_FOV           (1 << 3)  /* Bit 3:  FIFO Overrun */
#define MAC_RIS_RXER          (1 << 4)  /* Bit 4:  Receive Error */
#define MAC_RIS_MDINT         (1 << 5)  /* Bit 5:  MII Transaction Complete */
#define MAC_RIS_PHYINT        (1 << 6)  /* Bit 6:  PHY Interrupt */

#define MAC_IACK_RXINT        (1 << 0)  /* Bit 0:  Clear Packet Received */
#define MAC_IACK_TXER         (1 << 1)  /* Bit 1:  Clear Transmit Error */
#define MAC_IACK_TXEMP        (1 << 2)  /* Bit 2:  Clear Transmit FIFO Empty */
#define MAC_IACK_FOV          (1 << 3)  /* Bit 3:  Clear FIFO Overrun */
#define MAC_IACK_RXER         (1 << 4)  /* Bit 4:  Clear Receive Error */
#define MAC_IACK_MDINT        (1 << 5)  /* Bit 5:  Clear MII Transaction Complete */
#define MAC_IACK_PHYINT       (1 << 6)  /* Bit 6:  Clear PHY Interrupt */

/* Ethernet MAC Interrupt Mask (MACIM), offset 0x004 */

#define MAC_IM_RXINTM         (1 << 0)  /* Bit 0:  Mask Packet Received */
#define MAC_IM_TXERM          (1 << 1)  /* Bit 1:  Mask Transmit Error */
#define MAC_IM_TXEMPM         (1 << 2)  /* Bit 2:  Mask Transmit FIFO Empty */
#define MAC_IM_FOVM           (1 << 3)  /* Bit 3:  Mask FIFO Overrun */
#define MAC_IM_RXERM          (1 << 4)  /* Bit 4:  Mask Receive Error */
#define MAC_IM_MDINTM         (1 << 5)  /* Bit 5:  Mask MII Transaction Complete */
#define MAC_IM_PHYINTM        (1 << 6)  /* Bit 6:  Mask PHY Interrupt */
#define MAC_IM_ALLINTS        0x7f

/* Ethernet MAC Receive Control (MACRCTL), offset 0x008 */

#define MAC_RCTL_RXEN         (1 << 0)  /* Bit 0:  Enable Receiver */
#define MAC_RCTL_AMUL         (1 << 1)  /* Bit 1:  Enable Multicast Frames */
#define MAC_RCTL_PRMS         (1 << 2)  /* Bit 2:  Enable Promiscuous Mode */
#define MAC_RCTL_BADCRC       (1 << 3)  /* Bit 3:  Enable Reject Bad CRC */
#define MAC_RCTL_RSTFIFO      (1 << 4)  /* Bit 4:  Clear Receive FIFO */

/* Ethernet MAC Transmit Control (MACTCTL), offset 0x00c */

#define MAC_TCTL_TXEN         (1 << 0)  /* Bit 0:  Enable Transmitter */
#define MAC_TCTL_PADEN        (1 << 1)  /* Bit 1:  Enable Packet Padding */
#define MAC_TCTL_CRC          (1 << 2)  /* Bit 2:  Enable CRC Generation */
#define MAC_TCTL_DUPLEX       (1 << 4)  /* Bit 4:  Enable Duplex Mode */

/* Ethernet MAC Threshold (MACTHR), offset 0x01c */

#define MAC_THR_MASK          0x3f      /* Bits 5-0: Threshold Value */

/* Ethernet MAC Management Control (MACMCTL), offset 0x020 */

#define MAC_MCTL_START        (1 << 0)  /* Bit 0:  MII Register Transaction Enable */
#define MAC_MCTL_WRITE        (1 << 1)  /* Bit 1:  MII Register Transaction Type */
#define MAC_MCTL_REGADR_SHIFT 3         /* Bits 7-3: MII Register Address */
#define MAC_MCTL_REGADR_MASK  (0x1f << MAC_MCTL_REGADR_SHIFT)

/* Ethernet MAC Management Divider (MACMDV), offset 0x024 */

#define MAC_MDV_MASK          0xff      /* Bits 7-0: Clock Divider */

/* Ethernet MAC Management Transmit Data (MACTXD), offset 0x02c */

#define MAC_MTXD_MASK         0xffff    /* Bits 15-0: MII Register Transmit Data */

/* Ethernet MAC Management Receive Data (MACRXD), offset 0x030 */

#define MAC_MTRD_MASK         0xffff    /* Bits 15-0: MII Register Receive Data */

/* Ethernet MAC Number of Packets (MACNP), offset 0x034 */

#define MAC_NP_MASK           0x3f      /* Bits 5-0: Number of Packets in Receive FIFO */

/* Ethernet MAC Transmission Request (MACTR), offset 0x038 */

#define MAC_TR_NEWTX          (1 << 0)  /* Bit 0:  New Transmission */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LM_CHIP_LM_ETHERNET_H */
