/****************************************************************************
 * drivers/net/cs89x0.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __DRIVERS_NET_CS89x0_H
#define __DRIVERS_NET_CS89x0_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_CS89x0_ALIGN16/32 determines if the 16-bit CS89x0 registers are
 * aligned to 16-bit or 32-bit address boundaries.  NOTE: If there multiple
 * CS89x00 parts in the board architecture, we assume that the address
 * alignment is the same for all implementations.  If that is not the
 * case, then it will be necessary to move a shift value into
 * the cs89x0_driver_s structure and calculate the offsets dynamically in
 * the putreg and getreg functions.
 */

#if defined(CONFIG_CS89x0_ALIGN16)
#  define CS89x0_RTDATA_OFFSET   (0 << 1)
#  define CS89x0_TxCMD_OFFSET    (2 << 1)
#  define CS89x0_TxLEN_OFFSET    (3 << 1)
#  define CS89x0_ISQ_OFFSET      (4 << 1)
#  define CS89x0_PPTR_OFFSET     (5 << 1)
#  define CS89x0_PDATA_OFFSET    (6 << 1)
#elif  defined(CONFIG_CS89x0_ALIGN32)
#  define CS89x0_RTDATA_OFFSET   (0 << 2)
#  define CS89x0_TxCMD_OFFSET    (2 << 2)
#  define CS89x0_TxLEN_OFFSET    (3 << 2)
#  define CS89x0_ISQ_OFFSET      (4 << 2)
#  define CS89x0_PPTR_OFFSET     (5 << 2)
#  define CS89x0_PDATA_OFFSET    (6 << 2)
#else
#  error "CS89x00 address alignment is not defined"
#endif

/* ISQ register bit definitions */

#define ISQ_EVENTMASK            0x003f /* Bits 0-5 indicate the status register */
#define ISQ_RXEVENT              0x0004
#define ISQ_TXEVENT              0x0008
#define ISQ_BUFEVENT             0x000c
#define ISQ_RXMISSEVENT          0x0010
#define ISQ_TXCOLEVENT           0x0012

/* ISQ register TxEVENT bit definitions*/

#define ISQ_RXEVENT_IAHASH       (1 << 6)
#define ISQ_RXEVENT_DRIBBLE      (1 << 7)
#define ISQ_RXEVENT_RXOK         (1 << 8)
#define ISQ_RXEVENT_HASHED       (1 << 9)
#define ISQ_RXEVENT_HASHNDX_SHIFT 10
#define ISQ_RXEVENT_HASHNDX_MASK  (0x3f << ISQ_RXEVENT_HASHNDX_SHIFT)

/* ISQ register TxEVENT bit definitions*/

#define ISQ_TXEVENT_LOSSOFCRS    (1 << 6)
#define ISQ_TXEVENT_SQEERROR     (1 << 7)
#define ISQ_TXEVENT_TXOK         (1 << 8)
#define ISQ_TXEVENT_OUTWINDOW    (1 << 9)
#define ISQ_TXEVENT_JABBER       (1 << 10)
#define ISQ_TXEVENT_NCOLLISION_SHIFT 11
#define ISQ_TXEVENT_NCOLLISION_MASK  (15 << ISQ_TXEVENT_NCOLLISION_SHIFT)
#define ISQ_TXEVENT_16COLL       (1 << 15)

/* ISQ register BufEVENT bit definitions */

#define ISQ_BUFEVENT_SWINT       (1 << 6)
#define ISQ_BUFEVENT_RXDMAFRAME  (1 << 7)
#define ISQ_BUFEVENT_RDY4TX      (1 << 8)
#define ISQ_BUFEVENT_TXUNDERRUN  (1 << 9)
#define ISQ_BUFEVENT_RXMISS      (1 << 10)
#define ISQ_BUFEVENT_RX128       (1 << 11)
#define ISQ_BUFEVENT_RXDEST      (1 << 15)

/* Packet page register offsets *********************************************/

/* 0x0000 Bus interface registers */

#define PPR_CHIPID               0x0000    /* Chip identifier - must be 0x630E */
#define PPR_CHIPREV              0x0002    /* Chip revision, model codes */
#define PPR_IOBASEADDRESS        0x0020    /* I/O Base Address */
#define PPR_INTREG               0x0022    /* Interrupt configuration */
# define PPR_INTREG_IRQ0         0x0000    /* Use INTR0 pin */
# define PPR_INTREG_IRQ1         0x0001    /* Use INTR1 pin */
# define PPR_INTREG_IRQ2         0x0002    /* Use INTR2 pin */
# define PPR_INTREG_IRQ3         0x0003    /* Use INTR3 pin */

#define PPR_DMACHANNELNUMBER     0x0024    /* DMA Channel Number (0,1, or 2) */
#define PPR_DMASTARTOFFRAME      0x0026    /* DMA Start of Frame */
#define PPR_DMAFRAMECOUNT        0x0028    /* DMA Frame Count (12-bits) */
#define PPR_RXDMABYTECOUNT       0x002a    /* Rx DMA Byte Count */
#define PPR_MEMORYBASEADDRESS    0x002c    /* Memory Base Address Register (20-bit) */
#define PPR_BOOTPROMBASEADDRESS  0x0030    /* Boot PROM Base Address */
#define PPR_BOOTPROMADDRESSMASK  0x0034    /* Boot PROM Address Mask */
#define PPR_EEPROMCOMMAND        0x0040    /* EEPROM Command */
#define PPR_EEPROMDATA           0x0042    /* EEPROM Data */
#define PPR_RECVFRAMEBYTES       0x0050    /* Received Frame Byte Counter */

/* 0x0100 - Configuration and control registers */

#define PPR_RXCFG                0x0102    /* Receiver configuration */
# define PPR_RXCFG_SKIP1         (1 <<  6) /* Skip (discard) current frame */
# define PPR_RXCFG_STREAM        (1 <<  7) /* Enable streaming mode */
# define PPR_RXCFG_RXOK          (1 <<  8) /* RxOK interrupt enable */
# define PPR_RxCFG_RxDMAonly     (1 <<  9) /* Use RxDMA for all frames */
# define PPR_RxCFG_AutoRxDMA     (1 << 10) /* Select RxDMA automatically */
# define PPR_RxCFG_BufferCRC     (1 << 11) /* Include CRC characters in frame */
# define PPR_RxCFG_CRC           (1 << 12) /* Enable interrupt on CRC error */
# define PPR_RxCFG_RUNT          (1 << 13) /* Enable interrupt on RUNT frames */
# define PPR_RxCFG_EXTRA         (1 << 14) /* Enable interrupt on frames with extra data */

#define PPR_RXCTL                0x0104    /* Receiver control */
# define PPR_RXCTL_IAHASH        (1 <<  6) /* Accept frames that match hash */
# define PPR_RXCTL_PROMISCUOUS   (1 <<  7) /* Accept any frame */
# define PPR_RXCTL_RXOK          (1 <<  8) /* Accept well formed frames */
# define PPR_RXCTL_MULTICAST     (1 <<  9) /* Accept multicast frames */
# define PPR_RXCTL_IA            (1 << 10) /* Accept frame that matches IA */
# define PPR_RXCTL_BROADCAST     (1 << 11) /* Accept broadcast frames */
# define PPR_RXCTL_CRC           (1 << 12) /* Accept frames with bad CRC */
# define PPR_RXCTL_RUNT          (1 << 13) /* Accept runt frames */
# define PPR_RXCTL_EXTRA         (1 << 14) /* Accept frames that are too long */

#define PPR_TXCFG                0x0106    /* Transmit configuration */
# define PPR_TXCFG_CRS           (1 <<  6) /* Enable interrupt on loss of carrier */
# define PPR_TXCFG_SQE           (1 <<  7) /* Enable interrupt on Signal Quality Error */
# define PPR_TXCFG_TXOK          (1 <<  8) /* Enable interrupt on successful xmits */
# define PPR_TXCFG_LATE          (1 <<  9) /* Enable interrupt on "out of window" */
# define PPR_TXCFG_JABBER        (1 << 10) /* Enable interrupt on jabber detect */
# define PPR_TXCFG_COLLISION     (1 << 11) /* Enable interrupt if collision */
# define PPR_TXCFG_16COLLISIONS  (1 << 15) /* Enable interrupt if > 16 collisions */

#define PPR_TXCMD                0x0108    /* Transmit command status */
# define PPR_TXCMD_TXSTART5      (0 <<  6) /* Start after 5 bytes in buffer */
# define PPR_TXCMD_TXSTART381    (1 <<  6) /* Start after 381 bytes in buffer */
# define PPR_TXCMD_TXSTART1021   (2 <<  6) /* Start after 1021 bytes in buffer */
# define PPR_TXCMD_TXSTARTFULL   (3 <<  6) /* Start after all bytes loaded */
# define PPR_TXCMD_FORCE         (1 <<  8) /* Discard any pending packets */
# define PPR_TXCMD_ONECOLLISION  (1 <<  9) /* Abort after a single collision */
# define PPR_TXCMD_NOCRC         (1 << 12) /* Do not add CRC */
# define PPR_TXCMD_NOPAD         (1 << 13) /* Do not pad short packets */

#define PPR_BUFCFG               0x010a    /* Buffer configuration */
# define PPR_BUFCFG_SWI          (1 <<  6) /* Force interrupt via software */
# define PPR_BUFCFG_RXDMA        (1 <<  7) /* Enable interrupt on Rx DMA */
# define PPR_BUFCFG_TXRDY        (1 <<  8) /* Enable interrupt when ready for Tx */
# define PPR_BUFCFG_TXUE         (1 <<  9) /* Enable interrupt in Tx underrun */
# define PPR_BUFCFG_RXMISS       (1 << 10) /* Enable interrupt on missed Rx packets */
# define PPR_BUFCFG_RX128        (1 << 11) /* Enable Rx interrupt after 128 bytes */
# define PPR_BUFCFG_TXCOL        (1 << 12) /* Enable int on Tx collision ctr overflow */
# define PPR_BUFCFG_MISS         (1 << 13) /* Enable int on Rx miss ctr overflow */
# define PPR_BUFCFG_RXDEST       (1 << 15) /* Enable int on Rx dest addr match */

#define PPR_LINECTL              0x0112    /* Line control */
# define PPR_LINECTL_RX          (1 <<  6) /* Enable receiver */
# define PPR_LINECTL_TX          (1 <<  7) /* Enable transmitter */
# define PPR_LINECTL_AUIONLY     (1 <<  8) /* AUI interface only */
# define PPR_LINECTL_AUTOAUI10BT (1 <<  9) /* Autodetect AUI or 10BaseT interface */
# define PPR_LINECTL_MODBACKOFFE (1 << 11) /* Enable modified backoff algorithm */
# define PPR_LINECTL_POLARITYDIS (1 << 12) /* Disable Rx polarity autodetect */
# define PPR_LINECTL_2PARTDEFDIS (1 << 13) /* Disable two-part defferal */
# define PPR_LINECTL_LORXSQUELCH (1 << 14) /* Reduce receiver squelch threshold */

#define PPR_SELFCTL              0x0114    /* Chip self control */
# define PPR_SELFCTL_RESET       (1 <<  6) /* Self-clearing reset */
# define PPR_SELFCTL_SWSUSPEND   (1 <<  8) /* Initiate suspend mode */
# define PPR_SELFCTL_HWSLEEPE    (1 <<  9) /* Enable SLEEP input */
# define PPR_SELFCTL_HWSTANDBYE  (1 << 10) /* Enable standby mode */
# define PPR_SELFCTL_HC0E        (1 << 12) /* Use HCB0 for LINK LED */
# define PPR_SELFCTL_HC1E        (1 << 13) /* Use HCB1 for BSTATUS LED */
# define PPR_SELFCTL_HCB0        (1 << 14) /* Control LINK LED if HC0E set */
# define PPR_SELFCTL_HCB1        (1 << 15) /* Cntrol BSTATUS LED if HC1E set */

#define PPR_BUSCTL               0x0116    /* Bus control */
# define PPR_BUSCTL_RESETRXDMA   (1 <<  6) /* Reset RxDMA pointer */
# define PPR_BUSCTL_DMAEXTEND    (1 <<  8) /* Extend DMA cycle */
# define PPR_BUSCTL_USESA        (1 <<  9) /* Assert MEMCS16 on address decode */
# define PPR_BUSCTL_MEMORYE      (1 << 10) /* Enable memory mode */
# define PPR_BUSCTL_DMABURST     (1 << 11) /* Limit DMA access burst */
# define PPR_BUSCTL_IOCHRDYE     (1 << 12) /* Set IOCHRDY high impedence */
# define PPR_BUSCTL_RXDMASIZE    (1 << 13) /* Set DMA buffer size 64KB */
# define PPR_BUSCTL_ENABLEIRQ    (1 << 15) /* Generate interrupt on interrupt event */

#define PPR_TESTCTL              0x0118    /* Test control */
# define PPR_TESTCTL_DISABLELT   (1 <<  7) /* Disable link status */
# define PPR_TESTCTL_ENDECLOOP   (1 <<  9) /* Internal loopback */
# define PPR_TESTCTL_AUILOOP     (1 << 10) /* AUI loopback */
# define PPR_TESTCTL_DISBACKOFF  (1 << 11) /* Disable backoff algorithm */
# define PPR_TESTCTL_FDX         (1 << 14) /* Enable full duplex mode */

/* 0x0120 - Status and Event Registers */

#define PPR_ISQ                  0x0120    /* Interrupt Status Queue */
#define PPR_RER                  0x0124    /* Receive event */
# define PPR_RER_IAHASH          (1 <<  6) /* Frame hash match */
# define PPR_RER_DRIBBLE         (1 <<  7) /* Frame had 1-7 extra bits after last byte */
# define PPR_RER_RXOK            (1 <<  8) /* Frame received with no errors */
# define PPR_RER_HASHED          (1 <<  9) /* Frame address hashed OK */
# define PPR_RER_IA              (1 << 10) /* Frame address matched IA */
# define PPR_RER_BROADCAST       (1 << 11) /* Broadcast frame */
# define PPR_RER_CRC             (1 << 12) /* Frame had CRC error */
# define PPR_RER_RUNT            (1 << 13) /* Runt frame */
# define PPR_RER_EXTRA           (1 << 14) /* Frame was too long */

#define PPR_TER                  0x0128    /* Transmit event */
# define PPR_TER_CRS             (1 <<  6) /* Carrier lost */
# define PPR_TER_SQE             (1 <<  7) /* Signal Quality Error */
# define PPR_TER_TXOK            (1 <<  8) /* Packet sent without error */
# define PPR_TER_LATE            (1 <<  9) /* Out of window */
# define PPR_TER_JABBER          (1 << 10) /* Stuck transmit? */
# define PPR_TER_NUMCOLLISIONS_SHIFT 11
# define PPR_TER_NUMCOLLISIONS_MASK  (15 << PPR_TER_NUMCOLLISIONS_SHIFT)
# define PPR_TER_16COLLISIONS    (1 << 15) /* > 16 collisions */

#define PPR_BER                  0x012C    /* Buffer event */
# define PPR_BER_SWINT           (1 <<  6) /* Software interrupt */
# define PPR_BER_RXDMAFRAME      (1 <<  7) /* Received framed DMAed */
# define PPR_BER_RDY4TX          (1 <<  8) /* Ready for transmission */
# define PPR_BER_TXUNDERRUN      (1 <<  9) /* Transmit underrun */
# define PPR_BER_RXMISS          (1 << 10) /* Received frame missed */
# define PPR_BER_RX128           (1 << 11) /* 128 bytes received */
# define PPR_BER_RXDEST          (1 << 15) /* Received framed passed address filter */

#define PPR_RXMISS               0x0130    /*  Receiver miss counter */
#define PPR_TXCOL                0x0132    /*  Transmit collision counter */
#define PPR_LINESTAT             0x0134    /* Line status */
# define PPR_LINESTAT_LINKOK     (1 <<  7) /* Line is connected and working */
# define PPR_LINESTAT_AUI        (1 <<  8) /* Connected via AUI */
# define PPR_LINESTAT_10BT       (1 <<  9) /* Connected via twisted pair */
# define PPR_LINESTAT_POLARITY   (1 << 12) /* Line polarity OK (10BT only) */
# define PPR_LINESTAT_CRS        (1 << 14) /* Frame being received */

#define PPR_SELFSTAT             0x0136    /* Chip self status */
# define PPR_SELFSTAT_33VACTIVE  (1 <<  6) /* supply voltage is 3.3V */
# define PPR_SELFSTAT_INITD      (1 <<  7) /* Chip initialization complete */
# define PPR_SELFSTAT_SIBSY      (1 <<  8) /* EEPROM is busy */
# define PPR_SELFSTAT_EEPROM     (1 <<  9) /* EEPROM present */
# define PPR_SELFSTAT_EEPROMOK   (1 << 10) /* EEPROM checks out */
# define PPR_SELFSTAT_ELPRESENT  (1 << 11) /* External address latch logic available */
# define PPR_SELFSTAT_EESIZE     (1 << 12) /* Size of EEPROM */

#define PPR_BUSSTAT              0x0138    /* Bus status */
# define PPR_BUSSTAT_TXBID       (1 <<  7) /* Tx error */
# define PPR_BUSSTAT_TXRDY       (1 <<  8) /* Ready for Tx data */

#define PPR_TDR                  0x013C    /* AUI Time Domain Reflectometer */

/* 0x0144 - Initiate transmit registers */

#define PPR_TXCOMMAND            0x0144    /* Tx Command */
#define PPR_TXLENGTH             0x0146    /* Tx Length */

/* 0x0150 - Address filter registers */

#define PPR_LAF                  0x0150    /* Logical address filter (6 bytes) */
#define PPR_IA                   0x0158    /* Individual address (MAC) */

/* 0x0400 - Frame location registers */

#define PPR_RXSTATUS             0x0400  /* Rx Status */
#define PPR_RXLENGTH             0x0402  /* Rx Length */
#define PPR_RXFRAMELOCATION      0x0404  /* Rx Frame Location */
#define PPR_TXFRAMELOCATION      0x0a00  /* Tx Frame Location */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_NET_CS89x0_H */
