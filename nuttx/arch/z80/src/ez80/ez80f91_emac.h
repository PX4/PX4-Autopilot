/************************************************************************************
 * arch/z80/src/ez80/ez80f91_emac.h
 * arch/z80/src/chip/ez80f91_emac.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_Z80_SRC_EZ80_EZ80F91_EMAC_H
#define __ARCH_Z80_SRC_EZ80_EZ80F91_EMAC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* EMACC Registers  *****************************************************************/

/* Provided in ez80f91.h */

/* EMAC configuration 1/2/3 register bit settings ***********************************/

#define EMAC_CFG1_DCRCC        0x01  /* Bit 0: 1=4 bytes of proprietary header */
#define EMAC_CFG1_HUGEN        0x02  /* Bit 1: 1=Allow unlimited size frames to be recieved */
#define EMAC_CFG1_FLCHK        0x04  /* Bit 2: 1=Frame lengths compared to length/type */
#define EMAC_CFG1_FULLHD       0x08  /* Bit 3: 1=Enable full duplex mode */
#define EMAC_CFG1_CRCEN        0x10  /* Bit 4: 1=Append CRC to every frame */
#define EMAC_CFG1_VLPAD        0x20  /* Bit 5: 1=Pad all short frames to 64 bytes, append CRC */
#define EMAC_CFG1_ADPADN       0x40  /* Bit 6: 1=Enable frame detection by check VALN protocol ID */
#define EMAC_CFG1_PADEN        0x80  /* Bit 7: 1=Pad all short frames with zeros. */

#define EMAC_CFG2_LCOLMASK     0x3f  /* Bits 0-5: Number bytes after start frame for collision */
#define EMAC_CFG2_NOBO         0x40  /* Bit 6: 1=immediate transmit after collision */
#define EMAC_CFG2_BPNB         0x80  /* Bit 7: 1=after collision retransmit without back-off */

#define EMAC_CFG3_RETRYMASK    0x0f  /* Bits 0-3: Number retransmissions before abort */
#define EMAC_CFG3_BITMD        0x10  /* Bit 4: 1=Enable 10Mbps ENDEC mode */
#define EMAC_CFG3_XSDFR        0x20  /* Bit 5: 1=Defer to carrier indefinitely */
#define EMAC_CFG3_PUREP        0x40  /* Bit 6: 1=Verify preamble */
#define EMAC_CFG3_LONGP        0x80  /* Bit 7: 1=only allow preamble < 12 bytes */

#define EMAC_CFG4_RXEN         0x01  /* Bit 0: 1=Receive frames */
#define EMAC_CFG4_TPAUSE       0x02  /* Bit 1: 1=Force pause condition */
#define EMAC_CFG4_TXFC         0x04  /* Bit 2: 1=Transmit pause control frames */
#define EMAC_CFG4_RXFC         0x08  /* Bit 3: 1=Act on receive pause control frames */
#define EMAC_CFG4_PARF         0x10  /* Bit 4: 1=Receive all frames */
#define EMAC_CFG4_THDF         0x20  /* Bit 5: 1=Asser back-pressure */
#define EMAC_CFG4_TPCF         0x40  /* Bit 6: 1=Transmit pause control frame. */
                                     /* Bit 7: reserved */

/* EMAC AFR register bit settings ***************************************************/

#define EMAC_AFR_BC            0x01  /* Bit 0: 1=Accept broadcast messages */
#define EMAC_AFR_QMC           0x02  /* Bit 1: 1=Accept only qualified multicast messages */
#define EMAC_AFR_MC            0x04  /* Bit 2: 1=Accept any multicast message */
#define EMAC_AFR_PROM          0x08  /* Bit 3: 1=Enable promiscuous mode */
                                     /* Bits 4-7: Reserved */

/* EMAC MII management register bit settings ****************************************/

#define EMAC_MIIMGMT_CLKMASK   0x07  /* Bits 0-2: Divisor that produces MDC from SCLK */
#  define EMAC_MDC_DIV4        0x01  /*        MDC = SCLK / 4 */
#  define EMAC_MDC_DIV6        0x02  /*        MDC = SCLK / 6 */
#  define EMAC_MDC_DIV8        0x03  /*        MDC = SCLK / 8 */
#  define EMAC_MDC_DIV10       0x04  /*        MDC = SCLK / 10 */
#  define EMAC_MDC_DIV14       0x05  /*        MDC = SCLK / 14 */
#  define EMAC_MDC_DIV20       0x06  /*        MDC = SCLK / 20 */
#  define EMAC_MDC_DIV28       0x07  /*        MDC = SCLK / 28 */
#define EMAC_MIIMGMT_SPRE      0x08  /* Bit 3: 1=Suppress MDO preamble */
#define EMAC_MIIMGMT_SCAN      0x10  /* Bit 4: 1=Perform continus read cycles */
#define EMAC_MIIMGMT_SCINC     0x20  /* Bit 5: 1=Increment PHY address on scan cycle */
#define EMAC_MIIMGMT_RSTAT     0x40  /* Bit 6: 1=Read status from PHY (via PRSD) */
#define EMAC_MIIMGMT_LCTLD     0x80  /* Bit 7: 1=Send CTLD control data to PHY */

/* EMAC PHY unit select address register bit settings *******************************/

#define EMAC_RGAD_MASK         0x1f  /* 5-bit value selects address within PHY */

/* EMAC PHY address register bit settings *******************************************/

#define EMAC_FIAD_MASK         0x1f  /* 5-bit value selects the external PHY */

/* EMAC reset control register bit settings *****************************************/

#define EMAC_RST_HRMGT         0x01  /* Bit 0: 1=Reset EMAC management function */
#define EMAC_RST_HRRMC         0x02  /* Bit 1: 1=Reset EMAC receive control function */
#define EMAC_RST_HRTMC         0x04  /* Bit 2: 1=Reset EMAC transmit control function */
#define EMAC_RST_HRRFN         0x08  /* Bit 3: 1=Reset receive function */
#define EMAC_RST_HRTFN         0x10  /* Bit 4: 1=Reset transmit function */
#define EMAC_RST_SRST          0x20  /* Bit 5: 1=Software reset active */
                                     /* Bits 6-7: Reserved */

/* EMAC bufsize register bit settings ***********************************************/

#define EMAC_BUFSZ_BUFSZMASK   0xc0  /* Bits 6-6: Rx/Tx buffer size */
#  define EMAC_BUFSZ_256b      0x00  /*   EMAC Rx/Tx buffer size = 256 bytes */
#  define EMAC_BUFSZ_128b      0x40  /*   EMAC Rx/Tx buffer size = 128 bytes */
#  define EMAC_BUFSZ_64b       0x80  /*   EMAC Rx/Tx buffer size = 64 bytes */
#  define EMAC_BUFSZ_32b       0xc0  /*   EMAC Rx/Tx buffer size = 32 bytes */
#define EMAC_BUFSZ_TPCFLMASK   0x3f /* Bits 0-5: Tranmsit pause frame level */

/* EMAC interrupt enable register bit settings **************************************/

#define EMAC_EIN_TXDONE       0x01  /* Bit 0: 1=Enable transmit done interrupt */
#define EMAC_EIN_TXCF         0x02  /* Bit 1: 1=Enable transmit control frame interrupt */
#define EMAC_EIN_RXOVR        0x04  /* Bit 2: 1=Enable receive overrun interrupt */
#define EMAC_EIN_RXDONE       0x08  /* Bit 3: 1=Enable receive done interrupt */
#define EMAC_EIN_RXPCF        0x10  /* Bit 4: 1=Enable receive pause control frame interrupt */
#define EMAC_EIN_RXCF         0x20  /* Bit 5: 1=Enable receive control frame interrupt */
#define EMAC_EIN_MGTDONE      0x40  /* Bit 6: 1=Enable MII Mgmt done interrupt */
#define EMAC_EIN_TXFSMERR     0x80  /* Bit 7: 1=Enable transmit state machine error interrupt */

/* EMAC interrupt status register bit settings **************************************/

#define EMAC_ISTAT_TXDONE     0x01  /* Bit 0: 1=Transmit done interrupt */
#define EMAC_ISTAT_TXCF       0x02  /* Bit 1: 1=Transmit control frame interrupt */
#define EMAC_ISTAT_RXOVR      0x04  /* Bit 2: 1=Receive overrun interrupt */
#define EMAC_ISTAT_RXDONE     0x08  /* Bit 3: 1=Receive done interrupt */
#define EMAC_ISTAT_RXPCF      0x10  /* Bit 4: 1=Receive pause control frame interrupt */
#define EMAC_ISTAT_RXCF       0x20  /* Bit 5: 1=Receive control frame interrupt */
#define EMAC_ISTAT_MGTDONE    0x40  /* Bit 6: 1=MII Mgmt done interrupt */
#define EMAC_ISTAT_TXFSMERR   0x80  /* Bit 7: 1=Transmit state machine error interrupt */

/* EMAC MII status register bit settings ********************************************/

#define EMAC_MIISTAT_RDADRMK  0x1f  /* Bits 0-4: PHY addressed in current scan cyle */
#define EMAC_MIISTAT_NVALID   0x20  /* Bit 5: 1=PRSD is valid */
#define EMAC_MIISTAT_MIILF    0x40  /* Bit 6: 1=PHY link OK */
#define EMAC_MIISTAT_BUSY     0x80  /* Bit 7: 1=MII management in progress */

/* EMAC FIFO flags register bit settings ********************************************/

#define EMAC_FFLAGS_RFE       0x01 /* Bit 0: 1=Receive FIFO empty */
#define EMAC_FFLAGS_RFAE      0x02 /* Bit 0: 1=Receive FIFO almost empty */
#define EMAC_FFLAGS_RFAF      0x04 /* Bit 0: 1=Receive FIFO almost full */
#define EMAC_FFLAGS_RFF       0x08 /* Bit 0: 1=Receive FIFO full */
#define EMAC_FFLAGS_TFE       0x10 /* Bit 0: 1=Transmit FIFO empty */
#define EMAC_FFLAGS_TFAE      0x20 /* Bit 0: 1=Transmit FIFO almost empty */
#define EMAC_FFLAGS_TFF       0x80 /* Bit 0: 1=Trasnmit FIFO full */

/* EMAC Transmit Descriptor Status **************************************************/

#define EMAC_TXDESC_NCOLL     0x0001 /* Bits 0-3: Bumber of collisions that occurred
                                      * while transmitting the packet. */
#define EMAC_TXDESC_MXCOLL    0x0010 /* Bit 4: 1=maximum number of collisions.
                                      * number > CFG3[3:0]. packet aborted */
#define EMAC_TXDESC_LATECOLL  0x0020 /* Bit 5: 1=late collision. Collision is detected
                                      * at a byte count > CFG2[5:0]. Collisions detected
                                      * before the byte count reaches CFG2[5:0] are early
                                      * collisions and retried. */
#define EMAC_TXDESC_FIFOUNDR  0x0040 /* Bit 6: TxFIFO Underrun. Check the TxAbort
                                      * bit to see if the packet is aborted or retried. */
#define EMAC_TXDESC_XSDFR     0x0080 /* Bit 7: Packet is excessively deferred. (> 6071 nibble
                                      * times in 100 BaseT or 24,287 bit times in 10 BaseT). */
#define EMAC_TXDESC_PKTDEFFRD 0x0100 /* Bit 8: Packet is deferred */
#define EMAC_TXDESC_CRCERROR  0x0200 /* Bit 9: Invalid FCS (CRC). Set CRCEN = 0 and the
                                      * last 4 bytes of the packet are not the valid FCS. */
#define EMAC_TXDESC_LCERROR   0x0400 /* Bit 10: Type/Length field is not a Type field and
                                      * does not match the actual data byte length of
                                      * the Ethernet packet. The data byte length is
                                      * the number of bytes of data in the Ethernet
                                      * packet between the Type/Length field and the FCS. */
#define EMAC_TXDESC_LOOR      0x0800 /* Bit 11: Type/Length field is out of range (larger
                                      * than 1518 bytes). */
#define EMAC_TXDESC_HUGE      0x1000 /* Bit 12: 1=Packet size is very large(Pkt_Size > MAXF). */
#define EMAC_TXDESC_BPA       0x2000 /* Bit 13: 1=Back pressure applied */
#define EMAC_TXDESC_ABORT     0x4000 /* Bit 14: 1=Packet aborted (not transmitted). */
#define EMAC_TXDESC_OWNER     0x8000 /* Bit 15: 0=Host (eZ80 CPU) owns, 1=EMAC owns. */

/* Receive Descriptor Status ********************************************************/

#define EMAC_RXDESC_OVR       0x0001 /* Bit 0: 1=A Receive Overrun occurs in this packet. An
                                      * overrun occurs when all of the EMAC Receive
                                      * buffers are in use and the Receive FIFO is full.
                                      * The hardware ignores all incoming packets until the
                                      * ISTAT Register [RXOVR] bit is cleared by the software.
                                      * There is no indication as to how many packets are
                                      * ignored. */
#define EMAC_RXDESC_DVEVENT   0x0002 /* Bit 1: 1=Receive data (RxDV) event is previously seen.
                                      * Indicates that the last Receive event is not long
                                      * enough to be a valid packet. */
#define EMAC_RXDESC_CEVENT    0x0004 /* Bit 2: 1=Carrier event is previously seen. This event is
                                      * defined as Rx error RxER = 1, receive data valid
                                      * (RxDV) = 0 and receive data (RxD) = Eh */
#define EMAC_RXDESC_CODEV     0x0008 /* Bit 3: 1=A code violation is detected. The PHY asserts
                                      * Rx error (RxER). */
#define EMAC_RXDESC_LCERROR   0x0010 /* Bit 4 1=Type/Length field is not a Type field and it does
                                      * not match the actual data byte length of the
                                      * Ethernet packet. The data byte length is the
                                      * number of bytes of data in the Ethernet packet
                                      * between the Type/Length field and the FCS. */
#define EMAC_RXDESC_LOOR      0x0020 /* Bit 5: 1=Type/Length field is out of range (larger
                                      * than 1518 bytes). */
#define EMAC_RXDESC_UOPCODE   0x0040 /* Bit 6: 1=Unsupported Op Code is indicated in the Op
                                      * Code field of the Ethernet packet. */
#define EMAC_RXDESC_VLAN      0x0080 /* Bit 7: 1=The packet is a VLAN packet */
#define EMAC_RXDESC_BCPKT     0x0100 /* Bit 8: 1=packet contains a broadcast address */
#define EMAC_RXDESC_MCPKT     0x0200 /* Bit 9: 1=The packet contains a multicast address */
#define EMAC_RXDESC_CR        0x0400 /* Bit 10: 1=The packet is a control frame */
#define EMAC_RXDESC_PCF       0x0800 /* Bit 11: 1=The packet is a pause control frame */
#define EMAC_RXDESC_LONGEVNT  0x1000 /* Bit 12: 1= A Long or Dropped Event occurs. A Long Event is
                                      * when a packet over 50,000 bit times occurs. A
                                      * Dropped Packet can occur if the minimum interpacket
                                      * gap is not met, the preamble is not pure, and
                                      * the CFG3[PUREP] bit is set, or if a preamble over
                                      * 11 bytes in length is detected and the CFG3[LONGP]
                                      * bit is set to 1. */
#define EMAC_RXDESC_CRCERR    0x2000 /* Bit 13: 1=The CRC (FCS) is in error */
#define EMAC_RXDESC_ALGNERR   0x4000 /* Bit 14: 1=An odd number of nibbles is received. */
#define EMAC_RXDESC_OK        0x8000 /* Bit 15: 1=Packet received intact. */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* EMAC descriptor structure (7 bytes) */

#ifndef __ASSEMBLY__
struct ez80emac_desc_s
{
  uint24_t np;      /* Pointer to the start of the next packet */
  uint16_t pktsize; /* Number of bytes in the packet, including the 4 CRC
                     * bytes, but excluding the 7 descriptor table bytes. */
  uint16_t stat;    /* Status of the packet. Differs for TX and RX packets
                     * (see EMAC_RX/TXDESC_* definitions) */
};
#endif

#define SIZEOF_EMACSDESC 7

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif /* __cplusplus */

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_Z80_SRC_EZ80_EZ80F91_EMAC_H */
