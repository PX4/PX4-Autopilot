/****************************************************************************
 * drivers/net/enc28j60.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 * - ENC28J60 Data Sheet, Stand-Alone Ethernet Controller with SPI Interface,
 *   DS39662C, 2008 Microchip Technology Inc.
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

#ifndef __DRIVERS_NET_ENC28J60_H
#define __DRIVERS_NET_ENC28J60_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ENC28J60 Commands ********************************************************/
/* A total of seven instructions are implemented on the ENC28J60.  Where:
 *
 *     aaaaaa is the 5-bit address of a control register, and
 *   dddddddd is one or more bytes of data that may accompany the command.
 */

#define ENC_RCR             (0x00)    /* Read Control Register
                                       * 000 | aaaaa | (Register value returned)) */
#define ENC_RBM             (0x3a)    /* Read Buffer Memory
                                       * 001 | 11010 | (Read buffer data follows) */
#define ENC_WCR             (0x40)    /* Write Control Register
                                       * 010 | aaaaa | dddddddd */
#define ENC_WBM             (0x7a)    /* Write Buffer Memory
                                       * 011 | 11010 | (Write buffer data follows) */
#define ENC_BFS             (0x80)    /* Bit Field Set
                                       * 100 | aaaaa | dddddddd */
#define ENC_BFC             (0xa0)    /* Bit Field Clear
                                       * 101 | aaaaa | dddddddd */
#define ENC_SRC             (0xff)    /* System Reset
                                       * 111 | 11111 | (No data) */

/* Global Control Registers *************************************************/
/* Control registers are accessed with the RCR, RBM, WCR, BFS, and BFC
 * commands. The following identifies all ENC28J60 control registers.  The
 * control register memory is partitioned into four banks, selectable by the
 * bank select bits, BSEL1:BSEL0, in the ECON1 register.
 *
 * The last five locations (0x1b to 0x1f) of all banks point to a common set
 * of registers: EIE, EIR, ESTAT, ECON2 and ECON1. These are key registers
 * used in controlling and monitoring the operation of the device. Their
 * common mapping allows easy access without switching the bank.
 *
 * Control registers for the ENC28J60 are generically grouped as ETH, MAC and
 * MII registers. Register names starting with E belong to the ETH group.
 * Similarly, registers names starting with MA belong to the MAC group and
 * registers prefixed with MI belong to the MII group.
 */

#define ENC_EIE             (0x1b)   /* Ethernet Interrupt Enable Register */
#define ENC_EIR             (0x1c)   /* Ethernet Interupt Request Register */
#define ENC_ESTAT           (0x1d)   /* Ethernet Status Register */
#define ENC_ECON2           (0x1e)   /* Ethernet Control 2 Register */
#define ENC_ECON1           (0x1f)   /* Ethernet Control 1 Register */

/* Ethernet Interrupt Enable Register Bit Definitions */

#define EIE_RXERIE          (1 << 0) /* Bit 0: Receive Error Interrupt Enable */
#define EIE_TXERIE          (1 << 1) /* Bit 1: Transmit Error Interrupt Enable */
                                     /* Bit 2: Reserved */
#define EIE_TXIE            (1 << 3) /* Bit 3: Transmit Enable */
#define EIE_LINKIE          (1 << 4) /* Bit 4: Link Status Change Interrupt Enable */
#define EIE_DMAIE           (1 << 5) /* Bit 5: DMA Interrupt Enable */
#define EIE_PKTIE           (1 << 6) /* Bit 6: Receive Packet Pending Interrupt Enable */
#define EIE_INTIE           (1 << 7) /* Bit 7: Global INT Interrupt Enable */

/* Ethernet Interrupt Request Register Bit Definitions */

#define EIR_RXERIF          (1 << 0) /* Bit 0: Receive Error Interrupt */
#define EIR_TXERIF          (1 << 1) /* Bit 1: Transmit Error Interrupt */
                                     /* Bit 2: Reserved */
#define EIR_TXIF            (1 << 3) /* Bit 3: Transmit Interrupt */
#define EIR_LINKIF          (1 << 4) /* Bit 4: Link Change Interrupt */
#define EIR_DMAIF           (1 << 5) /* Bit 5: DMA Interrupt */
#define EIR_PKTIF           (1 << 6) /* Bit 6: Receive Packet Pending Interrupt */
                                     /* Bit 7: Reserved */
#define EIR_ALLINTS         (0x7b)   /* All interrupts */

/* Ethernet Status Register Bit Definitions */

#define ESTAT_CLKRDY        (1 << 0) /* Bit 0: Clock Ready */
#define ESTAT_TXABRT        (1 << 1) /* Bit 1: Transmit Abort Error */
#define ESTAT_RXBUSY        (1 << 2) /* Bit 2: Receive Busy */
                                     /* Bit 3: Reserved */
#define ESTAT_LATECOL       (1 << 4) /* Bit 4: Late Collision Error */
                                     /* Bit 5: Reserved */
#define ESTAT_BUFER         (1 << 6) /* Bit 6: Ethernet Buffer Error Status */
#define ESTAT_INT           (1 << 7) /* Bit 7: INT Interrupt */

/* Ethernet Control 1 Register Bit Definitions */

#define ECON1_BSEL_SHIFT    (0)      /* Bits 0-1: Bank select */
#define ECON1_BSEL_MASK     (3 << ECON1_BSEL_SHIFT)
#  define ECON1_BSEL_BANK0  (0 << ECON1_BSEL_SHIFT) /* Bank 0 */
#  define ECON1_BSEL_BANK1  (1 << ECON1_BSEL_SHIFT) /* Bank 1 */
#  define ECON1_BSEL_BANK2  (2 << ECON1_BSEL_SHIFT) /* Bank 2 */
#  define ECON1_BSEL_BANK3  (3 << ECON1_BSEL_SHIFT) /* Bank 3 */
#define ECON1_RXEN          (1 << 2) /* Bit 2: Receive Enable */
#define ECON1_TXRTS         (1 << 3) /* Bit 3: Transmit Request to Send */
#define ECON1_CSUMEN        (1 << 4) /* Bit 4: DMA Checksum Enable */
#define ECON1_DMAST         (1 << 5) /* Bit 5: DMA Start and Busy Status */
#define ECON1_RXRST         (1 << 6) /* Bit 6: Receive Logic Reset */
#define ECON1_TXRST         (1 << 7) /* Bit 7: Transmit Logic Reset */

/* Ethernet Control 2 Register */
                                     /* Bits 0-2: Reserved */
#define ECON2_VRPS          (1 << 3) /* Bit 3: Voltage Regulator Power Save Enable */
                                     /* Bit 4: Reserved */
#define ECON2_PWRSV         (1 << 5) /* Bit 5: Power Save Enable */
#define ECON2_PKTDEC        (1 << 6) /* Bit 6: Packet Decrement */
#define ECON2_AUTOINC       (1 << 7) /* Bit 7: Automatic Buffer Pointer Increment Enable */

/* Banked Control Registers *************************************************/
/* The remaining control registers are identified with a a 5 bit address and
 * a bank selection.  We pack the bank number and an indication if this is
 * a MAC/PHY register access together with the control register address
 * together to keep the design simpler.
 */

#define ENC_ADDR_SHIFT      (0)      /* Bits 0-4: Register address */
#define ENC_ADDR_MASK       (0x1f << ENC_ADDR_SHIFT)
#define ENC_BANK_SHIFT      (5)      /* Bits 5-6: Bank number */
#define ENC_BANK_MASK       (3 << ENC_BSEL_SHIFT)
#  define ENC_BANK0         (0 << ENC_BSEL_SHIFT)
#  define ENC_BANK1         (1 << ENC_BSEL_SHIFT)
#  define ENC_BANK2         (2 << ENC_BSEL_SHIFT)
#  define ENC_BANK3         (3 << ENC_BSEL_SHIFT)
#define ENC_PHYMAC_SHIFT    (7)      /* Bit 7: This is a PHY/MAC command */
#define ENC_PHYMAC          (1 << ENC_PHYMAC_SHIFT)

#define REGADDR(a,b,m)      ((m) << ENC_PHYMAC_SHIFT | (b) << ENC_BANK_SHIFT | (a))
#define GETADDR(a)          ((a) & ENC_ADDR_MASK)
#define GETBANK(a)          (((a) >> ENC_BANK_SHIFT) & 3)
#define ISPHYMAC(a)         (((a) & ENC_PHYMAC) != 0)

/* Bank 0 Control Register Addresses */

#define ENC_ERDPTL      REGADDR(0x00, 0, 0) /* Read Pointer Low Byte (ERDPT<7:0> */
#define ENC_ERDPTH      REGADDR(0x01, 0, 0) /* Read Pointer High Byte (ERDPT<12:8>) */
#define ENC_EWRPTL      REGADDR(0x02, 0, 0) /* Write Pointer Low Byte (EWRPT<7:0>) */
#define ENC_EWRPTH      REGADDR(0x03, 0, 0) /* Write Pointer High Byte (EWRPT<12:8>) */
#define ENC_ETXSTL      REGADDR(0x04, 0, 0) /* TX Start Low Byte (ETXST<7:0>) */
#define ENC_ETXSTH      REGADDR(0x05, 0, 0) /* TX Start High Byte (ETXST<12:8>) */
#define ENC_ETXNDL      REGADDR(0x06, 0, 0) /* TX End Low Byte (ETXND<7:0>) */
#define ENC_ETXNDH      REGADDR(0x07, 0, 0) /* TX End High Byte (ETXND<12:8>) */
#define ENC_ERXSTL      REGADDR(0x08, 0, 0) /* RX Start Low Byte (ERXST<7:0>) */
#define ENC_ERXSTH      REGADDR(0x09, 0, 0) /* RX Start High Byte (ERXST<12:8>) */
#define ENC_ERXNDL      REGADDR(0x0a, 0, 0) /* RX End Low Byte (ERXND<7:0>) */
#define ENC_ERXNDH      REGADDR(0x0b, 0, 0) /* RX End High Byte (ERXND<12:8>) */
#define ENC_ERXRDPTL    REGADDR(0x0c, 0, 0) /* RX RD Pointer Low Byte (ERXRDPT<7:0>) */
#define ENC_ERXRDPTH    REGADDR(0x0d, 0, 0) /* RX RD Pointer High Byte (ERXRDPT<12:8>) */
#define ENC_ERXWRPTL    REGADDR(0x0e, 0, 0) /* RX WR Pointer Low Byte (ERXWRPT<7:0>) */
#define ENC_ERXWRPTH    REGADDR(0x0f, 0, 0) /* RX WR Pointer High Byte (ERXWRPT<12:8>) */
#define ENC_EDMASTL     REGADDR(0x10, 0, 0) /* DMA Start Low Byte (EDMAST<7:0>) */
#define ENC_EDMASTH     REGADDR(0x11, 0, 0) /* DMA Start High Byte (EDMAST<12:8>) */
#define ENC_EDMANDL     REGADDR(0x12, 0, 0) /* DMA End Low Byte (EDMAND<7:0>) */
#define ENC_EDMANDH     REGADDR(0x13, 0, 0) /* DMA End High Byte (EDMAND<12:8>) */
#define ENC_EDMADSTL    REGADDR(0x14, 0, 0) /* DMA Destination Low Byte (EDMADST<7:0>) */
#define ENC_EDMADSTH    REGADDR(0x15, 0, 0) /* DMA Destination High Byte (EDMADST<12:8>) */
#define ENC_EDMACSL     REGADDR(0x16, 0, 0) /* DMA Checksum Low Byte (EDMACS<7:0>) */
#define ENC_EDMACSH     REGADDR(0x17, 0, 0) /* DMA Checksum High Byte (EDMACS<15:8>) */
                                            /* 0x18-0x1a: Reserved */
                                            /* 0x1b-0x1f: EIE, EIR, ESTAT, ECON2, ECON1 */
/* Bank 1 Control Register Addresses */

#define ENC_EHT0        REGADDR(0x00, 1, 0) /* Hash Table Byte 0 (EHT<7:0>) */
#define ENC_EHT1        REGADDR(0x01, 1, 0) /* Hash Table Byte 1 (EHT<15:8>) */
#define ENC_EHT2        REGADDR(0x02, 1, 0) /* Hash Table Byte 2 (EHT<23:16>) */
#define ENC_EHT3        REGADDR(0x03, 1, 0) /* Hash Table Byte 3 (EHT<31:24>) */
#define ENC_EHT4        REGADDR(0x04, 1, 0) /* Hash Table Byte 4 (EHT<39:32>) */
#define ENC_EHT5        REGADDR(0x05, 1, 0) /* Hash Table Byte 5 (EHT<47:40>) */
#define ENC_EHT6        REGADDR(0x06, 1, 0) /* Hash Table Byte 6 (EHT<55:48>) */
#define ENC_EHT7        REGADDR(0x07, 1, 0) /* Hash Table Byte 7 (EHT<63:56>) */
#define ENC_EPMM0       REGADDR(0x08, 1, 0) /* Pattern Match Mask Byte 0 (EPMM<7:0>) */
#define ENC_EPMM1       REGADDR(0x09, 1, 0) /* Pattern Match Mask Byte 1 (EPMM<15:8>) */
#define ENC_EPMM2       REGADDR(0x0a, 1, 0) /* Pattern Match Mask Byte 2 (EPMM<23:16>) */
#define ENC_EPMM3       REGADDR(0x0b, 1, 0) /* Pattern Match Mask Byte 3 (EPMM<31:24>) */
#define ENC_EPMM4       REGADDR(0x0c, 1, 0) /* Pattern Match Mask Byte 4 (EPMM<39:32>) */
#define ENC_EPMM5       REGADDR(0x0d, 1, 0) /* Pattern Match Mask Byte 5 (EPMM<47:40>) */
#define ENC_EPMM6       REGADDR(0x0e, 1, 0) /* Pattern Match Mask Byte 6 (EPMM<55:48>) */
#define ENC_EPMM7       REGADDR(0x0f, 1, 0) /* Pattern Match Mask Byte 7 (EPMM<63:56>) */
#define ENC_EPMCSL      REGADDR(0x10, 1, 0) /* Pattern Match Checksum Low Byte (EPMCS<7:0>) */
#define ENC_EPMCSH      REGADDR(0x11, 1, 0) /* Pattern Match Checksum High Byte (EPMCS<15:0>) */
                                            /* 0x12-0x13: Reserved */
#define ENC_EPMOL       REGADDR(0x14, 1, 0) /* Pattern Match Offset Low Byte (EPMO<7:0>) */
#define ENC_EPMOH       REGADDR(0x15, 1, 0) /* Pattern Match Offset High Byte (EPMO<12:8>) */
                                            /* 0x16-0x17: Reserved */
#define ENC_ERXFCON     REGADDR(0x18, 1, 0) /* Receive Filter Configuration */
#define ENC_EPKTCNT     REGADDR(0x19, 1, 0) /* Ethernet Packet Count */
                                            /* 0x1a: Reserved */
                                            /* 0x1b-0x1f: EIE, EIR, ESTAT, ECON2, ECON1 */

/* Receive Filter Configuration Bit Definitions */

#define ERXFCON_BCEN    (1 << 0) /* Bit 0: Broadcast Filter Enable */
#define ERXFCON_MCEN    (1 << 1) /* Bit 1: Multicast Filter Enable */
#define ERXFCON_HTEN    (1 << 2) /* Bit 2: Hash Table Filter Enable */
#define ERXFCON_MPEN    (1 << 3) /* Bit 3: Magic Packet Filter Enable */
#define ERXFCON_PMEN    (1 << 4) /* Bit 4: Pattern Match Filter Enable */
#define ERXFCON_CRCEN   (1 << 5) /* Bit 5: Post-Filter CRC Check Enable */
#define ERXFCON_ANDOR   (1 << 6) /* Bit 6: AND/OR Filter Select */
#define ERXFCON_UCEN    (1 << 7) /* Bit 7: Unicast Filter Enable */

/* Bank 2 Control Register Addresses */

#define ENC_MACON1      REGADDR(0x00, 2, 1) /* MAC Control 1 */
                                            /* 0x01: Reserved */
#define ENC_MACON3      REGADDR(0x02, 2, 1) /* MAC Control 3 */
#define ENC_MACON4      REGADDR(0x03, 2, 1) /* MAC Control 4 */
#define ENC_MABBIPG     REGADDR(0x04, 2, 1) /* Back-to-Back Inter-Packet Gap (BBIPG<6:0>) */
                                            /* 0x05: Reserved */
#define ENC_MAIPGL      REGADDR(0x06, 2, 1) /* Non-Back-to-Back Inter-Packet Gap Low Byte (MAIPGL<6:0>) */
#define ENC_MAIPGH      REGADDR(0x07, 2, 1) /* Non-Back-to-Back Inter-Packet Gap High Byte (MAIPGH<6:0>) */
#define ENC_MACLCON1    REGADDR(0x08, 2, 1) /* MAC Collision Control 1 */
#define ENC_MACLCON2    REGADDR(0x09, 2, 1) /* MAC Collision Control 2 */
#define ENC_MAMXFLL     REGADDR(0x0a, 2, 1) /* Maximum Frame Length Low Byte (MAMXFL<7:0>) */
#define ENC_MAMXFLH     REGADDR(0x0b, 2, 1) /* Maximum Frame Length High Byte (MAMXFL<15:8>) */
                                            /* 0x0c-0x11: Reserved */
#define ENC_MICMD       REGADDR(0x12, 2, 1) /* MII Command Register */
                                            /* 0x13: Reserved */
#define ENC_MIREGADR    REGADDR(0x14, 2, 1) /* MII Register Address */
                                            /* 0x15: Reserved */
#define ENC_MIWRL       REGADDR(0x16, 2, 1) /* MII Write Data Low Byte (MIWR<7:0>) */
#define ENC_MIWRH       REGADDR(0x17, 2, 1) /* MII Write Data High Byte (MIWR<15:8>) */
#define ENC_MIRDL       REGADDR(0x18, 2, 1) /* MII Read Data Low Byte (MIRD<7:0>) */
#define ENC_MIRDH       REGADDR(0x19, 2, 1) /* MII Read Data High Byte(MIRD<15:8>) */
                                            /* 0x1a: Reserved */
                                            /* 0x1b-0x1f: EIE, EIR, ESTAT, ECON2, ECON1 */

/* MAC Control 1 Register Bit Definitions */

#define MACON1_MARXEN   (1 << 0) /* Bit 0: MAC Receive Enable */
#define MACON1_PASSALL  (1 << 1) /* Bit 1: Pass All Received Frames Enable */
#define MACON1_RXPAUS   (1 << 2) /* Bit 2: Pause Control Frame Reception Enable */
#define MACON1_TXPAUS   (1 << 3) /* Bit 3: Pause Control Frame Transmission Enable */
                                 /* Bits 4-7: Unimplemented or reserved */

/* MAC Control 1 Register Bit Definitions */

#define MACON3_FULDPX   (1 << 0) /* Bit 0: MAC Full-Duplex Enable */
#define MACON3_FRMLNEN  (1 << 1) /* Bit 1: Frame Length Checking Enable */
#define MACON3_HFRMLEN  (1 << 2) /* Bit 2: Huge Frame Enable */
#define MACON3_PHDRLEN  (1 << 3) /* Bit 3: Proprietary Header Enable */
#define MACON3_TXCRCEN  (1 << 4) /* Bit 4: Transmit CRC Enable */
#define MACON3_PADCFG0  (1 << 5) /* Bit 5: Automatic Pad and CRC Configuration */
#define MACON3_PADCFG1  (1 << 6) /* Bit 6: "       " " " " " " " "           " */
#define MACON3_PADCFG2  (1 << 7) /* Bit 7: "       " " " " " " " "           " */

/* MAC Control 1 Register Bit Definitions */

#define MACON4_NOBKOFF  (1 << 4) /* Bit 4: No Backoff Enable */
#define MACON4_BPEN     (1 << 5) /* Bit 5: No Backoff During Backpressure Enable */
#define MACON4_DEFER    (1 << 6) /* Bit 6: Defer Transmission Enable bit */

/* MII Command Register Bit Definitions */

#define MICMD_MIIRD     (1 << 0) /* Bit 0: MII Read Enable */
#define MICMD_MIISCAN   (1 << 1) /* Bit 1: MII Scan Enable */

/* Bank 3 Control Register Addresses */

#define ENC_MAADR5      REGADDR(0x00, 3, 1) /* MAC Address Byte 5 (MAADR<15:8>) */
#define ENC_MAADR6      REGADDR(0x01, 3, 1) /* MAC Address Byte 6 (MAADR<7:0>) */
#define ENC_MAADR3      REGADDR(0x02, 3, 1) /* MAC Address Byte 3 (MAADR<31:24>), OUI Byte 3 */
#define ENC_MAADR4      REGADDR(0x03, 3, 1) /* MAC Address Byte 4 (MAADR<23:16>) */
#define ENC_MAADR1      REGADDR(0x04, 3, 1) /* MAC Address Byte 1 (MAADR<47:40>), OUI Byte 1 */
#define ENC_MAADR2      REGADDR(0x05, 3, 1) /* MAC Address Byte 2 (MAADR<39:32>), OUI Byte 2 */
#define ENC_EBSTSD      REGADDR(0x06, 3, 0) /* Built-in Self-Test Fill Seed (EBSTSD<7:0>) */
#define ENC_EBSTCON     REGADDR(0x07, 3, 0) /* Built-in Self-Test Control */
#define ENC_EBSTCSL     REGADDR(0x08, 3, 0) /* Built-in Self-Test Checksum Low Byte (EBSTCS<7:0>) */
#define ENC_EBSTCSH     REGADDR(0x09, 3, 0) /* Built-in Self-Test Checksum High Byte (EBSTCS<15:8>) */
#define ENC_MISTAT      REGADDR(0x0a, 3, 1) /* MII Status Register */
                                            /* 0x0b-0x11: Reserved */
#define ENC_EREVID      REGADDR(0x12, 3, 0) /* Ethernet Revision ID */
                                            /* 0x13-0x14: Reserved */
#define ENC_ECOCON      REGADDR(0x15, 3, 0) /* Clock Output Control */
                                            /* 0x16: Reserved */
#define ENC_EFLOCON     REGADDR(0x17, 3, 0) /* Ethernet Flow Control */
#define ENC_EPAUSL      REGADDR(0x18, 3, 0) /* Pause Timer Value Low Byte (EPAUS<7:0>) */
#define ENC_EPAUSH      REGADDR(0x19, 3, 0) /* Pause Timer Value High Byte (EPAUS<15:8>) */
                                            /* 0x1a: Reserved */
                                            /* 0x1b-0x1f: EIE, EIR, ESTAT, ECON2, ECON1 */

/* Built-in Self-Test Control Register Bit Definitions */

#define EBSTCON_BISTST  (1 << 0) /* Bit 0: Built-in Self-Test Start/Busy */
#define EBSTCON_TME     (1 << 1) /* Bit 1: Test Mode Enable */
#define EBSTCON_TMSEL0  (1 << 2) /* Bit 2: Test Mode Select */
#define EBSTCON_TMSEL1  (1 << 3) /* Bit 3: "  " "  " "    " */
#define EBSTCON_PSEL    (1 << 4) /* Bit 4: Port Select */
#define EBSTCON_PSV0    (1 << 5) /* Bit 5: Pattern Shift Value */
#define EBSTCON_PSV1    (1 << 6) /* Bit 6: "     " "   "     " */
#define EBSTCON_PSV2    (1 << 7) /* Bit 7: "     " "   "     " */

/* MII Status Register Register Bit Definitions */

#define MISTAT_BUSY     (1 << 0) /* Bit 0: MII Management Busy */
#define MISTAT_SCAN     (1 << 1) /* Bit 1: MII Management Scan Operation */
#define MISTAT_NVALID   (1 << 2) /* Bit 2: MII Management Read Data Not Valid */
                                 /* Bits 3-7: Reserved or unimplemented */

/* Ethernet Flow Control Register Bit Definitions */

#define EFLOCON_FCEN0   (1 << 0) /* Bit 0: Flow Control Enable */
#define EFLOCON_FCEN1   (1 << 1) /* Bit 1: "  " "     " "    " */
#define EFLOCON_FULDPXS (1 << 2) /* Bit 2: Read-Only MAC Full-Duplex Shadow */
                                 /* Bits 3-7: Reserved or unimplemented */

/* PHY Registers ************************************************************/

#define ENC_PHCON1        (0x00)    /* PHY Control Register 1 */
#define ENC_PHSTAT1       (0x01)    /* PHY Status 1 */
#define ENC_PHID1         (0x02)    /* PHY ID Register 1 */
#define ENC_PHID2         (0x03)    /* PHY ID Register 2 */
#define ENC_PHCON2        (0x10)    /* PHY Control Register 2 */
#define ENC_PHSTAT2       (0x11)    /* PHY Status 2 */
#define ENC_PHIE          (0x12)    /* PHY Interrupt Enable Register */
#define ENC_PHIR          (0x13)    /* PHY Interrupt Request Register */
#define ENC_PHLCON        (0x14)

/* PHY Control Register 1 Register Bit Definitions */

#define PHCON1_PDPXMD     (1 << 8)  /* Bit 8:  PHY Duplex Mode */
#define PHCON1_PPWRSV     (1 << 11) /* Bit 11: PHY Power-Down */
#define PHCON1_PLOOPBK    (1 << 14) /* Bit 14: PHY Loopback */
#define PHCON1_PRST       (1 << 15) /* Bit 15: PHY Software Reset */

/* PHY Status 1 Register Bit Definitions */

#define PHSTAT1_JBSTAT    (1 << 1)  /* Bit 1: PHY Latching Jabber Status */
#define PHSTAT1_LLSTAT    (1 << 2)  /* Bit 2: PHY Latching Link Status */
#define PHSTAT1_PHDPX     (1 << 11) /* Bit 11: PHY Half-Duplex Capable */
#define PHSTAT1_PFDPX     (1 << 12) /* Bit 12: PHY Full-Duplex Capable */

/* PHY Control Register 2 Register Bit Definitions */

#define PHCON2_HDLDIS     (1 << 8)  /* Bit 8:  PHY Half-Duplex Loopback Disable */
#define PHCON2_JABBER     (1 << 10) /* Bit 10: Jabber Correction Disable */
#define PHCON2_TXDIS      (1 << 13) /* Bit 13: Twisted-Pair Transmitter Disable */
#define PHCON2_FRCLINK    (1 << 14) /* Bit 14: PHY Force Linkup */

/* PHY Status 2 Register Bit Definitions */

#define PHSTAT2_PLRITY    (1 << 5)  /* Bit 5:  Polarity Status */
#define PHSTAT2_DPXSTAT   (1 << 9)  /* Bit 9:  PHY Duplex Status */
#define PHSTAT2_LSTAT     (1 << 10) /* Bit 10: PHY Link Status */
#define PHSTAT2_COLSTAT   (1 << 11) /* Bit 11: PHY Collision Status */
#define PHSTAT2_RXSTAT    (1 << 12) /* Bit 12: PHY Receive Status */
#define PHSTAT2_TXSTAT    (1 << 13) /* Bit 13: PHY Transmit Status */

/* PHY Interrupt Enable Register Bit Definitions */

#define PHIE_PGEIE        (1 << 1)  /* Bit 1:  PHY Global Interrupt Enable */
#define PHIE_PLNKIE       (1 << 4)  /* Bit 4:  PHY Link Change Interrupt Enable */

/* PHIR Regiser Bit Definitions */

#define PHIR_PGIF         (1 << 2)  /* Bit 2:  PHY Global Interrupt */
#define PHIR_PLNKIF       (1 << 4)  /* Bit 4:  PHY Link Change Interrupt */

/* PHLCON Regiser Bit Definitions */
                                    /* Bit 0:  Reserved */
#define PHLCON_STRCH      (1 << 1)  /* Bit 1:  LED Pulse Stretching Enable */
#define PHLCON_LFRQ0      (1 << 2)  /* Bit 2:  LED Pulse Stretch Time Configuration */
#define PHLCON_LFRQ1      (1 << 3)  /* Bit 3:  " " "   " "     " "  " " */      
#define PHLCON_LBCFG0     (1 << 4)  /* Bit 4:  LEDB Configuration */
#define PHLCON_LBCFG1     (1 << 5)  /* Bit 5:  "  " "           " */
#define PHLCON_LBCFG2     (1 << 6)  /* Bit 6:  "  " "           " */
#define PHLCON_LBCFG3     (1 << 7)  /* Bit 7:  "  " "           " */
#define PHLCON_LACFG0     (1 << 8)  /* Bit 8:  LEDA Configuration */
#define PHLCON_LACFG1     (1 << 9)  /* Bit 9:  "  " "           " */
#define PHLCON_LACFG2     (1 << 10) /* Bit 10: "  " "           " */
#define PHLCON_LACFG3     (1 << 11) /* Bit 11: "  " "           " */

/* Packet Memory ************************************************************/

/* 8-Kbyte Transmit/Receive Packet Dual Port SRAM */

#define PKTMEM_START      0x0000
#define PKTMEM_END        0x1fff

/* Ethernet frames are between 64 and 1518 bytes long */

#define MIN_FRAMELEN      64
#define MAX_FRAMELEN      1518

/* Packet Control Bits Definitions ******************************************/

#define PKTCTRL_POVERRIDE (1 << 0)  /* Bit 0:  Per Packet Override */
#define PKTCTRL_PCRCEN    (1 << 1)  /* Bit 1:  Per Packet CRC Enable */
#define PKTCTRL_PPADEN    (1 << 2)  /* Bit 2:  Per Packet Padding Enable */
#define PKTCTRL_PHUGEEN   (1 << 3)  /* Bit 3:  Per Packet Huge Frame Enable */

/* RX Status Bit Definitions ************************************************/

#define RXSTAT_LDEVENT    (1 << 0)  /* Bit 0:  Long event or pack dropped */
                                    /* Bit 1:  Reserved */
#define RXSTAT_CEPS       (1 << 2)  /* Bit 2:  Carrier event previously seen */
                                    /* Bit 3:  Reserved */
#define RXSTAT_CRCERROR   (1 << 4)  /* Bit 4:  Frame CRC field bad */
#define RXSTAT_LENERROR   (1 << 5)  /* Bit 5:  Packet length != data length */
#define RXSTAT_LENRANGE   (1 << 6)  /* Bit 6:  Type/length field > 1500 bytes */
#define RXSTAT_OK         (1 << 7)  /* Bit 7:  Packet with valid CRC and no symbol errors */
#define RXSTAT_MCAST      (1 << 8)  /* Bit 8:  Packet with multicast address */
#define RXSTAT_BCAST      (1 << 9)  /* Bit 9:  Packet with broadcast address */
#define RXSTAT_DRIBBLE    (1 << 10) /* Bit 10: Additional bits received after packet */
#define RXSTAT_CTRLFRAME  (1 << 11) /* Bit 11: Control frame with valid type/length */
#define RXSTAT_PAUSE      (1 << 12) /* Bit 12: Control frame with pause frame opcde */
#define RXSTAT_UNKOPCODE  (1 << 13) /* Bit 13: Control frame with unknown opcode */
#define RXSTAT_VLANTYPE   (1 << 14) /* Bit 14: Current frame is a VLAN tagged frame */
                                    /* Bit 15: Zero */

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

#endif /* __DRIVERS_NET_ENC28J60_H */
