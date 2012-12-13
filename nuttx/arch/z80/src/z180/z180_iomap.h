/************************************************************************************
 * arch/z80/src/z180/z180_iomap.h
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
 ************************************************************************************/

#ifndef __ARCH_Z80_SRC_Z180_Z180_IOMAP_H
#define __ARCH_Z80_SRC_Z180_Z180_IOMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <arch/z180/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

/* These registers may be relocated to multiples of 0x40 by setting the IO Control
 * Register (ICR).  Relocatable to 0x40-0x7f, or 0x80-0xbf. The configuration setting,
 * CONFIG_Z180_SFROFFSET, indicates that offset (but is not fully supported yet!)
 */

#ifdef CONFIG_Z180_SFROFFSET
#  define SFR_OFFSET CONFIG_Z180_SFROFFSET
#else
#  define SFR_OFFSET 0
#endif

/* Z180 Register Bit addresses ******************************************************/
/* ASCI Registers */

#define Z180_ASCI0_CNTLA     (SFR_OFFSET+0x00) /* ASCI Control Register A Ch 0 */
#define Z180_ASCI1_CNTLA     (SFR_OFFSET+0x01) /* ASCI Control Register A Ch 1 */
#define Z180_ASCI0_CNTLB     (SFR_OFFSET+0x02) /* ASCI Control Register B Ch 0 */
#define Z180_ASCI1_CNTLB     (SFR_OFFSET+0x03) /* ASCI Control Register B Ch 1 */
#define Z180_ASCI0_STAT      (SFR_OFFSET+0x04) /* ASCI Status Register Ch 0 */
#define Z180_ASCI1_STAT      (SFR_OFFSET+0x05) /* ASCI Status Register Ch 1 */
#define Z180_ASCI0_TDR       (SFR_OFFSET+0x06) /* ASCI Transmit Data Register Ch 0 */
#define Z180_ASCI1_TDR       (SFR_OFFSET+0x07) /* ASCI Transmit Data Register Ch 1 */
#define Z180_ASCI0_RDR       (SFR_OFFSET+0x08) /* ASCI Receive Data Register Ch 0 */
#define Z180_ASCI1_RDR       (SFR_OFFSET+0x09) /* ASCI Receive Data Register Ch 1 */

#ifdef HAVE_Z8S180 /* Z8S180/Z8L180 class processors */
#  define Z180_ASCI0_ASEXT   (SFR_OFFSET+0x12) /* ASCI Extension Control Register */
#  define Z180_ASCI1_ASEXT   (SFR_OFFSET+0x13) /* ASCI Extension Control Register */
#endif

#ifdef HAVE_Z8S180 /* Z8S180/Z8L180 class processors */
#  define Z180_ASCI0_ASTCL   (SFR_OFFSET+0x1a) /* ASCI Time Constant Low */
#  define Z180_ASCI0_ASTCH   (SFR_OFFSET+0x1b) /* ASCI Time Constant High */
#  define Z180_ASCI1_ASTCL   (SFR_OFFSET+0x1c) /* ASCI Time Constant Low */
#  define Z180_ASCI1_ASTCH   (SFR_OFFSET+0x1d) /* ASCI Time Constant High */
#endif

/* CSI/O Registers */

#define Z180_CSIO_CNTR       (SFR_OFFSET+0x0a) /* CSI/O Control Register */
#define Z180_CSIO_TRD        (SFR_OFFSET+0x0b) /* Transmit/Receive Data Register */

/* Timer Registers */

#define Z180_TMR0_DRL        (SFR_OFFSET+0x0c) /* Timer Data Register Ch 0 L */
#define Z180_TMR0_DRH        (SFR_OFFSET+0x0d) /* Data Register Ch 0 H */
#define Z180_TMR0_RLDRL      (SFR_OFFSET+0x0e) /* Reload Register Ch 0 L */
#define Z180_TMR0_RLDRH      (SFR_OFFSET+0x0f) /* Reload Register Ch 0 H */
#define Z180_TMR_TCR         (SFR_OFFSET+0x10) /* Timer Control Register */

#define Z180_TMR1_DRL        (SFR_OFFSET+0x14) /* Data Register Ch 1 L */
#define Z180_TMR1_DRH        (SFR_OFFSET+0x15) /* Data Register Ch 1 H */
#define Z180_TMR1_RLDRL      (SFR_OFFSET+0x16) /* Reload Register Ch 1 L */
#define Z180_TMR1_RLDRH      (SFR_OFFSET+0x17) /* Reload Register Ch 1 H */

#define Z180_FRC             (SFR_OFFSET+0x18) /* Free Running Counter */

/* DMA Registers */

#define Z180_DMA_SAR0L       (SFR_OFFSET+0x20) /* DMA Source Address Register Ch 0L */
#define Z180_DMA_SAR0H       (SFR_OFFSET+0x21) /* DMA Source Address Register Ch 0H */
#define Z180_DMA_SAR0B       (SFR_OFFSET+0x22) /* DMA Source Address Register Ch 0B */
#define Z180_DMA_DAR0L       (SFR_OFFSET+0x23) /* DMA Destination Address Register Ch 0L */
#define Z180_DMA_DAR0H       (SFR_OFFSET+0x24) /* DMA Destination Address Register Ch 0H */
#define Z180_DMA_DAR0B       (SFR_OFFSET+0x25) /* DMA Destination Address Register Ch 0B */
#define Z180_DMA_BCR0L       (SFR_OFFSET+0x26) /* DMA Byte Count Register Ch 0L */
#define Z180_DMA_BCR0H       (SFR_OFFSET+0x27) /* DMA Byte Count Register Ch 0H */
#define Z180_DMA_MAR1L       (SFR_OFFSET+0x28) /* DMA Memory Address Register Ch 1L */
#define Z180_DMA_MAR1H       (SFR_OFFSET+0x29) /* DMA Memory Address Register Ch 1H */
#define Z180_DMA_MAR1B       (SFR_OFFSET+0x2a) /* DMA Memory Address Register Ch 1B */
#define Z180_DMA_IAR1L       (SFR_OFFSET+0x2b) /* DMA I/0 Address Register Ch 1L */
#define Z180_DMA_IAR1H       (SFR_OFFSET+0x2c) /* DMA I/0 Address Register Ch 1H */
#ifdef HAVE_Z8S180 /* Z8S180/Z8L180 class processors */
#  define Z180_DMA_IAR1B     (SFR_OFFSET+0x2d) /* DMA I/O Address Register Ch 1B */
#endif
#define Z180_DMA_BCR1L       (SFR_OFFSET+0x2e) /* DMA Byte Count Register Ch 1L */
#define Z180_DMA_BCR1H       (SFR_OFFSET+0x2f) /* DMA Byte Count Register Ch 1H */
#define Z180_DMA_DSTAT       (SFR_OFFSET+0x30) /* DMA Status Register */
#define Z180_DMA_DMODE       (SFR_OFFSET+0x31) /* DMA Mode Register */
#define Z180_DMA_DCNTL       (SFR_OFFSET+0x32) /* DMA/WAIT Control Register */

/* System Control Registers */

#ifdef HAVE_Z8S180 /* Z8S180/Z8L180 class processors */
#  define Z180_CMR           (SFR_OFFSET+0x1e) /* Clock Multiplier Register */
#endif

#if defined(HAVE_Z8S180) || defined(HAVE_Z8X182)
#  define Z180_CCR           (SFR_OFFSET+0x1f) /* CPU Control Register */
#endif

#define Z180_INT_IL          (SFR_OFFSET+0x33) /* IL Register (Interrupt Vector Low Register) */
#define Z180_INT_ITC         (SFR_OFFSET+0x34) /* INT/TRAP Control Register */

#define Z180_RCR             (SFR_OFFSET+0x36) /* Refresh Control Register */

#define Z180_MMU_CBR         (SFR_OFFSET+0x38) /* MMU Common Base Register */
#define Z180_MMU_BBR         (SFR_OFFSET+0x39) /* MMU Bank Base Register */
#define Z180_MMU_CBAR        (SFR_OFFSET+0x3a) /* MMU Common/Bank Area Register */

#define Z180_OMCR            (SFR_OFFSET+0x3e) /* Operation Mode Control Register */
#define Z180_ICR             (SFR_OFFSET+0x3f) /* I/O Control Register */

/* The following registers are not relocatable */
/* Registers unique to Z8x181 class CPUs */

#ifdef HAVE_Z8X181

/* PIA Registers */

#  define Z181_PIA1_DDR      0xe0 /* PIA1 Data Direction Register */
#  define Z181_PIA1_DP       0xe1 /* PIA1 Data Port */
#  define Z181_PIA2_DDR      0xe2 /* PIA2 Data Direction Register */
#  define Z181_PIA1_DP       0xe3 /* PIA2 Data Register */

/* CTC Registers */

#  define Z181_CTC0          0xe4 /* CTC Channel 0 Control Register */
#  define Z181_CTC1          0xe5 /* CTC Channel 1 Control Register */
#  define Z181_CTC2          0xe6 /* CTC Channel 2 Control Register */
#  define Z181_CTC3          0xe7 /* CTC Channel 3 Control Register */

/* SCC Registers */

#  define Z181_SCC_CR        0xe8 /* SCC Control Register */
#  define Z181_SCC_DR        0xe9 /* SCC Data Register */

/* System Control Registers */

#  define Z181_RAM_UBR       0xea /* RAM Upper Boundary Address Register */
#  define Z181_RAM_LBR       0xeb /* RAM Lower Boundary Address Register*/
#  define Z181_ROM_BR        0xec /* ROM Address Boundary Register */
#  define Z181_SCR           0xed /* System Configuration Register */
#endif

/* Registers unique to Z8x182 class CPUs */

#ifdef HAVE_Z8X182
#  define Z182_WSGCS       0xd8 /* WSG Chip Select Register */
#  define Z182_ENH182      0xd9 /* Z80182 Enhancements Register */
#  define Z182_INTEDGE     0xdf /* Interrupt Edge/Pin MUX Control */
#  define Z182_PINMUX      0xdf /* Interrupt Edge/Pin MUX Control */

/* PIA Registers */

#  define Z182_PA_DDR      0xed /* PA Data Direction Register */
#  define Z182_PA_DR       0xee /* PA Data Register */
#  define Z182_PB_DDR      0xe4 /* PB Data Direction Register */
#  define Z182_PB_DR       0xe5 /* PB Data Register */
#  define Z182_PC_DDR      0xdd /* PC Data Direction Register */
#  define Z182_PC_DR       0xde /* PC Data Register */

/* ESCC Registers */

#  define Z182_ESCCA_CR    0xe0 /* ESCC Chan A Control Register */
#  define Z182_ESCCA_DR    0xe1 /* ESCC Chan A Data Register */
#  define Z182_ESCCB_CR    0xe2 /* ESCC Chan B Control Register */
#  define Z182_ESCCB_DR    0xe3 /* ESCC Chan B Data Register */

/* System Control Registers */

#  define Z182_RAM_UBR     0xe6 /* RAMUBR RAM Upper Boundary Register */
#  define Z182_RAM_LBR     0xe7 /* RAMLBR RAM Lower Boundary Register */
#  define Z182_ROM_BR      0xe8 /* ROM Address Boundary Register */
#  define Z182_SCR         0xef /* System Configuration Register */

/* 16550 MIMIC Registers */

#  define Z182_MIMIC_FCR   0xe9 /* FIFO Control Register */
#  define Z182_MIMIC_MM    0xe9 /* MM register */
#  define Z182_MIMIC_RTTC  0xea /* Receive Timeout Time Constant */
#  define Z182_MIMIC_TTTC  0xeb /* Transmit Timeout Time Constant */
#  define Z182_MIMIC_FSCR  0xec /* FIFO Status and Control */
#  define Z182_MIMIC_RBR   0xf0 /* Receive Buffer Register */
#  define Z182_MIMIC_THR   0xf0 /* Transmit Holding Register */
#  define Z182_MIMIC_IER   0xf1 /* Interrupt Enable Register */
#  define Z182_MIMIC_LCR   0xf3 /* Line Control Register */
#  define Z182_MIMIC_MCR   0xf4 /* Modem Control Register */
#  define Z182_MIMIC_LSR   0xf5 /* Line Status Register */
#  define Z182_MIMIC_MSR   0xf6 /* Modem Status Register */
#  define Z182_MIMIC_SCR   0xf7 /* Scratch Register */
#  define Z182_MIMIC_DLL   0xf8 /* Divisor Latch (LSByte) */
#  define Z182_MIMIC_DLM   0xf9 /* Divisor Latch (MSByte) */
#  define Z182_MIMIC_TTCR  0xfa /* Transmit Time Constant */
#  define Z182_MIMIC_RTCR  0xfb /* Receive Time Constant */
#  define Z182_MIMIC_IVEC  0xfc /* Interrupt Vector */
#  define Z182_MIMIC_IE    0xfd /* Interrupt Enable */

#  define Z182_MIMIC_IUSIP 0xfe /* Interrupt Under-Service/Interrupt Pending */
#  define Z182_MIMIC_MMC   0xff /* MIMIC Master Control Register  */

/* Some of the MIMIC registers are accessible to memory-mapped addresses */

#  define Z182_MIMIC_RBR_ADDR 0x0000 /* Receive Buffer Register */
#  define Z182_MIMIC_DLL_ADDR 0x0000 /* Divisor Latch (LSByte) */
#  define Z182_MIMIC_THR_ADDR 0x0000 /* Transmit Holding Register */
#  define Z182_MIMIC_DLM_ADDR 0x0001 /* Divisor Latch (MSByte) */
#  define Z182_MIMIC_IER_ADDR 0x0001 /* Interrupt Enable Register */
#  define Z182_MIMIC_IIR_ADDR 0x0002 /* Interrupt Identification */
#  define Z182_MIMIC_FCR_ADDR 0x0002 /* FIFO Control Register */
#  define Z182_MIMIC_LCR_ADDR 0x0003 /* Line Control Register */
#  define Z182_MIMIC_MCR_ADDR 0x0004 /* Modem Control Register */
#  define Z182_MIMIC_LSR_ADDR 0x0005 /* Line Status Register */
#  define Z182_MIMIC_MSR_ADDR 0x0006 /* Modem Status Register */
#  define Z182_MIMIC_SCR_ADDR 0x0007 /* Scratch Register */
#endif

/* [E]SCC Internal Register Definitions */

#define Z18X_SCC_RR0       0x00
#define Z18X_SCC_RR1       0x01
#define Z18X_SCC_RR2       0x02
#define Z18X_SCC_RR3       0x03
#define Z18X_SCC_RR6       0x06
#define Z18X_SCC_RR7       0x07
#define Z18X_SCC_RR10      0x0a
#define Z18X_SCC_RR12      0x0c
#define Z18X_SCC_RR13      0x0d
#define Z18X_SCC_RR15      0x0f

#define Z18X_SCC_WR0       0x00
#define Z18X_SCC_WR1       0x01
#define Z18X_SCC_WR2       0x02
#define Z18X_SCC_WR3       0x03
#define Z18X_SCC_WR4       0x04
#define Z18X_SCC_WR5       0x05
#define Z18X_SCC_WR6       0x06
#define Z18X_SCC_WR7       0x07
#define Z18X_SCC_WR9       0x09
#define Z18X_SCC_WR10      0x0a
#define Z18X_SCC_WR11      0x0b
#define Z18X_SCC_WR12      0x0c
#define Z18X_SCC_WR13      0x0d
#define Z18X_SCC_WR14      0x0e
#define Z18X_SCC_WR15      0x0f

/* Z180 Register Bit definitions ****************************************************/
/* ASCI Registers *******************************************************************/
/* ASCI Control Register A 0 (CNTLA0: 0x00) */
/* ASCI Control Register A 1 (CNTLA1: 0x01) */

#define ASCI_CNTRLA_MPE      (0x80) /* Bit 7: Multi-Processor Mode Enable */
#define ASCI_CNTRLA_RE       (0x40) /* Bit 6: Receiver Enab */
#define ASCI_CNTRLA_TE       (0x20) /* Bit 5: Transmitter Enable */
#define ASCI_CNTRLA_RTS0     (0x10) /* Bit 4: Request to Send Channel 0 (ASCI0 only) */
#define ASCI_CNTRLA_CKA1D    (0x10) /* Bit 4: CKA1 Clock Disable (ASCI1 only) */
#define ASCI_CNTRLA_MPBR     (0x08) /* Bit 3: Multiprocessor Bit Receive */
#define ASCI_CNTRLA_EFR      (0x08) /* Bit 3: Error Flag Reset */
#define ASCI_CNTRLA_MOD2     (0x04) /* Bit 2: 8 bit data */
#define ASCI_CNTRLA_MOD1     (0x02) /* Bit 1: Parity enabled */
#define ASCI_CNTRLA_MOD0     (0x01) /* Bit 0: Parity enabled */

/* ASCI Control Register B 0 (CNTLB0: 0x02) */
/* ASCI Control Register B 1 (CNTLB1: 0x03) */

#define ASCI_CNTRLB_MPBT     (0x80) /* Bit 7: Multiprocessor Bit Transmit */
#define ASCI_CNTRLB_MP       (0x40) /* Bit 6: Multiprocessor Mode */
#define ASCI_CNTRLB_CTS      (0x20) /* Bit 5: Clear to Send */
#define ASCI_CNTRLB_PS       (0x20) /* Bit 5: Prescale */
#define ASCI_CNTRLB_PEO      (0x10) /* Bit 4: Parity Even Odd */
#define ASCI_CNTRLB_DR       (0x08) /* Bit 3: Divide Ratio */

#define ASCI_CNTRLB_SS_SHIFT   (0) /* Bits 0-2: Source/Speed Select */
#define ASCI_CNTRLB_SS_MASK    (7 << ASCI_CNTRLB_SS_SHIFT)
#  define ASCI_CNTRLB_SS_DIV1  (0 << ASCI_CNTRLB_SS_SHIFT) /* Divide Ratio: 1 */
#  define ASCI_CNTRLB_SS_DIV2  (1 << ASCI_CNTRLB_SS_SHIFT) /* Divide Ratio: 2 */
#  define ASCI_CNTRLB_SS_DIV4  (2 << ASCI_CNTRLB_SS_SHIFT) /* Divide Ratio: 4 */
#  define ASCI_CNTRLB_SS_DIV8  (3 << ASCI_CNTRLB_SS_SHIFT) /* Divide Ratio: 8 */
#  define ASCI_CNTRLB_SS_DIV16 (4 << ASCI_CNTRLB_SS_SHIFT) /* Divide Ratio: 16 */
#  define ASCI_CNTRLB_SS_DIV32 (5 << ASCI_CNTRLB_SS_SHIFT) /* Divide Ratio: 32 */
#  define ASCI_CNTRLB_SS_DIV64 (6 << ASCI_CNTRLB_SS_SHIFT) /* Divide Ratio: 64 */
#  define ASCI_CNTRLB_SS_EXT   (7 << ASCI_CNTRLB_SS_SHIFT) /* External clock */

/* ASCI Status Register 0 (STAT0: 0x04) */
/* ASCI Status Register 1 (STAT1: 0x05) */

#define ASCI_STAT_RFRF       (0x80) /* Bit 7: Receive Data Register Full */
#define ASCI_STAT_OVRN       (0x40) /* Bit 6: Overrun Error */
#define ASCI_STAT_PE         (0x20) /* Bit 5: Parity Error */
#define ASCI_STAT_FE         (0x10) /* Bit 4: Framing Error */
#define ASCI_STAT_RIE        (0x08) /* Bit 3: Receive Interrupt Enable */
#define ASCI_STAT_DCD0       (0x04) /* Bit 2: Data Carrier Detect (ASCI0 only) */
#define ASCI_STAT_CTS1E      (0x04) /* Bit 2: Channel 1 CTS Enable (ASCI1 only) */
#define ASCI_STAT_TDRE       (0x02) /* Bit 1: Transmit Data Register Empty */
#define ASCI_STAT_TIE        (0x01) /* Bit 0: Transmit Interrupt Enable */

/* ASCI Transmit Data Register Ch. 0 (TDR0: 0x06) - 8-bit data */
/* ASCI Transmit Data Register Ch. 1 (TDR1: 0x07) - 8-bit data */
/* ASCI Receive Data Register Ch. 0 (RDR0: 0x08) - 8-bit data */
/* ASCI Receive Data Register Ch. 1 (RDR0: 0x09) - 8-bit data */

/* ASCI0 Extension Control Register (I/O Address: 0x12) (Z8S180/L180-Class Processors Only) */
/* ASCI1 Extension Control Register (I/O Address: 0x13) (Z8S180/L180-Class Processors Only) */

#ifdef HAVE_Z8S180 /* Z8S180/Z8L180 class processors */
#  define ASCI_ASEXT_RDRF    (0x80) /* Bit 7: RDRF Interrupt Inhibit */
#  define ASCI0_ASEXT_DCD0   (0x80) /* Bit 6: DCD0 advisory to SW (ASCI0 only) */
#  define ASCI0_ASEXT_CTS0   (0x80) /* Bit 5: CTS0 advisory to SW (ASCI0 only) */
#  define ASCI_ASEXT_X1BC    (0x80) /* Bit 4: CKA0 is bit clock */
#  define ASCI_ASEXT_BRG     (0x80) /* Bit 3: Enable 16-bit BRG counter */
#  define ASCI_ASEXT_BRKEN   (0x80) /* Bit 2: Break Feature Enable */
#  define ASCI_ASEXT_BRKDET  (0x80) /* Bit 1: Break Detect */
#  define ASCI_ASEXT_SNDBRK  (0x80) /* Bit 0: Send Break */
#endif

/* ASCI0 Time Constant Low Register (I/O Address: 0x1a) (Z8S180/L180-Class Processors Only) -- 8-bit data */
/* ASCI0 Time Constant High Register (I/O Address: 0x1b) (Z8S180/L180-Class Processors Only) -- 8-bit data */
/* ASCI1 Time Constant Low Register (I/O Address: 0x1c) (Z8S180/L180-Class Processors Only) -- 8-bit data */
/* ASCI1 Time Constant High Register (I/O Address: 0x1d) (Z8S180/L180-Class Processors Only) -- 8-bit data */

/* CSI/O Registers ******************************************************************/
/* CSI/O Control/Status Register (CNTR: 0x0a) */

#define CSIO_CNTR_EF         (0x80) /* Bit 7: End Flag */
#define CSIO_CNTR_EIE        (0x40) /* Bit 6: End Interrupt Enable */
#define CSIO_CNTR_RE         (0x20) /* Bit 5: Receive Enable */
#define CSIO_CNTR_TE         (0x10) /* Bit 4: Transmit Enable */
#define CSIO_CNTR_SS_SHIFT   (0)    /* Bits 0-2: Speed Select */
#define CSIO_CNTR_SS_MASK    (7 << CSIO_CNTR_SS_SHIFT)
#  define CSIO_CNTR_DIV20    (0 << CSIO_CNTR_SS_SHIFT) /* Divide Ratio: 20 Baud: 200000 */
#  define CSIO_CNTR_DIV40    (1 << CSIO_CNTR_SS_SHIFT) /* Divide Ratio: 40 Baud: 100000 */
#  define CSIO_CNTR_DIV80    (2 << CSIO_CNTR_SS_SHIFT) /* Divide Ratio: 80 Baud: 50000 */
#  define CSIO_CNTR_DIV160   (3 << CSIO_CNTR_SS_SHIFT) /* Divide Ratio: 160 Baud: 25000 */
#  define CSIO_CNTR_DIV320   (4 << CSIO_CNTR_SS_SHIFT) /* Divide Ratio: 320 Baud: 12500 */
#  define CSIO_CNTR_DIV640   (5 << CSIO_CNTR_SS_SHIFT) /* Divide Ratio: 640 Baud: 6250 */
#  define CSIO_CNTR_DIV1280  (6 << CSIO_CNTR_SS_SHIFT) /* Divide Ratio: 1280 Baud: 3125 */
#  define CSIO_CNTR_EXT      (7 << CSIO_CNTR_SS_SHIFT) /* External Clock input (less than 20) */
                                                       /* Baud at Phi = 4 MHz */
/* CSI/O Transmit/Receive Register (TRDR: 0x0b) -- 8-bit data */

/* Timer Registers ******************************************************************/
/* Timer Data Register 0L (TMDR0L: 0x0c) -- 8-bit data */
/* Timer Data Register 0H (TMDR0H: 0x0d) -- 8-bit data */
/* Timer Reload Register Channel 0L (RLDR0L: 0x0e) -- 8-bit data */
/* Timer Reload Register Channel 0H (RLDR0H: 0x0f) -- 8-bit data */

/* Timer Control Register (TCR: 0x10) */

#define TMR_TCR_TIF1         (0x80) /* Bit 7: Timer 1 Interrupt Flag */
#define TMR_TCR_TIF0         (0x40) /* Bit 6: Timer 0 Interrupt Flag */
#define TMR_TCR_TIE1         (0x20) /* Bit 5: Timer 1 Interrupt Enable */
#define TMR_TCR_TIE0         (0x10) /* Bit 4: Timer 0 Interrupt Enable */
#define TMR_TCR_TOC1         (0x08) /* Bit 3: Timer 1 Output Control */
#define TMR_TCR_TOC0         (0x04) /* Bit 2: Timer 0 Output Control */
#define TMR_TCR_TDE1         (0x02) /* Bit 1: Timer 1 Down Count Enable */
#define TMR_TCR_TDE0         (0x01) /* Bit 0: Timer 0 Down Count Enable */

/* Timer Data Register 1L (TMDR1L: 0x14) -- 8-bit data */
/* Timer Data Register 1H (TMDR1H: 0x15) -- 8-bit data */
/* Timer Reload Register Channel 1L (RLDR1L: 0x16) -- 8-bit data */
/* Timer Reload Register Channel 1H (RLDR1H: 0x17) -- 8-bit data */
/* Free Running counter (FRC: 0x18) -- 8-bit data */

/* DMA Registers ********************************************************************/
/* DMA Destination Address Register Channel 0 (DAR0 I/O Address 0x23 to 0x25) -- 8-bit data */
/* DMA Byte Count Register Channel 0 (BCR0 I/O Address = 0x26 to 0x27) -- 8-bit data */
/* DMA Memory Address Register Channel 1 (MAR1: I/O Address = 0x28 to 0x2a) -- 8-bit data */
/* DMA I/O Address Register Channel 1 (IAR1: I/O Address = 0x2b to 0x2c) -- 8-bit data */

/* DMA I/O Address Register Ch. 1 (IAR1B: 0x2d) (Z8S180/L180-Class Processor Only) */

#ifdef HAVE_Z8S180 /* Z8S180/Z8L180 class processors */
#  define IAR1B_ALTCH        (0x80) /* Bit 7: Alternating Channels */
#  define IAR1B_CURRCH       (0x40) /* Bit 6: Currently selected DMA channel */
#  define IAR1B_TOUT         (0x08) /* Bit 3: TOUT/DREQ is TOUT Out */
#  define IAR1B_IO_SHIFT     (0)    /* Bits 0-2: I/O selection*/
#  define IAR1B_IO_MASK      (3 << IAR1B_IO_SHIFT)
#    define IAR1B_IO_TOUT    (0 << IAR1B_IO_SHIFT) /* DMA1 ext TOUT */
#    define IAR1B_IO_DREQ    (0 << IAR1B_IO_SHIFT) /* DMA1 ext DREQ */
#    define IAR1B_IO_ASCI0   (1 << IAR1B_IO_SHIFT) /* DMA1 ASCI0 */
#    define IAR1B_IO_ASCI1   (2 << IAR1B_IO_SHIFT) /* DMA1 ASCI1 */
#    define IAR1B_IO_ESCC    (3 << IAR1B_IO_SHIFT) /* DMA1 ESCC */
#    define IAR1B_IO_PIA     (7 << IAR1B_IO_SHIFT) /* DMA1 PIA27-20 (P1284) */
#endif

/* DMA Byte Count Register Channel 1 (BCR1: I/O Address = 0x2e to 0x2f) -- 8-bit data */

/* DMA Status Register (DSTAT: 0x30) */

#define DSTAT_DE1            (0x80) /* Bit 7: Enable Channel 1 */
#define DSTAT_DE0            (0x40) /* Bit 6: Enable Channel 0 */
#define DSTAT_DWE1           (0x20) /* Bit 5: Bit Write Enable 1 */
#define DSTAT_DWE0           (0x10) /* Bit 4: Bit Write Enable 0 */
#define DSTAT_DIE1           (0x08) /* Bit 3: DMA Interrupt Enable Channel 1 */
#define DSTAT_DIE0           (0x04) /* Bit 2: DMA Interrupt Enable Channel 0 */
#define DSTAT_DME            (0x01) /* Bit 0: DMA Main Enable */

/* DMA Mode Register (DMODE: 0x31) */

#define DMODE_DM_SHIFT       (4)    /* Bits 4-5: Destination Mode Channel 0 */
#define DMODE_DM_MASK        (3 << DMODE_DM_SHIFT)
#  define DMODE_DM_MEMINCR   (0 << DMODE_DM_SHIFT) /* Memory with address increment */
#  define DMODE_DM_MEMDECR   (1 << DMODE_DM_SHIFT) /* Memory with address decrement */
#  define DMODE_DM_MEM       (2 << DMODE_DM_SHIFT) /* Memory with fixed address */
#  define DMODE_DM_IO        (3 << DMODE_DM_SHIFT) /* I/O */
#define DMODE_SM_SHIFT       (2)    /* Bits 2-3: Source Mode Channel */
#define DMODE_SM_MASK        (3 << DMODE_SM_SHIFT)
#  define DMODE_SM_MEMINCR   (0 << DMODE_SM_SHIFT) /* Memory with address increment */
#  define DMODE_SM_MEMDECR   (1 << DMODE_SM_SHIFT) /* Memory with address decrement */
#  define DMODE_SM_MEM       (2 << DMODE_SM_SHIFT) /* Memory with fixed address */
#  define DMODE_SM_IO        (3 << DMODE_SM_SHIFT) /* I/O */
#define DMODE_MMODE          (0x01) /* Bit 0: DMA Memory Mode Channel 0 */

/* DMA/WAIT Control Register (DCNTL: 0x32) */

#define DCNTL_MWI_SHIFT      (6)    /* Bits 6-7: Memory Wait Insertion */
#define DCNTL_MWI_MASK       (3 << DCNTL_MWI_SHIFT)
#define DCNTL_IWI_SHIFT      (4)    /* Bits 4-5: Wait Insertion */
#define DCNTL_IWI_MASK       (3 << DCNTL_IWI_SHIFT)
#define DCNTL_DMS_SHIFT      (4)    /* Bits 2-3: DMA Request Sense */
#define DCNTL_DMS_MASK       (3 << DCNTL_DMS_SHIFT)
#  define DCNTL_DMS_DREQ0    (1 << DCNTL_DMS_SHIFT)
#  define DCNTL_DMS_DREQ1    (2 << DCNTL_DMS_SHIFT)
#define DCNTL_DIM_SHIFT      (0)    /* Bits 0-1: DMA Channel 1 I/O and Memory Mode */
#define DCNTL_DIM_MASK       (3 << DCNTL_DIM_SHIFT)
#  define DCNTL_DIM_M2IOI    (0 << DCNTL_DIM_SHIFT) /* Memory to I/O, increment MARI */
#  define DCNTL_DIM_M2IOD    (1 << DCNTL_DIM_SHIFT) /* Memory to I/O, decrement MARI */
#  define DCNTL_DIM_IO2MI    (2 << DCNTL_DIM_SHIFT) /* I/O to memory, increment MARI */
#  define DCNTL_DIM_IO2MD    (3 << DCNTL_DIM_SHIFT) /* I/O to memory, decrement MARI */

/* System Control Registers *********************************************************/
/* Clock Multiplier Register (CMR: 0x1e) (Z8S180/L180-Class Processors Only) */

#ifdef HAVE_Z8S180 /* Z8S180/Z8L180 class processors */
#  define CMR_CMM            (0x80) /* Bit 7: X2 Clock Multiplier Mode */
#endif

/* CPU Control Register (CCR: 0x1f) (Z8S180/L180-Class Processors Only) */

#ifdef HAVE_Z8S180 /* Z8S180/Z8L180 class processors */
#  define CCR_XTAL_DIV       (0x80) /* Bit 7: Clock Divide */
#  define CCR_STBYIDLE       (0x48) /* Bits 3 & 6: STANDBY/IDLE mode */
#    define CCR_NOSTDBY      (0x00) /* No STANDBY */
#    define CCR_IDLE         (0x08) /* IDLE after SLEEP */
#    define CCR_STBY         (0x40) /* STANDBY after SLEEP */
#    define CCR_STBY64       (0x48) /* STANDBY after SLEEP 64 Cycle Exit */
#  define CCR_BREXT          (0x20) /* Bit 5: STANDBY/IDLE exit on BUSREQ */
#  define CCR_LNPHI          (0x10) /* Bit 4: 33% Drive on EXTPHI Clock */
#  define CCR_LNIO           (0x04) /* Bit 2: 33% Drive on certain external I/O */
#  define CCR_LNCPUCTLR      (0x02) /* Bit 1: 33% Drive on CPU control signals */
#  define LNADDATA           (0x01) /* Bit 0: 33% drive on A10-A0, D7-D0 */
#endif

/* Interrupt Vector Low Register (IL: 0x33) */

#define IL_SHIFT             (5)   /* Bits 5-7: 3-bits of vector interrupt table address */
#define IL_MASK              (7 << IL_SHIFT)

/* INT/TRAP Control Register (ITC: 0x34) */

#define ITC_TRAP             (0x80) /* Bit 7: Undefined opcode fetch */
#define ITC_UFO              (0x40) /* Bit 6: Undefined fetch object */
#define ITC_ITE_SHIFT        (0)    /* Bits 0-2: Interrupt enable bits */
#define ITC_ITE_MASK         (7 << ITC_ITE_SHIFT)
#  define ITC_ITE0           (1 << ITC_ITE_SHIFT)
#  define ITC_ITE1           (2 << ITC_ITE_SHIFT)
#  define ITC_ITE2           (4 << ITC_ITE_SHIFT)

/* Refresh Control Register (RCR: 0x36) */

#define RCR_REFE             (0x80) /* Bit 7: Refresh Enable */
#define RCR_REFW             (0x40) /* Bit 6: Refresh Wait */
#define RCR_CYC_SHIFT        (0)    /* Bits 0-1: Cycle Interval */
#define RCR_CYC_MASK         (3 << RCR_CYC_SHIFT)
#  define RCR_CYC0           (1 << RCR_CYC_SHIFT)
#  define RCR_CYC1           (2 << RCR_CYC_SHIFT)

/* MMU Common Base Register (CBR: 0x38) - 8-bit base address of Common Area 1 */
/* MMU Bank Base Register (BBR: 0x39) - 8-bit address of Bank area */

/* MMU Common/Bank Area Register (CBAR: 0x3a) */

#define CBAR_CA_SHIFT        (4)    /* Bits 4-7: Low address for Common Area 1 */
#define CBAR_CA_MASK         (15 << CBAR_CA_SHIFT)
#define CBAR_BA_SHIFT        (0)    /* Bits 0-3: Low address for Bank Area */
#define CBAR_BA_MASK         (15 << CBAR_BA_SHIFT)

/* Operation Mode Control Register (OMCR: 0x3e) */

#define OMCR_M1E             (0x80) /* Bit 7: M1 Enable */
#define OMCR_M1TE            (0x40) /* Bit 6: M1 Temporary Enable */
#define OMCR_IOC             (0x20) /* Bit 5: Controls the timing of the IORQ and RD signals */

/* I/O Control Register (ICR: 0x3f) */

#define ICR_IOA_SHIFT        (6)    /* Bits 6-7: Internal I/O address bits */
#define ICR_IOA_MASK         (3 << ICR_IOA_SHIFT)
#  define ICR_IOA6           (1 << ICR_IOA_SHIFT)
#  define ICR_IOA7           (2 << ICR_IOA_SHIFT)
#define ICR_IOSTP            (0x20) /* Bit 5: Enable I/O stop mode */

/* Registers unique to Z8x181 class CPUs ********************************************/

#ifdef HAVE_Z8X181
/* To be provided */

/* PIA Registers */

/* CTC Registers */

/* SCC Registers */

/* System Control Registers */

#endif

/* Registers unique to Z8x182 class CPUs ********************************************/

#ifdef HAVE_Z8X182
/* To be provided */

/* PIA Registers */

/* ESCC Registers */

/* System Control Registers */

/* 16550 MIMIC Registers */

#endif

#endif /* __ARCH_Z80_SRC_Z180_Z180_IOMAP_H */
