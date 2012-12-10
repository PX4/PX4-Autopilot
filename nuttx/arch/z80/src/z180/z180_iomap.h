/****************************************************************************
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
 ****************************************************************************/

#ifndef __ARCH_Z80_SRC_Z180_Z180_IOMAP_H
#define __ARCH_Z80_SRC_Z180_Z180_IOMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/z180/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 /* Z180 Register Bit addresses **********************************************/

#define Z180_ASCI0_CNTLA     0x00 /* ASCI Control Register A Ch 0 */
#define Z180_ASCI1_CNTLA     0x01 /* ASCI Control Register A Ch 1 */
#define Z180_ASCI0_CNTLB     0x02 /* ASCI Control Register B Ch 0 */
#define Z180_ASCI1_CNTLB     0x03 /* ASCI Control Register B Ch 1 */
#define Z180_ASCI0_STAT      0x04 /* ASCI Status Register Ch 0 */
#define Z180_ASCI1_STAT      0x05 /* ASCI Status Register Ch 1 */
#define Z180_ASCI0_TDR       0x06 /* ASCI Transmit Data Register Ch 0 */
#define Z180_ASCI1_TDR       0x07 /* ASCI Transmit Data Register Ch 1 */
#define Z180_ASCI0_RDR       0x08 /* ASCI Receive Data Register Ch 0 */
#define Z180_ASCI1_RDR       0x09 /* ASCI Receive Data Register Ch 1 */

#define Z180_CSIO_CNTR       0x0a /* CSI/O Control Register */
#define Z180_CSIO_TRD        0x0b /* Transmit/Receive Data Register */

#define Z180_TMR0_DRL        0x0c /* Timer Data Register Ch 0 L */
#define Z180_TMR0_DRH        0x0d /* Data Register Ch 0 H */
#define Z180_TMR0_RLDRL      0x0e /* Reload Register Ch 0 L */
#define Z180_TMR0_RLDRH      0x0f /* Reload Register Ch 0 H */
#define Z180_TMR_TCR         0x10 /* Timer Control Register */

#ifdef HAVE_Z8S180 /* Z8S180/Z8L180 class processors */
#  define Z180_ASCI0_ASEXT   0x12 /* ASCI Extension Control Register */
#  define Z180_ASCI1_ASEXT   0x13 /* ASCI Extension Control Register */
#endif

#define Z180_TMR1_DRL        0x14 /* Data Register Ch 1 L */
#define Z180_TMR1_DRH        0x15 /* Data Register Ch 1 H */
#define Z180_TMR1_RLDRL      0x16 /* Reload Register Ch 1 L */
#define Z180_TMR1_RLDRH      0x17 /* Reload Register Ch 1 H */

#define Z180_FRC             0x18 /* Free Running Counter */

#ifdef HAVE_Z8S180 /* Z8S180/Z8L180 class processors */
#  define Z180_ASCI0_ASTCL   0x1a /* ASCI Time Constant Low */
#  define Z180_ASCI0_ASTCH   0x1b /* ASCI Time Constant High */
#  define Z180_ASCI1_ASTCL   0x1c /* ASCI Time Constant Low */
#  define Z180_ASCI1_ASTCH   0x1d /* ASCI Time Constant High */

#  define Z180_CMR           0x1e /* Clock Multiplier Register */
#  define Z180_CCR           0x1f /* CPU Control Register */
#endif

#define Z180_DMA_SAR0L       0x20 /* DMA Source Address Register Ch 0L */
#define Z180_DMA_SAR0H       0x21 /* DMA Source Address Register Ch 0H */
#define Z180_DMA_SAR0B       0x22 /* DMA Source Address Register Ch 0B */
#define Z180_DMA_DAR0L       0x23 /* DMA Destination Address Register Ch 0L */
#define Z180_DMA_DAR0H       0x24 /* DMA Destination Address Register Ch 0H */
#define Z180_DMA_DAR0B       0x25 /* DMA Destination Address Register Ch 0B */
#define Z180_DMA_BCR0L       0x26 /* DMA Byte Count Register Ch 0L */
#define Z180_DMA_BCR0H       0x27 /* DMA Byte Count Register Ch 0H */
#define Z180_DMA_MAR1L       0x28 /* DMA Memory Address Register Ch 1L */
#define Z180_DMA_MAR1H       0x29 /* DMA Memory Address Register Ch 1H */
#define Z180_DMA_MAR1B       0x2a /* DMA Memory Address Register Ch 1B */
#define Z180_DMA_IAR1L       0x2b /* DMA I/0 Address Register Ch 1L */
#define Z180_DMA_IAR1H       0x2c /* DMA I/0 Address Register Ch 1H */
#ifdef HAVE_Z8S180 /* Z8S180/Z8L180 class processors */
#  define Z180_DMA_IAR1B     0x2d /* DMA I/O Address Register Ch 1B */
#endif
#define Z180_DMA_BCR1L       0x2e /* DMA Byte Count Register Ch 1L */
#define Z180_DMA_BCR1H       0x2f /* DMA Byte Count Register Ch 1H */
#define Z180_DMA_DSTAT       0x30 /* DMA Status Register */
#define Z180_DMA_DMODE       0x31 /* DMA Mode Register */
#define Z180_DMA_DCNTL       0x32 /* DMA/WAIT Control Register */

#define Z180_INT_IL          0x33 /* IL Register (Interrupt Vector Low Register) */
#define Z180_INT_ITC         0x34 /* INT/TRAP Control Register */

#define Z180_RCR             0x36 /*  Refresh Control Register */

#define Z180_MMU_CBR         0x38 /* MMU Common Base Register */
#define Z180_MMU_BBR         0x39 /* MMU Bank Base Register */
#define Z180_MMU_CBAR        0x3a /* MMU Common/Bank Area Register */

#define Z180_OMCR            0x3e /* Operation Mode Control Register */
#define Z180_ICR             0x3f /* I/O Control Register */

/* Z180 Register Bit definitions ********************************************/

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

/* Timer Data Register 1L (TMDR1L: 0x14) -- 8-bit data */
/* Timer Data Register 1H (TMDR1H: 0x15) -- 8-bit data */
/* Timer Reload Register Channel 1L (RLDR1L: 0x16) -- 8-bit data */
/* Timer Reload Register Channel 1H (RLDR1H: 0x17) -- 8-bit data */
/* Free Running counter (FRC: 0x18) -- 8-bit data */

/* ASCI0 Time Constant Low Register (I/O Address: 0x1a) (Z8S180/L180-Class Processors Only) -- 8-bit data */
/* ASCI0 Time Constant High Register (I/O Address: 0x1b) (Z8S180/L180-Class Processors Only) -- 8-bit data */
/* ASCI1 Time Constant Low Register (I/O Address: 0x1c) (Z8S180/L180-Class Processors Only) -- 8-bit data */
/* ASCI1 Time Constant High Register (I/O Address: 0x1d) (Z8S180/L180-Class Processors Only) -- 8-bit data */

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
#  define LNADDATA           (0x01) /* Bit 0: 33% drive on A10–A0, D7–D0 */
#endif

/* DMA Destination Address Register Channel 0 (DAR0 I/O Address 0x23 to 0x25) -- 8-bit data */
/*DMA Byte Count Register Channel 0 (BCR0 I/O Address = 0x26 to 0x27) -- 8-bit data */
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

#endif /* __ARCH_Z80_SRC_Z180_Z180_IOMAP_H */
