/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx-spi.h
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
 ****************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_SPI_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "pic32mx-memorymap.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Register Offsets *********************************************************/

#define PIC32MX_SPI_CON_OFFSET     0x0000 /* SPI control register */
#define PIC32MX_SPI_CONCLR_OFFSET  0x0004 /* SPI control clear register */
#define PIC32MX_SPI_CONSET_OFFSET  0x0008 /* SPI control set register */
#define PIC32MX_SPI_CONINV_OFFSET  0x000c /* SPI control invert register */
#define PIC32MX_SPI_STAT_OFFSET    0x0010 /* SPI status register */
#define PIC32MX_SPI_STATCLR_OFFSET 0x0014 /* SPI status clear register */
#define PIC32MX_SPI_BUF_OFFSET     0x0020 /* SPI buffer register */
#define PIC32MX_SPI_BRG_OFFSET     0x0030 /* SPI baud rate register */
#define PIC32MX_SPI_BRGCLR_OFFSET  0x0034 /* SPI baud rate clear register */
#define PIC32MX_SPI_BRGSET_OFFSET  0x0038 /* SPI baud rate set register */
#define PIC32MX_SPI_BRGINV_OFFSET  0x003c /* SPI baud rate invert register */

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#define PIC32MX_SPI_CON2_OFFSET     0x0020 /* SPI control register 2*/
#endif

/* Register Addresses *******************************************************/

#ifdef PIC32MX_SPI1_K1BASE
#  define PIC32MX_SPI1_CON         (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_CON_OFFSET)
#  define PIC32MX_SPI1_CONCLR      (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_CONCLR_OFFSET)
#  define PIC32MX_SPI1_CONSET      (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_CONSET_OFFSET)
#  define PIC32MX_SPI1_CONINV      (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_CONINV_OFFSET)
#  define PIC32MX_SPI1_STAT        (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_STAT_OFFSET)
#  define PIC32MX_SPI1_STATCLR     (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_STATCLR_OFFSET)
#  define PIC32MX_SPI1_BUF         (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_BUF_OFFSET)
#  define PIC32MX_SPI1_BRG         (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_BRG_OFFSET)
#  define PIC32MX_SPI1_BRGCLR      (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_BRGCLR_OFFSET)
#  define PIC32MX_SPI1_BRGSET      (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_BRGSET_OFFSET)
#  define PIC32MX_SPI1_BRGINV      (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_BRGINV_OFFSET)
#  if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#    define PIC32MX_SPI1_CON2      (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_CON2_OFFSET)
#  endif
#endif

#ifdef PIC32MX_SPI2_K1BASE
#  define PIC32MX_SPI2_CON         (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_CON_OFFSET)
#  define PIC32MX_SPI2_CONCLR      (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_CONCLR_OFFSET)
#  define PIC32MX_SPI2_CONSET      (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_CONSET_OFFSET)
#  define PIC32MX_SPI2_CONINV      (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_CONINV_OFFSET)
#  define PIC32MX_SPI2_STAT        (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_STAT_OFFSET)
#  define PIC32MX_SPI2_STATCLR     (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_STATCLR_OFFSET)
#  define PIC32MX_SPI2_BUF         (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_BUF_OFFSET)
#  define PIC32MX_SPI2_BRG         (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_BRG_OFFSET)
#  define PIC32MX_SPI2_BRGCLR      (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_BRGCLR_OFFSET)
#  define PIC32MX_SPI2_BRGSET      (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_BRGSET_OFFSET)
#  define PIC32MX_SPI2_BRGINV      (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_BRGINV_OFFSET)
#  if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#    define PIC32MX_SPI2_CON2      (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_CON2_OFFSET)
#  endif
#endif

#ifdef PIC32MX_SPI3_K1BASE
#  define PIC32MX_SPI3_CON         (PIC32MX_SPI3_K1BASE+PIC32MX_SPI_CON_OFFSET)
#  define PIC32MX_SPI3_CONCLR      (PIC32MX_SPI3_K1BASE+PIC32MX_SPI_CONCLR_OFFSET)
#  define PIC32MX_SPI3_CONSET      (PIC32MX_SPI3_K1BASE+PIC32MX_SPI_CONSET_OFFSET)
#  define PIC32MX_SPI3_CONINV      (PIC32MX_SPI3_K1BASE+PIC32MX_SPI_CONINV_OFFSET)
#  define PIC32MX_SPI3_STAT        (PIC32MX_SPI3_K1BASE+PIC32MX_SPI_STAT_OFFSET)
#  define PIC32MX_SPI3_STATCLR     (PIC32MX_SPI3_K1BASE+PIC32MX_SPI_STATCLR_OFFSET)
#  define PIC32MX_SPI3_BUF         (PIC32MX_SPI3_K1BASE+PIC32MX_SPI_BUF_OFFSET)
#  define PIC32MX_SPI3_BRG         (PIC32MX_SPI3_K1BASE+PIC32MX_SPI_BRG_OFFSET)
#  define PIC32MX_SPI3_BRGCLR      (PIC32MX_SPI3_K1BASE+PIC32MX_SPI_BRGCLR_OFFSET)
#  define PIC32MX_SPI3_BRGSET      (PIC32MX_SPI3_K1BASE+PIC32MX_SPI_BRGSET_OFFSET)
#  define PIC32MX_SPI3_BRGINV      (PIC32MX_SPI3_K1BASE+PIC32MX_SPI_BRGINV_OFFSET)
#endif

#ifdef PIC32MX_SPI4_K1BASE
#  define PIC32MX_SPI4_CON         (PIC32MX_SPI4_K1BASE+PIC32MX_SPI_CON_OFFSET)
#  define PIC32MX_SPI4_CONCLR      (PIC32MX_SPI4_K1BASE+PIC32MX_SPI_CONCLR_OFFSET)
#  define PIC32MX_SPI4_CONSET      (PIC32MX_SPI4_K1BASE+PIC32MX_SPI_CONSET_OFFSET)
#  define PIC32MX_SPI4_CONINV      (PIC32MX_SPI4_K1BASE+PIC32MX_SPI_CONINV_OFFSET)
#  define PIC32MX_SPI4_STAT        (PIC32MX_SPI4_K1BASE+PIC32MX_SPI_STAT_OFFSET)
#  define PIC32MX_SPI4_STATCLR     (PIC32MX_SPI4_K1BASE+PIC32MX_SPI_STATCLR_OFFSET)
#  define PIC32MX_SPI4_BUF         (PIC32MX_SPI4_K1BASE+PIC32MX_SPI_BUF_OFFSET)
#  define PIC32MX_SPI4_BRG         (PIC32MX_SPI4_K1BASE+PIC32MX_SPI_BRG_OFFSET)
#  define PIC32MX_SPI4_BRGCLR      (PIC32MX_SPI4_K1BASE+PIC32MX_SPI_BRGCLR_OFFSET)
#  define PIC32MX_SPI4_BRGSET      (PIC32MX_SPI4_K1BASE+PIC32MX_SPI_BRGSET_OFFSET)
#  define PIC32MX_SPI4_BRGINV      (PIC32MX_SPI4_K1BASE+PIC32MX_SPI_BRGINV_OFFSET)
#endif

/* Register Bit-Field Definitions *******************************************/

/* SPI control register */

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2) || defined(CHIP_PIC32MX5) || \
    defined(CHIP_PIC32MX6) || defined(CHIP_PIC32MX7)
#  define SPI_CON_RTXISEL_SHIFT    (0)       /* Bits 0-1: SPI Receive Buffer Full Interrupt Mode */
#  define SPI_CON_RTXISEL_MASK     (3 << SPI_CON_RTXISEL_SHIFT)
#    define SPI_CON_RTXISEL_EMPTY  (0 << SPI_CON_RTXISEL_SHIFT) /* Buffer empty */
#    define SPI_CON_RTXISEL_NEMPTY (1 << SPI_CON_RTXISEL_SHIFT) /* Buffer not empty */
#    define SPI_CON_RTXISEL_HALF   (2 << SPI_CON_RTXISEL_SHIFT) /* Buffer half full or more */
#    define SPI_CON_RTXISEL_FULL   (3 << SPI_CON_RTXISEL_SHIFT) /* Buffer full */
#  define SPI_CON_STXISEL_SHIFT    (2)       /* Bits 2-3: SPI Transmit Buffer Empty Interrupt Mode */
#  define SPI_CON_STXISEL_MASK     (3 << SPI_CON_STXISEL_SHIFT)
#    define SPI_CON_STXISEL_DONE   (0 << SPI_CON_STXISEL_SHIFT) /* Buffer empty (and data shifted out) */
#    define SPI_CON_STXISEL_EMPTY  (1 << SPI_CON_STXISEL_SHIFT) /* Buffer empty */
#    define SPI_CON_STXISEL_HALF   (2 << SPI_CON_STXISEL_SHIFT) /* Buffer half empty or more */
#    define SPI_CON_STXISEL_NFULL  (3 << SPI_CON_STXISEL_SHIFT) /* Buffer not full */
#endif

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define SPI_CON_DISSDI           (1 << 4)  /* Bit 4: Disable SDI */
#else
                                             /* Bit 4:  Reserved */
#endif

#define SPI_CON_MSTEN              (1 << 5)  /* Bits 5: Master mode enable */
#define SPI_CON_CKP                (1 << 6)  /* Bits 6: Clock polarity select */
#define SPI_CON_SSEN               (1 << 7)  /* Bits 7: Slave select enable (slave mode) */
#define SPI_CON_CKE                (1 << 8)  /* Bits 8: SPI clock edge select */
#define SPI_CON_SMP                (1 << 9)  /* Bits 9: SPI data input sample phase */
#define SPI_CON_MODE_SHIFT         (10)      /* Bits 10-11: 32/16-Bit Communication Select */
#define SPI_CON_MODE_MASK          (3 << SPI_CON_MODE_SHIFT)
#  define SPI_CON_MODE_8BIT        (0 << SPI_CON_MODE_SHIFT) /* 8-bit data width */
#  define SPI_CON_MODE_16BIT       (1 << SPI_CON_MODE_SHIFT) /* 16-bit data width */
#  define SPI_CON_MODE_32BIT       (2 << SPI_CON_MODE_SHIFT) /* 32-bit data width */
#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
                                                             /* With AUDEN=1: */
#  define SPI_CON_MODE_243232      (0 << SPI_CON_MODE_SHIFT) /* 24-bit data, 32-bit FIFO, 32-bit channel */
#  define SPI_CON_MODE_323232      (0 << SPI_CON_MODE_SHIFT) /* 32-bit data, 32-bit FIFO, 32-bit channel */
#  define SPI_CON_MODE_161632      (0 << SPI_CON_MODE_SHIFT) /* 16-bit data, 16-bit FIFO, 32-bit channel */
#  define SPI_CON_MODE_161616      (0 << SPI_CON_MODE_SHIFT) /* 16-bit data, 16-bit FIFO, 16-bit channel */
#endif
#define SPI_CON_DISSDO             (1 << 12) /* Bits 12: Disable SDOx pin */
#define SPI_CON_SIDL               (1 << 13) /* Bits 13: Stop in idle mode */
#if !defined(CHIP_PIC32MX1) && !defined(CHIP_PIC32MX2)
#  define SPI_CON_FRZ              (1 << 14) /* Bits 14: Freeze in debug exception */
#endif
#define SPI_CON_ON                 (1 << 15) /* Bits 15: SPI peripheral on */
#define SPI_CON_ENHBUF             (1 << 16) /* Bits 16: Enhanced buffer enable */
#define SPI_CON_SPIFE              (1 << 17) /* Bits 17: Frame sync pulse edge select */
#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
                                             /* Bits 18-22: Reserved */
#  define SPI_CON_MCLKSEL          (1 << 18) /* Master clock enable */
#else
                                             /* Bits 18-23: Reserved */
#endif
#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2) || defined(CHIP_PIC32MX5) || \
    defined(CHIP_PIC32MX6) || defined(CHIP_PIC32MX7)
#  define SPI_CON_FRMCNT_SHIFT     (24)      /* Bits 24-26: Frame Sync Pulse Counter bits */
#  define SPI_CON_FRMCNT_MASK      (7 << SPI_CON_FRMCNT_SHIFT)
#    define SPI_CON_FRMCNT_CHAR1   (0 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse each char */
#    define SPI_CON_FRMCNT_CHAR2   (1 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 2 chars */
#    define SPI_CON_FRMCNT_CHAR4   (2 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 4 chars */
#    define SPI_CON_FRMCNT_CHAR8   (3 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 8 chars */
#    define SPI_CON_FRMCNT_CHAR16  (4 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 16 chars */
#    define SPI_CON_FRMCNT_CHAR32  (5 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 32 chars */
#  define SPI_CON_FRMSYPW          (1 << 27) /* Bits 27: Frame sync pulse width */
#  define SPI_CON_MSSEN            (1 << 28) /* Bits 28: Master mode slave select enable */
#endif
#define SPI_CON_FRMPOL             (1 << 29) /* Bits 29: Frame sync polarity */
#define SPI_CON_FRMSYNC            (1 << 30) /* Bits 30: Frame sync pulse direction control on SSx pin */
#define SPI_CON_FRMEN              (1 << 31) /* Bits 31: Framed SPI support */

/* SPI control register 2 */

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)

#  define SPI2_CON2_AUDMOD_SHIFT   (0)       /* Bits 0-1: Audio Protocol Mode */
#  define SPI2_CON2_AUDMOD_MASK    (3 << SPI2_CON2_AUDMOD_SHIFT)
#    define SPI2_CON2_AUDMOD_I2S   (0 << SPI2_CON2_AUDMOD_SHIFT) /* I2S mode */
#    define SPI2_CON2_AUDMOD_LJ    (1 << SPI2_CON2_AUDMOD_SHIFT) /* Left Justified mode */
#    define SPI2_CON2_AUDMOD_RJ    (2 << SPI2_CON2_AUDMOD_SHIFT) /* Right Justified mode */
#    define SPI2_CON2_AUDMOD_PCM   (3 << SPI2_CON2_AUDMOD_SHIFT) /* PCM/DSP mode */
                                             /* Bit 2: Reserved */
#  define SPI2_CON2_AUDMONO        (1 << 6)  /* Bit 3:  Transmit Audio Data Format */
                                             /* Bits 5-6: Reserved */
#  define SPI2_CON2_AUDEN          (1 << 7)  /* Bit 7:  Enable Audio CODEC Support */
#  define SPI2_CON2_IGNTUR         (1 << 8)  /* Bit 8:  Ignore Transmit Underrun bit */
#  define SPI2_CON2_IGNROV         (1 << 9)  /* Bit 9:  Ignore Receive Overflow */
#  define SPI2_CON2_SPITUREN       (1 << 10) /* Bit 10: Enable Interrupt Events via SPITUR */
#  define SPI2_CON2_SPIROVEN       (1 << 11) /* Bit 11: Enable Interrupt Events via SPIROV */
#  define SPI2_CON2_FRMERREN       (1 << 12) /* Bit 12: Enable Interrupt Events via FRMERR */
                                             /* Bits 13-14: Reserved */
#  define SPI2_CON2_SPISGNEXT      (1 << 15) /* Bit 15 : Sign Extend Read Data from the RX FIFO */
                                             /* Bits 16-31: Reserved */
#endif

/* SPI status register */

#if defined(CHIP_PIC32MX3) || defined(CHIP_PIC32MX4)
#  define SPI_STAT_SPIRBF          (1 << 0)  /* Bit 0: SPI receive buffer full status */
#  define SPI_STAT_SPITBE          (1 << 3)  /* Bit 3: SPI transmit buffer empty status */
#  define SPI_STAT_SPIROV          (1 << 6)  /* Bit 6: Receive overflow flag */
#  define SPI_STAT_SPIBUSY         (1 << 11) /* Bit 11: SPI activity status */
#elif defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2) || defined(CHIP_PIC32MX5) || \
      defined(CHIP_PIC32MX6) || defined(CHIP_PIC32MX7)
#  define SPI_STAT_SPIRBF          (1 << 0)  /* Bit 0: SPI receive buffer full status */
#  define SPI_STAT_SPITBF          (1 << 1)  /* Bit 1: SPI transmit buffer full status */
#  define SPI_STAT_SPITBE          (1 << 3)  /* Bit 3: SPI transmit buffer empty status */
#  define SPI_STAT_SPIRBE          (1 << 5)  /* Bit 5: RX FIFO Empty */
#  define SPI_STAT_SPIROV          (1 << 6)  /* Bit 6: Receive overflow flag */
#  define SPI_STAT_SRMT            (1 << 7)  /* Bit 6: Shift Register Empty */
#  define SPI_STAT_SPITUR          (1 << 8)  /* Bit 8: Transmit under run */
#  define SPI_STAT_SPIBUSY         (1 << 11) /* Bit 11: SPI activity status */
#  if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#    define SPI_STAT_FRMERR        (1 << 12) /* Bit 12: SPI Frame Error status */
#  endif
#  define SPI_STAT_TXBUFELM_SHIFT  (16)      /* Bits 16-20: Transmit Buffer Element Count bits */
#  define SPI_STAT_TXBUFELM_MASK   (31 << SPI_STAT_TXBUFELM_SHIFT)
#  define SPI_STAT_RXBUFELM_SHIFT  (24)      /* Bits 24-28: Receive Buffer Element Count bits */
#  define SPI_STAT_RXBUFELM_MASK   (31 << SPI_STAT_RXBUFELM_SHIFT)
#endif

/* SPI buffer register (32-bits wide) */

/* SPI baud rate register */

#define SPI_BRG_MASK               0x1ff

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_SPI_H */
