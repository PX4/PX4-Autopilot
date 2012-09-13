/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_usart.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_USART_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_USART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_USART_CR_OFFSET      0x0000 /* Control Register */
#define AVR32_USART_MR_OFFSET      0x0004 /* Mode Register */
#define AVR32_USART_IER_OFFSET     0x0008 /* Interrupt Enable Register */
#define AVR32_USART_IDR_OFFSET     0x000c /* Interrupt Disable Register */
#define AVR32_USART_IMR_OFFSET     0x0010 /* Interrupt Mask Register */
#define AVR32_USART_CSR_OFFSET     0x0014 /* Channel Status Register */
#define AVR32_USART_RHR_OFFSET     0x0018 /* Receiver Holding Register */
#define AVR32_USART_THR_OFFSET     0x001c /* Transmitter Holding Register */
#define AVR32_USART_BRGR_OFFSET    0x0020 /* Baud Rate Generator Register */
#define AVR32_USART_RTOR_OFFSET    0x0024 /* Receiver Time-out Register */
#define AVR32_USART_TTGR_OFFSET    0x0028 /* Transmitter Timeguard Register */
#define AVR32_USART_FIDI_OFFSET    0x0040 /* FI DI Ratio Register */
#define AVR32_USART_NER_OFFSET     0x0044 /* Number of Errors Register */
#define AVR32_USART_IFR_OFFSET     0x004c /* IrDA Filter Register */
#define AVR32_USART_MAN_OFFSET     0x0050 /* Manchester Encoder Decoder Register */
#define AVR32_USART_VERSION_OFFSET 0x00fc /* Version Register */

/* Register Addresses ***************************************************************/

#define AVR32_USART0_CR            (AVR32_USART0_BASE+AVR32_USART_CR_OFFSET)
#define AVR32_USART0_MR            (AVR32_USART0_BASE+AVR32_USART_MR_OFFSET)
#define AVR32_USART0_IER           (AVR32_USART0_BASE+AVR32_USART_IER_OFFSET)
#define AVR32_USART0_IDR           (AVR32_USART0_BASE+AVR32_USART_IDR_OFFSET)
#define AVR32_USART0_IMR           (AVR32_USART0_BASE+AVR32_USART_IMR_OFFSET)
#define AVR32_USART0_CSR           (AVR32_USART0_BASE+AVR32_USART_CSR_OFFSET)
#define AVR32_USART0_RHR           (AVR32_USART0_BASE+AVR32_USART_RHR_OFFSET)
#define AVR32_USART0_THR           (AVR32_USART0_BASE+AVR32_USART_THR_OFFSET)
#define AVR32_USART0_BRGR          (AVR32_USART0_BASE+AVR32_USART_BRGR_OFFSET)
#define AVR32_USART0_RTOR          (AVR32_USART0_BASE+AVR32_USART_RTOR_OFFSET)
#define AVR32_USART0_TTGR          (AVR32_USART0_BASE+AVR32_USART_TTGR_OFFSET)
#define AVR32_USART0_FIDI          (AVR32_USART0_BASE+AVR32_USART_FIDI_OFFSET)
#define AVR32_USART0_NER           (AVR32_USART0_BASE+AVR32_USART_NER_OFFSET)
#define AVR32_USART0_IFR           (AVR32_USART0_BASE+AVR32_USART_IFR_OFFSET)
#define AVR32_USART0_MAN           (AVR32_USART0_BASE+AVR32_USART_MAN_OFFSET)
#define AVR32_USART0_VERSION       (AVR32_USART0_BASE+AVR32_USART_VERSION_OFFSET)

#define AVR32_USART1_CR            (AVR32_USART1_BASE+AVR32_USART_CR_OFFSET)
#define AVR32_USART1_MR            (AVR32_USART1_BASE+AVR32_USART_MR_OFFSET)
#define AVR32_USART1_IER           (AVR32_USART1_BASE+AVR32_USART_IER_OFFSET)
#define AVR32_USART1_IDR           (AVR32_USART1_BASE+AVR32_USART_IDR_OFFSET)
#define AVR32_USART1_IMR           (AVR32_USART1_BASE+AVR32_USART_IMR_OFFSET)
#define AVR32_USART1_CSR           (AVR32_USART1_BASE+AVR32_USART_CSR_OFFSET)
#define AVR32_USART1_RHR           (AVR32_USART1_BASE+AVR32_USART_RHR_OFFSET)
#define AVR32_USART1_THR           (AVR32_USART1_BASE+AVR32_USART_THR_OFFSET)
#define AVR32_USART1_BRGR          (AVR32_USART1_BASE+AVR32_USART_BRGR_OFFSET)
#define AVR32_USART1_RTOR          (AVR32_USART1_BASE+AVR32_USART_RTOR_OFFSET)
#define AVR32_USART1_TTGR          (AVR32_USART1_BASE+AVR32_USART_TTGR_OFFSET)
#define AVR32_USART1_FIDI          (AVR32_USART1_BASE+AVR32_USART_FIDI_OFFSET)
#define AVR32_USART1_NER           (AVR32_USART1_BASE+AVR32_USART_NER_OFFSET)
#define AVR32_USART1_IFR           (AVR32_USART1_BASE+AVR32_USART_IFR_OFFSET)
#define AVR32_USART1_MAN           (AVR32_USART1_BASE+AVR32_USART_MAN_OFFSET)
#define AVR32_USART1_VERSION       (AVR32_USART1_BASE+AVR32_USART_VERSION_OFFSET)

#define AVR32_USART2_CR            (AVR32_USART2_BASE+AVR32_USART_CR_OFFSET)
#define AVR32_USART2_MR            (AVR32_USART2_BASE+AVR32_USART_MR_OFFSET)
#define AVR32_USART2_IER           (AVR32_USART2_BASE+AVR32_USART_IER_OFFSET)
#define AVR32_USART2_IDR           (AVR32_USART2_BASE+AVR32_USART_IDR_OFFSET)
#define AVR32_USART2_IMR           (AVR32_USART2_BASE+AVR32_USART_IMR_OFFSET)
#define AVR32_USART2_CSR           (AVR32_USART2_BASE+AVR32_USART_CSR_OFFSET)
#define AVR32_USART2_RHR           (AVR32_USART2_BASE+AVR32_USART_RHR_OFFSET)
#define AVR32_USART2_THR           (AVR32_USART2_BASE+AVR32_USART_THR_OFFSET)
#define AVR32_USART2_BRGR          (AVR32_USART2_BASE+AVR32_USART_BRGR_OFFSET)
#define AVR32_USART2_RTOR          (AVR32_USART2_BASE+AVR32_USART_RTOR_OFFSET)
#define AVR32_USART2_TTGR          (AVR32_USART2_BASE+AVR32_USART_TTGR_OFFSET)
#define AVR32_USART2_FIDI          (AVR32_USART2_BASE+AVR32_USART_FIDI_OFFSET)
#define AVR32_USART2_NER           (AVR32_USART2_BASE+AVR32_USART_NER_OFFSET)
#define AVR32_USART2_IFR           (AVR32_USART2_BASE+AVR32_USART_IFR_OFFSET)
#define AVR32_USART2_MAN           (AVR32_USART2_BASE+AVR32_USART_MAN_OFFSET)
#define AVR32_USART2_VERSION       (AVR32_USART2_BASE+AVR32_USART_VERSION_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* CR Register Bit-field Definitions */

#define USART_CR_RSTRX             (1 << 2)  /* Bit 2:  Reset Receiver */
#define USART_CR_RSTTX             (1 << 3)  /* Bit 3:  Reset Transmitter */
#define USART_CR_RXEN              (1 << 4)  /* Bit 4:  Receiver Enable */
#define USART_CR_RXDIS             (1 << 5)  /* Bit 5:  Receiver Disable */
#define USART_CR_TXEN              (1 << 6)  /* Bit 6:  Transmitter Enable */
#define USART_CR_TXDIS             (1 << 7)  /* Bit 7:  Transmitter Disable */
#define USART_CR_RSTSTA            (1 << 8)  /* Bit 8:  Reset Status Bits */
#define USART_CR_STTBRK            (1 << 9)  /* Bit 9:  Start Break */
#define USART_CR_STPBRK            (1 << 10) /* Bit 10: Stop Break */
#define USART_CR_STTTO             (1 << 11) /* Bit 11: Start Time-out */
#define USART_CR_SENDA             (1 << 12) /* Bit 12: Send Address */
#define USART_CR_RSTIT             (1 << 13) /* Bit 13: Reset Iterations */
#define USART_CR_RSTNACK           (1 << 14) /* Bit 14: Reset Non Acknowledge */
#define USART_CR_RETTO             (1 << 15) /* Bit 15: Rearm Time-out */
#define USART_CR_DTREN             (1 << 16) /* Bit 16: Data Terminal Ready Enable */
#define USART_CR_DTRDIS            (1 << 17) /* Bit 17: Data Terminal Ready Disable */
#define USART_CR_RTSEN             (1 << 18) /* Bit 18: Request to Send Enable */
#define USART_CR_FCS               (1 << 18) /* Bit 18: Force SPI Chip Select */
#define USART_CR_RTSDIS            (1 << 19) /* Bit 19: Request to Send Disable */
#define USART_CR_RCS               (1 << 19) /* Bit 19: Release SPI Chip Select */

/* MR Register Bit-field Definitions */

#define USART_MR_MODE_SHIFT         (0)
#define USART_MR_MODE_MASK          (15 << USART_MR_MODE_SHIFT)
#  define USART_MR_MODE_NORMAL      (0 << USART_MR_MODE_SHIFT)  /* Normal */
#  define USART_MR_MODE_RS485       (1 << USART_MR_MODE_SHIFT)  /* RS485 */
#  define USART_MR_MODE_HW          (2 << USART_MR_MODE_SHIFT)  /* Hardware Handshaking */
#  define USART_MR_MODE_MODEM       (3 << USART_MR_MODE_SHIFT)  /* Modem */
#  define USART_MR_MODE_T0          (4 << USART_MR_MODE_SHIFT)  /* IS07816 Protocol: T = 0 */
#  define USART_MR_MODE_T1          (6 << USART_MR_MODE_SHIFT)  /* IS07816 Protocol: T = 1 */
#  define USART_MR_MODE_IRDA        (8 << USART_MR_MODE_SHIFT)  /* IrDA */
#  define USART_MR_MODE_MASTER      (14 << USART_MR_MODE_SHIFT) /* SPI Master */
#  define USART_MR_MODE_SLAVE       (15 << USART_MR_MODE_SHIFT) /* SPI Slave */
#define USART_MR_USCLKS_SHIFT       (4)       /* Bits 4-5:  Clock Selection */
#define USART_MR_USCLKS_MASK        (3 << USART_MR_USCLKS_SHIFT)
#  define USART_MR_USCLKS_CLKUSART  (0 << USART_MR_USCLKS_SHIFT) /* CLK_USART */
#  define USART_MR_USCLKS_DIV       (1 << USART_MR_USCLKS_SHIFT) /* CLK_USART/DIV */
#  define USART_MR_USCLKS_CLK       (3 << USART_MR_USCLKS_SHIFT) /* CLK */
#define USART_MR_CHRL_SHIFT         (6)       /* Bit 6-7:  Character Length */
#define USART_MR_CHRL_MASK          (3 << USART_MR_CHRL_SHIFT)
#  define USART_MR_CHRL_BITS(n)     (((n) - 5) << USART_MR_CHRL_SHIFT)
#  define USART_MR_CHRL_5BITS       (0 << USART_MR_CHRL_SHIFT) /* 5 bits */
#  define USART_MR_CHRL_6BITS       (1 << USART_MR_CHRL_SHIFT) /* 6 bits */
#  define USART_MR_CHRL_7BITS       (2 << USART_MR_CHRL_SHIFT) /* 7 bits */
#  define USART_MR_CHRL_8BITS       (3 << USART_MR_CHRL_SHIFT) /* 8 bits */
#define USART_MR_SYNC               (1 << 8)  /* Bit 8:  Synchronous Mode Select */
#define USART_MR_CPHA               (1 << 8)  /* Bit 8:  SPI Clock Phase */
#define USART_MR_PAR_SHIFT          (9)       /* Bits 9-11:  Parity Type */
#define USART_MR_PAR_MASK           (7 << USART_MR_PAR_SHIFT)
#  define USART_MR_PAR_EVEN         (0 << USART_MR_PAR_SHIFT) /* Even parity */
#  define USART_MR_PAR_ODD          (1 << USART_MR_PAR_SHIFT) /* Odd parity */
#  define USART_MR_PAR_SPACE        (2 << USART_MR_PAR_SHIFT) /* Parity forced to 0 (Space) */
#  define USART_MR_PAR_MARK         (3 << USART_MR_PAR_SHIFT) /* Parity forced to 1 (Mark) */
#  define USART_MR_PAR_NONE         (4 << USART_MR_PAR_SHIFT) /* No parity */
#  define USART_MR_PAR_MULTIDROP    (6 << USART_MR_PAR_SHIFT) /* Multidrop mode */
#define USART_MR_NBSTOP_SHIFT       (12)      /* Bits 12-13:  Number of Stop Bits */
#define USART_MR_NBSTOP_MASK        (3 << USART_MR_NBSTOP_SHIFT)
#  define USART_MR_NBSTOP_1         (0 << USART_MR_NBSTOP_SHIFT) /* 1 stop bit */
#  define USART_MR_NBSTOP_1p5       (1 << USART_MR_NBSTOP_SHIFT) /* 1.5 stop bits */
#  define USART_MR_NBSTOP_2         (2 << USART_MR_NBSTOP_SHIFT) /* 2 stop bits */
#define USART_MR_CHMODE_SHIFT       (14)      /* Bits 14-15:  Channel Mode */
#define USART_MR_CHMODE_MASK        (3 << USART_MR_CHMODE_SHIFT)
#  define USART_MR_CHMODE_NORMAL    (0 << USART_MR_CHMODE_SHIFT) /* Normal Mode */
#  define USART_MR_CHMODE_AUTO      (1 << USART_MR_CHMODE_SHIFT) /* Automatic Echo */
#  define USART_MR_CHMODE_LLPBK     (2 << USART_MR_CHMODE_SHIFT) /* Local Loopback */
#  define USART_MR_CHMODE_RLPBK     (3 << USART_MR_CHMODE_SHIFT) /* Remote Loopback */
#define USART_MR_MSBF               (1 << 16) /* Bit 16: Bit Order */
#define USART_MR_CPOL               (1 << 16) /* Bit 16: SPI Clock Polarity */
#define USART_MR_MODE9              (1 << 17) /* Bit 17: 9-bit Character Length */
#define USART_MR_CLKO               (1 << 18) /* Bit 18: Clock Output Select */
#define USART_MR_OVER               (1 << 19) /* Bit 19: Oversampling Mode */
#define USART_MR_INACK              (1 << 20) /* Bit 20: Inhibit Non Acknowledge */
#define USART_MR_DSNACK             (1 << 21) /* Bit 21: Disable Successive NACK */
#define USART_MR_VAR_SYNC           (1 << 22) /* Bit 22: Variable Synchronization */
#define USART_MR_MAXITER_SHIFT      (24)      /* Bits 24-26: Maximum iterations */
#define USART_MR_MAXITER_MASK       (7 << USART_MR_MAXITER_SHIFT)
#define USART_MR_FILTER             (1 << 28) /* Bit 28: Infrared Receive Line Filter */
#define USART_MR_MAN                (1 << 29) /* Bit 29: Manchester Encoder/Decoder Enable */
#define USART_MR_MODSYNC            (1 << 30) /* Bit 30: Manchester Synchronization Mode */
#define USART_MR_ONEBIT             (1 << 31) /* Bit 31: Start Frame Delimiter Selector */

/* IER, IDR, and IMR (and CSR) Register Bit-field Definitions */

#define USART_INT_RXRDY             (1 << 0)  /* Bit 0:  */
#define USART_INT_TXRDY             (1 << 1)  /* Bit 1:  */
#define USART_INT_RXBRK             (1 << 2)  /* Bit 2:  */
#define USART_INT_OVRE              (1 << 5)  /* Bit 5:  */
#define USART_INT_FRAME             (1 << 6)  /* Bit 6:  */
#define USART_INT_PARE              (1 << 7)  /* Bit 7:  */
#define USART_INT_TIMEOUT           (1 << 8)  /* Bit 8:  */
#define USART_INT_TXEMPTY           (1 << 9)  /* Bit 9:  */
#define USART_INT_ITER              (1 << 10) /* Bit 10: */
#define USART_INT_UNRE              (1 << 10) /* Bit 10: */
#define USART_INT_RXBUFF            (1 << 12) /* Bit 12: */
#define USART_INT_NACK              (1 << 13) /* Bit 13: */
#define USART_INT_RIIC              (1 << 16) /* Bit 16: */
#define USART_INT_DSRIC             (1 << 17) /* Bit 17: */
#define USART_INT_DCDIC             (1 << 18) /* Bit 18: */
#define USART_INT_CTSIC             (1 << 19) /* Bit 19: */
#define USART_INT_MANE              (1 << 20) /* Bit 20: */
#define USART_INT_MANEA             (1 << 24) /* Bit 24: */
#define USART_INT_ALL               0x011f37e7

/* CSR Register Bit-field Definitions */

#define USART_CSR_RXRDY             (1 << 0)  /* Bit 0:  */
#define USART_CSR_TXRDY             (1 << 1)  /* Bit 1:  */
#define USART_CSR_RXBRK             (1 << 2)  /* Bit 2:  */
#define USART_CSR_OVRE              (1 << 5)  /* Bit 5:  */
#define USART_CSR_FRAME             (1 << 6)  /* Bit 6:  */
#define USART_CSR_PARE              (1 << 7)  /* Bit 7:  */
#define USART_CSR_TIMEOUT           (1 << 8)  /* Bit 8:  */
#define USART_CSR_TXEMPTY           (1 << 9)  /* Bit 9:  */
#define USART_CSR_ITER              (1 << 10) /* Bit 10: */
#define USART_CSR_UNRE              (1 << 10) /* Bit 10: */
#define USART_CSR_RXBUFF            (1 << 12) /* Bit 12: */
#define USART_CSR_NACK              (1 << 13) /* Bit 13: */
#define USART_CSR_RIIC              (1 << 16) /* Bit 16: */
#define USART_CSR_DSRIC             (1 << 17) /* Bit 17: */
#define USART_CSR_DCDIC             (1 << 18) /* Bit 18: */
#define USART_CSR_CTSIC             (1 << 19) /* Bit 19: */
#define USART_CSR_RI                (1 << 20) /* Bit 20: */
#define USART_CSR_DSR               (1 << 21) /* Bit 21: Image of DSR Input */
#define USART_CSR_DCD               (1 << 22) /* Bit 22: Image of DCD Input*/
#define USART_CSR_CTS               (1 << 23) /* Bit 23: Image of CTS Input */
#define USART_CSR_MANERR            (1 << 24) /* Bit 24: Manchester Error */

/* RHR Register Bit-field Definitions */

#define USART_RHR_RXCHR_SHIFT       (0)       /* Bits 0-8: Received Character */
#define USART_RHR_RXCHR_MASK        (0x1ff << USART_RHR_RXCHR_SHIFT)
#define USART_RHR_RXSYNH            (1 << 15) /* Bit 15: Received Sync */

/* THR Register Bit-field Definitions */

#define USART_THR_TXCHR_SHIFT       (0)       /* Bits 0-8: Character to be Transmitted */
#define USART_THR_TXCHR_MASK        (0x1ff << USART_RHR_RXCHR_SHIFT)
#define USART_THR_TXSYNH            (1 << 15) /* Bit 15: Sync Field to be transmitted */

/* BRGR Register Bit-field Definitions */

#define USART_BRGR_CD_SHIFT         (0)       /* Bits 0-15: Clock Divider */
#define USART_BRGR_CD_MASK          (0xffff << USART_BRGR_CD_SHIFT)
#define USART_BRGR_FP_SHIFT         (16)      /* Bits 16-18: Fractional Part */
#define USART_BRGR_FP_MASK          (7 << USART_BRGR_FP_SHIFT)

/* RTOR Register Bit-field Definitions */

#define USART_RTOR_SHIFT            (0)       /* Bits0-15: Time-out Value */
#define USART_RTOR_MASK             (0xffff << USART_RTOR_SHIFT)

/* TTGR Register Bit-field Definitions */

#define USART_TTGR_SHIFT            (0)       /* Bits 0-7: Timeguard Value */
#define USART_TTGR_MASK             (0xff << USART_TTGR_SHIFT)

/* FIDI Register Bit-field Definitions */

#define USART_FIDI_SHIFT            (0)       /* Bits 0-10: FI Over DI Ratio Value */
#define USART_FIDI_MASK             (0x7ff<< USART_FIDI_SHIFT)

/* NER Register Bit-field Definitions */

#define USART_NER_SHIFT             (0)       /* Bits 0-7: Number of Errors */
#define USART_NER_MASK              (0xff << USART_NER_SHIFT)

/* IFR Register Bit-field Definitions */

#define USART_IFR_SHIFT             (0)       /* Bits 0-7: IrDA Filter */
#define USART_IFR_MASK              (0xff << USART_IFR_SHIFT)

/* MAN Register Bit-field Definitions */

#define USART_MAN_TXPL_SHIFT        (0)       /* Bits 0-3: Transmitter Preamble Length */
#define USART_MAN_TXPL_MASK         (15 << USART_MAN_TXPL_SHIFT)
#define USART_MAN_TXPP_SHIFT        (8)       /* Bits 8-9: Transmitter Preamble Pattern */
#define USART_MAN_TXPP_MASK         (3 << USART_MAN_TXPP_SHIFT)
#  define USART_MAN_TXPP_ALLONE     (0 << USART_MAN_TXPP_SHIFT) /* ALL_ONE */
#  define USART_MAN_TXPP_ALLZERO    (1 << USART_MAN_TXPP_SHIFT) /* ALL_ZERO */
#  define USART_MAN_TXPP_ZER0ONE    (2 << USART_MAN_TXPP_SHIFT) /* ZERO_ONE */
#  define USART_MAN_TXPP_ONEZERO    (3 << USART_MAN_TXPP_SHIFT) /* ONE_ZERO */
#define USART_MAN_TXMPOL            (1 << 12) /* Bit 12: Transmitter Manchester Polarity */
#define USART_MAN_RXPL_SHIFT        (16)      /* Bits 16-19: Receiver Preamble Length */
#define USART_MAN_RXPL_MASK         (15 << USART_MAN_RXPL_SHIFT)
#define USART_MAN_RXPP_SHIFT        (24)      /* Bits 24-25: Receiver Preamble Pattern detected */
#define USART_MAN_RXPP_MASK         (3 << USART_MAN_RXPP_SHIFT)
#  define USART_MAN_RXPP_ALLONE     (0 << USART_MAN_TXPP_SHIFT) /* ALL_ONE */
#  define USART_MAN_RXPP_ALLZERO    (1 << USART_MAN_TXPP_SHIFT) /* ALL_ZERO */
#  define USART_MAN_RXPP_ZER0ONE    (2 << USART_MAN_TXPP_SHIFT) /* ZERO_ONE */
#  define USART_MAN_RXPP_ONEZERO    (3 << USART_MAN_TXPP_SHIFT) /* ONE_ZERO */
#define USART_MAN_RXMPOL            (1 << 28) /* Bit 28: Receiver Manchester Polarity */
#define USART_MAN_DRIFT             (1 << 30) /* Bit 30: Drift compensation */

/* VERSION Register Bit-field Definitions */

#define USART_VERSION_SHIFT         (0)       /* Bits 0-12: Version of the module */
#define USART_VERSION_MASK          (0xfff << USART_VERSION_SHIFT)
#define USART_VARIANT_SHIFT         (16)      /* Bits 16-19: (Reserved) */
#define USART_VARIANT_MASK          (15 << USART_VARIANT_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_USART_H */

