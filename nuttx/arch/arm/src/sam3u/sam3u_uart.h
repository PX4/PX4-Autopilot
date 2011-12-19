/************************************************************************************************
 * arch/arm/src/sam3u/sam3u_uart.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_UART_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_UART_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* UART register offsets ************************************************************************/

#define SAM3U_UART_CR_OFFSET           0x0000 /* Control Register (Common) */
#define SAM3U_UART_MR_OFFSET           0x0004 /* Mode Register (Common) */
#define SAM3U_UART_IER_OFFSET          0x0008 /* Interrupt Enable Register (Common) */
#define SAM3U_UART_IDR_OFFSET          0x000c /* Interrupt Disable Register (Common) */
#define SAM3U_UART_IMR_OFFSET          0x0010 /* Interrupt Mask Register (Common) */
#define SAM3U_UART_SR_OFFSET           0x0014 /* Status Register (Common) */
#define SAM3U_UART_RHR_OFFSET          0x0018 /* Receive Holding Register (Common) */
#define SAM3U_UART_THR_OFFSET          0x001c /* Transmit Holding Register (Common) */
#define SAM3U_UART_BRGR_OFFSET         0x0020 /* Baud Rate Generator Register (Common) */
                                              /* 0x0024-0x003c: Reserved (UART) */
#define SAM3U_USART_RTOR_OFFSET        0x0024 /* Receiver Time-out Register (USART only) */
#define SAM3U_USART_TTGR_OFFSET        0x0028 /* Transmitter Timeguard Register (USART only) */
                                              /* 0x002c-0x003c: Reserved (UART) */       
#define SAM3U_USART_FIDI_OFFSET        0x0040 /* FI DI Ratio Register (USART only) */
#define SAM3U_USART_NER_OFFSET         0x0044 /* Number of Errors Register ((USART only) */
                                              /* 0x0048: Reserved (USART) */
#define SAM3U_USART_IF_OFFSET          0x004c /* IrDA Filter Register (USART only) */
#define SAM3U_USART_MAN_OFFSET         0x0050 /* Manchester Encoder Decoder Register (USART only) */
#define SAM3U_USART_WPMR_OFFSET        0x00e4 /* Write Protect Mode Register (USART only) */
#define SAM3U_USART_WPSR_OFFSET        0x00e8 /* Write Protect Status Register (USART only) */
                                              /* 0x005c-0xf008: Reserved (USART) */
#define SAM3U_USART_VERSION_OFFSET     0x00fc /* Version Register (USART only) */
                                              /* 0x0100-0x0124: PDC Area (Common) */

/* UART register adresses ***********************************************************************/

#define SAM3U_UART_CR                  (SAM3U_UART_BASE+SAM3U_UART_CR_OFFSET)
#define SAM3U_UART_MR                  (SAM3U_UART_BASE+SAM3U_UART_MR_OFFSET)
#define SAM3U_UART_IER                 (SAM3U_UART_BASE+SAM3U_UART_IER_OFFSET)
#define SAM3U_UART_IDR                 (SAM3U_UART_BASE+SAM3U_UART_IDR_OFFSET)
#define SAM3U_UART_IMR                 (SAM3U_UART_BASE+SAM3U_UART_IMR_OFFSET)
#define SAM3U_UART_SR                  (SAM3U_UART_BASE+SAM3U_UART_SR_OFFSET)
#define SAM3U_UART_RHR                 (SAM3U_UART_BASE+SAM3U_UART_RHR_OFFSET)
#define SAM3U_UART_THR                 (SAM3U_UART_BASE+SAM3U_UART_THR_OFFSET)
#define SAM3U_UART_BRGR                (SAM3U_UART_BASE+SAM3U_UART_BRGR_OFFSET)

#define SAM3U_USART_CR(n)              (SAM3U_USARTN_BASE(n)+SAM3U_UART_CR_OFFSET)
#define SAM3U_USART_MR(n)              (SAM3U_USARTN_BASE(n)+SAM3U_UART_MR_OFFSET)
#define SAM3U_USART_IER(n)             (SAM3U_USARTN_BASE(n)+SAM3U_UART_IER_OFFSET)
#define SAM3U_USART_IDR(n)             (SAM3U_USARTN_BASE(n)+SAM3U_UART_IDR_OFFSET)
#define SAM3U_USART_IMR(n)             (SAM3U_USARTN_BASE(n)+SAM3U_UART_IMR_OFFSET)
#define SAM3U_USART_SR(n)              (SAM3U_USARTN_BASE(n)+SAM3U_UART_SR_OFFSET)
#define SAM3U_USART_RHR(n)             (SAM3U_USARTN_BASE(n)+SAM3U_UART_RHR_OFFSET)
#define SAM3U_USART_THR(n)             (SAM3U_USARTN_BASE(n)+SAM3U_UART_THR_OFFSET)
#define SAM3U_USART_BRGR(n)            (SAM3U_USARTN_BASE(n)+SAM3U_UART_BRGR_OFFSET)
#define SAM3U_USART_RTOR(n)            (SAM3U_USARTN_BASE(n)+SAM3U_USART_RTOR_OFFSET)
#define SAM3U_USART_TTGR(n)            (SAM3U_USARTN_BASE(n)+SAM3U_USART_TTGR_OFFSET)
#define SAM3U_USART_FIDI(n)            (SAM3U_USARTN_BASE(n)+SAM3U_USART_FIDI_OFFSET)
#define SAM3U_USART_NER(n)             (SAM3U_USARTN_BASE(n)+SAM3U_USART_NER_OFFSET)
#define SAM3U_USART_IF(n)              (SAM3U_USARTN_BASE(n)+SAM3U_USART_IF_OFFSET)
#define SAM3U_USART_MAN(n)             (SAM3U_USARTN_BASE(n)+SAM3U_USART_MAN_OFFSET)
#define SAM3U_USART_WPMR(n)            (SAM3U_USARTN_BASE(n)+SAM3U_USART_WPMR_OFFSET)
#define SAM3U_USART_WPSR(n)            (SAM3U_USARTN_BASE(n)+SAM3U_USART_WPSR_OFFSET)
#define SAM3U_USART_VERSION(n)         (SAM3U_USARTN_BASE(n)+SAM3U_USART_VERSION_OFFSET)

#define SAM3U_USART0_CR                (SAM3U_USART0_BASE+SAM3U_UART_CR_OFFSET)
#define SAM3U_USART0_MR_               (SAM3U_USART0_BASE+SAM3U_UART_MR_OFFSET)
#define SAM3U_USART0_IER               (SAM3U_USART0_BASE+SAM3U_UART_IER_OFFSET)
#define SAM3U_USART0_IDR               (SAM3U_USART0_BASE+SAM3U_UART_IDR_OFFSET)
#define SAM3U_USART0_IMR               (SAM3U_USART0_BASE+SAM3U_UART_IMR_OFFSET)
#define SAM3U_USART0_SR                (SAM3U_USART0_BASE+SAM3U_UART_SR_OFFSET)
#define SAM3U_USART0_RHR               (SAM3U_USART0_BASE+SAM3U_UART_RHR_OFFSET)
#define SAM3U_USART0_THR               (SAM3U_USART0_BASE+SAM3U_UART_THR_OFFSET)
#define SAM3U_USART0_BRGR              (SAM3U_USART0_BASE+SAM3U_UART_BRGR_OFFSET)
#define SAM3U_USART0_RTOR              (SAM3U_USART0_BASE+SAM3U_USART_RTOR_OFFSET)
#define SAM3U_USART0_TTGR              (SAM3U_USART0_BASE+SAM3U_USART_TTGR_OFFSET)
#define SAM3U_USART0_FIDI              (SAM3U_USART0_BASE+SAM3U_USART_FIDI_OFFSET)
#define SAM3U_USART0_NER               (SAM3U_USART0_BASE+SAM3U_USART_NER_OFFSET)
#define SAM3U_USART0_IF                (SAM3U_USART0_BASE+SAM3U_USART_IF_OFFSET)
#define SAM3U_USART0_MAN               (SAM3U_USART0_BASE+SAM3U_USART_MAN_OFFSET)
#define SAM3U_USART0_WPMR              (SAM3U_USART0_BASE+SAM3U_USART_WPMR_OFFSET)
#define SAM3U_USART0_WPSR              (SAM3U_USART0_BASE+SAM3U_USART_WPSR_OFFSET)
#define SAM3U_USART0_VERSION           (SAM3U_USART0_BASE+SAM3U_USART_VERSION_OFFSET)

#define SAM3U_USART1_CR                (SAM3U_USART1_BASE+SAM3U_UART_CR_OFFSET)
#define SAM3U_USART1_MR_               (SAM3U_USART1_BASE+SAM3U_UART_MR_OFFSET)
#define SAM3U_USART1_IER               (SAM3U_USART1_BASE+SAM3U_UART_IER_OFFSET)
#define SAM3U_USART1_IDR               (SAM3U_USART1_BASE+SAM3U_UART_IDR_OFFSET)
#define SAM3U_USART1_IMR               (SAM3U_USART1_BASE+SAM3U_UART_IMR_OFFSET)
#define SAM3U_USART1_SR                (SAM3U_USART1_BASE+SAM3U_UART_SR_OFFSET)
#define SAM3U_USART1_RHR               (SAM3U_USART1_BASE+SAM3U_UART_RHR_OFFSET)
#define SAM3U_USART1_THR               (SAM3U_USART1_BASE+SAM3U_UART_THR_OFFSET)
#define SAM3U_USART1_BRGR              (SAM3U_USART1_BASE+SAM3U_UART_BRGR_OFFSET)
#define SAM3U_USART1_RTOR              (SAM3U_USART1_BASE+SAM3U_USART_RTOR_OFFSET)
#define SAM3U_USART1_TTGR              (SAM3U_USART1_BASE+SAM3U_USART_TTGR_OFFSET)
#define SAM3U_USART1_FIDI              (SAM3U_USART1_BASE+SAM3U_USART_FIDI_OFFSET)
#define SAM3U_USART1_NER               (SAM3U_USART1_BASE+SAM3U_USART_NER_OFFSET)
#define SAM3U_USART1_IF                (SAM3U_USART1_BASE+SAM3U_USART_IF_OFFSET)
#define SAM3U_USART1_MAN               (SAM3U_USART1_BASE+SAM3U_USART_MAN_OFFSET)
#define SAM3U_USART1_WPMR              (SAM3U_USART1_BASE+SAM3U_USART_WPMR_OFFSET)
#define SAM3U_USART1_WPSR              (SAM3U_USART1_BASE+SAM3U_USART_WPSR_OFFSET)
#define SAM3U_USART1_VERSION           (SAM3U_USART1_BASE+SAM3U_USART_VERSION_OFFSET)

#define SAM3U_USART2_CR                (SAM3U_USART2_BASE+SAM3U_UART_CR_OFFSET)
#define SAM3U_USART2_MR_               (SAM3U_USART2_BASE+SAM3U_UART_MR_OFFSET)
#define SAM3U_USART2_IER               (SAM3U_USART2_BASE+SAM3U_UART_IER_OFFSET)
#define SAM3U_USART2_IDR               (SAM3U_USART2_BASE+SAM3U_UART_IDR_OFFSET)
#define SAM3U_USART2_IMR               (SAM3U_USART2_BASE+SAM3U_UART_IMR_OFFSET)
#define SAM3U_USART2_SR                (SAM3U_USART2_BASE+SAM3U_UART_SR_OFFSET)
#define SAM3U_USART2_RHR               (SAM3U_USART2_BASE+SAM3U_UART_RHR_OFFSET)
#define SAM3U_USART2_THR               (SAM3U_USART2_BASE+SAM3U_UART_THR_OFFSET)
#define SAM3U_USART2_BRGR              (SAM3U_USART2_BASE+SAM3U_UART_BRGR_OFFSET)
#define SAM3U_USART2_RTOR              (SAM3U_USART2_BASE+SAM3U_USART_RTOR_OFFSET)
#define SAM3U_USART2_TTGR              (SAM3U_USART2_BASE+SAM3U_USART_TTGR_OFFSET)
#define SAM3U_USART2_FIDI              (SAM3U_USART2_BASE+SAM3U_USART_FIDI_OFFSET)
#define SAM3U_USART2_NER               (SAM3U_USART2_BASE+SAM3U_USART_NER_OFFSET)
#define SAM3U_USART2_IF                (SAM3U_USART2_BASE+SAM3U_USART_IF_OFFSET)
#define SAM3U_USART2_MAN               (SAM3U_USART2_BASE+SAM3U_USART_MAN_OFFSET)
#define SAM3U_USART2_WPMR              (SAM3U_USART2_BASE+SAM3U_USART_WPMR_OFFSET)
#define SAM3U_USART2_WPSR              (SAM3U_USART2_BASE+SAM3U_USART_WPSR_OFFSET)
#define SAM3U_USART2_VERSION           (SAM3U_USART2_BASE+SAM3U_USART_VERSION_OFFSET)

#define SAM3U_USART3_CR                (SAM3U_USART3_BASE+SAM3U_UART_CR_OFFSET)
#define SAM3U_USART3_MR_               (SAM3U_USART3_BASE+SAM3U_UART_MR_OFFSET)
#define SAM3U_USART3_IER               (SAM3U_USART3_BASE+SAM3U_UART_IER_OFFSET)
#define SAM3U_USART3_IDR               (SAM3U_USART3_BASE+SAM3U_UART_IDR_OFFSET)
#define SAM3U_USART3_IMR               (SAM3U_USART3_BASE+SAM3U_UART_IMR_OFFSET)
#define SAM3U_USART3_SR                (SAM3U_USART3_BASE+SAM3U_UART_SR_OFFSET)
#define SAM3U_USART3_RHR               (SAM3U_USART3_BASE+SAM3U_UART_RHR_OFFSET)
#define SAM3U_USART3_THR               (SAM3U_USART3_BASE+SAM3U_UART_THR_OFFSET)
#define SAM3U_USART3_BRGR              (SAM3U_USART3_BASE+SAM3U_UART_BRGR_OFFSET)
#define SAM3U_USART3_RTOR              (SAM3U_USART3_BASE+SAM3U_USART_RTOR_OFFSET)
#define SAM3U_USART3_TTGR              (SAM3U_USART3_BASE+SAM3U_USART_TTGR_OFFSET)
#define SAM3U_USART3_FIDI              (SAM3U_USART3_BASE+SAM3U_USART_FIDI_OFFSET)
#define SAM3U_USART3_NER               (SAM3U_USART3_BASE+SAM3U_USART_NER_OFFSET)
#define SAM3U_USART3_IF                (SAM3U_USART3_BASE+SAM3U_USART_IF_OFFSET)
#define SAM3U_USART3_MAN               (SAM3U_USART3_BASE+SAM3U_USART_MAN_OFFSET)
#define SAM3U_USART3_WPMR              (SAM3U_USART3_BASE+SAM3U_USART_WPMR_OFFSET)
#define SAM3U_USART3_WPSR              (SAM3U_USART3_BASE+SAM3U_USART_WPSR_OFFSET)
#define SAM3U_USART3_VERSION           (SAM3U_USART3_BASE+SAM3U_USART_VERSION_OFFSET)

/* UART register bit definitions ****************************************************************/

/* UART Control Register */

#define UART_CR_RSTRX                  (1 << 2)  /* Bit 2:  Reset Receiver (Common) */
#define UART_CR_RSTTX                  (1 << 3)  /* Bit 3:  Reset Transmitter (Common) */
#define UART_CR_RXEN                   (1 << 4)  /* Bit 4:  Receiver Enable (Common) */
#define UART_CR_RXDIS                  (1 << 5)  /* Bit 5:  Receiver Disable (Common) */
#define UART_CR_TXEN                   (1 << 6)  /* Bit 6:  Transmitter Enable (Common) */
#define UART_CR_TXDIS                  (1 << 7)  /* Bit 7:  Transmitter Disable (Common) */
#define UART_CR_RSTSTA                 (1 << 8)  /* Bit 8:  Reset Status Bits (Common) */
#define USART_CR_STTBRK                (1 << 9)  /* Bit 9:  Start Break (USART only) */
#define USART_CR_STPBRK                (1 << 10) /* Bit 10: Stop Break (USART only) */
#define USART_CR_STTTO                 (1 << 11) /* Bit 11: Start Time-out (USART only) */
#define USART_CR_SENDA                 (1 << 12) /* Bit 12: Send Address (USART only) */
#define USART_CR_RSTIT                 (1 << 13) /* Bit 13: Reset Iterations (USART only) */
#define USART_CR_RSTNACK               (1 << 14) /* Bit 14: Reset Non Acknowledge (USART only) */
#define USART_CR_RETTO                 (1 << 15) /* Bit 15: Rearm Time-out (USART only) */
#define USART_CR_RTSEN                 (1 << 18) /* Bit 18: Request to Send Enable (USART only) */
#define USART_CR_FCS                   (1 << 18) /* Bit 18: Force SPI Chip Select (USART only) */
#define USART_CR_RTSDIS                (1 << 19) /* Bit 19: Request to Send Disable (USART only) */
#define USART_CR_RCS                   (1 << 19) /* Bit 19: Release SPI Chip Select (USART only) */

/* UART Mode Register */

#define USART_MR_MODE_SHIFT            (0)       /* Bits 0-3: (USART only) */
#define USART_MR_MODE_MASK             (15 << USART_MR_MODE_SHIFT)
#  define USART_MR_MODE_NORMAL         (0  << USART_MR_MODE_SHIFT) /* Normal */
#  define USART_MR_MODE_RS485          (1  << USART_MR_MODE_SHIFT) /* RS485 */
#  define USART_MR_MODE_HWHS           (2  << USART_MR_MODE_SHIFT) /* Hardware Handshaking */
#  define USART_MR_MODE_ISO7816_0      (4  << USART_MR_MODE_SHIFT) /* IS07816 Protocol: T = 0 */
#  define USART_MR_MODE_ISO7816_1      (6  << USART_MR_MODE_SHIFT) /* IS07816 Protocol: T = 1 */
#  define USART_MR_MODE_IRDA           (8  << USART_MR_MODE_SHIFT) /* IrDA */
#  define USART_MR_MODE_SPIMSTR        (14 << USART_MR_MODE_SHIFT) /* SPI Master */
#  define USART_MR_MODE_SPISLV         (15 << USART_MR_MODE_SHIFT) /* SPI Slave */
#define USART_MR_USCLKS_SHIFT          (4)       /* Bits 4-5: Clock Selection (USART only) */
#define USART_MR_USCLKS_MASK           (3 << USART_MR_USCLKS_SHIFT)
#  define USART_MR_USCLKS_MCK          (0 << USART_MR_USCLKS_SHIFT) /* MCK */
#  define USART_MR_USCLKS_MCKDIV       (1 << USART_MR_USCLKS_SHIFT) /* MCK/DIV (DIV = 8) */
#  define USART_MR_USCLKS_SCK          (3 << USART_MR_USCLKS_SHIFT) /* SCK */
#define USART_MR_CHRL_SHIFT            (6)       /* Bits 6-7: Character Length (USART only) */
#define USART_MR_CHRL_MASK             (3 << USART_MR_CHRL_SHIFT)
#  define USART_MR_CHRL_5BITS          (0 << USART_MR_CHRL_SHIFT) /* 5 bits */
#  define USART_MR_CHRL_6BITS          (1 << USART_MR_CHRL_SHIFT) /* 6 bits */
#  define USART_MR_CHRL_7BITS          (2 << USART_MR_CHRL_SHIFT) /* 7 bits */
#  define USART_MR_CHRL_8BITS          (3 << USART_MR_CHRL_SHIFT) /* 8 bits */
#define USART_MR_YNC                   (1 << 8)  /* Bit 8: Synchronous Mode Select (USART only) */
#define USART_MR_CPHA                  (1 << 8)  /* Bit 8: SPI Clock Phase (USART only) */
#define UART_MR_PAR_SHIFT              (9)       /* Bits 9-11: Parity Type (Common) */
#define UART_MR_PAR_MASK               (7 << UART_MR_PAR_SHIFT)
#  define UART_MR_PAR_EVEN             (0 << UART_MR_PAR_SHIFT) /* Even parity (Common) */
#  define UART_MR_PAR_ODD              (1 << UART_MR_PAR_SHIFT) /* Odd parity (Common) */
#  define UART_MR_PAR_SPACE            (2 << UART_MR_PAR_SHIFT) /* Space: parity forced to 0 (Common) */
#  define UART_MR_PAR_MARK             (3 << UART_MR_PAR_SHIFT) /* Mark: parity forced to 1 (Common) */
#  define UART_MR_PAR_NONE             (4 << UART_MR_PAR_SHIFT) /* No parity (Common) */
#  define UART_MR_PAR_MULTIDROP        (6 << UART_MR_PAR_SHIFT) /* Multidrop mode (USART only) */
#define USART_MR_NBSTOP_SHIFT          (12)      /* Bits 12-13: Number of Stop Bits (USART only) */
#define USART_MR_NBSTOP_MASK           (3 << USART_MR_NBSTOP_SHIFT)
#  define USART_MR_NBSTOP_1            (0 << USART_MR_NBSTOP_SHIFT) /* 1 stop bit 1 stop bit */
#  define USART_MR_NBSTOP_1p5          (1 << USART_MR_NBSTOP_SHIFT) /* 1.5 stop bits */
#  define USART_MR_NBSTOP_2            (2 << USART_MR_NBSTOP_SHIFT) /* 2 stop bits 2 stop bits */
#define UART_MR_CHMODE_SHIFT           (14)      /* Bits 14-15: Channel Mode (Common) */
#define UART_MR_CHMODE_MASK            (3 << UART_MR_CHMODE_SHIFT)
#  define UART_MR_CHMODE_NORMAL        (0 << UART_MR_CHMODE_SHIFT) /* Normal Mode */
#  define UART_MR_CHMODE_ECHO          (1 << UART_MR_CHMODE_SHIFT) /* Automatic Echo */
#  define UART_MR_CHMODE_LLPBK         (2 << UART_MR_CHMODE_SHIFT) /* Local Loopback */
#  define UART_MR_CHMODE_RLPBK         (3 << UART_MR_CHMODE_SHIFT) /* Remote Loopback */
#define USART_MR_MSBF                  (1 << 16) /* Bit 16: Bit Order or SPI Clock Polarity (USART only) */
#define USART_MR_CPOL                  (1 << 16)
#define USART_MR_MODE9                 (1 << 17) /* Bit 17: 9-bit Character Length (USART only) */
#define USART_MR_CLKO                  (1 << 18) /* Bit 18: Clock Output Select (USART only) */
#define USART_MR_OVER                  (1 << 19) /* Bit 19: Oversampling Mode (USART only) */
#define USART_MR_INACK                 (1 << 20) /* Bit 20: Inhibit Non Acknowledge (USART only) */
#define USART_MR_DSNACK                (1 << 21) /* Bit 21: Disable Successive NACK (USART only) */
#define USART_MR_VARSYNC               (1 << 22) /* Bit 22: Variable Synchronization of Command/Data Sync Start Frame Delimiter (USART only) */
#define USART_MR_INVDATA               (1 << 23) /* Bit 23: INverted Data (USART only) */
#define USART_MR_MAXITER_SHIFT         (24)      /* Bits 24-26: Max iterations (ISO7816 T=0 (USART only) */
#define USART_MR_MAXITER_MASK          (7 << USART_MR_MAXITER_SHIFT)
#define USART_MR_FILTER                (1 << 28) /* Bit 28: Infrared Receive Line Filter (USART only) */
#define USART_MR_MAN                   (1 << 29) /* Bit 29: Manchester Encoder/Decoder Enable (USART only) */
#define USART_MR_MODSYNC               (1 << 30) /* Bit 30: Manchester Synchronization Mode (USART only) */
#define USART_MR_ONEBIT                (1 << 31) /* Bit 31: Start Frame Delimiter Selector (USART only) */

/* UART Interrupt Enable Register, UART Interrupt Disable Register, UART Interrupt Mask
 * Register, and UART Status Register common bit field definitions
 */

#define UART_INT_RXRDY                 (1 << 0)  /* Bit 0:  RXRDY Interrupt (Common) */
#define UART_INT_TXRDY                 (1 << 1)  /* Bit 1:  TXRDY Interrupt (Common) */
#define UART_INT_RXBRK                 (1 << 2)  /* Bit 2:  Break Received/End of Break */
#define UART_INT_ENDRX                 (1 << 3)  /* Bit 3:  End of Receive Transfer Interrupt (Common) */
#define UART_INT_ENDTX                 (1 << 4)  /* Bit 4:  End of Transmit Interrupt (Common) */
#define UART_INT_OVRE                  (1 << 5)  /* Bit 5:  Overrun Error Interrupt (Common) */
#define UART_INT_FRAME                 (1 << 6)  /* Bit 6:  Framing Error Interrupt (Common) */
#define UART_INT_PARE                  (1 << 7)  /* Bit 7:  Parity Error Interrupt (Common) */
#define USART_INT_TIMEOUT              (1 << 8)  /* Bit 8:  Time-out Interrupt (USART only) */
#define UART_INT_TXEMPTY               (1 << 9)  /* Bit 9:  TXEMPTY Interrupt (Common) */
#define USART_INT_ITER                 (1 << 10) /* Bit 10: Iteration Interrupt (USART only) */
#define USART_INT_UNRE                 (1 << 10) /* Bit 10: SPI Underrun Error Interrupt (USART only) */
#define UART_INT_TXBUFE                (1 << 11) /* Bit 11: Buffer Empty Interrupt (Common) */
#define UART_INT_RXBUFF                (1 << 12) /* Bit 12: Buffer Full Interrupt (Common) */
#define USART_INT_NACK                 (1 << 13) /* Bit 13: Non Acknowledge Interrupt (USART only) */
#define USART_INT_CTSIC                (1 << 19) /* Bit 19: Clear to Send Input Change Interrupt (USART only) */
#define USART_INT_MANE                 (1 << 24) /* Bit 24: Manchester Error Interrupt (USART only) */

/* UART Receiver Holding Register */

#define UART_RHR_RXCHR_SHIFT           (0)       /* Bits 0-7: Received Character (UART only) */
#define UART_RHR_RXCHR_MASK            (0xff << UART_RHR_RXCHR_SHIFT)
#define USART_RHR_RXCHR_SHIFT          (0)       /* Bits 0-8: Received Character (USART only) */
#define USART_RHR_RXCHR_MASK           (0x1ff << UART_RHR_RXCHR_SHIFT)
#define USART_RHR_RXSYNH               (1 << 15) /* Bit 15: Received Sync (USART only) */

/* UART Transmit Holding Register */

#define UART_THR_TXCHR_SHIFT           (0)       /* Bits 0-7: Character to be Transmitted (UART only) */
#define UART_THR_TXCHR_MASK            (0xff << UART_THR_TXCHR_SHIFT)
#define USART_THR_TXCHR_SHIFT          (0)       /* Bits 0-8: Character to be Transmitted (USART only) */
#define USART_THR_TXCHR_MASK           (0x1ff << USART_THR_TXCHR_SHIFT)
#define USART_THR_TXSYNH               (1 << 15) /* Bit 15: Sync Field to be tran (USART only) */

/* UART Baud Rate Generator Register */

#define UART_BRGR_CD_SHIFT             (0)      /* Bits 0-15: Clock Divisor (Common) */
#define UART_BRGR_CD_MASK              (0xffff << UART_BRGR_CD_SHIFT)
#define UART_BRGR_FP_SHIFT             (16)      /* Bits 16-18: Fractional Part (USART only) */
#define UART_BRGR_FP_MASK              (7 << UART_BRGR_FP_SHIFT)

/* USART Receiver Time-out Register (USART only) */

#define USART_RTOR_TO_SHIFT            (0)       /* Bits 0-15: Time-out Value (USART only) */
#define USART_RTOR_TO_MASK             (0xffff << USART_RTOR_TO_SHIFT)

/* USART Transmitter Timeguard Register (USART only) */

#define USART_TTGR_TG_SHIFT            (0)       /* Bits 0-7: Timeguard Value (USART only) */
#define USART_TTGR_TG_MASK             (0xff << USART_TTGR_TG_SHIFT)

/* USART FI DI RATIO Register (USART only) */

#define USART_FIDI_RATIO_SHIFT         (0)       /* Bits 0-10: FI Over DI Ratio Value (USART only) */
#define USART_FIDI_RATIO_MASK          (0x7ff << USART_FIDI_RATIO_SHIFT)

/* USART Number of Errors Register (USART only) */

#define USART_NER_NBERRORS_SHIFT       (0)       /* Bits 0-7: Number of Errrors (USART only) */
#define USART_NER_NBERRORS_MASK        (0xff << USART_NER_NBERRORS_SHIFT)

/* USART IrDA FILTER Register (USART only) */

#define USART_IF_IRDAFILTER_SHIFT      (0)       /* Bits 0-7: IrDA Filter (USART only) */
#define USART_IF_IRDAFILTER_MASK       (0xff << USART_IF_IRDAFILTER_SHIFT)

/* USART Manchester Configuration Register (USART only) */

#define USART_MAN_TXPL_SHIFT           (0)       /* Bits 0-3: Transmitter Preamble Length (USART only) */
#define USART_MAN_TXPL_MASK            (15 << USART_MAN_TXPL_SHIFT)
#define USART_MAN_TXPP_SHIFT           (8)       /* Bits 8-9: Transmitter Preamble Pattern (USART only) */
#define USART_MAN_TXPP_MASK            (3 << USART_MAN_TXPP_SHIFT)
#  define USART_MAN_TXPP_ALLONE        (0 << USART_MAN_TXPP_SHIFT) /* ALL_ONE */
#  define USART_MAN_TXPP_ALLZERO       (1 << USART_MAN_TXPP_SHIFT) /* ALL_ZERO */
#  define USART_MAN_TXPP_ZEROONE       (2 << USART_MAN_TXPP_SHIFT) /* ZERO_ONE */
#  define USART_MAN_TXPP_ONEZERO       (3 << USART_MAN_TXPP_SHIFT) /* ONE_ZERO */
#define USART_MAN_TXMPOL               (1 << 12) /* Bit 12: Transmitter Manchester Polarity (USART only) */
#define USART_MAN_RXPL_SHIFT           (16)      /* Bits 16-19: Receiver Preamble Length (USART only) */
#define USART_MAN_RXPL_MASK            (15 << USART_MAN_RXPL_SHIFT)
#define USART_MAN_RXPP_SHIFT           (24)      /* Bits 24-25: Receiver Preamble Pattern detected (USART only) */
#define USART_MAN_RXPP_MASK            (3 << USART_MAN_RXPP_SHIFT)
#  define USART_MAN_RXPP_ALLONE        (0 << USART_MAN_RXPP_SHIFT) /* ALL_ONE */
#  define USART_MAN_RXPP_ALLZERO       (1 << USART_MAN_RXPP_SHIFT) /* ALL_ZERO */
#  define USART_MAN_RXPP_ZEROONE       (2 << USART_MAN_RXPP_SHIFT) /* ZERO_ONE */
#  define USART_MAN_RXPP_ONEZERO       (3 << USART_MAN_RXPP_SHIFT) /* ONE_ZERO */
#define USART_MAN_RXMPOL               (1 << 28) /* Bit 28: Receiver Manchester Polarity (USART only) */
#define USART_MAN_DRIFT                (1 << 30) /* Bit 30: Drift compensation (USART only) */

/* USART Write Protect Mode Register (USART only) */

#define USART_WPMR_WPEN                (1 << 0)  /* Bit 0: Write Protect Enable (USART only) */
#define USART_WPMR_WPKEY_SHIFT         (8)       /* Bits 8-31: Write Protect KEY (USART only) */
#define USART_WPMR_WPKEY_MASK          (0x00ffffff << USART_WPMR_WPKEY_SHIFT)

/* USART Write Protect Status Register (USART only) */

#define USART_WPSR_WPVS                (1 << 0)  /* Bit 0: Write Protect Violation Status (USART only) */
#define USART_WPSR_WPVSRC_SHIFT        (8)       /* Bits 8-23: Write Protect Violation Source (USART only) */
#define USART_WPSR_WPVSRC_MASK         (0xffff << USART_WPSR_WPVSRC_SHIFT)

/* USART Version Register */

#define USART_VERSION_VERSION_SHIFT    (0)       /* Bits 0-11: Macrocell version number (USART only) */
#define USART_VERSION_VERSION_MASK     (0xfff << USART_VERSION_VERSION_SHIFT)
#define USART_VERSION_MFN_SHIFT        (16)      /* Bits 16-18: Reserved (USART only) */
#define USART_VERSION_MFN_MASK         (7 << USART_VERSION_MFN_SHIFT)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_UART_H */
