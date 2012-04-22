/************************************************************************************
 * arch/arm/src/stm32/chip/stm32_uart.h
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_STC_STM32_CHIP_STM32_UART_H
#define __ARCH_ARM_STC_STM32_CHIP_STM32_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_USART_SR_OFFSET     0x0000  /* Status register (32-bits) */
#define STM32_USART_DR_OFFSET     0x0004  /* Data register (32-bits) */
#define STM32_USART_BRR_OFFSET    0x0008  /* Baud Rate Register (32-bits) */
#define STM32_USART_CR1_OFFSET    0x000c  /* Control register 1 (32-bits) */
#define STM32_USART_CR2_OFFSET    0x0010  /* Control register 2 (32-bits) */
#define STM32_USART_CR3_OFFSET    0x0014  /* Control register 3 (32-bits) */
#define STM32_USART_GTPR_OFFSET   0x0018  /* Guard time and prescaler register (32-bits) */

/* Register Addresses ***************************************************************/

#if STM32_NUSART > 0
#  define STM32_USART1_SR         (STM32_USART1_BASE+STM32_USART_SR_OFFSET)
#  define STM32_USART1_DR         (STM32_USART1_BASE+STM32_USART_DR_OFFSET)
#  define STM32_USART1_BRR        (STM32_USART1_BASE+STM32_USART_BRR_OFFSET)
#  define STM32_USART1_CR1        (STM32_USART1_BASE+STM32_USART_CR1_OFFSET)
#  define STM32_USART1_CR2        (STM32_USART1_BASE+STM32_USART_CR2_OFFSET)
#  define STM32_USART1_CR3        (STM32_USART1_BASE+STM32_USART_CR3_OFFSET)
#  define STM32_USART1_GTPR       (STM32_USART1_BASE+STM32_USART_GTPR_OFFSET)
#endif

#if STM32_NUSART > 1
#  define STM32_USART2_SR         (STM32_USART2_BASE+STM32_USART_SR_OFFSET)
#  define STM32_USART2_DR         (STM32_USART2_BASE+STM32_USART_DR_OFFSET)
#  define STM32_USART2_BRR        (STM32_USART2_BASE+STM32_USART_BRR_OFFSET)
#  define STM32_USART2_CR1        (STM32_USART2_BASE+STM32_USART_CR1_OFFSET)
#  define STM32_USART2_CR2        (STM32_USART2_BASE+STM32_USART_CR2_OFFSET)
#  define STM32_USART2_CR3        (STM32_USART2_BASE+STM32_USART_CR3_OFFSET)
#  define STM32_USART2_GTPR       (STM32_USART2_BASE+STM32_USART_GTPR_OFFSET)
#endif

#if STM32_NUSART > 2
#  define STM32_USART3_SR         (STM32_USART3_BASE+STM32_USART_SR_OFFSET)
#  define STM32_USART3_DR         (STM32_USART3_BASE+STM32_USART_DR_OFFSET)
#  define STM32_USART3_BRR        (STM32_USART3_BASE+STM32_USART_BRR_OFFSET)
#  define STM32_USART3_CR1        (STM32_USART3_BASE+STM32_USART_CR1_OFFSET)
#  define STM32_USART3_CR2        (STM32_USART3_BASE+STM32_USART_CR2_OFFSET)
#  define STM32_USART3_CR3        (STM32_USART3_BASE+STM32_USART_CR3_OFFSET)
#  define STM32_USART3_GTPR       (STM32_USART3_BASE+STM32_USART_GTPR_OFFSET)
#endif

#if STM32_NUSART > 3
#  define STM32_UART4_SR          (STM32_UART4_BASE+STM32_USART_SR_OFFSET)
#  define STM32_UART4_DR          (STM32_UART4_BASE+STM32_USART_DR_OFFSET)
#  define STM32_UART4_BRR         (STM32_UART4_BASE+STM32_USART_BRR_OFFSET)
#  define STM32_UART4_CR1         (STM32_UART4_BASE+STM32_USART_CR1_OFFSET)
#  define STM32_UART4_CR2         (STM32_UART4_BASE+STM32_USART_CR2_OFFSET)
#  define STM32_UART4_CR3         (STM32_UART4_BASE+STM32_USART_CR3_OFFSET)
#endif

#if STM32_NUSART > 4
#  define STM32_UART5_SR          (STM32_UART5_BASE+STM32_USART_SR_OFFSET)
#  define STM32_UART5_DR          (STM32_UART5_BASE+STM32_USART_DR_OFFSET)
#  define STM32_UART5_BRR         (STM32_UART5_BASE+STM32_USART_BRR_OFFSET)
#  define STM32_UART5_CR1         (STM32_UART5_BASE+STM32_USART_CR1_OFFSET)
#  define STM32_UART5_CR2         (STM32_UART5_BASE+STM32_USART_CR2_OFFSET)
#  define STM32_UART5_CR3         (STM32_UART5_BASE+STM32_USART_CR3_OFFSET)
#endif

#if STM32_NUSART > 5
#  define STM32_USART6_SR         (STM32_USART6_BASE+STM32_USART_SR_OFFSET)
#  define STM32_USART6_DR         (STM32_USART6_BASE+STM32_USART_DR_OFFSET)
#  define STM32_USART6_BRR        (STM32_USART6_BASE+STM32_USART_BRR_OFFSET)
#  define STM32_USART6_CR1        (STM32_USART6_BASE+STM32_USART_CR1_OFFSET)
#  define STM32_USART6_CR2        (STM32_USART6_BASE+STM32_USART_CR2_OFFSET)
#  define STM32_USART6_CR3        (STM32_USART6_BASE+STM32_USART_CR3_OFFSET)
#  define STM32_USART6_GTPR       (STM32_USART6_BASE+STM32_USART_GTPR_OFFSET)
#endif

/* Register Bitfield Definitions ****************************************************/

/* Status register */

#define USART_SR_PE               (1 << 0)  /* Bit 0: Parity Error */
#define USART_SR_FE               (1 << 1)  /* Bit 1: Framing Error */
#define USART_SR_NE               (1 << 2)  /* Bit 2: Noise Error Flag */
#define USART_SR_ORE              (1 << 3)  /* Bit 3: OverRun Error */
#define USART_SR_IDLE             (1 << 4)  /* Bit 4: IDLE line detected */
#define USART_SR_RXNE             (1 << 5)  /* Bit 5: Read Data Register Not Empty */
#define USART_SR_TC               (1 << 6)  /* Bit 6: Transmission Complete */
#define USART_SR_TXE              (1 << 7)  /* Bit 7: Transmit Data Register Empty */
#define USART_SR_LBD              (1 << 8)  /* Bit 8: LIN Break Detection Flag */
#define USART_SR_CTS              (1 << 9)  /* Bit 9: CTS Flag */

#define USART_SR_ALLBITS          (0x03ff)
#define USART_SR_CLRBITS          (USART_SR_CTS|USART_SR_LBD) /* Cleared by SW write to SR */

/* Data register */

#define USART_DR_SHIFT            (0)       /* Bits 8:0: Data value */
#define USART_DR_MASK             (0xff << USART_DR_SHIFT)

/* Baud Rate Register */

#define USART_BRR_FRAC_SHIFT      (0)       /* Bits 3-0: fraction of USARTDIV */
#define USART_BRR_FRAC_MASK       (0x0f << USART_BRR_FRAC_SHIFT)
#define USART_BRR_MANT_SHIFT      (4)       /* Bits 15-4: mantissa of USARTDIV */
#define USART_BRR_MANT_MASK       (0x0fff << USART_BRR_MANT_SHIFT)

/* Control register 1 */

#define USART_CR1_SBK             (1 << 0)  /* Bit 0: Send Break */
#define USART_CR1_RWU             (1 << 1)  /* Bit 1: Receiver wakeup */
#define USART_CR1_RE              (1 << 2)  /* Bit 2: Receiver Enable */
#define USART_CR1_TE              (1 << 3)  /* Bit 3: Transmitter Enable */
#define USART_CR1_IDLEIE          (1 << 4)  /* Bit 4: IDLE Interrupt Enable */
#define USART_CR1_RXNEIE          (1 << 5)  /* Bit 5: RXNE Interrupt Enable */
#define USART_CR1_TCIE            (1 << 6)  /* Bit 6: Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE           (1 << 7)  /* Bit 7: TXE Interrupt Enable */
#define USART_CR1_PEIE            (1 << 8)  /* Bit 8: PE Interrupt Enable */
#define USART_CR1_PS              (1 << 9)  /* Bit 9: Parity Selection */
#define USART_CR1_PCE             (1 << 10) /* Bit 10: Parity Control Enable */
#define USART_CR1_WAKE            (1 << 11) /* Bit 11: Wakeup method */
#define USART_CR1_M               (1 << 12) /* Bit 12: word length */
#define USART_CR1_UE              (1 << 13) /* Bit 13: USART Enable */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define USART_CR1_OVER8         (1 << 15) /* Bit 15: Oversampling mode */
#endif

#define USART_CR1_ALLINTS         (USART_CR1_IDLEIE|USART_CR1_RXNEIE|USART_CR1_TCIE|USART_CR1_PEIE)

/* Control register 2 */

#define USART_CR2_ADD_SHIFT       (0)       /* Bits 3-0: Address of the USART node */
#define USART_CR2_ADD_MASK        (0x0f << USART_CR2_ADD_SHIFT)
#define USART_CR2_LBDL            (1 << 5)  /* Bit 5: LIN Break Detection Length */
#define USART_CR2_LBDIE           (1 << 6)  /* Bit 6: LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL            (1 << 8)  /* Bit 8: Last Bit Clock pulse */
#define USART_CR2_CPHA            (1 << 9)  /* Bit 9: Clock Phase */
#define USART_CR2_CPOL            (1 << 10) /* Bit 10: Clock Polarity */
#define USART_CR2_CLKEN           (1 << 11) /* Bit 11: Clock Enable */
#define USART_CR2_STOP_SHIFT      (12)      /* Bits 13-12: STOP bits */
#define USART_CR2_STOP_MASK       (3 << USART_CR2_STOP_SHIFT)
#  define USART_CR2_STOP1         (0 << USART_CR2_STOP_SHIFT) /* 00: 1 Stop bit */
#  define USART_CR2_STOP0p5       (1 << USART_CR2_STOP_SHIFT) /* 01: 0.5 Stop bit */
#  define USART_CR2_STOP2         (2 << USART_CR2_STOP_SHIFT) /* 10: 2 Stop bits */
#  define USART_CR2_STOP1p5       (3 << USART_CR2_STOP_SHIFT) /* 11: 1.5 Stop bit */
#define USART_CR2_LINEN           (1 << 14) /* Bit 14: LIN mode enable */

/* Control register 3 */

#define USART_CR3_EIE             (1 << 0)  /* Bit 0: Error Interrupt Enable */
#define USART_CR3_IREN            (1 << 1)  /* Bit 1: IrDA mode Enable */
#define USART_CR3_IRLP            (1 << 2)  /* Bit 2: IrDA Low-Power */
#define USART_CR3_HDSEL           (1 << 3)  /* Bit 3: Half-Duplex Selection */
#define USART_CR3_NACK            (1 << 4)  /* Bit 4: Smartcard NACK enable */
#define USART_CR3_SCEN            (1 << 5)  /* Bit 5: Smartcard mode enable */
#define USART_CR3_DMAR            (1 << 6)  /* Bit 6: DMA Enable Receiver */
#define USART_CR3_DMAT            (1 << 7)  /* Bit 7: DMA Enable Transmitter */
#define USART_CR3_RTSE            (1 << 8)  /* Bit 8: RTS Enable */
#define USART_CR3_CTSE            (1 << 9)  /* Bit 9: CTS Enable */
#define USART_CR3_CTSIE           (1 << 10) /* Bit 10: CTS Interrupt Enable */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define USART_CR1_ONEBIT        (1 << 11) /* Bit 11: One sample bit method enable */
#endif

/* Guard time and prescaler register */

#define USART_GTPR_PSC_SHIFT      (0) /* Bits 0-7: Prescaler value */
#define USART_GTPR_PSC_MASK       (0xff << USART_GTPR_PSC_SHIFT)
#define USART_GTPR_GT_SHIFT       (8) /* Bits 8-15: Guard time value */
#define USART_GTPR_GT_MASK        (0xff <<  USART_GTPR_GT_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_STC_STM32_CHIP_STM32_UART_H */
