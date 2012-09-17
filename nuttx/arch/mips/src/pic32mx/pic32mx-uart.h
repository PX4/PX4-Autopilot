/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-uart.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_UART_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "pic32mx-memorymap.h"

/************************************************************************************
 * Pre-Processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define PIC32MX_UART_MODE_OFFSET    0x0000 /* UARTx mode register */
#define PIC32MX_UART_MODECLR_OFFSET 0x0004 /* UARTx mode clear register */
#define PIC32MX_UART_MODESET_OFFSET 0x0008 /* UARTx mode set register */
#define PIC32MX_UART_MODEINV_OFFSET 0x000c /* UARTx mode invert register */
#define PIC32MX_UART_STA_OFFSET     0x0010 /* UARTx status and control register */
#define PIC32MX_UART_STACLR_OFFSET  0x0014 /* UARTx status and control clear register */
#define PIC32MX_UART_STASET_OFFSET  0x0018 /* UARTx status and control set register */
#define PIC32MX_UART_STAINV_OFFSET  0x001c /* UARTx status and control invert register */
#define PIC32MX_UART_TXREG_OFFSET   0x0020 /* UARTx transmit register */
#define PIC32MX_UART_RXREG_OFFSET   0x0030 /* UARTx receive register */
#define PIC32MX_UART_BRG_OFFSET     0x0040 /* UARTx baud rate register */
#define PIC32MX_UART_BRGCLR_OFFSET  0x0044 /* UARTx baud rate clear register */
#define PIC32MX_UART_BRGSET_OFFSET  0x0048 /* UARTx baud rate set register */
#define PIC32MX_UART_BRGINV_OFFSET  0x004c /* UARTx baud rate invert register */

/* Register Addresses ****************************************************************/

#if CHIP_NUARTS > 0
#  define PIC32MX_UART1_MODE        (PIC32MX_UART1_K1BASE+PIC32MX_UART_MODE_OFFSET)
#  define PIC32MX_UART1_MODECLR     (PIC32MX_UART1_K1BASE+PIC32MX_UART_MODECLR_OFFSET)
#  define PIC32MX_UART1_MODESET     (PIC32MX_UART1_K1BASE+PIC32MX_UART_MODESET_OFFSET)
#  define PIC32MX_UART1_MODEINV     (PIC32MX_UART1_K1BASE+PIC32MX_UART_MODEINV_OFFSET)
#  define PIC32MX_UART1_STA         (PIC32MX_UART1_K1BASE+PIC32MX_UART_STA_OFFSET)
#  define PIC32MX_UART1_STACLR      (PIC32MX_UART1_K1BASE+PIC32MX_UART_STACLR_OFFSET)
#  define PIC32MX_UART1_STASET      (PIC32MX_UART1_K1BASE+PIC32MX_UART_STASET_OFFSET)
#  define PIC32MX_UART1_STAINV      (PIC32MX_UART1_K1BASE+PIC32MX_UART_STAINV_OFFSET)
#  define PIC32MX_UART1_TXREG       (PIC32MX_UART1_K1BASE+PIC32MX_UART_TXREG_OFFSET)
#  define PIC32MX_UART1_RXREG       (PIC32MX_UART1_K1BASE+PIC32MX_UART_RXREG_OFFSET)
#  define PIC32MX_UART1_BRG         (PIC32MX_UART1_K1BASE+PIC32MX_UART_BRG_OFFSET)
#  define PIC32MX_UART1_BRGCLR      (PIC32MX_UART1_K1BASE+PIC32MX_UART_BRGCLR_OFFSET)
#  define PIC32MX_UART1_BRGSET      (PIC32MX_UART1_K1BASE+PIC32MX_UART_BRGSET_OFFSET)
#  define PIC32MX_UART1_BRGINV      (PIC32MX_UART1_K1BASE+PIC32MX_UART_BRGINV_OFFSET)
#endif

#if CHIP_NUARTS > 1
#  define PIC32MX_UART2_MODE        (PIC32MX_UART2_K1BASE+PIC32MX_UART_MODE_OFFSET)
#  define PIC32MX_UART2_MODECLR     (PIC32MX_UART2_K1BASE+PIC32MX_UART_MODECLR_OFFSET)
#  define PIC32MX_UART2_MODESET     (PIC32MX_UART2_K1BASE+PIC32MX_UART_MODESET_OFFSET)
#  define PIC32MX_UART2_MODEINV     (PIC32MX_UART2_K1BASE+PIC32MX_UART_MODEINV_OFFSET)
#  define PIC32MX_UART2_STA         (PIC32MX_UART2_K1BASE+PIC32MX_UART_STA_OFFSET)
#  define PIC32MX_UART2_STACLR      (PIC32MX_UART2_K1BASE+PIC32MX_UART_STACLR_OFFSET)
#  define PIC32MX_UART2_STASET      (PIC32MX_UART2_K1BASE+PIC32MX_UART_STASET_OFFSET)
#  define PIC32MX_UART2_STAINV      (PIC32MX_UART2_K1BASE+PIC32MX_UART_STAINV_OFFSET)
#  define PIC32MX_UART2_TXREG       (PIC32MX_UART2_K1BASE+PIC32MX_UART_TXREG_OFFSET)
#  define PIC32MX_UART2_RXREG       (PIC32MX_UART2_K1BASE+PIC32MX_UART_RXREG_OFFSET)
#  define PIC32MX_UART2_BRG         (PIC32MX_UART2_K1BASE+PIC32MX_UART_BRG_OFFSET)
#  define PIC32MX_UART2_BRGCLR      (PIC32MX_UART2_K1BASE+PIC32MX_UART_BRGCLR_OFFSET)
#  define PIC32MX_UART2_BRGSET      (PIC32MX_UART2_K1BASE+PIC32MX_UART_BRGSET_OFFSET)
#  define PIC32MX_UART2_BRGINV      (PIC32MX_UART2_K1BASE+PIC32MX_UART_BRGINV_OFFSET)
#endif

#if CHIP_NUARTS > 2
#  define PIC32MX_UART3_MODE        (PIC32MX_UART3_K1BASE+PIC32MX_UART_MODE_OFFSET)
#  define PIC32MX_UART3_MODECLR     (PIC32MX_UART3_K1BASE+PIC32MX_UART_MODECLR_OFFSET)
#  define PIC32MX_UART3_MODESET     (PIC32MX_UART3_K1BASE+PIC32MX_UART_MODESET_OFFSET)
#  define PIC32MX_UART3_MODEINV     (PIC32MX_UART3_K1BASE+PIC32MX_UART_MODEINV_OFFSET)
#  define PIC32MX_UART3_STA         (PIC32MX_UART3_K1BASE+PIC32MX_UART_STA_OFFSET)
#  define PIC32MX_UART3_STACLR      (PIC32MX_UART3_K1BASE+PIC32MX_UART_STACLR_OFFSET)
#  define PIC32MX_UART3_STASET      (PIC32MX_UART3_K1BASE+PIC32MX_UART_STASET_OFFSET)
#  define PIC32MX_UART3_STAINV      (PIC32MX_UART3_K1BASE+PIC32MX_UART_STAINV_OFFSET)
#  define PIC32MX_UART3_TXREG       (PIC32MX_UART3_K1BASE+PIC32MX_UART_TXREG_OFFSET)
#  define PIC32MX_UART3_RXREG       (PIC32MX_UART3_K1BASE+PIC32MX_UART_RXREG_OFFSET)
#  define PIC32MX_UART3_BRG         (PIC32MX_UART3_K1BASE+PIC32MX_UART_BRG_OFFSET)
#  define PIC32MX_UART3_BRGCLR      (PIC32MX_UART3_K1BASE+PIC32MX_UART_BRGCLR_OFFSET)
#  define PIC32MX_UART3_BRGSET      (PIC32MX_UART3_K1BASE+PIC32MX_UART_BRGSET_OFFSET)
#  define PIC32MX_UART3_BRGINV      (PIC32MX_UART3_K1BASE+PIC32MX_UART_BRGINV_OFFSET)
#endif

#if CHIP_NUARTS > 3
#  define PIC32MX_UART4_MODE        (PIC32MX_UART4_K1BASE+PIC32MX_UART_MODE_OFFSET)
#  define PIC32MX_UART4_MODECLR     (PIC32MX_UART4_K1BASE+PIC32MX_UART_MODECLR_OFFSET)
#  define PIC32MX_UART4_MODESET     (PIC32MX_UART4_K1BASE+PIC32MX_UART_MODESET_OFFSET)
#  define PIC32MX_UART4_MODEINV     (PIC32MX_UART4_K1BASE+PIC32MX_UART_MODEINV_OFFSET)
#  define PIC32MX_UART4_STA         (PIC32MX_UART4_K1BASE+PIC32MX_UART_STA_OFFSET)
#  define PIC32MX_UART4_STACLR      (PIC32MX_UART4_K1BASE+PIC32MX_UART_STACLR_OFFSET)
#  define PIC32MX_UART4_STASET      (PIC32MX_UART4_K1BASE+PIC32MX_UART_STASET_OFFSET)
#  define PIC32MX_UART4_STAINV      (PIC32MX_UART4_K1BASE+PIC32MX_UART_STAINV_OFFSET)
#  define PIC32MX_UART4_TXREG       (PIC32MX_UART4_K1BASE+PIC32MX_UART_TXREG_OFFSET)
#  define PIC32MX_UART4_RXREG       (PIC32MX_UART4_K1BASE+PIC32MX_UART_RXREG_OFFSET)
#  define PIC32MX_UART4_BRG         (PIC32MX_UART4_K1BASE+PIC32MX_UART_BRG_OFFSET)
#  define PIC32MX_UART4_BRGCLR      (PIC32MX_UART4_K1BASE+PIC32MX_UART_BRGCLR_OFFSET)
#  define PIC32MX_UART4_BRGSET      (PIC32MX_UART4_K1BASE+PIC32MX_UART_BRGSET_OFFSET)
#  define PIC32MX_UART4_BRGINV      (PIC32MX_UART4_K1BASE+PIC32MX_UART_BRGINV_OFFSET)
#endif

#if CHIP_NUARTS > 4
#  define PIC32MX_UART5_MODE        (PIC32MX_UART5_K1BASE+PIC32MX_UART_MODE_OFFSET)
#  define PIC32MX_UART5_MODECLR     (PIC32MX_UART5_K1BASE+PIC32MX_UART_MODECLR_OFFSET)
#  define PIC32MX_UART5_MODESET     (PIC32MX_UART5_K1BASE+PIC32MX_UART_MODESET_OFFSET)
#  define PIC32MX_UART5_MODEINV     (PIC32MX_UART5_K1BASE+PIC32MX_UART_MODEINV_OFFSET)
#  define PIC32MX_UART5_STA         (PIC32MX_UART5_K1BASE+PIC32MX_UART_STA_OFFSET)
#  define PIC32MX_UART5_STACLR      (PIC32MX_UART5_K1BASE+PIC32MX_UART_STACLR_OFFSET)
#  define PIC32MX_UART5_STASET      (PIC32MX_UART5_K1BASE+PIC32MX_UART_STASET_OFFSET)
#  define PIC32MX_UART5_STAINV      (PIC32MX_UART5_K1BASE+PIC32MX_UART_STAINV_OFFSET)
#  define PIC32MX_UART5_TXREG       (PIC32MX_UART5_K1BASE+PIC32MX_UART_TXREG_OFFSET)
#  define PIC32MX_UART5_RXREG       (PIC32MX_UART5_K1BASE+PIC32MX_UART_RXREG_OFFSET)
#  define PIC32MX_UART5_BRG         (PIC32MX_UART5_K1BASE+PIC32MX_UART_BRG_OFFSET)
#  define PIC32MX_UART5_BRGCLR      (PIC32MX_UART5_K1BASE+PIC32MX_UART_BRGCLR_OFFSET)
#  define PIC32MX_UART5_BRGSET      (PIC32MX_UART5_K1BASE+PIC32MX_UART_BRGSET_OFFSET)
#  define PIC32MX_UART5_BRGINV      (PIC32MX_UART5_K1BASE+PIC32MX_UART_BRGINV_OFFSET)
#endif

#if CHIP_NUARTS > 5
#  define PIC32MX_UART6_MODE        (PIC32MX_UART6_K1BASE+PIC32MX_UART_MODE_OFFSET)
#  define PIC32MX_UART6_MODECLR     (PIC32MX_UART6_K1BASE+PIC32MX_UART_MODECLR_OFFSET)
#  define PIC32MX_UART6_MODESET     (PIC32MX_UART6_K1BASE+PIC32MX_UART_MODESET_OFFSET)
#  define PIC32MX_UART6_MODEINV     (PIC32MX_UART6_K1BASE+PIC32MX_UART_MODEINV_OFFSET)
#  define PIC32MX_UART6_STA         (PIC32MX_UART6_K1BASE+PIC32MX_UART_STA_OFFSET)
#  define PIC32MX_UART6_STACLR      (PIC32MX_UART6_K1BASE+PIC32MX_UART_STACLR_OFFSET)
#  define PIC32MX_UART6_STASET      (PIC32MX_UART6_K1BASE+PIC32MX_UART_STASET_OFFSET)
#  define PIC32MX_UART6_STAINV      (PIC32MX_UART6_K1BASE+PIC32MX_UART_STAINV_OFFSET)
#  define PIC32MX_UART6_TXREG       (PIC32MX_UART6_K1BASE+PIC32MX_UART_TXREG_OFFSET)
#  define PIC32MX_UART6_RXREG       (PIC32MX_UART6_K1BASE+PIC32MX_UART_RXREG_OFFSET)
#  define PIC32MX_UART6_BRG         (PIC32MX_UART6_K1BASE+PIC32MX_UART_BRG_OFFSET)
#  define PIC32MX_UART6_BRGCLR      (PIC32MX_UART6_K1BASE+PIC32MX_UART_BRGCLR_OFFSET)
#  define PIC32MX_UART6_BRGSET      (PIC32MX_UART6_K1BASE+PIC32MX_UART_BRGSET_OFFSET)
#  define PIC32MX_UART6_BRGINV      (PIC32MX_UART6_K1BASE+PIC32MX_UART_BRGINV_OFFSET)
#endif

/* Register Bit-Field Definitions ****************************************************/

/* UARTx mode register */

#define UART_MODE_STSEL             (1 << 0)  /* Bit 0:  Stop selection 1=2 stop bits */
#define UART_MODE_PDSEL_SHIFT       (1)       /* Bits: 1-2: Parity and data selection */
#define UART_MODE_PDSEL_MASK        (3 << UART_MODE_PDSEL_SHIFT)
#  define UART_MODE_PDSEL_8NONE     (0 << UART_MODE_PDSEL_SHIFT) /* 8-bit data, no parity */
#  define UART_MODE_PDSEL_8EVEN     (1 << UART_MODE_PDSEL_SHIFT) /* 8-bit data, even parity */
#  define UART_MODE_PDSEL_8ODD      (2 << UART_MODE_PDSEL_SHIFT) /* 8-bit data, odd parity */
#  define UART_MODE_PDSEL_9NONE     (3 << UART_MODE_PDSEL_SHIFT) /* 9-bit data, no parity */
#define UART_MODE_BRGH              (1 << 3)  /* Bit 3:  High baud rate enable */
#define UART_MODE_RXINV             (1 << 4)  /* Bit 4:  Receive polarity inversion */
#define UART_MODE_ABAUD             (1 << 5)  /* Bit 5:  Auto-baud enable */
#define UART_MODE_LPBACK            (1 << 6)  /* Bit 6:  UARTx loopback mode select */
#define UART_MODE_WAKE              (1 << 7)  /* Bit 7:  Enable wake-up on start bit detect during sleep mode */
#define UART_MODE_UEN_SHIFT         (8)       /* Bits: 8-9: UARTx enable */
#define UART_MODE_UEN_MASK          (3 << UART_MODE_UEN_SHIFT)
#  define UART_MODE_UEN_PORT        (0 << UART_MODE_UEN_SHIFT) /* UxCTS+UxRTS/UxBCLK=PORTx register */
#  define UART_MODE_UEN_ENR_CPORT   (1 << UART_MODE_UEN_SHIFT) /* UxRTS=enabled; UxCTS=ORTx register */
#  define UART_MODE_UEN_ENCR        (2 << UART_MODE_UEN_SHIFT) /* UxCTS+UxRTS=enabled */
#  define UART_MODE_UEN_CPORT       (3 << UART_MODE_UEN_SHIFT) /* UxCTS=PORTx register */
#define UART_MODE_RTSMD             (1 << 11) /* Bit 11: Mode selection for ~UxRTS pin */
#define UART_MODE_IREN              (1 << 12) /* Bit 12: IrDA encoder and decoder enable */
#define UART_MODE_SIDL              (1 << 13) /* Bit 13: Stop in idle mode */

#if !defined(CHIP_PIC32MX1) && !defined(CHIP_PIC32MX2)
#  define UART_MODE_FRZ             (1 << 14) /* Bit 14: Freeze in debug exception mode */
#endif

#define UART_MODE_ON                (1 << 15) /* Bit 15: UARTx enable */

/* UARTx status and control register */

#define UART_STA_URXDA              (1 << 0)  /* Bit 0: Receive buffer data available */
#define UART_STA_OERR               (1 << 1)  /* Bit 1: Receive buffer overrun error status */
#define UART_STA_FERR               (1 << 2)  /* Bit 2: Framing error status */
#define UART_STA_PERR               (1 << 3)  /* Bit 3: Parity error status */
#define UART_STA_RIDLE              (1 << 4)  /* Bit 4: Receiver idle */
#define UART_STA_ADDEN              (1 << 5)  /* Bit 5: Address character detect */
#define UART_STA_URXISEL_SHIFT      (6)       /* Bits: 6-7: Receive interrupt mode selection */
#define UART_STA_URXISEL_MASK       (3 << UART_STA_URXISEL_SHIFT)
#if CHIP_UARTFIFOD == 4
#  define UART_STA_URXISEL_RECVD    (0 << UART_STA_URXISEL_SHIFT) /* Character received */
#  define UART_STA_URXISEL_RXB3     (2 << UART_STA_URXISEL_SHIFT) /* RX buffer 3/4 */
#  define UART_STA_URXISEL_RXBF     (3 << UART_STA_URXISEL_SHIFT) /* RX buffer full */
#elif CHIP_UARTFIFOD == 8
#  define UART_STA_URXISEL_RECVD    (0 << UART_STA_URXISEL_SHIFT) /* Character received */
#  define UART_STA_URXISEL_RXB4     (1 << UART_STA_URXISEL_SHIFT) /* RX buffer 1/2 */
#  define UART_STA_URXISEL_RXB6     (2 << UART_STA_URXISEL_SHIFT) /* RX buffer 3/4 */
#endif
#define UART_STA_UTRMT              (1 << 8)  /* Bit 8: Transmit shift register is empty */
#define UART_STA_UTXBF              (1 << 9)  /* Bit 9: Transmit buffer full status */
#define UART_STA_UTXEN              (1 << 10) /* Bit 10: Transmit enable */
#define UART_STA_UTXBRK             (1 << 11) /* Bit 11: Transmit break */
#define UART_STA_URXEN              (1 << 12) /* Bit 12: Receiver enable */
#define UART_STA_UTXINV             (1 << 13) /* Bit 13: Transmit polarity inversion */
#define UART_STA_UTXISEL_SHIFT      (14)      /* Bits:  14-15: TX interrupt mode selection bi */
#define UART_STA_UTXISEL_MASK       (3 << UART_STA_UTXISEL_SHIFT)
#  define UART_STA_UTXISEL_TXBNF    (0 << UART_STA_UTXISEL_SHIFT) /* TX buffer not full */
#  define UART_STA_UTXISEL_DRAINED  (1 << UART_STA_UTXISEL_SHIFT) /* All characters sent */
#  define UART_STA_UTXISEL_TXBE     (2 << UART_STA_UTXISEL_SHIFT) /* TX buffer empty */
#define UART_STA_ADDR_SHIFT         (16)      /* Bits:16-23: Automatic address mask */
#define UART_STA_ADDR_MASK          (0xff << UART_STA_ADDR_SHIFT)
#define UART_STA_ADM_EN             (1 << 24) /* Bit 24: Automatic address detect mode enable */

/* UARTx transmit register */

#define UART_TXREG_MASK             0x1ff

/* UARTx receive register */

#define UART_RXREG_MASK             0x1ff

/* UARTx baud rate register */

#define UART_BRG_MASK               0xffff

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

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
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_UART_H */
