/************************************************************************************
 * arch/arm/src/lm/chip/lm_uart.h
 *
 *   Copyright (C) 2009, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LM_CHIP_LM_UART_H
#define __ARCH_ARM_SRC_LM_CHIP_LM_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* UART register offsets ************************************************************/

#define LM_UART_DR_OFFSET          0x000 /* UART Data */
#define LM_UART_RSR_OFFSET         0x004 /* UART Receive Status */
#define LM_UART_ECR_OFFSET         0x004 /* UART Error Clear */
#define LM_UART_FR_OFFSET          0x018 /* UART Flag */
#define LM_UART_ILPR_OFFSET        0x020 /* UART IrDA Low-Power Register */
#define LM_UART_IBRD_OFFSET        0x024 /* UART Integer Baud-Rate Divisor*/
#define LM_UART_FBRD_OFFSET        0x028 /* UART Fractional Baud-Rate Divisor */
#define LM_UART_LCRH_OFFSET        0x02c /* UART Line Control */
#define LM_UART_CTL_OFFSET         0x030 /* UART Control */
#define LM_UART_IFLS_OFFSET        0x034 /* UART Interrupt FIFO Level Select */
#define LM_UART_IM_OFFSET          0x038 /* UART Interrupt Mask */
#define LM_UART_RIS_OFFSET         0x03c /* UART Raw Interrupt Status */
#define LM_UART_MIS_OFFSET         0x040 /* UART Masked Interrupt Status */
#define LM_UART_ICR_OFFSET         0x044 /* UART Interrupt Clear */
#define LM_UART_PERIPHID4_OFFSET   0xfd0 /* UART Peripheral Identification 4 */
#define LM_UART_PERIPHID5_OFFSET   0xfd4 /* UART Peripheral Identification 5 */
#define LM_UART_PERIPHID6_OFFSET   0xfd8 /* UART Peripheral Identification 6 */
#define LM_UART_PERIPHID7_OFFSET   0xfdc /* UART Peripheral Identification 7 */
#define LM_UART_PERIPHID0_OFFSET   0xfe0 /* UART Peripheral Identification 0 */
#define LM_UART_PERIPHID1_OFFSET   0xfe4 /* UART Peripheral Identification 1 */
#define LM_UART_PERIPHID2_OFFSET   0xfe8 /* UART Peripheral Identification 2 */
#define LM_UART_PERIPHID3_OFFSET   0xfec /* UART Peripheral Identification 3 */
#define LM_UART_PCELLID0_OFFSET    0xff0 /* UART PrimeCell Identification 0 */
#define LM_UART_PCELLID1_OFFSET    0xff4 /* UART PrimeCell Identification 1 */
#define LM_UART_PCELLID2_OFFSET    0xff8 /* UART PrimeCell Identification 2 */
#define LM_UART_PCELLID3_OFFSET    0xffc /* UART PrimeCell Identification 3 */

/* UART register addresses **********************************************************/

#define LM_UART_BASE(n)            (LM_UART0_BASE + (n)*0x01000)

#define LM_UART_DR(n)              (LM_UART_BASE(n) + LM_UART_DR_OFFSET)
#define LM_UART_RSR(n)             (LM_UART_BASE(n) + LM_UART_RSR_OFFSET)
#define LM_UART_ECR(n)             (LM_UART_BASE(n) + LM_UART_ECR_OFFSET)
#define LM_UART_FR(n)              (LM_UART_BASE(n) + LM_UART_FR_OFFSET)
#define LM_UART_ILPR(n)            (LM_UART_BASE(n) + LM_UART_ILPR_OFFSET)
#define LM_UART_IBRD(n)            (LM_UART_BASE(n) + LM_UART_IBRD_OFFSET)
#define LM_UART_FBRD(n)            (LM_UART_BASE(n) + LM_UART_FBRD_OFFSET)
#define LM_UART_LCRH(n)            (LM_UART_BASE(n) + LM_UART_LCRH_OFFSET)
#define LM_UART_CTL(n)             (LM_UART_BASE(n) + LM_UART_CTL_OFFSET)
#define LM_UART_IFLS(n)            (LM_UART_BASE(n) + LM_UART_IFLS_OFFSET)
#define LM_UART_IM(n)              (LM_UART_BASE(n) + LM_UART_IM_OFFSET)
#define LM_UART_RIS(n)             (LM_UART_BASE(n) + LM_UART_RIS_OFFSET)
#define LM_UART_MIS(n)             (LM_UART_BASE(n) + LM_UART_MIS_OFFSET)
#define LM_UART_ICR(n)             (LM_UART_BASE(n) + LM_UART_ICR_OFFSET)
#define LM_UART_PERIPHID4(n)       (LM_UART_BASE(n) + LM_UART_PERIPHID4_OFFSET)
#define LM_UART_PERIPHID5(n)       (LM_UART_BASE(n) + LM_UART_PERIPHID5_OFFSET)
#define LM_UART_PERIPHID6(n)       (LM_UART_BASE(n) + LM_UART_PERIPHID6_OFFSET)
#define LM_UART_PERIPHID7(n)       (LM_UART_BASE(n) + LM_UART_PERIPHID7_OFFSET)
#define LM_UART_PERIPHID0(n)       (LM_UART_BASE(n) + LM_UART_PERIPHID0_OFFSET)
#define LM_UART_PERIPHID1(n)       (LM_UART_BASE(n) + LM_UART_PERIPHID1_OFFSET)
#define LM_UART_PERIPHID2(n)       (LM_UART_BASE(n) + LM_UART_PERIPHID2_OFFSET)
#define LM_UART_PERIPHID3(n)       (LM_UART_BASE(n) + LM_UART_PERIPHID3_OFFSET)
#define LM_UART_PCELLID0(n)        (LM_UART_BASE(n) + LM_UART_PCELLID0_OFFSET)
#define LM_UART_PCELLID1(n)        (LM_UART_BASE(n) + LM_UART_PCELLID1_OFFSET)
#define LM_UART_PCELLID2(n)        (LM_UART_BASE(n) + LM_UART_PCELLID2_OFFSET)
#define LM_UART_PCELLID3(n)        (LM_UART_BASE(n) + LM_UART_PCELLID3_OFFSET)

#define LM_UART0_DR                (LM_UART0_BASE + LM_UART_TDR_OFFSET)
#define LM_UART0_RSR               (LM_UART0_BASE + LM_UART_RSR_OFFSET)
#define LM_UART0_ECR               (LM_UART0_BASE + LM_UART_ECR_OFFSET)
#define LM_UART0_FR                (LM_UART0_BASE + LM_UART_FR_OFFSET)
#define LM_UART0_ILPR              (LM_UART0_BASE + LM_UART_ILPR_OFFSET)
#define LM_UART0_IBRD              (LM_UART0_BASE + LM_UART_IBRD_OFFSET)
#define LM_UART0_FBRD              (LM_UART0_BASE + LM_UART_FBRD_OFFSET)
#define LM_UART0_LCRH              (LM_UART0_BASE + LM_UART_LCRH_OFFSET)
#define LM_UART0_CTL               (LM_UART0_BASE + LM_UART_CTL_OFFSET)
#define LM_UART0_IFLS              (LM_UART0_BASE + LM_UART_IFLS_OFFSET)
#define LM_UART0_IM                (LM_UART0_BASE + LM_UART_IM_OFFSET)
#define LM_UART0_RIS               (LM_UART0_BASE + LM_UART_RIS_OFFSET)
#define LM_UART0_MIS               (LM_UART0_BASE + LM_UART_MIS_OFFSET)
#define LM_UART0_ICR               (LM_UART0_BASE + LM_UART_ICR_OFFSET)
#define LM_UART0_PERIPHID4         (LM_UART0_BASE + LM_UART_PERIPHID4_OFFSET)
#define LM_UART0_PERIPHID5         (LM_UART0_BASE + LM_UART_PERIPHID5_OFFSET)
#define LM_UART0_PERIPHID6         (LM_UART0_BASE + LM_UART_PERIPHID6_OFFSET)
#define LM_UART0_PERIPHID7         (LM_UART0_BASE + LM_UART_PERIPHID7_OFFSET)
#define LM_UART0_PERIPHID0         (LM_UART0_BASE + LM_UART_PERIPHID0_OFFSET)
#define LM_UART0_PERIPHID1         (LM_UART0_BASE + LM_UART_PERIPHID1_OFFSET)
#define LM_UART0_PERIPHID2         (LM_UART0_BASE + LM_UART_PERIPHID2_OFFSET)
#define LM_UART0_PERIPHID3         (LM_UART0_BASE + LM_UART_PERIPHID3_OFFSET)
#define LM_UART0_PCELLID0          (LM_UART0_BASE + LM_UART_PCELLID0_OFFSET)
#define LM_UART0_PCELLID1          (LM_UART0_BASE + LM_UART_PCELLID1_OFFSET)
#define LM_UART0_PCELLID2          (LM_UART0_BASE + LM_UART_PCELLID2_OFFSET)
#define LM_UART0_PCELLID3          (LM_UART0_BASE + LM_UART_PCELLID3_OFFSET)

#define LM_UART1_DR                (LM_UART1_BASE + LM_UART_DR_OFFSET)
#define LM_UART1_RSR               (LM_UART1_BASE + LM_UART_RSR_OFFSET)
#define LM_UART1_ECR               (LM_UART1_BASE + LM_UART_ECR_OFFSET)
#define LM_UART1_FR                (LM_UART1_BASE + LM_UART_FR_OFFSET)
#define LM_UART1_ILPR              (LM_UART1_BASE + LM_UART_ILPR_OFFSET)
#define LM_UART1_IBRD              (LM_UART1_BASE + LM_UART_IBRD_OFFSET)
#define LM_UART1_FBRD              (LM_UART1_BASE + LM_UART_FBRD_OFFSET)
#define LM_UART1_LCRH              (LM_UART1_BASE + LM_UART_LCRH_OFFSET)
#define LM_UART1_CTL               (LM_UART1_BASE + LM_UART_CTL_OFFSET)
#define LM_UART1_IFLS              (LM_UART1_BASE + LM_UART_IFLS_OFFSET)
#define LM_UART1_IM                (LM_UART1_BASE + LM_UART_IM_OFFSET)
#define LM_UART1_RIS               (LM_UART1_BASE + LM_UART_RIS_OFFSET)
#define LM_UART1_MIS               (LM_UART1_BASE + LM_UART_MIS_OFFSET)
#define LM_UART1_ICR               (LM_UART1_BASE + LM_UART_ICR_OFFSET)
#define LM_UART1_PERIPHID4         (LM_UART1_BASE + LM_UART_PERIPHID4_OFFSET)
#define LM_UART1_PERIPHID5         (LM_UART1_BASE + LM_UART_PERIPHID5_OFFSET)
#define LM_UART1_PERIPHID6         (LM_UART1_BASE + LM_UART_PERIPHID6_OFFSET)
#define LM_UART1_PERIPHID7         (LM_UART1_BASE + LM_UART_PERIPHID7_OFFSET)
#define LM_UART1_PERIPHID0         (LM_UART1_BASE + LM_UART_PERIPHID0_OFFSET)
#define LM_UART1_PERIPHID1         (LM_UART1_BASE + LM_UART_PERIPHID1_OFFSET)
#define LM_UART1_PERIPHID2         (LM_UART1_BASE + LM_UART_PERIPHID2_OFFSET)
#define LM_UART1_PERIPHID3         (LM_UART1_BASE + LM_UART_PERIPHID3_OFFSET)
#define LM_UART1_PCELLID0          (LM_UART1_BASE + LM_UART_PCELLID0_OFFSET)
#define LM_UART1_PCELLID1          (LM_UART1_BASE + LM_UART_PCELLID1_OFFSET)
#define LM_UART1_PCELLID2          (LM_UART1_BASE + LM_UART_PCELLID2_OFFSET)
#define LM_UART1_PCELLID3          (LM_UART1_BASE + LM_UART_PCELLID3_OFFSET)

/* UART register bit settings *******************************************************/

/* UART Data (DR), offset 0x000 */

#define UART_DR_DATA_SHIFT         0         /* Bits 7-0: Data Transmitted or Received */
#define UART_DR_DATA_MASK          (0xff << UART_DR_DATA_SHIFT)
#define UART_DR_FE                 (1 << 8)  /* Bit 8:  UART Framing Error */
#define UART_DR_PE                 (1 << 9)  /* Bit 9:  UART Parity Error */
#define UART_DR_BE                 (1 << 10) /* Bit 10: UART Break Error */
#define UART_DR_OE                 (1 << 11) /* Bit 11: UART Overrun Error */

/* UART Receive Status (RSR), offset 0x004 */

#define UART_RSR_FE                (1 << 0)  /* Bit 0:  UART Framing Error */
#define UART_RSR_PE                (1 << 1)  /* Bit 1:  UART Parity Error */
#define UART_RSR_BE                (1 << 2)  /* Bit 2:  UART Break Error */
#define UART_RSR_OE                (1 << 3)  /* Bit 3:  UART Overrun Error */

/* UART Error Clear (ECR), offset 0x004 */
/* Writing any value to this register clears pending error indications */

/* UART Flag (FR), offset 0x018 */

#define UART_FR_BUSY               (1 << 3)  /* Bit 3:  UART Busy */
#define UART_FR_RXFE               (1 << 4)  /* Bit 4:  UART Receive FIFO Empty */
#define UART_FR_TXFF               (1 << 5)  /* Bit 5:  UART Transmit FIFO Full */
#define UART_FR_RXFF               (1 << 6)  /* Bit 6:  UART Receive FIFO Full */
#define UART_FR_TXFE               (1 << 7)  /* Bit 7:  UART Transmit FIFO Empty */

/* UART IrDA Low-Power Register (ILPR), offset 0x020 */

#define UART_ILPR_DVSR_MASK        (0xff)    /* Bits 7-0: IrDA Low-Power Divisor */

/* UART Integer Baud-Rate Divisor (IBRD), offset 0x024 */

#define UART_IBRD_DIVINT_MASK      (0xffff)  /* Bits 15-0: Integer Baud-Rate Divisor */

/* UART Fractional Baud-Rate Divisor (UARTFBRD), offset 0x028 */

#define UART_FBRD_DIVFRAC_MASK     (0x3f)    /* Bits 5-0: Fractional Baud-Rate Divisor */

/* Register 7: UART Line Control (LCRH), offset 0x02C */

#define UART_LCRH_BRK              (1 << 0)  /* Bit 0:  UART Send Break */
#define UART_LCRH_PEN              (1 << 1)  /* Bit 1:  UART Parity Enable */
#define UART_LCRH_EPS              (1 << 2)  /* Bit 2:  UART Even Parity Select */
#define UART_LCRH_STP2             (1 << 3)  /* Bit 3:  UART Two Stop Bits Select */
#define UART_LCRH_FEN              (1 << 4)  /* Bit 4:  UART Enable FIFOs */
#define UART_LCRH_WLEN_SHIFT       5         /* Bits 6-5: UART Word Length */
#define UART_LCRH_WLEN_MASK        (3 << UART_LCRH_WLEN_SHIFT)
#  define UART_LCRH_WLEN_5BITS     (0 << UART_LCRH_WLEN_SHIFT) /* 5-bits (reset) */
#  define UART_LCRH_WLEN_6BITS     (1 << UART_LCRH_WLEN_SHIFT) /* 6-bits */
#  define UART_LCRH_WLEN_7BITS     (2 << UART_LCRH_WLEN_SHIFT) /* 7-bits */
#  define UART_LCRH_WLEN_8BITS     (3 << UART_LCRH_WLEN_SHIFT) /* 8-bits */
#define UART_LCRH_SPS              (1 << 7)  /* Bit 7:  UART Stick Parity Select */

/* UART Control (CTL), offset 0x030 */

#define UART_CTL_UARTEN            (1 << 0)  /* Bit 0:  UART Enable */
#define UART_CTL_SIREN             (1 << 1)  /* Bit 1:  UART SIR Enable */
#define UART_CTL_SIRLP             (1 << 2)  /* Bit 2:  UART SIR Low Power Mode */
#define UART_CTL_LBE               (1 << 7)  /* Bit 7:  UART Loop Back Enable */
#define UART_CTL_TXE               (1 << 8)  /* Bit 8:  UART Transmit Enable */
#define UART_CTL_RXE               (1 << 9)  /* Bit 9:  UART Receive Enable */

/* UART Interrupt FIFO Level Select (IFLS), offset 0x034 */

#define UART_IFLS_TXIFLSEL_SHIFT   0         /* Bits 2-0: UART Transmit Interrupt FIFO Level Select */
#define UART_IFLS_TXIFLSEL_MASK    (7 << UART_IFLS_TXIFLSEL_SHIFT)
#  define UART_IFLS_TXIFLSEL_18th  (0 << UART_IFLS_TXIFLSEL_SHIFT) /* 1/8th full */
#  define UART_IFLS_TXIFLSEL_14th  (1 << UART_IFLS_TXIFLSEL_SHIFT) /* 1/4th full */
#  define UART_IFLS_TXIFLSEL_half  (2 << UART_IFLS_TXIFLSEL_SHIFT) /* half full */
#  define UART_IFLS_TXIFLSEL_34th  (3 << UART_IFLS_TXIFLSEL_SHIFT) /* 3/4th full */
#  define UART_IFLS_TXIFLSEL_78th  (4 << UART_IFLS_TXIFLSEL_SHIFT) /* 7/8th full */
#define UART_IFLS_RXIFLSEL_SHIFT   3         /* Bits 5-3: UART Receive Interrupt FIFO Level Select */
#define UART_IFLS_RXIFLSEL_MASK    (7 << UART_IFLS_RXIFLSEL_SHIFT)
#  define UART_IFLS_RXIFLSEL_18th  (0 << UART_IFLS_RXIFLSEL_SHIFT) /* 1/8th full */
#  define UART_IFLS_RXIFLSEL_14th  (1 << UART_IFLS_RXIFLSEL_SHIFT) /* 1/4th full */
#  define UART_IFLS_RXIFLSEL_half  (2 << UART_IFLS_RXIFLSEL_SHIFT) /* half full */
#  define UART_IFLS_RXIFLSEL_34th  (3 << UART_IFLS_RXIFLSEL_SHIFT) /* 3/4th full */
#  define UART_IFLS_RXIFLSEL_78th  (4 << UART_IFLS_RXIFLSEL_SHIFT) /* 7/8th full */

/* UART Interrupt Mask (IM), offset 0x038 */

#define UART_IM_RXIM               (1 << 4)  /* Bit 4:  UART Receive Interrupt Mask */
#define UART_IM_TXIM               (1 << 5)  /* Bit 5:  UART Transmit Interrupt Mask */
#define UART_IM_RTIM               (1 << 6)  /* Bit 6:  UART Receive Time-Out Interrupt Mask */
#define UART_IM_FEIM               (1 << 7)  /* Bit 7:  UART Framing Error Interrupt Mask */
#define UART_IM_PEIM               (1 << 8)  /* Bit 8:  UART Parity Error Interrupt Mask */
#define UART_IM_BEIM               (1 << 9)  /* Bit 9:  UART Break Error Interrupt Mask */
#define UART_IM_OEIM               (1 << 10) /* Bit 10: UART Overrun Error Interrupt Mask */


/* UART Raw Interrupt Status (RIS), offset 0x03c */

#define UART_RIS_RXRIS             (1 << 4)  /* Bit 4:  UART Receive Raw Interrupt Status */
#define UART_RIS_TXRIS             (1 << 5)  /* Bit 5:  UART Transmit Raw Interrupt Status */
#define UART_RIS_RTRIS             (1 << 6)  /* Bit 6:  UART Receive Time-Out Raw Interrupt Status */
#define UART_RIS_FERIS             (1 << 7)  /* Bit 7:  UART Framing Error Raw Interrupt Status */
#define UART_RIS_PERIS             (1 << 8)  /* Bit 8:  UART Parity Error Raw Interrupt Status */
#define UART_RIS_BERIS             (1 << 9)  /* Bit 9:  UART Break Error Raw Interrupt Status */
#define UART_RIS_OERIS             (1 << 10) /* Bit 10: UART Overrun Error Raw Interrupt Status */

/* UART Masked Interrupt Status (MIS), offset 0x040 */

#define UART_MIS_RXMIS             (1 << 4)  /* Bit 4:  UART Receive Masked Interrupt Status */
#define UART_MIS_TXMIS             (1 << 5)  /* Bit 5:  UART Transmit Masked Interrupt Status */
#define UART_MIS_RTMIS             (1 << 6)  /* Bit 6:  UART Receive Time-Out Masked Interrupt Status */
#define UART_MIS_FEMIS             (1 << 7)  /* Bit 7:  UART Framing Error Masked Interrupt Status */
#define UART_MIS_PEMIS             (1 << 8)  /* Bit 8:  UART Parity Error Masked Interrupt Status */
#define UART_MIS_BEMIS             (1 << 9)  /* Bit 9:  UART Break Error Masked Interrupt Status */
#define UART_MIS_OEMIS             (1 << 10) /* Bit 10: UART Overrun Error Masked Interrupt Status */

/* UART Interrupt Clear (ICR), offset 0x044 */

#define UART_ICR_RXIC              (1 << 4)  /* Bit 4:  Receive Interrupt Clear */
#define UART_ICR_TXIC              (1 << 5)  /* Bit 5:  Transmit Interrupt Clear */
#define UART_ICR_RTIC              (1 << 6)  /* Bit 6:  Receive Time-Out Interrupt Clear */
#define UART_ICR_FEIC              (1 << 7)  /* Bit 7:  Framing Error Interrupt Clear */
#define UART_ICR_PEIC              (1 << 8)  /* Bit 8:  Parity Error Interrupt Clear */
#define UART_ICR_BEIC              (1 << 9)  /* Bit 9:  Break Error Interrupt Clear */
#define UART_ICR_OEIC              (1 << 10) /* Bit 10: Overrun Error Interrupt Clear
 */

/* UART Peripheral Identification 4 (PERIPHID4), offset 0xfd0 */

#define UART_PERIPHID4_MASK        (0xff)    /* UART Peripheral ID Register[7:0] */

/* UART Peripheral Identification 5 (UARTPERIPHID5), offset 0xfd4 */

#define UART_PERIPHID5_MASK        (0xff)    /* UART Peripheral ID Register[15:8] */

/* UART Peripheral Identification 6 (UARTPERIPHID6), offset 0xfd8 */

#define UART_PERIPHID6_MASK        (0xff)    /* UART Peripheral ID Register[23:16] */

/* UART Peripheral Identification 7 (UARTPERIPHID7), offset 0xfdc */

#define UART_PERIPHID7_MASK        (0xff)    /* UART Peripheral ID Register[31:24] */

/* UART Peripheral Identification 0 (UARTPERIPHID0), offset 0xfe0 */

#define UART_PERIPHID0_MASK        (0xff)    /* UART Peripheral ID Register[7:0] */

/* UART Peripheral Identification 1 (UARTPERIPHID1), offset 0xfe4 */

#define UART_PERIPHID1_MASK        (0xff)    /* UART Peripheral ID Register[15:8] */

/* UART Peripheral Identification 2 (UARTPERIPHID2), offset 0xfe8 */

#define UART_PERIPHID2_MASK        (0xff)    /* UART Peripheral ID Register[23:16] */

/* UART Peripheral Identification 3 (UARTPERIPHID3), offset 0xfec */

#define UART_PERIPHID3_MASK        (0xff)    /* UART Peripheral ID Register[31:24] */

/* UART PrimeCell Identification 0 (CELLID0), offset 0xff0 */

#define UART_CELLID0_MASK          (0xff)    /* UART PrimeCell ID Register[7:0] */

/* UART PrimeCell Identification 1 (UARTPCELLID1), offset 0xff4 */

#define UART_CELLID1_MASK          (0xff)    /* UART PrimeCell ID Register[15:8] */

/* UART PrimeCell Identification 2 (UARTPCELLID2), offset 0xff8 */

#define UART_CELLID02MASK          (0xff)    /* UART PrimeCell ID Register[23:16] */

/* UART PrimeCell Identification 3 (UARTPCELLID3), offset 0xffc */

#define UART_CELLID3_MASK          (0xff)    /* UART PrimeCell ID Register[31:24] */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LM_CHIP_LM_UART_H */
