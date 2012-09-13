/************************************************************************************
 * arch/arm/src/kinetis/kinetis_uart.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_UART_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "kinetis_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_UART_BDH_OFFSET      0x0000 /* UART Baud Rate Register High */
#define KINETIS_UART_BDL_OFFSET      0x0001 /* UART Baud Rate Register Low */
#define KINETIS_UART_C1_OFFSET       0x0002 /* UART Control Register 1 */
#define KINETIS_UART_C2_OFFSET       0x0003 /* UART Control Register 2 */
#define KINETIS_UART_S1_OFFSET       0x0004 /* UART Status Register 1 */
#define KINETIS_UART_S2_OFFSET       0x0005 /* UART Status Register 2 */
#define KINETIS_UART_C3_OFFSET       0x0006 /* UART Control Register 3 */
#define KINETIS_UART_D_OFFSET        0x0007 /* UART Data Register */
#define KINETIS_UART_MA1_OFFSET      0x0008 /* UART Match Address Registers 1 */
#define KINETIS_UART_MA2_OFFSET      0x0009 /* UART Match Address Registers 2 */
#define KINETIS_UART_C4_OFFSET       0x000a /* UART Control Register 4 */
#define KINETIS_UART_C5_OFFSET       0x000b /* UART Control Register 5 */
#define KINETIS_UART_ED_OFFSET       0x000c /* UART Extended Data Register */
#define KINETIS_UART_MODEM_OFFSET    0x000d /* UART Modem Register */
#define KINETIS_UART_IR_OFFSET       0x000e /* UART Infrared Register */
#define KINETIS_UART_PFIFO_OFFSET    0x0010 /* UART FIFO Parameters */
#define KINETIS_UART_CFIFO_OFFSET    0x0011 /* UART FIFO Control Register */
#define KINETIS_UART_SFIFO_OFFSET    0x0012 /* UART FIFO Status Register */
#define KINETIS_UART_TWFIFO_OFFSET   0x0013 /* UART FIFO Transmit Watermark */
#define KINETIS_UART_TCFIFO_OFFSET   0x0014 /* UART FIFO Transmit Count */
#define KINETIS_UART_RWFIFO_OFFSET   0x0015 /* UART FIFO Receive Watermark */
#define KINETIS_UART_RCFIFO_OFFSET   0x0016 /* UART FIFO Receive Count */
#define KINETIS_UART_C7816_OFFSET    0x0017 /* UART 7816 Control Register */
#define KINETIS_UART_IE7816_OFFSET   0x0018 /* UART 7816 Interrupt Enable Register */
#define KINETIS_UART_IS7816_OFFSET   0x0019 /* UART 7816 Interrupt Status Register */
#define KINETIS_UART_WP7816T0_OFFSET 0x001a /* UART 7816 Wait Parameter Register */
#define KINETIS_UART_WP7816T1_OFFSET 0x001b /* UART 7816 Wait Parameter Register */
#define KINETIS_UART_WN7816_OFFSET   0x001c /* UART 7816 Wait N Register */
#define KINETIS_UART_WF7816_OFFSET   0x001d /* UART 7816 Wait FD Register */
#define KINETIS_UART_ET7816_OFFSET   0x001e /* UART 7816 Error Threshold Register */
#define KINETIS_UART_TL7816_OFFSET   0x001f /* UART 7816 Transmit Length Register */

/* Register Addresses ***************************************************************/

#if (KINETIS_NISO7816+KINETIS_NUART) > 0
#  define KINETIS_UART0_BDH          (KINETIS_UART0_BASE+KINETIS_UART_BDH_OFFSET)
#  define KINETIS_UART0_BDL          (KINETIS_UART0_BASE+KINETIS_UART_BDL_OFFSET)
#  define KINETIS_UART0_C1           (KINETIS_UART0_BASE+KINETIS_UART_C1_OFFSET)
#  define KINETIS_UART0_C2           (KINETIS_UART0_BASE+KINETIS_UART_C2_OFFSET)
#  define KINETIS_UART0_S1           (KINETIS_UART0_BASE+KINETIS_UART_S1_OFFSET)
#  define KINETIS_UART0_S2           (KINETIS_UART0_BASE+KINETIS_UART_S2_OFFSET)
#  define KINETIS_UART0_C3           (KINETIS_UART0_BASE+KINETIS_UART_C3_OFFSET)
#  define KINETIS_UART0_D            (KINETIS_UART0_BASE+KINETIS_UART_D_OFFSET)
#  define KINETIS_UART0_MA1          (KINETIS_UART0_BASE+KINETIS_UART_MA1_OFFSET)
#  define KINETIS_UART0_MA2          (KINETIS_UART0_BASE+KINETIS_UART_MA2_OFFSET)
#  define KINETIS_UART0_C4           (KINETIS_UART0_BASE+KINETIS_UART_C4_OFFSET)
#  define KINETIS_UART0_C5           (KINETIS_UART0_BASE+KINETIS_UART_C5_OFFSET)
#  define KINETIS_UART0_ED           (KINETIS_UART0_BASE+KINETIS_UART_ED_OFFSET)
#  define KINETIS_UART0_MODEM        (KINETIS_UART0_BASE+KINETIS_UART_MODEM_OFFSET)
#  define KINETIS_UART0_IR           (KINETIS_UART0_BASE+KINETIS_UART_IR_OFFSET)
#  define KINETIS_UART0_PFIFO        (KINETIS_UART0_BASE+KINETIS_UART_PFIFO_OFFSET)
#  define KINETIS_UART0_CFIFO        (KINETIS_UART0_BASE+KINETIS_UART_CFIFO_OFFSET)
#  define KINETIS_UART0_SFIFO        (KINETIS_UART0_BASE+KINETIS_UART_SFIFO_OFFSET)
#  define KINETIS_UART0_TWFIFO       (KINETIS_UART0_BASE+KINETIS_UART_TWFIFO_OFFSET)
#  define KINETIS_UART0_TCFIFO       (KINETIS_UART0_BASE+KINETIS_UART_TCFIFO_OFFSET)
#  define KINETIS_UART0_RWFIFO       (KINETIS_UART0_BASE+KINETIS_UART_RWFIFO_OFFSET)
#  define KINETIS_UART0_RCFIFO       (KINETIS_UART0_BASE+KINETIS_UART_RCFIFO_OFFSET)
#  define KINETIS_UART0_C7816        (KINETIS_UART0_BASE+KINETIS_UART_C7816_OFFSET)
#  define KINETIS_UART0_IE7816       (KINETIS_UART0_BASE+KINETIS_UART_IE7816_OFFSET)
#  define KINETIS_UART0_IS7816       (KINETIS_UART0_BASE+KINETIS_UART_IS7816_OFFSET)
#  define KINETIS_UART0_WP7816T0     (KINETIS_UART0_BASE+KINETIS_UART_WP7816T0_OFFSET)
#  define KINETIS_UART0_WP7816T1     (KINETIS_UART0_BASE+KINETIS_UART_WP7816T1_OFFSET)
#  define KINETIS_UART0_WN7816       (KINETIS_UART0_BASE+KINETIS_UART_WN7816_OFFSET)
#  define KINETIS_UART0_WF7816       (KINETIS_UART0_BASE+KINETIS_UART_WF7816_OFFSET)
#  define KINETIS_UART0_ET7816       (KINETIS_UART0_BASE+KINETIS_UART_ET7816_OFFSET)
#  define KINETIS_UART0_TL7816       (KINETIS_UART0_BASE+KINETIS_UART_TL7816_OFFSET)
#endif

#if (KINETIS_NISO7816+KINETIS_NUART) > 1
#  define KINETIS_UART1_BDH          (KINETIS_UART1_BASE+KINETIS_UART_BDH_OFFSET)
#  define KINETIS_UART1_BDL          (KINETIS_UART1_BASE+KINETIS_UART_BDL_OFFSET)
#  define KINETIS_UART1_C1           (KINETIS_UART1_BASE+KINETIS_UART_C1_OFFSET)
#  define KINETIS_UART1_C2           (KINETIS_UART1_BASE+KINETIS_UART_C2_OFFSET)
#  define KINETIS_UART1_S1           (KINETIS_UART1_BASE+KINETIS_UART_S1_OFFSET)
#  define KINETIS_UART1_S2           (KINETIS_UART1_BASE+KINETIS_UART_S2_OFFSET)
#  define KINETIS_UART1_C3           (KINETIS_UART1_BASE+KINETIS_UART_C3_OFFSET)
#  define KINETIS_UART1_D            (KINETIS_UART1_BASE+KINETIS_UART_D_OFFSET)
#  define KINETIS_UART1_MA1          (KINETIS_UART1_BASE+KINETIS_UART_MA1_OFFSET)
#  define KINETIS_UART1_MA2          (KINETIS_UART1_BASE+KINETIS_UART_MA2_OFFSET)
#  define KINETIS_UART1_C4           (KINETIS_UART1_BASE+KINETIS_UART_C4_OFFSET)
#  define KINETIS_UART1_C5           (KINETIS_UART1_BASE+KINETIS_UART_C5_OFFSET)
#  define KINETIS_UART1_ED           (KINETIS_UART1_BASE+KINETIS_UART_ED_OFFSET)
#  define KINETIS_UART1_MODEM        (KINETIS_UART1_BASE+KINETIS_UART_MODEM_OFFSET)
#  define KINETIS_UART1_IR           (KINETIS_UART1_BASE+KINETIS_UART_IR_OFFSET)
#  define KINETIS_UART1_PFIFO        (KINETIS_UART1_BASE+KINETIS_UART_PFIFO_OFFSET)
#  define KINETIS_UART1_CFIFO        (KINETIS_UART1_BASE+KINETIS_UART_CFIFO_OFFSET)
#  define KINETIS_UART1_SFIFO        (KINETIS_UART1_BASE+KINETIS_UART_SFIFO_OFFSET)
#  define KINETIS_UART1_TWFIFO       (KINETIS_UART1_BASE+KINETIS_UART_TWFIFO_OFFSET)
#  define KINETIS_UART1_TCFIFO       (KINETIS_UART1_BASE+KINETIS_UART_TCFIFO_OFFSET)
#  define KINETIS_UART1_RWFIFO       (KINETIS_UART1_BASE+KINETIS_UART_RWFIFO_OFFSET)
#  define KINETIS_UART1_RCFIFO       (KINETIS_UART1_BASE+KINETIS_UART_RCFIFO_OFFSET)
#  define KINETIS_UART1_C7816        (KINETIS_UART1_BASE+KINETIS_UART_C7816_OFFSET)
#  define KINETIS_UART1_IE7816       (KINETIS_UART1_BASE+KINETIS_UART_IE7816_OFFSET)
#  define KINETIS_UART1_IS7816       (KINETIS_UART1_BASE+KINETIS_UART_IS7816_OFFSET)
#  define KINETIS_UART1_WP7816T0     (KINETIS_UART1_BASE+KINETIS_UART_WP7816T0_OFFSET)
#  define KINETIS_UART1_WP7816T1     (KINETIS_UART1_BASE+KINETIS_UART_WP7816T1_OFFSET)
#  define KINETIS_UART1_WN7816       (KINETIS_UART1_BASE+KINETIS_UART_WN7816_OFFSET)
#  define KINETIS_UART1_WF7816       (KINETIS_UART1_BASE+KINETIS_UART_WF7816_OFFSET)
#  define KINETIS_UART1_ET7816       (KINETIS_UART1_BASE+KINETIS_UART_ET7816_OFFSET)
#  define KINETIS_UART1_TL7816       (KINETIS_UART1_BASE+KINETIS_UART_TL7816_OFFSET)
#endif

#if (KINETIS_NISO7816+KINETIS_NUART) > 2
#  define KINETIS_UART2_BDH          (KINETIS_UART2_BASE+KINETIS_UART_BDH_OFFSET)
#  define KINETIS_UART2_BDL          (KINETIS_UART2_BASE+KINETIS_UART_BDL_OFFSET)
#  define KINETIS_UART2_C1           (KINETIS_UART2_BASE+KINETIS_UART_C1_OFFSET)
#  define KINETIS_UART2_C2           (KINETIS_UART2_BASE+KINETIS_UART_C2_OFFSET)
#  define KINETIS_UART2_S1           (KINETIS_UART2_BASE+KINETIS_UART_S1_OFFSET)
#  define KINETIS_UART2_S2           (KINETIS_UART2_BASE+KINETIS_UART_S2_OFFSET)
#  define KINETIS_UART2_C3           (KINETIS_UART2_BASE+KINETIS_UART_C3_OFFSET)
#  define KINETIS_UART2_D            (KINETIS_UART2_BASE+KINETIS_UART_D_OFFSET)
#  define KINETIS_UART2_MA1          (KINETIS_UART2_BASE+KINETIS_UART_MA1_OFFSET)
#  define KINETIS_UART2_MA2          (KINETIS_UART2_BASE+KINETIS_UART_MA2_OFFSET)
#  define KINETIS_UART2_C4           (KINETIS_UART2_BASE+KINETIS_UART_C4_OFFSET)
#  define KINETIS_UART2_C5           (KINETIS_UART2_BASE+KINETIS_UART_C5_OFFSET)
#  define KINETIS_UART2_ED           (KINETIS_UART2_BASE+KINETIS_UART_ED_OFFSET)
#  define KINETIS_UART2_MODEM        (KINETIS_UART2_BASE+KINETIS_UART_MODEM_OFFSET)
#  define KINETIS_UART2_IR           (KINETIS_UART2_BASE+KINETIS_UART_IR_OFFSET)
#  define KINETIS_UART2_PFIFO        (KINETIS_UART2_BASE+KINETIS_UART_PFIFO_OFFSET)
#  define KINETIS_UART2_CFIFO        (KINETIS_UART2_BASE+KINETIS_UART_CFIFO_OFFSET)
#  define KINETIS_UART2_SFIFO        (KINETIS_UART2_BASE+KINETIS_UART_SFIFO_OFFSET)
#  define KINETIS_UART2_TWFIFO       (KINETIS_UART2_BASE+KINETIS_UART_TWFIFO_OFFSET)
#  define KINETIS_UART2_TCFIFO       (KINETIS_UART2_BASE+KINETIS_UART_TCFIFO_OFFSET)
#  define KINETIS_UART2_RWFIFO       (KINETIS_UART2_BASE+KINETIS_UART_RWFIFO_OFFSET)
#  define KINETIS_UART2_RCFIFO       (KINETIS_UART2_BASE+KINETIS_UART_RCFIFO_OFFSET)
#  define KINETIS_UART2_C7816        (KINETIS_UART2_BASE+KINETIS_UART_C7816_OFFSET)
#  define KINETIS_UART2_IE7816       (KINETIS_UART2_BASE+KINETIS_UART_IE7816_OFFSET)
#  define KINETIS_UART2_IS7816       (KINETIS_UART2_BASE+KINETIS_UART_IS7816_OFFSET)
#  define KINETIS_UART2_WP7816T0     (KINETIS_UART2_BASE+KINETIS_UART_WP7816T0_OFFSET)
#  define KINETIS_UART2_WP7816T1     (KINETIS_UART2_BASE+KINETIS_UART_WP7816T1_OFFSET)
#  define KINETIS_UART2_WN7816       (KINETIS_UART2_BASE+KINETIS_UART_WN7816_OFFSET)
#  define KINETIS_UART2_WF7816       (KINETIS_UART2_BASE+KINETIS_UART_WF7816_OFFSET)
#  define KINETIS_UART2_ET7816       (KINETIS_UART2_BASE+KINETIS_UART_ET7816_OFFSET)
#  define KINETIS_UART2_TL7816       (KINETIS_UART2_BASE+KINETIS_UART_TL7816_OFFSET)
#endif

#if (KINETIS_NISO7816+KINETIS_NUART) > 3
#  define KINETIS_UART3_BDH          (KINETIS_UART3_BASE+KINETIS_UART_BDH_OFFSET)
#  define KINETIS_UART3_BDL          (KINETIS_UART3_BASE+KINETIS_UART_BDL_OFFSET)
#  define KINETIS_UART3_C1           (KINETIS_UART3_BASE+KINETIS_UART_C1_OFFSET)
#  define KINETIS_UART3_C2           (KINETIS_UART3_BASE+KINETIS_UART_C2_OFFSET)
#  define KINETIS_UART3_S1           (KINETIS_UART3_BASE+KINETIS_UART_S1_OFFSET)
#  define KINETIS_UART3_S2           (KINETIS_UART3_BASE+KINETIS_UART_S2_OFFSET)
#  define KINETIS_UART3_C3           (KINETIS_UART3_BASE+KINETIS_UART_C3_OFFSET)
#  define KINETIS_UART3_D            (KINETIS_UART3_BASE+KINETIS_UART_D_OFFSET)
#  define KINETIS_UART3_MA1          (KINETIS_UART3_BASE+KINETIS_UART_MA1_OFFSET)
#  define KINETIS_UART3_MA2          (KINETIS_UART3_BASE+KINETIS_UART_MA2_OFFSET)
#  define KINETIS_UART3_C4           (KINETIS_UART3_BASE+KINETIS_UART_C4_OFFSET)
#  define KINETIS_UART3_C5           (KINETIS_UART3_BASE+KINETIS_UART_C5_OFFSET)
#  define KINETIS_UART3_ED           (KINETIS_UART3_BASE+KINETIS_UART_ED_OFFSET)
#  define KINETIS_UART3_MODEM        (KINETIS_UART3_BASE+KINETIS_UART_MODEM_OFFSET)
#  define KINETIS_UART3_IR           (KINETIS_UART3_BASE+KINETIS_UART_IR_OFFSET)
#  define KINETIS_UART3_PFIFO        (KINETIS_UART3_BASE+KINETIS_UART_PFIFO_OFFSET)
#  define KINETIS_UART3_CFIFO        (KINETIS_UART3_BASE+KINETIS_UART_CFIFO_OFFSET)
#  define KINETIS_UART3_SFIFO        (KINETIS_UART3_BASE+KINETIS_UART_SFIFO_OFFSET)
#  define KINETIS_UART3_TWFIFO       (KINETIS_UART3_BASE+KINETIS_UART_TWFIFO_OFFSET)
#  define KINETIS_UART3_TCFIFO       (KINETIS_UART3_BASE+KINETIS_UART_TCFIFO_OFFSET)
#  define KINETIS_UART3_RWFIFO       (KINETIS_UART3_BASE+KINETIS_UART_RWFIFO_OFFSET)
#  define KINETIS_UART3_RCFIFO       (KINETIS_UART3_BASE+KINETIS_UART_RCFIFO_OFFSET)
#  define KINETIS_UART3_C7816        (KINETIS_UART3_BASE+KINETIS_UART_C7816_OFFSET)
#  define KINETIS_UART3_IE7816       (KINETIS_UART3_BASE+KINETIS_UART_IE7816_OFFSET)
#  define KINETIS_UART3_IS7816       (KINETIS_UART3_BASE+KINETIS_UART_IS7816_OFFSET)
#  define KINETIS_UART3_WP7816T0     (KINETIS_UART3_BASE+KINETIS_UART_WP7816T0_OFFSET)
#  define KINETIS_UART3_WP7816T1     (KINETIS_UART3_BASE+KINETIS_UART_WP7816T1_OFFSET)
#  define KINETIS_UART3_WN7816       (KINETIS_UART3_BASE+KINETIS_UART_WN7816_OFFSET)
#  define KINETIS_UART3_WF7816       (KINETIS_UART3_BASE+KINETIS_UART_WF7816_OFFSET)
#  define KINETIS_UART3_ET7816       (KINETIS_UART3_BASE+KINETIS_UART_ET7816_OFFSET)
#  define KINETIS_UART3_TL7816       (KINETIS_UART3_BASE+KINETIS_UART_TL7816_OFFSET)
#endif

#if (KINETIS_NISO7816+KINETIS_NUART) > 4
#  define KINETIS_UART4_BDH          (KINETIS_UART4_BASE+KINETIS_UART_BDH_OFFSET)
#  define KINETIS_UART4_BDL          (KINETIS_UART4_BASE+KINETIS_UART_BDL_OFFSET)
#  define KINETIS_UART4_C1           (KINETIS_UART4_BASE+KINETIS_UART_C1_OFFSET)
#  define KINETIS_UART4_C2           (KINETIS_UART4_BASE+KINETIS_UART_C2_OFFSET)
#  define KINETIS_UART4_S1           (KINETIS_UART4_BASE+KINETIS_UART_S1_OFFSET)
#  define KINETIS_UART4_S2           (KINETIS_UART4_BASE+KINETIS_UART_S2_OFFSET)
#  define KINETIS_UART4_C3           (KINETIS_UART4_BASE+KINETIS_UART_C3_OFFSET)
#  define KINETIS_UART4_D            (KINETIS_UART4_BASE+KINETIS_UART_D_OFFSET)
#  define KINETIS_UART4_MA1          (KINETIS_UART4_BASE+KINETIS_UART_MA1_OFFSET)
#  define KINETIS_UART4_MA2          (KINETIS_UART4_BASE+KINETIS_UART_MA2_OFFSET)
#  define KINETIS_UART4_C4           (KINETIS_UART4_BASE+KINETIS_UART_C4_OFFSET)
#  define KINETIS_UART4_C5           (KINETIS_UART4_BASE+KINETIS_UART_C5_OFFSET)
#  define KINETIS_UART4_ED           (KINETIS_UART4_BASE+KINETIS_UART_ED_OFFSET)
#  define KINETIS_UART4_MODEM        (KINETIS_UART4_BASE+KINETIS_UART_MODEM_OFFSET)
#  define KINETIS_UART4_IR           (KINETIS_UART4_BASE+KINETIS_UART_IR_OFFSET)
#  define KINETIS_UART4_PFIFO        (KINETIS_UART4_BASE+KINETIS_UART_PFIFO_OFFSET)
#  define KINETIS_UART4_CFIFO        (KINETIS_UART4_BASE+KINETIS_UART_CFIFO_OFFSET)
#  define KINETIS_UART4_SFIFO        (KINETIS_UART4_BASE+KINETIS_UART_SFIFO_OFFSET)
#  define KINETIS_UART4_TWFIFO       (KINETIS_UART4_BASE+KINETIS_UART_TWFIFO_OFFSET)
#  define KINETIS_UART4_TCFIFO       (KINETIS_UART4_BASE+KINETIS_UART_TCFIFO_OFFSET)
#  define KINETIS_UART4_RWFIFO       (KINETIS_UART4_BASE+KINETIS_UART_RWFIFO_OFFSET)
#  define KINETIS_UART4_RCFIFO       (KINETIS_UART4_BASE+KINETIS_UART_RCFIFO_OFFSET)
#  define KINETIS_UART4_C7816        (KINETIS_UART4_BASE+KINETIS_UART_C7816_OFFSET)
#  define KINETIS_UART4_IE7816       (KINETIS_UART4_BASE+KINETIS_UART_IE7816_OFFSET)
#  define KINETIS_UART4_IS7816       (KINETIS_UART4_BASE+KINETIS_UART_IS7816_OFFSET)
#  define KINETIS_UART4_WP7816T0     (KINETIS_UART4_BASE+KINETIS_UART_WP7816T0_OFFSET)
#  define KINETIS_UART4_WP7816T1     (KINETIS_UART4_BASE+KINETIS_UART_WP7816T1_OFFSET)
#  define KINETIS_UART4_WN7816       (KINETIS_UART4_BASE+KINETIS_UART_WN7816_OFFSET)
#  define KINETIS_UART4_WF7816       (KINETIS_UART4_BASE+KINETIS_UART_WF7816_OFFSET)
#  define KINETIS_UART4_ET7816       (KINETIS_UART4_BASE+KINETIS_UART_ET7816_OFFSET)
#  define KINETIS_UART4_TL7816       (KINETIS_UART4_BASE+KINETIS_UART_TL7816_OFFSET)
#endif

#if (KINETIS_NISO7816+KINETIS_NUART) > 5
#  define KINETIS_UART5_BDH          (KINETIS_UART5_BASE+KINETIS_UART_BDH_OFFSET)
#  define KINETIS_UART5_BDL          (KINETIS_UART5_BASE+KINETIS_UART_BDL_OFFSET)
#  define KINETIS_UART5_C1           (KINETIS_UART5_BASE+KINETIS_UART_C1_OFFSET)
#  define KINETIS_UART5_C2           (KINETIS_UART5_BASE+KINETIS_UART_C2_OFFSET)
#  define KINETIS_UART5_S1           (KINETIS_UART5_BASE+KINETIS_UART_S1_OFFSET)
#  define KINETIS_UART5_S2           (KINETIS_UART5_BASE+KINETIS_UART_S2_OFFSET)
#  define KINETIS_UART5_C3           (KINETIS_UART5_BASE+KINETIS_UART_C3_OFFSET)
#  define KINETIS_UART5_D            (KINETIS_UART5_BASE+KINETIS_UART_D_OFFSET)
#  define KINETIS_UART5_MA1          (KINETIS_UART5_BASE+KINETIS_UART_MA1_OFFSET)
#  define KINETIS_UART5_MA2          (KINETIS_UART5_BASE+KINETIS_UART_MA2_OFFSET)
#  define KINETIS_UART5_C4           (KINETIS_UART5_BASE+KINETIS_UART_C4_OFFSET)
#  define KINETIS_UART5_C5           (KINETIS_UART5_BASE+KINETIS_UART_C5_OFFSET)
#  define KINETIS_UART5_ED           (KINETIS_UART5_BASE+KINETIS_UART_ED_OFFSET)
#  define KINETIS_UART5_MODEM        (KINETIS_UART5_BASE+KINETIS_UART_MODEM_OFFSET)
#  define KINETIS_UART5_IR           (KINETIS_UART5_BASE+KINETIS_UART_IR_OFFSET)
#  define KINETIS_UART5_PFIFO        (KINETIS_UART5_BASE+KINETIS_UART_PFIFO_OFFSET)
#  define KINETIS_UART5_CFIFO        (KINETIS_UART5_BASE+KINETIS_UART_CFIFO_OFFSET)
#  define KINETIS_UART5_SFIFO        (KINETIS_UART5_BASE+KINETIS_UART_SFIFO_OFFSET)
#  define KINETIS_UART5_TWFIFO       (KINETIS_UART5_BASE+KINETIS_UART_TWFIFO_OFFSET)
#  define KINETIS_UART5_TCFIFO       (KINETIS_UART5_BASE+KINETIS_UART_TCFIFO_OFFSET)
#  define KINETIS_UART5_RWFIFO       (KINETIS_UART5_BASE+KINETIS_UART_RWFIFO_OFFSET)
#  define KINETIS_UART5_RCFIFO       (KINETIS_UART5_BASE+KINETIS_UART_RCFIFO_OFFSET)
#  define KINETIS_UART5_C7816        (KINETIS_UART5_BASE+KINETIS_UART_C7816_OFFSET)
#  define KINETIS_UART5_IE7816       (KINETIS_UART5_BASE+KINETIS_UART_IE7816_OFFSET)
#  define KINETIS_UART5_IS7816       (KINETIS_UART5_BASE+KINETIS_UART_IS7816_OFFSET)
#  define KINETIS_UART5_WP7816T0     (KINETIS_UART5_BASE+KINETIS_UART_WP7816T0_OFFSET)
#  define KINETIS_UART5_WP7816T1     (KINETIS_UART5_BASE+KINETIS_UART_WP7816T1_OFFSET)
#  define KINETIS_UART5_WN7816       (KINETIS_UART5_BASE+KINETIS_UART_WN7816_OFFSET)
#  define KINETIS_UART5_WF7816       (KINETIS_UART5_BASE+KINETIS_UART_WF7816_OFFSET)
#  define KINETIS_UART5_ET7816       (KINETIS_UART5_BASE+KINETIS_UART_ET7816_OFFSET)
#  define KINETIS_UART5_TL7816       (KINETIS_UART5_BASE+KINETIS_UART_TL7816_OFFSET)
#endif

/* Register Bit Definitions *********************************************************/
/* UART Baud Rate Register High */

#define UART_BDH_SBR_SHIFT           (0)       /* Bits 0-4: MS Bits 8-13 of the UART Baud Rate Bits */
#define UART_BDH_SBR_MASK            (31 << UART_BDH_SBR_SHIFT)
                                               /* Bit 5: Reserved */
#define UART_BDH_RXEDGIE             (1 << 6)  /* Bit 6: RxD Input Active Edge Interrupt Enable */
#define UART_BDH_LBKDIE              (1 << 7)  /* Bit 7: LIN Break Detect Interrupt Enable */

/* UART Baud Rate Register Low.  Bits 0-7 of the UART baud rate bits. */

/* UART Control Register 1 */

#define UART_C1_PT                   (1 << 0)  /* Bit 0: Parity Type */
#define UART_C1_PE                   (1 << 1)  /* Bit 1: Parity Enable */
#define UART_C1_ILT                  (1 << 2)  /* Bit 2: Idle Line Type Select */
#define UART_C1_WAKE                 (1 << 3)  /* Bit 3: Receiver Wakeup Method Select */
#define UART_C1_M                    (1 << 4)  /* Bit 4: 9-bit or 8-bit Mode Select */
#define UART_C1_RSRC                 (1 << 5)  /* Bit 5: Receiver Source Select */
#define UART_C1_UARTSWAI             (1 << 6)  /* Bit 6: UART Stops in Wait Mode */
#define UART_C1_LOOPS                (1 << 7)  /* Bit 7: Loop Mode Select */

/* UART Control Register 2 */

#define UART_C2_SBK                  (1 << 0)  /* Bit 0: Send Break */
#define UART_C2_RWU                  (1 << 1)  /* Bit 1: Receiver Wakeup Control */
#define UART_C2_RE                   (1 << 2)  /* Bit 2: Receiver Enable */
#define UART_C2_TE                   (1 << 3)  /* Bit 3: Transmitter Enable */
#define UART_C2_ILIE                 (1 << 4)  /* Bit 4: Idle Line Interruptor Enable */
#define UART_C2_RIE                  (1 << 5)  /* Bit 5: Receiver Full Interrupt or DMA Transfer Enable */
#define UART_C2_TCIE                 (1 << 6)  /* Bit 6: Transmission Complete Interrupt Enable */
#define UART_C2_TIE                  (1 << 7)  /* Bit 7: Transmitter Interrupt or DMA Transfer Enable */
#define UART_C2_ALLINTS              (0xf0)

/* UART Status Register 1 */

#define UART_S1_PF                   (1 << 0)  /* Bit 0: Parity Error Flag */
#define UART_S1_FE                   (1 << 1)  /* Bit 1: Framing Error Flag */
#define UART_S1_NF                   (1 << 2)  /* Bit 2: Noise Flag */
#define UART_S1_OR                   (1 << 3)  /* Bit 3: Receiver Overrun Flag */
#define UART_S1_IDLE                 (1 << 4)  /* Bit 4: Idle Line Flag */
#define UART_S1_RDRF                 (1 << 5)  /* Bit 5: Receive Data Register Full Flag */
#define UART_S1_TC                   (1 << 6)  /* Bit 6: Transmit Complete Flag */
#define UART_S1_TDRE                 (1 << 7)  /* Bit 7: Transmit Data Register Empty Flag */

/* UART Status Register 2 */

#define UART_S2_RAF                  (1 << 0)  /* Bit 0: Receiver Active Flag */
#define UART_S2_LBKDE                (1 << 1)  /* Bit 1: LIN Break Detection Enable */
#define UART_S2_BRK13                (1 << 2)  /* Bit 2: Break Transmit Character Length */
#define UART_S2_RWUID                (1 << 3)  /* Bit 3: Receive Wakeup Idle Detect */
#define UART_S2_RXINV                (1 << 4)  /* Bit 4: Receive Data Inversion */
#define UART_S2_MSBF                 (1 << 5)  /* Bit 5: Most Significant Bit First */
#define UART_S2_RXEDGIF              (1 << 6)  /* Bit 6: RxD Pin Active Edge Interrupt Flag */
#define UART_S2_LBKDIF               (1 << 7)  /* Bit 7: LIN Break Detect Interrupt Flag */

/* UART Control Register 3 */

#define UART_C3_PEIE                 (1 << 0)  /* Bit 0: Parity Error Interrupt Enable */
#define UART_C3_FEIE                 (1 << 1)  /* Bit 1: Framing Error Interrupt Enable */
#define UART_C3_NEIE                 (1 << 2)  /* Bit 2: Noise Error Interrupt Enable */
#define UART_C3_ORIE                 (1 << 3)  /* Bit 3: Overrun Error Interrupt Enable */
#define UART_C3_TXINV                (1 << 4)  /* Bit 4: Transmit Data Inversion */
#define UART_C3_TXDIR                (1 << 5)  /* Bit 5: Transmitter Pin Data Direction in Single-Wire mode */
#define UART_C3_T8                   (1 << 6)  /* Bit 6: Transmit Bit 8 */
#define UART_C3_R8                   (1 << 7)  /* Bit 7: Received Bit 8 */

/* UART Data Register: 8-bit data register. */
/* UART Match Address Registers 1 & 2: 8-bit address registers */

/* UART Control Register 4 */

#define UART_C4_BRFA_SHIFT           (0)       /* Bits 0-4: Baud Rate Fine Adjust */
#define UART_C4_BRFA_MASK            (31 << UART_C4_BRFA_SHIFT)
#define UART_C4_M10                  (1 << 5)  /* Bit 5: 10-bit Mode select */
#define UART_C4_MAEN2                (1 << 6)  /* Bit 6: Match Address Mode Enable 2 */
#define UART_C4_MAEN1                (1 << 7)  /* Bit 7: Match Address Mode Enable 1 */

/* UART Control Register 5 */

                                               /* Bit 0-4: Reserved */
#define UART_C5_RDMAS                (1 << 5)  /* Bit 5: Receiver Full DMA Select */
                                               /* Bit 6: Reserved */
#define UART_C5_TDMAS                (1 << 7)  /* Bit 7: Transmitter DMA Select */

/* UART Extended Data Register */

                                               /* Bit 0-5: Reserved */
#define UART_ED_PARITYE              (1 << 6)  /* Bit 6: The current received dataword contained
                                                *        in D and C3[R8] was received with a parity error */
#define UART_ED_NOISY                (1 << 7)  /* Bit 7: The current received dataword contained
                                                *        in D and C3[R8] was received with noise */

/* UART Modem Register */

#define UART_MODEM_TXCTSE            (1 << 0)  /* Bit 0: Transmitter clear-to-send enable */
#define UART_MODEM_TXRTSE            (1 << 1)  /* Bit 1: Transmitter request-to-send enable */
#define UART_MODEM_TXRTSPOL          (1 << 2)  /* Bit 2: Transmitter request-to-send polarity */
#define UART_MODEM_RXRTSE            (1 << 3)  /* Bit 3: Receiver request-to-send enable */
                                               /* Bits 4-7: Reserved */

/* UART Infrared Register */

#define UART_IR_TNP_SHIFT            (0)       /* Bits 0-1: Transmitter narrow pulse */
#define UART_IR_TNP_MASK             (3 << UART_IR_TNP_SHIFT)
#  define UART_IR_TNP_316THS         (0 << UART_IR_TNP_SHIFT) /* 3/16 */
#  define UART_IR_TNP_16TH           (1 << UART_IR_TNP_SHIFT) /* 1/16 */
#  define UART_IR_TNP_32ND           (2 << UART_IR_TNP_SHIFT) /* 1/32 */
#  define UART_IR_TNP_4TH            (3 << UART_IR_TNP_SHIFT) /* 1/4 */
#define UART_IR_IREN                 (1 << 2)  /* Bit 2: Infrared enable */
                                               /* Bits 3-7: Reserved */

/* UART FIFO Parameters */

#define UART_PFIFO_RXFIFOSIZE_SHIFT  (0)       /* Bits 0-2: Receive FIFO. Buffer Depth */
#define UART_PFIFO_RXFIFOSIZE_MASK   (7 << UART_PFIFO_RXFIFOSIZE_SHIFT)
#  define UART_PFIFO_RXFIFOSIZE_1    (0 << UART_PFIFO_RXFIFOSIZE_SHIFT) /* 1 */
#  define UART_PFIFO_RXFIFOSIZE_4    (1 << UART_PFIFO_RXFIFOSIZE_SHIFT) /* 4 */
#  define UART_PFIFO_RXFIFOSIZE_8    (2 << UART_PFIFO_RXFIFOSIZE_SHIFT) /* 8 */
#  define UART_PFIFO_RXFIFOSIZE_16   (3 << UART_PFIFO_RXFIFOSIZE_SHIFT) /* 16 */
#  define UART_PFIFO_RXFIFOSIZE_32   (4 << UART_PFIFO_RXFIFOSIZE_SHIFT) /* 32 */
#  define UART_PFIFO_RXFIFOSIZE_64   (5 << UART_PFIFO_RXFIFOSIZE_SHIFT) /* 64 */
#  define UART_PFIFO_RXFIFOSIZE_128  (6 << UART_PFIFO_RXFIFOSIZE_SHIFT) /* 128 */
#define UART_PFIFO_RXFE              (1 << 3)  /* Bit 3: Receive FIFO Enable */
#define UART_PFIFO_TXFIFOSIZE_SHIFT  (4)       /* Bits 4-6: Transmit FIFO. Buffer Depth */
#define UART_PFIFO_TXFIFOSIZE_MASK   (7 << UART_PFIFO_TXFIFOSIZE_SHIFT)
#  define UART_PFIFO_TXFIFOSIZE_1    (0 << UART_PFIFO_TXFIFOSIZE_SHIFT) /* 1 */
#  define UART_PFIFO_TXFIFOSIZE_4    (1 << UART_PFIFO_TXFIFOSIZE_SHIFT) /* 4 */
#  define UART_PFIFO_TXFIFOSIZE_8    (2 << UART_PFIFO_TXFIFOSIZE_SHIFT) /* 8 */
#  define UART_PFIFO_TXFIFOSIZE_16   (3 << UART_PFIFO_TXFIFOSIZE_SHIFT) /* 16 */
#  define UART_PFIFO_TXFIFOSIZE_32   (4 << UART_PFIFO_TXFIFOSIZE_SHIFT) /* 32 */
#  define UART_PFIFO_TXFIFOSIZE_64   (5 << UART_PFIFO_TXFIFOSIZE_SHIFT) /* 64 */
#  define UART_PFIFO_TXFIFOSIZE_128  (6 << UART_PFIFO_TXFIFOSIZE_SHIFT) /* 128 */
#define UART_PFIFO_TXFE              (1 << 7)  /* Bit 7: Transmit FIFO Enable */

/* UART FIFO Control Register */

#define UART_CFIFO_RXUFE             (1 << 0)  /* Bit 0: Receive FIFO Underflow Interrupt Enable */
#define UART_CFIFO_TXOFE             (1 << 1)  /* Bit 1: Transmit FIFO Overflow Interrupt Enable */
                                               /* Bits 2-5: Reserved */
#define UART_CFIFO_RXFLUSH           (1 << 6)  /* Bit 6: Receive FIFO/Buffer Flush */
#define UART_CFIFO_TXFLUSH           (1 << 7)  /* Bit 7: Transmit FIFO/Buffer Flush */

/* UART FIFO Status Register */

#define UART_SFIFO_RXUF              (1 << 0)  /* Bit 0: Receiver Buffer Underflow Flag */
#define UART_SFIFO_TXOF              (1 << 1)  /* Bit 1: Transmitter Buffer Overflow Flag */
                                               /* Bits 2-5: Reserved */
#define UART_SFIFO_RXEMPT            (1 << 6)  /* Bit 6: Receive Buffer/FIFO Empty */
#define UART_SFIFO_TXEMPT            (1 << 7)  /* Bit 7: Transmit Buffer/FIFO Empty */

/* UART FIFO Transmit Watermark.  8-bit watermark value. */
/* UART FIFO Transmit Count. 8-bit count value */
/* UART FIFO Receive Watermark.  8-bit watermark value. */
/* UART FIFO Receive Count. 8-bit count value */

/* UART 7816 Control Register */

#define UART_C7816_ISO7816E          (1 << 0)  /* Bit 0: ISO-7816 Functionality Enabled */
#define UART_C7816_TTYPE             (1 << 1)  /* Bit 1: Transfer Type */
#define UART_C7816_INIT              (1 << 2)  /* Bit 2: Detect Initial Character */
#define UART_C7816_ANACK             (1 << 3)  /* Bit 3: Generate NACK on Error */
#define UART_C7816_ONACK             (1 << 4)  /* Bit 4: Generate NACK on Overflow */
                                               /* Bits 5-7: Reserved */

/* UART 7816 Interrupt Enable Register */

#define UART_IE7816_RXTE             (1 << 0)  /* Bit 0: Receive Threshold Exceeded Interrupt Enable */
#define UART_IE7816_TXTE             (1 << 1)  /* Bit 1: Transmit Threshold Exceeded Interrupt Enable */
#define UART_IE7816_GTVE             (1 << 2)  /* Bit 2: Guard Timer Violated Interrupt Enable */
                                               /* Bit 3: Reserved */
#define UART_IE7816_INITDE           (1 << 4)  /* Bit 4: Initial Character Detected Interrupt Enable */
#define UART_IE7816_BWTE             (1 << 5)  /* Bit 5: Block Wait Timer Interrupt Enable */
#define UART_IE7816_CWTE             (1 << 6)  /* Bit 6: Character Wait Timer Interrupt Enable */
#define UART_IE7816_WTE              (1 << 7)  /* Bit 7: Wait Timer Interrupt Enable */

/* UART 7816 Interrupt Status Register */

#define UART_IS7816_RXT              (1 << 0)  /* Bit 0: Receive Threshold Exceeded Interrupt */
#define UART_IS7816_TXT              (1 << 1)  /* Bit 1: Transmit Threshold Exceeded Interrupt */
#define UART_IS7816_GTV              (1 << 2)  /* Bit 2: Guard Timer Violated Interrupt */
                                               /* Bit 3: Reserved */
#define UART_IS7816_INITD            (1 << 4)  /* Bit 4: Initial Character Detected Interrupt */
#define UART_IS7816_BWT              (1 << 5)  /* Bit 5: Block Wait Timer Interrupt */
#define UART_IS7816_CWT              (1 << 6)  /* Bit 6: Character Wait Timer Interrupt */
#define UART_IS7816_WT               (1 << 7)  /* Bit 7: Wait Timer Interrupt */

/* UART 7816 Wait Parameter Register.  8-bit Wait Timer Interrupt value. */

/* UART 7816 Wait Parameter Register */

#define UART_WP7816T1_BWI_SHIFT      (0)       /* Bit 0-3: Block Wait Time Integer(C7816[TTYPE] = 1) */
#define UART_WP7816T1_BWI_MASK       (15 << UART_WP7816T1_BWI_SHIFT)
#define UART_WP7816T1_CWI_SHIFT      (4)       /* Bits 4-7: Character Wait Time Integer (C7816[TTYPE] = 1) */
#define UART_WP7816T1_CWI_MASK       (15 << UART_WP7816T1_CWI_SHIFT)

/* UART 7816 Wait N Register.  8-bit Guard Band value. */
/* UART 7816 Wait FD Register. 8-bit FD Multiplier value. */

/* UART 7816 Error Threshold Register */

#define UART_ET7816_RXTHRESH_SHIFT   (0)       /* Bit 0-3: Receive NACK Threshold */
#define UART_ET7816_RXTHRESH_MASK    (15 << UART_ET7816_RXTHRESHOLD_SHIFT)
#define UART_ET7816_TXTHRESH_SHIFT   (4)       /* Bits 4-7: Transmit NACK Threshold */
#define UART_ET7816_TXTHRESH_MASK    (15 << UART_ET7816_TXTHRESHOLD_MASK)

/* UART 7816 Transmit Length Register. 8-bit Transmit Length value */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_UART_H */
