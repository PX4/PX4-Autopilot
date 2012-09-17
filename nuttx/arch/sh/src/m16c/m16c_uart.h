/************************************************************************************
 * arch/sh/src/m16c/m16c_uart.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_SH_SRC_M16C_M16C_UART_H
#define __ARCH_SH_SRC_M16C_M16C_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* UART Register Block Base Addresses ***********************************************/

#define M16C_UART0_BASE    0x003a0    /* First UART0 register */
#define M16C_UART1_BASE    0x003a8    /* First UART1 register */
#define M16C_UART2_BASE    0x00378    /* First UART2 register (ignoring special regs) */

/* UART Register Offsets ************************************************************/

#define M16C_UART_MR       0x00       /*  8-bit UART transmit/receive mode  */
#define M16C_UART_BRG      0x01       /*  8-bit UART bit rate generator    */
#define M16C_UART_TB       0x02       /* 16-bit UART transmit buffer  */
#define M16C_UART_C0       0x04       /*  8-bit UART transmit/receive control 0  */
#define M16C_UART_C1       0x05       /*  8-bit UART transmit/receive control 1  */
#define M16C_UART_RB       0x06       /* 16-bit UART receive buffer  */

/* UART Register Bit Definitions ****************************************************/

/* UART transmit/receive mode */

#define UART_MR_SMDMASK    0x07       /* Serial I/O mode select */
#define UART_MR_SMDINVALID 0x00       /*  000: Serial I/O invalid */
#define UART_MR_SMDSYNCH   0x01       /*  001: Required in Sync Mode (UART0/1) Serial I/O (UART2) */
#define UART_MR_SMDINHIB1  0x02       /*  010: Inhibited (UART0/1) I2C mode (UART2) */
#define UART_MR_SMDINHIB2  0x03       /*  011: Inhibited */
#define UART_MR_SMD7BITS   0x04       /*  100: Transfer data 7 bits long */
#define UART_MR_SMD8BITS   0x05       /*  101: Transfer data 8 bits long */
#define UART_MR_SMD9BITS   0x06       /*  110: Transfer data 9 bits long */
#define UART_MR_SMDINHIB3  0x07       /*  111: Inhibited */
#define UART_MR_CKDIR      0x08       /* Bit 3: Internal/external clock select 1=external */
#define UART_MR_STPS       0x10       /* Bit 4: Stop bit length select 1=2 stop bits */
#define UART_MR_PRY        0x20       /* Bit 5: Odd/even parity select bit 1=Even parity */
#define UART_MR_PRYE       0x40       /* Bit 6: Parity enable 1=enabled */
#define UART_MR_IOPOL      0x80       /* Bit 7: Reserved (UART0/1) Tx/Rx polarity reverse (UART2) */

/* UART receive buffer register (16-bit) */

#define UART_RB_DATAMASK   0x01ff     /* Bits 0-8: Receive data */
                                      /* Bits 9-10: Reserved */
#define UART_RB_ABT        0x0800     /* Bit 11: Arbitration lost detecting flag */
#define UART_RB_OER        0x1000     /* Bit 12: Overrun error flag */
#define UART_RB_FER        0x2000     /* Bit 13: Framing error flag */
#define UART_RB_PER        0x4000     /* Bit 14: Parity error flag */
#define UART_RB_SUM        0x8000     /* Bit 15: Error sum flag */

/* UART Transmit/Receive Control 0 */

#define UART_C0_CLKMASK    0x02       /* Bits 0-1: BRG count source select */
#define UART_C0_F1SIO      0x00       /*   00 : f1SIO or f2SIO is selected */
#define UART_C0_F8SIO      0x01       /*   01 : f8SIO is selected */
#define UART_C0_F32SIO     0x02       /*   10 : f32SIO is selected */
#define UART_C0_INHIB      0x03       /*   11 : Inhibited */
#define UART_C0_CRS        0x04       /* Bit 2: CTS/RTS function select 1=RTS */
#define UART_C0_TXEPT      0x08       /* Bit 3: Transmit register empty 1=empty */
#define UART_C0_CRD        0x10       /* Bit 4: CTS/RTS disable bit 1=CTS/RTS disabled */
#define UART_C0_NCH        0x20       /* Bit 5: Data output select 1=TxDi is N-channel open drain output */
#define UART_C0_CKPOL      0x40       /* Bit 6: CLK polarity select 1=XMT rising, recieve falling */
#define UART_C0_UFORM      0x80       /* Bit 7: Transfer format select 1=MSB first */

/* UART Transmit/Receive Control 1 */

#define UART_C1_TE         0x01       /* Bit 0: Transmit enable 1=enable */
#define UART_C1_TI         0x02       /* Bit 1: Transmit buffer empty 1=empty */
#define UART_C1_RE         0x04       /* Bit 2: Receive enable 1=enable */
#define UART_C1_RI         0x08       /* Bit 3: Receive complete 1=data in read buffer */
                                      /* The following are only defined for UART2: */
#define UART_C1_U2IRS      0x10       /* Bit 4: UART2 transmit interrupt cause select */
#define UART_C1_U2RRM      0x20       /* Bit 5: UART2 continuous receive mode enable */
#define UART_C1_U2LCH      0x40       /* Bit 6: UART2 Data logic select */
#define UART_C1_U2ERE      0x80       /* Bit 7: UART2 Error signal output enable */

/* UART2 Transmit/Receive Control 2 */

#define UART_CON_U0IRS      0x01       /* Bit 0: UART0 transmit interrupt cause select */
#define UART_CON_U1IRS      0x02       /* Bit 1: UART1 transmit interrupt cause select */
#define UART_CON_U0RRM      0x04       /* Bit 2: UART0 continuous receive mode enable */
#define UART_CON_U1RRM      0x08       /* Bit 3: UART1 continuous receive mode enable */
#define UART_CON_CLKMD0     0x10       /* Bit 4: CLK/CLKS select bit 0 */
#define UART_CON_CLKMD1     0x20       /* Bit 5: CLK/CLKS select bit  */
#define UART_CON_RCSP       0x40       /* Bit 6: Separate CTS/RTS bit */
                                       /* Bit 7: Reserved */
/* UART2 special mode register 1 (to be provided) */

/* UART2 special mode register 2 (to be provided) */

/* UART2 special mode register 3 (to be provided) */

/* UART2 special mode register 4 (to be provided) */

/************************************************************************************
 * Global Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_SH_SRC_M16C_M16C_UART_H */
