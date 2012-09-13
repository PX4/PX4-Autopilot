/************************************************************************************
 * dm320/dm320_uart.h
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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

#ifndef __DM320_UART_H
#define __DM320_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* UART definitions *****************************************************************/

/* UART Registers (offsets from the register base) */

#define UART_DTRR              0    /* Data Transmission/Reception Register */
#define UART_BRSR              2    /* Bit Rate Set Register */
#define UART_MSR               4    /* Mode Set Register */
#define UART_RFCR              6    /* Reception FIFO Control Register */
#define UART_TFCR              8    /* Transmission FIFO Control Register */
#define UART_LCR              10    /* Line Control Register */
#define UART_SR               12    /* Status Register */

/* UART DTRR register bit definitions */

#define UART_DTRR_RVF         0x1000 /* Receive word valid flag */
#define UART_DTRR_BF          0x0800 /* Break flag */
#define UART_DTRR_FE          0x0400 /* Framing error */
#define UART_DTRR_ORF         0x0200 /* Overrun flag */
#define UART_DTRR_PEF         0x0100 /* Parity error */
#define UART_DTRR_DTR_MASK    0x00ff /* Data transmit/receive */

/* UART BRSR register bit definitions */
/* The UART module is clocked by either the AHB clock or PLLIN / 16 */

#ifdef CONFIG_DM320_UARTPPLIN
#  define UART_REFCLK (27000000 / 16)
#else
#  define UART_REFCLK (DM320_AHB_CLOCK / 16)
#endif

/* And baud = UART_REFCLK / (brsr+1) */

#define UART_BAUD_2400        ((uint16_t)((UART_REFCLK / 2400  ) - 1))
#define UART_BAUD_4800        ((uint16_t)((UART_REFCLK / 4800  ) - 1))
#define UART_BAUD_9600        ((uint16_t)((UART_REFCLK / 9600  ) - 1))
#define UART_BAUD_14400       ((uint16_t)((UART_REFCLK / 14400 ) - 1))
#define UART_BAUD_19200       ((uint16_t)((UART_REFCLK / 19200 ) - 1))
#define UART_BAUD_28800       ((uint16_t)((UART_REFCLK / 28800 ) - 1))
#define UART_BAUD_38400       ((uint16_t)((UART_REFCLK / 38400 ) - 1))
#define UART_BAUD_57600       ((uint16_t)((UART_REFCLK / 57600 ) - 1))
#define UART_BAUD_115200      ((uint16_t)((UART_REFCLK / 115200) - 1))
#define UART_BAUD_230400      ((uint16_t)((UART_REFCLK / 230400) - 1))
#define UART_BAUD_460800      ((uint16_t)((UART_REFCLK / 460800) - 1))
#define UART_BAUD_921600      ((uint16_t)((UART_REFCLK / 921600) - 1))

/* UART MSR register bit definitions */

#define UART_MSR_MODE_BITS    0x001f /* Aata length, stop, & parity */
#define UART_MSR_CLS          0x0001 /* Char length (1=7bit, 0=8bit) */
#define UART_DATABIT_7        0x0001 /*  Data bit = 7bit */
#define UART_DATABIT_8        0x0000 /*  Data bit = 8bit */
#define UART_MSR_SBLS         0x0004 /* Stop bit length selection */
#define UART_STOPBIT_1        0x0000 /*  Stop bit = 1bit */
#define UART_STOPBIT_2        0x0004 /*  Stop bit = 2bit */
#define UART_MSR_PSB          0x0008 /* Parity selection bit */
#define UART_MSR_PEB          0x0010 /* Parity enable bit */
#define UART_NOPARITY         0x0000 /*   No-parity */
#define UART_ODDPARITY        0x0018 /*   Odd parity */
#define UART_EVENPARITY       0x0010 /*   Even parity */
#define UART_MSR_RTSC         0x0020 /* RTS receive FIFO control */
#define UART_MSR_CSTC         0x0040 /* CTS send control */
#define UART_MSR_TOIC_MASK    0x0c00 /* Timeout interrupt control */
#define UART_MSR_TOIC_DIS     0x0000 /*   Disabled */
#define UART_MSR_TOIC_3       0x0400 /*   3 bytes */
#define UART_MSR_TOIC_7       0x0800 /*   7 bytes */
#define UART_MSR_TOIC_15      0x0c00 /*  15 bytes */
#define UART_MSR_ALLIE        0xfc00 /* All interrupt bits */
#define UART_MSR_LSIE         0x1000 /*  Line status change int. enable */
#define UART_MSR_REIE         0x2000 /*  Receive error interrupt enable */
#define UART_MSR_TFTIE        0x4000 /*  Transmit FIFO trigger int. enable */
#define UART_MSR_RFTIE        0x8000 /*  Receive FIFO trigger int. enable */

#define  UART_MSR_INIT (UART_NOPARITY | UART_STOPBIT_1 | UART_DATABIT_8)

/* UART RFCR register bit definitions */

#define UART_RFCR_RWC_MASK    0x003f /* Receive byte count */
#define UART_RFCR_RTL_MASK    0x0700 /* Receive trigger level */
#define UART_RFCR_RTL_1       0x0000 /*   1 byte */
#define UART_RFCR_RTL_4       0x0100 /*   4 bytes */
#define UART_RFCR_RTL_8       0x0200 /*   8 bytes */
#define UART_RFCR_RTL_16      0x0300 /*  16 bytes */
#define UART_RFCR_RTL_24      0x0400 /*  24 bytes */
#define UART_RFCR_RTL_32      0x0500 /*  32 bytes */
#define UART_RFCR_RDEF        0x4000 /* Receive data error flag */
#define UART_RFCR_RFCB        0x8000 /* Receive FIFO clear bit */

/* UART TFCR register bit definitions */

#define UART_TFCR_TWC_MASK    0x003f /* Transmit byte count */
#define UART_TFCR_TTL_MASK    0x0700 /* Transmit trigger level */
#define UART_TFCR_TTL_1       0x0000 /*   1 byte */
#define UART_TFCR_TTL_4       0x0100 /*   4 bytes */
#define UART_TFCR_TTL_8       0x0200 /*   8 bytes */
#define UART_TFCR_TTL_16      0x0300 /*  16 bytes */
#define UART_TFCR_TTL_24      0x0400 /*  24 bytes */
#define UART_TFCR_TTL_32      0x0500 /*  32 bytes */
#define UART_TFCR_TFCB        0x8000 /* Transmit FIFO clear bit */

/* UART LCR register bit definitions */

#define UART_LCR_RTS          0x0004 /* Current RTS value */
#define UART_LCR_CTS          0x0010 /* Current CTS value */
#define UART_LCR_DSR          0x0080 /* Current DSR value */
#define UART_LCR_BOC          0x0100 /* Break output control */
#define UART_LCR_UTST         0x4000 /* Test mode setting */

#define UART_LCR_INIT         0x0000

/* UART SR register bit definitions */

#define UART_SR_TREF          0x0001 /* Transmit register empty flag */
#define UART_SR_TFEF          0x0002 /* Transmit FIFO empty flag */
#define UART_SR_RFNEF         0x0004 /* Receive FIFO not empty flag */
#define UART_SR_TOIF          0x0100 /* Timeout Interrupt flag */
#define UART_SR_RFER          0x0200 /* Receive data error flag */
#define UART_SR_TFTI          0x0400 /* Transmit FIFO trigger level */
#define UART_SR_RFTI          0x0800 /* Receive FIFO trigger level */
#define UART_SR_CTSS          0x1000 /* CTS status */
#define UART_SR_DSRS          0x8000 /* DSR status */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __DM320_UART_H */
