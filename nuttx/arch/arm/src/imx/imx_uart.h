/************************************************************************************
 * arch/arm/src/imx/imx_uart.h
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

#ifndef __ARCH_ARM_IMX_UART_H
#define __ARCH_ARM_IMX_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
 
/************************************************************************************
 * Definitions
 ************************************************************************************/

/* UART Register Offsets ************************************************************/

#define UART_RXD0             0x0000 /* UART receiver register 0 */
#define UART_RXD1             0x0004 /* UART receiver register 1 */
#define UART_RXD2             0x0008 /* UART receiver register 2 */
#define UART_RXD3             0x000c /* UART receiver register 3 */
#define UART_TXD0             0x0040 /* UART receiver register 0 */
#define UART_TXD1             0x0044 /* UART receiver register 1 */
#define UART_TXD2             0x0048 /* UART receiver register 2 */
#define UART_TXD3             0x004c /* UART receiver register 3 */
#define UART_UCR1             0x0080 /* UART control register 1 */
#define UART_UCR2             0x0084 /* UART control register 2 */
#define UART_UCR3             0x0088 /* UART control register 3 */
#define UART_UCR4             0x008c /* UART control register 4 */
#define UART_UFCR             0x0090 /* UART FIFO control register */
#define UART_USR1             0x0094 /* UART status register 1 */
#define UART_USR2             0x0098 /* UART status register 2 */
#define UART_UESC             0x009c /* UART escape character register */
#define UART_UTIM             0x00a0 /* UART escape timer register */
#define UART_UBIR             0x00a4 /* UART BRM incremental register */
#define UART_UBMR             0x00a8 /* UART BRM modulator register */
#define UART_UBRC             0x00ac /* UART baud rate counter register */
#define UART_BIPR1            0x00b0 /* UART BRM incremental preset register 1 */
#define UART_BIPR2            0x00b4 /* UART BRM incremental preset register 2 */
#define UART_BIPR3            0x00b8 /* UART BRM incremental preset register 3 */
#define UART_BIPR4            0x00bc /* UART BRM incremental preset register 4 */
#define UART_BMPR1            0x00c0 /* UART BRM modulator preset register 1 */
#define UART_BMPR2            0x00c4 /* UART BRM modulator preset register 2 */
#define UART_BMPR3            0x00c8 /* UART BRM modulator preset register 3 */
#define UART_BMPR4            0x00cc /* UART BRM modulator preset register 4 */
#define UART_UTS              0x00d0 /* UART test register */

/* UART Register Bit Definitions ****************************************************/

/* UART Receiver Register */

#define UART_RXD_DATA_SHIFT   0         /* Bits 0-7: Received Data */
#define UART_RXD_DATA_MASK    (0xff << UART_RXD_DATA_SHIFT)
#define UART_RXD_PRERR        (1 << 10) /* Bit 10: Parity Error */
#define UART_RXD_BRK          (1 << 11) /* Bit 11: Break Detect */
#define UART_RXD_FRMERR       (1 << 12) /* Bit 12: Frame Error */
#define UART_RXD_OVRRUN       (1 << 13) /* Bit 13: Receiver Overrun */
#define UART_RXD_ERR          (1 << 14) /* Bit 14: Error Detect */
#define UART_RXD_CHARRDY      (1 << 15) /* Bit 15: Character Ready */

/* UART Transmitter Register */

#define UART_TXDATA_SHIFT     0 /* Bits 0-7: Transmit Data */
#define UART_TXDATA_MASK      (0xff << UART_UCR4_TXDATA_SHIFT)

/* UART Control Register 1 */

#define UART_UCR1_UARTEN      (1 << 0)  /* Bit 0: Enable/disable uart */
#define UART_UCR1_DOZE        (1 << 1)  /* Bit 1: UART Doze enable */
#define UART_UCR1_UARTCLEN    (1 << 2)  /* Bit 2: UART clock enable */
#define UART_UCR1_TDMAEN      (1 << 3)  /* Bit 3: Transmitter ready data enable */
#define UART_UCR1_SNDBRK      (1 << 4)  /* Bit 4: Send BREAK */
#define UART_UCR1_RTSDEN      (1 << 5)  /* Bit 5: RTS Delta interrupt enable */
#define UART_UCR1_TXEMPTYEN   (1 << 6)  /* Bit 6: Transmitter empty interrupt enable */
#define UART_UCR1_IREN        (1 << 7)  /* Bit 7: Infrared Interface enable */
#define UART_UCR1_RDMAEN      (1 << 8)  /* Bit 8: Receive ready DMA enable */
#define UART_UCR1_RRDYEN      (1 << 9)  /* Bit 9: Receiver ready interrupt enable */
#define UART_UCR1_ICD_SHIFT   10        /* Bit 10-11: Idle condition detect */
#define UART_UCR1_ICD_MASK    (0x03 << UART_UCR1_ICD_SHIFT)
#define UART_UCR1_IDEN        (1 << 12) /* Bit 12: Idle condition detected interrupt enable */
#define UART_UCR1_TRDYEN      (1 << 13) /* Bit 13: Transmitter ready interrupt enable */
#define UART_UCR1_ADBR        (1 << 14) /* Bit 14: Automatic detection of baud rate */
#define UART_UCR1_ADEN        (1 << 15) /* Bit 15: Automatic baud rate detection interrupt enable */

/* UART Control Register 2 */

#define UART_UCR2_SRST        (1 << 0)  /* Bit 0: Software reset */
#define UART_UCR2_RXEN        (1 << 1)  /* Bit 1: Receiver enable */
#define UART_UCR2_TXEN        (1 << 2)  /* Bit 2: Transmitter enable */
#define UART_UCR2_RTSEN       (1 << 4)  /* Bit 4: RTS interrupt enable/disable */
#define UART_UCR2_WS          (1 << 5)  /* Bit 5: Word size */
#define UART_UCR2_STPB        (1 << 6)  /* Bit 6: Controls number of stop bits */
#define UART_UCR2_PROE        (1 << 7)  /* Bit 7: Parity Odd/Even */
#define UART_UCR2_PREN        (1 << 8)  /* Bit 8: Parity enable */
#define UART_UCR2_RTEC_SHIFT  9         /* Bit 9-10: Request to send edge control */
#define UART_UCR2_RTEC_MASK   (0x03 << UART_UCR2_RTEC_SHIFT)
#define UART_UCR2_ESCEN       (1 << 11) /* Bit 11: Escape enable */
#define UART_UCR2_CTS         (1 << 12) /* Bit 12: Clear To Send pin */
#define UART_UCR2_CTSC        (1 << 13) /* Bit 13: CTS Pin control */
#define UART_UCR2_IRTS        (1 << 14) /* Bit 14: Ignore RTS Pin */
#define UART_UCR2_ESCI        (1 << 15) /* Bit 15: Escape Sequence Interrupt Enable */

/* UART1 Control Register 3 */

#define UART1_UCR3_BPEN       (1 << 0)  /* Bit 0: Preset Registers Enable */
#define UART1_UCR3_INVT       (1 << 1)  /* Bit 1: Inverted Infrared Transmission */
#define UART1_UCR3_REF30      (1 << 2)  /* Bit 2: Reference frequency 30 mhz */
#define UART1_UCR3_REF25      (1 << 3)  /* Bit 3: Reference frequency 25 mhz */
#define UART1_UCR3_AWAKEN     (1 << 4)  /* Bit 4: Asynchronous wake interrupt enable */
#define UART1_UCR3_AIRINTEN   (1 << 5)  /* Bit 5: Asynchronous IR Wake interrupt enable */
#define UART1_UCR3_RXDSEN     (1 << 6)  /* Bit 6: Receive status interrupt enable */
#define UART1_UCR3_FRAERREN   (1 << 11) /* Bit 11: Frame error interrupt enable */
#define UART1_UCR3_PARERREN   (1 << 12) /* Bit 12: Parity error interrupt enable */

/* UART2/3 Control Register 4 */

#define UART2_UCR3_BPEN       (1 << 0)  /* Bit 0: Preset Registers Enable */
#define UART2_UCR3_INVT       (1 << 1)  /* Bit 1: Inverted Infrared Transmission */
#define UART2_UCR3_REF30      (1 << 2)  /* Bit 2: Reference frequency 30 mhz */
#define UART2_UCR3_REF25      (1 << 3)  /* Bit 3: Reference frequency 25 mhz */
#define UART2_UCR3_AWAKEN     (1 << 4)  /* Bit 4: Asychronous WAKE Interrupt Enable */
#define UART2_UCR3_AIRINTEN   (1 << 5)  /* Bit 5: Asychronous IR WAKE Interrupt Enable */
#define UART2_UCR3_RXDSEN     (1 << 6)  /* Bit 6: Receive Status Interrupt Enable */
#define UART2_UCR3_RI         (1 << 7)  /* Bit 7: Ring Indicator */
#define UART2_UCR3_Reserved2  (1 << 8)  /* Bit 8: Reserved */
#define UART2_UCR3_DCD        (1 << 9)  /* Bit 9: Data Carrier Detect */
#define UART2_UCR3_DSR        (1 << 10) /* Bit 10: Data Set Ready */
#define UART2_UCR3_FRAERREN   (1 << 11) /* Bit 11: Frame Error Interrupt Enable */
#define UART2_UCR3_PARERREN   (1 << 12) /* Bit 12: Parity Error Interrupt Enable */
#define UART2_UCR3_DTREN      (1 << 13) /* Bit 13: Data Terminal Ready Interrupt Enable */
#define UART2_UCR3_DPEC_SHIFT 14        /* Bit 14-15: DTR Interrupt Edge Control */
#define UART2_UCR3_DPEC_MASK  (0x03 << UART_UCR4_DPEC_SHIFT)

/* UART Control Register 4 */

#define UART_UCR4_DREN        (1 << 0)  /* Bit 0: Receive data ready interrupt enable */
#define UART_UCR4_OREN        (1 << 1)  /* Bit 1: Receiver overrun interrupt enable */
#define UART_UCR4_BKEN        (1 << 2)  /* Bit 2: Break condition detected interrupt enable */
#define UART_UCR4_TCEN        (1 << 3)  /* Bit 3: Transmit complete interrupt enable */
#define UART_UCR4_IRSC        (1 << 5)  /* Bit 5: IR special case */
#define UART_UCR4_REF16       (1 << 6)  /* Bit 6: Reference Frequency 16 mhz */
#define UART_UCR4_WKEN        (1 << 7)  /* Bit 7: Wake interrupt enable */
#define UART_UCR4_ENIRI       (1 << 8)  /* Bit 8: Serial infrared interrupt enable */
#define UART_UCR4_INVR        (1 << 9)  /* Bit 9: Inverted infrared reception */
#define UART_UCR4_CTSTL_SHIFT 10        /* Bits 10-15: CTS trigger level */
#define UART_UCR4_CTSTL_MASK  (0x3f << UART_UCR4_CTSTL_SHIFT)

/* UART FIFO Control Register */

#define UART_UFCR_RXTL_SHIFT  0         /* Bits 0-6: Receiver Trigger Level */
#define UART_UFCR_RXTL_MASK   (0x3f << UART_UFCR_RXTL_SHIFT)
#define UART_UFCR_RFDIV_SHIFT 7         /* Bits 7-9: Reference Frequency Divider */
#define UART_UFCR_RFDIV_MASK  (0x07 << UART_UFCR_RFDIV_SHIFT)
#define UART_UFCR_TXTL_SHIFT  10        /* Bits 10-15: Transmitter Trigger Level */
#define UART_UFCR_TXTL_MASK   (0x3f << UART_UFCR_TXTL_SHIFT)

/* UART Status 1 Register  */

#define UART_USR1_AWAKE       (1 << 4)  /* Bit 4: Asynchronous WAKE Interrupt Flag */
#define UART_USR1_AIRINT      (1 << 5)  /* Bit 5: Asynchronous IR WAKE Interrupt Flag */
#define UART_USR1_RXDS        (1 << 6)  /* Bit 6: Receiver IDLE Interrupt Flag */
#define UART_USR1_RRDY        (1 << 9)  /* Bit 9: RX Ready Interrupt/DMA Flag */
#define UART_USR1_FRAMERR     (1 << 10) /* Bit 10: Frame Error Interrupt Flag */
#define UART_USR1_ESCF        (1 << 11) /* Bit 11: Escape Sequence Interrupt Flag */
#define UART_USR1_RTSD        (1 << 12) /* Bit 12: RTS Delta */
#define UART_USR1_TRDY        (1 << 13) /* Bit 13: TX Ready Interrupt/DMA Flag */
#define UART_USR1_RTSS        (1 << 14) /* Bit 14: RTS Pin Status */
#define UART_USR1_PARITYERR   (1 << 15) /* Bit 15: Parity Error Interrupt Flag */

/* UART Status 2 Register */

#define UART_USR2_RDR         (1 << 0)  /* Bit 0: Receive data ready  */
#define UART_USR2_ORE         (1 << 1)  /* Bit 1: Overrun error  */
#define UART_USR2_BRCD        (1 << 2)  /* Bit 2: Break condition detected */
#define UART_USR2_TXDC        (1 << 3)  /* Bit 3: Transmitter complete */
#define UART_USR2_RTSF        (1 << 4)  /* Bit 4: RTS Edge Triggered Interrupt flag */
#define UART_USR2_WAKE        (1 << 7)  /* Bit 7: Wake */
#define UART_USR2_IRINT       (1 << 8)  /* Bit 8: Serial infrared interrupt flag */
#define UART_USR2_IDLE        (1 << 12) /* Bit 12: Idle condition */
#define UART_USR2_DTRF        (1 << 13) /* Bit 13: DTR edge triggered interrupt flag */
#define UART_USR2_TXFE        (1 << 14) /* Bit 14: Transmit Buffer FIFO empty */
#define UART_USR2_ADET        (1 << 15) /* Bit 15: Automatic baud rate detection complete */

/* UART Test Register */

#define UART_UTS_TXFULL       (1 << 4)  /* Bit 4: TxFIFO FULL */
#define UART_UTS_RXEMPTY      (1 << 5)  /* Bit 5: RxFIFO Empty */
#define UART_UTS_TXEMPTY      (1 << 6)  /* Bit 6: TxFIFO */
#define UART_UTS_LOOP         (1 << 12) /* Bit 12: Loop TX and RX for Test */
#define UART_UTS_FRCPERR      (1 << 13) /* Bit 13: Force Parity Error */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_IMX_UART_H */
