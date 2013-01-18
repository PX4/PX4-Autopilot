/************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc17_uart.h
 *
 *   Copyright (C) 2010, 2012-2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_UART_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC17_UART_RBR_OFFSET        0x0000 /* (DLAB =0) Receiver Buffer Register (all) */
#define LPC17_UART_THR_OFFSET        0x0000 /* (DLAB =0) Transmit Holding Register (all) */
#define LPC17_UART_DLL_OFFSET        0x0000 /* (DLAB =1) Divisor Latch LSB (all) */
#define LPC17_UART_DLM_OFFSET        0x0004 /* (DLAB =1) Divisor Latch MSB (all) */
#define LPC17_UART_IER_OFFSET        0x0004 /* (DLAB =0) Interrupt Enable Register (all) */
#define LPC17_UART_IIR_OFFSET        0x0008 /* Interrupt ID Register (all) */
#define LPC17_UART_FCR_OFFSET        0x0008 /* FIFO Control Register (all) */
#define LPC17_UART_LCR_OFFSET        0x000c /* Line Control Register (all) */
#define LPC17_UART_MCR_OFFSET        0x0010 /* Modem Control Register (UART1 only) */
#define LPC17_UART_LSR_OFFSET        0x0014 /* Line Status Register (all) */
#define LPC17_UART_MSR_OFFSET        0x0018 /* Modem Status Register (UART1 only) */
#define LPC17_UART_SCR_OFFSET        0x001c /* Scratch Pad Register (all) */
#define LPC17_UART_ACR_OFFSET        0x0020 /* Auto-baud Control Register (all) */
#define LPC17_UART_ICR_OFFSET        0x0024 /* IrDA Control Register (UART0,2,3 only) */
#define LPC17_UART_FDR_OFFSET        0x0028 /* Fractional Divider Register (all) */
#define LPC17_UART_TER_OFFSET        0x0030 /* Transmit Enable Register (all) */
#define LPC17_UART_RS485CTRL_OFFSET  0x004c /* RS-485/EIA-485 Control (UART1 only) */
#define LPC17_UART_ADRMATCH_OFFSET   0x0050 /* RS-485/EIA-485 address match (UART1 only) */
#define LPC17_UART_RS485DLY_OFFSET   0x0054 /* RS-485/EIA-485 direction control delay (UART1 only) */
#define LPC17_UART_FIFOLVL_OFFSET    0x0058 /* FIFO Level register (all) */

/* Register addresses ***************************************************************/

#define LPC17_UART0_RBR              (LPC17_UART0_BASE+LPC17_UART_RBR_OFFSET)
#define LPC17_UART0_THR              (LPC17_UART0_BASE+LPC17_UART_THR_OFFSET)
#define LPC17_UART0_DLL              (LPC17_UART0_BASE+LPC17_UART_DLL_OFFSET)
#define LPC17_UART0_DLM              (LPC17_UART0_BASE+LPC17_UART_DLM_OFFSET)
#define LPC17_UART0_IER              (LPC17_UART0_BASE+LPC17_UART_IER_OFFSET)
#define LPC17_UART0_IIR              (LPC17_UART0_BASE+LPC17_UART_IIR_OFFSET)
#define LPC17_UART0_FCR              (LPC17_UART0_BASE+LPC17_UART_FCR_OFFSET)
#define LPC17_UART0_LCR              (LPC17_UART0_BASE+LPC17_UART_LCR_OFFSET)
#define LPC17_UART0_LSR              (LPC17_UART0_BASE+LPC17_UART_LSR_OFFSET)
#define LPC17_UART0_SCR              (LPC17_UART0_BASE+LPC17_UART_SCR_OFFSET)
#define LPC17_UART0_ACR              (LPC17_UART0_BASE+LPC17_UART_ACR_OFFSET)
#define LPC17_UART0_ICR              (LPC17_UART0_BASE+LPC17_UART_ICR_OFFSET)
#define LPC17_UART0_FDR              (LPC17_UART0_BASE+LPC17_UART_FDR_OFFSET)
#define LPC17_UART0_TER              (LPC17_UART0_BASE+LPC17_UART_TER_OFFSET)
#define LPC17_UART0_FIFOLVL          (LPC17_UART0_BASE+LPC17_UART_FIFOLVL_OFFSET)

#define LPC17_UART1_RBR              (LPC17_UART1_BASE+LPC17_UART_RBR_OFFSET)
#define LPC17_UART1_THR              (LPC17_UART1_BASE+LPC17_UART_THR_OFFSET)
#define LPC17_UART1_DLL              (LPC17_UART1_BASE+LPC17_UART_DLL_OFFSET)
#define LPC17_UART1_DLM              (LPC17_UART1_BASE+LPC17_UART_DLM_OFFSET)
#define LPC17_UART1_IER              (LPC17_UART1_BASE+LPC17_UART_IER_OFFSET)
#define LPC17_UART1_IIR              (LPC17_UART1_BASE+LPC17_UART_IIR_OFFSET)
#define LPC17_UART1_FCR              (LPC17_UART1_BASE+LPC17_UART_FCR_OFFSET)
#define LPC17_UART1_LCR              (LPC17_UART1_BASE+LPC17_UART_LCR_OFFSET)
#define LPC17_UART1_MCR              (LPC17_UART1_BASE+LPC17_UART_MCR_OFFSET)
#define LPC17_UART1_LSR              (LPC17_UART1_BASE+LPC17_UART_LSR_OFFSET)
#define LPC17_UART1_MSR              (LPC17_UART1_BASE+LPC17_UART_MSR_OFFSET)
#define LPC17_UART1_SCR              (LPC17_UART1_BASE+LPC17_UART_SCR_OFFSET)
#define LPC17_UART1_ACR              (LPC17_UART1_BASE+LPC17_UART_ACR_OFFSET)
#define LPC17_UART1_FDR              (LPC17_UART1_BASE+LPC17_UART_FDR_OFFSET)
#define LPC17_UART1_TER              (LPC17_UART1_BASE+LPC17_UART_TER_OFFSET)
#define LPC17_UART1_RS485CTRL        (LPC17_UART1_BASE+LPC17_UART_RS485CTRL_OFFSET)
#define LPC17_UART1_ADRMATCH         (LPC17_UART1_BASE+LPC17_UART_ADRMATCH_OFFSET)
#define LPC17_UART1_RS485DLY         (LPC17_UART1_BASE+LPC17_UART_RS485DLY_OFFSET)
#define LPC17_UART1_FIFOLVL          (LPC17_UART1_BASE+LPC17_UART_FIFOLVL_OFFSET)

#define LPC17_UART2_RBR              (LPC17_UART2_BASE+LPC17_UART_RBR_OFFSET)
#define LPC17_UART2_THR              (LPC17_UART2_BASE+LPC17_UART_THR_OFFSET)
#define LPC17_UART2_DLL              (LPC17_UART2_BASE+LPC17_UART_DLL_OFFSET)
#define LPC17_UART2_DLM              (LPC17_UART2_BASE+LPC17_UART_DLM_OFFSET)
#define LPC17_UART2_IER              (LPC17_UART2_BASE+LPC17_UART_IER_OFFSET)
#define LPC17_UART2_IIR              (LPC17_UART2_BASE+LPC17_UART_IIR_OFFSET)
#define LPC17_UART2_FCR              (LPC17_UART2_BASE+LPC17_UART_FCR_OFFSET)
#define LPC17_UART2_LCR              (LPC17_UART2_BASE+LPC17_UART_LCR_OFFSET)
#define LPC17_UART2_LSR              (LPC17_UART2_BASE+LPC17_UART_LSR_OFFSET)
#define LPC17_UART2_SCR              (LPC17_UART2_BASE+LPC17_UART_SCR_OFFSET)
#define LPC17_UART2_ACR              (LPC17_UART2_BASE+LPC17_UART_ACR_OFFSET)
#define LPC17_UART2_ICR              (LPC17_UART2_BASE+LPC17_UART_ICR_OFFSET)
#define LPC17_UART2_FDR              (LPC17_UART2_BASE+LPC17_UART_FDR_OFFSET)
#define LPC17_UART2_TER              (LPC17_UART2_BASE+LPC17_UART_TER_OFFSET)
#define LPC17_UART2_FIFOLVL          (LPC17_UART2_BASE+LPC17_UART_FIFOLVL_OFFSET)

#define LPC17_UART3_RBR              (LPC17_UART3_BASE+LPC17_UART_RBR_OFFSET)
#define LPC17_UART3_THR              (LPC17_UART3_BASE+LPC17_UART_THR_OFFSET)
#define LPC17_UART3_DLL              (LPC17_UART3_BASE+LPC17_UART_DLL_OFFSET)
#define LPC17_UART3_DLM              (LPC17_UART3_BASE+LPC17_UART_DLM_OFFSET)
#define LPC17_UART3_IER              (LPC17_UART3_BASE+LPC17_UART_IER_OFFSET)
#define LPC17_UART3_IIR              (LPC17_UART3_BASE+LPC17_UART_IIR_OFFSET)
#define LPC17_UART3_FCR              (LPC17_UART3_BASE+LPC17_UART_FCR_OFFSET)
#define LPC17_UART3_LCR              (LPC17_UART3_BASE+LPC17_UART_LCR_OFFSET)
#define LPC17_UART3_LSR              (LPC17_UART3_BASE+LPC17_UART_LSR_OFFSET)
#define LPC17_UART3_SCR              (LPC17_UART3_BASE+LPC17_UART_SCR_OFFSET)
#define LPC17_UART3_ACR              (LPC17_UART3_BASE+LPC17_UART_ACR_OFFSET)
#define LPC17_UART3_ICR              (LPC17_UART3_BASE+LPC17_UART_ICR_OFFSET)
#define LPC17_UART3_FDR              (LPC17_UART3_BASE+LPC17_UART_FDR_OFFSET)
#define LPC17_UART3_TER              (LPC17_UART3_BASE+LPC17_UART_TER_OFFSET)
#define LPC17_UART3_FIFOLVL          (LPC17_UART3_BASE+LPC17_UART_FIFOLVL_OFFSET)

/* Register bit definitions *********************************************************/

/* RBR (DLAB =0) Receiver Buffer Register (all) */

#define UART_RBR_MASK                (0xff)    /* Bits 0-7: Oldest received byte in RX FIFO */
                                               /* Bits 8-31: Reserved */

/* THR (DLAB =0) Transmit Holding Register (all) */

#define UART_THR_MASK                (0xff)    /* Bits 0-7: Adds byte to TX FIFO */
                                               /* Bits 8-31: Reserved */

/* DLL (DLAB =1) Divisor Latch LSB (all) */

#define UART_DLL_MASK                (0xff)    /* Bits 0-7: DLL */
                                               /* Bits 8-31: Reserved */

/* DLM (DLAB =1) Divisor Latch MSB (all) */

#define UART_DLM_MASK                (0xff)    /* Bits 0-7: DLM */
                                               /* Bits 8-31: Reserved */

/* IER (DLAB =0) Interrupt Enable Register (all) */

#define UART_IER_RBRIE               (1 << 0)  /* Bit 0: RBR Interrupt Enable */
#define UART_IER_THREIE              (1 << 1)  /* Bit 1: THRE Interrupt Enable */
#define UART_IER_RLSIE               (1 << 2)  /* Bit 2: RX Line Status Interrupt Enable */
#define UART_IER_MSIE                (1 << 3)  /* Bit 3: Modem Status Interrupt Enable (UART1 only) */
                                               /* Bits 4-6: Reserved */
#define UART_IER_CTSIE               (1 << 7)  /* Bit 7: CTS transition interrupt (UART1 only) */
#define UART_IER_ABEOIE              (1 << 8)  /* Bit 8: Enables the end of auto-baud interrupt */
#define UART_IER_ABTOIE              (1 << 9)  /* Bit 9: Enables the auto-baud time-out interrupt */
                                               /* Bits 10-31: Reserved */
#define UART_IER_ALLIE               (0x038f)

/* IIR Interrupt ID Register (all) */

#define UART_IIR_INTSTATUS           (1 << 0)  /* Bit 0:  Interrupt status (active low) */
#define UART_IIR_INTID_SHIFT         (1)       /* Bits 1-3: Interrupt identification */
#define UART_IIR_INTID_MASK          (7 << UART_IIR_INTID_SHIFT)
#  define UART_IIR_INTID_MSI         (0 << UART_IIR_INTID_SHIFT) /* Modem Status (UART1 only) */
#  define UART_IIR_INTID_THRE        (1 << UART_IIR_INTID_SHIFT) /* THRE Interrupt */
#  define UART_IIR_INTID_RDA         (2 << UART_IIR_INTID_SHIFT) /* 2a - Receive Data Available (RDA) */
#  define UART_IIR_INTID_RLS         (3 << UART_IIR_INTID_SHIFT) /* 1 - Receive Line Status (RLS) */
#  define UART_IIR_INTID_CTI         (6 << UART_IIR_INTID_SHIFT) /* 2b - Character Time-out Indicator (CTI) */
                                               /* Bits 4-5: Reserved */
#define UART_IIR_FIFOEN_SHIFT        (6)       /* Bits 6-7: Copies of FCR bit 0 */
#define UART_IIR_FIFOEN_MASK         (3 << UART_IIR_FIFOEN_SHIFT)
#define UART_IIR_ABEOINT             (1 << 8)  /* Bit 8:  End of auto-baud interrupt */
#define UART_IIR_ABTOINT             (1 << 9)  /* Bit 9:  Auto-baud time-out interrupt */
                                               /* Bits 10-31: Reserved */
/* FCR FIFO Control Register (all) */

#define UART_FCR_FIFOEN              (1 << 0)  /* Bit 0:  Enable FIFOs */
#define UART_FCR_RXRST               (1 << 1)  /* Bit 1:  RX FIFO Reset */
#define UART_FCR_TXRST               (1 << 2)  /* Bit 2:  TX FIFO Reset */
#define UART_FCR_DMAMODE             (1 << 3)  /* Bit 3:  DMA Mode Select */
                                               /* Bits 4-5: Reserved */
#define UART_FCR_RXTRIGGER_SHIFT     (6)       /* Bits 6-7: RX Trigger Level */
#define UART_FCR_RXTRIGGER_MASK      (3 << UART_FCR_RXTRIGGER_SHIFT)
#  define UART_FCR_RXTRIGGER_0       (0 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 0 (1 character) */
#  define UART_FCR_RXTRIGGER_4       (1 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 1 (4 characters) */
#  define UART_FCR_RXTRIGGER_8       (2 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 2 (8 characters) */
#  define UART_FCR_RXTRIGGER_14      (3 << UART_FCR_RXTRIGGER_SHIFT) /* Trigger level 3 (14 characters) */
                                               /* Bits 8-31: Reserved */
/* LCR Line Control Register (all) */

#define UART_LCR_WLS_SHIFT           (0)       /* Bit 0-1: Word Length Select */
#define UART_LCR_WLS_MASK            (3 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_5BIT          (0 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_6BIT          (1 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_7BIT          (2 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_8BIT          (3 << UART_LCR_WLS_SHIFT)
#define UART_LCR_STOP                (1 << 2)  /* Bit 2:  Stop Bit Select */
#define UART_LCR_PE                  (1 << 3)  /* Bit 3:  Parity Enable */
#define UART_LCR_PS_SHIFT            (4)       /* Bits 4-5: Parity Select */
#define UART_LCR_PS_MASK             (3 << UART_LCR_PS_SHIFT)
#  define UART_LCR_PS_ODD            (0 << UART_LCR_PS_SHIFT) /* Odd parity */
#  define UART_LCR_PS_EVEN           (1 << UART_LCR_PS_SHIFT) /* Even Parity */
#  define UART_LCR_PS_STICK1         (2 << UART_LCR_PS_SHIFT) /* Forced "1" stick parity */
#  define UART_LCR_PS_STICK0         (3 << UART_LCR_PS_SHIFT) /* Forced "0" stick parity */
#define UART_LCR_BRK                 (1 << 6)  /* Bit 6: Break Control */
#define UART_LCR_DLAB                (1 << 7)  /* Bit 7: Divisor Latch Access Bit (DLAB) */
                                               /* Bits 8-31: Reserved */
/* MCR Modem Control Register (UART1 only) */

#define UART_MCR_DTR                 (1 << 0)  /* Bit 0:  DTR Control Source for DTR output */
#define UART_MCR_RTS                 (1 << 1)  /* Bit 1:  Control Source for  RTS output */
                                               /* Bits 2-3: Reserved */
#define UART_MCR_LPBK                (1 << 4)  /* Bit 4:  Loopback Mode Select */
                                               /* Bit 5:  Reserved */
#define UART_MCR_RTSEN               (1 << 6)  /* Bit 6:  Enable auto-rts flow control */
#define UART_MCR_CTSEN               (1 << 7)  /* Bit 7:  Enable auto-cts flow control */
                                               /* Bits 8-31: Reserved */
/* LSR Line Status Register (all) */

#define UART_LSR_RDR                 (1 << 0)  /* Bit 0:  Receiver Data Ready */
#define UART_LSR_OE                  (1 << 1)  /* Bit 1:  Overrun Error */
#define UART_LSR_PE                  (1 << 2)  /* Bit 2:  Parity Error */
#define UART_LSR_FE                  (1 << 3)  /* Bit 3:  Framing Error */
#define UART_LSR_BI                  (1 << 4)  /* Bit 4:  Break Interrupt */
#define UART_LSR_THRE                (1 << 5)  /* Bit 5:  Transmitter Holding Register Empty */
#define UART_LSR_TEMT                (1 << 6)  /* Bit 6:  Transmitter Empty */
#define UART_LSR_RXFE                (1 << 7)  /* Bit 7:  Error in RX FIFO (RXFE) */
                                               /* Bits 8-31: Reserved */
/* MSR Modem Status Register (UART1 only) */

#define UART_MSR_DELTACTS            (1 << 0)  /* Bit 0:  CTS state change */
#define UART_MSR_DELTADSR            (1 << 1)  /* Bit 1:  DSR state change */
#define UART_MSR_RIEDGE              (1 << 2)  /* Bit 2:  RI ow to high transition */
#define UART_MSR_DELTADCD            (1 << 3)  /* Bit 3:  DCD state change */
#define UART_MSR_CTS                 (1 << 4)  /* Bit 4:  CTS State */
#define UART_MSR_DSR                 (1 << 5)  /* Bit 5:  DSR State */
#define UART_MSR_RI                  (1 << 6)  /* Bit 6:  Ring Indicator State */
#define UART_MSR_DCD                 (1 << 7)  /* Bit 7:  Data Carrier Detect State */
                                               /* Bits 8-31: Reserved */
/* SCR Scratch Pad Register (all) */

#define UART_SCR_MASK                (0xff)    /* Bits 0-7: SCR data */
                                               /* Bits 8-31: Reserved */
/* ACR Auto-baud Control Register (all) */

#define UART_ACR_START               (1 << 0)  /* Bit 0:  Auto-baud start/running*/
#define UART_ACR_MODE                (1 << 1)  /* Bit 1:  Auto-baud mode select*/
#define UART_ACR_AUTORESTART         (1 << 2)  /* Bit 2:  Restart in case of time-out*/
                                               /* Bits 3-7: Reserved */
#define UART_ACR_ABEOINTCLR          (1 << 8)  /* Bit 8:  End of auto-baud interrupt clear */
#define UART_ACR_ABTOINTCLRT         (1 << 9)  /* Bit 9:  Auto-baud time-out interrupt clear */
                                               /* Bits 10-31: Reserved */
/* ICA IrDA Control Register (UART0,2,3 only) */

#define UART_ICR_IRDAEN              (1 << 0)  /* Bit 0:  Enable IrDA mode */
#define UART_ICR_IRDAINV             (1 << 1)  /* Bit 1:  Invert serial input */
#define UART_ICR_FIXPULSEEN          (1 << 2)  /* Bit 2:  Enable IrDA fixed pulse width mode */
#define UART_ICR_PULSEDIV_SHIFT      (3)       /* Bits 3-5: Configures the pulse when FixPulseEn = 1 */
#define UART_ICR_PULSEDIV_MASK       (7 << UART_ICR_PULSEDIV_SHIFT)
#  define UART_ICR_PULSEDIV_2TPCLK   (0 << UART_ICR_PULSEDIV_SHIFT) /* 2 x TPCLK */
#  define UART_ICR_PULSEDIV_4TPCLK   (1 << UART_ICR_PULSEDIV_SHIFT) /* 4 x TPCLK */
#  define UART_ICR_PULSEDIV_8TPCLK   (2 << UART_ICR_PULSEDIV_SHIFT) /* 8 x TPCLK */
#  define UART_ICR_PULSEDIV_16TPCLK  (3 << UART_ICR_PULSEDIV_SHIFT) /* 16 x TPCLK */
#  define UART_ICR_PULSEDIV_32TPCLK  (4 << UART_ICR_PULSEDIV_SHIFT) /* 32 x TPCLK */
#  define UART_ICR_PULSEDIV_64TPCLK  (5 << UART_ICR_PULSEDIV_SHIFT) /* 64 x TPCLK */
#  define UART_ICR_PULSEDIV_128TPCLK (6 << UART_ICR_PULSEDIV_SHIFT) /* 128 x TPCLK */
#  define UART_ICR_PULSEDIV_256TPCLK (7 << UART_ICR_PULSEDIV_SHIFT) /* 246 x TPCLK */
                                               /* Bits 6-31: Reserved */
/* FDR Fractional Divider Register (all) */

#define UART_FDR_DIVADDVAL_SHIFT     (0)       /* Bits 0-3: Baud-rate generation pre-scaler divisor value */
#define UART_FDR_DIVADDVAL_MASK      (15 << UART_FDR_DIVADDVAL_SHIFT)
#define UART_FDR_MULVAL_SHIFT        (3)       /* Bits 4-7 Baud-rate pre-scaler multiplier value */
#define UART_FDR_MULVAL_MASK         (15 << UART_FDR_MULVAL_SHIFT)
                                               /* Bits 8-31: Reserved */
/* TER Transmit Enable Register (all) */
                                               /* Bits 0-6: Reserved */
#define UART_TER_TXEN                (1 << 7)  /* Bit 7:  TX Enable */
                                               /* Bits 8-31: Reserved */
/* RS-485/EIA-485 Control (UART1 only) */

#define UART_RS485CTRL_NMMEN         (1 << 0)  /* Bit 0: RS-485/EIA-485 Normal Multidrop Mode (NMM) enabled */
#define UART_RS485CTRL_RXDIS         (1 << 1)  /* Bit 1: Receiver is disabled */
#define UART_RS485CTRL_AADEN         (1 << 2)  /* Bit 2: Auto Address Detect (AAD) is enabled */
#define UART_RS485CTRL_SEL           (1 << 3)  /* Bit 3: RTS/DTR used for direction control (DCTRL=1) */
#define UART_RS485CTRL_DCTRL         (1 << 4)  /* Bit 4: Enable Auto Direction Control */
#define UART_RS485CTRL_OINV          (1 << 5)  /* Bit 5: Polarity of the direction control signal on RTS/DTR */
                                               /* Bits 6-31: Reserved */
/* RS-485/EIA-485 address match (UART1 only) */

#define UART_ADRMATCH_MASK           (0xff)    /* Bits 0-7: Address match value */
                                               /* Bits 8-31: Reserved */
/* RS-485/EIA-485 direction control delay (UART1 only) */

#define UART_RS485DLY_MASK           (0xff)    /* Bits 0-7: Direction control (RTS/DTR) delay */
                                               /* Bits 8-31: Reserved */
/* FIFOLVL FIFO Level register (all) */

#define UART_FIFOLVL_RX_SHIFT        (0)       /* Bits 0-3: Current level of the UART RX FIFO */
#define UART_FIFOLVL_RX_MASK         (15 << UART_FIFOLVL_RX_SHIFT)
                                               /* Bits 4-7: Reserved */
#define UART_FIFOLVL_TX_SHIFT        (8)       /* Bits 8-11: Current level of the UART TX FIFO */
#define UART_FIFOLVL_TX_MASK         (15 << UART_FIFOLVL_TX_SHIFT)
                                               /* Bits 12-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_UART_H */
