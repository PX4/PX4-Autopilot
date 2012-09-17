/****************************************************************************
 * calypso/chip.h
 *
 *   Copyright (C) 2011 Stefan Richter. All rights reserved.
 *   Author: Stefan Richter <ichgeh@l--putt.de>
 *
 * based on: c5471/chip.h
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ****************************************************************************/

#ifndef __CALYPSO_CHIP_H
#define __CALYPSO_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* UARTs ********************************************************************/

#define UART_IRDA_BASE         0xffff5000
#define UART_MODEM_BASE        0xffff5800
#define UART_UIR               0xffff6000
#define UARTn_IO_RANGE         0x00000800

/* Common UART Registers.  Expressed as offsets from the BASE address */

#define UART_RHR_OFFS          0x00000000 /* Rcv Holding Register */
#define UART_THR_OFFS          0x00000000 /* Xmit Holding Register */
#define UART_FCR_OFFS          0x00000002 /* FIFO Control Register */
#define UART_RFCR_OFFS         0x00000002 /* Rcv FIFO Control Register */
#define UART_TFCR_OFFS         0x00000002 /* Xmit FIFO Control Register */
#define UART_SCR_OFFS          0x00000010 /* Status Control Register */
#define UART_LCR_OFFS          0x00000003 /* Line Control Register */
#define UART_LSR_OFFS          0x00000005 /* Line Status Register */
#define UART_SSR_OFFS          0x00000011 /* Supplementary Status Register */
#define UART_MCR_OFFS          0x00000004 /* Modem Control Register */
#define UART_MSR_OFFS          0x00000006 /* Modem Status Register */
#define UART_IER_OFFS          0x00000001 /* Interrupt Enable Register */
#define UART_ISR_OFFS          0x00000002 /* Interrupt Status Register */
#define UART_EFR_OFFS          0x00000002 /* Enhanced Feature Register */
#define UART_XON1_OFFS         0x00000004 /* XON1 Character Register */
#define UART_XON2_OFFS         0x00000005 /* XON2 Character Register */
#define UART_XOFF1_OFFS        0x00000006 /* XOFF1 Character Register */
#define UART_XOFF2_OFFS        0x00000007 /* XOFF2 Character Register */
#define UART_SPR_OFFS          0x00000007 /* Scratch-pad Register */
#define UART_DIV_LOW_OFFS      0x00000000 /* Divisor for baud generation */
#define UART_DIV_HIGH_OFFS     0x00000001
#define UART_TCR_OFFS          0x00000006 /* Transmission Control Register */
#define UART_TLR_OFFS          0x00000007 /* Trigger Level Register */
#define UART_MDR_OFFS          0x00000008 /* Mode Definition Register */

/* UART Settings ************************************************************/

/* Miscellaneous UART settings. */

#define UART_REGISTER_BITS   8
#define UART_IRQ_MODEM       IRQ_UART_MODEM
#define UART_IRQ_IRDA        IRQ_UART_IRDA

#define UART_RX_FIFO_NOEMPTY 0x00000001
#define UART_SSR_TXFULL      0x00000001
#define UART_LSR_TREF        0x00000020

#define UART_XMIT_FIFO_SIZE      64
#define UART_IRDA_XMIT_FIFO_SIZE 64

/* UART_LCR Register */
                                        /* Bits 31-7: Reserved */
#define UART_LCR_BOC         0x00000040 /* Bit 6: Break Control */
                                        /* Bit 5: Parity Type 2 */
#define UART_LCR_PAREVEN     0x00000010 /* Bit 4: Parity Type 1 */
#define UART_LCR_PARODD      0x00000000
#define UART_LCR_PARMARK     0x00000010
#define UART_LCR_PARSPACE    0x00000011
#define UART_LCR_PAREN       0x00000008 /* Bit 3: Paity Enable */
#define UART_LCR_PARDIS      0x00000000
#define UART_LCR_2STOP       0x00000004 /* Bit 2: Number of stop bits */
#define UART_LCR_1STOP       0x00000000
#define UART_LCR_5BITS       0x00000000 /* Bits 0-1: Word-length */
#define UART_LCR_6BITS       0x00000001
#define UART_LCR_7BITS       0x00000002
#define UART_LCR_8BITS       0x00000003

#define UART_FCR_FTL         0x000000f0
#define UART_FCR_FIFO_EN     0x00000001
#define UART_FCR_TX_CLR      0x00000002
#define UART_FCR_RX_CLR      0x00000004

#define UART_IER_RECVINT     0x00000001
#define UART_IER_XMITINT     0x00000002
#define UART_IER_LINESTSINT  0x00000004
#define UART_IER_MODEMSTSINT 0x00000008 /* IrDA UART only */
#define UART_IER_XOFFINT     0x00000020
#define UART_IER_RTSINT      0x00000040 /* IrDA UART only */
#define UART_IER_CTSINT      0x00000080 /* IrDA UART only */
#define UART_IER_INTMASK     0x000000ff

#define BAUD_115200          0x00000007
#define BAUD_57600           0x00000014
#define BAUD_38400           0x00000021
#define BAUD_19200           0x00000006
#define BAUD_9600            0x0000000C
#define BAUD_4800            0x00000018
#define BAUD_2400            0x00000030
#define BAUD_1200            0x00000060

#define MDR_UART_MODE        0x00000000  /* Both IrDA and Modem UARTs */
#define MDR_SIR_MODE         0x00000001  /* IrDA UART only */
#define MDR_AUTOBAUDING_MODE 0x00000002  /* Modem UART only */
#define MDR_RESET_MODE       0x00000007  /* Both IrDA and Modem UARTs */

/* SPI **********************************************************************/

#define MAX_SPI 3

#define SPI_REGISTER_BASE    0xffff2000

/* ARMIO ********************************************************************/
/* Timers / Watchdog ********************************************************/

#define C5471_TIMER0_CTRL    0xffff2a00
#define C5471_TIMER0_CNT     0xffff2a04
#define C5471_TIMER1_CTRL    0xffff2b00
#define C5471_TIMER1_CNT     0xffff2b04
#define C5471_TIMER2_CTRL    0xffff2c00
#define C5471_TIMER2_CNT     0xffff2c04

/* Interrupts ***************************************************************/

#define HAVE_SRC_IRQ_BIN_REG 0

#define INT_FIRST_IO         0xffff2d00
#define INT_IO_RANGE         0x5C

#define IT_REG               0xffff2d00
#define MASK_IT_REG          0xffff2d04
#define SRC_IRQ_REG          0xffff2d08
#define SRC_FIQ_REG          0xffff2d0c
#define SRC_IRQ_BIN_REG      0xffff2d10
#define INT_CTRL_REG         0xffff2d18

#define ILR_IRQ0_REG         0xffff2d1C /*  0-Timer 0       */
#define ILR_IRQ1_REG         0xffff2d20 /*  1-Timer 1       */
#define ILR_IRQ2_REG         0xffff2d24 /*  2-Timer 2       */
#define ILR_IRQ3_REG         0xffff2d28 /*  3-GPIO0         */
#define ILR_IRQ4_REG         0xffff2d2c /*  4-Ethernet      */
#define ILR_IRQ5_REG         0xffff2d30 /*  5-KBGPIO[7:0]   */
#define ILR_IRQ6_REG         0xffff2d34 /*  6-Uart serial   */
#define ILR_IRQ7_REG         0xffff2d38 /*  7-Uart IRDA     */
#define ILR_IRQ8_REG         0xffff2d3c /*  8-KBGPIO[15:8]  */
#define ILR_IRQ9_REG         0xffff2d40 /*  9-GPIO3         */
#define ILR_IRQ10_REG        0xffff2d44 /* 10-GPIO2         */
#define ILR_IRQ11_REG        0xffff2d48 /* 11-I2C           */
#define ILR_IRQ12_REG        0xffff2d4c /* 12-GPIO1         */
#define ILR_IRQ13_REG        0xffff2d50 /* 13-SPI           */
#define ILR_IRQ14_REG        0xffff2d54 /* 14-GPIO[19:4]    */
#define ILR_IRQ15_REG        0xffff2d58 /* 15-API           */

/* CLKM *********************************************************************/

#define CLKM                 0xffff2f00
#define CLKM_CTL_RST         0xffff2f10
#define CLKM_RESET           0xffff2f18

#define CLKM_RESET_EIM          0x00000008
#define CLKM_EIM_CLK_STOP       0x00000010
#define CLKM_CTL_RST_LEAD_RESET 0x00000000
#define CLKM_CTL_RST_EXT_RESET  0x00000002

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif  /* __CALYPSO_CHIP_H */
