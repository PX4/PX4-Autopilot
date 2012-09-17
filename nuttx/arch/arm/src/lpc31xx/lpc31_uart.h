/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_uart.h
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_UART_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_UART_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* UART register base address offset into the APB2 domain ***************************************/

#define LPC31_UART_VBASE                (LPC31_APB2_VSECTION+LPC31_APB2_UART_OFFSET)
#define LPC31_UART_PBASE                (LPC31_APB2_PSECTION+LPC31_APB2_UART_OFFSET)

/* UART register offsets (with respect to the UART base) ****************************************/

#define LPC31_UART_RBR_OFFSET           0x000 /* Receiver Buffer Register */
#define LPC31_UART_THR_OFFSET           0x000 /* Transmitter Holding Register */
#define LPC31_UART_DLL_OFFSET           0x000 /* Divisor Latch LSB */
#define LPC31_UART_DLM_OFFSET           0x004 /* Divisor Latch MSB */
#define LPC31_UART_IER_OFFSET           0x004 /* Interrupt Enable Register */
#define LPC31_UART_IIR_OFFSET           0x008 /* Interrupt Identification Register */
#define LPC31_UART_FCR_OFFSET           0x008 /* FIFO Control Register */
#define LPC31_UART_LCR_OFFSET           0x00c /* Line Control Register */
#define LPC31_UART_MCR_OFFSET           0x010 /* Modem Control Register */
#define LPC31_UART_LSR_OFFSET           0x014 /* Line Status Register */
#define LPC31_UART_MSR_OFFSET           0x018 /* Modem status Register */
#define LPC31_UART_SCR_OFFSET           0x01c /* Scratch Register */
                                                /* 0x020: Reserved */
#define LPC31_UART_ICR_OFFSET           0x024 /* IrDA Control Register */
#define LPC31_UART_FDR_OFFSET           0x028 /* Fractional Divider Register */
                                                /* 0x02c: Reserved */
#define LPC31_UART_POP_OFFSET           0x030 /* NHP Pop Register */
#define LPC31_UART_MODE_OFFSET          0x034 /* NHP Mode Selection Register */
                                                /* 0x038-0xfd4: Reserved */
#define LPC31_UART_INTCE_OFFSET         0xfd8 /* Interrupt Clear Enable Register */
#define LPC31_UART_INTSE_OFFSET         0xfdc /* Interrupt Set Enable Register */
#define LPC31_UART_INTS_OFFSET          0xfe0 /* Interrupt Status Register */
#define LPC31_UART_INTE_OFFSET          0xfe4 /* Interrupt Enable Register */
#define LPC31_UART_INTCS_OFFSET         0xfe8 /* Interrupt Clear Status Register */
#define LPC31_UART_INTSS_OFFSET         0xfec /* Interrupt Set Status Register */
                                                /* 0xff0-0xff8: Reserved */

/* UART register (virtual) addresses ************************************************************/

#define LPC31_UART_RBR                  (LPC31_UART_VBASE+LPC31_UART_RBR_OFFSET)
#define LPC31_UART_THR                  (LPC31_UART_VBASE+LPC31_UART_THR_OFFSET)
#define LPC31_UART_DLL                  (LPC31_UART_VBASE+LPC31_UART_DLL_OFFSET)
#define LPC31_UART_DLM                  (LPC31_UART_VBASE+LPC31_UART_DLM_OFFSET)
#define LPC31_UART_IER                  (LPC31_UART_VBASE+LPC31_UART_IER_OFFSET)
#define LPC31_UART_IIR                  (LPC31_UART_VBASE+LPC31_UART_IIR_OFFSET)
#define LPC31_UART_FCR                  (LPC31_UART_VBASE+LPC31_UART_FCR_OFFSET)
#define LPC31_UART_LCR                  (LPC31_UART_VBASE+LPC31_UART_LCR_OFFSET)
#define LPC31_UART_MCR                  (LPC31_UART_VBASE+LPC31_UART_MCR_OFFSET)
#define LPC31_UART_LSR                  (LPC31_UART_VBASE+LPC31_UART_LSR_OFFSET)
#define LPC31_UART_MSR                  (LPC31_UART_VBASE+LPC31_UART_MSR_OFFSET)
#define LPC31_UART_SCR                  (LPC31_UART_VBASE+LPC31_UART_SCR_OFFSET)
#define LPC31_UART_ICR                  (LPC31_UART_VBASE+LPC31_UART_ICR_OFFSET)
#define LPC31_UART_FDR                  (LPC31_UART_VBASE+LPC31_UART_FDR_OFFSET)
#define LPC31_UART_POP                  (LPC31_UART_VBASE+LPC31_UART_POP_OFFSET)
#define LPC31_UART_MODE                 (LPC31_UART_VBASE+LPC31_UART_MODE_OFFSET)
#define LPC31_UART_INTCE                (LPC31_UART_VBASE+LPC31_UART_INTCE_OFFSET)
#define LPC31_UART_INTSE                (LPC31_UART_VBASE+LPC31_UART_INTSE_OFFSET)
#define LPC31_UART_INTS                 (LPC31_UART_VBASE+LPC31_UART_INTS_OFFSET)
#define LPC31_UART_INTE                 (LPC31_UART_VBASE+LPC31_UART_INTE_OFFSET)
#define LPC31_UART_INTCS                (LPC31_UART_VBASE+LPC31_UART_INTCS_OFFSET)
#define LPC31_UART_INTSS                (LPC31_UART_VBASE+LPC31_UART_INTSS_OFFSET)

/* UART register bit definitions ****************************************************************/
/* Receive Buffer Register RBR, address 0x15001000 */

#define UART_RBR_SHIFT                    (0)       /* Bits 0-7 */
#define UART_RBR_MASK                     (0xff << UART_RBR_SHIFT)

/* Transmitter Holding Register THR, address 0x15001000 */

#define UART_THR_SHIFT                    (0)       /* Bits 0-7 */
#define UART_THR_MASK                     (0xff << UART_THR_SHIFT)

/* Divisor register Latch LSB DLL, address 0x15001000 */

#define UART_DLL_SHIFT                    (0)       /* Bits 0-7 */
#define UART_DLL_MASK                     (0xff << UART_DLL_SHIFT)

/* Divisor latch register MSB DLM, address 0x15001004 */

#define UART_DLM_SHIFT                    (0)       /* Bits 0-7 */
#define UART_DLM_MASK                     (0xff << UART_DLM_SHIFT)

/* Interrupt Enable Register IER, address 0x15001004 */

#define UART_IER_CTSINTEN                 (1 << 7)  /* Bit 7:  Enable modem status interrupt on CTS transition  */
#define UART_IER_MSINTEN                  (1 << 3)  /* Bit 3:  Enable Modem Status interrupt */
#define UART_IER_RLSINTEN                 (1 << 2)  /* Bit 2:  Receiver Line Status interrupt enable */
#define UART_IER_THREINTEN                (1 << 1)  /* Bit 1:  Transmitter Holding Register Empty interrupt enable */
#define UART_IER_RDAINTEN                 (1 << 0)  /* Bit 0:  Receive Data Available interrupt enable */
#define UART_IER_ALLINTS                  (0x1f)

/* Interrupt Identification Register IIR, address 0x15001008 */

#define UART_IIR_FIFOEN_SHIFT             (6)       /* Bits 6-7: Copies of FCR[0] */
#define UART_IIR_FIFOEN_MASK              (3 << UART_IIR_FIFOEN_SHIFT)
#define UART_IIR_INTID_SHIFT              (1)       /* Bits 1-3: Interrupt identification */
#define UART_IIR_INTID_MASK               (7 << UART_IIR_INTID_SHIFT)
#  define UART_IIR_INTID_MS               (0 << UART_IIR_INTID_SHIFT) /* Modem status */
#  define UART_IIR_INTID_THRE             (1 << UART_IIR_INTID_SHIFT) /* Transmitter Holding Register empty */
#  define UART_IIR_INTID_RDA              (2 << UART_IIR_INTID_SHIFT) /* Received Data Available */
#  define UART_IIR_INTID_RLS              (3 << UART_IIR_INTID_SHIFT) /* Receiver Line Status */
#  define UART_IIR_INTID_TIMEOUT          (6 << UART_IIR_INTID_SHIFT) /* Character time-out */
#define UART_IIR_NOINT                    (1 << 0)  /* Bit 0:  Interrupt status, 1=no interrupt */

/* FIFO Control Register FCR, address 0x15001008 */

#define UART_FCR_RXTRIGLEVEL_SHIFT        (6)       /* Bits 6-7: 7:6 Receiver trigger level selection */
#define UART_FCR_RXTRIGLEVEL_MASK         (3 << UART_FCR_RXTRIGLEVEL_SHIFT)
#  define UART_FCR_RXTRIGLEVEL_1          (0 << UART_FCR_RXTRIGLEVEL_SHIFT) /* Rx trigger at character 1 */
#  define UART_FCR_RXTRIGLEVEL_16         (1 << UART_FCR_RXTRIGLEVEL_SHIFT) /* Rx trigger at character 16 */
#  define UART_FCR_RXTRIGLEVEL_32         (2 << UART_FCR_RXTRIGLEVEL_SHIFT) /* Rx trigger at character 32 */
#  define UART_FCR_RXTRIGLEVEL_56         (3 << UART_FCR_RXTRIGLEVEL_SHIFT) /* Rx trigger at character 56 */
#define UART_FCR_DMAMODE                  (1 << 3)  /* Bit 3:  DMA mode select */
#define UART_FCR_TXFIFORST                (1 << 2)  /* Bit 2:  Transmitter FIFO reset */
#define UART_FCR_RXFIFORST                (1 << 1)  /* Bit 1:  Receiver FIFO reset */
#define UART_FCR_FIFOENABLE               (1 << 0)  /* Bit 0:  Transmit and receive FIFO enable */

/* Line Control Register LCR, address 0x1500100c */

#define UART_LCR_DLAB                     (1 << 7)  /* Bit 7: Divisor Latch Access bit */
#define UART_LCR_BRKCTRL                  (1 << 6)  /* Bit 6: Break control bit */
#define UART_LCR_PARSTICK                 (1 << 5)  /* Bit 5: Enable sticky parity mode */
#define UART_LCR_PAREVEN                  (1 << 4)  /* Bit 4: Select even parity */
#define UART_LCR_PAREN                    (1 << 3)  /* Bit 3: Parity enable */
#define UART_LCR_NSTOPBITS                (1 << 2)  /* Bit 2: Number of stop bits selector */
#define UART_LCR_WDLENSEL_SHIFT           (0)       /* Bits 0-1: Word length selector */
#define UART_LCR_WDLENSEL_MASK            (3 << UART_LCR_WDLENSEL_SHIFT)
#  define UART_LCR_WDLENSEL_5BITS         (0 << UART_LCR_WDLENSEL_SHIFT) /* Char length=5 stopbits=1 or 1.5*/
#  define UART_LCR_WDLENSEL_6BITS         (1 << UART_LCR_WDLENSEL_SHIFT) /* Char length=6 stopbits=1 or 2 */
#  define UART_LCR_WDLENSEL_7BITS         (2 << UART_LCR_WDLENSEL_SHIFT) /* Char length=7 stopbits=1 or 2 */
#  define UART_LCR_WDLENSEL_8BITS         (3 << UART_LCR_WDLENSEL_SHIFT) /* Char length=8 stopbits=1 or 2 */

/* Modem Control Register MCR, address 0x15001010 */

#define UART_MCR_AUTOCTSEN                (1 << 7)  /* Bit 7:  Auto-cts flow control enable */
#define UART_MCR_AUTORTSEN                (1 << 6)  /* Bit 6:  Auto-rts flow control enable */
#define UART_MCR_LOOPEN                   (1 << 4)  /* Bit 4:  Loop-back mode enable */
#define UART_MCR_RTS                      (1 << 1)  /* Bit 1:  Request To Send */

/* Line Status Register LSR, address 0x15001014 */

#define UART_LSR_RXER                     (1 << 7)  /* Bit 7:  Error in receiver FIFO */
#define UART_LSR_TEMT                     (1 << 6)  /* Bit 6:  Transmitter empty (TSR and THR) */
#define UART_LSR_THRE                     (1 << 5)  /* Bit 5:  Transmitter Holding Register empty */
#define UART_LSR_BI                       (1 << 4)  /* Bit 4:  Break indication */
#define UART_LSR_FE                       (1 << 3)  /* Bit 3:  Framing error */
#define UART_LSR_PE                       (1 << 2)  /* Bit 2:  Parity error */
#define UART_LSR_OE                       (1 << 1)  /* Bit 1:  Overrun error */
#define UART_LSR_RDR                      (1 << 0)  /* Bit 0:  Read Data ready */

/* Modem Status Register MSR, address 0x15001018 */

#define UART_MSR_CTS                      (1 << 4)  /* Bit 4:  CTS is modem flow control signal */
#define UART_MSR_DCTS                     (1 << 0)  /* Bit 0:  Delta Clear To Send */

/* Scratch Register SCR, address 0x1500101c */

#define UART_SCR_SCRVAL_SHIFT             (0)       /* Bits 0-7: Scratch Value */
#define UART_SCR_SCRVAL_MASK              (0xff << bb)

/* IrDA Control Register ICR, address 0x15001024 */

#define UART_ICR_PULSEDIV_SHIFT           (3)       /* Bits 3-5: Configures fixed pulse width mode */
#define UART_ICR_PULSEDIV_MASK            (7 << UART_ICR_PULSEDIV_SHIFT)
#define UART_ICR_FIXPULSEEN               (1 << 2)  /* Bit 2: Enables IrDA fixed pulse width mode */
#define UART_ICR_IRDAINV                  (1 << 1)  /* Bit 1: Serial input is inverted */
#define UART_ICR_IRDAEN                   (1 << 0)  /* Bit 0: Enable IrDA */

/* Fractional Divider Register FDR, address 0x15001028 */

#define UART_FDR_MULVAL_SHIFT             (4)       /* Bits 4-7: Baud pre-scaler multiplier value */
#define UART_FDR_MULVAL_MASK              (15 << UART_FDR_MULVAL_SHIFT)
#define UART_FDR_DIVADDVAL_SHIFT          (0)       /* Bits 0-3: Baud pre-scaler divisor value */
#define UART_FDR_DIVADDVAL_MASK           (15 << UART_FDR_DIVADDVAL_SHIFT)

/* NHP POP Register POP, address 0x15001030 */

#define UART_POP_POPRBR                   (1 << 0)  /* Bit 0: Pop from RBR as if RBR read in non-NHP mode */

/* Mode Selection Register MODE, 0x15001034 */

#define UART_MODE_NHP                     (1 << 0)  /* Bit 0: Enable UART NHP mode */

/* Interrupt Clear Enable Register INTCE, address 0x15001fd8
 * Interrupt Set Enable Register INTSE, address 0x15001fdc
 * Interrupt Status Register INTS, address 0x15001fe0
 * Interrupt Enable Register INTE, address 0x15001fe4
 * Interrupt Clear Status Register INTCS, address 0x15001fe8
 * Interrupt Set Status Register INTSS, address 0x15001fec
 */

#define UART_OEINT                        (1 << 15) /* Bit 15: Overrun Error Interrupt */
#define UART_PEINT                        (1 << 14) /* Bit 14: Parity Error Interrupt (not INTSS) */
#define UART_FEINT                        (1 << 13) /* Bit 13: Frame Error Interrupt (not INTSS) */
#define UART_BIINT                        (1 << 12) /* Bit 12: Break Indication Interrupt (not INTSS) */
#define UART_ABTOINT                      (1 << 9)  /* Bit 9:  Auto-Baud Time-Out Interrupt */
#define UART_ABEOINT                      (1 << 8)  /* Bit 8:  End of Auto-Baud Interrupt */
#define UART_RXDAINT                      (1 << 6)  /* Bit 6:  Receiver Data Available Interrupt (not INTSS) */
#define UART_RXTOINT                      (1 << 5)  /* Bit 5:  Receiver Time-Out Interrupt (not INTCE) */
#define UART_THREINT                      (1 << 4)  /* Bit 4:  Transmitter Holding Register Empty Interrupt */
#define UART_DCTSINT                      (1 << 0)  /* Bit 0:  Delta Clear To Send Interrupt */


/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_UART_H */
