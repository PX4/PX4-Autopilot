/****************************************************************************
 * arch/arm/include/dm320/irq.h
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_DM320_IRQ_H
#define __ARCH_ARM_INCLUDE_DM320_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* DM320 Interrupts */

#define DM320_IRQ_TMR0      0 /* IRQ0:  Timer 0 Interrupt */
#define DM320_IRQ_TMR1      1 /* IRQ1:  Timer 1 Interrupt */
#define DM320_IRQ_TMR2      2 /* IRQ2:  Timer 2 Interrupt (CCD timer 0) */
#define DM320_IRQ_TMR3      3 /* IRQ3:  Timer 3 Interrupt (CCD timer 1) */
#define DM320_IRQ_CCDVD0    4 /* IRQ4:  CCD VD Interrupt #0 */
#define DM320_IRQ_CCDVD1    5 /* IRQ5:  CCD VD Interrupt #1 */
#define DM320_IRQ_CCDWEN    6 /* IRQ6:  CCD WEN Interrupt */
#define DM320_IRQ_VENC      7 /* IRQ7:  Video Encoder Interrupt */
#define DM320_IRQ_SP0       8 /* IRQ8:  Serial Port 0 Interrupt (with DMA) */
#define DM320_IRQ_SP1       9 /* IRQ9:  Serial Port 1 Interrupt */
#define DM320_IRQ_EXTHOST  10 /* IRQ10: External host interrupt */
#define DM320_IRQ_IMGBUF   11 /* IRQ11: Image Buffer */
#define DM320_IRQ_UART0    12 /* IRQ12: UART0 Interrupt */
#define DM320_IRQ_UART1    13 /* IRQ13: UART1 Interrupt */
#define DM320_IRQ_USB0     14 /* IRQ14: USB 0 Interrupt (DMA) */
#define DM320_IRQ_USB1     15 /* IRQ15: USB 1 Interrupt (Core) */
#define DM320_IRQ_VLYNQ    16 /* IRQ16: VLYNQ Interrupt */
#define DM320_IRQ_MTC0     17 /* IRQ17: Memory Traffic Controller 0 (DMA) */
#define DM320_IRQ_MTC1     18 /* IRQ18: Memory Traffic Controller 1 (CFC_RDY) */
#define DM320_IRQ_MMCSD0   19 /* IRQ19: MMC/SD or MS 0 Interrupt */
#define DM320_IRQ_MMCSD1   20 /* IRQ20: MMC/SD or MS 1 Interrupt */
#define DM320_IRQ_EXT0     21 /* IRQ21: External Interrupt #0 (GIO0) */
#define DM320_IRQ_EXT1     22 /* IRQ22: External Interrupt #1 (GIO1) */
#define DM320_IRQ_EXT2     23 /* IRQ23: External Interrupt #2 (GIO2) */
#define DM320_IRQ_EXT3     24 /* IRQ24: External Interrupt #3 (GIO3) */
#define DM320_IRQ_EXT4     25 /* IRQ25: External Interrupt #4 (GIO4) */
#define DM320_IRQ_EXT5     26 /* IRQ26: External Interrupt #5 (GIO5) */
#define DM320_IRQ_EXT6     27 /* IRQ27: External Interrupt #6 (GIO6) */
#define DM320_IRQ_EXT7     28 /* IRQ28: External Interrupt #7 (GIO7) */
#define DM320_IRQ_EXT8     29 /* IRQ29: External Interrupt #8 (GIO8) */
#define DM320_IRQ_EXT9     30 /* IRQ30: External Interrupt #9 (GIO9) */
#define DM320_IRQ_EXT10    31 /* IRQ31: External Interrupt #10 (GIO10) */
#define DM320_IRQ_EXT11    32 /* IRQ32: External Interrupt #11 (GIO11) */
#define DM320_IRQ_EXT12    33 /* IRQ33: External Interrupt #12 (GIO12) */
#define DM320_IRQ_EXT13    34 /* IRQ34: External Interrupt #13 (GIO13) */
#define DM320_IRQ_EXT14    35 /* IRQ35: External Interrupt #14 (GIO14) */
#define DM320_IRQ_EXT15    36 /* IRQ36: External Interrupt #15 (GIO15) */
#define DM320_IRQ_PREV0    37 /* IRQ37: Preview Engine 0 (Preview Over) */
#define DM320_IRQ_PREV1    38 /* IRQ38: Preview Engine 1 (Preview Historgram Over) */
#define DM320_IRQ_WDT      39 /* IRQ39: Watchdog Timer Interrupt */
#define DM320_IRQ_I2C      40 /* IRQ40: I2C Interrupt */
#define DM320_IRQ_CLKC     41 /* IRQ41: Clock controller Interrupt (wake up) */
#define DM320_IRQ_E2ICE    42 /* IRQ42: Embedded ICE Interrupt */
#define DM320_IRQ_ARMCOMRX 43 /* IRQ43: ARMCOMM Receive Interrupt */
#define DM320_IRQ_ARMCOMTX 44 /* IRQ44: ARMCOMM Transmit Interrupt */
#define DM320_IRQ_RSV      45 /* IRQ45: Reserved Interrupt */

#define DM320_IRQ_SYSTIMER DM320_IRQ_TMR0
#define NR_IRQS            (DM320_IRQ_RSV+1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
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
#endif

#endif /* __ARCH_ARM_INCLUDE_DM320_IRQ_H */

