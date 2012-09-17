/************************************************************************************
 * arch/arm/src/lpc43xx/lpc43_mcpwm.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_MCPWM_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_MCPWM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC43_MCPWM_CON_OFFSET       0x0000 /* PWM Control read address */
#define LPC43_MCPWM_CONSET_OFFSET    0x0004 /* PWM Control set address */
#define LPC43_MCPWM_CONCLR_OFFSET    0x0008 /* PWM Control clear address */
#define LPC43_MCPWM_CAPCON_OFFSET    0x000c /* Capture Control read address */
#define LPC43_MCPWM_CAPCONSET_OFFSET 0x0010 /* Capture Control set address */
#define LPC43_MCPWM_CAPCONCLR_OFFSET 0x0014 /* Event Control clear address */
#define LPC43_MCPWM_TC0_OFFSET       0x0018 /* Timer Counter register, channel 0 */
#define LPC43_MCPWM_TC1_OFFSET       0x001c /* Timer Counter register, channel 1 */
#define LPC43_MCPWM_TC2_OFFSET       0x0020 /* Timer Counter register, channel 2 */
#define LPC43_MCPWM_LIM0_OFFSET      0x0024 /* Limit register, channel 0 */
#define LPC43_MCPWM_LIM1_OFFSET      0x0028 /* Limit register, channel 1 */
#define LPC43_MCPWM_LIM2_OFFSET      0x002c /* Limit register, channel 2 */
#define LPC43_MCPWM_MAT0_OFFSET      0x0030 /* Match register, channel 0 */
#define LPC43_MCPWM_MAT1_OFFSET      0x0034 /* Match register, channel 1 */
#define LPC43_MCPWM_MAT2_OFFSET      0x0038 /* Match register, channel 2 */
#define LPC43_MCPWM_DT_OFFSET        0x003c /* Dead time register */
#define LPC43_MCPWM_MCCP_OFFSET      0x0040 /* Communication Pattern register */
#define LPC43_MCPWM_CAP0_OFFSET      0x0044 /* Capture register, channel 0 */
#define LPC43_MCPWM_CAP1_OFFSET      0x0048 /* Capture register, channel 1 */
#define LPC43_MCPWM_CAP2_OFFSET      0x004c /* Capture register, channel 2 */
#define LPC43_MCPWM_INTEN_OFFSET     0x0050 /* Interrupt Enable read address */
#define LPC43_MCPWM_INTENSET_OFFSET  0x0054 /* Interrupt Enable set address */
#define LPC43_MCPWM_INTENCLR_OFFSET  0x0058 /* Interrupt Enable clear address */
#define LPC43_MCPWM_CNTCON_OFFSET    0x005c /* Count Control read address */
#define LPC43_MCPWM_CNTCONSET_OFFSET 0x0060 /* Count Control set address */
#define LPC43_MCPWM_CNTCONCLR_OFFSET 0x0064 /* Count Control clear address */
#define LPC43_MCPWM_INTF_OFFSET      0x0068 /* Interrupt flags read address */
#define LPC43_MCPWM_INTFSET_OFFSET   0x006c /* Interrupt flags set address */
#define LPC43_MCPWM_INTFCLR_OFFSET   0x0070 /* Interrupt flags clear address */
#define LPC43_MCPWM_CAPCLR_OFFSET    0x0074 /* Capture clear address */

/* Register addresses ***************************************************************/

#define LPC43_MCPWM_CON              (LPC43_MCPWM_BASE+LPC43_MCPWM_CON_OFFSET)
#define LPC43_MCPWM_CONSET           (LPC43_MCPWM_BASE+LPC43_MCPWM_CONSET_OFFSET)
#define LPC43_MCPWM_CONCLR           (LPC43_MCPWM_BASE+LPC43_MCPWM_CONCLR_OFFSET)
#define LPC43_MCPWM_CAPCON           (LPC43_MCPWM_BASE+LPC43_MCPWM_CAPCON_OFFSET)
#define LPC43_MCPWM_CAPCONSET        (LPC43_MCPWM_BASE+LPC43_MCPWM_CAPCONSET_OFFSET)
#define LPC43_MCPWM_CAPCONCLR        (LPC43_MCPWM_BASE+LPC43_MCPWM_CAPCONCLR_OFFSET)
#define LPC43_MCPWM_TC0              (LPC43_MCPWM_BASE+LPC43_MCPWM_TC0_OFFSET)
#define LPC43_MCPWM_TC1              (LPC43_MCPWM_BASE+LPC43_MCPWM_TC1_OFFSET)
#define LPC43_MCPWM_TC2              (LPC43_MCPWM_BASE+LPC43_MCPWM_TC2_OFFSET)
#define LPC43_MCPWM_LIM0             (LPC43_MCPWM_BASE+LPC43_MCPWM_LIM0_OFFSET)
#define LPC43_MCPWM_LIM1             (LPC43_MCPWM_BASE+LPC43_MCPWM_LIM1_OFFSET)
#define LPC43_MCPWM_LIM2             (LPC43_MCPWM_BASE+LPC43_MCPWM_LIM2_OFFSET)
#define LPC43_MCPWM_MAT0             (LPC43_MCPWM_BASE+LPC43_MCPWM_MAT0_OFFSET)
#define LPC43_MCPWM_MAT1             (LPC43_MCPWM_BASE+LPC43_MCPWM_MAT1_OFFSET)
#define LPC43_MCPWM_MAT2             (LPC43_MCPWM_BASE+LPC43_MCPWM_MAT2_OFFSET)
#define LPC43_MCPWM_DT               (LPC43_MCPWM_BASE+LPC43_MCPWM_DT_OFFSET)
#define LPC43_MCPWM_MCCP             (LPC43_MCPWM_BASE+LPC43_MCPWM_MCCP_OFFSET)
#define LPC43_MCPWM_CAP0             (LPC43_MCPWM_BASE+LPC43_MCPWM_CAP0_OFFSET)
#define LPC43_MCPWM_CAP1             (LPC43_MCPWM_BASE+LPC43_MCPWM_CAP1_OFFSET)
#define LPC43_MCPWM_CAP2             (LPC43_MCPWM_BASE+LPC43_MCPWM_CAP2_OFFSET)
#define LPC43_MCPWM_INTEN            (LPC43_MCPWM_BASE+LPC43_MCPWM_INTEN_OFFSET)
#define LPC43_MCPWM_INTENSET         (LPC43_MCPWM_BASE+LPC43_MCPWM_INTENSET_OFFSET)
#define LPC43_MCPWM_INTENCLR         (LPC43_MCPWM_BASE+LPC43_MCPWM_INTENCLR_OFFSET)
#define LPC43_MCPWM_CNTCON           (LPC43_MCPWM_BASE+LPC43_MCPWM_CNTCON_OFFSET)
#define LPC43_MCPWM_CNTCONSET        (LPC43_MCPWM_BASE+LPC43_MCPWM_CNTCONSET_OFFSET)
#define LPC43_MCPWM_CNTCONCLR        (LPC43_MCPWM_BASE+LPC43_MCPWM_CNTCONCLR_OFFSET)
#define LPC43_MCPWM_INTF             (LPC43_MCPWM_BASE+LPC43_MCPWM_INTF_OFFSET)
#define LPC43_MCPWM_INTFSET          (LPC43_MCPWM_BASE+LPC43_MCPWM_INTFSET_OFFSET)
#define LPC43_MCPWM_INTFCLR          (LPC43_MCPWM_BASE+LPC43_MCPWM_INTFCLR_OFFSET)
#define LPC43_MCPWM_CAPCLR           (LPC43_MCPWM_BASE+LPC43_MCPWM_CAPCLR_OFFSET)

/* Register bit definitions *********************************************************/
/* There are no bit field definitions for the following registers because they support
 * 32-bit values:
 *
 * - Timer Counter register, channel 0 (TC0), Timer Counter register, channel 1 (TC1),
 *   and Timer Counter register, channel 2 (TC2): 32-bit Timer/Counter values for
 *   channels 0, 1, 2 (no bit field definitions)
 *
 * - Limit register, channel 0 (LIM0), Limit register, channel 1 (LIM1), and Limit
 *   register, channel 2 (LIM2): 32-bit Limit values for TC0, 1, 2 (no bit field
 *   definitions)
 *
 * - Match register, channel 0 MAT0), Match register, channel 1 (MAT1), and Match
 *   register, channel 2 (MAT2): 32-bit Match values for TC0, 1, 2 (no bit field
 *   definitions).
 *
 * - Capture register, channel 0 (CAP0), Capture register, channel 1 (CAP1), and
 *   Capture register, channel 2 (CAP2): 32-bit TC value at a capture event for
 *  channels 0, 1, 2 (no bit field definitions)
 */

/* PWM Control read address (CON), PWM Control set address (CONSET), and PWM Control
 * clear address (CONCLR) common regiser bit definitions.
 */

#define MCPWM_CON_RUN0               (1 << 0)  /* Bit 0:  Stops/starts timer channel 0 */
#define MCPWM_CON_CENTER0            (1 << 1)  /* Bit 1:  Chan 0 edge/center aligned operation */
#define MCPWM_CON_POLA0              (1 << 2)  /* Bit 2:  Polarity of MCOA0 and MCOB0 */
#define MCPWM_CON_DTE0               (1 << 3)  /* Bit 3:  Dead time feature control */
#define MCPWM_CON_DISUP0             (1 << 4)  /* Bit 4:  Enable/disable register updates */
                                               /* Bits 5-7: Reserved */
#define MCPWM_CON_RUN1               (1 << 8)  /* Bit 8:  Stops/starts timer channel 1 */
#define MCPWM_CON_CENTER1            (1 << 9)  /* Bit 9:  Chan 1 edge/center aligned operation */
#define MCPWM_CON_POLA1              (1 << 10) /* Bit 10: Polarity of MCOA1 and MCOB1 */
#define MCPWM_CON_DTE1               (1 << 11) /* Bit 11: Dead time feature control */
#define MCPWM_CON_DISUP1             (1 << 12) /* Bit 12: Enable/disable register updates */
                                               /* Bits 13-15: Reserved */
#define MCPWM_CON_RUN2               (1 << 16) /* Bit 16:  Stops/starts timer channel 2 */
#define MCPWM_CON_CENTER2            (1 << 17) /* Bit 17:  Chan 2 edge/center aligned operation */
#define MCPWM_CON_POLA2              (1 << 18) /* Bit 18: Polarity of MCOA1 and MCOB1 */
#define MCPWM_CON_DTE2               (1 << 19) /* Bit 19: Dead time feature control */
#define MCPWM_CON_DISUP2             (1 << 20) /* Bit 20: Enable/disable register updates */
                                               /* Bits 21-28: Reserved */
#define MCPWM_CON_INVBDC             (1 << 29) /* Bit 29: Polarity of MCOB outputs (all channels)  */
#define MCPWM_CON_ACMODE             (1 << 30) /* Bit 30: 3-phase AC mode select */
#define MCPWM_CON_DCMODE             (1 << 31) /* Bit 31: 3-phase DC mode select */

/* Capture Control read address (CAPCON), Capture Control set address (CAPCONSET),
 * and Event Control clear address (CAPCONCLR) common register bit defintions
 */

#define MCPWM_CAPCON_CAP0MCI0RE      (1 << 0)  /* Bit 0:  Enable chan0 rising edge capture MCI0 */
#define MCPWM_CAPCON_CAP0MCI0FE      (1 << 1)  /* Bit 1:  Enable chan 0 falling edge capture MCI0 */
#define MCPWM_CAPCON_CAP0MCI1RE      (1 << 2)  /* Bit 2:  Enable chan 0 rising edge capture MCI1 */
#define MCPWM_CAPCON_CAP0MCI1FE      (1 << 3)  /* Bit 3:  Enable chan 0 falling edge capture MCI1 */
#define MCPWM_CAPCON_CAP0MCI2RE      (1 << 4)  /* Bit 4:  Enable chan 0 rising edge capture MCI2 */
#define MCPWM_CAPCON_CAP0MCI2FE      (1 << 5)  /* Bit 5:  Enable chan 0 falling edge capture MCI2 */
#define MCPWM_CAPCON_CAP1MCI0RE      (1 << 6)  /* Bit 6:  Enable chan 1 rising edge capture MCI0 */
#define MCPWM_CAPCON_CAP1MCI0FE      (1 << 7)  /* Bit 7:  Enable chan 1 falling edge capture MCI0 */
#define MCPWM_CAPCON_CAP1MCI1RE      (1 << 8)  /* Bit 8:  Enable chan 1 rising edge capture MCI1 */
#define MCPWM_CAPCON_CAP1MCI1FE      (1 << 9)  /* Bit 9:  Enable chan 1 falling edge capture MCI1 */
#define MCPWM_CAPCON_CAP1MCI2RE      (1 << 10) /* Bit 10: Enable chan 1 rising edge capture MCI2 */
#define MCPWM_CAPCON_CAP1MCI2FE      (1 << 11) /* Bit 11: Enable chan 1 falling edge capture MCI2 */
#define MCPWM_CAPCON_CAP2MCI0RE      (1 << 12) /* Bit 12: Enable chan 2 rising edge capture MCI0 */
#define MCPWM_CAPCON_CAP2MCI0FE      (1 << 13) /* Bit 13: Enable chan 2 falling edge capture MCI0 */
#define MCPWM_CAPCON_CAP2MCI1RE      (1 << 14) /* Bit 14: Enable chan 2 rising edge capture MCI1 */
#define MCPWM_CAPCON_CAP2MCI1FE      (1 << 15) /* Bit 15: Enable chan 2 falling edge capture MCI1 */
#define MCPWM_CAPCON_CAP2MCI2RE      (1 << 16) /* Bit 16: Enable chan 2 rising edge capture MCI2 */
#define MCPWM_CAPCON_CAP2MCI2FE      (1 << 17) /* Bit 17: Enable chan 2 falling edge capture MCI2 */
#define MCPWM_CAPCON_RT0             (1 << 18) /* Bit 18: TC0 reset by chan 0 capture event */
#define MCPWM_CAPCON_RT1             (1 << 19) /* Bit 19: TC1 reset by chan 1 capture event */
#define MCPWM_CAPCON_RT2             (1 << 20) /* Bit 20: TC2 reset by chan 2 capture event */
                                               /* Bits 21-31: Reserved
/* Dead time register */

#define MCPWM_DT_DT0_SHIFT           (0)       /* Bits 0-9: Dead time for channel 0 */
#define MCPWM_DT_DT0_MASK            (0x03ff << MCPWM_DT_DT0_SHIFT)
#define MCPWM_DT_DT1_SHIFT           (10)      /* Bits 10-19: Dead time for channel 1 */
#define MCPWM_DT_DT1_MASK            (0x03ff << MCPWM_DT_DT1_SHIFT)
#define MCPWM_DT_DT2_SHIFT           (20)      /* Bits 20-29: Dead time for channel 2 */
#define MCPWM_DT_DT2_MASK            (0x03ff << MCPWM_DT_DT2_SHIFT)
                                               /* Bits 30-31: reserved */
/* Communication Pattern register */

#define MCPWM_MCCP_CCPA0             (1 << 0)  /* Bit 0:  Iinternal MCOA0 */
#define MCPWM_MCCP_CCPB0             (1 << 1)  /* Bit 1:  MCOB0 tracks internal MCOA0 */
#define MCPWM_MCCP_CCPA1             (1 << 2)  /* Bit 2:  MCOA1 tracks internal MCOA0 */
#define MCPWM_MCCP_CCPB1             (1 << 3)  /* Bit 3:  MCOB1 tracks internal MCOA0 */
#define MCPWM_MCCP_CCPA2             (1 << 4)  /* Bit 4:  MCOA2 tracks internal MCOA0 */
#define MCPWM_MCCP_CCPB2             (1 << 5)  /* Bit 5:  MCOB2 tracks internal MCOA0 */
                                               /* Bits 6-31: reserved */

/* Interrupt Enable read address (INTEN), Interrupt Enable set address (INTENSET),
 * Interrupt Enable clear address (INTENCLR), Interrupt flags read address (INTF),
 * Interrupt flags set address (INTFSET), and Interrupt flags clear address (INTFCLR)
 * common bit field definitions
 */

#define MCPWM_INT_ILIM0              (1 << 0)  /* Bit 0:  Limit interrupts for channel 0 */
#define MCPWM_INT_IMAT0              (1 << 1)  /* Bit 1:  Match interrupts for channel 0 */
#define MCPWM_INT_ICAP0              (1 << 2)  /* Bit 2:  Capture interrupts for channel 0 */
                                               /* Bit 3:  Reserved */
#define MCPWM_INT_ILIM1              (1 << 4)  /* Bit 4:  Limit interrupts for channel 1 */
#define MCPWM_INT_IMAT1              (1 << 5)  /* Bit 5:  Match interrupts for channel 1 */
#define MCPWM_INT_ICAP1              (1 << 6)  /* Bit 6:  Capture interrupts for channel 1 */
                                               /* Bit 7:  Reserved */
#define MCPWM_INT_ILIM2              (1 << 8)  /* Bit 8:  Limit interrupts for channel 2 */
#define MCPWM_INT_IMAT2              (1 << 9)  /* Bit 9:  Match interrupts for channel 2 */
#define MCPWM_INT_ICAP2              (1 << 10) /* Bit 10: Capture interrupts for channel 2 */
                                               /* Bits 11-14:  Reserved */
#define MCPWM_INT_ABORT              (1 << 15) /* Bit 15:  Fast abort interrupt */
                                               /* Bits 16-31: Reserved */

/* Count Control read address (CNTCON), Count Control set address (CNTCONSET), and
 * Count Control clear address (CNTCONCLR) common register bit definitions.
 */

#define MCPWM_CNTCON_TC0MCI0RE       (1 << 0)  /* Bit 0:  Counter 0 incr on rising edge MCI0 */
#define MCPWM_CNTCON_TC0MCI0FE       (1 << 1)  /* Bit 1:  Counter 0 incr onfalling edge MCI0 */
#define MCPWM_CNTCON_TC0MCI1RE       (1 << 2)  /* Bit 2:  Counter 0 incr onrising edge MCI1 */
#define MCPWM_CNTCON_TC0MCI1FE       (1 << 3)  /* Bit 3:  Counter 0 incr onfalling edge MCI1 */
#define MCPWM_CNTCON_TC0MCI2RE       (1 << 4)  /* Bit 4:  Counter 0 incr onrising edge MCI2 */
#define MCPWM_CNTCON_TC0MCI2FE       (1 << 5)  /* Bit 5:  Counter 0 incr onfalling edge MCI2 */
#define MCPWM_CNTCON_TC1MCI0RE       (1 << 6)  /* Bit 6:  Counter 1 incr onrising edge MCI0 */
#define MCPWM_CNTCON_TC1MCI0FE       (1 << 7)  /* Bit 7:  Counter 1 incr onfalling edge MCI0 */
#define MCPWM_CNTCON_TC1MCI1RE       (1 << 8)  /* Bit 8:  Counter 1 incr onrising edge MCI1 */
#define MCPWM_CNTCON_TC1MCI1FE       (1 << 9)  /* Bit 9:  Counter 1 incr onfalling edge MCI1 */
#define MCPWM_CNTCON_TC1MCI2RE       (1 << 10) /* Bit 10: Counter 1 incr onrising edge MCI2 */
#define MCPWM_CNTCON_TC1MCI2FE       (1 << 11) /* Bit 11: Counter 1 incr onfalling edge MCI2 */
#define MCPWM_CNTCON_TC2MCI0RE       (1 << 12) /* Bit 12: Counter 2 incr onrising edge MCI0 */
#define MCPWM_CNTCON_TC2MCI0FE       (1 << 13) /* Bit 13: Counter 2 incr onfalling edge MCI0 */
#define MCPWM_CNTCON_TC2MCI1RE       (1 << 14) /* Bit 14: Counter 2 incr onrising edge MCI1 */
#define MCPWM_CNTCON_TC2MCI1FE       (1 << 15) /* Bit 15: Counter 2 incr onfalling edge MCI1 */
#define MCPWM_CNTCON_TC2MCI2RE       (1 << 16) /* Bit 16: Counter 2 incr onrising edge MCI2 */
#define MCPWM_CNTCON_TC2MCI2FE       (1 << 17) /* Bit 17: Counter 2 incr onfalling edge MCI2 */
                                               /* Bits 18-28: Reserved */
#define MCPWM_CNTCON_CNTR0           (1 << 29) /* Bit 29: Channel 0 counter mode */
#define MCPWM_CNTCON_CNTR1           (1 << 30) /* Bit 30: Channel 1 counter mode */
#define MCPWM_CNTCON_CNTR2           (1 << 31) /* Bit 31: Channel 2 counter mode */

/* Capture clear address */

#define MCPWM_CAPCLR_CLR0            (1 << 0)  /* Bit 0:  Clear CAP0 register */
#define MCPWM_CAPCLR_CLR1            (1 << 1)  /* Bit 1:  Clear CAP1 register */
#define MCPWM_CAPCLR_CLR2            (1 << 2)  /* Bit 2:  Clear CAP2 register */
                                               /* Bits 2-31: Reserved */ 

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_MCPWM_H */
