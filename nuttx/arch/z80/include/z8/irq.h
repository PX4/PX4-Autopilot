/****************************************************************************
 * arch/z8/include/z8/irq.h
 * arch/chip/irq.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
 * only indirectly through nuttx/irq.h (via arch/irq.h)
 */

#ifndef __ARCH_Z8_IRQ_H
#define __ARCH_Z8_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* This is similar configuration information that is contained in ez8.h.
 * However, this file must be include-able by assembly language files and,
 * hence, cannot include ez8.h.  The logic is fragmentary at present.
 */

#ifndef ENCORE_VECTORS
#  if defined(_Z8ENCORE_F642X) || defined(_Z8ENCORE_64K_SERIES)
#    define ENCORE_VECTORS 1
#  endif

#  if defined(_Z8ENCORE_F640X) || defined(_Z8ENCORE_640_FAMILY)
#    define ENCORE_VECTORS 1
#  endif

#  if defined(_Z8ENCORE_F08X) || defined(_Z8ENCORE_8K_SERIES)
#    define ENCORE_VECTORS 1
#  endif

#  if defined(_Z8ENCORE_4K_SERIES)
#    define ENCORE_VECTORS 1
#  endif
#endif

/* ez8 Interrupt Numbers ****************************************************/

#if defined(ENCORE_VECTORS)

#  define  Z8_WDT_IRQ        0
#  define  Z8_TRAP_IRQ       1
#  define  Z8_TIMER2_IRQ     2 /* Only if EZ8_TIMER3 defined */
#  define  Z8_TIMER1_IRQ     3
#  define  Z8_TIMER0_IRQ     4
#  define  Z8_UART0_RX_IRQ   5 /* Only if EZ8_UART0 defined */
#  define  Z8_UART0_TX_IRQ   6 /* Only if EZ8_UART0 defined */
#  define  Z8_I2C_IRQ        7 /* Only if EZ8_I2C defined */
#  define  Z8_SPI_IRQ        8 /* Only if EZ8_SPI defined */
#  define  Z8_ADC_IRQ        9 /* Only if EZ8_ADC defined */
#  define  Z8_P7AD_IRQ      10
#  define  Z8_P6AD_IRQ      11
#  define  Z8_P5AD_IRQ      12
#  define  Z8_P4AD_IRQ      13
#  define  Z8_P3AD_IRQ      14
#  define  Z8_P2AD_IRQ      15
#  define  Z8_P1AD_IRQ      16
#  define  Z8_P0AD_IRQ      17
#  define  Z8_TIMER3_IRQ    18 /* Only if EZ8_TIMER4 defined */
#  define  Z8_UART1_RX_IRQ  19 /* Only if EZ8_UART1 defined */
#  define  Z8_UART1_TX_IRQ  20 /* Only if EZ8_UART1 defined */
#  define  Z8_DMA_IRQ       21 /* Only if EZ8_DMA defined */
#  define  Z8_C3_IRQ        22 /* Only if EZ8_PORT1 defined */
#  define  Z8_C2_IRQ        23 /* Only if EZ8_PORT1 defined */
#  define  Z8_C1_IRQ        24 /* Only if EZ8_PORT1 defined */
#  define  Z8_C0_IRQ        25 /* Only if EZ8_PORT1 defined */

#  define NR_IRQS          (26)

#elif defined(ENCORE_XP_VECTORS)

#  define  Z8_WDT_IRQ        0
#  define  Z8_TRAP_IRQ       1
#  define  Z8_TIMER2_IRQ     2 /* Only if EZ8_TIMER3 defined */
#  define  Z8_TIMER1_IRQ     3
#  define  Z8_TIMER0_IRQ     4
#  define  Z8_UART0_RX_IRQ   5 /* Only if EZ8_UART0 defined */
#  define  Z8_UART0_TX_IRQ   6 /* Only if EZ8_UART0 defined */
#  define  Z8_I2C_IRQ        7 /* Only if EZ8_I2C defined */
#  define  Z8_SPI_IRQ        8 /* Only if EZ8_SPI defined */
#  define  Z8_ADC_IRQ        9 /* Only if EZ8_ADC or EZ8_ADC_NEW defined */
#  define  Z8_P7AD_IRQ      10
#  define  Z8_P6AD_IRQ      11
#  define  Z8_P5AD_IRQ      12
#  define  Z8_P4AD_IRQ      13
#  define  Z8_P3AD_IRQ      14
#  define  Z8_P2AD_IRQ      15
#  define  Z8_P1AD_IRQ      16
#  define  Z8_P0AD_IRQ      17
#  define  Z8_TIMER3_IRQ    18 /* Only if EZ8_TIMER4 defined */
#  define  Z8_UART1_RX_IRQ  19 /* Only if EZ8_UART1 defined */
#  define  Z8_UART1_TX_IRQ  20 /* Only if EZ8_UART1 defined */
#  define  Z8_DMA_IRQ       21 /* Only if EZ8_DMA defined */
#  define  Z8_C3_IRQ        22 /* Only if EZ8_PORT1 defined */
#  define  Z8_C2_IRQ        23 /* Only if EZ8_PORT1 defined */
#  define  Z8_C1_IRQ        24 /* Only if EZ8_PORT1 defined */
#  define  Z8_C0_IRQ        25 /* Only if EZ8_PORT1 defined */
#  define  Z8_POTRAP_IRQ    27
#  define  Z8_WOTRAP_IRQ    28

#  define NR_IRQS          (29)

#elif defined(ENCORE_XP16K_VECTORS)

#  define  Z8_WDT_IRQ        0
#  define  Z8_TRAP_IRQ       1
#  define  Z8_TIMER2_IRQ     2 /* Only if EZ8_TIMER3 defined */
#  define  Z8_TIMER1_IRQ     3
#  define  Z8_TIMER0_IRQ     4
#  define  Z8_UART0_RX_IRQ   5 /* Only if EZ8_UART0 defined */
#  define  Z8_UART0_TX_IRQ   6 /* Only if EZ8_UART0 defined */
#  define  Z8_I2C_IRQ        7 /* Only if EZ8_I2C defined */
#  define  Z8_SPI_IRQ        8 /* Only if EZ8_ESPI defined */
#  define  Z8_ADC_IRQ        9 /* Only if EZ8_ADC_NEW defined */
#  define  Z8_P7AD_IRQ      10
#  define  Z8_P6AD_IRQ      11
#  define  Z8_P5AD_IRQ      12
#  define  Z8_P4AD_IRQ      13
#  define  Z8_P3AD_IRQ      14
#  define  Z8_P2AD_IRQ      15
#  define  Z8_P1AD_IRQ      16
#  define  Z8_P0AD_IRQ      17
#  define  Z8_MCT_IRQ       19 /* Only if EZ8_MCT defined */
#  define  Z8_UART1_RX_IRQ  20 /* Only if EZ8_UART1 defined */
#  define  Z8_UART1_TX_IRQ  21 /* Only if EZ8_UART1 defined */
#  define  Z8_C3_IRQ        22
#  define  Z8_C2_IRQ        23
#  define  Z8_C1_IRQ        24
#  define  Z8_C0_IRQ        25
#  define  Z8_POTRAP_IRQ    27
#  define  Z8_WOTRAP_IRQ    28

#  define NR_IRQS          (29)

#elif defined(ENCORE_MC_VECTORS)

#  define  Z8_WDT_IRQ        0
#  define  Z8_TRAP_IRQ       1
#  define  Z8_PWMTIMER_IRQ   2
#  define  Z8_PWMFAULT_IRQ   3
#  define  Z8_ADC_IRQ        4 /* Only if EZ8_ADC_NEW defined */
#  define  Z8_CMP_IRQ        5
#  define  Z8_TIMER0_IRQ     6
#  define  Z8_UART0_RX_IRQ   7 /* Only if EZ8_UART0 defined */
#  define  Z8_UART0_TX_IRQ   8 /* Only if EZ8_UART0 defined */
#  define  Z8_SPI_IRQ        9 /* Only if EZ8_SPI defined */
#  define  Z8_I2C_IRQ       10 /* Only if EZ8_I2C defined */
#  define  Z8_C0_IRQ        12
#  define  Z8_PB_IRQ        13
#  define  Z8_P7AP3A_IRQ    14
#  define  Z8_P6AP2A_IRQ    15
#  define  Z8_P5AP1A_IRQ    16
#  define  Z8_P4AP0A_IRQ    17
#  define  Z8_POTRAP_IRQ    27
#  define  Z8_WOTRAP_IRQ    28

#  define NR_IRQS          (29)

#endif 

#define Z8_IRQ_SYSTIMER Z8_TIMER0_IRQ

/* IRQ Management Macros ****************************************************/

/* These macros map IRQ numbers to IRQ registers and bits.
 * WARNING: These have only been verified for the Z8F640X family!
 */

#ifdef ENCORE_VECTORS

#  define Z8_IRQ0_MIN      Z8_TIMER2_IRQ
#  define Z8_IRQ0_BIT(irq) (1 << (Z8_ADC_IRQ - (irq)))
#  define Z8_IRQ0_MAX      Z8_ADC_IRQ

#  define Z8_IRQ1_MIN      Z8_P7AD_IRQ
#  define Z8_IRQ1_BIT(irq) (1 << (Z8_P0AD_IRQ - (irq)))
#  define Z8_IRQ1_MAX      Z8_P0AD_IRQ

#  define Z8_IRQ2_MIN      Z8_TIMER3_IRQ
#  define Z8_IRQ2_BIT(irq) (1 << (Z8_C0_IRQ - (irq)))
#  define Z8_IRQ2_MAX      Z8_C0_IRQ

#else
#  error "Add IRQ support for Z8F family"
#endif

/* IRQ State Save Format ****************************************************
 *
 * These indices describe how the ez8 context is save in the state save array
 *
 * Byte offsets:
 */

#define XCPT8_R0              (0) /* Offset 0-15:  R0-R15 */
#define XCPT8_R1              (1)
#define XCPT8_R2              (2)
#define XCPT8_R3              (3)
#define XCPT8_R4              (4)
#define XCPT8_R5              (5)
#define XCPT8_R6              (6)
#define XCPT8_R7              (7)
#define XCPT8_R8              (8)
#define XCPT8_R9              (9)
#define XCPT8_R10            (10)
#define XCPT8_R11            (11)
#define XCPT8_R12            (12)
#define XCPT8_R13            (13)
#define XCPT8_R14            (14)
#define XCPT8_R15            (15)
#define XCPT8_SPH            (16) /* Offset 16: SP[8:15] */
#define XCPT8_SPL            (17) /* Offset 17: SP[0:7] */
#define XCPT8_RP             (18) /* Offset 18: Register pointer */
#define XCPT8_FLAGS          (19) /* Offset 19: FLAGS */
#define XCPT8_PCH            (20) /* Offset 20: PC[8:15] */
#define XCPT8_PCL            (21) /* Offset 21: PC[0:7] */

/* 16-bit "word" offsets */

#define XCPT_RR0              (0) /* Indices 0-7:  RR0-RR14 */
#define XCPT_RR2              (1)
#define XCPT_RR4              (2)
#define XCPT_RR6              (3)
#define XCPT_RR8              (4)
#define XCPT_RR10             (5)
#define XCPT_RR12             (6)
#define XCPT_RR14             (7)
#define XCPT_IRQCTL           (8) /* Index  8: IRQCTL register */
#define XCPT_SP               (9) /* Index  9: SP[8:15] */
#define XCPT_RPFLAGS         (10) /* Index 10: RP (MS) and FLAGS (LS) */
#define XCPT_PC              (11) /* Index 11: PC[8:15] */

#define XCPTCONTEXT_REGS     (12)

/* Byte offsets: */

#define XCPT_R0_OFFS         (2*XCPT_RR0)       /* Offset 0-15:  R0-R15 */
#define XCPT_R1_OFFS         (2*XCPT_RR0+1)
#define XCPT_R2_OFFS         (2*XCPT_RR2)
#define XCPT_R3_OFFS         (2*XCPT_RR2+1)
#define XCPT_R4_OFFS         (2*XCPT_RR4)
#define XCPT_R5_OFFS         (2*XCPT_RR4+1)
#define XCPT_R6_OFFS         (2*XCPT_RR6)
#define XCPT_R7_OFFS         (2*XCPT_RR6+1)
#define XCPT_R8_OFFS         (2*XCPT_RR8)
#define XCPT_R9_OFFS         (2*XCPT_RR8+1)
#define XCPT_R10_OFFS        (2*XCPT_RR10)
#define XCPT_R11_OFFS        (2*XCPT_RR10+1)
#define XCPT_R12_OFFS        (2*XCPT_RR12)
#define XCPT_R13_OFFS        (2*XCPT_RR12+1)
#define XCPT_R14_OFFS        (2*XCPT_RR14)
#define XCPT_R15_OFFS        (2*XCPT_RR14+1)
#define XCPT_UNUSED_OFFS     (2*XCPT_IRQCTL)    /* Offset 16: Unused (zero) */
#define XCPT_IRQCTL_OFFS     (2*XCPT_IRQCTL+1)  /* offset 17: IRQCTL register */
#define XCPT_SPH_OFFS        (2*XCPT_SP)        /* Offset 18: SP[8:15] */
#define XCPT_SPL_OFFS        (2*XCPT_SP+1)      /* Offset 19: SP[0:7] */
#define XCPT_RP_OFFS         (2*XCPT_RPFLAGS)   /* Offset 20: Register pointer */
#define XCPT_FLAGS_OFFS      (2*XCPT_RPFLAGS+1) /* Offset 21: FLAGS */
#define XCPT_PCH_OFFS        (2*XCPT_PC)        /* Offset 22: PC[8:15] */
#define XCPT_PCL_OFFS        (2*XCPT_PC+1)      /* Offset 23: PC[0:7] */

#define XCPTCONTEXT_SIZE     (2*XCPTCONTEXT_REGS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This is the type of the register save array */

typedef uint16_t chipreg_t;

/* This struct defines the way the registers are stored. */

struct xcptcontext
{
  /* Register save area */

  chipreg_t regs[XCPTCONTEXT_REGS];

  /* The following function pointer is non-zero if there
   * are pending signals to be processed.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  CODE void *sigdeliver; 	/* Actual type is sig_deliver_t */

  /* The following retains that state during signal execution */

  uint16_t saved_pc;		/* Saved return address */
  uint16_t saved_irqctl;	/* Saved interrupt state */
#endif
};
#endif

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

EXTERN irqstate_t irqsave(void);
EXTERN void irqrestore(irqstate_t flags);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_Z8_IRQ_H */

