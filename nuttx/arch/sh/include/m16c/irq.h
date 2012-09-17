/************************************************************************************
 * arch/sh/include/m16c/irq.h
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_SH_INCLUDE_M16C_IRQ_H
#define __ARCH_SH_INCLUDE_M16C_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* IRQ numbers **********************************************************************/

/* Fixed vector table */

#define M16C_UNDEFINST_IRQ     0                 /* fffdc: Undefined instruction */
#define M16C_OVERFLOW_IRQ      1                 /* fffe0: Overflow */
#define M16C_BRK_IRQ           2                 /* fffe4: BRK instruction */
#define M16C_ADDRMATCH_IRQ     3                 /* fffe8: Address match */
#ifdef CONFIG_M16C_DEBUGGER
#  define M16C_SSTEP_IRQ       4                 /* fffec: Single step */
#  define M16C_WDOG_IRQ        5                 /* ffff0: Watchdog timer */
#  define M16C_DBC_IRQ         6                 /* ffff4: DBC */
#  define M16C_NMI_IRQ         7                 /* ffff8: NMI */
#  define _LAST_FIXED          7
#else
#  define M16C_WDOG_IRQ        4                 /* ffff0: Watchdog timer */
#  define M16C_NMI_IRQ         5                 /* ffff8: NMI */
#  define _LAST_FIXED          5
#endif

/* Variable vector table (fixed at address 0xffd00) */

#ifdef CONFIG_M16C_SWINTS
#  define M16C_BRK_IRQ        (_LAST_FIXED+1)    /* ffd00: BRK instruction */
#  define M16C_SWINT0_IRQ     M16C_BRK_IRQ       /*        S/W interrupt  0 */
#  define M16C_INT3_IRQ       (_LAST_FIXED+2)    /* ffd10: INT3 */
#  define M16C_SWINT4_IRQ     M16C_INT3_IRQ      /*        S/W interrupt  4 */
#  define M16C_CTXSV_SWINT    (_LAST_FIXED+3)    /* ffd14: Reserved -- used by NuttX */
#  define M16C_SWINT5_IRQ     M16C_CTXSV_SWINT   /*        S/W interrupt 5 */
#  define M16C_SWINT6_IRQ     (_LAST_FIXED+4)    /* ffd18: Reserved / S/W interrupt 6 */
#  define M16C_SWINT7_IRQ     (_LAST_FIXED+5)    /* ffd1c: Reserved / S/W interrupt 7 */
#  define M16C_INT5_IRQ       (_LAST_FIXED+6)    /* ffd20: INT5 */
#  define M16C_SWINT8_IRQ     M16C_INT5_IRQ      /*        S/W interrupt  8 */
#  define M16C_INT4_IRQ       (_LAST_FIXED+7)    /* ffd24: INT4 */
#  define M16C_SWINT9_IRQ     M16C_INT4_IRQ      /*         S/W interrupt  9 */
#  define M16C_UART2BCD_IRQ   (_LAST_FIXED+8)    /* ffd28: UART2 bus collision detection */
#  define M16C_SWINT10_IRQ    M16C_UART2BCD_IRQ  /*         S/W interrupt  10 */
#  define M16C_DMA0_IRQ       (_LAST_FIXED+9)    /* ffd2c: DMA0 */
#  define M16C_SWINT11_IRQ    M16C_DMA0_IRQ      /*         S/W interrupt  11 */
#  define M16C_DMA1_IRQ       (_LAST_FIXED+10)   /* ffd30: DMA1 */
#  define M16C_SWINT12_IRQ    M16C_DMA1_IRQ      /*         S/W interrupt  12 */
#  define M16C_KEYINP_IRQ     (_LAST_FIXED+11)   /* ffd34: Key input interrupt */
#  define M16C_SWINT13_IRQ    M16C_KEYINP_IRQ    /*         S/W interrupt 13 */
#  define M16C_ADC_IRQ        (_LAST_FIXED+12)   /* ffd38: A-D */
#  define M16C_SWINT14_IRQ    M16C_ADC_IRQ       /*         S/W interrupt  14 */
#  define M16C_UARTXNAK_IRQ   (_LAST_FIXED+13)   /* ffd3c UART2 transmit/NACK2 */
#  define M16C_SWINT15_IRQ    M16C_UARTNAK_IRQ   /*         S/W interrupt  15 */
#  define M16C_UARTRACK_IRQ   (_LAST_FIXED+14)   /* ffd40: UART2 receive/ACK2 */
#  define M16C_SWINT16_IRQ    M16C_UARTRACK_IRQ  /*         S/W interrupt 16 */
#  define M16C_UART0XMT_IRQ   (_LAST_FIXED+15)   /* ffd44: UART0 transmit */
#  define M16C_SWINT17_IRQ    M16C_UART0XMT_IRQ  /*         S/W interrupt 17 */
#  define M16C_UART0RCV_IRQ   (_LAST_FIXED+16)   /* ffd48: UART0 receive */
#  define M16C_SWINT18_IRQ    M16C_UART0RCV_IRQ  /*        S/W interrupt 18 */
#  define M16C_UART1XMT_IRQ   (_LAST_FIXED+17)   /* ffd4c: UART1 transmit */
#  define M16C_SWINT19_IRQ    M16C_UART1XMT_IRQ  /*        S/W interrupt 19 */
#  define M16C_UART1RCV_IRQ   (_LAST_FIXED+18)   /* ffd50: UART1 receive */
#  define M16C_SWINT20_IRQ    M16C_UART1RCV_IRQ  /*        S/W interrupt 20 */
#  define M16C_TMRA0_IRQ      (_LAST_FIXED+19)   /* ffd54: Timer A0 */
#  define M16C_SWINT21_IRQ    M16C_TMRA0_IRQ     /*        S/W interrupt 21 */
#  define M16C_TMRA1_IRQ      (_LAST_FIXED+20)   /* ffd58: Timer A1 */
#  define M16C_SWINT22_IRQ    M16C_TMRA1_IRQ     /*        S/W interrupt 22 */
#  define M16C_TMRA2_IRQ      (_LAST_FIXED+21)   /* ffd5c: Timer A2 */
#  define M16C_SWINT23_IRQ    M16C_TMRA2_IRQ     /*        S/W interrupt 23 */
#  define M16C_TMRA3_IRQ      (_LAST_FIXED+22)   /* ffd60: Timer A3 */
#  define M16C_SWINT24_IRQ    M16C_TMRA3_IRQ     /*        S/W interrupt 24 */
#  define M16C_TMRA4_IRQ      (_LAST_FIXED+23)   /* ffd64: Timer A4 */
#  define M16C_SWINT25_IRQ    M16C_TMRA4_IRQ     /*        S/W interrupt 25 */
#  define M16C_TMRB0_IRQ      (_LAST_FIXED+24)   /* ffd68: Timer B0 */
#  define M16C_SWINT26_IRQ    M16C_TMRB0_IRQ     /*        S/W interrupt 26 */
#  define M16C_TMRB1_IRQ      (_LAST_FIXED+25)   /* ffd6c: Timer B1 */
#  define M16C_SWINT27_IRQ    M16C_TMRB1_IRQ     /*        S/W interrupt 27 */
#  define M16C_TMRB2_IRQ      (_LAST_FIXED+26)   /* ffd70: Timer B2 */
#  define M16C_SWINT28_IRQ    M16C_TMRB2_IRQ     /*        S/W interrupt 28 */
#  define M16C_INT0_IRQ       (_LAST_FIXED+27)   /* ffd74: INT0 */
#  define M16C_SWINT29_IRQ    M16C_INT0_IRQ      /*        S/W interrupt 29 */
#  define M16C_INT1_IRQ       (_LAST_FIXED+28)   /* ffd78: INT1 */
#  define M16C_SWINT30_IRQ    M16C_INT1_IRQ      /*        S/W interrupt 30 */
#  define M16C_SWINT31_IRQ    (_LAST_FIXED+29)   /* ffd7c: Reserved / S/W interrupt 31 */
#  define M16C_CTXRSTR_SWINT  (_LAST_FIXED+30)   /* ffd80: Used by NuttX */
#  define M16C_SWINT32IRQ     M16C_CTXRSTR_SWINT /*        S/W interrupt 32 */
#  define M16C_SWINT33_IRQ    (_LAST_FIXED+31)   /* ffd84: S/W interrupt 33 */
#  define M16C_SWINT34_IRQ    (_LAST_FIXED+32)   /* ffd88: S/W interrupt 34 */
#  define M16C_SWINT35_IRQ    (_LAST_FIXED+33)   /* ffd8c: S/W interrupt 35 */
#  define M16C_SWINT36_IRQ    (_LAST_FIXED+34)   /* ffd90: S/W interrupt 36 */
#  define M16C_SWINT37_IRQ    (_LAST_FIXED+35)   /* ffd94: S/W interrupt 37 */
#  define M16C_SWINT38_IRQ    (_LAST_FIXED+36)   /* ffd98: S/W interrupt 38 */
#  define M16C_SWINT39_IRQ    (_LAST_FIXED+37)   /* ffd9c: S/W interrupt 39 */
#  define M16C_SWINT40_IRQ    (_LAST_FIXED+38)   /* ffda0: S/W interrupt 40 */
#  define M16C_SWINT41_IRQ    (_LAST_FIXED+39)   /* ffda4: S/W interrupt 41 */
#  define M16C_SWINT42_IRQ    (_LAST_FIXED+40)   /* ffda8: S/W interrupt 42 */
#  define M16C_SWINT43_IRQ    (_LAST_FIXED+41)   /* ffdac: S/W interrupt 43 */
#  define M16C_SWINT44_IRQ    (_LAST_FIXED+42)   /* ffdb0: S/W interrupt 44 */
#  define M16C_SWINT45_IRQ    (_LAST_FIXED+43)   /* ffdb4: S/W interrupt 45 */
#  define M16C_SWINT46_IRQ    (_LAST_FIXED+44)   /* ffdb8: S/W interrupt 46 */
#  define M16C_SWINT47_IRQ    (_LAST_FIXED+45)   /* ffdbc: S/W interrupt 47 */
#  define M16C_SWINT48_IRQ    (_LAST_FIXED+46)   /* ffdc0: S/W interrupt 48 */
#  define M16C_SWINT49_IRQ    (_LAST_FIXED+47)   /* ffdc4: S/W interrupt 49 */
#  define M16C_SWINT50_IRQ    (_LAST_FIXED+48)   /* ffdc8: S/W interrupt 50 */
#  define M16C_SWINT51_IRQ    (_LAST_FIXED+49)   /* ffdcc: S/W interrupt 51 */
#  define M16C_SWINT52_IRQ    (_LAST_FIXED+50)   /* ffdd0: S/W interrupt 52 */
#  define M16C_SWINT53_IRQ    (_LAST_FIXED+51)   /* ffdd4: S/W interrupt 53 */
#  define M16C_SWINT54_IRQ    (_LAST_FIXED+52)   /* ffdd8: S/W interrupt 54 */
#  define M16C_SWINT55_IRQ    (_LAST_FIXED+53)   /* ffddc: S/W interrupt 55 */
#  define M16C_SWINT56_IRQ    (_LAST_FIXED+54)   /* ffde0: S/W interrupt 56 */
#  define M16C_SWINT57_IRQ    (_LAST_FIXED+55)   /* ffde4: S/W interrupt 57 */
#  define M16C_SWINT58_IRQ    (_LAST_FIXED+56)   /* ffde8: S/W interrupt 58 */
#  define M16C_SWINT59_IRQ    (_LAST_FIXED+57)   /* ffdec: S/W interrupt 59 */
#  define M16C_SWINT60_IRQ    (_LAST_FIXED+58)   /* ffdf0: S/W interrupt 60 */
#  define M16C_SWINT61_IRQ    (_LAST_FIXED+59)   /* ffdf4: S/W interrupt 61 */
#  define M16C_SWINT62_IRQ    (_LAST_FIXED+60)   /* ffdf8: S/W interrupt 62 */
#  define M16C_SWINT63_IRQ    (_LAST_FIXED+61)   /* ffdfc: S/W interrupt 63 */

#  define NR_IRQS             (_LAST_FIXED+62)   /* Total number of supported IRQs */
#else
#  define M16C_BRK_IRQ        (_LAST_FIXED+1)    /* ffd00: BRK instruction */
#  define M16C_INT3_IRQ       (_LAST_FIXED+2)    /* ffd10: INT3 */
#  define M16C_CTXSV_SWINT    (_LAST_FIXED+3)    /* ffd14: Reserved -- SWINT5 used by NuttX */
#  define M16C_INT5_IRQ       (_LAST_FIXED+5)    /* ffd20: INT5 */
#  define M16C_INT4_IRQ       (_LAST_FIXED+6)    /* ffd24: INT4 */
#  define M16C_UART2BCD_IRQ   (_LAST_FIXED+7)    /* ffd28: UART2 bus collision detection */
#  define M16C_DMA0_IRQ       (_LAST_FIXED+8)    /* ffd2c: DMA0 */
#  define M16C_DMA1_IRQ       (_LAST_FIXED+9)    /* ffd30: DMA1 */
#  define M16C_KEYINP_IRQ     (_LAST_FIXED+10)   /* ffd34: Key input interrupt */
#  define M16C_ADC_IRQ        (_LAST_FIXED+11)   /* ffd38: A-D */
#  define M16C_UARTXNAK_IRQ   (_LAST_FIXED+12)   /* ffd3c UART2 transmit/NACK2 */
#  define M16C_UARTRACK_IRQ   (_LAST_FIXED+13)   /* ffd40: UART2 receive/ACK2 */
#  define M16C_UART0XMT_IRQ   (_LAST_FIXED+14)   /* ffd44: UART0 transmit */
#  define M16C_UART0RCV_IRQ   (_LAST_FIXED+15)   /* ffd48: UART0 receive */
#  define M16C_UART1XMT_IRQ   (_LAST_FIXED+16)   /* ffd4c: UART1 transmit */
#  define M16C_UART1RCV_IRQ   (_LAST_FIXED+17)   /* ffd50: UART1 receive */
#  define M16C_TMRA0_IRQ      (_LAST_FIXED+18)   /* ffd54: Timer A0 */
#  define M16C_TMRA1_IRQ      (_LAST_FIXED+19)   /* ffd58: Timer A1 */
#  define M16C_TMRA2_IRQ      (_LAST_FIXED+20)   /* ffd5c: Timer A2 */
#  define M16C_TMRA3_IRQ      (_LAST_FIXED+21)   /* ffd60: Timer A3 */
#  define M16C_TMRA4_IRQ      (_LAST_FIXED+22)   /* ffd64: Timer A4 */
#  define M16C_TMRB0_IRQ      (_LAST_FIXED+23)   /* ffd68: Timer B0 */
#  define M16C_TMRB1_IRQ      (_LAST_FIXED+24)   /* ffd6c: Timer B1 */
#  define M16C_TMRB2_IRQ      (_LAST_FIXED+25)   /* ffd70: Timer B2 */
#  define M16C_INT0_IRQ       (_LAST_FIXED+26)   /* ffd74: INT0 */
#  define M16C_INT1_IRQ       (_LAST_FIXED+27)   /* ffd78: INT1 */
#  define M16C_CTXRSTR_SWINT  (_LAST_FIXED+4)    /* ffd80: S/W interrupt 32, used by NuttX */

#  define NR_IRQS             (_LAST_FIXED+28)   /* Total number of supported IRQs */
#endif

/* Timer A0 is the system timer used by NuttX */

#define M16C_SYSTIMER_IRQ     M16C_TMRA0_IRQ

/* IRQ Stack Frame Format.  The M16C has a push down stack.  The CPU performs
 * the following actions when an interrupt is taken:
 *
 *  - Save FLG register
 *  - Clear I, D, and U flags in FLG register
 *  - Builds stack frame like:
 * 
 *    sp   -> PC bits 0-7
 *    sp+1 -> PC bits 8-15
 *    sp+2 -> FLG bits 0-7
 *    sp+3 -> FLG (Bits 12-14) + PC (bits 16-19)
 *
 *  - Sets IPL
 *  - Vectors to interrupt handler
 */

#define REG_FLGPCHI            0    /* 8-bit FLG (bits 12-14) + PC (bits 16-19) as would be
                                     * presented by an interrupt */
#define REG_FLG                1    /* 8-bit FLG register (bits 0-7) */
#define REG_PC                 2    /* 16-bit PC [0]:bits8-15 [1]:bits 0-7 */
#define REG_FB                 4    /* 16-bit FB register */
#define REG_SB                 6    /* 16-bit SB register */
#define REG_A1                 8    /* 16-bit A1 register */
#define REG_A0                10    /* 16-bit A0 register */
#define REG_R3                12    /* 16-bit R3 register */
#define REG_R2                14    /* 16-bit R2 register */
#define REG_R1                16    /* 16-bit R1 register */
#define REG_R0                18    /* 16-bit R0 register */
#define REG_SP                20    /* 16-bit user stack pointer */

#define XCPTCONTEXT_SIZE      22

/************************************************************************************
 * Public Types
 ************************************************************************************/
/* This struct defines the way the registers are stored.  We need to save: */

#ifndef __ASSEMBLY__
struct xcptcontext
{
  /* The following function pointer is non-zero if there are pending signals
   * to be processed.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of LR and SR used during signal processing. */

  uint8_t saved_pc[2];
  uint8_t saved_flg;
#endif

  /* Register save area */

  uint8_t regs[XCPTCONTEXT_SIZE];
};
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/* Save the current interrupt enable state & disable IRQs */

static inline irqstate_t irqsave(void)
{
  irqstate_t flags;
  __asm__ __volatile__
    (
     "\tstc	flg, %0\n"
     "\tfclr	I\n"
     : "=r" (flags)
     :
     : "memory");
  return flags;
}

/* Restore saved IRQ & FIQ state */

static inline void irqrestore(irqstate_t flags)
{
  __asm__ __volatile__
    (
     "ldc	%0, flg"
     :
     : "r" (flags)
     : "memory");
}

#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_SH_INCLUDE_M16C_IRQ_H */

