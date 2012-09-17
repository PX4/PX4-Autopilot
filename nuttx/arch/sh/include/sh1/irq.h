/************************************************************************************
 * arch/sh/include/sh1/irq.h
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
 ************************************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_SH_INCLUDE_SH1_IRQ_H
#define __ARCH_SH_INCLUDE_SH1_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* IRQ channels */

/* In the current implementation, CMON catches the following IRQ.
 * Support for traps can be provided by simply enabling the following, adding
 * vectors in sh1_head.S and adding handlers in sh1_vector.S
 */

/* Illegal instructions / Address errors */

#if 0 /* Handled by CMON */
#  define SH1_INVINSTR_IRQ  (0)                  /* General invalid instruction */
#  define SH1_INVSLOT_IRQ   (1)                  /* Invalid slot instruction */
#  define SH1_BUSERR_IRQ    (2)                  /* CPU bus error */
#  define SH1_DMAERR_IRQ    (3)                  /* DMA bus error */
#  define SH1_NMI_IRQ       (4)                  /* NMI */
#  define SH1_USRBRK_IRQ    (6)                  /* User break */
#  define SH1_TRAP_IRQBASE  (7)

#  define SH1_TRAP0_IRQ     SH1_TRAP_IRQBASE       /* TRAPA instruction (user break) */
#  define SH1_TRAP1_IRQ     (SH1_TRAP_IRQBASE+1)   /* "   " "         "  "  " "   " */
#  define SH1_TRAP2_IRQ     (SH1_TRAP_IRQBASE+2)   /* "   " "         "  "  " "   " */
#  define SH1_TRAP3_IRQ     (SH1_TRAP_IRQBASE+3)   /* "   " "         "  "  " "   " */
#  define SH1_TRAP4_IRQ     (SH1_TRAP_IRQBASE+4)   /* "   " "         "  "  " "   " */
#  define SH1_TRAP5_IRQ     (SH1_TRAP_IRQBASE+5)   /* "   " "         "  "  " "   " */
#  define SH1_TRAP6_IRQ     (SH1_TRAP_IRQBASE+6)   /* "   " "         "  "  " "   " */
#  define SH1_TRAP7_IRQ     (SH1_TRAP_IRQBASE+7)   /* "   " "         "  "  " "   " */
#  define SH1_TRAP8_IRQ     (SH1_TRAP_IRQBASE+8)   /* "   " "         "  "  " "   " */
#  define SH1_TRAP9_IRQ     (SH1_TRAP_IRQBASE+9)   /* "   " "         "  "  " "   " */
#  define SH1_TRAP10_IRQ    (SH1_TRAP_IRQBASE+10)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP11_IRQ    (SH1_TRAP_IRQBASE+11)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP12_IRQ    (SH1_TRAP_IRQBASE+12)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP13_IRQ    (SH1_TRAP_IRQBASE+13)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP14_IRQ    (SH1_TRAP_IRQBASE+14)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP15_IRQ    (SH1_TRAP_IRQBASE+15)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP16_IRQ    (SH1_TRAP_IRQBASE+16)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP17_IRQ    (SH1_TRAP_IRQBASE+17)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP18_IRQ    (SH1_TRAP_IRQBASE+18)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP19_IRQ    (SH1_TRAP_IRQBASE+19)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP20_IRQ    (SH1_TRAP_IRQBASE+20)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP21_IRQ    (SH1_TRAP_IRQBASE+21)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP22_IRQ    (SH1_TRAP_IRQBASE+22)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP23_IRQ    (SH1_TRAP_IRQBASE+23)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP24_IRQ    (SH1_TRAP_IRQBASE+24)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP25_IRQ    (SH1_TRAP_IRQBASE+25)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP26_IRQ    (SH1_TRAP_IRQBASE+26)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP27_IRQ    (SH1_TRAP_IRQBASE+27)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP28_IRQ    (SH1_TRAP_IRQBASE+28)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP29_IRQ    (SH1_TRAP_IRQBASE+29)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP30_IRQ    (SH1_TRAP_IRQBASE+30)  /* "   " "         "  "  " "   " */
#  define SH1_TRAP31_IRQ    (SH1_TRAP_IRQBASE+31)  /* "   " "         "  "  " "   " */
#  define SH1_IRQ_IRQBASE   (SH1_TRAP_IRQBASE+32)

/* Interrupts */

#  define SH1_IRQ0_IRQ      SH1_IRQ_IRQBASE      /* IRQ0 */
#  define SH1_IRQ1_IRQ      (SH1_IRQ_IRQBASE+1)  /* IRQ1 */
#  define SH1_IRQ2_IRQ      (SH1_IRQ_IRQBASE+2)  /* IRQ2 */
#  define SH1_IRQ3_IRQ      (SH1_IRQ_IRQBASE+3)  /* IRQ3 */
#  define SH1_IRQ4_IRQ      (SH1_IRQ_IRQBASE+4)  /* IRQ4 */
#  define SH1_IRQ5_IRQ      (SH1_IRQ_IRQBASE+5)  /* IRQ5 */
#  define SH1_IRQ6_IRQ      (SH1_IRQ_IRQBASE+6)  /* IRQ6 */
#  define SH1_IRQ7_IRQ      (SH1_IRQ_IRQBASE+7)  /* IRQ7 */
#  define SH1_CHIP_IRQBASE  (SH1_IRQ_IRQBASE+8)
#else
#  define SH1_CHIP_IRQBASE  (0)
#endif

/* On-chip modules -- The following may be unique to the 7032 */

#ifdef CONFIG_ARCH_CHIP_SH7032

/* DMAC */

#ifdef CONFIG_SH1_DMAC0                           /* DMAC0 */
#  define SH1_DEI0_IRQ      SH1_CHIP_IRQBASE      /*   DEI0 */
#  define SH1_DMAC1_IRQBASE (SH1_CHIP_IRQBASE+1)
#else
#  define SH1_DMAC1_IRQBASE SH1_CHIP_IRQBASE
#endif

#ifdef CONFIG_SH1_DMAC1                           /* DMAC1 */
#  define SH1_DEI1_IRQ      SH1_DMAC1_IRQBASE     /*   DEI1 */
#  define SH1_DMAC2_IRQBASE (SH1_DMAC1_IRQBASE+1)
#else
#  define SH1_DMAC2_IRQBASE SH1_DMAC1_IRQBASE
#endif

#ifdef CONFIG_SH1_DMAC2                           /* DMAC2 */
#  define SH1_DEI2_IRQ      SH1_DMAC2_IRQBASE     /*   DEI2 */
#  define SH1_DMAC3_IRQBASE (SH1_DMAC2_IRQBASE+1)
#else
#  define SH1_DMAC3_IRQBASE SH1_DMAC2_IRQBASE
#endif

#ifdef CONFIG_SH1_DMAC3                           /* DMAC3 */
#  define SH1_DEI3_IRQ      SH1_DMAC3_IRQBASE     /*   DEI3 */
#  define SH1_ITU0_IRQBASE  (SH1_DMAC3_IRQBASE+1)
#else
#  define SH1_ITU0_IRQBASE  SH1_DMAC3_IRQBASE
#endif

/* ITU */

/* ITU0 is the system clock and is always defined */

#define SH1_IMIA0_IRQ       SH1_ITU0_IRQBASE      /*   IMIA0 */
#define SH1_IMIB0_IRQ       (SH1_ITU0_IRQBASE+1)  /*   IMIB0 */
#define SH1_OVI0_IRQ        (SH1_ITU0_IRQBASE+2)  /*   OVI0 */
#define SH1_ITU1_IRQBASE    (SH1_ITU0_IRQBASE+3)

#ifdef CONFIG_SH1_ITU1                            /* ITU1 */
#  define SH1_IMIA1_IRQ     SH1_ITU1_IRQBASE      /*   IMIA1 */
#  define SH1_IMIB1_IRQ     (SH1_ITU1_IRQBASE+1)  /*   IMIB1 */
#  define SH1_OVI1_IRQ      (SH1_ITU1_IRQBASE+2)  /*   OVI1 */
#  define SH1_ITU2_IRQBASE  (SH1_ITU1_IRQBASE+3)
#else
#  define SH1_ITU2_IRQBASE  SH1_ITU1_IRQBASE
#endif

#ifdef CONFIG_SH1_ITU2                            /* ITU2 */
#  define SH1_IMIA2_IRQ     SH1_ITU2_IRQBASE      /*   IMIA2 */
#  define SH1_IMIB2_IRQ     (SH1_ITU2_IRQBASE+1)  /*   IMIB2 */
#  define SH1_OVI2_IRQ      (SH1_ITU2_IRQBASE+2)  /*   OVI2 */
#  define SH1_ITU3_IRQBASE  (SH1_ITU2_IRQBASE+3)
#else
#  define SH1_ITU3_IRQBASE  SH1_ITU2_IRQBASE
#endif

#ifdef CONFIG_SH1_ITU3                            /* ITU3 */
#  define SH1_IMIA3_IRQ     SH1_ITU3_IRQBASE      /*   IMIA3 */
#  define SH1_IMIB3_IRQ     (SH1_ITU3_IRQBASE+1)  /*   IMIB3 */
#  define SH1_OVI3_IRQ      (SH1_ITU3_IRQBASE+2)  /*   OVI3 */
#  define SH1_ITU4_IRQBASE  (SH1_ITU3_IRQBASE+3)
#else
#  define SH1_ITU4_IRQBASE  SH1_ITU3_IRQBASE
#endif

#ifdef CONFIG_SH1_ITU4                            /* ITU4 */
#  define SH1_IMIA4_IRQ     SH1_ITU4_IRQBASE      /*   IMIA4 */
#  define SH1_IMIB4_IRQ     (SH1_ITU4_IRQBASE+1)  /*   IMIB4 */
#  define SH1_OVI4_IRQ      (SH1_ITU4_IRQBASE+2)  /*   OVI4 */
#  define SH1_SCI0_IRQBASE  (SH1_ITU4_IRQBASE+3)
#else
#  define SH1_SCI0_IRQBASE  SH1_ITU4_IRQBASE
#endif

/* SCI */

#define SH1_ERI_IRQ_OFFSET (0)                                   /* ERI0 */
#define SH1_RXI_IRQ_OFFSET (1)                                   /* RxI0 */
#define SH1_TXI_IRQ_OFFSET (2)                                   /* TxI0 */
#define SH1_TEI_IRQ_OFFSET (3)                                   /* TEI0 */
#define SH1_SCI_NIRQS      (4)

#ifdef CONFIG_SH1_SCI0                                           /* SCI0 */
#  define SH1_ERI0_IRQ     (SH1_SCI0_IRQBASE+SH1_ERI_IRQ_OFFSET) /*  ERI0 */
#  define SH1_RXI0_IRQ     (SH1_SCI0_IRQBASE+SH1_RXI_IRQ_OFFSET) /*  RxI0 */
#  define SH1_TXI0_IRQ     (SH1_SCI0_IRQBASE+SH1_TXI_IRQ_OFFSET) /*  TxI0 */
#  define SH1_TEI0_IRQ     (SH1_SCI0_IRQBASE+SH1_TEI_IRQ_OFFSET) /*  TEI0 */
#  define SH1_SCI1_IRQBASE (SH1_SCI0_IRQBASE+SH1_SCI_NIRQS)
#else
#  define SH1_SCI1_IRQBASE  SH1_SCI0_IRQBASE
#endif

#ifdef CONFIG_SH1_SCI1                                           /* SCI1 */
#  define SH1_ERI1_IRQ     (SH1_SCI1_IRQBASE+SH1_ERI_IRQ_OFFSET) /*  ERI1 */
#  define SH1_RXI1_IRQ     (SH1_SCI1_IRQBASE+SH1_RXI_IRQ_OFFSET) /*  RxI1 */
#  define SH1_TXI1_IRQ     (SH1_SCI1_IRQBASE+SH1_TXI_IRQ_OFFSET) /*  TxI1 */
#  define SH1_TEI1_IRQ     (SH1_SCI1_IRQBASE+SH1_TEI_IRQ_OFFSET) /*  TEI1 */
#  define SH1_PEI_IRQBASE  (SH1_SCI1_IRQBASE+SH1_SCI_NIRQS)
#else
#  define SH1_PEI_IRQBASE  SH1_SCI1_IRQBASE
#endif

#ifdef CONFIG_SH1_PCU
#  define SH1_PEI_IRQ      SH1_PEI_IRQBASE       /* Parity control unit PEI */
#  define SH1_AD_IRQBASE   (SH1_PEI_IRQBASE+1)
#else
#  define SH1_AD_IRQBASE   SH1_PEI_IRQBASE
#endif

#ifdef CONFIG_SH1_AD
#  define SH1_ADITI_IRQ    SH1_AD_IRQBASE        /* A/D ITI */
#  define SH1_WDT_IRQBASE  (SH1_AD_IRQBASE+1)
#else
#  define SH1_WDT_IRQBASE  SH1_AD_IRQBASE
#endif

#ifdef CONFIG_SH1_WDT
#  define SH1_WDTITI_IRQ   SH1_WDT_IRQBASE       /* WDT ITI */
#  define SH1_CMI_IRQBASE  (SH1_WDT_IRQBASE+1)
#else
#  define SH1_CMI_IRQBASE  SH1_WDT_IRQBASE
#endif

#ifdef CONFIG_SH1_CMI
#  define SH1_CMI_IRQ      SH1_CMI_IRQBASE       /* REF CMI */
#  define NR_IRQS          (SH1_CMI_IRQBASE+1)   /* Total number of supported IRQs */
#else
#  define NR_IRQS          SH1_CMI_IRQBASE       /* Total number of supported IRQs */
#endif

#define SH1_SYSTIMER_IRQ   SH1_IMIA0_IRQ
#endif

/* Vector table offets **************************************************************/

/* The following provides the vector numbers for each IRQ.  The IRQ numbers (above)
 * form the densely packet number space used by the system to identify IRQs.  The
 * following are the (relatively) loosely spaced offsets that identify the location
 * of the corresponding vector in the vector table.
 *
 * These offsets are specified as a vector number (suitably for indexing an array
 * of uint32_t) but would have to by multiplied by 4 to get an addressable, byte
 * offset.
 */

/* Resets */

#define SH1_PWRONPC_VNDX   (0)   /* 0: Power-on reset (hard, NMI high) PC*/
#define SH1_PWRONSP_VNDX   (1)   /* 1: Power-on reset (hard, NMI high) SP */
#define SH1_MRESETPC_VNDX  (2)   /* 2: Power-on reset (hard, NMI high) PC*/
#define SH1_MRESETSP_VNDX  (3)   /* 3: Power-on reset (hard, NMI high) SP */

/* Illegal instructions / Address errors */

#define SH1_INVINSTR_VNDX  (4)   /* 4: General invalid instruction */
                                 /* 5: Reserved for system */
#define SH1_INVSLOT_VNDX   (6)   /* 6: Invalid slot instruction */
                                 /* 7-8: Reserved for system */
#define SH1_BUSERR_VNDX    (9)   /* 9: CPU bus error */
#define SH1_DMAERR_VNDX    (10)  /* 10: DMA bus error */

/* NMI, user break */

#define SH1_NMI_VNDX       (11)  /* 11: NMI */
#define SH1_USRBRK_VNDX    (12)  /* 12: User break */
                                 /* 13-31: Reserved for system */
/* Trap instruction */

#define SH1_TRAP_VNDX      (32)  /* 32-63: TRAPA instruction (user break) */
#define SH1_TRAP0_VNDX     (32)  /* 32: TRAPA instruction (user break) */
#define SH1_TRAP1_VNDX     (33)  /* 33: "   " "         "  "  " "   "  */
#define SH1_TRAP2_VNDX     (34)  /* 34: "   " "         "  "  " "   "  */
#define SH1_TRAP3_VNDX     (35)  /* 35: "   " "         "  "  " "   "  */
#define SH1_TRAP4_VNDX     (36)  /* 36: "   " "         "  "  " "   "  */
#define SH1_TRAP5_VNDX     (37)  /* 37: "   " "         "  "  " "   "  */
#define SH1_TRAP6_VNDX     (38)  /* 38: "   " "         "  "  " "   "  */
#define SH1_TRAP7_VNDX     (39)  /* 39: "   " "         "  "  " "   "  */
#define SH1_TRAP8_VNDX     (40)  /* 40: "   " "         "  "  " "   "  */
#define SH1_TRAP9_VNDX     (41)  /* 41: "   " "         "  "  " "   "  */
#define SH1_TRAP10_VNDX    (42)  /* 42: "   " "         "  "  " "   "  */
#define SH1_TRAP11_VNDX    (43)  /* 43: "   " "         "  "  " "   "  */
#define SH1_TRAP12_VNDX    (44)  /* 44: "   " "         "  "  " "   "  */
#define SH1_TRAP13_VNDX    (45)  /* 45: "   " "         "  "  " "   "  */
#define SH1_TRAP14_VNDX    (46)  /* 46: "   " "         "  "  " "   "  */
#define SH1_TRAP15_VNDX    (47)  /* 47: "   " "         "  "  " "   "  */
#define SH1_TRAP16_VNDX    (48)  /* 48: "   " "         "  "  " "   "  */
#define SH1_TRAP17_VNDX    (49)  /* 49: "   " "         "  "  " "   "  */
#define SH1_TRAP18_VNDX    (50)  /* 50: "   " "         "  "  " "   "  */
#define SH1_TRAP19_VNDX    (51)  /* 51: "   " "         "  "  " "   "  */
#define SH1_TRAP20_VNDX    (52)  /* 52: "   " "         "  "  " "   "  */
#define SH1_TRAP21_VNDX    (53)  /* 53: "   " "         "  "  " "   "  */
#define SH1_TRAP22_VNDX    (54)  /* 54: "   " "         "  "  " "   "  */
#define SH1_TRAP23_VNDX    (55)  /* 55: "   " "         "  "  " "   "  */
#define SH1_TRAP24_VNDX    (56)  /* 56: "   " "         "  "  " "   "  */
#define SH1_TRAP25_VNDX    (57)  /* 57: "   " "         "  "  " "   "  */
#define SH1_TRAP26_VNDX    (58)  /* 58: "   " "         "  "  " "   "  */
#define SH1_TRAP27_VNDX    (59)  /* 59: "   " "         "  "  " "   "  */
#define SH1_TRAP28_VNDX    (60)  /* 60: "   " "         "  "  " "   "  */
#define SH1_TRAP29_VNDX    (61)  /* 61: "   " "         "  "  " "   "  */
#define SH1_TRAP30_VNDX    (62)  /* 62: "   " "         "  "  " "   "  */
#define SH1_TRAP31_VNDX    (63)  /* 63: "   " "         "  "  " "   "  */

/* Interrupts */

#define SH1_IRQ_VNDX       (64)  /* 64-71: IRQ0-7 */
#define SH1_IRQ0_VNDX      (64)  /* 64: IRQ0 */
#define SH1_IRQ1_VNDX      (65)  /* 65: IRQ1 */
#define SH1_IRQ2_VNDX      (66)  /* 66: IRQ2 */
#define SH1_IRQ3_VNDX      (67)  /* 67: IRQ3 */
#define SH1_IRQ4_VNDX      (68)  /* 68: IRQ4 */
#define SH1_IRQ5_VNDX      (69)  /* 69: IRQ5 */
#define SH1_IRQ6_VNDX      (70)  /* 70: IRQ6 */
#define SH1_IRQ7_VNDX      (71)  /* 71: IRQ7 */

#define SH1_LASTCMN_VNDX   (71)
#define SH1_NCMN_VECTORS   (72)

/* On-chip modules -- The following may be unique to the 7032 */

#ifdef CONFIG_ARCH_CHIP_SH7032

/* DMAC */

#define SH1_DMAC0_VNDX     (72)  /* 72-73: DMAC0 */
#define SH1_DEI0_VNDX      (72)  /* 72: DMAC0 DEI0 */
                                 /* 73: Reserved */
#define SH1_DMAC1_VNDX     (74)  /* 74-75: DMAC1 */
#define SH1_DEI1_VNDX      (74)  /* 74: DMAC1 DEI1 */
                                 /* 75: Reserved */
#define SH1_DMAC2_VNDX     (76)  /* 76-77: DMAC2 */
#define SH1_DEI2_VNDX      (76)  /* 76: DMAC2 DEI2 */
                                 /* 77: Reserved */
#define SH1_DMAC3_VNDX     (78)  /* 78-79: DMAC3 */
#define SH1_DEI3_VNDX      (78)  /* 78: DMAC3 DEI3 */
                                 /* 79: Reserved */
/* ITU */

#define SH1_IMIA0_VNDX     (80)  /* 80: ITU0 IMIA0 */
#define SH1_IMIB0_VNDX     (81)  /* 81:      IMIB0 */
#define SH1_OVI0_VNDX      (82)  /* 82:      OVI0 */
                                 /* 83:      Reserved */
#define SH1_IMIA1_VNDX     (84)  /* 84: ITU1 IMIA1 */
#define SH1_IMIB1_VNDX     (85)  /* 85:      IMIB1 */
#define SH1_OVI1_VNDX      (86)  /* 86:      OVI1 */
                                 /* 87:      Reserved */
#define SH1_IMIA2_VNDX     (88)  /* 88: ITU2 IMIA2 */
#define SH1_IMIB2_VNDX     (89)  /* 89:      IMIB2 */
#define SH1_OVI2_VNDX      (90)  /* 90:      OVI2 */
                                 /* 91:      Reserved */
#define SH1_IMIA3_VNDX     (92)  /* 92: ITU3 IMIA3 */
#define SH1_IMIB3_VNDX     (93)  /* 93:      IMIB3 */
#define SH1_OVI3_VNDX      (94)  /* 94:      OVI3 */
                                 /* 95:      Reserved */
#define SH1_IMIA4_VNDX     (96)  /* 96: ITU4 IMIA4 */
#define SH1_IMIB4_VNDX     (97)  /* 97:      IMIB4 */
#define SH1_OVI4_VNDX      (98)  /* 98:      OVI4 */
                                 /* 99:      Reserved */
/* SCI */

#define SH1_ERI0_VNDX      (100) /* 100: SCI0 ERI0 */
#define SH1_RXI0_VNDX      (101) /* 101:      RxI0 */
#define SH1_TXI0_VNDX      (102) /* 102:      TxI0 */
#define SH1_TEI0_VNDX      (103) /* 103:      TEI0 */

#define SH1_ERI1_VNDX      (104) /* 104: SCI1 ERI1 */
#define SH1_RXI1_VNDX      (105) /* 105:      RxI1 */
#define SH1_TXI1_VNDX      (106) /* 106:      TxI1 */
#define SH1_TEI1_VNDX      (107) /* 107:      TEI1 */

#define SH1_PEI_VNDX       (108) /* 108: Parity control unit PEI */
#define SH1_ADITI_VNDX     (109) /* 109: A/D ITI */
                                 /* 110-111: Reserved */
#define SH1_WDTITI_VNDX    (112) /* 112: WDT ITI */
#define SH1_CMI_VNDX       (113) /* 113: REF CMI */
                                 /* 114-115: Reserved */
/* 116-255 reserved */
#endif

#define SH1_LAST_VNDX      (255)
#define SH1_NVECTORS       (256)

/* IRQ Stack Frame Format.  The SH-1 has a push down stack.  The PC
 * and SR are pushed by hardware at the time an IRQ is taken.
 */

/* Saved to the stacked by up_vector */

#define REG_R8              (0)
#define REG_R9              (1)
#define REG_R10             (2)
#define REG_R11             (3)
#define REG_R12             (4)
#define REG_R13             (5)
#define REG_R14             (6)

#define REG_PR              (7)
#define REG_GBR             (8)

/* The value of the stack pointer *before* the interrupt occurred */

#define REG_R15             (9)
#define REG_SP              REG_R15

/* These registers do not need to be preserved by up_saveusercontext */

#define REG_MACL            (10)
#define REG_MACH            (11)
#define REG_R0              (12)
#define REG_R1              (13)
#define REG_R2              (14)
#define REG_R3              (15)
#define REG_R5              (16)
#define REG_R6              (17)
#define REG_R7              (18)

/* Saved to the stack by the trampoline logic */

#define REG_R4              (19)

/* Pushed by hardware when the exception is taken */

#define REG_PC              (20)
#define REG_SR              (21)

#define XCPTCONTEXT_REGS    (22)
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

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

  uint32_t saved_pc;
  uint32_t saved_sr;
#endif

  /* Register save area */

  uint32_t regs[XCPTCONTEXT_REGS];
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

/* Get the current value of the SR */

static inline irqstate_t __getsr(void)
{
  irqstate_t flags;

  __asm__ __volatile__ ("stc sr, %0" : "=r" (flags));
  return flags;
}

/* Set the new value of the SR */

static inline void __setsr(irqstate_t sr)
{
  __asm__ __volatile__ ("ldc %0, sr" : : "r" (sr));
}

/* Return the current interrupt enable state & disable IRQs */

static inline irqstate_t irqsave(void)
{
  irqstate_t flags = __getsr();
  __setsr(flags | 0x000000f0);
  return flags;
}

/* Disable IRQs */

static inline void irqdisable(void)
{
  uint32_t flags = __getsr();
  __setsr(flags | 0x000000f0);
}
/* Enable IRQs */

static inline void irqenable(void)
{
  uint32_t flags = __getsr();
  __setsr(flags & ~0x000000f0);
}

/* Restore saved IRQ state */

static inline void irqrestore(irqstate_t flags)
{
  if ((flags & 0x000000f0) != 0x000000f0)
    {
      irqenable();
    }
  else
    {
      irqdisable();
    }
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

#endif /* __ARCH_SH_INCLUDE_SH1_IRQ_H */

