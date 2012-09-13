/************************************************************************************
 * arch/arm/src/kinetis/kinetis_slcd.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_SLCD_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_SLCD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_LCD_GCR_OFFSET       0x0000 /* LCD general control register */
#define KINETIS_LCD_AR_OFFSET        0x0004 /* LCD auxiliary register */
#define KINETIS_LCD_FDCR_OFFSET      0x0008 /* LCD fault detect control register */
#define KINETIS_LCD_FDSR_OFFSET      0x000c /* LCD fault detect status register */
#define KINETIS_LCD_PENL_OFFSET      0x0010 /* LCD pin enable register */
#define KINETIS_LCD_PENH_OFFSET      0x0014 /* LCD pin enable register */
#define KINETIS_LCD_BPENL_OFFSET     0x0018 /* LCD backplane enable register */
#define KINETIS_LCD_BPENH_OFFSET     0x001c /* LCD backplane enable register */
#define KINETIS_LCD_WF3TO0_OFFSET    0x0020 /* LCD waveform register */
#define KINETIS_LCD_WF7TO4_OFFSET    0x0024 /* LCD waveform register */
#define KINETIS_LCD_WF11TO8_OFFSET   0x0028 /* LCD waveform register */
#define KINETIS_LCD_WF15TO12_OFFSET  0x002c /* LCD waveform register */
#define KINETIS_LCD_WF19TO16_OFFSET  0x0030 /* LCD waveform register */
#define KINETIS_LCD_WF23TO20_OFFSET  0x0034 /* LCD waveform register */
#define KINETIS_LCD_WF27TO24_OFFSET  0x0038 /* LCD waveform register */
#define KINETIS_LCD_WF31TO28_OFFSET  0x003c /* LCD waveform register */
#define KINETIS_LCD_WF35TO32_OFFSET  0x0040 /* LCD waveform register */
#define KINETIS_LCD_WF39TO36_OFFSET  0x0044 /* LCD waveform register */
#define KINETIS_LCD_WF43TO40_OFFSET  0x0048 /* LCD waveform register */
#define KINETIS_LCD_WF47TO44_OFFSET  0x004c /* LCD waveform register */
#define KINETIS_LCD_WF51TO48_OFFSET  0x0050 /* LCD waveform register */
#define KINETIS_LCD_WF55TO52_OFFSET  0x0054 /* LCD waveform register */
#define KINETIS_LCD_WF59TO56_OFFSET  0x0058 /* LCD waveform register */
#define KINETIS_LCD_WF63TO60_OFFSET  0x005C /* LCD waveform register */

/* Register Addresses ***************************************************************/

#define KINETIS_LCD_GCR              (KINETIS_SLCD_BASE+KINETIS_LCD_GCR_OFFSET)
#define KINETIS_LCD_AR               (KINETIS_SLCD_BASE+KINETIS_LCD_AR_OFFSET)
#define KINETIS_LCD_FDCR             (KINETIS_SLCD_BASE+KINETIS_LCD_FDCR_OFFSET)
#define KINETIS_LCD_FDSR             (KINETIS_SLCD_BASE+KINETIS_LCD_FDSR_OFFSET)
#define KINETIS_LCD_PENL             (KINETIS_SLCD_BASE+KINETIS_LCD_PENL_OFFSET)
#define KINETIS_LCD_PENH             (KINETIS_SLCD_BASE+KINETIS_LCD_PENH_OFFSET)
#define KINETIS_LCD_BPENL            (KINETIS_SLCD_BASE+KINETIS_LCD_BPENL_OFFSET)
#define KINETIS_LCD_BPENH            (KINETIS_SLCD_BASE+KINETIS_LCD_BPENH_OFFSET)
#define KINETIS_LCD_WF3TO0           (KINETIS_SLCD_BASE+KINETIS_LCD_WF3TO0_OFFSET)
#define KINETIS_LCD_WF7TO4           (KINETIS_SLCD_BASE+KINETIS_LCD_WF7TO4_OFFSET)
#define KINETIS_LCD_WF11TO8          (KINETIS_SLCD_BASE+KINETIS_LCD_WF11TO8_OFFSET)
#define KINETIS_LCD_WF15TO12         (KINETIS_SLCD_BASE+KINETIS_LCD_WF15TO12_OFFSET)
#define KINETIS_LCD_WF19TO16         (KINETIS_SLCD_BASE+KINETIS_LCD_WF19TO16_OFFSET)
#define KINETIS_LCD_WF23TO20         (KINETIS_SLCD_BASE+KINETIS_LCD_WF23TO20_OFFSET)
#define KINETIS_LCD_WF27TO24         (KINETIS_SLCD_BASE+KINETIS_LCD_WF27TO24_OFFSET)
#define KINETIS_LCD_WF31TO28         (KINETIS_SLCD_BASE+KINETIS_LCD_WF31TO28_OFFSET)
#define KINETIS_LCD_WF35TO32         (KINETIS_SLCD_BASE+KINETIS_LCD_WF35TO32_OFFSET)
#define KINETIS_LCD_WF39TO36         (KINETIS_SLCD_BASE+KINETIS_LCD_WF39TO36_OFFSET)
#define KINETIS_LCD_WF43TO40         (KINETIS_SLCD_BASE+KINETIS_LCD_WF43TO40_OFFSET)
#define KINETIS_LCD_WF47TO44         (KINETIS_SLCD_BASE+KINETIS_LCD_WF47TO44_OFFSET)
#define KINETIS_LCD_WF51TO48         (KINETIS_SLCD_BASE+KINETIS_LCD_WF51TO48_OFFSET)
#define KINETIS_LCD_WF55TO52         (KINETIS_SLCD_BASE+KINETIS_LCD_WF55TO52_OFFSET)
#define KINETIS_LCD_WF59TO56         (KINETIS_SLCD_BASE+KINETIS_LCD_WF59TO56_OFFSET)
#define KINETIS_LCD_WF63TO60         (KINETIS_SLCD_BASE+KINETIS_LCD_WF63TO60_OFFSET)

/* Register Bit Definitions *********************************************************/

/* LCD general control register */

#define LCD_GCR_DUTYSHIFT            (0)       /* Bits 0-2: LCD duty select */
#define LCD_GCR_DUTY_MASK            (7 << LCD_GCR_DUTYSHIFT)
#  define LCD_GCR_DUTY_BP(n)         (((n)-1) << LCD_GCR_DUTYSHIFT) /* Use n BP (1/n duty cyle) */
#define LCD_GCR_LCLK_SHIFT           (3)       /* Bits 3-5: LCD clock prescaler */
#define LCD_GCR_LCLK_MASK            (7 << LCD_GCR_LCLK_SHIFT)
#define LCD_GCR_SOURCE               (1 << 6)  /* Bit 6:  LCD clock source select */
#define LCD_GCR_LCDEN                (1 << 7)  /* Bit 7:  LCD driver enable */
#define LCD_GCR_LCDSTP               (1 << 8)  /* Bit 8:  Stop mode */
#define LCD_GCR_LCDWAIT              (1 << 9)  /* Bit 9:  Wait mode */
                                               /* Bits 10-11: Reserved */
#define LCD_GCR_ALTDIV_SHIFT         (12)      /* Bits 12-13: LCD alternate clock divider */
#define LCD_GCR_ALTDIV_MASK          (3 << LCD_GCR_ALTDIV_SHIFT)
#  define LCD_GCR_ALTDIV_DIV         (0 << LCD_GCR_ALTDIV_SHIFT) /* Divide factor = 1 (No divide) */
#  define LCD_GCR_ALTDIV_DIV         (1 << LCD_GCR_ALTDIV_SHIFT) /* Divide factor = 8 */
#  define LCD_GCR_ALTDIV_DIV         (2 << LCD_GCR_ALTDIV_SHIFT) /* Divide factor = 64 */
#  define LCD_GCR_ALTDIV_DIV         (3 << LCD_GCR_ALTDIV_SHIFT) /* Divide factor = 512 */
#define LCD_GCR_FDCIEN               (1 << 14) /* Bit 14: LCD fault detection complete interrupt enable */
#define LCD_GCR_LCDIEN               (1 << 15) /* Bit 15: LCD frame frequency interrupt enable */
#define LCD_GCR_VSUPPLY_SHIFT        (16)      /* Bits 16-17: Voltage supply control */
#define LCD_GCR_VSUPPLY_MASK         (3 << LCD_GCR_VSUPPLY_SHIFT)
#define LCD_GCR_VSUPPLY_INTVLL2      (0 << LCD_GCR_VSUPPLY_SHIFT) /* Drive VLL2 internally from VDD */
#define LCD_GCR_VSUPPLY_INTVLL3      (1 << LCD_GCR_VSUPPLY_SHIFT) /* Drive VLL3 internally from VDD */
#define LCD_GCR_VSUPPLY_EXTVLL3      (3 << LCD_GCR_VSUPPLY_SHIFT) /* Drive VLL3 externally from VDD */
#define LCD_GCR_VSUPPLY_INTVLL1      (3 << LCD_GCR_VSUPPLY_SHIFT) /* Drive VLL1 internally from VIREG */
                                               /* Bits 18-19: Reserved */
#define LCD_GCR_LADJ_SHIFT           (20)      /* Bits 20-21: Load adjust */
#define LCD_GCR_LADJ_MASK            (3 << LCD_GCR_LADJ_SHIFT)
#  define LCD_GCR_LADJ_LOW           (0 << LCD_GCR_LADJ_SHIFT) /* For CPSEL=0, Low load <=2000pF */
#  define LCD_GCR_LADJ_MIDLOW        (1 << LCD_GCR_LADJ_SHIFT) /* For CPSEL=0, Low load <=2000pF */
#  define LCD_GCR_LADJ_MIDHIGH       (2 << LCD_GCR_LADJ_SHIFT) /* For CPSEL=0, High load <=8000pF */
#  define LCD_GCR_LADJ_HIGH          (3 << LCD_GCR_LADJ_SHIFT) /* For CPSEL=0, High load <=8000pF */
#  define LCD_GCR_LADJ_FAST          (0 << LCD_GCR_LADJ_SHIFT) /* For CPSEL=1, <=8000pF */
#  define LCD_GCR_LADJ_MIDFAST       (1 << LCD_GCR_LADJ_SHIFT) /* For CPSEL=1, <=6000pF */
#  define LCD_GCR_LADJ_MIDSLOW       (2 << LCD_GCR_LADJ_SHIFT) /* For CPSEL=1, <=4000pF */
#  define LCD_GCR_LADJ_SLOW          (3 << LCD_GCR_LADJ_SHIFT) /* For CPSEL=1, <=2000pF */
#define LCD_GCR_HREFSEL              (1 << 22) /* Bit 22: High reference select */
#define LCD_GCR_CPSEL                (1 << 23) /* Bit 23: Charge pump or resistor bias select */
#define LCD_GCR_RVTRIM_SHIFT         (24)      /* Bits 24-27: Regulated voltage trim */
#define LCD_GCR_RVTRIM_MASK          (15 << LCD_GCR_RVTRIM_SHIFT)
                                               /* Bits 28-30: Reserved */
#define LCD_GCR_RVEN                 (1 << 31) /* Bit 31: Regulated voltage enable */

/* LCD auxiliary register */

#define LCD_AR_BRATE_SHIFT           (0)       /* Bits 0-2: Blink-rate configuration */
#define LCD_AR_BRATE_MASK            (7 << LCD_AR_BRATE_SHIFT)
#define LCD_AR_BMODE                 (1 << 3)  /* Bit 3:  Blink mode */
                                               /* Bit 4: Reserved */
#define LCD_AR_BLANK                 (1 << 5)  /* Bit 5:  Blank display mode
#define LCD_AR_ALT                   (1 << 6)  /* Bit 6:  Alternate display mode */
#define LCD_AR_BLINK                 (1 << 7)  /* Bit 7:  Blink command */
                                               /* Bits 8-14: Reserved */
#define LCD_AR_LCDIF                 (1 << 15) /* Bit 15: LCD frame frequency interrupt flag */
                                               /* Bits 16-31: Reserved */
/* LCD fault detect control register */

#define LCD_FDCR_FDPINID_SHIFT       (0)       /* Bits 0-5: Fault detect pin ID */
#define LCD_FDCR_FDPINID_MASK        (63 << LCD_FDCR_FDPINID_SHIFT)
#define LCD_FDCR_FDBPEN              (1 << 6)  /* Bit 6:  Fault detect backplane enable */
#define LCD_FDCR_FDEN                (1 << 7)  /* Bit 7:  Fault detect enable */
                                               /* Bit 8: Reserved */
#define LCD_FDCR_FDSWW_SHIFT         (9)       /* Bits 9-11: Fault detect sample window width */
#define LCD_FDCR_FDSWW_MASK          (7 << LCD_FDCR_FDSWW_SHIFT)
#define LCD_FDCR_FDPRS_SHIFT         (12)      /* Bits 12-14: Fault detect clock prescaler */
#define LCD_FDCR_FDPRS_MASK          (7 << LCD_FDCR_FDPRS_SHIFT)
#  define LCD_FDCR_FDPRS_DIV1        (0 << LCD_FDCR_FDPRS_SHIFT) /* Bus clock */
#  define LCD_FDCR_FDPRS_DIV2        (1 << LCD_FDCR_FDPRS_SHIFT) /* 1/2 bus clock */
#  define LCD_FDCR_FDPRS_DIV4        (2 << LCD_FDCR_FDPRS_SHIFT) /* 1/4 bus clock */
#  define LCD_FDCR_FDPRS_DIV8        (3 << LCD_FDCR_FDPRS_SHIFT) /* 1/8 bus clock */
#  define LCD_FDCR_FDPRS_DIV16       (4 << LCD_FDCR_FDPRS_SHIFT) /* 1/16 bus clock */
#  define LCD_FDCR_FDPRS_DIV32       (5 << LCD_FDCR_FDPRS_SHIFT) /* 1/32 bus clock */
#  define LCD_FDCR_FDPRS_DIV64       (6 << LCD_FDCR_FDPRS_SHIFT) /* 1/64 bus clock */
#  define LCD_FDCR_FDPRS_DIV128      (7 << LCD_FDCR_FDPRS_SHIFT) /* 1/128 bus clock */
                                               /* Bits 15-31: Reserved */
/* LCD fault detect status register */

#define LCD_FDSR_FDCNT_SHIFT         (0)       /* Bits 0-7: Fault detect counter */
#define LCD_FDSR_FDCNT_MASK          (0xff << LCD_FDSR_FDCNT_SHIFT)
                                               /* Bits 8-14: Reserved */
#define LCD_FDSR_FDCF                (1 << 15) /* Bit 15: Fault detection complete flag */
                                               /* Bits 16-31: Reserved */
/* LCD pin enable register low/high (64 pin bits in two 32-bit registers) */


/* LCD backplane enable register (64 pin bits in two 32-bit registers) */

#define LCD_BPENL(n)                 (1 << (n)) /* Bit n: Enable backplane operation pin n, n=0-31 */
#define LCD_BPENH(n)                 (1 << ((n)-32)) /* Bit n-32: Enable backplane operation pin n, n=32-63 */

/* LCD waveform registers */

#define LCD_WF3TO0_WF0_SHIFT           (0)        /* Bits 0-7: Waveform control field 0 segment bits */
#define LCD_WF3TO0_WF0_MASK            (0xff << LCD_WF3TO0_WF0_SHIFT)
#  define LCD_WF3TO0_WF0_SEGMENT(n)    ((1 << (n)) << LCD_WF3TO0_WF0_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF3TO0_WF1_SHIFT           (8)        /* Bits 8-15: Waveform control field 1 segment bits */
#define LCD_WF3TO0_WF1_MASK            (0xff << LCD_WF3TO0_WF1_SHIFT)
#  define LCD_WF3TO0_WF1_SEGMENT(n)    ((1 << (n)) << LCD_WF3TO0_WF1_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF3TO0_WF2_SHIFT           (16)       /* Bits 16-23: Waveform control field 2 segment bits */
#define LCD_WF3TO0_WF2_MASK            (0xff << LCD_WF3TO0_WF2_SHIFT)
#  define LCD_WF3TO0_WF2_SEGMENT(n)    ((1 << (n)) << LCD_WF3TO0_WF2_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF3TO0_WF3_SHIFT           (24)       /* Bits 24-31: Waveform control field 3 segment bits */
#define LCD_WF3TO0_WF3_MASK            (0xff << LCD_WF3TO0_WF3_SHIFT)
#  define LCD_WF3TO0_WF3_SEGMENT(n)    ((1 << (n)) << LCD_WF3TO0_WF3_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF7TO4_WF4_SHIFT           (0)        /* Bits 0-7: Waveform control field 4 segment bits */
#define LCD_WF7TO4_WF4_MASK            (0xff << LCD_WF7TO4_WF4_SHIFT)
#  define LCD_WF7TO4_WF4_SEGMENT(n)    ((1 << (n)) << LCD_WF7TO4_WF4_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF7TO4_WF5_SHIFT           (8)        /* Bits 8-15: Waveform control field 5 segment bits */
#define LCD_WF7TO4_WF5_MASK            (0xff << LCD_WF7TO4_WF5_SHIFT)
#  define LCD_WF7TO4_WF5_SEGMENT(n)    ((1 << (n)) << LCD_WF7TO4_WF5_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF7TO4_WF6_SHIFT           (16)       /* Bits 16-23: Waveform control field 6 segment bits */
#define LCD_WF7TO4_WF6_MASK            (0xff << LCD_WF7TO4_WF6_SHIFT)
#  define LCD_WF7TO4_WF6_SEGMENT(n)    ((1 << (n)) << LCD_WF7TO4_WF6_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF7TO4_WF7_SHIFT           (24)       /* Bits 24-31: Waveform control field 7 segment bits */
#define LCD_WF7TO4_WF7_MASK            (0xff << LCD_WF7TO4_WF7_SHIFT)
#  define LCD_WF7TO4_WF7_SEGMENT(n)    ((1 << (n)) << LCD_WF7TO4_WF7_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF11TO8_WF8_SHIFT          (0)        /* Bits 0-7: Waveform control field 8 segment bits */
#define LCD_WF11TO8_WF8_MASK           (0xff << LCD_WF11TO8_WF8_SHIFT)
#  define LCD_WF11TO8_WF8_SEGMENT(n)   ((1 << (n)) << LCD_WF11TO8_WF8_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF11TO8_WF9_SHIFT          (8)        /* Bits 8-15: Waveform control field 9 segment bits */
#define LCD_WF11TO8_WF9_MASK           (0xff << LCD_WF11TO8_WF9_SHIFT)
#  define LCD_WF11TO8_WF9_SEGMENT(n)   ((1 << (n)) << LCD_WF11TO8_WF9_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF11TO8_WF10_SHIFT         (16)       /* Bits 16-23: Waveform control field 10 segment bits */
#define LCD_WF11TO8_WF10_MASK          (0xff << LCD_WF11TO8_WF10_SHIFT)
#  define LCD_WF11TO8_WF10_SEGMENT(n)  ((1 << (n)) << LCD_WF11TO8_WF10_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF11TO8_WF11_SHIFT         (24)       /* Bits 24-31: Waveform control field 11 segment bits */
#define LCD_WF11TO8_WF11_MASK          (0xff << LCD_WF11TO8_WF11_SHIFT)
#  define LCD_WF11TO8_WF11_SEGMENT(n)  ((1 << (n)) << LCD_WF11TO8_WF11_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF15TO12_WF12_SHIFT        (0)        /* Bits 0-7: Waveform control field 12 segment bits */
#define LCD_WF15TO12_WF12_MASK         (0xff << LCD_WF15TO12_WF12_SHIFT)
#  define LCD_WF15TO12_WF12_SEGMENT(n) ((1 << (n)) << LCD_WF15TO12_WF12_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF15TO12_WF13_SHIFT        (8)        /* Bits 8-15: Waveform control field 13 segment bits */
#define LCD_WF15TO12_WF13_MASK         (0xff << LCD_WF15TO12_WF13_SHIFT)
#  define LCD_WF15TO12_WF13_SEGMENT(n) ((1 << (n)) << LCD_WF15TO12_WF13_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF15TO12_WF14_SHIFT        (16)       /* Bits 16-23: Waveform control field 14 segment bits */
#define LCD_WF15TO12_WF14_MASK         (0xff << LCD_WF15TO12_WF14_SHIFT)
#  define LCD_WF15TO12_WF14_SEGMENT(n) ((1 << (n)) << LCD_WF15TO12_WF14_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF15TO12_WF15_SHIFT        (24)       /* Bits 24-31: Waveform control field 15 segment bits */
#define LCD_WF15TO12_WF15_MASK         (0xff << LCD_WF15TO12_WF15_SHIFT)
#  define LCD_WF15TO12_WF15_SEGMENT(n) ((1 << (n)) << LCD_WF15TO12_WF15_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF19TO16_WF16_SHIFT        (0)        /* Bits 0-7: Waveform control field 16 segment bits */
#define LCD_WF19TO16_WF16_MASK         (0xff << LCD_WF19TO16_WF16_SHIFT)
#  define LCD_WF19TO16_WF16_SEGMENT(n) ((1 << (n)) << LCD_WF19TO16_WF16_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF19TO16_WF17_SHIFT        (8)        /* Bits 8-15: Waveform control field 17 segment bits */
#define LCD_WF19TO16_WF17_MASK         (0xff << LCD_WF19TO16_WF17_SHIFT)
#  define LCD_WF19TO16_WF17_SEGMENT(n) ((1 << (n)) << LCD_WF19TO16_WF17_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF19TO16_WF18_SHIFT        (16)       /* Bits 16-23: Waveform control field 18 segment bits */
#define LCD_WF19TO16_WF18_MASK         (0xff << LCD_WF19TO16_WF18_SHIFT)
#  define LCD_WF19TO16_WF18_SEGMENT(n) ((1 << (n)) << LCD_WF19TO16_WF18_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF19TO16_WF19_SHIFT        (24)       /* Bits 24-31: Waveform control field 19 segment bits */
#define LCD_WF19TO16_WF19_MASK         (0xff << LCD_WF19TO16_WF19_SHIFT)
#  define LCD_WF19TO16_WF19_SEGMENT(n) ((1 << (n)) << LCD_WF19TO16_WF19_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF23TO20_WF20_SHIFT        (0)        /* Bits 0-7: Waveform control field 20 segment bits */
#define LCD_WF23TO20_WF20_MASK         (0xff << LCD_WF23TO20_WF20_SHIFT)
#  define LCD_WF23TO20_WF20_SEGMENT(n) ((1 << (n)) << LCD_WF23TO20_WF20_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF23TO20_WF21_SHIFT        (8)        /* Bits 8-15: Waveform control field 21 segment bits */
#define LCD_WF23TO20_WF21_MASK         (0xff << LCD_WF23TO20_WF21_SHIFT)
#  define LCD_WF23TO20_WF21_SEGMENT(n) ((1 << (n)) << LCD_WF23TO20_WF21_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF23TO20_WF22_SHIFT        (16)       /* Bits 16-23: Waveform control field 22 segment bits */
#define LCD_WF23TO20_WF22_MASK         (0xff << LCD_WF23TO20_WF22_SHIFT)
#  define LCD_WF23TO20_WF22_SEGMENT(n) ((1 << (n)) << LCD_WF23TO20_WF22_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF23TO20_WF23_SHIFT        (24)       /* Bits 24-31: Waveform control field 23 segment bits */
#define LCD_WF23TO20_WF23_MASK         (0xff << LCD_WF23TO20_WF23_SHIFT)
#  define LCD_WF23TO20_WF23_SEGMENT(n) ((1 << (n)) << LCD_WF23TO20_WF23_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF27TO24_WF24_SHIFT        (0)        /* Bits 0-7: Waveform control field 24 segment bits */
#define LCD_WF27TO24_WF24_MASK         (0xff << LCD_WF27TO24_WF24_SHIFT)
#  define LCD_WF27TO24_WF24_SEGMENT(n) ((1 << (n)) << LCD_WF27TO24_WF24_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF27TO24_WF25_SHIFT        (8)        /* Bits 8-15: Waveform control field 25 segment bits */
#define LCD_WF27TO24_WF25_MASK         (0xff << LCD_WF27TO24_WF25_SHIFT)
#  define LCD_WF27TO24_WF25_SEGMENT(n) ((1 << (n)) << LCD_WF27TO24_WF25_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF27TO24_WF26_SHIFT        (16)       /* Bits 16-23: Waveform control field 26 segment bits */
#define LCD_WF27TO24_WF26_MASK         (0xff << LCD_WF27TO24_WF26_SHIFT)
#  define LCD_WF27TO24_WF26_SEGMENT(n) ((1 << (n)) << LCD_WF27TO24_WF26_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF27TO24_WF27_SHIFT        (24)       /* Bits 24-31: Waveform control field 27 segment bits */
#define LCD_WF27TO24_WF27_MASK         (0xff << LCD_WF27TO24_WF27_SHIFT)
#  define LCD_WF27TO24_WF27_SEGMENT(n) ((1 << (n)) << LCD_WF27TO24_WF27_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF31TO28_WF28_SHIFT        (0)        /* Bits 0-7: Waveform control field 28 segment bits */
#define LCD_WF31TO28_WF28_MASK         (0xff << LCD_WF31TO28_WF28_SHIFT)
#  define LCD_WF31TO28_WF28_SEGMENT(n) ((1 << (n)) << LCD_WF31TO28_WF28_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF31TO28_WF29_SHIFT        (8)        /* Bits 8-15: Waveform control field 29 segment bits */
#define LCD_WF31TO28_WF29_MASK         (0xff << LCD_WF31TO28_WF29_SHIFT)
#  define LCD_WF31TO28_WF29_SEGMENT(n) ((1 << (n)) << LCD_WF31TO28_WF29_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF31TO28_WF30_SHIFT        (16)       /* Bits 16-23: Waveform control field 30 segment bits */
#define LCD_WF31TO28_WF30_MASK         (0xff << LCD_WF31TO28_WF30_SHIFT)
#  define LCD_WF31TO28_WF30_SEGMENT(n) ((1 << (n)) << LCD_WF31TO28_WF30_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF31TO28_WF31_SHIFT        (24)       /* Bits 24-31: Waveform control field 31 segment bits */
#define LCD_WF31TO28_WF31_MASK         (0xff << LCD_WF31TO28_WF31_SHIFT)
#  define LCD_WF31TO28_WF31_SEGMENT(n) ((1 << (n)) << LCD_WF31TO28_WF31_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF35TO32_WF32_SHIFT        (0)        /* Bits 0-7: Waveform control field 32 segment bits */
#define LCD_WF35TO32_WF32_MASK         (0xff << LCD_WF35TO32_WF32_SHIFT)
#  define LCD_WF35TO32_WF32_SEGMENT(n) ((1 << (n)) << LCD_WF35TO32_WF32_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF35TO32_WF33_SHIFT        (8)        /* Bits 8-15: Waveform control field 33 segment bits */
#define LCD_WF35TO32_WF33_MASK         (0xff << LCD_WF35TO32_WF33_SHIFT)
#  define LCD_WF35TO32_WF33_SEGMENT(n) ((1 << (n)) << LCD_WF35TO32_WF33_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF35TO32_WF34_SHIFT        (16)       /* Bits 16-23: Waveform control field 34 segment bits */
#define LCD_WF35TO32_WF34_MASK         (0xff << LCD_WF35TO32_WF34_SHIFT)
#  define LCD_WF35TO32_WF34_SEGMENT(n) ((1 << (n)) << LCD_WF35TO32_WF34_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF35TO32_WF35_SHIFT        (24)       /* Bits 24-31: Waveform control field 35 segment bits */
#define LCD_WF35TO32_WF35_MASK         (0xff << LCD_WF35TO32_WF35_SHIFT)
#  define LCD_WF35TO32_WF35_SEGMENT(n) ((1 << (n)) << LCD_WF35TO32_WF35_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF39TO36_WF36_SHIFT        (0)        /* Bits 0-7: Waveform control field 36 segment bits */
#define LCD_WF39TO36_WF36_MASK         (0xff << LCD_WF39TO36_WF36_SHIFT)
#  define LCD_WF39TO36_WF36_SEGMENT(n) ((1 << (n)) << LCD_WF39TO36_WF36_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF39TO36_WF37_SHIFT        (8)        /* Bits 8-15: Waveform control field 37 segment bits */
#define LCD_WF39TO36_WF37_MASK         (0xff << LCD_WF39TO36_WF37_SHIFT)
#  define LCD_WF39TO36_WF37_SEGMENT(n) ((1 << (n)) << LCD_WF39TO36_WF37_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF39TO36_WF38_SHIFT        (16)       /* Bits 16-23: Waveform control field 38 segment bits */
#define LCD_WF39TO36_WF38_MASK         (0xff << LCD_WF39TO36_WF38_SHIFT)
#  define LCD_WF39TO36_WF38_SEGMENT(n) ((1 << (n)) << LCD_WF39TO36_WF38_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF39TO36_WF39_SHIFT        (24)       /* Bits 24-31: Waveform control field 39 segment bits */
#define LCD_WF39TO36_WF39_MASK         (0xff << LCD_WF39TO36_WF39_SHIFT)
#  define LCD_WF39TO36_WF39_SEGMENT(n) ((1 << (n)) << LCD_WF39TO36_WF39_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF43TO40_WF40_SHIFT        (0)        /* Bits 0-7: Waveform control field 40 segment bits */
#define LCD_WF43TO40_WF40_MASK         (0xff << LCD_WF43TO40_WF40_SHIFT)
#  define LCD_WF43TO40_WF40_SEGMENT(n) ((1 << (n)) << LCD_WF43TO40_WF40_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF43TO40_WF41_SHIFT        (8)        /* Bits 8-15: Waveform control field 41 segment bits */
#define LCD_WF43TO40_WF41_MASK         (0xff << LCD_WF43TO40_WF41_SHIFT)
#  define LCD_WF43TO40_WF41_SEGMENT(n) ((1 << (n)) << LCD_WF43TO40_WF41_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF43TO40_WF42_SHIFT        (16)       /* Bits 16-23: Waveform control field 42 segment bits */
#define LCD_WF43TO40_WF42_MASK         (0xff << LCD_WF43TO40_WF42_SHIFT)
#  define LCD_WF43TO40_WF42_SEGMENT(n) ((1 << (n)) << LCD_WF43TO40_WF42_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF43TO40_WF43_SHIFT        (24)       /* Bits 24-31: Waveform control field 43 segment bits */
#define LCD_WF43TO40_WF43_MASK         (0xff << LCD_WF43TO40_WF43_SHIFT)
#  define LCD_WF43TO40_WF43_SEGMENT(n) ((1 << (n)) << LCD_WF43TO40_WF43_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF47TO44_WF44_SHIFT        (0)        /* Bits 0-7: Waveform control field 44 segment bits */
#define LCD_WF47TO44_WF44_MASK         (0xff << LCD_WF47TO44_WF44_SHIFT)
#  define LCD_WF47TO44_WF44_SEGMENT(n) ((1 << (n)) << LCD_WF47TO44_WF44_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF47TO44_WF45_SHIFT        (8)        /* Bits 8-15: Waveform control field 45 segment bits */
#define LCD_WF47TO44_WF45_MASK         (0xff << LCD_WF47TO44_WF45_SHIFT)
#  define LCD_WF47TO44_WF45_SEGMENT(n) ((1 << (n)) << LCD_WF47TO44_WF45_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF47TO44_WF46_SHIFT        (16)       /* Bits 16-23: Waveform control field 46 segment bits */
#define LCD_WF47TO44_WF46_MASK         (0xff << LCD_WF47TO44_WF46_SHIFT)
#  define LCD_WF47TO44_WF46_SEGMENT(n) ((1 << (n)) << LCD_WF47TO44_WF46_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF47TO44_WF47_SHIFT        (24)       /* Bits 24-31: Waveform control field 47 segment bits */
#define LCD_WF47TO44_WF47_MASK         (0xff << LCD_WF47TO44_WF47_SHIFT)
#  define LCD_WF47TO44_WF47_SEGMENT(n) ((1 << (n)) << LCD_WF47TO44_WF47_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF51TO48_WF48_SHIFT        (0)        /* Bits 0-7: Waveform control field 48 segment bits */
#define LCD_WF51TO48_WF48_MASK         (0xff << LCD_WF51TO48_WF48_SHIFT)
#  define LCD_WF51TO48_WF48_SEGMENT(n) ((1 << (n)) << LCD_WF51TO48_WF48_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF51TO48_WF49_SHIFT        (8)        /* Bits 8-15: Waveform control field 49 segment bits */
#define LCD_WF51TO48_WF49_MASK         (0xff << LCD_WF51TO48_WF49_SHIFT)
#  define LCD_WF51TO48_WF49_SEGMENT(n) ((1 << (n)) << LCD_WF51TO48_WF49_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF51TO48_WF50_SHIFT        (16)       /* Bits 16-23: Waveform control field 50 segment bits */
#define LCD_WF51TO48_WF50_MASK         (0xff << LCD_WF51TO48_WF50_SHIFT)
#  define LCD_WF51TO48_WF50_SEGMENT(n) ((1 << (n)) << LCD_WF51TO48_WF50_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF51TO48_WF51_SHIFT        (24)       /* Bits 24-31: Waveform control field 51 segment bits */
#define LCD_WF51TO48_WF51_MASK         (0xff << LCD_WF51TO48_WF51_SHIFT)
#  define LCD_WF51TO48_WF51_SEGMENT(n) ((1 << (n)) << LCD_WF51TO48_WF51_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF55TO52_WF52_SHIFT        (0)        /* Bits 0-7: Waveform control field 52 segment bits */
#define LCD_WF55TO52_WF52_MASK         (0xff << LCD_WF55TO52_WF52_SHIFT)
#  define LCD_WF55TO52_WF52_SEGMENT(n) ((1 << (n)) << LCD_WF55TO52_WF52_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF55TO52_WF53_SHIFT        (8)        /* Bits 8-15: Waveform control field 53 segment bits */
#define LCD_WF55TO52_WF53_MASK         (0xff << LCD_WF55TO52_WF53_SHIFT)
#  define LCD_WF55TO52_WF53_SEGMENT(n) ((1 << (n)) << LCD_WF55TO52_WF53_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF55TO52_WF54_SHIFT        (16)       /* Bits 16-23: Waveform control field 54 segment bits */
#define LCD_WF55TO52_WF54_MASK         (0xff << LCD_WF55TO52_WF54_SHIFT)
#  define LCD_WF55TO52_WF54_SEGMENT(n) ((1 << (n)) << LCD_WF55TO52_WF54_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF55TO52_WF55_SHIFT        (24)       /* Bits 24-31: Waveform control field 55 segment bits */
#define LCD_WF55TO52_WF55_MASK         (0xff << LCD_WF55TO52_WF55_SHIFT)
#  define LCD_WF55TO52_WF55_SEGMENT(n) ((1 << (n)) << LCD_WF55TO52_WF55_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF59TO56_WF56_SHIFT        (0)        /* Bits 0-7: Waveform control field 56 segment bits */
#define LCD_WF59TO56_WF56_MASK         (0xff << LCD_WF59TO56_WF56_SHIFT)
#  define LCD_WF59TO56_WF56_SEGMENT(n) ((1 << (n)) << LCD_WF59TO56_WF56_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF59TO56_WF57_SHIFT        (8)        /* Bits 8-15: Waveform control field 57 segment bits */
#define LCD_WF59TO56_WF57_MASK         (0xff << LCD_WF59TO56_WF57_SHIFT)
#  define LCD_WF59TO56_WF57_SEGMENT(n) ((1 << (n)) << LCD_WF59TO56_WF57_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF59TO56_WF58_SHIFT        (16)       /* Bits 16-23: Waveform control field 58 segment bits */
#define LCD_WF59TO56_WF58_MASK         (0xff << LCD_WF59TO56_WF58_SHIFT)
#  define LCD_WF59TO56_WF58_SEGMENT(n) ((1 << (n)) << LCD_WF59TO56_WF58_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF59TO56_WF59_SHIFT        (24)       /* Bits 24-31: Waveform control field 59 segment bits */
#define LCD_WF59TO56_WF59_MASK         (0xff << LCD_WF59TO56_WF59_SHIFT)
#  define LCD_WF59TO56_WF59_SEGMENT(n) ((1 << (n)) << LCD_WF59TO56_WF59_SHIFT) /* Segment n, n=0..7 */

#define LCD_WF63TO60_WF60_SHIFT        (0)        /* Bits 0-7: Waveform control field 60 segment bits */
#define LCD_WF63TO60_WF60_MASK         (0xff << LCD_WF63TO60_WF60_SHIFT)
#  define LCD_WF63TO60_WF60_SEGMENT(n) ((1 << (n)) << LCD_WF63TO60_WF60_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF63TO60_WF61_SHIFT        (8)        /* Bits 8-15: Waveform control field 61 segment bits */
#define LCD_WF63TO60_WF61_MASK         (0xff << LCD_WF63TO60_WF61_SHIFT)
#  define LCD_WF63TO60_WF61_SEGMENT(n) ((1 << (n)) << LCD_WF63TO60_WF61_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF63TO60_WF62_SHIFT        (16)       /* Bits 16-23: Waveform control field 62 segment bits */
#define LCD_WF63TO60_WF62_MASK         (0xff << LCD_WF63TO60_WF62_SHIFT)
#  define LCD_WF63TO60_WF62_SEGMENT(n) ((1 << (n)) << LCD_WF63TO60_WF62_SHIFT) /* Segment n, n=0..7 */
#define LCD_WF63TO60_WF63_SHIFT        (24)       /* Bits 24-31: Waveform control field 63 segment bits */
#define LCD_WF63TO60_WF63_MASK         (0xff << LCD_WF63TO60_WF63_SHIFT)
#  define LCD_WF63TO60_WF63_SEGMENT(n) ((1 << (n)) << LCD_WF63TO60_WF63_SHIFT) /* Segment n, n=0..7 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_SLCD_H */
