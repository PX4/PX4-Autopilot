/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx-devcfg.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_DEVCFG_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_DEVCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mx-memorymap.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Register Offsets *********************************************************/

#define PIC32MX_DEVCFG3_OFFSET      0x0000 /* Device configuration word 3 */
#define PIC32MX_DEVCFG2_OFFSET      0x0004 /* Device configuration word 2 */
#define PIC32MX_DEVCFG1_OFFSET      0x0008 /* Device configuration word 1 */
#define PIC32MX_DEVCFG0_OFFSET      0x000c /* Device configuration word 0 */

/* Register Addresses *******************************************************/

#define PIC32MX_DEVCFG3             (PIC32MX_DEVCFG_K1BASE+PIC32MX_DEVCFG3_OFFSET)
#define PIC32MX_DEVCFG2             (PIC32MX_DEVCFG_K1BASE+PIC32MX_DEVCFG2_OFFSET)
#define PIC32MX_DEVCFG1             (PIC32MX_DEVCFG_K1BASE+PIC32MX_DEVCFG1_OFFSET)
#define PIC32MX_DEVCFG0             (PIC32MX_DEVCFG_K1BASE+PIC32MX_DEVCFG0_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Device configuration word 3 */

#define DEVCFG3_USERID_SHIFT        (0)        /* Bits 0-15: User-defined, readable via ICSP™ and JTAG */
#define DEVCFG3_USERID_MASK         (0xffff << DEVCFG3_USERID_SHIFT)

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)

#  define DEVCFG3_PMDL1WAY          (1 << 28)  /* Bit 28: Peripheral Module Disable Configuration */
#  define DEVCFG3_IOL1WAY           (1 << 29)  /* Bit 29: Peripheral Pin Select Configuration */
#  define DEVCFG3_FUSBIDIO          (1 << 30)  /* Bit 30: USB USBID selection */
#  define DEVCFG3_FVBUSIO           (1 << 31)  /* Bit 31: USB VBUSON selection */
#  define DEVCFG3_UNUSED            0x0fff0000 /* Bits 16-28 */

#elif defined(CHIP_PIC32MX3) || defined(CHIP_PIC32MX4)

#  define DEVCFG3_UNUSED            0xffff0000 /* Bits 16-31 */

#elif defined(CHIP_PIC32MX5) || defined(CHIP_PIC32MX6) || defined(CHIP_PIC32MX7)

#  define DEVCFG3_FSRSSEL_SHIFT     (16)       /* Bits 16-18: SRS select */
#  define DEVCFG3_FSRSSEL_MASK      (7 << DEVCFG3_FSRSSEL_SHIFT)
#  define DEVCFG3_FMIIEN            (1 << 24)  /* Bit 24: Ethernet MII enable */
#  define DEVCFG3_FETHIO            (1 << 25)  /* Bit 25: Ethernet I/O pin selection */
#  define DEVCFG3_FCANIO            (1 << 26)  /* Bit 26: CAN I/O pin selection */
#  define DEVCFG3_FSCM1IO           (1 << 29)  /* Bit 29: SCM1 pin C selection */
#  define DEVCFG3_FUSBIDIO          (1 << 30)  /* Bit 30: USB USBID selection */
#  define DEVCFG3_FVBUSIO           (1 << 31)  /* Bit 31: USB VBUSON selection */
#  define DEVCFG3_UNUSED            0x18f80000 /* Bits 19-23, 27-28 */

#endif

/* Device configuration word 2 */

#define DEVCFG2_FPLLIDIV_SHIFT      (0)       /* Bits 0-2: PLL input divider value */
#define DEVCFG2_FPLLIDIV_MASK       (7 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV1     (0 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV2     (1 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV3     (2 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV4     (3 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV5     (4 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV6     (5 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV10    (6 << DEVCFG2_FPLLIDIV_SHIFT)
#  define DEVCFG2_FPLLIDIV_DIV12    (7 << DEVCFG2_FPLLIDIV_SHIFT)
#define DEVCFG2_FPLLMULT_SHIFT      (4)       /* Bits 4-6: Initial PLL multiplier value */
#define DEVCFG2_FPLLMULT_MASK       (7 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL15    (0 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL16    (1 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL17    (2 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL18    (3 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL19    (4 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL20    (5 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL21    (6 << DEVCFG2_FPLLMULT_SHIFT)
#  define DEVCFG2_FPLLMULT_MUL24    (7 << DEVCFG2_FPLLMULT_SHIFT)
#define DEVCFG2_FUPLLIDIV_SHIFT     (8)       /* Bits 8-10: PLL input divider */
#define DEVCFG2_FUPLLIDIV_MASK      (7 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV1    (0 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV2    (1 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV3    (2 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV4    (3 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV5    (4 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV6    (5 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV10   (6 << DEVCFG2_FUPLLIDIV_SHIFT)
#  define DEVCFG2_FUPLLIDIV_DIV12   (7 << DEVCFG2_FUPLLIDIV_SHIFT)
#define DEVCFG2_FUPLLEN             (1 << 15) /* Bit 15: USB PLL enable */
#define DEVCFG2_FPLLODIV_SHIFT      (16)      /* Bits 16-18: Default postscaler for PLL bits */
#define DEVCFG2_FPLLODIV_MASK       (7 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV1     (0 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV2     (1 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV4     (2 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV8     (3 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV16    (4 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV32    (5 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV64    (6 << DEVCFG2_FPLLODIV_SHIFT)
#  define DEVCFG2_FPLLODIV_DIV256   (7 << DEVCFG2_FPLLODIV_SHIFT)
#define DEVCFG2_UNUSED              0xfff87888 /* Bits 3, 7, 11-14, 19-31 */

/* Device configuration word 1 */

#define DEVCFG1_FNOSC_SHIFT         (0)       /* Bits 0-2: Oscillator selection */
#define DEVCFG1_FNOSC_MASK          (7 << DEVCFG1_FNOSC_SHIFT)
#  define DEVCFG1_FNOSC_FRC         (0 << DEVCFG1_FNOSC_SHIFT) /* FRC oscillator */
#  define DEVCFG1_FNOSC_FRCPLL      (1 << DEVCFG1_FNOSC_SHIFT) /* FRC w/PLL module */
#  define DEVCFG1_FNOSC_POSC        (2 << DEVCFG1_FNOSC_SHIFT) /* Primary oscillator */
#  define DEVCFG1_FNOSC_POSCPLL     (3 << DEVCFG1_FNOSC_SHIFT) /* Primary oscillator w/PLL */
#  define DEVCFG1_FNOSC_SOSC        (4 << DEVCFG1_FNOSC_SHIFT) /* Secondary oscillator */
#  define DEVCFG1_FNOSC_LPRC        (5 << DEVCFG1_FNOSC_SHIFT) /* Low power RC oscillator */
#  define DEVCFG1_FNOSC_FRCDIV      (7 << DEVCFG1_FNOSC_SHIFT) /* FRC oscillator with FRCDIV */
#define DEVCFG1_FSOSCEN             (1 << 5)  /* Bit 5: Secondary oscillator (sosc) enable bit */
#define DEVCFG1_IESO                (1 << 7)  /* Bit 7: Internal external switch over */
#define DEVCFG1_POSCMOD_SHIFT       (8)       /* Bits 8-9: Primary oscillator (posc) configuration */
#define DEVCFG1_POSCMOD_MASK        (3 << DEVCFG1_POSCMOD_SHIFT)
#  define DEVCFG1_POSCMOD_EC        (0 << DEVCFG1_POSCMOD_SHIFT) /* EC mode */
#  define DEVCFG1_POSCMOD_XT        (1 << DEVCFG1_POSCMOD_SHIFT) /* XT mode */
#  define DEVCFG1_POSCMOD_HS        (2 << DEVCFG1_POSCMOD_SHIFT) /* HS mode */
#  define DEVCFG1_POSCMOD_DIS       (3 << DEVCFG1_POSCMOD_SHIFT) /* Primary Oscillator disabled */
#define DEVCFG1_OSCIOFNC            (1 << 10) /* Bit 10: CLKO (clock-out) enable configuration */
#define DEVCFG1_FPBDIV_SHIFT        (12)      /* Bits 12-13: Peripheral bus clock divisor default value */
#define DEVCFG1_FPBDIV_MASK         (3 << DEVCFG1_FPBDIV_SHIFT)
#  define DEVCFG1_FPBDIV_DIV1       (0 << DEVCFG1_FPBDIV_SHIFT) /* PBCLK is SYSCLK/1 */
#  define DEVCFG1_FPBDIV_DIV2       (1 << DEVCFG1_FPBDIV_SHIFT) /* PBCLK is SYSCLK/2 */
#  define DEVCFG1_FPBDIV_DIV4       (2 << DEVCFG1_FPBDIV_SHIFT) /* PBCLK is SYSCLK/4 */
#  define DEVCFG1_FPBDIV_DIV8       (3 << DEVCFG1_FPBDIV_SHIFT) /* PBCLK is SYSCLK /8 */
#define DEVCFG1_FCKSM_SHIFT         (14)      /* Bits 14-15: Clock switching and monitor selection configuration */
#define DEVCFG1_FCKSM_MASK          (3 << DEVCFG1_FCKSM_SHIFT)
#  define DEVCFG1_FCKSM_BOTH        (0 << DEVCFG1_FCKSM_SHIFT) /* Clock switching and FSCM are enabled */
#  define DEVCFG1_FCKSM_CSONLY      (1 << DEVCFG1_FCKSM_SHIFT) /* Clock switching is enabled, FSCM is disabled */
#  define DEVCFG1_FCKSM_NONE        (3 << DEVCFG1_FCKSM_SHIFT) /* Clock switching and FSCM are disabled */
#define DEVCFG1_WDTPS_SHIFT         (16)      /* Bits 16-20: WDT postscaler select */
#define DEVCFG1_WDTPS_MASK          (31 << DEVCFG1_WDTPS_SHIFT)
#  define DEVCFG1_WDTPS_1           (0 << DEVCFG1_WDTPS_SHIFT)  /* 1:1 */
#  define DEVCFG1_WDTPS_2           (1 << DEVCFG1_WDTPS_SHIFT)  /* 1:2 */
#  define DEVCFG1_WDTPS_4           (2 << DEVCFG1_WDTPS_SHIFT)  /* 1:4 */
#  define DEVCFG1_WDTPS_8           (3 << DEVCFG1_WDTPS_SHIFT)  /* 1:8 */
#  define DEVCFG1_WDTPS_16          (4 << DEVCFG1_WDTPS_SHIFT)  /* 1:16 */
#  define DEVCFG1_WDTPS_32          (5 << DEVCFG1_WDTPS_SHIFT)  /* 1:32 */
#  define DEVCFG1_WDTPS_64          (6 << DEVCFG1_WDTPS_SHIFT)  /* 1:64 */
#  define DEVCFG1_WDTPS_128         (7 << DEVCFG1_WDTPS_SHIFT)  /* 1:128 */
#  define DEVCFG1_WDTPS_256         (8 << DEVCFG1_WDTPS_SHIFT)  /* 1:256 */
#  define DEVCFG1_WDTPS_512         (9 << DEVCFG1_WDTPS_SHIFT)  /* 1:512 */
#  define DEVCFG1_WDTPS_1024        (10 << DEVCFG1_WDTPS_SHIFT) /* 1:1024 */
#  define DEVCFG1_WDTPS_2048        (11 << DEVCFG1_WDTPS_SHIFT) /* 1:2048 */
#  define DEVCFG1_WDTPS_4096        (12 << DEVCFG1_WDTPS_SHIFT) /* 1:4096 */
#  define DEVCFG1_WDTPS_8192        (13 << DEVCFG1_WDTPS_SHIFT) /* 1:8192 */
#  define DEVCFG1_WDTPS_16384       (14 << DEVCFG1_WDTPS_SHIFT) /* 1:16384 */
#  define DEVCFG1_WDTPS_32768       (15 << DEVCFG1_WDTPS_SHIFT) /* 1:32768 */
#  define DEVCFG1_WDTPS_65536       (16 << DEVCFG1_WDTPS_SHIFT) /* 1:65536 */
#  define DEVCFG1_WDTPS_131072      (17 << DEVCFG1_WDTPS_SHIFT) /* 1:131072 */
#  define DEVCFG1_WDTPS_262144      (18 << DEVCFG1_WDTPS_SHIFT) /* 1:262144 */
#  define DEVCFG1_WDTPS_524288      (19 << DEVCFG1_WDTPS_SHIFT) /* 1:524288 */
#  define DEVCFG1_WDTPS_1048576     (20 << DEVCFG1_WDTPS_SHIFT) /* 1:1048576 */
#define DEVCFG1_FWDTEN              (1 << 23) /* Bit 23: WDT enable */

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define DEVCFG1_WINDIS            (1 << 22) /* Bit 22: Windowed watchdog timer enable */
#  define DEVCFG1_FWDTWINSZ_SHIFT   (24)      /* Bits 24-25: Watchdog Timer Window Size bits */
#  define DEVCFG1_FWDTWINSZ_MASK    (3 << DEVCFG1_FWDTWINSZ_SHIFT)
#    define DEVCFG1_FWDTWINSZ_25    (0 << DEVCFG1_FWDTWINSZ_SHIFT) /* 25% */
#    define DEVCFG1_FWDTWINSZ_37p5  (1 << DEVCFG1_FWDTWINSZ_SHIFT) /* 37.5% */
#    define DEVCFG1_FWDTWINSZ_50    (2 << DEVCFG1_FWDTWINSZ_SHIFT) /* 50% */
#    define DEVCFG1_FWDTWINSZ_75    (3 << DEVCFG1_FWDTWINSZ_SHIFT) /* 75% */
#  define DEVCFG1_UNUSED            0xfc200858 /* Bits 3-4, 6, 11, 21, 26-31 */
#else
#  define DEVCFG1_UNUSED            0xff600858 /* Bits 3-4, 6, 11, 21-22, 24-31 */
#endif

/* Device configuration word 0 */

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define PWP_CODE(a)               REVISIT (((~((a) >> 12)) - 1) & 0x3ff)

#  define DEVCFG0_DEBUG_SHIFT       (0)       /* Bits 0-1: Background debugger enable */
#  define DEVCFG0_DEBUG_MASK        (3 << DEVCFG0_DEBUG_SHIFT)
#    define DEVCFG0_DEBUG_ENABLED   (1 << DEVCFG0_DEBUG_SHIFT)
#    define DEVCFG0_DEBUG_DISABLED  (3 << DEVCFG0_DEBUG_SHIFT)
#  define DEVCFG0_JTAGEN            (1 << 2)  /* Bit 2: JTAG enable */
#  define DEVCFG0_ICESEL_SHIFT      (3)       /* Bits 3-4: Background debugger enable */
#  define DEVCFG0_ICESEL_MASK       (3 << DEVCFG0_ICESEL_SHIFT)
#    define DEVCFG0_ICESEL_CHAN4    (0 << DEVCFG0_ICESEL_SHIFT) /* PGEC4/PGED4 pair is used */
#    define DEVCFG0_ICESEL_CHAN3    (1 << DEVCFG0_ICESEL_SHIFT) /* PGEC3/PGED3 pair is used */
#    define DEVCFG0_ICESEL_CHAN2    (2 << DEVCFG0_ICESEL_SHIFT) /* PGEC2/PGED2 pair is used */
#    define DEVCFG0_ICESEL_CHAN1    (3 << DEVCFG0_ICESEL_SHIFT) /* PGEC1/PGED1 pair is used */
#  define DEVCFG0_PWP_SHIFT         (10)      /* Bits 10-15: Program flash write-protect */
#  define DEVCFG0_PWP_MASK          (0x3ff << DEVCFG0_PWP_SHIFT)
#    define DEVCFG0_PWP_DISABLE     (0x3ff << DEVCFG0_PWP_SHIFT)
#    define DEVCFG0_PWP(code)       ((code) << DEVCFG0_PWP_SHIFT) /* See PWP_CODE above */
#  define DEVCFG0_BWP               (1 << 24) /* Bit 24: Boot flash write-protect */
#  define DEVCFG0_CP                (1 << 28) /* Bit 28: Code-protect */
#  define DEVCFG0_SIGN              (1 << 31) /* Bit 31: Signature */

#  define DEVCFG0_UNUSED            0x6eff03e0 /* Bits 5-9, 16-23, 25-27, 29-30 */
#else
#  define PWP_CODE(a)                 (((~((a) >> 12)) - 1) & 0xff)

#  define DEVCFG0_DEBUG_SHIFT       (0)       /* Bits 0-1: Background debugger enable */
#  define DEVCFG0_DEBUG_MASK        (3 << DEVCFG0_DEBUG_SHIFT)
#    define DEVCFG0_DEBUG_ENABLED   (2 << DEVCFG0_DEBUG_SHIFT)
#    define DEVCFG0_DEBUG_DISABLED  (3 << DEVCFG0_DEBUG_SHIFT)
#  define DEVCFG0_ICESEL            (1 << 3)  /* Bit 3: ICE/debugger channel select */
#  define DEVCFG0_PWP_SHIFT         (12)      /* Bits 12-19: Program flash write-protect */
#  define DEVCFG0_PWP_MASK          (0xff << DEVCFG0_PWP_SHIFT)
#    define DEVCFG0_PWP_DISABLE     (0xff << DEVCFG0_PWP_SHIFT)
#    define DEVCFG0_PWP(code)       ((code) << DEVCFG0_PWP_SHIFT) /* See PWP_CODE above */
#  define DEVCFG0_BWP               (1 << 24) /* Bit 24: Boot flash write-protect */
#  define DEVCFG0_CP                (1 << 28) /* Bit 28: Code-protect */
#  define DEVCFG0_SIGN              (1 << 31) /* Bit 31: Signature */

#  define DEVCFG0_UNUSED            0x6ef00ff0 /* Bits 4-11, 20-23, 25-27, 29-30 */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_DEVCFG_H */
