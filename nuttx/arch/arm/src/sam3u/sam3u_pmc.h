/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_pmc.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_PMC_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_PMC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* PMC register offsets *****************************************************************/

#define SAM3U_PMC_SCER_OFFSET         0x0000 /* System Clock Enable Register */
#define SAM3U_PMC_SCDR_OFFSET         0x0004 /* System Clock Disable Register */
#define SAM3U_PMC_SCSR_OFFSET         0x0008 /* System Clock Status Register */
                                             /* 0x000c: Reserved */
#define SAM3U_PMC_PCER_OFFSET         0x0010 /* Peripheral Clock Enable Register */
#define SAM3U_PMC_PCDR_OFFSET         0x0014 /* Peripheral Clock Disable Register */
#define SAM3U_PMC_PCSR_OFFSET         0x0018 /* Peripheral Clock Status Register */
#define SAM3U_CKGR_UCKR_OFFSET        0x001c /* UTMI Clock Register */
#define SAM3U_CKGR_MOR_OFFSET         0x0020 /* Main Oscillator Register */
#define SAM3U_CKGR_MCFR_OFFSET        0x0024 /* Main Clock Frequency Register */
#define SAM3U_CKGR_PLLAR_OFFSET       0x0028 /* PLLA Register */
                                             /* 0x002c: Reserved */
#define SAM3U_PMC_MCKR_OFFSET         0x0030 /* Master Clock Register */
                                             /* 0x0034-0x003C Reserved */
#define SAM3U_PMC_PCK_OFFSET(n)       (0x0040+((n)<<2))
#define SAM3U_PMC_PCK0_OFFSET         0x0040 /* Programmable Clock 0 Register */
#define SAM3U_PMC_PCK1_OFFSET         0x0044 /* Programmable Clock 1 Register */
#define SAM3U_PMC_PCK2_OFFSET         0x0048 /* Programmable Clock 2 Register */
                                             /* 0x004c-0x005c: Reserved */
#define SAM3U_PMC_IER_OFFSET          0x0060 /* Interrupt Enable Register */
#define SAM3U_PMC_IDR_OFFSET          0x0064 /* Interrupt Disable Register */
#define SAM3U_PMC_SR_OFFSET           0x0068 /* Status Register */
#define SAM3U_PMC_IMR_OFFSET          0x006c /* Interrupt Mask Register */
#define SAM3U_PMC_FSMR_OFFSET         0x0070 /* Fast Startup Mode Register */
#define SAM3U_PMC_FSPR_OFFSET         0x0074 /* Fast Startup Polarity Register */
#define SAM3U_PMC_FOCR_OFFSET         0x0078 /* Fault Output Clear Register */
                                             /* 0x007c-0x00fc: Reserved */
#define SAM3U_PMC_WPMR_OFFSET         0x00e4 /* Write Protect Mode Register */
#define SAM3U_PMC_WPSR_OFFSET         0x00e8 /* Write Protect Status Register */

/* PMC register adresses ****************************************************************/

#define SAM3U_PMC_SCER                (SAM3U_PMC_BASE+SAM3U_PMC_SCER_OFFSET)
#define SAM3U_PMC_SCDR                (SAM3U_PMC_BASE+SAM3U_PMC_SCDR_OFFSET)
#define SAM3U_PMC_SCSR                (SAM3U_PMC_BASE+SAM3U_PMC_SCSR_OFFSET)
#define SAM3U_PMC_PCER                (SAM3U_PMC_BASE+SAM3U_PMC_PCER_OFFSET)
#define SAM3U_PMC_PCDR                (SAM3U_PMC_BASE+SAM3U_PMC_PCDR_OFFSET)
#define SAM3U_PMC_PCSR                (SAM3U_PMC_BASE+SAM3U_PMC_PCSR_OFFSET)
#define SAM3U_CKGR_UCKR               (SAM3U_PMC_BASE+SAM3U_CKGR_UCKR_OFFSET)
#define SAM3U_CKGR_MOR                (SAM3U_PMC_BASE+SAM3U_CKGR_MOR_OFFSET)
#define SAM3U_CKGR_MCFR               (SAM3U_PMC_BASE+SAM3U_CKGR_MCFR_OFFSET)
#define SAM3U_CKGR_PLLAR              (SAM3U_PMC_BASE+SAM3U_CKGR_PLLAR_OFFSET)
#define SAM3U_PMC_MCKR                (SAM3U_PMC_BASE+SAM3U_PMC_MCKR_OFFSET)
#define SAM3U_PMC_PCK(n)              (SAM3U_PMC_BASE+SAM3U_PMC_PCK_OFFSET(n))
#define SAM3U_PMC_PCK0                (SAM3U_PMC_BASE+SAM3U_PMC_PCK0_OFFSET)
#define SAM3U_PMC_PCK1                (SAM3U_PMC_BASE+SAM3U_PMC_PCK1_OFFSET)
#define SAM3U_PMC_PCK2                (SAM3U_PMC_BASE+SAM3U_PMC_PCK2_OFFSET)
#define SAM3U_PMC_IER                 (SAM3U_PMC_BASE+SAM3U_PMC_IER_OFFSET)
#define SAM3U_PMC_IDR                 (SAM3U_PMC_BASE+SAM3U_PMC_IDR_OFFSET)
#define SAM3U_PMC_SR                  (SAM3U_PMC_BASE+SAM3U_PMC_SR_OFFSET)
#define SAM3U_PMC_IMR                 (SAM3U_PMC_BASE+SAM3U_PMC_IMR_OFFSET)
#define SAM3U_PMC_FSMR                (SAM3U_PMC_BASE+SAM3U_PMC_FSMR_OFFSET)
#define SAM3U_PMC_FSPR                (SAM3U_PMC_BASE+SAM3U_PMC_FSPR_OFFSET)
#define SAM3U_PMC_FOCR                (SAM3U_PMC_BASE+SAM3U_PMC_FOCR_OFFSET)
#define SAM3U_PMC_WPMR                (SAM3U_PMC_BASE+SAM3U_PMC_WPMR_OFFSET)
#define SAM3U_PMC_WPSR                (SAM3U_PMC_BASE+SAM3U_PMC_WPSR_OFFSET)

/* PMC register bit definitions *********************************************************/

/* PMC System Clock Enable Register, PMC System Clock Disable Register, and PMC System
 * Clock Status Register common bit-field definitions
 */

#define PMC_PCK(n)                    (1 <<((n)+8)
#define PMC_PCK0                      (1 << 8)  /* Bit 8:  Programmable Clock 0 Output Enable */
#define PMC_PCK1                      (1 << 9)  /* Bit 9:  Programmable Clock 1 Output Enable */
#define PMC_PCK2                      (1 << 10) /* Bit 10: Programmable Clock 2 Output Enable */

/* PMC Peripheral Clock Enable Register, PMC Peripheral Clock Disable Register, and PMC
 * Peripheral Clock Status Register common bit-field definitions.
 */

#define PMC_PID(n)                    (1<<(n))
#define PMC_PID2                      (1 << 2)  /* Bit 2:  Peripheral Clock 2  Enable */
#define PMC_PID3                      (1 << 3)  /* Bit 3:  Peripheral Clock 3  Enable */
#define PMC_PID4                      (1 << 4)  /* Bit 4:  Peripheral Clock 4  Enable */
#define PMC_PID5                      (1 << 5)  /* Bit 5:  Peripheral Clock 5  Enable */
#define PMC_PID6                      (1 << 6)  /* Bit 6:  Peripheral Clock 6  Enable */
#define PMC_PID7                      (1 << 7)  /* Bit 7:  Peripheral Clock 7  Enable */
#define PMC_PID8                      (1 << 8)  /* Bit 8:  Peripheral Clock 8  Enable */
#define PMC_PID9                      (1 << 9)  /* Bit 9:  Peripheral Clock 9  Enable */
#define PMC_PID10                     (1 << 10) /* Bit 10: Peripheral Clock 10 Enable */
#define PMC_PID11                     (1 << 11) /* Bit 11: Peripheral Clock 11 Enable */
#define PMC_PID12                     (1 << 12) /* Bit 12: Peripheral Clock 12 Enable */
#define PMC_PID13                     (1 << 13) /* Bit 13: Peripheral Clock 13 Enable */
#define PMC_PID14                     (1 << 14) /* Bit 14: Peripheral Clock 14 Enable */
#define PMC_PID15                     (1 << 15) /* Bit 15: Peripheral Clock 15 Enable */
#define PMC_PID16                     (1 << 16) /* Bit 16: Peripheral Clock 16 Enable */
#define PMC_PID17                     (1 << 17) /* Bit 17: Peripheral Clock 17 Enable */
#define PMC_PID18                     (1 << 18) /* Bit 18: Peripheral Clock 18 Enable */
#define PMC_PID19                     (1 << 19) /* Bit 19: Peripheral Clock 19 Enable */
#define PMC_PID20                     (1 << 20) /* Bit 20: Peripheral Clock 20 Enable */
#define PMC_PID21                     (1 << 21) /* Bit 21: Peripheral Clock 21 Enable */
#define PMC_PID22                     (1 << 22) /* Bit 22: Peripheral Clock 22 Enable */
#define PMC_PID23                     (1 << 23) /* Bit 23: Peripheral Clock 23 Enable */
#define PMC_PID24                     (1 << 24) /* Bit 24: Peripheral Clock 24 Enable */
#define PMC_PID25                     (1 << 25) /* Bit 25: Peripheral Clock 25 Enable */
#define PMC_PID26                     (1 << 26) /* Bit 26: Peripheral Clock 26 Enable */
#define PMC_PID27                     (1 << 27) /* Bit 27: Peripheral Clock 27 Enable */
#define PMC_PID28                     (1 << 28) /* Bit 28: Peripheral Clock 28 Enable */
#define PMC_PID29                     (1 << 29) /* Bit 29: Peripheral Clock 29 Enable */
#define PMC_PID30                     (1 << 30) /* Bit 30: Peripheral Clock 30 Enable */
#define PMC_PID31                     (1 << 31) /* Bit 31: Peripheral Clock 31 Enable */

/* PMC UTMI Clock Configuration Register */

#define CKGR_UCKR_UPLLEN              (1 << 16) /* Bit 16: UTMI PLL Enable */
#define CKGR_UCKR_UPLLCOUNT_SHIFT     (20)      /* Bits 20-23: UTMI PLL Start-up Time */
#define CKGR_UCKR_UPLLCOUNT_MASK      (15 << CKGR_UCKR_UPLLCOUNT_SHIFT)

/* PMC Clock Generator Main Oscillator Register */

#define CKGR_MOR_MOSCXTEN             (1 << 0)  /* Bit 0:  Main Crystal Oscillator Enable */
#define CKGR_MOR_MOSCXTBY             (1 << 1)  /* Bit 1:  Main Crystal Oscillator Bypass */
#define CKGR_MOR_WAITMODE             (1 << 2)  /* Bit 2:  Wait Mode Command */
#define CKGR_MOR_MOSCRCEN             (1 << 3)  /* Bit 3:  Main On-Chip RC Oscillator Enable */
#define CKGR_MOR_MOSCRCF_SHIFT        (4)       /* Bits 4-6: Main On-Chip RC Oscillator Frequency Selection */
#define CKGR_MOR_MOSCRCF_MASK         (7 << CKGR_MOR_MOSCRCF_SHIFT)
#define CKGR_MOR_MOSCXTST_SHIFT       (8)       /* Bits 8-16: Main Crystal Oscillator Start-up Time */
#define CKGR_MOR_MOSCXTST_MASK        (0x1ff << CKGR_MOR_MOSCXTST_SHIFT)
#define CKGR_MOR_KEY_SHIFT            (16)      /* Bits 16-23: Password */
#define CKGR_MOR_KEY_MASK             (0xff << CKGR_MOR_KEY_SHIFT)
#define CKGR_MOR_MOSCSEL              (1 << 24) /* Bit 24: Main Oscillator Selection */
#define CKGR_MOR_CFDEN                (1 << 25) /* Bit 25: Clock Failure Detector Enable */


/* PMC Clock Generator Main Clock Frequency Register */

#define CKGR_MCFR_MAINF_SHIFT         (0)       /* Bits 0-15: Main Clock Frequency */
#define CKGR_MCFR_MAINF_MASK          (0xffff << CKGR_MCFR_MAINF_SHIFT)
#define CKGR_MCFR_MAINFRDY            (1 << 16) /* Bit 16: Main Clock Ready */

/* PMC Clock Generator PLLA Register */

#define CKGR_PLLAR_DIVA_SHIFT         (0)       /* Bits 0-7: Divider */
#define CKGR_PLLAR_DIVA_MASK          (0xff << CKGR_PLLAR_DIVA_SHIFT)
#  define CKGR_PLLAR_DIVA_ZERO        (0 << CKGR_PLLAR_DIVA_SHIFT)   /* Divider output is 0 */
#  define CKGR_PLLAR_DIVA_BYPASS      (1 << CKGR_PLLAR_DIVA_SHIFT)   /* Divider is bypassed (DIVA=1) */
#  define CKGR_PLLAR_DIVA(n)          ((n) << CKGR_PLLAR_DIVA_SHIFT) /* Divider output is DIVA=n, n=2..255 */
#define CKGR_PLLAR_PLLACOUNT_SHIFT    (8)       /* Bits 8-13: PLLA Counter */
#define CKGR_PLLAR_PLLACOUNT_MASK     (63 << CKGR_PLLAR_PLLACOUNT_SHIFT)
#define CKGR_PLLAR_STMODE_SHIFT       (14)      /* Bits 14-15: Start Mode */
#define CKGR_PLLAR_STMODE_MASK        (3 << CKGR_PLLAR_STMODE_SHIFT)
#  define CKGR_PLLAR_STMODE_FAST      (0 << CKGR_PLLAR_STMODE_SHIFT) /* Fast Startup */
#  define CKGR_PLLAR_STMODE_NORMAL    (2 << CKGR_PLLAR_STMODE_SHIFT) /* Normal Startup */
#define CKGR_PLLAR_MULA_SHIFT         (16)      /* Bits 16-26: PLLA Multiplier */
#define CKGR_PLLAR_MULA_MASK          (0x7ff << CKGR_PLLAR_MULA_SHIFT)
#define CKGR_PLLAR_ONE                (1 << 29) /* Bit 29: Always one */

/* PMC Master Clock Register */

#define PMC_MCKR_CSS_SHIFT            (0)       /* Bits 0-1: Master Clock Source Selection */
#define PMC_MCKR_CSS_MASK             (3 << PMC_MCKR_CSS_SHIFT)
#  define PMC_MCKR_CSS_SLOW           (0 << PMC_MCKR_CSS_SHIFT) /* Slow Clock */
#  define PMC_MCKR_CSS_MAIN           (1 << PMC_MCKR_CSS_SHIFT) /* Main Clock */
#  define PMC_MCKR_CSS_PLLA           (2 << PMC_MCKR_CSS_SHIFT) /* PLLA Clock */
#  define PMC_MCKR_CSS_UPLL           (3 << PMC_MCKR_CSS_SHIFT) /* UPLL Clock */
#define PMC_MCKR_PRES_SHIFT           (4)       /* Bits 4-6: Processor Clock Prescaler */
#define PMC_MCKR_PRES_MASK            (7 << PMC_MCKR_PRES_SHIFT)
#  define PMC_MCKR_PRES_DIV1          (0 << PMC_MCKR_PRES_SHIFT) /* Selected clock */
#  define PMC_MCKR_PRES_DIV2          (1 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 2 */
#  define PMC_MCKR_PRES_DIV4          (2 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 4 */
#  define PMC_MCKR_PRES_DIV8          (3 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 8 */
#  define PMC_MCKR_PRES_DIV16         (4 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 16 */
#  define PMC_MCKR_PRES_DIV32K        (5 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 32 */
#  define PMC_MCKR_PRES_DIV64         (6 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 64 */
#  define PMC_MCKR_PRES_DIV3          (7 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 3 */
#define PMC_MCKR_UPLLDIV              (1 << 13) /* Bit 13: UPLL Divider */

/* PMC Programmable Clock Register (0,1,2) */

#define PMC_PCK_CSS_SHIFT             (0)       /* Bits 0-2: Master Clock Source Selection */
#define PMC_PCK_CSS_MASK              (7 << PMC_PCK_CSS_MASK)
#  define PMC_PCK_CSS_SLOW            (0 << PMC_PCK_CSS_MASK) /* Slow Clock */
#  define PMC_PCK_CSS_MAIN            (1 << PMC_PCK_CSS_MASK) /* Main Clock */
#  define PMC_PCK_CSS_PLLA            (2 << PMC_PCK_CSS_MASK) /* PLLA Clock */
#  define PMC_PCK_CSS_UPLL            (3 << PMC_PCK_CSS_MASK) /* UPLL Clock */
#  define PMC_PCK_CSS_MASTER          (4 << PMC_PCK_CSS_MASK) /* Master Clock */
#define PMC_PCK_PRES_SHIFT            (4)       /* Bits 4-6: Programmable Clock Prescaler */
#define PMC_PCK_PRES_MASK             (7 << PMC_PCK_PRES_SHIFT)
#  define PMC_PCK_PRES_DIV1           (0 << PMC_PCK_PRES_SHIFT) /* Selected clock */
#  define PMC_PCK_PRES_DIV2           (1 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 2 */
#  define PMC_PCK_PRES_DIV4           (2 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 4 */
#  define PMC_PCK_PRES_DIV8           (3 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 8 */
#  define PMC_PCK_PRES_DIV16          (4 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 16 */
#  define PMC_PCK_PRES_DIV32K         (5 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 32 */
#  define PMC_PCK_PRES_DIV64          (6 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 64 */

/* PMC Interrupt Enable Register, PMC Interrupt Disable Register, PMC Status Register,
 * and PMC Interrupt Mask Register common bit-field definitions
 */

#define PMC_INT_MOSCXTS               (1 << 0)  /* Bit 0:  Main Crystal Oscillator Status Interrupt */
#define PMC_INT_LOCKA                 (1 << 1)  /* Bit 1:  PLL A Lock Interrupt */
#define PMC_INT_MCKRDY                (1 << 3)  /* Bit 3:  Master Clock Ready Interrupt */
#define PMC_INT_LOCKU                 (1 << 6)  /* Bit 6:  UTMI PLL Lock Interrupt */
#define PMC_SR_OSCSELS                (1 << 7)  /* Bit 7: Slow Clock Oscillator Selection (SR only) */
#define PMC_INT_PCKRDY(n)             (1<<((n)+8)
#define PMC_INT_PCKRDY0               (1 << 8)  /* Bit 8:  Programmable Clock Ready 0 Interrupt */
#define PMC_INT_PCKRDY1               (1 << 9)  /* Bit 9:  Programmable Clock Ready 1 Interrupt */
#define PMC_INT_PCKRDY2               (1 << 10) /* Bit 10: Programmable Clock Ready 2 Interrupt */
#define PMC_INT_MOSCSELS              (1 << 16) /* Bit 16: Main Oscillator Selection Status Interrupt */
#define PMC_INT_MOSCRCS               (1 << 17) /* Bit 17: Main On-Chip RC Status Interrupt */
#define PMC_INT_CFDEV                 (1 << 18) /* Bit 18: Clock Failure Detector Event Interrupt */
#define PMC_SR_CFDS                   (1 << 19) /* Bit 19: Clock Failure Detector Status (SR only) */
#define PMC_SR_FOS                    (1 << 20) /* Bit 20: Clock Failure Detector Fault Output Status (SR only) */

/* PMC Fast Startup Mode Register and PMC Fast Startup Polarity Register common bit-field
 * definitions
 */

#define PMC_FSTI(n)                   (1<<(n))
#define PMC_FSTI0                     (1 << 0)  /* Bit 0:  Fast Startup Input 0 */
#define PMC_FSTI1                     (1 << 1)  /* Bit 1:  Fast Startup Input 1 */
#define PMC_FSTI2                     (1 << 2)  /* Bit 2:  Fast Startup Input 2 */
#define PMC_FSTI3                     (1 << 3)  /* Bit 3:  Fast Startup Input 3 */
#define PMC_FSTI4                     (1 << 4)  /* Bit 4:  Fast Startup Input 4 */
#define PMC_FSTI5                     (1 << 5)  /* Bit 5:  Fast Startup Input 5 */
#define PMC_FSTI6                     (1 << 6)  /* Bit 6:  Fast Startup Input 6 */
#define PMC_FSTI7                     (1 << 7)  /* Bit 7:  Fast Startup Input 7 */
#define PMC_FSTI8                     (1 << 8)  /* Bit 8:  Fast Startup Input 8 */
#define PMC_FSTI9                     (1 << 9)  /* Bit 9:  Fast Startup Input 9 */
#define PMC_FSTI10                    (1 << 10) /* Bit 10: Fast Startup Input 10 */
#define PMC_FSTI11                    (1 << 11) /* Bit 11: Fast Startup Input 11 */
#define PMC_FSTI12                    (1 << 12) /* Bit 12: Fast Startup Input 12 */
#define PMC_FSTI13                    (1 << 13) /* Bit 13: Fast Startup Input 13 */
#define PMC_FSTI14                    (1 << 14) /* Bit 14: Fast Startup Input 14 */
#define PMC_FSTI15                    (1 << 15) /* Bit 15: Fast Startup Input 15 */

#define PMC_FSMR_RTTAL                (1 << 16) /* Bit 16: RTT Alarm Enable (MR only) */
#define PMC_FSMR_RTCAL                (1 << 17) /* Bit 17: RTC Alarm Enable (MR only) */
#define PMC_FSMR_USBAL                (1 << 18) /* Bit 18: USB Alarm Enable (MR only) */
#define PMC_FSMR_LPM                  (1 << 20) /* Bit 20: Low Power Mode (MR only) */

/* PMC Fault Output Clear Register */

#define PMC_FOCLR                     (1 << 0)  /* Bit 0:  Fault Output Clear */

/* PMC Write Protect Mode Register */

#define PMC_WPMR_WPEN                 (1 << 0)  /* Bit 0:  Write Protect Enable */
#define PMC_WPMR_WPKEY_SHIFT          (8)       /* Bits 8-31: Write Protect KEY */
#define PMC_WPMR_WPKEY_MASK           (0x00ffffff << PMC_WPMR_WPKEY_SHIFT)

/* PMC Write Protect Status Register */

#define PMC_WPSR_WPVS                 (1 << 0)  /* Bit 0:  Write Protect Violation Status */
#define PMC_WPSR_WPVSRC_SHIFT         (8)       /* Bits 8-23: Write Protect Violation Source */
#define PMC_WPSR_WPVSRC_MASK          (0xffff << PMC_WPSR_WPVSRC_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_PMC_H */
