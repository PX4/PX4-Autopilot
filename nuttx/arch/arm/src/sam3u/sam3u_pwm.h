/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_pwm.h
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_PWM_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_PWM_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* PWM register offsets *****************************************************************/

#define SAM3U_PWM_CLK_OFFSET           0x000 /* PWM Clock Register */
#define SAM3U_PWM_ENA_OFFSET           0x004 /* PWM Enable Register */
#define SAM3U_PWM_DIS_OFFSET           0x008 /* PWM Disable Register */
#define SAM3U_PWM_SR_OFFSET            0x00c /* PWM Status Register */
#define SAM3U_PWM_IER1_OFFSET          0x010 /* PWM Interrupt Enable Register 1 */
#define SAM3U_PWM_IDR1_OFFSET          0x014 /* PWM Interrupt Disable Register 1 */
#define SAM3U_PWM_IMR1_OFFSET          0x018 /* PWM Interrupt Mask Register 1 */
#define SAM3U_PWM_ISR1_OFFSET          0x01c /* PWM Interrupt Status Register 1 */
#define SAM3U_PWM_SCM_OFFSET           0x020 /* PWM Sync Channels Mode Register */
                                             /* 0x024: Reserved */
#define SAM3U_PWM_SCUC_OFFSET          0x028 /* PWM Sync Channels Update Control Register */
#define SAM3U_PWM_SCUP_OFFSET          0x02c /* PWM Sync Channels Update Period Register */
#define SAM3U_PWM_SCUPUPD_OFFSET       0x030 /* PWM Sync Channels Update Period Update Register */
#define SAM3U_PWM_IER2_OFFSET          0x034 /* PWM Interrupt Enable Register 2 */
#define SAM3U_PWM_IDR2_OFFSET          0x038 /* PWM Interrupt Disable Register 2 */
#define SAM3U_PWM_IMR2_OFFSET          0x03c /* PWM Interrupt Mask Register 2 */
#define SAM3U_PWM_ISR2_OFFSET          0x040 /* PWM Interrupt Status Register 2 */
#define SAM3U_PWM_OOV_OFFSET           0x044 /* PWM Output Override Value Register */
#define SAM3U_PWM_OS_OFFSET            0x048 /* PWM Output Selection Register */
#define SAM3U_PWM_OSS_OFFSET           0x04c /* PWM Output Selection Set Register */
#define SAM3U_PWM_OSC_OFFSET           0x050 /* PWM Output Selection Clear Register */
#define SAM3U_PWM_OSSUPD_OFFSET        0x054 /* PWM Output Selection Set Update Register */
#define SAM3U_PWM_OSCUPD_OFFSET        0x058 /* PWM Output Selection Clear Update Register */
#define SAM3U_PWM_FMR_OFFSET           0x05c /* PWM Fault Mode Register */
#define SAM3U_PWM_FSR_OFFSET           0x060 /* PWM Fault Status Register */
#define SAM3U_PWM_FCR_OFFSET           0x064 /* PWM Fault Clear Register */
#define SAM3U_PWM_FPV_OFFSET           0x068 /* PWM Fault Protection Value Register */
#define SAM3U_PWM_FPE_OFFSET           0x06c /* PWM Fault Protection Enable Register */
                                             /* 0x070-0x078: Reserved */
#define SAM3U_PWM_EL0MR_OFFSET         0x07c /* PWM Event Line 0 Mode Register */
#define SAM3U_PWM_EL1MR_OFFSET         0x080 /* PWM Event Line 1 Mode Register */
                                             /* 0x084-0x0ac: Reserved */
                                             /* 0x0b4-0x0e0: Reserved */
#define SAM3U_PWM_WPCR_OFFSET          0x0e4 /* PWM Write Protect Control Register */
#define SAM3U_PWM_WPSR_OFFSET          0x0e8 /* PWM Write Protect Status Register */
                                             /* 0x100-0x128: Reserved for PDC registers */
                                             /* 0x12c: Reserved */
/* PWM Comparison Registers */

#define SAM3U_PWMCMP_OFFSET(n)        (0x130+((n)<<4))
#define SAM3U_PWMCMP_V_OFFSET          0x00  /* PWM Comparison Value Register */
#define SAM3U_PWMCMP_VUPD_OFFSET       0x04  /* PWM Comparison Value Update Register */
#define SAM3U_PWMCMP_M_OFFSET          0x08  /* PWM Comparison Mode Register */
#define SAM3U_PWMCMP_MUPD_OFFSET       0x0c  /* PWM Comparison Mode Update Register */

#define SAM3U_PWMCMP0_V_OFFSET         0x130 /* PWM Comparison 0 Value Register */
#define SAM3U_PWMCMP0_VUPD_OFFSET      0x134 /* PWM Comparison 0 Value Update Register */
#define SAM3U_PWMCMP0_M_OFFSET         0x138 /* PWM Comparison 0 Mode Register */
#define SAM3U_PWMCMP0_MUPD_OFFSET      0x13c /* PWM Comparison 0 Mode Update Register */

#define SAM3U_PWMCMP1_V_OFFSET         0x140 /* PWM Comparison 1 Value Register */
#define SAM3U_PWMCMP1_VUPD_OFFSET      0x144 /* PWM Comparison 1 Value Update Register */
#define SAM3U_PWMCMP1_M_OFFSET         0x148 /* PWM Comparison 1 Mode Register */
#define SAM3U_PWMCMP1_MUPD_OFFSET      0x14c /* PWM Comparison 1 Mode Update Register */

#define SAM3U_PWMCMP2_V_OFFSET         0x150 /* PWM Comparison 2 Value Register */
#define SAM3U_PWMCMP2_VUPD_OFFSET      0x154 /* PWM Comparison 2 Value Update Register */
#define SAM3U_PWMCMP2_M_OFFSET         0x158 /* PWM Comparison 2 Mode Register */
#define SAM3U_PWMCMP2_MUPD_OFFSET      0x15c /* PWM Comparison 2 Mode Update Register */

#define SAM3U_PWMCMP3_V_OFFSET         0x160 /* PWM Comparison 3 Value Register */
#define SAM3U_PWMCMP3_VUPD_OFFSET      0x164 /* PWM Comparison 3 Value Update Register */
#define SAM3U_PWMCMP3_M_OFFSET         0x168 /* PWM Comparison 3 Mode Register */
#define SAM3U_PWMCMP3_MUPD_OFFSET      0x16c /* PWM Comparison 3 Mode Update Register */

#define SAM3U_PWMCMP4_V_OFFSET         0x170 /* PWM Comparison 4 Value Register */
#define SAM3U_PWMCMP4_VUPD_OFFSET      0x174 /* PWM Comparison 4 Value Update Register */
#define SAM3U_PWMCMP4_M_OFFSET         0x178 /* PWM Comparison 4 Mode Register */
#define SAM3U_PWMCMP4_MUPD_OFFSET      0x17c /* PWM Comparison 4 Mode Update Register */

#define SAM3U_PWMCMP5_V_OFFSET         0x180 /* PWM Comparison 5 Value Register */
#define SAM3U_PWMCMP5_VUPD_OFFSET      0x184 /* PWM Comparison 5 Value Update Register */
#define SAM3U_PWMCMP5_M_OFFSET         0x188 /* PWM Comparison 5 Mode Register */
#define SAM3U_PWMCMP5_MUPD_OFFSET      0x18c /* PWM Comparison 5 Mode Update Register */

#define SAM3U_PWMCMP6_V_OFFSET         0x190 /* PWM Comparison 6 Value Register */
#define SAM3U_PWMCMP6_VUPD_OFFSET      0x194 /* PWM Comparison 6 Value Update Register */
#define SAM3U_PWMCMP6_M_OFFSET         0x198 /* PWM Comparison 6 Mode Register */
#define SAM3U_PWMCMP6_MUPD_OFFSET      0x19c /* PWM Comparison 6 Mode Update Register */

#define SAM3U_PWMCMP7_V_OFFSET         0x1a0 /* PWM Comparison 7 Value Register */
#define SAM3U_PWMCMP7_VUPD_OFFSET      0x1a4 /* PWM Comparison 7 Value Update Register */
#define SAM3U_PWMCMP7_M_OFFSET         0x1a8 /* PWM Comparison 7 Mode Register */
#define SAM3U_PWMCMP7_MUPD_OFFSET      0x1ac /* PWM Comparison 7 Mode Update Register */
                                             /* 0x1b0-0x1fc: Reserved */
/* PWM Channel Registers */

#define SAM3U_PWMCH_OFFSET(n)          (0x200+((n)<< 5))
#define SAM3U_PWMCH_MR_OFFSET          0x00  /* PWM Channel Mode Register */
#define SAM3U_PWMCH_DTY_OFFSET         0x04  /* PWM Channel Duty Cycle Register */
#define SAM3U_PWMCH_DTYUPD_OFFSET      0x08  /* PWM Channel Duty Cycle Update Register */
#define SAM3U_PWMCH_PRD_OFFSET         0x0c  /* PWM Channel Period Register */
#define SAM3U_PWMCH_PRDUPD_OFFSET      0x10  /* PWM Channel Period Update Register */
#define SAM3U_PWMCH_CCNT_OFFSET        0x14  /* PWM Channel Counter Register */
#define SAM3U_PWMCH_DT_OFFSET          0x18  /* PWM Channel Dead Time Register */
#define SAM3U_PWMCH_DTUPD_OFFSET       0x1c  /* PWM Channel Dead Time Update Register */

#define SAM3U_PWMCH0_MR_OFFSET         0x200 /* PWM Channel 0 Mode Register */
#define SAM3U_PWMCH0_DTY_OFFSET        0x204 /* PWM Channel 0 Duty Cycle Register */
#define SAM3U_PWMCH0_DTYUPD_OFFSET     0x208 /* PWM Channel 0 Duty Cycle Update Register */
#define SAM3U_PWMCH0_PRD_OFFSET        0x20c /* PWM Channel 0 Period Register */
#define SAM3U_PWMCH0_PRDUPD_OFFSET     0x210 /* PWM Channel 0 Period Update Register */
#define SAM3U_PWMCH0_CCNT_OFFSET       0x214 /* PWM Channel 0 Counter Register */
#define SAM3U_PWMCH0_DT_OFFSET         0x218 /* PWM Channel 0 Dead Time Register */
#define SAM3U_PWMCH0_DTUPD_OFFSET      0x21c /* PWM Channel 0 Dead Time Update Register */

#define SAM3U_PWMCH1_MR_OFFSET         0x220 /* PWM Channel 1 Mode Register */
#define SAM3U_PWMCH1_DTY_OFFSET        0x224 /* PWM Channel 1 Duty Cycle Register */
#define SAM3U_PWMCH1_DTYUPD_OFFSET     0x228 /* PWM Channel 1 Duty Cycle Update Register */
#define SAM3U_PWMCH1_PRD_OFFSET        0x22c /* PWM Channel 1 Period Register */
#define SAM3U_PWMCH1_PRDUPD_OFFSET     0x230 /* PWM Channel 1 Period Update Register */
#define SAM3U_PWMCH1_CCNT_OFFSET       0x234 /* PWM Channel 1 Counter Register */
#define SAM3U_PWMCH1_DT_OFFSET         0x238 /* PWM Channel 1 Dead Time Register */
#define SAM3U_PWMCH1_DTUPD_OFFSET      0x23c /* PWM Channel 1 Dead Time Update Register */

#define SAM3U_PWMCH2_MR_OFFSET         0x240 /* PWM Channel 2 Mode Register */
#define SAM3U_PWMCH2_DTY_OFFSET        0x244 /* PWM Channel 2 Duty Cycle Register */
#define SAM3U_PWMCH2_DTYUPD_OFFSET     0x248 /* PWM Channel 2 Duty Cycle Update Register */
#define SAM3U_PWMCH2_PRD_OFFSET        0x24c /* PWM Channel 2 Period Register */
#define SAM3U_PWMCH2_PRDUPD_OFFSET     0x250 /* PWM Channel 2 Period Update Register */
#define SAM3U_PWMCH2_CCNT_OFFSET       0x254 /* PWM Channel 2 Counter Register */
#define SAM3U_PWMCH2_DT_OFFSET         0x258 /* PWM Channel 2 Dead Time Register */
#define SAM3U_PWMCH2_DTUPD_OFFSET      0x25c /* PWM Channel 2 Dead Time Update Register */

#define SAM3U_PWMCH3_MR_OFFSET         0x260 /* PWM Channel 3 Mode Register */
#define SAM3U_PWMCH3_DTY_OFFSET        0x264 /* PWM Channel 3 Duty Cycle Register */
#define SAM3U_PWMCH3_DTYUPD_OFFSET     0x268 /* PWM Channel 3 Duty Cycle Update Register */
#define SAM3U_PWMCH3_PRD_OFFSET        0x26c /* PWM Channel 3 Period Register */
#define SAM3U_PWMCH3_PRDUPD_OFFSET     0x270 /* PWM Channel 3 Period Update Register */
#define SAM3U_PWMCH3_CCNT_OFFSET       0x274 /* PWM Channel 3 Counter Register */
#define SAM3U_PWMCH3_DT_OFFSET         0x278 /* PWM Channel 3 Dead Time Register */
#define SAM3U_PWMCH3_DTUPD_OFFSET      0x27c /* PWM Channel 3 Dead Time Update Register */

/* PWM register adresses ****************************************************************/

#define SAM3U_PWM_CLK                  (SAM3U_PWM_BASE+SAM3U_PWM_CLK_OFFSET)
#define SAM3U_PWM_ENA                  (SAM3U_PWM_BASE+SAM3U_PWM_ENA_OFFSET)
#define SAM3U_PWM_DIS                  (SAM3U_PWM_BASE+SAM3U_PWM_DIS_OFFSET)
#define SAM3U_PWM_SR                   (SAM3U_PWM_BASE+SAM3U_PWM_SR_OFFSET)
#define SAM3U_PWM_IER1                 (SAM3U_PWM_BASE+SAM3U_PWM_IER1_OFFSET)
#define SAM3U_PWM_IDR1                 (SAM3U_PWM_BASE+SAM3U_PWM_IDR1_OFFSET)
#define SAM3U_PWM_IMR1                 (SAM3U_PWM_BASE+SAM3U_PWM_IMR1_OFFSET)
#define SAM3U_PWM_ISR1                 (SAM3U_PWM_BASE+SAM3U_PWM_ISR1_OFFSET)
#define SAM3U_PWM_SCM                  (SAM3U_PWM_BASE+SAM3U_PWM_SCM_OFFSET)
#define SAM3U_PWM_SCUC                 (SAM3U_PWM_BASE+SAM3U_PWM_SCUC_OFFSET)
#define SAM3U_PWM_SCUP                 (SAM3U_PWM_BASE+SAM3U_PWM_SCUP_OFFSET)
#define SAM3U_PWM_SCUPUPD              (SAM3U_PWM_BASE+SAM3U_PWM_SCUPUPD_OFFSET)
#define SAM3U_PWM_IER2                 (SAM3U_PWM_BASE+SAM3U_PWM_IER2_OFFSET)
#define SAM3U_PWM_IDR2                 (SAM3U_PWM_BASE+SAM3U_PWM_IDR2_OFFSET)
#define SAM3U_PWM_IMR2                 (SAM3U_PWM_BASE+SAM3U_PWM_IMR2_OFFSET)
#define SAM3U_PWM_ISR2                 (SAM3U_PWM_BASE+SAM3U_PWM_ISR2_OFFSET)
#define SAM3U_PWM_OOV                  (SAM3U_PWM_BASE+SAM3U_PWM_OOV_OFFSET)
#define SAM3U_PWM_OS                   (SAM3U_PWM_BASE+SAM3U_PWM_OS_OFFSET)
#define SAM3U_PWM_OSS                  (SAM3U_PWM_BASE+SAM3U_PWM_OSS_OFFSET)
#define SAM3U_PWM_OSC                  (SAM3U_PWM_BASE+SAM3U_PWM_OSC_OFFSET)
#define SAM3U_PWM_OSSUPD               (SAM3U_PWM_BASE+SAM3U_PWM_OSSUPD_OFFSET)
#define SAM3U_PWM_OSCUPD               (SAM3U_PWM_BASE+SAM3U_PWM_OSCUPD_OFFSET)
#define SAM3U_PWM_FMR                  (SAM3U_PWM_BASE+SAM3U_PWM_FMR_OFFSET)
#define SAM3U_PWM_FSR                  (SAM3U_PWM_BASE+SAM3U_PWM_FSR_OFFSET)
#define SAM3U_PWM_FCR                  (SAM3U_PWM_BASE+SAM3U_PWM_FCR_OFFSET)
#define SAM3U_PWM_FPV                  (SAM3U_PWM_BASE+SAM3U_PWM_FPV_OFFSET)
#define SAM3U_PWM_FPE                  (SAM3U_PWM_BASE+SAM3U_PWM_FPE_OFFSET)
#define SAM3U_PWM_EL0MR                (SAM3U_PWM_BASE+SAM3U_PWM_EL0MR_OFFSET)
#define SAM3U_PWM_EL1MR                (SAM3U_PWM_BASE+SAM3U_PWM_EL1MR_OFFSET)
#define SAM3U_PWM_WPCR                 (SAM3U_PWM_BASE+SAM3U_PWM_WPCR_OFFSET)
#define SAM3U_PWM_WPSR                 (SAM3U_PWM_BASE+SAM3U_PWM_WPSR_OFFSET)

/* PWM Comparison Registers */

#define SAM3U_PWCMP_BASE(n)            (SAM3U_PWM_BASE+SAM3U_PWCMP_OFFSET(n))
#define SAM3U_PWMCMP0_BASE             (SAM3U_PWM_BASE+0x0130)
#define SAM3U_PWMCMP1_BASE             (SAM3U_PWM_BASE+0x0140)
#define SAM3U_PWMCMP2_BASE             (SAM3U_PWM_BASE+0x0150)
#define SAM3U_PWMCMP3_BASE             (SAM3U_PWM_BASE+0x0160)
#define SAM3U_PWMCMP4_BASE             (SAM3U_PWM_BASE+0x0170)
#define SAM3U_PWMCMP5_BASE             (SAM3U_PWM_BASE+0x0180)
#define SAM3U_PWMCMP6_BASE             (SAM3U_PWM_BASE+0x0190)
#define SAM3U_PWMCMP7_BASE             (SAM3U_PWM_BASE+0x01a0)

#define SAM3U_PWMCMP0_V                (SAM3U_PWMCMP0_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP0_VUPD             (SAM3U_PWMCMP0_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP0_M                (SAM3U_PWMCMP0_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP0_MUPD             (SAM3U_PWMCMP0_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP1_V                (SAM3U_PWMCMP1_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP1_VUPD             (SAM3U_PWMCMP1_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP1_M                (SAM3U_PWMCMP1_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP1_MUPD             (SAM3U_PWMCMP1_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP2_V                (SAM3U_PWMCMP2_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP2_VUPD             (SAM3U_PWMCMP2_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP2_M                (SAM3U_PWMCMP2_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP2_MUPD             (SAM3U_PWMCMP2_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP3_V                (SAM3U_PWMCMP3_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP3_VUPD             (SAM3U_PWMCMP3_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP3_M                (SAM3U_PWMCMP3_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP3_MUPD             (SAM3U_PWMCMP3_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP4_V                (SAM3U_PWMCMP4_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP4_VUPD             (SAM3U_PWMCMP4_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP4_M                (SAM3U_PWMCMP4_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP4_MUPD             (SAM3U_PWMCMP4_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP5_V                (SAM3U_PWMCMP5_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP5_VUPD             (SAM3U_PWMCMP5_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP5_M                (SAM3U_PWMCMP5_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP5_MUPD             (SAM3U_PWMCMP5_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP6_V                (SAM3U_PWMCMP6_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP6_VUPD             (SAM3U_PWMCMP6_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP6_M                (SAM3U_PWMCMP6_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP6_MUPD             (SAM3U_PWMCMP6_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

#define SAM3U_PWMCMP7_V                (SAM3U_PWMCMP7_BASE+SAM3U_PWMCMP_V_OFFSET)
#define SAM3U_PWMCMP7_VUPD             (SAM3U_PWMCMP7_BASE+SAM3U_PWMCMP_VUPD_OFFSET)
#define SAM3U_PWMCMP7_M                (SAM3U_PWMCMP7_BASE+SAM3U_PWMCMP_M_OFFSET)
#define SAM3U_PWMCMP7_MUPD             (SAM3U_PWMCMP7_BASE+SAM3U_PWMCMP_MUPD_OFFSET)

/* PWM Channel Registers */

#define SAM3U_PWCH_BASE(n)             (SAM3U_PWM_BASE+SAM3U_PWCH_OFFSET(n))
#define SAM3U_PWMCH0_BASE              (SAM3U_PWM_BASE+0x0200)
#define SAM3U_PWMCH1_BASE              (SAM3U_PWM_BASE+0x0220)
#define SAM3U_PWMCH2_BASE              (SAM3U_PWM_BASE+0x0240)
#define SAM3U_PWMCH3_BASE              (SAM3U_PWM_BASE+0x0260)

#define SAM3U_PWMCH0_MR                (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_MR_OFFSET)
#define SAM3U_PWMCH0_DTY               (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_DTY_OFFSET)
#define SAM3U_PWMCH0_DTYUPD            (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_DTYUPD_OFFSET)
#define SAM3U_PWMCH0_PRD               (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_PRD_OFFSET)
#define SAM3U_PWMCH0_PRDUPD            (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_PRDUPD_OFFSET)
#define SAM3U_PWMCH0_CCNT              (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_CCNT_OFFSET)
#define SAM3U_PWMCH0_DT                (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_DT_OFFSET)
#define SAM3U_PWMCH0_DTUPD             (SAM3U_PWMCH0_BASE+SAM3U_PWMCH_DTUPD_OFFSET)

#define SAM3U_PWMCH1_MR                (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_MR_OFFSET)
#define SAM3U_PWMCH1_DTY               (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_DTY_OFFSET)
#define SAM3U_PWMCH1_DTYUPD            (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_DTYUPD_OFFSET)
#define SAM3U_PWMCH1_PRD               (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_PRD_OFFSET)
#define SAM3U_PWMCH1_PRDUPD            (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_PRDUPD_OFFSET)
#define SAM3U_PWMCH1_CCNT              (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_CCNT_OFFSET)
#define SAM3U_PWMCH1_DT                (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_DT_OFFSET)
#define SAM3U_PWMCH1_DTUPD             (SAM3U_PWMCH1_BASE+SAM3U_PWMCH_DTUPD_OFFSET)

#define SAM3U_PWMCH2_MR                (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_MR_OFFSET)
#define SAM3U_PWMCH2_DTY               (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_DTY_OFFSET)
#define SAM3U_PWMCH2_DTYUPD            (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_DTYUPD_OFFSET)
#define SAM3U_PWMCH2_PRD               (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_PRD_OFFSET)
#define SAM3U_PWMCH2_PRDUPD            (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_PRDUPD_OFFSET)
#define SAM3U_PWMCH2_CCNT              (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_CCNT_OFFSET)
#define SAM3U_PWMCH2_DT                (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_DT_OFFSET)
#define SAM3U_PWMCH2_DTUPD             (SAM3U_PWMCH2_BASE+SAM3U_PWMCH_DTUPD_OFFSET)

#define SAM3U_PWMCH3_MR                (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_MR_OFFSET)
#define SAM3U_PWMCH3_DTY               (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_DTY_OFFSET)
#define SAM3U_PWMCH3_DTYUPD            (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_DTYUPD_OFFSET)
#define SAM3U_PWMCH3_PRD               (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_PRD_OFFSET)
#define SAM3U_PWMCH3_PRDUPD            (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_PRDUPD_OFFSET)
#define SAM3U_PWMCH3_CCNT              (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_CCNT_OFFSET)
#define SAM3U_PWMCH3_DT                (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_DT_OFFSET)
#define SAM3U_PWMCH3_DTUPD             (SAM3U_PWMCH3_BASE+SAM3U_PWMCH_DTUPD_OFFSET)

/* PWM register bit definitions *********************************************************/

/* PWM Clock Register */

#define PWM_CLK_DIVA_SHIFT             (0)       /* Bits 0-7: CLKA Divide Factor */
#define PWM_CLK_DIVA_MASK              (0xff << PWM_CLK_DIVA_SHIFT)
#define PWM_CLK_PREA_SHIFT             (8)       /* Bits 8-11: CLKA Source Clock Selection */
#define PWM_CLK_PREA_MASK              (15 << PWM_CLK_PREA_SHIFT)
#  define PWM_CLK_PREA_MCK             (0  << PWM_CLK_PREA_SHIFT) /* MCK */
#  define PWM_CLK_PREA_MCKDIV2         (1  << PWM_CLK_PREA_SHIFT) /* MCK/2 */
#  define PWM_CLK_PREA_MCKDIV4         (2  << PWM_CLK_PREA_SHIFT) /* MCK/4 */
#  define PWM_CLK_PREA_MCKDIV8         (3  << PWM_CLK_PREA_SHIFT) /* MCK/8 */
#  define PWM_CLK_PREA_MCKDIV16        (4  << PWM_CLK_PREA_SHIFT) /* MCK/16 */
#  define PWM_CLK_PREA_MCKDIV32        (5  << PWM_CLK_PREA_SHIFT) /* MCK/32 */
#  define PWM_CLK_PREA_MCKDIV64        (6  << PWM_CLK_PREA_SHIFT) /* MCK/64 */
#  define PWM_CLK_PREA_MCKDIV128       (7  << PWM_CLK_PREA_SHIFT) /* MCK/128 */
#  define PWM_CLK_PREA_MCKDIV256       (8  << PWM_CLK_PREA_SHIFT) /* MCK/256 */
#  define PWM_CLK_PREA_MCKDIV512       (9  << PWM_CLK_PREA_SHIFT) /* MCK/512 */
#  define PWM_CLK_PREA_MCKDIV1024      (10 << PWM_CLK_PREA_SHIFT) /* MCK/1024 */
#define PWM_CLK_DIVB_SHIFT             (16)      /* Bits 16-23: CLKB Divide Factor */
#define PWM_CLK_DIVB_MASK              (0xff << PWM_CLK_DIVB_SHIFT)
#define PWM_CLK_PREB_SHIFT             (24)      /* Bit 24-27: CLKB Source Clock Selection */
#define PWM_CLK_PREB_MASK              (15 << PWM_CLK_PREB_SHIFT)
#  define PWM_CLK_PREB_MCK             (0  << PWM_CLK_PREB_SHIFT) /* MCK */
#  define PWM_CLK_PREB_MCKDIV2         (1  << PWM_CLK_PREB_SHIFT) /* MCK/2 */
#  define PWM_CLK_PREB_MCKDIV4         (2  << PWM_CLK_PREB_SHIFT) /* MCK/4 */
#  define PWM_CLK_PREB_MCKDIV8         (3  << PWM_CLK_PREB_SHIFT) /* MCK/8 */
#  define PWM_CLK_PREB_MCKDIV16        (4  << PWM_CLK_PREB_SHIFT) /* MCK/16 */
#  define PWM_CLK_PREB_MCKDIV32        (5  << PWM_CLK_PREB_SHIFT) /* MCK/32 */
#  define PWM_CLK_PREB_MCKDIV64        (6  << PWM_CLK_PREB_SHIFT) /* MCK/64 */
#  define PWM_CLK_PREB_MCKDIV128       (7  << PWM_CLK_PREB_SHIFT) /* MCK/128 */
#  define PWM_CLK_PREB_MCKDIV256       (8  << PWM_CLK_PREB_SHIFT) /* MCK/256 */
#  define PWM_CLK_PREB_MCKDIV512       (9  << PWM_CLK_PREB_SHIFT) /* MCK/512 */
#  define PWM_CLK_PREB_MCKDIV1024      (10 << PWM_CLK_PREB_SHIFT) /* MCK/1024 */

/* PWM Enable Register, PWM Disable Register, and PWM Status Register common bit-field definitions */

#define SAM3U_ENAB_CHID(n)             (1 << ((n))
#define SAM3U_ENAB_CHID0               (1 << 0)  /* Bit 0:  Counter Event Channel 0 Interrupt */
#define SAM3U_ENAB_CHID1               (1 << 1)  /* Bit 1:  Counter Event Channel 1 Interrupt */
#define SAM3U_ENAB_CHID2               (1 << 2)  /* Bit 2:  Counter Event Channel 2 Interrupt */
#define SAM3U_ENAB_CHID3               (1 << 3)  /* Bit 3:  Counter Event Channel 3 Interrupt */

/* PWM Interrupt Enable Register 1, PWM Interrupt Disable Register 1, PWM Interrupt
 * Mask Register 1, and PWM Interrupt Status Register 1 common bit definitions
 */

#define SAM3U_INT_CHID(n)              (1 << (n))
#define SAM3U_INT_CHID0                (1 << 0)  /* Bit 0:  Counter Event Channel 0 Interrupt */
#define SAM3U_INT_CHID1                (1 << 1)  /* Bit 1:  Counter Event Channel 1 Interrupt */
#define SAM3U_INT_CHID2                (1 << 2)  /* Bit 2:  Counter Event Channel 2 Interrupt */
#define SAM3U_INT_CHID3                (1 << 3)  /* Bit 3:  Counter Event Channel 3 Interrupt */
#define SAM3U_INT_FCHID(n)             (1 << ((n)+16))
#define SAM3U_INT_FCHID0               (1 << 16) /* Bit 16: Fault Protection Trigger Channel 0 Interrupt */
#define SAM3U_INT_FCHID1               (1 << 17) /* Bit 17: Fault Protection Trigger Channel 1 Interrupt */
#define SAM3U_INT_FCHID2               (1 << 18) /* Bit 18: Fault Protection Trigger Channel 2 Interrupt */
#define SAM3U_INT_FCHID3               (1 << 19) /* Bit 19: Fault Protection Trigger Channel 3 Interrupt */

/* PWM Sync Channels Mode Register */

#define PWM_SCM_SYNC(n)                (1 << (n))
#define PWM_SCM_SYNC0                  (1 << 0)  /* Bit 0:  Synchronous Channel 0 */
#define PWM_SCM_SYNC1                  (1 << 1)  /* Bit 1:  Synchronous Channel 1 */
#define PWM_SCM_SYNC2                  (1 << 2)  /* Bit 2:  Synchronous Channel 2 */
#define PWM_SCM_SYNC3                  (1 << 3)  /* Bit 3:  Synchronous Channel 3 */
#define PWM_SCM_UPDM_SHIFT             (16)      /* Bits 16-17: Synchronous Channels Update Mode */
#define PWM_SCM_UPDM_MASK              (3 << PWM_SCM_UPDM_SHIFT)
#  define PWM_SCM_UPDM_MANMAN          (0 << PWM_SCM_UPDM_SHIFT) /* Manual write/manual update */
#  define PWM_SCM_UPDM_MANAUTO         (1 << PWM_SCM_UPDM_SHIFT) /* Manual write/automatic update */
#  define PWM_SCM_UPDM_AUTOAUTO        (2 << PWM_SCM_UPDM_SHIFT) /* Auto write/automatic update */
#define PWM_SCM_PTRM                   (1 << 20) /* Bit 20: PDC Transfer Request Mode */
#define PWM_SCM_PTRCS_SHIFT            (21)      /* Bits 21-23: PDC Transfer Request Comparison Selection */
#define PWM_SCM_PTRCS_MASK             (7 << PWM_SCM_PTRCS_SHIFT)

/* PWM Sync Channels Update Control Register */

#define PWM_SCUC_UPDULOCK              (1 << 0)  /* Bit 0:  Synchronous Channels Update Unlock */

/* PWM Sync Channels Update Period Register */

#define PWM_SCUP_UPR_SHIFT             (0)       /* Bits 0-3:  Update Period */
#define PWM_SCUP_UPR_MASK              (15 << PWM_SCUP_UPR_MASK)
#define PWM_SCUP_UPRCNT_SHIFT          (4)       /* Bits 4-7:  Update Period Counter */
#define PWM_SCUP_UPRCNT_MASK           (15 << PWM_SCUP_UPRCNT_SHIFT)

/* PWM Sync Channels Update Period Update Register */

#define PWM_SCUPUPD_SHIFT              (0)       /* Bits 0-3: Update Period Update */
#define PWM_SCUPUPD_MASK               (15 << PWM_SCUPUPD_SHIFT)

/* PWM Interrupt Enable Register 2, PWM Interrupt Disable Register 2, PWM Interrupt Mask Register 2, and PWM Interrupt Status Register 2 common bit-field definitions */

#define SAM3U_INT_WRDY                 (1 << 0)  /* Bit 0:  Write Ready Update Interrupt */
#define SAM3U_INT_ENDTX                (1 << 1)  /* Bit 1:  PDC End of TX Buffer Interrupt */
#define SAM3U_INT_TXBUFE               (1 << 2)  /* Bit 2:  PDC TX Buffer Empty Interrupt */
#define SAM3U_INT_UNRE                 (1 << 3)  /* Bit 3:  Synch Update Underrun Error Interrupt */
#define SAM3U_INT_CMPM(n)              (1 << ((n)+8))
#define SAM3U_INT_CMPM0                (1 << 8)  /* Bit 8:  Comparison 0 Match Interrupt */
#define SAM3U_INT_CMPM1                (1 << 9)  /* Bit 9:  Comparison 1 Match Interrupt */
#define SAM3U_INT_CMPM2                (1 << 10) /* Bit 10: Comparison 2 Match Interrupt */
#define SAM3U_INT_CMPM3                (1 << 11) /* Bit 11: Comparison 3 Match Interrupt */
#define SAM3U_INT_CMPM4                (1 << 12) /* Bit 12: Comparison 4 Match Interrupt */
#define SAM3U_INT_CMPM5                (1 << 13) /* Bit 13: Comparison 5 Match Interrupt */
#define SAM3U_INT_CMPM6                (1 << 14) /* Bit 14: Comparison 6 Match Interrupt */
#define SAM3U_INT_CMPM7                (1 << 15) /* Bit 15: Comparison 7 Match Interrupt */
#define SAM3U_INT_CMPU(n)              (1 << ((n)+16))
#define SAM3U_INT_CMPU0                (1 << 16) /* Bit 16: Comparison o Update Interrupt */
#define SAM3U_INT_CMPU1                (1 << 17) /* Bit 17: Comparison 1 Update Interrupt */
#define SAM3U_INT_CMPU2                (1 << 18) /* Bit 18: Comparison 2 Update Interrupt */
#define SAM3U_INT_CMPU3                (1 << 19) /* Bit 19: Comparison 3 Update Interrupt */
#define SAM3U_INT_CMPU4                (1 << 20) /* Bit 20: Comparison 4 Update Interrupt */
#define SAM3U_INT_CMPU5                (1 << 21) /* Bit 21: Comparison 5 Update Interrupt */
#define SAM3U_INT_CMPU6                (1 << 22) /* Bit 22: Comparison 6 Update Interrupt */
#define SAM3U_INT_CMPU7                (1 << 23) /* Bit 23: Comparison 7 Update Interrupt */

/* PWM Output Override Value Register, PWM Output Selection Register, PWM Output
 * Selection Set Register, PWM Output Selection Clear Register, PWM Output Selection
 * Set Update Register,  and PWM Output Selection Clear Update Register common bit-field
 * definitions
 */

#define PWM_OUT_OH(n)                  (1 << (n))
#define PWM_OUT_OH0                    (1 << 0)  /* Bit 0:  Value for PWMH output of the channel 0 */
#define PWM_OUT_OH1                    (1 << 1)  /* Bit 1:  Value for PWMH output of the channel 1 */
#define PWM_OUT_OH2                    (1 << 2)  /* Bit 2:  Value for PWMH output of the channel 2 */
#define PWM_OUT_OH3                    (1 << 3)  /* Bit 3:  Value for PWMH output of the channel 3 */
#define PWM_OUT_OL(n)                  (1 << ((n)+16))
#define PWM_OUT_OL0                    (1 << 16) /* Bit 16: Value for PWML output of the channel 0 */
#define PWM_OUT_OL1                    (1 << 17) /* Bit 17: Value for PWML output of the channel 1 */
#define PWM_OUT_OL2                    (1 << 18) /* Bit 18: Value for PWML output of the channel 2 */
#define PWM_OUT_OL3                    (1 << 19) /* Bit 19: Value for PWML output of the channel 3 */

/* PWM Fault Mode Register */

#define PWM_FMR_FPOL(n)                (1 << (n))
#define PWM_FMR_FPOL0                  (1 << 0)  /* Bit 0:  Fault 0 Polarity */
#define PWM_FMR_FPOL1                  (1 << 1)  /* Bit 1:  Fault 1 Polarity */
#define PWM_FMR_FPOL2                  (1 << 2)  /* Bit 2:  Fault 2 Polarity */
#define PWM_FMR_FPOL3                  (1 << 3)  /* Bit 3:  Fault 3 Polarity */
#define PWM_FMR_FMOD(n)                (1 << ((n)+8))
#define PWM_FMR_FMOD0                  (1 << 8)  /* Bit 8:  Fault 0 Activation Mode */
#define PWM_FMR_FMOD1                  (1 << 9)  /* Bit 9:  Fault 1 Activation Mode */
#define PWM_FMR_FMOD2                  (1 << 10) /* Bit 10: Fault 2 Activation Mode */
#define PWM_FMR_FMOD3                  (1 << 11) /* Bit 11: Fault 3 Activation Mode */
#define PWM_FMR_FFIL(n)                (1 << ((n)+16))
#define PWM_FMR_FFIL0                  (1 << 16) /* Bit 16: Fault 0 Filter */
#define PWM_FMR_FFIL1                  (1 << 17) /* Bit 17: Fault 1 Filter */
#define PWM_FMR_FFIL2                  (1 << 18) /* Bit 18: Fault 2 Filter */
#define PWM_FMR_FFIL3                  (1 << 19) /* Bit 19: Fault 3 Filter */

/* PWM Fault Status Register */

#define PWM_FSR_FIV(n)                 (1 << (n))
#define PWM_FSR_FIV0                   (1 << 0)  /* Bit 0:  Fault Input 0 Value */
#define PWM_FSR_FIV1                   (1 << 1)  /* Bit 1:  Fault Input 1 Value */
#define PWM_FSR_FIV2                   (1 << 2)  /* Bit 2:  Fault Input 2 Value */
#define PWM_FSR_FIV3                   (1 << 3)  /* Bit 3:  Fault Input 3 Value */
#define PWM_FSR_FS(n)                  (1 << ((n)+8))
#define PWM_FSR_FS0                    (1 << 8)  /* Bit 8:  Fault 0 Status */
#define PWM_FSR_FS1                    (1 << 9)  /* Bit 9:  Fault 1 Status */
#define PWM_FSR_FS2                    (1 << 10) /* Bit 10: Fault 2 Status */
#define PWM_FSR_FS3                    (1 << 11) /* Bit 11: Fault 3 Status */

/* PWM Fault Clear Register */

#define PWM_FCR_FCLR(n)                (1 << (n))
#define PWM_FCR_FCLR0                  (1 << 0)  /* Bit 0:  Fault 0 Clear */
#define PWM_FCR_FCLR1                  (1 << 1)  /* Bit 1:  Fault 1 Clear */
#define PWM_FCR_FCLR2                  (1 << 2)  /* Bit 2:  Fault 2 Clear */
#define PWM_FCR_FCLR3                  (1 << 3)  /* Bit 3:  Fault 3 Clear */

/* PWM Fault Protection Value Register */

#define PWM_FPV_FPVH(n)                (1 << (n))
#define PWM_FPV_FPVH0                  (1 << 0)  /* Bit 0:  Fault Protection Value PWMH output channel 0 */
#define PWM_FPV_FPVH1                  (1 << 1)  /* Bit 1:  Fault Protection Value PWMH output channel 1 */
#define PWM_FPV_FPVH2                  (1 << 2)  /* Bit 2:  Fault Protection Value PWMH output channel 2 */
#define PWM_FPV_FPVH3                  (1 << 3)  /* Bit 3:  Fault Protection Value PWMH output channel 3 */
#define PWM_FPV_FPVL(n)                (1 << ((n)+16))
#define PWM_FPV_FPVL0                  (1 << 16) /* Bit 16: Fault Protection Value PWML output channel 0 */
#define PWM_FPV_FPVL1                  (1 << 17) /* Bit 17: Fault Protection Value PWML output channel 1 */
#define PWM_FPV_FPVL2                  (1 << 18) /* Bit 18: Fault Protection Value PWML output channel 2 */
#define PWM_FPV_FPVL3                  (1 << 19) /* Bit 19: Fault Protection Value PWML output channel 3 */

/* PWM Fault Protection Enable Register */

#define PWM_FPE_FPEN(n,y)              (1 << (((n)<<8)+y))
#define PWM_FPE_FPE0(y)                (1 << (y))      /* Bits 0-7:   Fault Protection Enable Fault=y chan=0 */
#define PWM_FPE_FPE1(y)                (1 << ((y)+8))  /* Bits 8-15:  Fault Protection Enable Fault=y chan=1 */
#define PWM_FPE_FPE2(y)                (1 << ((y)+16)) /* Bits 16-23: Fault Protection Enable Fault=y chan=2 */
#define PWM_FPE_FPE3(y)                (1 << ((y)+24)  /* Bits 24-31: Fault Protection Enable Fault=y chan=3 */

/* PWM Event Line 1/2 Register */

#define PWM_ELMR_CSEL(n)               (1 << (n))
#define PWM_ELMR_CSEL0                 (1 << 0)  /* Bit 0:  Comparison 0 Selection */
#define PWM_ELMR_CSEL1                 (1 << 1)  /* Bit 1:  Comparison 1 Selection */
#define PWM_ELMR_CSEL2                 (1 << 2)  /* Bit 2:  Comparison 2 Selection */
#define PWM_ELMR_CSEL3                 (1 << 3)  /* Bit 3:  Comparison 3 Selection */
#define PWM_ELMR_CSEL4                 (1 << 4)  /* Bit 4:  Comparison 4 Selection */
#define PWM_ELMR_CSEL5                 (1 << 5)  /* Bit 5:  Comparison 5 Selection */
#define PWM_ELMR_CSEL6                 (1 << 6)  /* Bit 6:  Comparison 6 Selection */
#define PWM_ELMR_CSEL7                 (1 << 7)  /* Bit 7:  Comparison 7 Selection */

/* PWM Write Protect Control Register */

#define PWM_WPCR_WPCMD_SHIFT           (0)       /* Bits 0-1: Write Protect Command */
#define PWM_WPCR_WPCMD_MASK            (3 << PWM_WPCR_WPCMD_SHIFT)
#define PWM_WPCR_WPRG(n)               (1 << ((n)+2))
#define PWM_WPCR_WPRG0                 (1 << 2)  /* Bit 2:  Write Protect Register Group 0 */
#define PWM_WPCR_WPRG1                 (1 << 3)  /* Bit 3:  Write Protect Register Group 1 */
#define PWM_WPCR_WPRG2                 (1 << 4)  /* Bit 4:  Write Protect Register Group 2 */
#define PWM_WPCR_WPRG3                 (1 << 5)  /* Bit 5:  Write Protect Register Group 3 */
#define PWM_WPCR_WPRG4                 (1 << 6)  /* Bit 6:  Write Protect Register Group 4 */
#define PWM_WPCR_WPRG5                 (1 << 7)  /* Bit 7:  Write Protect Register Group 5 */
#define PWM_WPCR_WPKEY_SHIFT           (8)       /* Bits 8-31:  Write Protect Key */
#define PWM_WPCR_WPKEY_MASK            (0x00ffffff << PWM_WPCR_WPKEY_SHIFT)

/* PWM Write Protect Status Register */

#define PWM_WPSR_WPSWS(n)              (1 << (n))
#define PWM_WPSR_WPSWS0                (1 << 0)  /* Bit 0:  Write Protect SW Status */
#define PWM_WPSR_WPSWS1                (1 << 1)  /* Bit 1:  Write Protect SW Status */
#define PWM_WPSR_WPSWS2                (1 << 2)  /* Bit 2:  Write Protect SW Status */
#define PWM_WPSR_WPSWS3                (1 << 3)  /* Bit 3:  Write Protect SW Status */
#define PWM_WPSR_WPSWS4                (1 << 4)  /* Bit 4:  Write Protect SW Status */
#define PWM_WPSR_WPSWS5                (1 << 5)  /* Bit 5:  Write Protect SW Status */
#define PWM_WPSR_WPVS                  (1 << 7)  /* Bit 7:  Write Protect Violation Status */
#define PWM_WPSR_WPHWS(n)              (1 << ((n)+8))
#define PWM_WPSR_WPHWS0                (1 << 8)  /* Bit 8:  Write Protect HW Status */
#define PWM_WPSR_WPHWS1                (1 << 9)  /* Bit 9:  Write Protect HW Status */
#define PWM_WPSR_WPHWS2                (1 << 10) /* Bit 10: Write Protect HW Status */
#define PWM_WPSR_WPHWS3                (1 << 11) /* Bit 11: Write Protect HW Status */
#define PWM_WPSR_WPHWS4                (1 << 12) /* Bit 12: Write Protect HW Status */
#define PWM_WPSR_WPHWS5                (1 << 13) /* Bit 13: Write Protect HW Status */
#define PWM_WPSR_WPVSRC_SHIFT          (16)      /* Bits 16-31: Write Protect Violation Source */
#define PWM_WPSR_WPVSRC_MASK           (0xffff << PWM_WPSR_WPVSRC_SHIFT)

/* PWM Comparison x Value Register and PWM Comparison x Value Update Register */

#define PWMCMP_CV_SHIFT                (0)       /* Bits 0-23: Comparison x Value */
#define PWMCMP_CV_MASK                 (0x00ffffff << PWMCMP_CV_SHIFT)
#define PWMCMP_CVM                     (1 << 24) /* Bit 24: Comparison x Value Mode */

/* PWM Comparison x Mode Register  and PWM Comparison x Mode Update Register */

#define PWMCMP_CEN                     (1 << 0)  /* Bit 0:  Comparison x Enable */
#define PWMCMP_CTR_SHIFT               (4)       /* Bits 4-7: Comparison x Trigger */
#define PWMCMP_CTR_MASK                (15 << PWMCMP_CTR_SHIFT)
#define PWMCMP_CPR_SHIFT               (8)       /* Bits 8-11: Comparison x Period */
#define PWMCMP_CPR_MASK                (15 << PWMCMP_CPR_SHIFT)
#define PWMCMP_M_CPRCNT_SHIFT          (12)      /* Bits 12-15: Comparison x Period Count (M only) */
#define PWMCMP_M_CPRCNT_MASK           (15 << PWMCMP_M_CPRCNT_SHIFT)
#define PWMCMP_CUPR_SHIFT              (16)      /* Bits 16-19: Comparison x Update Period */
#define PWMCMP_CUPR_MASK               (15 << PWMCMP_CUPR_SHIFT)
#define PWMCMP_M_CUPRCNT_SHIFT         (20)      /* Bits 20-23: Comparison x Update Period Counter (M only) */
#define PWMCMP_M_CUPRCNT_MASK          (15 << PWMCMP_M_CUPRCNT_SHIFT)

/* PWM Channel Mode Register */

#define PWMCH_MR_CPRE_SHIFT            (0)       /* Bits 0-3: Channel Pre-scaler */
#define PWMCH_MR_CPRE_MASK             (15 << PWMCH_MR_CPRE_SHIFT)
#  define PWMCH_MR_CPRE_MCK            (0  << PWMCH_MR_CPRE_SHIFT) /* MCK */
#  define PWMCH_MR_CPRE_MCKDIV2        (1  << PWMCH_MR_CPRE_SHIFT) /* MCK/2 */
#  define PWMCH_MR_CPRE_MCKDIV4        (2  << PWMCH_MR_CPRE_SHIFT) /* MCK/4 */
#  define PWMCH_MR_CPRE_MCKDIV8        (3  << PWMCH_MR_CPRE_SHIFT) /* MCK/8 */
#  define PWMCH_MR_CPRE_MCKDIV16       (4  << PWMCH_MR_CPRE_SHIFT) /* MCK/16 */
#  define PWMCH_MR_CPRE_MCKDIV32       (5  << PWMCH_MR_CPRE_SHIFT) /* MCK/32 */
#  define PWMCH_MR_CPRE_MCKDIV64       (6  << PWMCH_MR_CPRE_SHIFT) /* MCK/64 */
#  define PWMCH_MR_CPRE_MCKDIV128      (7  << PWMCH_MR_CPRE_SHIFT) /* MCK/128 */
#  define PWMCH_MR_CPRE_MCKDIV256      (8  << PWMCH_MR_CPRE_SHIFT) /* MCK/256 */
#  define PWMCH_MR_CPRE_MCKDIV512      (9  << PWMCH_MR_CPRE_SHIFT) /* MCK/512 */
#  define PWMCH_MR_CPRE_MCKDIV1024     (10 << PWMCH_MR_CPRE_SHIFT) /* MCK/1024 */
#  define PWMCH_MR_CPRE_CLKA           (11 << PWMCH_MR_CPRE_SHIFT) /*CLKA */
#  define PWMCH_MR_CPRE_CLKB           (12 << PWMCH_MR_CPRE_SHIFT) /* CLKB */
#define PWMCH_MR_CALG                  (1 << 8)  /* Bit 8:  Channel Alignment */
#define PWMCH_MR_CPOL                  (1 << 9)  /* Bit 9:  Channel Polarity */
#define PWMCH_MR_CES                   (1 << 10) /* Bit 10:  Counter Event Selection */
#define PWMCH_MR_DTE                   (1 << 16) /* Bit 16: Dead-Time Generator Enable */
#define PWMCH_MR_DTHI                  (1 << 17) /* Bit 17: Dead-Time PWMHx Output Inverted */
#define PWMCH_MR_DTLI                  (1 << 18) /* Bit 18: Dead-Time PWMLx Output Inverted */

/* PWM Channel Duty Cycle Register and PWM Channel Duty Cycle Update Register common bit-field definitions */

#define PWMCH_DTY_SHIFT                (0)       /* Bits 0-23: Channel Duty-Cycle */
#define PWMCH_DTY_MASK                 (0x00ffffff << PWMCH_DTY_SHIFT)

/* PWM Channel Period Register and PWM Channel Period Update Register common bit-field definitions */

#define PWMCH_PRD_SHIFT                (0)       /* Bits 0-23: Channel Period */
#define PWMCH_PRD_MASK                 (0x00ffffff << PWMCH_PRD_SHIFT)

/* PWM Channel Counter Register */

#define PWMCH_CCNT_SHIFT               (0)       /* Bits 0-23: Channel Counter Register */
#define PWMCH_CCNT_MASK                (0x00ffffff << PWMCH_CCNT_SHIFT)

/* PWM Channel Dead Time Register and PWM Channel Dead Time Update Register common bit-field definitions */

#define PWMCH_DTH_SHIFT                (0)       /* Bits 0-15: Dead-Time Value for PWMHx Output */
#define PWMCH_DTH_MASK                 (0xffff << PWMCH_DTH_SHIFT)
#define PWMCH_DTL_SHIFT                (16)      /* Bits 16-31: Dead-Time Value for PWMLx Output */
#define PWMCH_DTL_MASK                 (0xffff << PWMCH_DTL_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_PWM_H */
