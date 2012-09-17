/********************************************************************************************
 * arch/arm/src/kinetis/kinetis_ftm.h
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_FTM_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_FTM_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINETIS_FTM_SC_OFFSET       0x0000 /* Status and Control */
#define KINETIS_FTM_CNT_OFFSET      0x0004 /* Counter */
#define KINETIS_FTM_MOD_OFFSET      0x0008 /* Modulo */

#define KINETIS_FTM_CSC_OFFSET(n)   (0x000c+((n)<<3) /* Channel (n) Status and Control */
#define KINETIS_FTM_CV_OFFSET(n)    (0x0010+((n)<<3) /* Channel (n) Value */
#define KINETIS_FTM_C0SC_OFFSET     0x000c /* Channel 0 Status and Control */
#define KINETIS_FTM_C0V_OFFSET      0x0010 /* Channel 0 Value */
#define KINETIS_FTM_C1SC_OFFSET     0x0014 /* Channel 1 Status and Control */
#define KINETIS_FTM_C1V_OFFSET      0x0018 /* Channel 1 Value */
#define KINETIS_FTM_C2SC_OFFSET     0x001c /* Channel 2 Status and Control */
#define KINETIS_FTM_C2V_OFFSET      0x0020 /* Channel 2 Value */
#define KINETIS_FTM_C3SC_OFFSET     0x0024 /* Channel 3 Status and Control */
#define KINETIS_FTM_C3V_OFFSET      0x0028 /* Channel 3 Value */
#define KINETIS_FTM_C4SC_OFFSET     0x002c /* Channel 4 Status and Control */
#define KINETIS_FTM_C4V_OFFSET      0x0030 /* Channel 4 Value */
#define KINETIS_FTM_C5SC_OFFSET     0x0034 /* Channel 5 Status and Control */
#define KINETIS_FTM_C5V_OFFSET      0x0038 /* Channel 5 Value */
#define KINETIS_FTM_C6SC_OFFSET     0x003c /* Channel 6 Status and Control */
#define KINETIS_FTM_C6V_OFFSET      0x0040 /* Channel 6 Value */
#define KINETIS_FTM_C7SC_OFFSET     0x0044 /* Channel 7 Status and Control */
#define KINETIS_FTM_C7V_OFFSET      0x0048 /* Channel 7 Value */

#define KINETIS_FTM_CNTIN_OFFSET    0x004c /* Counter Initial Value */
#define KINETIS_FTM_STATUS_OFFSET   0x0050 /* Capture and Compare Status */
#define KINETIS_FTM_MODE_OFFSET     0x0054 /* Features Mode Selection */
#define KINETIS_FTM_SYNC_OFFSET     0x0058 /* Synchronization */
#define KINETIS_FTM_OUTINIT_OFFSET  0x005c /* Initial State for Channels Output */
#define KINETIS_FTM_OUTMASK_OFFSET  0x0060 /* Output Mask */
#define KINETIS_FTM_COMBINE_OFFSET  0x0064 /* Function for Linked Channels */
#define KINETIS_FTM_DEADTIME_OFFSET 0x0068 /* Deadtime Insertion Control */
#define KINETIS_FTM_EXTTRIG_OFFSET  0x006c /* FTM External Trigger */
#define KINETIS_FTM_POL_OFFSET      0x0070 /* Channels Polarity */
#define KINETIS_FTM_FMS_OFFSET      0x0074 /* Fault Mode Status */
#define KINETIS_FTM_FILTER_OFFSET   0x0078 /* Input Capture Filter Control */
#define KINETIS_FTM_FLTCTRL_OFFSET  0x007c /* Fault Control */
#define KINETIS_FTM_QDCTRL_OFFSET   0x0080 /* Quadrature Decoder Control and Status */
#define KINETIS_FTM_CONF_OFFSET     0x0084 /* Configuration */
#define KINETIS_FTM_FLTPOL_OFFSET   0x0088 /* FTM Fault Input Polarity */
#define KINETIS_FTM_SYNCONF_OFFSET  0x008c /* Synchronization Configuration */
#define KINETIS_FTM_INVCTRL_OFFSET  0x0090 /* FTM Inverting Control */
#define KINETIS_FTM_SWOCTRL_OFFSET  0x0094 /* FTM Software Output Control */
#define KINETIS_FTM_PWMLOAD_OFFSET  0x0098 /* FTM PWM Load */

/* Register Addresses ***********************************************************************/

#define KINETIS_FTM0_SC              (KINETIS_FTM0_BASE+KINETIS_FTM_SC_OFFSET)
#define KINETIS_FTM0_CNT             (KINETIS_FTM0_BASE+KINETIS_FTM_CNT_OFFSET)
#define KINETIS_FTM0_MOD             (KINETIS_FTM0_BASE+KINETIS_FTM_MOD_OFFSET)

#define KINETIS_FTM0_CSC(n)          (KINETIS_FTM0_BASE+KINETIS_FTM_CSC_OFFSET(n))
#define KINETIS_FTM0_CV(n)           (KINETIS_FTM0_BASE+KINETIS_FTM_CV_OFFSET(n))
#define KINETIS_FTM0_C0SC            (KINETIS_FTM0_BASE+KINETIS_FTM_C0SC_OFFSET)
#define KINETIS_FTM0_C0V             (KINETIS_FTM0_BASE+KINETIS_FTM_C0V_OFFSET)
#define KINETIS_FTM0_C1SC            (KINETIS_FTM0_BASE+KINETIS_FTM_C1SC_OFFSET)
#define KINETIS_FTM0_C1V             (KINETIS_FTM0_BASE+KINETIS_FTM_C1V_OFFSET)
#define KINETIS_FTM0_C2SC            (KINETIS_FTM0_BASE+KINETIS_FTM_C2SC_OFFSET)
#define KINETIS_FTM0_C2V             (KINETIS_FTM0_BASE+KINETIS_FTM_C2V_OFFSET)
#define KINETIS_FTM0_C3SC            (KINETIS_FTM0_BASE+KINETIS_FTM_C3SC_OFFSET)
#define KINETIS_FTM0_C3V             (KINETIS_FTM0_BASE+KINETIS_FTM_C3V_OFFSET)
#define KINETIS_FTM0_C4SC            (KINETIS_FTM0_BASE+KINETIS_FTM_C4SC_OFFSET)
#define KINETIS_FTM0_C4V             (KINETIS_FTM0_BASE+KINETIS_FTM_C4V_OFFSET)
#define KINETIS_FTM0_C5SC            (KINETIS_FTM0_BASE+KINETIS_FTM_C5SC_OFFSET)
#define KINETIS_FTM0_C5V             (KINETIS_FTM0_BASE+KINETIS_FTM_C5V_OFFSET)
#define KINETIS_FTM0_C6SC            (KINETIS_FTM0_BASE+KINETIS_FTM_C6SC_OFFSET)
#define KINETIS_FTM0_C6V             (KINETIS_FTM0_BASE+KINETIS_FTM_C6V_OFFSET)
#define KINETIS_FTM0_C7SC            (KINETIS_FTM0_BASE+KINETIS_FTM_C7SC_OFFSET)
#define KINETIS_FTM0_C7V             (KINETIS_FTM0_BASE+KINETIS_FTM_C7V_OFFSET)

#define KINETIS_FTM0_CNTIN           (KINETIS_FTM0_BASE+KINETIS_FTM_CNTIN_OFFSET)
#define KINETIS_FTM0_STATUS          (KINETIS_FTM0_BASE+KINETIS_FTM_STATUS_OFFSET)
#define KINETIS_FTM0_MODE            (KINETIS_FTM0_BASE+KINETIS_FTM_MODE_OFFSET)
#define KINETIS_FTM0_SYNC            (KINETIS_FTM0_BASE+KINETIS_FTM_SYNC_OFFSET)
#define KINETIS_FTM0_OUTINIT         (KINETIS_FTM0_BASE+KINETIS_FTM_OUTINIT_OFFSET)
#define KINETIS_FTM0_OUTMASK         (KINETIS_FTM0_BASE+KINETIS_FTM_OUTMASK_OFFSET)
#define KINETIS_FTM0_COMBINE         (KINETIS_FTM0_BASE+KINETIS_FTM_COMBINE_OFFSET)
#define KINETIS_FTM0_DEADTIME        (KINETIS_FTM0_BASE+KINETIS_FTM_DEADTIME_OFFSET)
#define KINETIS_FTM0_EXTTRIG         (KINETIS_FTM0_BASE+KINETIS_FTM_EXTTRIG_OFFSET)
#define KINETIS_FTM0_POL             (KINETIS_FTM0_BASE+KINETIS_FTM_POL_OFFSET)
#define KINETIS_FTM0_FMS             (KINETIS_FTM0_BASE+KINETIS_FTM_FMS_OFFSET)
#define KINETIS_FTM0_FILTER          (KINETIS_FTM0_BASE+KINETIS_FTM_FILTER_OFFSET)
#define KINETIS_FTM0_FLTCTRL         (KINETIS_FTM0_BASE+KINETIS_FTM_FLTCTRL_OFFSET)
#define KINETIS_FTM0_QDCTRL          (KINETIS_FTM0_BASE+KINETIS_FTM_QDCTRL_OFFSET)
#define KINETIS_FTM0_CONF            (KINETIS_FTM0_BASE+KINETIS_FTM_CONF_OFFSET)
#define KINETIS_FTM0_FLTPOL          (KINETIS_FTM0_BASE+KINETIS_FTM_FLTPOL_OFFSET)
#define KINETIS_FTM0_SYNCONF         (KINETIS_FTM0_BASE+KINETIS_FTM_SYNCONF_OFFSET)
#define KINETIS_FTM0_INVCTRL         (KINETIS_FTM0_BASE+KINETIS_FTM_INVCTRL_OFFSET)
#define KINETIS_FTM0_SWOCTRL         (KINETIS_FTM0_BASE+KINETIS_FTM_SWOCTRL_OFFSET)
#define KINETIS_FTM0_PWMLOAD         (KINETIS_FTM0_BASE+KINETIS_FTM_PWMLOAD_OFFSET)

#define KINETIS_FTM1_SC              (KINETIS_FTM1_BASE+KINETIS_FTM_SC_OFFSET)
#define KINETIS_FTM1_CNT             (KINETIS_FTM1_BASE+KINETIS_FTM_CNT_OFFSET)
#define KINETIS_FTM1_MOD             (KINETIS_FTM1_BASE+KINETIS_FTM_MOD_OFFSET)

#define KINETIS_FTM1_CSC(n)          (KINETIS_FTM1_BASE+KINETIS_FTM_CSC_OFFSET(n))
#define KINETIS_FTM1_CV(n)           (KINETIS_FTM1_BASE+KINETIS_FTM_CV_OFFSET(n))
#define KINETIS_FTM1_C0SC            (KINETIS_FTM1_BASE+KINETIS_FTM_C0SC_OFFSET)
#define KINETIS_FTM1_C0V             (KINETIS_FTM1_BASE+KINETIS_FTM_C0V_OFFSET)
#define KINETIS_FTM1_C1SC            (KINETIS_FTM1_BASE+KINETIS_FTM_C1SC_OFFSET)
#define KINETIS_FTM1_C1V             (KINETIS_FTM1_BASE+KINETIS_FTM_C1V_OFFSET)
#define KINETIS_FTM1_C2SC            (KINETIS_FTM1_BASE+KINETIS_FTM_C2SC_OFFSET)
#define KINETIS_FTM1_C2V             (KINETIS_FTM1_BASE+KINETIS_FTM_C2V_OFFSET)
#define KINETIS_FTM1_C3SC            (KINETIS_FTM1_BASE+KINETIS_FTM_C3SC_OFFSET)
#define KINETIS_FTM1_C3V             (KINETIS_FTM1_BASE+KINETIS_FTM_C3V_OFFSET)
#define KINETIS_FTM1_C4SC            (KINETIS_FTM1_BASE+KINETIS_FTM_C4SC_OFFSET)
#define KINETIS_FTM1_C4V             (KINETIS_FTM1_BASE+KINETIS_FTM_C4V_OFFSET)
#define KINETIS_FTM1_C5SC            (KINETIS_FTM1_BASE+KINETIS_FTM_C5SC_OFFSET)
#define KINETIS_FTM1_C5V             (KINETIS_FTM1_BASE+KINETIS_FTM_C5V_OFFSET)
#define KINETIS_FTM1_C6SC            (KINETIS_FTM1_BASE+KINETIS_FTM_C6SC_OFFSET)
#define KINETIS_FTM1_C6V             (KINETIS_FTM1_BASE+KINETIS_FTM_C6V_OFFSET)
#define KINETIS_FTM1_C7SC            (KINETIS_FTM1_BASE+KINETIS_FTM_C7SC_OFFSET)
#define KINETIS_FTM1_C7V             (KINETIS_FTM1_BASE+KINETIS_FTM_C7V_OFFSET)

#define KINETIS_FTM1_CNTIN           (KINETIS_FTM1_BASE+KINETIS_FTM_CNTIN_OFFSET)
#define KINETIS_FTM1_STATUS          (KINETIS_FTM1_BASE+KINETIS_FTM_STATUS_OFFSET)
#define KINETIS_FTM1_MODE            (KINETIS_FTM1_BASE+KINETIS_FTM_MODE_OFFSET)
#define KINETIS_FTM1_SYNC            (KINETIS_FTM1_BASE+KINETIS_FTM_SYNC_OFFSET)
#define KINETIS_FTM1_OUTINIT         (KINETIS_FTM1_BASE+KINETIS_FTM_OUTINIT_OFFSET)
#define KINETIS_FTM1_OUTMASK         (KINETIS_FTM1_BASE+KINETIS_FTM_OUTMASK_OFFSET)
#define KINETIS_FTM1_COMBINE         (KINETIS_FTM1_BASE+KINETIS_FTM_COMBINE_OFFSET)
#define KINETIS_FTM1_DEADTIME        (KINETIS_FTM1_BASE+KINETIS_FTM_DEADTIME_OFFSET)
#define KINETIS_FTM1_EXTTRIG         (KINETIS_FTM1_BASE+KINETIS_FTM_EXTTRIG_OFFSET)
#define KINETIS_FTM1_POL             (KINETIS_FTM1_BASE+KINETIS_FTM_POL_OFFSET)
#define KINETIS_FTM1_FMS             (KINETIS_FTM1_BASE+KINETIS_FTM_FMS_OFFSET)
#define KINETIS_FTM1_FILTER          (KINETIS_FTM1_BASE+KINETIS_FTM_FILTER_OFFSET)
#define KINETIS_FTM1_FLTCTRL         (KINETIS_FTM1_BASE+KINETIS_FTM_FLTCTRL_OFFSET)
#define KINETIS_FTM1_QDCTRL          (KINETIS_FTM1_BASE+KINETIS_FTM_QDCTRL_OFFSET)
#define KINETIS_FTM1_CONF            (KINETIS_FTM1_BASE+KINETIS_FTM_CONF_OFFSET)
#define KINETIS_FTM1_FLTPOL          (KINETIS_FTM1_BASE+KINETIS_FTM_FLTPOL_OFFSET)
#define KINETIS_FTM1_SYNCONF         (KINETIS_FTM1_BASE+KINETIS_FTM_SYNCONF_OFFSET)
#define KINETIS_FTM1_INVCTRL         (KINETIS_FTM1_BASE+KINETIS_FTM_INVCTRL_OFFSET)
#define KINETIS_FTM1_SWOCTRL         (KINETIS_FTM1_BASE+KINETIS_FTM_SWOCTRL_OFFSET)
#define KINETIS_FTM1_PWMLOAD         (KINETIS_FTM1_BASE+KINETIS_FTM_PWMLOAD_OFFSET)

#define KINETIS_FTM2_SC              (KINETIS_FTM2_BASE+KINETIS_FTM_SC_OFFSET)
#define KINETIS_FTM2_CNT             (KINETIS_FTM2_BASE+KINETIS_FTM_CNT_OFFSET)
#define KINETIS_FTM2_MOD             (KINETIS_FTM2_BASE+KINETIS_FTM_MOD_OFFSET)

#define KINETIS_FTM2_CSC(n)          (KINETIS_FTM2_BASE+KINETIS_FTM_CSC_OFFSET(n))
#define KINETIS_FTM2_CV(n)           (KINETIS_FTM2_BASE+KINETIS_FTM_CV_OFFSET(n))
#define KINETIS_FTM2_C0SC            (KINETIS_FTM2_BASE+KINETIS_FTM_C0SC_OFFSET)
#define KINETIS_FTM2_C0V             (KINETIS_FTM2_BASE+KINETIS_FTM_C0V_OFFSET)
#define KINETIS_FTM2_C1SC            (KINETIS_FTM2_BASE+KINETIS_FTM_C1SC_OFFSET)
#define KINETIS_FTM2_C1V             (KINETIS_FTM2_BASE+KINETIS_FTM_C1V_OFFSET)
#define KINETIS_FTM2_C2SC            (KINETIS_FTM2_BASE+KINETIS_FTM_C2SC_OFFSET)
#define KINETIS_FTM2_C2V             (KINETIS_FTM2_BASE+KINETIS_FTM_C2V_OFFSET)
#define KINETIS_FTM2_C3SC            (KINETIS_FTM2_BASE+KINETIS_FTM_C3SC_OFFSET)
#define KINETIS_FTM2_C3V             (KINETIS_FTM2_BASE+KINETIS_FTM_C3V_OFFSET)
#define KINETIS_FTM2_C4SC            (KINETIS_FTM2_BASE+KINETIS_FTM_C4SC_OFFSET)
#define KINETIS_FTM2_C4V             (KINETIS_FTM2_BASE+KINETIS_FTM_C4V_OFFSET)
#define KINETIS_FTM2_C5SC            (KINETIS_FTM2_BASE+KINETIS_FTM_C5SC_OFFSET)
#define KINETIS_FTM2_C5V             (KINETIS_FTM2_BASE+KINETIS_FTM_C5V_OFFSET)
#define KINETIS_FTM2_C6SC            (KINETIS_FTM2_BASE+KINETIS_FTM_C6SC_OFFSET)
#define KINETIS_FTM2_C6V             (KINETIS_FTM2_BASE+KINETIS_FTM_C6V_OFFSET)
#define KINETIS_FTM2_C7SC            (KINETIS_FTM2_BASE+KINETIS_FTM_C7SC_OFFSET)
#define KINETIS_FTM2_C7V             (KINETIS_FTM2_BASE+KINETIS_FTM_C7V_OFFSET)

#define KINETIS_FTM2_CNTIN           (KINETIS_FTM2_BASE+KINETIS_FTM_CNTIN_OFFSET)
#define KINETIS_FTM2_STATUS          (KINETIS_FTM2_BASE+KINETIS_FTM_STATUS_OFFSET)
#define KINETIS_FTM2_MODE            (KINETIS_FTM2_BASE+KINETIS_FTM_MODE_OFFSET)
#define KINETIS_FTM2_SYNC            (KINETIS_FTM2_BASE+KINETIS_FTM_SYNC_OFFSET)
#define KINETIS_FTM2_OUTINIT         (KINETIS_FTM2_BASE+KINETIS_FTM_OUTINIT_OFFSET)
#define KINETIS_FTM2_OUTMASK         (KINETIS_FTM2_BASE+KINETIS_FTM_OUTMASK_OFFSET)
#define KINETIS_FTM2_COMBINE         (KINETIS_FTM2_BASE+KINETIS_FTM_COMBINE_OFFSET)
#define KINETIS_FTM2_DEADTIME        (KINETIS_FTM2_BASE+KINETIS_FTM_DEADTIME_OFFSET)
#define KINETIS_FTM2_EXTTRIG         (KINETIS_FTM2_BASE+KINETIS_FTM_EXTTRIG_OFFSET)
#define KINETIS_FTM2_POL             (KINETIS_FTM2_BASE+KINETIS_FTM_POL_OFFSET)
#define KINETIS_FTM2_FMS             (KINETIS_FTM2_BASE+KINETIS_FTM_FMS_OFFSET)
#define KINETIS_FTM2_FILTER          (KINETIS_FTM2_BASE+KINETIS_FTM_FILTER_OFFSET)
#define KINETIS_FTM2_FLTCTRL         (KINETIS_FTM2_BASE+KINETIS_FTM_FLTCTRL_OFFSET)
#define KINETIS_FTM2_QDCTRL          (KINETIS_FTM2_BASE+KINETIS_FTM_QDCTRL_OFFSET)
#define KINETIS_FTM2_CONF            (KINETIS_FTM2_BASE+KINETIS_FTM_CONF_OFFSET)
#define KINETIS_FTM2_FLTPOL          (KINETIS_FTM2_BASE+KINETIS_FTM_FLTPOL_OFFSET)
#define KINETIS_FTM2_SYNCONF         (KINETIS_FTM2_BASE+KINETIS_FTM_SYNCONF_OFFSET)
#define KINETIS_FTM2_INVCTRL         (KINETIS_FTM2_BASE+KINETIS_FTM_INVCTRL_OFFSET)
#define KINETIS_FTM2_SWOCTRL         (KINETIS_FTM2_BASE+KINETIS_FTM_SWOCTRL_OFFSET)
#define KINETIS_FTM2_PWMLOAD         (KINETIS_FTM2_BASE+KINETIS_FTM_PWMLOAD_OFFSET)

/* Register Bit Definitions *****************************************************************/

/* Status and Control */

#define FTM_SC_PS_SHIFT              (0)       /* Bits 0-2: Prescale Factor Selection */
#define FTM_SC_PS_MASK               (7 << FTM_SC_PS_SHIFT)
#  define FTM_SC_PS_1                (0 << FTM_SC_PS_SHIFT)
#  define FTM_SC_PS_2                (1 << FTM_SC_PS_SHIFT)
#  define FTM_SC_PS_4                (2 << FTM_SC_PS_SHIFT)
#  define FTM_SC_PS_8                (3 << FTM_SC_PS_SHIFT)
#  define FTM_SC_PS_16               (4 << FTM_SC_PS_SHIFT)
#  define FTM_SC_PS_32               (5 << FTM_SC_PS_SHIFT)
#  define FTM_SC_PS_64               (6 << FTM_SC_PS_SHIFT)
#  define FTM_SC_PS_128              (7 << FTM_SC_PS_SHIFT)
#define FTM_SC_CLKS_SHIFT            (3)       /* Bits 3-4: Clock Source Selection */
#define FTM_SC_CLKS_MASK             (3 << FTM_SC_CLKS_SHIFT)
#  define FTM_SC_CLKS_NONE           (0 << FTM_SC_CLKS_SHIFT) /* No clock selected */
#  define FTM_SC_CLKS_SYSCLK         (1 << FTM_SC_CLKS_SHIFT) /* System clock */
#  define FTM_SC_CLKS_FIXED          (2 << FTM_SC_CLKS_SHIFT) /* Fixed frequency clock */
#  define FTM_SC_CLKS_EXTCLK         (3 << FTM_SC_CLKS_SHIFT) /* External clock */
#define FTM_SC_CPWMS                 (1 << 5)  /* Bit 5:  Center-aligned PWM Select */
#define FTM_SC_TOIE                  (1 << 6)  /* Bit 6:  Timer Overflow Interrupt Enable */
#define FTM_SC_TOF                   (1 << 7)  /* Bit 7:  Timer Overflow Flag */
                                               /* Bits 8-31: Reserved */
/* Counter */

#define FTM_CNT_SHIFT                (0)       /* Bits 0-15: Counter value */
#define FTM_CNT_MASK                 (0xffff << FTM_CNT_SHIFT)
                                               /* Bits 16-31: Reserved */

/* Modulo */

#define FTM_MOD_SHIFT                (0)       /* Bits 0-15: Modulo value */
#define FTM_MOD_MASK                 (0xffff << FTM_MOD_SHIFT)
                                               /* Bits 16-31: Reserved */

/* Channel (n) Status and Control */

#define FTM_CSC_DMA                  (1 << 0)  /* Bit 0:  DMA Enable */
                                               /* Bit 1: Reserved */
#define FTM_CSC_ELSA                 (1 << 2)  /* Bit 2:  Edge or Level Select */
#define FTM_CSC_ELSB                 (1 << 3)  /* Bit 3:  Edge or Level Select */
#define FTM_CSC_MSA                  (1 << 4)  /* Bit 4:  Channel Mode Select */
#define FTM_CSC_MSB                  (1 << 5)  /* Bit 5:  Channel Mode Select */
#define FTM_CSC_CHIE                 (1 << 6)  /* Bit 6:  Channel Interrupt Enable */
#define FTM_CSC_CHF                  (1 << 7)  /* Bit 7:  Channel Flag */
                                               /* Bits 8-31: Reserved */
/* Channel (n) Value */

#define FTM_CV_SHIFT                 (0)       /* Bits 0-15: Channel Value */
#define FTM_CV_MASK                  (0xffff << FTM_CV_SHIFT)
                                               /* Bits 16-31: Reserved */
/* Counter Initial Value */

#define FTM_CNTIN_SHIFT              (0)       /* Bits 0-15: Initial Value of the FTM Counter */
#define FTM_CNTIN_MASK               (0xffff << FTM_CNTIN_SHIFT)
                                               /* Bits 16-31: Reserved */
/* Capture and Compare Status */

#define FTM_STATUS(n)                (1 << (n)) /* Channel (n) Flag, n=0..7 */
                                               /* Bits 8-31: Reserved */

/* Features Mode Selection */

#define FTM_MODE_FTMEN               (1 << 0)  /* Bit 0:  FTM Enable */
#define FTM_MODE_INIT                (1 << 1)  /* Bit 1:  Initialize the Channels Output */
#define FTM_MODE_WPDIS               (1 << 2)  /* Bit 2:  Write Protection Disable */
#define FTM_MODE_PWMSYNC             (1 << 3)  /* Bit 3:  PWM Synchronization Mode */
#define FTM_MODE_CAPTEST             (1 << 4)  /* Bit 4:  Capture Test Mode Enable */
#define FTM_MODE_FAULTM_SHIFT        (5)       /* Bits 5-6: Fault Control Mode */
#define FTM_MODE_FAULTM_MASK         (3 << FTM_MODE_FAULTM_SHIFT)
#  define FTM_MODE_FAULTM_DISABLED   (0 << FTM_MODE_FAULTM_SHIFT) /* Disabled */
#  define FTM_MODE_FAULTM_EVEN       (1 << FTM_MODE_FAULTM_SHIFT) /* Enable even channels, manual fault clearing */
#  define FTM_MODE_FAULTM_MANUAL     (2 << FTM_MODE_FAULTM_SHIFT) /* Enable all channels, manual fault clearing */
#  define FTM_MODE_FAULTM_AUTO       (3 << FTM_MODE_FAULTM_SHIFT) /* Enable all channels, automatic fault clearing */
#define FTM_MODE_FAULTIE             (1 << 7)  /* Bit 7:  Fault Interrupt Enable */
                                               /* Bits 8-31: Reserved */
/* Synchronization */

#define FTM_SYNC_CNTMIN              (1 << 0)  /* Bit 0:  Minimum loading point enable */
#define FTM_SYNC_CNTMAX              (1 << 1)  /* Bit 1:  Maximum loading point enable */
#define FTM_SYNC_REINIT              (1 << 2)  /* Bit 2:  FTM Counter Reinitialization by Synchron */
#define FTM_SYNC_SYNCHOM             (1 << 3)  /* Bit 3:  Output Mask Synchronization */
#define FTM_SYNC_TRIG0               (1 << 4)  /* Bit 4:  PWM Synchronization Hardware Trigger 0 */
#define FTM_SYNC_TRIG1               (1 << 5)  /* Bit 5:  PWM Synchronization Hardware Trigger 1 */
#define FTM_SYNC_TRIG2               (1 << 6)  /* Bit 6:  PWM Synchronization Hardware Trigger 2 */
#define FTM_SYNC_SWSYNC              (1 << 7)  /* Bit 7:  PWM Synchronization Software Trigger */
                                               /* Bits 8-31: Reserved */
/* Initial State for Channels Output */

#define FTM_OUTINIT(n)               (1 << (n)) /* Channel (n) Output Initialization Value, n=0..7 */
                                               /* Bits 8-31: Reserved */
/* Output Mask */

#define FTM_OUTMASK(n)               (1 << (n)) /* Channel (n) Output Mask, n=0..7 */
                                               /* Bits 8-31: Reserved */
/* Function for Linked Channels */

#define FTM_COMBINE_COMBINE0         (1 << 0)  /* Bit 0:  Combine Channels for n = 0 */
#define FTM_COMBINE_COMP0            (1 << 1)  /* Bit 1:  Complement of Channel (n) for n = 0 */
#define FTM_COMBINE_DECAPEN0         (1 << 2)  /* Bit 2:  Dual Edge Capture Mode Enable for n = 0 */
#define FTM_COMBINE_DECAP0           (1 << 3)  /* Bit 3:  Dual Edge Capture Mode Captures for n = 0 */
#define FTM_COMBINE_DTEN0            (1 << 4)  /* Bit 4:  Deadtime Enable for n = 0 */
#define FTM_COMBINE_SYNCEN0          (1 << 5)  /* Bit 5:  Synchronization Enable for n = 0 */
#define FTM_COMBINE_FAULTEN0         (1 << 6)  /* Bit 6:  Fault Control Enable for n = 0 */
                                               /* Bit 7: Reserved */
#define FTM_COMBINE_COMBINE1         (1 << 8)  /* Bit 8:  Combine Channels for n = 2 */
#define FTM_COMBINE_COMP1            (1 << 9)  /* Bit 9:  Complement of Channel (n) for n = 2 */
#define FTM_COMBINE_DECAPEN1         (1 << 10) /* Bit 10: Dual Edge Capture Mode Enable for n = 2 */
#define FTM_COMBINE_DECAP1           (1 << 11) /* Bit 11: Dual Edge Capture Mode Captures for n = 2 */
#define FTM_COMBINE_DTEN1            (1 << 12) /* Bit 12: Deadtime Enable for n = 2 */
#define FTM_COMBINE_SYNCEN1          (1 << 13) /* Bit 13: Synchronization Enable for n = 2 */
#define FTM_COMBINE_FAULTEN1         (1 << 14) /* Bit 14: Fault Control Enable for n = 2 */
                                               /* Bit 15: Reserved */
#define FTM_COMBINE_COMBINE2         (1 << 16) /* Bit 16: Combine Channels for n = 4 */
#define FTM_COMBINE_COMP2            (1 << 17) /* Bit 17: Complement of Channel (n) for n = 4 */
#define FTM_COMBINE_DECAPEN2         (1 << 18) /* Bit 18: Dual Edge Capture Mode Enable for n = 4 */
#define FTM_COMBINE_DECAP2           (1 << 19) /* Bit 19: Dual Edge Capture Mode Captures for n = 4 */
#define FTM_COMBINE_DTEN2            (1 << 20) /* Bit 20: Deadtime Enable for n = 4 */
#define FTM_COMBINE_SYNCEN2          (1 << 21) /* Bit 21: Synchronization Enable for n = 4 */
#define FTM_COMBINE_FAULTEN2         (1 << 22) /* Bit 22: Fault Control Enable for n = 4 */
                                               /* Bit 23: Reserved */
#define FTM_COMBINE_COMBINE3         (1 << 24) /* Bit 24: Combine Channels for n = 6 */
#define FTM_COMBINE_COMP3            (1 << 25) /* Bit 25: Complement of Channel (n) for n = 6 */
#define FTM_COMBINE_DECAPEN3         (1 << 26) /* Bit 26: Dual Edge Capture Mode Enable for n = 6 */
#define FTM_COMBINE_DECAP3           (1 << 27) /* Bit 27: Dual Edge Capture Mode Captures for n = 6 */
#define FTM_COMBINE_DTEN3            (1 << 28) /* Bit 28: Deadtime Enable for n = 6 */
#define FTM_COMBINE_SYNCEN3          (1 << 29) /* Bit 29: Synchronization Enable for n = 6 */
#define FTM_COMBINE_FAULTEN3         (1 << 30) /* Bit 30: Fault Control Enable for n = 6 */
                                               /* Bit 31: Reserved */
/* Deadtime Insertion Control */

#define FTM_DEADTIME_DTVAL_SHIFT     (0)       /* Bits 0-5: Deadtime Value */
#define FTM_DEADTIME_DTVAL_MASK      (63 << FTM_DEADTIME_DTVAL_SHIFT)
#define FTM_DEADTIME_DTPS_SHIFT      (6)       /* Bits 6-7: Deadtime Prescaler Value */
#define FTM_DEADTIME_DTPS_MASK       (3 << FTM_DEADTIME_DTPS_SHIFT)
#  define FTM_DEADTIME_DTPS_DIV1     (0 << FTM_DEADTIME_DTPS_SHIFT)
#  define FTM_DEADTIME_DTPS_DIV4     (2 << FTM_DEADTIME_DTPS_SHIFT)
#  define FTM_DEADTIME_DTPS_DIV16    (3 << FTM_DEADTIME_DTPS_SHIFT)
                                               /* Bits 8-31: Reserved */
/* FTM External Trigger */

#define FTM_EXTTRIG_CH2TRIG          (1 << 0)  /* Bit 0:  Channel 2 Trigger Enable */
#define FTM_EXTTRIG_CH3TRIG          (1 << 1)  /* Bit 1:  Channel 3 Trigger Enable */
#define FTM_EXTTRIG_CH4TRIG          (1 << 2)  /* Bit 2:  Channel 4 Trigger Enable */
#define FTM_EXTTRIG_CH5TRIG          (1 << 3)  /* Bit 3:  Channel 5 Trigger Enable */
#define FTM_EXTTRIG_CH0TRIG          (1 << 4)  /* Bit 4:  Channel 0 Trigger Enable */
#define FTM_EXTTRIG_CH1TRIG          (1 << 5)  /* Bit 5:  Channel 1 Trigger Enable */
#define FTM_EXTTRIG_INITTRIGEN       (1 << 6)  /* Bit 6:  Initialization Trigger Enable */
#define FTM_EXTTRIG_TRIGF            (1 << 7)  /* Bit 7:  Channel Trigger Flag */
                                               /* Bits 8-31: Reserved */
/* Channels Polarity */

#define FTM_POL(n)                   (1 << (n)) /* Channel (n) Polarity, n=0..7 */
                                               /* Bits 8-31: Reserved */

/* Fault Mode Status */

#define FTM_FMS_FAULTF0              (1 << 0)  /* Bit 0:  Fault Detection Flag 0 */
#define FTM_FMS_FAULTF1              (1 << 1)  /* Bit 1:  Fault Detection Flag 1 */
#define FTM_FMS_FAULTF2              (1 << 2)  /* Bit 2:  Fault Detection Flag 2 */
#define FTM_FMS_FAULTF3              (1 << 3)  /* Bit 3:  Fault Detection Flag 3 */
                                               /* Bit 4: Reserved */
#define FTM_FMS_FAULTIN              (1 << 5)  /* Bit 5:  Fault Inputs */
#define FTM_FMS_WPEN                 (1 << 6)  /* Bit 6:  Write Protection Enable */
#define FTM_FMS_FAULTF               (1 << 7)  /* Bit 7:  Fault Detection Flag */
                                               /* Bits 8-31: Reserved */
/* Input Capture Filter Control */

#define FTM_FILTER_CH0FVAL_SHIFT     (0)       /* Bits 0-3: Channel 0 Input Filter */
#define FTM_FILTER_CH0FVAL_MASK      (15 << FTM_FILTER_CH0FVAL_SHIFT)
#define FTM_FILTER_CH1FVAL_SHIFT     (4)       /* Bits 4-7: Channel 1 Input Filter */
#define FTM_FILTER_CH1FVAL_MASK      (15 << FTM_FILTER_CH1FVAL_SHIFT)
#define FTM_FILTER_CH2FVAL_SHIFT     (8)       /* Bits 8-11: Channel 2 Input Filter */
#define FTM_FILTER_CH2FVAL_MASK      (15 << FTM_FILTER_CH2FVAL_SHIFT)
#define FTM_FILTER_CH3FVAL_SHIFT     (12)      /* Bits 12-15: Channel 3 Input Filter */
#define FTM_FILTER_CH3FVAL_MASK      (15 << FTM_FILTER_CH3FVAL_SHIFT)
                                               /* Bits 16-31: Reserved */
/* Fault Control */

#define FTM_FLTCTRL_FAULT0EN         (1 << 0)  /* Bit 0:  Fault Input 0 Enable */
#define FTM_FLTCTRL_FAULT1EN         (1 << 1)  /* Bit 1:  Fault Input 1 Enable */
#define FTM_FLTCTRL_FAULT2EN         (1 << 2)  /* Bit 2:  Fault Input 2 Enable */
#define FTM_FLTCTRL_FAULT3EN         (1 << 3)  /* Bit 3:  Fault Input 3 Enable */
#define FTM_FLTCTRL_FFLTR0EN         (1 << 4)  /* Bit 4:  Fault Input 0 Filter Enable */
#define FTM_FLTCTRL_FFLTR1EN         (1 << 5)  /* Bit 5:  Fault Input 1 Filter Enable */
#define FTM_FLTCTRL_FFLTR2EN         (1 << 6)  /* Bit 6:  Fault Input 2 Filter Enable */
#define FTM_FLTCTRL_FFLTR3EN         (1 << 7)  /* Bit 7:  Fault Input 3 Filter Enable */
#define FTM_FLTCTRL_FFVAL_SHIFT      (8)       /* Bits 8-11: Fault Input Filter */
#define FTM_FLTCTRL_FFVAL_MASK       (15 << FTM_FLTCTRL_FFVAL_SHIFT)
                                               /* Bits 12-31: Reserved */
/* Quadrature Decoder Control and Status */

#define FTM_QDCTRL_QUADEN            (1 << 0)  /* Bit 0:  Quadrature Decoder Mode Enable */
#define FTM_QDCTRL_TOFDIR            (1 << 1)  /* Bit 1:  Timer Overflow Direction in Quadrature Decoder Mode */
#define FTM_QDCTRL_QUADIR            (1 << 2)  /* Bit 2:  FTM Counter Direction in Quadrature Decoder Mode */
#define FTM_QDCTRL_QUADMODE          (1 << 3)  /* Bit 3:  Quadrature Decoder Mode */
#define FTM_QDCTRL_PHBPOL            (1 << 4)  /* Bit 4:  Phase B Input Polarity */
#define FTM_QDCTRL_PHAPOL            (1 << 5)  /* Bit 5:  Phase A Input Polarity */
#define FTM_QDCTRL_PHBFLTREN         (1 << 6)  /* Bit 6:  Phase B Input Filter Enable */
#define FTM_QDCTRL_PHAFLTREN         (1 << 7)  /* Bit 7:  Phase A Input Filter Enable */
                                               /* Bits 8-31: Reserved */
/* Configuration */

#define FTM_CONF_NUMTOF_SHIFT        (0)       /* Bits 0-4: TOF Frequency */
#define FTM_CONF_NUMTOF_MASK         (31 << FTM_CONF_NUMTOF_SHIFT)
                                               /* Bit 5: Reserved */
#define FTM_CONF_BDMMODE_SHIFT       (6)       /* Bits 6-7: BDM Mode */
#define FTM_CONF_BDMMODE_MASK        (3 << FTM_CONF_BDMMODE_SHIFT)
                                               /* Bit 8: Reserved */
#define FTM_CONF_GTBEEN              (1 << 9)  /* Bit 9:  Global time base enable */
#define FTM_CONF_GTBEOUT             (1 << 10) /* Bit 10: Global time base output */
                                               /* Bits 11-31: Reserved */
/* FTM Fault Input Polarity */

#define FTM_FLTPOL_FLT0POL           (1 << 0)  /* Bit 0:  Fault Input 0 Polarity */
#define FTM_FLTPOL_FLT1POL           (1 << 1)  /* Bit 1:  Fault Input 1 Polarity */
#define FTM_FLTPOL_FLT2POL           (1 << 2)  /* Bit 2:  Fault Input 2 Polarity */
#define FTM_FLTPOL_FLT3POL           (1 << 3)  /* Bit 3:  Fault Input 3 Polarity */
                                               /* Bits 4-31: Reserved */
/* Synchronization Configuration */

#define FTM_SYNCONF_HWTRIGMODE       (1 << 0)  /* Bit 0:  Hardware Trigger Mode */
                                               /* Bit 1:  Reserved */
#define FTM_SYNCONF_CNTINC           (1 << 2)  /* Bit 2:  CNTIN register synchronization */
                                               /* Bit 3:  Reserved */
#define FTM_SYNCONF_INVC             (1 << 4)  /* Bit 4:  INVCTRL register synchronization */
#define FTM_SYNCONF_SWOC             (1 << 5)  /* Bit 5:  SWOCTRL register synchronization */
                                               /* Bit 6:  Reserved */
#define FTM_SYNCONF_SYNCMODE         (1 << 7)  /* Bit 7:  Synchronization Mode */
#define FTM_SYNCONF_SWRSTCNT         (1 << 8)  /* Bit 8:  FTM counter synchronization (S/W) */
#define FTM_SYNCONF_SWWRBUF          (1 << 9)  /* Bit 9:  MOD, CNTIN, and CV registers synchronization (S/W) */
#define FTM_SYNCONF_SWOM             (1 << 10) /* Bit 10: Output mask synchronization (S/W) */
#define FTM_SYNCONF_SWINVC           (1 << 11) /* Bit 11: Inverting control synchronization (S/W) */
#define FTM_SYNCONF_SWSOC            (1 << 12) /* Bit 12: Software output control synchronization (S/W) */
                                               /* Bits 13-15: Reserved */
#define FTM_SYNCONF_HWRSTCNT         (1 << 16) /* Bit 16: FTM counter synchronization (H/W) */
#define FTM_SYNCONF_HWWRBUF          (1 << 17) /* Bit 17: MOD, CNTIN, and CV registers synchronization (H/W) */
#define FTM_SYNCONF_HWOM             (1 << 18) /* Bit 18: Output mask synchronization (H/W) */
#define FTM_SYNCONF_HWINVC           (1 << 19) /* Bit 19: Inverting control synchronization (H/W) */
#define FTM_SYNCONF_HWSOC            (1 << 20) /* Bit 20: Software output control synchronization (H/W) */
                                               /* Bits 21-31: Reserved */
/* FTM Inverting Control */

#define FTM_INVCTRL_INV0EN           (1 << 0)  /* Bit 0:  Pair Channels 0 Inverting Enable */
#define FTM_INVCTRL_INV1EN           (1 << 1)  /* Bit 1:  Pair Channels 1 Inverting Enable */
#define FTM_INVCTRL_INV2EN           (1 << 2)  /* Bit 2:  Pair Channels 2 Inverting Enable */
#define FTM_INVCTRL_INV3EN           (1 << 3)  /* Bit 3:  Pair Channels 3 Inverting Enable */
                                               /* Bits 4-31: Reserved */
/* FTM Software Output Control */

#define FTM_SWOCTRL_CH7OC(n)         (1 << (n)) /* Bits 0-7: Channel (n) Software Output Control Enable */
#define FTM_SWOCTRL_CH0OC            (1 << 0)  /* Bit 0:  Channel 0 Software Output Control Enable */
#define FTM_SWOCTRL_CH1OC            (1 << 1)  /* Bit 1:  Channel 1 Software Output Control Enable */
#define FTM_SWOCTRL_CH2OC            (1 << 2)  /* Bit 2:  Channel 2 Software Output Control Enable */
#define FTM_SWOCTRL_CH3OC            (1 << 3)  /* Bit 3:  Channel 3 Software Output Control Enable */
#define FTM_SWOCTRL_CH4OC            (1 << 4)  /* Bit 4:  Channel 4 Software Output Control Enable */
#define FTM_SWOCTRL_CH5OC            (1 << 5)  /* Bit 5:  Channel 5 Software Output Control Enable */
#define FTM_SWOCTRL_CH6OC            (1 << 6)  /* Bit 6:  Channel 6 Software Output Control Enable */
#define FTM_SWOCTRL_CH7OC            (1 << 7)  /* Bit 7:  Channel 7 Software Output Control Enable */
#define FTM_SWOCTRL_CHOCV(n)         (1 << ((n)+8)) /* Bits 8-15: Channel (n) Software Output Control Value */
#define FTM_SWOCTRL_CH0OCV           (1 << 8)  /* Bit 8:  Channel 0 Software Output Control Value */
#define FTM_SWOCTRL_CH1OCV           (1 << 9)  /* Bit 9:  Channel 1 Software Output Control Value */
#define FTM_SWOCTRL_CH2OCV           (1 << 10) /* Bit 10: Channel 2 Software Output Control Value */
#define FTM_SWOCTRL_CH3OCV           (1 << 11) /* Bit 11: Channel 3 Software Output Control Value */
#define FTM_SWOCTRL_CH4OCV           (1 << 12) /* Bit 12: Channel 4 Software Output Control Value */
#define FTM_SWOCTRL_CH5OCV           (1 << 13) /* Bit 13: Channel 5 Software Output Control Value */
#define FTM_SWOCTRL_CH6OCV           (1 << 14) /* Bit 14: Channel 6 Software Output Control Value */
#define FTM_SWOCTRL_CH7OCV           (1 << 15) /* Bit 15: Channel 7 Software Output Control Value */
                                               /* Bits 16-31: Reserved */
/* FTM PWM Load */

#define FTM_PWMLOAD_CH7SEL(n)        (1 << (n)) /* Bits 0-7: Channel (n) Select */
#define FTM_PWMLOAD_CH0SEL           (1 << 0)  /* Bit 0:  Channel 0 Select */
#define FTM_PWMLOAD_CH1SEL           (1 << 1)  /* Bit 1:  Channel 1 Select */
#define FTM_PWMLOAD_CH2SEL           (1 << 2)  /* Bit 2:  Channel 2 Select */
#define FTM_PWMLOAD_CH3SEL           (1 << 3)  /* Bit 3:  Channel 3 Select */
#define FTM_PWMLOAD_CH4SEL           (1 << 4)  /* Bit 4:  Channel 4 Select */
#define FTM_PWMLOAD_CH5SEL           (1 << 5)  /* Bit 5:  Channel 5 Select */
#define FTM_PWMLOAD_CH6SEL           (1 << 6)  /* Bit 6:  Channel 6 Select */
#define FTM_PWMLOAD_CH7SEL           (1 << 7)  /* Bit 7:  Channel 7 Select */
                                               /* Bit 8: Reserved */
#define FTM_PWMLOAD_LDOK             (1 << 9)  /* Bit 9: Load Enable */
                                               /* Bits 10-31: Reserved */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_FTM_H */
