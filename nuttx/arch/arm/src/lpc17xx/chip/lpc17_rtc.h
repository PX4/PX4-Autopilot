/************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc17_rtc.h
 *
 *   Copyright (C) 2010, 2012-2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_RTC_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_RTC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* Miscellaneous registers */

#define LPC17_RTC_ILR_OFFSET    0x0000 /* Interrupt Location Register */
#define LPC17_RTC_CCR_OFFSET    0x0008 /* Clock Control Register */
#define LPC17_RTC_CIIR_OFFSET   0x000c /* Counter Increment Interrupt Register */
#define LPC17_RTC_AMR_OFFSET    0x0010 /* Alarm Mask Register */
#define LPC17_RTC_AUXEN_OFFSET  0x0058 /* RTC Auxiliary Enable register */
#define LPC17_RTC_AUX_OFFSET    0x005c /* RTC Auxiliary control register */

/* Consolidated time registers */

#define LPC17_RTC_CTIME0_OFFSET 0x0014 /* Consolidated Time Register 0 */
#define LPC17_RTC_CTIME1_OFFSET 0x0018 /* Consolidated Time Register 1 */
#define LPC17_RTC_CTIME2_OFFSET 0x001c /* Consolidated Time Register 2 */

/* Time counter registers */

#define LPC17_RTC_SEC_OFFSET    0x0020 /* Seconds Counter */
#define LPC17_RTC_MIN_OFFSET    0x0024 /* Minutes Register */
#define LPC17_RTC_HOUR_OFFSET   0x0028 /* Hours Register */
#define LPC17_RTC_DOM_OFFSET    0x002c /* Day of Month Register */
#define LPC17_RTC_DOW_OFFSET    0x0030 /* Day of Week Register */
#define LPC17_RTC_DOY_OFFSET    0x0034 /* Day of Year Register */
#define LPC17_RTC_MONTH_OFFSET  0x0038 /* Months Register */
#define LPC17_RTC_YEAR_OFFSET   0x003c /* Years Register */
#define LPC17_RTC_CALIB_OFFSET  0x0040 /* Calibration Value Register */

/* General purpose registers */

#define LPC17_RTC_GPREG0_OFFSET 0x0044 /* General Purpose Register 0 */
#define LPC17_RTC_GPREG1_OFFSET 0x0048 /* General Purpose Register 1 */
#define LPC17_RTC_GPREG2_OFFSET 0x004c /* General Purpose Register 2 */
#define LPC17_RTC_GPREG3_OFFSET 0x0050 /* General Purpose Register 3 */
#define LPC17_RTC_GPREG4_OFFSET 0x0054 /* General Purpose Register 4 */

/* Alarm register group */

#define LPC17_RTC_ALSEC_OFFSET  0x0060 /* Alarm value for Seconds */
#define LPC17_RTC_ALMIN_OFFSET  0x0064 /* Alarm value for Minutes */
#define LPC17_RTC_ALHOUR_OFFSET 0x0068 /* Alarm value for Hours */
#define LPC17_RTC_ALDOM_OFFSET  0x006c /* Alarm value for Day of Month */
#define LPC17_RTC_ALDOW_OFFSET  0x0070 /* Alarm value for Day of Week */
#define LPC17_RTC_ALDOY_OFFSET  0x0074 /* Alarm value for Day of Year */
#define LPC17_RTC_ALMON_OFFSET  0x0078 /* Alarm value for Months  */
#define LPC17_RTC_ALYEAR_OFFSET 0x007c /* Alarm value for Year */

/* Register addresses ***************************************************************/
/* Miscellaneous registers */

#define LPC17_RTC_ILR           (LPC17_RTC_BASE+LPC17_RTC_ILR_OFFSET)
#define LPC17_RTC_CCR           (LPC17_RTC_BASE+LPC17_RTC_CCR_OFFSET)
#define LPC17_RTC_CIIR          (LPC17_RTC_BASE+LPC17_RTC_CIIR_OFFSET)
#define LPC17_RTC_AMR           (LPC17_RTC_BASE+LPC17_RTC_AMR_OFFSET)
#define LPC17_RTC_AUXEN         (LPC17_RTC_BASE+LPC17_RTC_AUXEN_OFFSET)
#define LPC17_RTC_AUX           (LPC17_RTC_BASE+LPC17_RTC_AUX_OFFSET)

/* Consolidated time registers */

#define LPC17_RTC_CTIME0        (LPC17_RTC_BASE+LPC17_RTC_CTIME0_OFFSET)
#define LPC17_RTC_CTIME1        (LPC17_RTC_BASE+LPC17_RTC_CTIME1_OFFSET)
#define LPC17_RTC_CTIME2        (LPC17_RTC_BASE+LPC17_RTC_CTIME2_OFFSET)

/* Time counter registers */

#define LPC17_RTC_SEC           (LPC17_RTC_BASE+LPC17_RTC_SEC_OFFSET)
#define LPC17_RTC_MIN           (LPC17_RTC_BASE+LPC17_RTC_MIN_OFFSET)
#define LPC17_RTC_HOUR          (LPC17_RTC_BASE+LPC17_RTC_HOUR_OFFSET)
#define LPC17_RTC_DOM           (LPC17_RTC_BASE+LPC17_RTC_DOM_OFFSET)
#define LPC17_RTC_DOW           (LPC17_RTC_BASE+LPC17_RTC_DOW_OFFSET)
#define LPC17_RTC_DOY           (LPC17_RTC_BASE+LPC17_RTC_DOY_OFFSET)
#define LPC17_RTC_MONTH         (LPC17_RTC_BASE+LPC17_RTC_MONTH_OFFSET)
#define LPC17_RTC_YEAR          (LPC17_RTC_BASE+LPC17_RTC_YEAR_OFFSET)
#define LPC17_RTC_CALIB         (LPC17_RTC_BASE+LPC17_RTC_CALIB_OFFSET)

/* General purpose registers */

#define LPC17_RTC_GPREG0        (LPC17_RTC_BASE+LPC17_RTC_GPREG0_OFFSET)
#define LPC17_RTC_GPREG1        (LPC17_RTC_BASE+LPC17_RTC_GPREG1_OFFSET)
#define LPC17_RTC_GPREG2        (LPC17_RTC_BASE+LPC17_RTC_GPREG2_OFFSET)
#define LPC17_RTC_GPREG3        (LPC17_RTC_BASE+LPC17_RTC_GPREG3_OFFSET)
#define LPC17_RTC_GPREG4        (LPC17_RTC_BASE+LPC17_RTC_GPREG4_OFFSET)

/* Alarm register group */

#define LPC17_RTC_ALSEC         (LPC17_RTC_BASE+LPC17_RTC_ALSEC_OFFSET)
#define LPC17_RTC_ALMIN         (LPC17_RTC_BASE+LPC17_RTC_ALMIN_OFFSET)
#define LPC17_RTC_ALHOUR        (LPC17_RTC_BASE+LPC17_RTC_ALHOUR_OFFSET)
#define LPC17_RTC_ALDOM         (LPC17_RTC_BASE+LPC17_RTC_ALDOM_OFFSET)
#define LPC17_RTC_ALDOW         (LPC17_RTC_BASE+LPC17_RTC_ALDOW_OFFSET)
#define LPC17_RTC_ALDOY         (LPC17_RTC_BASE+LPC17_RTC_ALDOY_OFFSET)
#define LPC17_RTC_ALMON         (LPC17_RTC_BASE+LPC17_RTC_ALMON_OFFSET)
#define LPC17_RTC_ALYEAR        (LPC17_RTC_BASE+LPC17_RTC_ALYEAR_OFFSET)

/* Register bit definitions *********************************************************/
/* The following registers hold 32-bit values and have no bit fields to be defined:
 *
 *   General Purpose Register 0
 *   General Purpose Register 1
 *   General Purpose Register 2
 *   General Purpose Register 3
 *   General Purpose Register 4
 */

/* Miscellaneous registers */
/* Interrupt Location Register */

#define RTC_ILR_RTCCIF          (1 << 0)  /* Bit 0:  Counter Increment Interrupt */
#define RTC_ILR_RTCALF          (1 << 1)  /* Bit 1:  Alarm interrupt */
                                          /* Bits 2-31: Reserved */
/* Clock Control Register */

#define RTC_CCR_CLKEN           (1 << 0)  /* Bit 0:  Clock Enable */
#define RTC_CCR_CTCRST          (1 << 1)  /* Bit 1:  CTC Reset */
                                          /* Bits 2-3: Internal test mode controls */
#define RTC_CCR_CCALEN          (1 << 4)  /* Bit 4:  Calibration counter enable */
                                          /* Bits 5-31: Reserved */
/* Counter Increment Interrupt Register */

#define RTC_CIIR_IMSEC          (1 << 0)  /* Bit 0:  Second interrupt */
#define RTC_CIIR_IMMIN          (1 << 1)  /* Bit 1:  Minute interrupt */
#define RTC_CIIR_IMHOUR         (1 << 2)  /* Bit 2:  Hour interrupt */
#define RTC_CIIR_IMDOM          (1 << 3)  /* Bit 3:  Day of Month value interrupt */
#define RTC_CIIR_IMDOW          (1 << 4)  /* Bit 4:  Day of Week value interrupt */
#define RTC_CIIR_IMDOY          (1 << 5)  /* Bit 5:  Day of Year interrupt */
#define RTC_CIIR_IMMON          (1 << 6)  /* Bit 6:  Month interrupt */
#define RTC_CIIR_IMYEAR         (1 << 7)  /* Bit 7:  Yearinterrupt */
                                          /* Bits 8-31: Reserved */
/* Alarm Mask Register */

#define RTC_AMR_SEC             (1 << 0)  /* Bit 0:  Second not compared for alarm */
#define RTC_AMR_MIN             (1 << 1)  /* Bit 1:  Minutes not compared for alarm */
#define RTC_AMR_HOUR            (1 << 2)  /* Bit 2:  Hour not compared for alarm */
#define RTC_AMR_DOM             (1 << 3)  /* Bit 3:  Day of Monthnot compared for alarm */
#define RTC_AMR_DOW             (1 << 4)  /* Bit 4:  Day of Week not compared for alarm */
#define RTC_AMR_DOY             (1 << 5)  /* Bit 5:  Day of Year not compared for alarm */
#define RTC_AMR_MON             (1 << 6)  /* Bit 6:  Month not compared for alarm */
#define RTC_AMR_YEAR            (1 << 7)  /* Bit 7:  Year not compared for alarm */
                                          /* Bits 8-31: Reserved */
/* RTC Auxiliary Enable register */
                                          /* Bits 0-3: Reserved */
#define RTC_AUXEN_RTCOSCF       (1 << 4)  /* Bit 4:  RTC Oscillator Fail detect flag */
                                          /* Bits 5-31: Reserved */
/* RTC Auxiliary control register */
                                          /* Bits 0-3: Reserved */
#define RTC_AUX_OSCFEN          (1 << 4)  /* Bit 4:  Oscillator Fail Detect interrupt enable */
                                          /* Bits 5-31: Reserved */
/* Consolidated time registers */
/* Consolidated Time Register 0 */

#define RTC_CTIME0_SEC_SHIFT    (0)       /* Bits 0-5: Seconds */
#define RTC_CTIME0_SEC_MASK     (63 << RTC_CTIME0_SEC_SHIFT)
                                          /* Bits 6-7: Reserved */
#define RTC_CTIME0_MIN_SHIFT    (8)       /* Bits 8-13: Minutes */
#define RTC_CTIME0_MIN_MASK     (63 << RTC_CTIME0_MIN_SHIFT)
                                          /* Bits 14-15: Reserved */
#define RTC_CTIME0_HOURS_SHIFT  (16)      /* Bits 16-20: Hours */
#define RTC_CTIME0_HOURS_MASK   (31 << RTC_CTIME0_HOURS_SHIFT)
                                          /* Bits 21-23: Reserved */
#define RTC_CTIME0_DOW_SHIFT    (24)      /* Bits 24-26: Day of Week */
#define RTC_CTIME0_DOW_MASK     (7 << RTC_CTIME0_DOW_SHIFT)
                                          /* Bits 27-31: Reserved */
/* Consolidated Time Register 1 */

#define RTC_CTIME1_DOM_SHIFT    (0)       /* Bits 0-4: Day of Month */
#define RTC_CTIME1_DOM_MASK     (31 << RTC_CTIME1_DOM_SHIFT)
                                          /* Bits 5-7: Reserved */
#define RTC_CTIME1_MON_SHIFT    (8)       /* Bits 8-11: Month */
#define RTC_CTIME1_MON_MASK     (15 << RTC_CTIME1_MON_SHIFT)
                                          /* Bits 12-15: Reserved */
#define RTC_CTIME1_YEAR_SHIFT   (16)      /* Bits 16-27: Year */
#define RTC_CTIME1_YEAR_MASK    (0x0fff << RTC_CTIME1_YEAR_SHIFT)
                                          /* Bits 28-31: Reserved */
/* Consolidated Time Register 2 */

#define RTC_CTIME2_DOY_SHIFT    (0)       /* Bits 0-11: Day of Year */
#define RTC_CTIME2_DOY_MASK     (0x0fff << RTC_CTIME2_DOY_SHIFT)
                                          /* Bits 12-31: Reserved */
/* Time counter registers */

#define RTC_SEC_MASK            (0x003f)
#define RTC_MIN_MASK            (0x003f)
#define RTC_HOUR_MASK           (0x001f)
#define RTC_DOM_MASK            (0x001f)
#define RTC_DOW_MASK            (0x0007)
#define RTC_DOY_MASK            (0x01ff)
#define RTC_MONTH_MASK          (0x000f)
#define RTC_YEAR_MASK           (0x0fff)

/* Calibration Value Register */

#define RTC_CALIB_CALVAL_SHIFT  (0)       /* Bits 0-16: calibration counter counts to this value */
#define RTC_CALIB_CALVAL_MASK   (0xffff << RTC_CALIB_CALVAL_SHIFT)
#define RTC_CALIB_CALDIR        (1 << 17) /* Bit 17: Calibration direction */
                                          /* Bits 18-31: Reserved */
/* Alarm register group */

#define RTC_ALSEC_MASK          (0x003f)
#define RTC_ALMIN_MASK          (0x003f)
#define RTC_ALHOUR_MASK         (0x001f)
#define RTC_ALDOM_MASK          (0x001f)
#define RTC_ALDOW_MASK          (0x0007)
#define RTC_ALDOY_MASK          (0x01ff)
#define RTC_ALMON_MASK          (0x000f)
#define RTC_ALYEAR_MASK         (0x0fff)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_RTC_H */
