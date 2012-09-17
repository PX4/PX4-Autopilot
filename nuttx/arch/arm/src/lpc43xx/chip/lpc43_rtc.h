/************************************************************************************
 * arch/arm/src/lpc43xx/lpc43_rtc.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_LPC43_RTC_H
#define __ARCH_ARM_SRC_LPC43XX_LPC43_RTC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* Miscellaneous registers */

#define LPC43_RTC_ILR_OFFSET    0x0000 /* Interrupt Location Register */
#define LPC43_RTC_CCR_OFFSET    0x0008 /* Clock Control Register */
#define LPC43_RTC_CIIR_OFFSET   0x000c /* Counter Increment Interrupt Register */
#define LPC43_RTC_AMR_OFFSET    0x0010 /* Alarm Mask Register */

/* Consolidated time registers */

#define LPC43_RTC_CTIME0_OFFSET 0x0014 /* Consolidated Time Register 0 */
#define LPC43_RTC_CTIME1_OFFSET 0x0018 /* Consolidated Time Register 1 */
#define LPC43_RTC_CTIME2_OFFSET 0x001c /* Consolidated Time Register 2 */

/* Time counter registers */

#define LPC43_RTC_SEC_OFFSET    0x0020 /* Seconds Counter */
#define LPC43_RTC_MIN_OFFSET    0x0024 /* Minutes Register */
#define LPC43_RTC_HOUR_OFFSET   0x0028 /* Hours Register */
#define LPC43_RTC_DOM_OFFSET    0x002c /* Day of Month Register */
#define LPC43_RTC_DOW_OFFSET    0x0030 /* Day of Week Register */
#define LPC43_RTC_DOY_OFFSET    0x0034 /* Day of Year Register */
#define LPC43_RTC_MONTH_OFFSET  0x0038 /* Months Register */
#define LPC43_RTC_YEAR_OFFSET   0x003c /* Years Register */
#define LPC43_RTC_CALIB_OFFSET  0x0040 /* Calibration Value Register */

/* Alarm register group */

#define LPC43_RTC_ASEC_OFFSET   0x0060 /* Alarm value for Seconds */
#define LPC43_RTC_AMIN_OFFSET   0x0064 /* Alarm value for Minutes */
#define LPC43_RTC_AHOUR_OFFSET  0x0068 /* Alarm value for Hours */
#define LPC43_RTC_ADOM_OFFSET   0x006c /* Alarm value for Day of Month */
#define LPC43_RTC_ADOW_OFFSET   0x0070 /* Alarm value for Day of Week */
#define LPC43_RTC_ADOY_OFFSET   0x0074 /* Alarm value for Day of Year */
#define LPC43_RTC_AMON_OFFSET   0x0078 /* Alarm value for Months  */
#define LPC43_RTC_AYEAR_OFFSET  0x007c /* Alarm value for Year */

/* General Purpose Registers.
 *
 * In addition to the RTC registers, 64 general purpose registers are available
 * to store data when the main power supply is switched off. The general purpose
 * registers reside in the RTC power domain and can be battery powered.
 */

#define LPC43_REGFILE_OFFSET(n) (0x0000 + ((n) << 2))
#define LPC43_REGFILE0_OFFSET   0x0000
#define LPC43_REGFILE1_OFFSET   0x0004
#define LPC43_REGFILE2_OFFSET   0x0008
#define LPC43_REGFILE3_OFFSET   0x000c
#define LPC43_REGFILE4_OFFSET   0x0010
#define LPC43_REGFILE5_OFFSET   0x0014
#define LPC43_REGFILE6_OFFSET   0x0018
#define LPC43_REGFILE7_OFFSET   0x001c
#define LPC43_REGFILE8_OFFSET   0x0020
#define LPC43_REGFILE9_OFFSET   0x0024
#define LPC43_REGFILE10_OFFSET  0x0028
#define LPC43_REGFILE11_OFFSET  0x002c
#define LPC43_REGFILE12_OFFSET  0x0030
#define LPC43_REGFILE13_OFFSET  0x0034
#define LPC43_REGFILE14_OFFSET  0x0038
#define LPC43_REGFILE15_OFFSET  0x003c
#define LPC43_REGFILE16_OFFSET  0x0040
#define LPC43_REGFILE17_OFFSET  0x0044
#define LPC43_REGFILE18_OFFSET  0x0048
#define LPC43_REGFILE19_OFFSET  0x004c
#define LPC43_REGFILE20_OFFSET  0x0050
#define LPC43_REGFILE21_OFFSET  0x0054
#define LPC43_REGFILE22_OFFSET  0x0058
#define LPC43_REGFILE23_OFFSET  0x005c
#define LPC43_REGFILE24_OFFSET  0x0060
#define LPC43_REGFILE25_OFFSET  0x0064
#define LPC43_REGFILE26_OFFSET  0x0068
#define LPC43_REGFILE27_OFFSET  0x006c
#define LPC43_REGFILE28_OFFSET  0x0070
#define LPC43_REGFILE29_OFFSET  0x0074
#define LPC43_REGFILE30_OFFSET  0x0078
#define LPC43_REGFILE31_OFFSET  0x007c
#define LPC43_REGFILE32_OFFSET  0x0080
#define LPC43_REGFILE33_OFFSET  0x0084
#define LPC43_REGFILE34_OFFSET  0x0088
#define LPC43_REGFILE35_OFFSET  0x008c
#define LPC43_REGFILE36_OFFSET  0x0090
#define LPC43_REGFILE37_OFFSET  0x0094
#define LPC43_REGFILE38_OFFSET  0x0098
#define LPC43_REGFILE39_OFFSET  0x009c
#define LPC43_REGFILE40_OFFSET  0x00a0
#define LPC43_REGFILE41_OFFSET  0x00a4
#define LPC43_REGFILE42_OFFSET  0x00a8
#define LPC43_REGFILE43_OFFSET  0x00ac
#define LPC43_REGFILE44_OFFSET  0x00b0
#define LPC43_REGFILE45_OFFSET  0x00b4
#define LPC43_REGFILE46_OFFSET  0x00b8
#define LPC43_REGFILE47_OFFSET  0x00bc
#define LPC43_REGFILE48_OFFSET  0x00c0
#define LPC43_REGFILE49_OFFSET  0x00c4
#define LPC43_REGFILE50_OFFSET  0x00c8
#define LPC43_REGFILE51_OFFSET  0x00cc
#define LPC43_REGFILE52_OFFSET  0x00d0
#define LPC43_REGFILE53_OFFSET  0x00d4
#define LPC43_REGFILE54_OFFSET  0x00d8
#define LPC43_REGFILE55_OFFSET  0x00dc
#define LPC43_REGFILE56_OFFSET  0x00e0
#define LPC43_REGFILE57_OFFSET  0x00e4
#define LPC43_REGFILE58_OFFSET  0x00e8
#define LPC43_REGFILE59_OFFSET  0x00ec
#define LPC43_REGFILE60_OFFSET  0x00f0
#define LPC43_REGFILE61_OFFSET  0x00f4
#define LPC43_REGFILE62_OFFSET  0x00f8
#define LPC43_REGFILE63_OFFSET  0x00fc

/* Register addresses ***************************************************************/
/* Miscellaneous registers */

#define LPC43_RTC_ILR           (LPC43_RTC_BASE+LPC43_RTC_ILR_OFFSET)
#define LPC43_RTC_CCR           (LPC43_RTC_BASE+LPC43_RTC_CCR_OFFSET)
#define LPC43_RTC_CIIR          (LPC43_RTC_BASE+LPC43_RTC_CIIR_OFFSET)
#define LPC43_RTC_AMR           (LPC43_RTC_BASE+LPC43_RTC_AMR_OFFSET)

/* Consolidated time registers */

#define LPC43_RTC_CTIME0        (LPC43_RTC_BASE+LPC43_RTC_CTIME0_OFFSET)
#define LPC43_RTC_CTIME1        (LPC43_RTC_BASE+LPC43_RTC_CTIME1_OFFSET)
#define LPC43_RTC_CTIME2        (LPC43_RTC_BASE+LPC43_RTC_CTIME2_OFFSET)

/* Time counter registers */

#define LPC43_RTC_SEC           (LPC43_RTC_BASE+LPC43_RTC_SEC_OFFSET)
#define LPC43_RTC_MIN           (LPC43_RTC_BASE+LPC43_RTC_MIN_OFFSET)
#define LPC43_RTC_HOUR          (LPC43_RTC_BASE+LPC43_RTC_HOUR_OFFSET)
#define LPC43_RTC_DOM           (LPC43_RTC_BASE+LPC43_RTC_DOM_OFFSET)
#define LPC43_RTC_DOW           (LPC43_RTC_BASE+LPC43_RTC_DOW_OFFSET)
#define LPC43_RTC_DOY           (LPC43_RTC_BASE+LPC43_RTC_DOY_OFFSET)
#define LPC43_RTC_MONTH         (LPC43_RTC_BASE+LPC43_RTC_MONTH_OFFSET)
#define LPC43_RTC_YEAR          (LPC43_RTC_BASE+LPC43_RTC_YEAR_OFFSET)
#define LPC43_RTC_CALIB         (LPC43_RTC_BASE+LPC43_RTC_CALIB_OFFSET)

/* Alarm register group */

#define LPC43_RTC_ASEC          (LPC43_RTC_BASE+LPC43_RTC_ASEC_OFFSET)
#define LPC43_RTC_AMIN          (LPC43_RTC_BASE+LPC43_RTC_AMIN_OFFSET)
#define LPC43_RTC_AHOUR         (LPC43_RTC_BASE+LPC43_RTC_AHOUR_OFFSET)
#define LPC43_RTC_ADOM          (LPC43_RTC_BASE+LPC43_RTC_ADOM_OFFSET)
#define LPC43_RTC_ADOW          (LPC43_RTC_BASE+LPC43_RTC_ADOW_OFFSET)
#define LPC43_RTC_ADOY          (LPC43_RTC_BASE+LPC43_RTC_ADOY_OFFSET)
#define LPC43_RTC_AMON          (LPC43_RTC_BASE+LPC43_RTC_AMON_OFFSET)
#define LPC43_RTC_AYEAR         (LPC43_RTC_BASE+LPC43_RTC_AYEAR_OFFSET)

/* General Purpose Registers */

#define LPC43_REGFILE(n)        (LPC43_BACKUP_BASE+LPC43_REGFILE_OFFSET(n))
#define LPC43_REGFILE0          (LPC43_BACKUP_BASE+LPC43_REGFILE0_OFFSET)
#define LPC43_REGFILE1          (LPC43_BACKUP_BASE+LPC43_REGFILE1_OFFSET)
#define LPC43_REGFILE2          (LPC43_BACKUP_BASE+LPC43_REGFILE2_OFFSET)
#define LPC43_REGFILE3          (LPC43_BACKUP_BASE+LPC43_REGFILE3_OFFSET)
#define LPC43_REGFILE4          (LPC43_BACKUP_BASE+LPC43_REGFILE4_OFFSET)
#define LPC43_REGFILE5          (LPC43_BACKUP_BASE+LPC43_REGFILE5_OFFSET)
#define LPC43_REGFILE6          (LPC43_BACKUP_BASE+LPC43_REGFILE6_OFFSET)
#define LPC43_REGFILE7          (LPC43_BACKUP_BASE+LPC43_REGFILE7_OFFSET)
#define LPC43_REGFILE8          (LPC43_BACKUP_BASE+LPC43_REGFILE8_OFFSET)
#define LPC43_REGFILE9          (LPC43_BACKUP_BASE+LPC43_REGFILE9_OFFSET)
#define LPC43_REGFILE10         (LPC43_BACKUP_BASE+LPC43_REGFILE10_OFFSET)
#define LPC43_REGFILE11         (LPC43_BACKUP_BASE+LPC43_REGFILE11_OFFSET)
#define LPC43_REGFILE12         (LPC43_BACKUP_BASE+LPC43_REGFILE12_OFFSET)
#define LPC43_REGFILE13         (LPC43_BACKUP_BASE+LPC43_REGFILE13_OFFSET)
#define LPC43_REGFILE14         (LPC43_BACKUP_BASE+LPC43_REGFILE14_OFFSET)
#define LPC43_REGFILE15         (LPC43_BACKUP_BASE+LPC43_REGFILE15_OFFSET)
#define LPC43_REGFILE16         (LPC43_BACKUP_BASE+LPC43_REGFILE16_OFFSET)
#define LPC43_REGFILE17         (LPC43_BACKUP_BASE+LPC43_REGFILE17_OFFSET)
#define LPC43_REGFILE18         (LPC43_BACKUP_BASE+LPC43_REGFILE18_OFFSET)
#define LPC43_REGFILE19         (LPC43_BACKUP_BASE+LPC43_REGFILE19_OFFSET)
#define LPC43_REGFILE20         (LPC43_BACKUP_BASE+LPC43_REGFILE20_OFFSET)
#define LPC43_REGFILE21         (LPC43_BACKUP_BASE+LPC43_REGFILE21_OFFSET)
#define LPC43_REGFILE22         (LPC43_BACKUP_BASE+LPC43_REGFILE22_OFFSET)
#define LPC43_REGFILE23         (LPC43_BACKUP_BASE+LPC43_REGFILE23_OFFSET)
#define LPC43_REGFILE24         (LPC43_BACKUP_BASE+LPC43_REGFILE24_OFFSET)
#define LPC43_REGFILE25         (LPC43_BACKUP_BASE+LPC43_REGFILE25_OFFSET)
#define LPC43_REGFILE26         (LPC43_BACKUP_BASE+LPC43_REGFILE26_OFFSET)
#define LPC43_REGFILE27         (LPC43_BACKUP_BASE+LPC43_REGFILE27_OFFSET)
#define LPC43_REGFILE28         (LPC43_BACKUP_BASE+LPC43_REGFILE28_OFFSET)
#define LPC43_REGFILE29         (LPC43_BACKUP_BASE+LPC43_REGFILE29_OFFSET)
#define LPC43_REGFILE30         (LPC43_BACKUP_BASE+LPC43_REGFILE30_OFFSET)
#define LPC43_REGFILE31         (LPC43_BACKUP_BASE+LPC43_REGFILE31_OFFSET)
#define LPC43_REGFILE32         (LPC43_BACKUP_BASE+LPC43_REGFILE32_OFFSET)
#define LPC43_REGFILE33         (LPC43_BACKUP_BASE+LPC43_REGFILE33_OFFSET)
#define LPC43_REGFILE34         (LPC43_BACKUP_BASE+LPC43_REGFILE34_OFFSET)
#define LPC43_REGFILE35         (LPC43_BACKUP_BASE+LPC43_REGFILE35_OFFSET)
#define LPC43_REGFILE36         (LPC43_BACKUP_BASE+LPC43_REGFILE36_OFFSET)
#define LPC43_REGFILE37         (LPC43_BACKUP_BASE+LPC43_REGFILE37_OFFSET)
#define LPC43_REGFILE38         (LPC43_BACKUP_BASE+LPC43_REGFILE38_OFFSET)
#define LPC43_REGFILE39         (LPC43_BACKUP_BASE+LPC43_REGFILE39_OFFSET)
#define LPC43_REGFILE40         (LPC43_BACKUP_BASE+LPC43_REGFILE40_OFFSET)
#define LPC43_REGFILE41         (LPC43_BACKUP_BASE+LPC43_REGFILE41_OFFSET)
#define LPC43_REGFILE42         (LPC43_BACKUP_BASE+LPC43_REGFILE42_OFFSET)
#define LPC43_REGFILE43         (LPC43_BACKUP_BASE+LPC43_REGFILE43_OFFSET)
#define LPC43_REGFILE44         (LPC43_BACKUP_BASE+LPC43_REGFILE44_OFFSET)
#define LPC43_REGFILE45         (LPC43_BACKUP_BASE+LPC43_REGFILE45_OFFSET)
#define LPC43_REGFILE46         (LPC43_BACKUP_BASE+LPC43_REGFILE46_OFFSET)
#define LPC43_REGFILE47         (LPC43_BACKUP_BASE+LPC43_REGFILE47_OFFSET)
#define LPC43_REGFILE48         (LPC43_BACKUP_BASE+LPC43_REGFILE48_OFFSET)
#define LPC43_REGFILE49         (LPC43_BACKUP_BASE+LPC43_REGFILE49_OFFSET)
#define LPC43_REGFILE50         (LPC43_BACKUP_BASE+LPC43_REGFILE50_OFFSET)
#define LPC43_REGFILE51         (LPC43_BACKUP_BASE+LPC43_REGFILE51_OFFSET)
#define LPC43_REGFILE52         (LPC43_BACKUP_BASE+LPC43_REGFILE52_OFFSET)
#define LPC43_REGFILE53         (LPC43_BACKUP_BASE+LPC43_REGFILE53_OFFSET)
#define LPC43_REGFILE54         (LPC43_BACKUP_BASE+LPC43_REGFILE54_OFFSET)
#define LPC43_REGFILE55         (LPC43_BACKUP_BASE+LPC43_REGFILE55_OFFSET)
#define LPC43_REGFILE56         (LPC43_BACKUP_BASE+LPC43_REGFILE56_OFFSET)
#define LPC43_REGFILE57         (LPC43_BACKUP_BASE+LPC43_REGFILE57_OFFSET)
#define LPC43_REGFILE58         (LPC43_BACKUP_BASE+LPC43_REGFILE58_OFFSET)
#define LPC43_REGFILE59         (LPC43_BACKUP_BASE+LPC43_REGFILE59_OFFSET)
#define LPC43_REGFILE60         (LPC43_BACKUP_BASE+LPC43_REGFILE60_OFFSET)
#define LPC43_REGFILE61         (LPC43_BACKUP_BASE+LPC43_REGFILE61_OFFSET)
#define LPC43_REGFILE62         (LPC43_BACKUP_BASE+LPC43_REGFILE62_OFFSET)
#define LPC43_REGFILE63         (LPC43_BACKUP_BASE+LPC43_REGFILE63_OFFSET)

/* Register bit definitions *********************************************************/
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

#define RTC_ASEC_MASK           (0x003f)
#define RTC_AMIN_MASK           (0x003f)
#define RTC_AHOUR_MASK          (0x001f)
#define RTC_ADOM_MASK           (0x001f)
#define RTC_ADOW_MASK           (0x0007)
#define RTC_ADOY_MASK           (0x01ff)
#define RTC_AMON_MASK           (0x000f)
#define RTC_AYEAR_MASK          (0x0fff)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_LPC43_RTC_H */
