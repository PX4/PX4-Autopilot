/********************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-rtcc.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_RTCC_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_RTCC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "pic32mx-memorymap.h"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/
/* Register Offsets *************************************************************************/

#define PIC32MX_RTCC_CON_OFFSET         0x0000 /* RTC Control Register */
#define PIC32MX_RTCC_CONCLR_OFFSET      0x0004 /* RTC Control Clear Register */
#define PIC32MX_RTCC_CONSET_OFFSET      0x0008 /* RTC Control Set Register */
#define PIC32MX_RTCC_CONINV_OFFSET      0x000c /* RTC Control Invert Register */
#define PIC32MX_RTCC_ALRM_OFFSET        0x0010 /* RTC ALARM Control Register */
#define PIC32MX_RTCC_ALRMCLR_OFFSET     0x0014 /* RTC ALARM Control Clear Register */
#define PIC32MX_RTCC_ALRMSET_OFFSET     0x0018 /* RTC ALARM Control Set Register */
#define PIC32MX_RTCC_ALRMINV_OFFSET     0x001c /* RTC ALARM Control Invert Register */
#define PIC32MX_RTCC_TIME_OFFSET        0x0020 /* RTC Time Value Register */
#define PIC32MX_RTCC_TIMECLR_OFFSET     0x0024 /* RTC Time Value Clear Register */
#define PIC32MX_RTCC_TIMESET_OFFSET     0x0028 /* RTC Time Value Set Register */
#define PIC32MX_RTCC_TIMEINV_OFFSET     0x002c /* RTC Time Value Invert Register */
#define PIC32MX_RTCC_DATE_OFFSET        0x0030 /* RTC Date Value Register */
#define PIC32MX_RTCC_DATECLR_OFFSET     0x0034 /* RTC Date Value Clear Register */
#define PIC32MX_RTCC_DATESET_OFFSET     0x0038 /* RTC Date Value Set Register */
#define PIC32MX_RTCC_DATEINV_OFFSET     0x003c /* RTC Date Value Invert Register */
#define PIC32MX_RTCC_ALRMTIME_OFFSET    0x0040 /* Alarm Time Value Register */
#define PIC32MX_RTCC_ALRMTIMECLR_OFFSET 0x0044 /* Alarm Time Value Clear Register */
#define PIC32MX_RTCC_ALRMTIMESET_OFFSET 0x0048 /* Alarm Time Value Set Register */
#define PIC32MX_RTCC_ALRMTIMEINV_OFFSET 0x004c /* Alarm Time Value Invert Register */
#define PIC32MX_RTCC_ALRMDATE_OFFSET    0x0050 /* Alarm Date Value Register */
#define PIC32MX_RTCC_ALRMDATECLR_OFFSET 0x0054 /* Alarm Date Value Clear Register */
#define PIC32MX_RTCC_ALRMDATESET_OFFSET 0x0058 /* Alarm Date Value Set Register */
#define PIC32MX_RTCC_ALRMDATEINV_OFFSET 0x005c /* Alarm Date Value Invert Register */

/* Register Addresses ***********************************************************************/

#define PIC32MX_RTCC_CON                (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_CON_OFFSET)
#define PIC32MX_RTCC_CONCLR             (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_CONCLR_OFFSET)
#define PIC32MX_RTCC_CONSET             (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_CONSET_OFFSET)
#define PIC32MX_RTCC_CONINV             (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_CONINV_OFFSET)
#define PIC32MX_RTCC_ALRM               (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_ALRM_OFFSET)
#define PIC32MX_RTCC_ALRMCLR            (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_ALRMCLR_OFFSET)
#define PIC32MX_RTCC_ALRMSET            (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_ALRMSET_OFFSET)
#define PIC32MX_RTCC_ALRMINV            (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_ALRMINV_OFFSET)
#define PIC32MX_RTCC_TIME               (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_TIME_OFFSET)
#define PIC32MX_RTCC_TIMECLR            (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_TIMECLR_OFFSET)
#define PIC32MX_RTCC_TIMESET            (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_TIMESET_OFFSET)
#define PIC32MX_RTCC_TIMEINV            (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_TIMEINV_OFFSET)
#define PIC32MX_RTCC_DATE               (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_DATE_OFFSET)
#define PIC32MX_RTCC_DATECLR            (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_DATECLR_OFFSET)
#define PIC32MX_RTCC_DATESET            (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_DATESET_OFFSET)
#define PIC32MX_RTCC_DATEINV            (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_DATEINV_OFFSET)
#define PIC32MX_RTCC_ALRMTIME           (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_ALRMTIME_OFFSET)
#define PIC32MX_RTCC_ALRMTIMECLR        (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_ALRMTIMECLR_OFFSET)
#define PIC32MX_RTCC_ALRMTIMESET        (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_ALRMTIMESET_OFFSET)
#define PIC32MX_RTCC_ALRMTIMEINV        (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_ALRMTIMEINV_OFFSET)
#define PIC32MX_RTCC_ALRMDATE           (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_ALRMDATE_OFFSET)
#define PIC32MX_RTCC_ALRMDATECLR        (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_ALRMDATECLR_OFFSET)
#define PIC32MX_RTCC_ALRMDATESET        (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_ALRMDATESET_OFFSET)
#define PIC32MX_RTCC_ALRMDATEINV        (PIC32MX_RTCC_K1BASE+PIC32MX_RTCC_ALRMDATEINV_OFFSET)

/* Register Bit-Field Definitions ***********************************************************/

/* RTC Control Register */

#define RTCC_CON_CAL_SHIFT              (16)      /* Bits 16-25: RTC drift calibration */
#define RTCC_CON_CAL_MASK               (0x3ff << RTCC_CON_CAL_SHIFT) /* 10-bit 2's complement */
#  define RTCC_CON_CAL_MAX              (0x1ff << RTCC_CON_CAL_SHIFT)
#  define RTCC_CON_CAL_CENTER           (0x000 << RTCC_CON_CAL_SHIFT)
#  define RTCC_CON_CAL_MIN              (0x200 << RTCC_CON_CAL_SHIFT)
#define RTCC_CON_ON                     (1 << 15) /* Bit 15: RTCC on */
#define RTCC_CON_FRZ                    (1 << 14) /* Bit 14: Freeze in debug mode */
#define RTCC_CON_SIDL                   (1 << 13) /* Bit 13: Stop in idle mode */
#define RTCC_CON_RTSECSEL               (1 << 9)  /* Bit 7:  RTCC seconds clock output select */
#define RTCC_CON_RTCCLKON               (1 << 8)  /* Bit 6:  RTCC clock enable status */
#define RTCC_CON_RTCWREN                (1 << 3)  /* Bit 3:  RTC value registers write enable */
#define RTCC_CON_RTCSYNC                (1 << 2)  /* Bit 2:  RTCC value registers read synchronization */
#define RTCC_CON_HALFSEC                (1 << 1)  /* Bit 1:  Half-second status */
#define RTCC_CON_RTCOE                  (1 << 0)  /* Bit 0:  RTCC output enable */

/* RTC ALARM Control Register */

#define RTCC_ALRM_ARPT_SHIFT            (0)       /* Bits 0-7: Alarm repeat counter value */
#define RTCC_ALRM_ARPT_MASK             (0xff << RTCC_ALRM_ARPT_SHIFT)
#define RTCC_ALRM_AMASK_SHIFT           (8)       /* Bits 8-11: Alarm mask configuration */
#define RTCC_ALRM_AMASK_MASK            (15 << RTCC_ALRM_AMASK_SHIFT)
#define RTCC_ALRM_ALRMSYNC              (1 << 12) /* Bit 12: Alarm sync */
#define RTCC_ALRM_PIV                   (1 << 13) /* Bit 13: Alarm pulse initial value */
#define RTCC_ALRM_CHIME                 (1 << 14) /* Bit 14: Chime enable */
#define RTCC_ALRM_ALRMEN                (1 << 15) /* Bit 15: Alarm enable */

/* RTC Time Value Register */

#define RTCC_TIME_SEC01_SHIFT           (8)       /* Bits 8-11: BCD seconds, 1 digit */
#define RTCC_TIME_SEC01_MASK            (15 << RTCC_TIME_SEC01_SHIFT)
#define RTCC_TIME_SEC10_SHIFT           (12)      /* Bits 12-14: BCD seconds, 10 digits */
#define RTCC_TIME_SEC10_MASK            (7 << RTCC_TIME_SEC10_SHIFT)
#define RTCC_TIME_MIN01_SHIFT           (16)      /* Bits 16-19: BCD minutes, 1 digit */
#define RTCC_TIME_MIN01_MASK            (15 << RTCC_TIME_MIN01_SHIFT)
#define RTCC_TIME_MIN10_SHIFT           (20)      /* Bits 20-22: BCD minutes, 10 digits */
#define RTCC_TIME_MIN10_MASK            (7 << RTCC_TIME_MIN10_SHIFT)
#define RTCC_TIME_HR01_SHIFT            (24)      /* Bits 24-27: BCD hours, 1 digit */
#define RTCC_TIME_HR01_MASK             (15 << RTCC_TIME_HR01_SHIFT)
#define RTCC_TIME_HR10_SHIFT            (28)      /* Bits 28-29: BCD hours, 10 digits */
#define RTCC_TIME_HR10_MASK             (3 << RTCC_TIME_HR10_SHIFT)

/* RTC Date Value Register */

#define RTCC_DATE_WDAY01_SHIFT          (0)       /* Bits 0-2: BCD weekday, 1 digit */
#define RTCC_DATE_WDAY01_MASK           (7 << RTCC_DATE_WDAY01_SHIFT)
#define RTCC_DATE_DAY01_SHIFT           (8)       /* Bits 8-11: BCD day, 1 digit */
#define RTCC_DATE_DAY01_MASK            (15 << RTCC_DATE_DAY01_SHIFT)
#define RTCC_DATE_DAY10_SHIFT           (12)      /* Bits 12-13: BCD day, 10 digits */
#define RTCC_DATE_DAY10_MASK            (3 << RTCC_DATE_DAY10_SHIFT)
#define RTCC_DATE_MONTH01_SHIFT         (16)      /* Bits 16-19: BCD month, 1 digit */
#define RTCC_DATE_MONTH01_MASK          (15 << RTCC_DATE_MONTH01_SHIFT)
#define RTCC_DATE_MONTH10               (1 << 20) /* Bit 20: BCD month, 10 digits */
#define RTCC_DATE_YEAR01_SHIFT          (24)      /* Bits 24-27: BCD year, 1 digit */
#define RTCC_DATE_YEAR01_MASK           (15 << RTCC_DATE_YEAR01_SHIFT)
#define RTCC_DATE_YEAR10_SHIFT          (28)      /* Bits 28-31: BCD year, 10 digits */
#define RTCC_DATE_YEAR10_MASK           (15 << RTCC_DATE_YEAR10_SHIFT)

/* Alarm Time Value Register */

#define RTCC_ALRMTIME_SEC01_SHIFT       (8)       /* Bits 8-11: BCD seconds, 1 digit */
#define RTCC_ALRMTIME_SEC01_MASK        (15 << RTCC_ALRMTIME_SEC01_SHIFT)
#define RTCC_ALRMTIME_SEC10_SHIFT       (12)      /* Bits 12-14: BCD seconds, 1 digit */
#define RTCC_ALRMTIME_SEC10_MASK        (7 << RTCC_ALRMTIME_SEC10_SHIFT)
#define RTCC_ALRMTIME_MIN01_SHIFT       (16)      /* Bits 16-19: BCD minutes, 1 digit */
#define RTCC_ALRMTIME_MIN01_MASK        (15 << RTCC_ALRMTIME_MIN01_SHIFT)
#define RTCC_ALRMTIME_MIN10_SHIFT       (20)      /* Bits 20-22: BCD minutes, 1 digit */
#define RTCC_ALRMTIME_MIN10_MASK        (7 << RTCC_ALRMTIME_MIN10_SHIFT)
#define RTCC_ALRMTIME_HR01_SHIFT        (24)      /* Bits 24-27: BCD hours, 1 digit */
#define RTCC_ALRMTIME_HR01_MASK         (15 << RTCC_ALRMTIME_HR01_SHIFT)
#define RTCC_ALRMTIME_HR10_SHIFT        (28)      /* Bits 28-29: BCD hours, 1 digit */
#define RTCC_ALRMTIME_HR10_MASK         (3 << RTCC_ALRMTIME_HR10_SHIFT)

/* Alarm Date Value Register */

#define RTCC_ALRMDATE_WDAY01_SHIFT      (0)       /* Bits 0-2: BCD weekday, 1 digit */
#define RTCC_ALRMDATE_WDAY01_MASK       (7 << RTCC_ALRMDATE_WDAY01_SHIFT)
#define RTCC_ALRMDATE_DAY01_SHIFT       (8)       /* Bits 8-11: BCD days, 1 digit */
#define RTCC_ALRMDATE_DAY01_MASK        (15 << RTCC_ALRMDATE_DAY01_SHIFT)
#define RTCC_ALRMDATE_DAY10_SHIFT       (12)      /* Bits 12-13: BCD days, 1 digit */
#define RTCC_ALRMDATE_DAY10_MASK        (3 << RTCC_ALRMDATE_DAY10_SHIFT)
#define RTCC_ALRMDATE_MONTH01_SHIFT     (16)      /* Bits 16-19: BCD month, 1 digit */
#define RTCC_ALRMDATE_MONTH01_MASK      (15 << RTCC_ALRMDATE_MONTH01_SHIFT)
#define RTCC_DATE_MONTH10               (1 << 20) /* Bit 20: BCD month, 10 digits */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

#ifndef __ASSEMBLY__

/********************************************************************************************
 * Inline Functions
 ********************************************************************************************/

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

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
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_RTCC_H */
