/************************************************************************************
 * include/nuttx/input/touchscreen.h
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
 ************************************************************************************/

/* The TOUCHSCREEN driver exports a standard character driver interface. By
 * convention, the touchscreen driver is registers as an input device at
 * /dev/inputN where N uniquely identifies the driver instance.
 *
 * This header file documents the generic interface that all NuttX
 * touchscreen devices must conform.  It adds standards and conventions on
 * top of the standard character driver interface.
 */

#ifndef __INCLUDE_NUTTX_INPUT_TOUCHSCREEN_H
#define __INCLUDE_NUTTX_INPUT_TOUCHSCREEN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_INPUT

/************************************************************************************
 * Pre-Processor Definitions
 ************************************************************************************/
/* IOCTL Commands *******************************************************************/

#define TSIOC_SETCALIB       _TSIOC(0x0001)  /* arg: Pointer to int calibration value */
#define TSIOC_GETCALIB       _TSIOC(0x0002)  /* arg: Pointer to int calibration value */
#define TSIOC_SETFREQUENCY   _TSIOC(0x0003)  /* arg: Pointer to uint32_t frequency value */
#define TSIOC_GETFREQUENCY   _TSIOC(0x0004)  /* arg: Pointer to uint32_t frequency value */

/* Specific touchscreen drivers may support additional, device specific ioctal
 * commands, beginning with this value:
 */

#define TSIOC_USER           0x0005         /* Lowest, unused TSC ioctl command */

/* These definitions provide the meaning of all of the bits that may be
 * reported in the struct touch_point_s flags.
 */

#define TOUCH_DOWN           (1 << 0) /* A new touch contact is established */
#define TOUCH_MOVE           (1 << 1) /* Movement occurred with previously reported contact */
#define TOUCH_UP             (1 << 2) /* The touch contact was lost */
#define TOUCH_ID_VALID       (1 << 3) /* Touch ID is uncertain */
#define TOUCH_POS_VALID      (1 << 4) /* Hardware provided a valid X/Y position */
#define TOUCH_PRESSURE_VALID (1 << 5) /* Hardware provided a valid pressure */
#define TOUCH_SIZE_VALID     (1 << 6) /* Hardware provided a valid H/W contact size */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* This structure contains information about a single touch point.
 * Positional units are device specific.
 */

struct touch_point_s
{
  uint8_t  id;       /* Unique identifies contact; Same in all reports for the contact */
  uint8_t  flags;    /* See TOUCH_* definitions above */
  int16_t  x;        /* X coordinate of the touch point (uncalibrated) */
  int16_t  y;        /* Y coordinate of the touch point (uncalibrated) */
  int16_t  h;        /* Height of touch point (uncalibrated) */
  int16_t  w;        /* Width of touch point (uncalibrated) */
  uint16_t pressure; /* Touch pressure */
};

/* The typical touchscreen driver is a read-only, input character device driver.
 * the driver write() method is not supported and any attempt to open the
 * driver in any mode other than read-only will fail.
 *
 * Data read from the touchscreen device consists only of touch events and
 * touch sample data.  This is reflected by struct touch_sample_s.  This
 * structure is returned by either the driver read method.
 *
 * On some devices, multiple touchpoints may be supported. So this top level
 * data structure is a struct touch_sample_s that "contains" a set of touch
 * points.  Each touch point is managed individually using an ID that identifies
 * a touch from first contact until the end of the contact.
 */

struct touch_sample_s
{
  int npoints;                   /* The number of touch points in point[] */
  struct touch_point_s point[1]; /* Actual dimension is npoints */
};
#define SIZEOF_TOUCH_SAMPLE_S(n) (sizeof(struct touch_sample_s) + ((n)-1)*sizeof(struct touch_point_s))

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: arch_tcinitialize
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   configure the touchscreen device.  This function will register the driver
 *   as /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

EXTERN int arch_tcinitialize(int minor);

/****************************************************************************
 * Name: arch_tcuninitialize
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   uninitialize the touchscreen device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

EXTERN void arch_tcuninitialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT */
#endif /* __INCLUDE_NUTTX_INPUT_TOUCHSCREEN_H */
