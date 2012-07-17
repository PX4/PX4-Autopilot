/****************************************************************************
 * include/wdog.h
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_WDOG_H
#define __INCLUDE_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Global Type Declarations
 ****************************************************************************/

/* The arguments are passed as uint32_t values.  For systems where the
 * sizeof(pointer) < sizeof(uint32_t), the following union defines the
 * alignment of the pointer within the uint32_t.  For example, the SDCC
 * MCS51 general pointer is 24-bits, but uint32_t is 32-bits (of course).
 *
 * For systems where sizeof(pointer) > sizeof(uint32_t), we will have to do
 * some redesign.
 */

union wdparm_u
{
  FAR void     *pvarg;
  FAR uint32_t *dwarg;
};

typedef union wdparm_u wdparm_t;

/* This is the form of the function that is called when the
 * watchdog function expires.  Up to four parameters may be passed.
 */

typedef CODE void (*wdentry_t)(int argc, uint32_t arg1, ...);

/* Watchdog 'handle' */

typedef FAR struct wdog_s *WDOG_ID;

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN WDOG_ID wd_create(void);
EXTERN int     wd_delete(WDOG_ID wdog);
EXTERN int     wd_start(WDOG_ID wdog, int delay, wdentry_t wdentry, int argc, ...);
EXTERN int     wd_cancel(WDOG_ID wdog);
EXTERN int     wd_gettime(WDOG_ID wdog);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* _WDOG_H_ */
