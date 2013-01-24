/****************************************************************************
 * include/nuttx/clock.h
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef _INCLUDE_NUTTX_CLOCK_H
#define _INCLUDE_NUTTX_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pro-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Efficient, direct access to OS global timer variables will be supported
 * if the execution environment has direct access to kernel global data.
 * The code in this execution context can access the kernel global data
 * directly if:  (1) this is an un-protected, non-kernel build, or (2)
 * this code is being built for execution within the kernel.
 */

#undef __HAVE_KERNEL_GLOBALS
#if !defined(CONFIG_NUTTX_KERNEL) || defined(__KERNEL__)
#  define __HAVE_KERNEL_GLOBALS 1
#else
#  define __HAVE_KERNEL_GLOBALS 0
#endif

/* If CONFIG_SYSTEM_TIME64 is selected and the CPU supports long long types,
 * then a 64-bit system time will be used.
 */

#ifndef CONFIG_HAVE_LONG_LONG
#  undef CONFIG_SYSTEM_TIME64
#endif

/* Timing constants *********************************************************/

#define NSEC_PER_SEC          1000000000
#define USEC_PER_SEC             1000000
#define MSEC_PER_SEC                1000
#define DSEC_PER_SEC                  10
#define NSEC_PER_DSEC          100000000
#define USEC_PER_DSEC             100000
#define MSEC_PER_DSEC                100
#define NSEC_PER_MSEC            1000000
#define USEC_PER_MSEC               1000
#define NSEC_PER_USEC               1000

/* The interrupt interval of the system timer is given by MSEC_PER_TICK.
 * This is the expected number of milliseconds between calls from the
 * processor-specific logic to sched_process_timer().  The default value
 * of MSEC_PER_TICK is 10 milliseconds (100KHz).  However, this default
 * setting can be overridden by defining the interval in milliseconds as
 * CONFIG_MSEC_PER_TICK in the board configuration file.
 *
 * The following calculations are only accurate when (1) there is no
 * truncation involved and (2) the underlying system timer is an even
 * multiple of milliseconds.  If (2) is not true, you will probably want
 * to redefine all of the following.
 */

#ifdef CONFIG_MSEC_PER_TICK
# define MSEC_PER_TICK        (CONFIG_MSEC_PER_TICK)
#else
# define MSEC_PER_TICK        (10)
#endif

#define TICK_PER_DSEC         (MSEC_PER_DSEC / MSEC_PER_TICK)            /* Truncates! */
#define TICK_PER_SEC          (MSEC_PER_SEC / MSEC_PER_TICK)             /* Truncates! */
#define NSEC_PER_TICK         (MSEC_PER_TICK * NSEC_PER_MSEC)            /* Exact */
#define USEC_PER_TICK         (MSEC_PER_TICK * USEC_PER_MSEC)            /* Exact */

#define NSEC2TICK(nsec)       (((nsec)+(NSEC_PER_TICK/2))/NSEC_PER_TICK) /* Rounds */
#define USEC2TICK(usec)       (((usec)+(USEC_PER_TICK/2))/USEC_PER_TICK) /* Rounds */
#define MSEC2TICK(msec)       (((msec)+(MSEC_PER_TICK/2))/MSEC_PER_TICK) /* Rounds */
#define DSEC2TICK(dsec)       MSEC2TICK((dsec)*MSEC_PER_DSEC)
#define SEC2TICK(sec)         MSEC2TICK((sec)*MSEC_PER_SEC)              /* Exact */

#define TICK2NSEC(tick)       ((tick)*NSEC_PER_TICK)                     /* Exact */
#define TICK2USEC(tick)       ((tick)*USEC_PER_TICK)                     /* Exact */
#define TICK2MSEC(tick)       ((tick)*MSEC_PER_TICK)                     /* Exact */
#define TICK2DSEC(tick)       (((tick)+(TICK_PER_DSEC/2))/TICK_PER_DSEC) /* Rounds */
#define TICK2SEC(tick)        (((tick)+(TICK_PER_SEC/2))/TICK_PER_SEC)   /* Rounds */

/****************************************************************************
 * Global Data
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_CLOCK)

/* Access to raw system clock ***********************************************/
/* Direct access to the system timer/counter is supported only if (1) the
 * system timer counter is available (i.e., we are not configured to use
 * a hardware periodic timer), and (2) the execution environment has direct
 * access to kernel global data
 */

#if __HAVE_KERNEL_GLOBALS
#  ifdef CONFIG_SYSTEM_TIME64

extern volatile uint64_t g_system_timer;
#define clock_systimer()  (uint32_t)(g_system_timer & 0x00000000ffffffff)
#define clock_systimer64() g_system_timer

#  else

extern volatile uint32_t g_system_timer;
#define clock_systimer() g_system_timer

#  endif
#endif

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Function:  clock_synchronize
 *
 * Description:
 *   Synchronize the system timer to a hardware RTC.  This operation is
 *   normally performed automatically by the system during clock
 *   initialization.  However, the user may also need to explicitly re-
 *   synchronize the system timer to the RTC under certain conditions where
 *   the system timer is known to be in error.  For example, in certain low-
 *   power states, the system timer may be stopped but the RTC will continue
 *   keep correct time.  After recovering from such low-power state, this
 *   function should be called to restore the correct system time.
 *
 *   Calling this function could result in system time going "backward" in
 *   time, especially with certain lower resolution RTC implementations.
 *   Time going backward could have bad consequences if there are ongoing
 *   timers and delays.  So use this interface with care.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_RTC
EXTERN void clock_synchronize(void);
#endif

/****************************************************************************
 * Function:  clock_systimer
 *
 * Description:
 *   Return the current value of the 32-bit system timer counter.  Indirect
 *   access to the system timer counter is required through this function if
 *   the execution environment does not have direct access to kernel global
 *   data
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   The current value of the system timer counter
 *
 * Assumptions:
 *
 ****************************************************************************/

#if !__HAVE_KERNEL_GLOBALS
#  ifdef CONFIG_SYSTEM_TIME64
#    define clock_systimer()  (uint32_t)(clock_systimer64() & 0x00000000ffffffff)
#  else
EXTERN uint32_t clock_systimer(void);
#  endif
#endif

/****************************************************************************
 * Function:  clock_systimer64
 *
 * Description:
 *   Return the current value of the 64-bit system timer counter.  Indirect
 *   access to the system timer counter is required through this function if
 *   the execution environment does not have direct access to kernel global
 *   data
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   The current value of the system timer counter
 *
 * Assumptions:
 *
 ****************************************************************************/

#if !__HAVE_KERNEL_GLOBALS && defined(CONFIG_SYSTEM_TIME64)
EXTERN uint64_t clock_systimer64(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* !CONFIG_DISABLE_CLOCK */
#endif /* _INCLUDE_NUTTX_CLOCK_H */
