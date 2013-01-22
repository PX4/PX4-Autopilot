/****************************************************************************
 * include/nuttx/power/pm.h
 * NuttX Power Management Interfaces
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
/* Definition of terms.  Various "sleep" and low power consumption states
 * have various names and are sometimes used in conflicting ways.  In the
 * PM logic, we will use the following terminology:
 *
 * NORMAL  - The normal, full power operating mode.
 * IDLE    - This is still basically normal operational mode, the system is,
 *           however, IDLE and some simple simple steps to reduce power
 *           consumption provided that they do not interfere with normal
 *           Operation.  Simply dimming the a backlight might be an example
 *           somethat that would be done when the system is idle.
 * STANDBY - Standby is a lower power consumption mode that may involve more
 *           extensive power management steps such has disabling clocking or
 *           setting the processor into reduced power consumption modes. In
 *           this state, the system should still be able to resume normal
 *           activity almost immediately.
 * SLEEP   - The lowest power consumption mode.  The most drastic power
 *           reduction measures possible should be taken in this state. It
 *           may require some time to get back to normal operation from
 *           SLEEP (some MCUs may even require going through reset).
 *
 * State changes always proceed from higher to lower power usage:
 *
 * NORMAL->IDLE->STANDBY->SLEEP
 *   ^       |      |        |
 *   |       V      V        V
 *   +-------+------+--------+
 */

#ifndef __INCLUDE_NUTTX_POWER_PM_H
#define __INCLUDE_NUTTX_POWER_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <queue.h>

#ifdef CONFIG_PM

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_IDLE_CUSTOM. Some architectures support this definition.  This,
 * if defined, will allow you replace the default IDLE loop with your
 * own, custom idle loop to support board-specific IDLE time power management
 */

/* Time slices.  The power management module collects activity counts in
 * time slices.  At the end of the time slice, the count accumulated during
 * that interval is applied to an averaging algorithm to determine the
 * activity level.
 *
 * CONFIG_PM_SLICEMS provides the duration of that time slice.  Default: 100
 * Milliseconds
 */

#ifndef CONFIG_PM_SLICEMS
#  define CONFIG_PM_SLICEMS  100 /* Default is 100 msec */
#endif

/* The averaging algorithm is simply: Y = (An*X + SUM(Ai*Yi))/SUM(Aj), where
 * i = 1..n-1 and j= 1..n, n is the length of the "memory", Ai is the
 * weight applied to each value, and X is the current activity.  These weights
 * may be negative and a limited to the range of int16_t.
 *
 * CONFIG_PM_MEMORY provides the memory for the algorithm.  Default: 2
 * CONFIG_PM_COEFn provides weight for each sample.  Default: 1
 *
 * Setting CONFIG_PM_MEMORY=1 disables all smoothing.
 */

#ifndef CONFIG_PM_MEMORY
#  define CONFIG_PM_MEMORY 2
#endif

#ifndef CONFIG_PM_MEMORY < 1
#  error "CONFIG_PM_MEMORY must be >= 1"
#endif

#ifndef CONFIG_PM_COEFN
#  define CONFIG_PM_COEFN 1
#endif

#if CONFIG_PM_MEMORY > 1 && !defined(CONFIG_PM_COEF1)
#  define CONFIG_PM_COEF1 1
#endif

#if CONFIG_PM_MEMORY > 2 && !defined(CONFIG_PM_COEF2)
#  define CONFIG_PM_COEF2 1
#endif

#if CONFIG_PM_MEMORY > 3 && !defined(CONFIG_PM_COEF3)
#  define CONFIG_PM_COEF3 1
#endif

#if CONFIG_PM_MEMORY > 4 && !defined(CONFIG_PM_COEF4)
#  define CONFIG_PM_COEF4 1
#endif

#if CONFIG_PM_MEMORY > 5 && !defined(CONFIG_PM_COEF5)
#  define CONFIG_PM_COEF5 1
#endif

#if CONFIG_PM_MEMORY > 6
#  warning "This logic needs to be extended"
#endif

/* State changes then occur when the weight activity account crosses
 * threshold values for certain periods of time (time slice count).
 *
 * CONFIG_PM_xxxENTER_THRESH is the threshold value for entering state xxx.
 * CONFIG_PM_xxxENTER_COUNT is the count for entering state xxx.
 *
 * Resuming to normal state, on the other hand, is usually immediate and
 * controlled by wakeup conditions established by the platform.  The PM
 * module only recommends reduced power states.
 */

#ifndef CONFIG_PM_IDLEENTER_THRESH
#  define CONFIG_PM_IDLEENTER_THRESH    1   /* <=1: Essentially no activity */
#endif

#ifndef CONFIG_PM_IDLEEXIT_THRESH
#  define CONFIG_PM_IDLEEXIT_THRESH     2   /* >=2: Active */
#endif

#if CONFIG_PM_IDLEENTER_THRESH >= CONFIG_PM_IDLEEXIT_THRESH
#  error "Must have CONFIG_PM_IDLEENTER_THRESH < CONFIG_PM_IDLEEXIT_THRESH"
#endif

#ifndef CONFIG_PM_IDLEENTER_COUNT
#  define CONFIG_PM_IDLEENTER_COUNT     30  /* Thirty IDLE slices to enter
                                             * IDLE mode from normal
                                             */
#endif

#ifndef CONFIG_PM_STANDBYENTER_THRESH
#  define CONFIG_PM_STANDBYENTER_THRESH 1   /*  <=1: Essentially no activity */
#endif

#ifndef CONFIG_PM_STANDBYEXIT_THRESH
#  define CONFIG_PM_STANDBYEXIT_THRESH  2   /* >=2: Active */
#endif

#if CONFIG_PM_STANDBYENTER_THRESH >= CONFIG_PM_STANDBYEXIT_THRESH
#  error "Must have CONFIG_PM_STANDBYENTER_THRESH < CONFIG_PM_STANDBYEXIT_THRESH"
#endif

#ifndef CONFIG_PM_STANDBYENTER_COUNT
#  define CONFIG_PM_STANDBYENTER_COUNT  50  /* Fifty IDLE slices to enter
                                             * STANDBY mode from IDLE
                                             */
#endif

#ifndef CONFIG_PM_SLEEPENTER_THRESH
#  define CONFIG_PM_SLEEPENTER_THRESH   1   /*  <=1: Essentially no activity */
#endif

#ifndef CONFIG_PM_SLEEPEXIT_THRESH
#  define CONFIG_PM_SLEEPEXIT_THRESH    2   /* >=2: Active */
#endif

#if CONFIG_PM_SLEEPENTER_THRESH >= CONFIG_PM_SLEEPEXIT_THRESH
#  error "Must have CONFIG_PM_SLEEPENTER_THRESH < CONFIG_PM_SLEEPEXIT_THRESH"
#endif

#ifndef CONFIG_PM_SLEEPENTER_COUNT
#  define CONFIG_PM_SLEEPENTER_COUNT    70  /* 70 IDLE slices to enter SLEEP
                                             * mode from STANDBY
                                             */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This enumeration provides all power management states.  Receipt of the
 * state indication is the state transition event.
 */

enum pm_state_e
{
  PM_NORMAL = 0,   /* Normal full power operating mode.  If the driver is in
                    * a reduced power usage mode, it should immediately re-
                    * initialize for normal operatin.
                    *
                    * PM_NORMAL may be followed by PM_IDLE.
                    */
  PM_IDLE,         /* Drivers will receive this state change if it is
                    * appropriate to enter a simple IDLE power state.  This
                    * would include simple things such as reducing display back-
                    * lighting.  The driver should be ready to resume normal
                    * activity instantly.
                    *
                    * PM_IDLE may be followed by PM_STANDBY or PM_NORMAL.
                    */
  PM_STANDBY,      /* The system is entering standby mode. Standby is a lower
                    * power consumption mode that may involve more extensive
                    * power management steps such has disabling clocking or
                    * setting the processor into reduced power consumption
                    * modes. In this state, the system should still be able
                    * to resume normal activity almost immediately.
                    *
                    * PM_STANDBY may be followed PM_SLEEP or by PM_NORMAL
                    */
  PM_SLEEP,        /* The system is entering deep sleep mode.  The most drastic
                    * power reduction measures possible should be taken in this
                    * state. It may require some time to get back to normal
                    * operation from SLEEP (some MCUs may even require going
                    * through reset).
                    *
                    * PM_SLEEP may be following by PM_NORMAL
                    */
};

/* This structure contain pointers callback functions in the driver.  These
 * callback functions can be used to provide power management information
 * to the driver.
 */

struct pm_callback_s
{
  struct sq_entry_s entry;   /* Supports a singly linked list */

  /**************************************************************************
   * Name: prepare
   *
   * Description:
   *   Request the driver to prepare for a new power state. This is a
   *   warning that the system is about to enter into a new power state.  The
   *   driver should begin whatever operations that may be required to enter
   *   power state.  The driver may abort the state change mode by returning
   *   a non-zero value from the callback function
   *
   * Input Parameters:
   *   cb      - Returned to the driver.  The driver version of the callback
   *             strucure may include additional, driver-specific state
   *             data at the end of the structure.
   *   pmstate - Identifies the new PM state
   *
   * Returned Value:
   *   0 (OK) means the event was successfully processed and that the driver
   *   is prepared for the PM state change.  Non-zero means that the driver
   *   is not prepared to perform the tasks needed achieve this power setting
   *   and will cause the state change to be aborted.  NOTE:  The prepare
   *   method will also be recalled when reverting from lower back to higher
   *   power consumption modes (say because another driver refused a lower
   *   power state change).  Drivers are not permitted to return non-zero
   *   values when reverting back to higher power consumption modes!
   *
   **************************************************************************/

  int (*prepare)(FAR struct pm_callback_s *cb, enum pm_state_e pmstate);

  /**************************************************************************
   * Name: notify
   *
   * Description:
   *   Notify the driver of new power state.  This callback is called after
   *   all drivers have had the opportunity to prepare for the new power
   *   state.
   *
   * Input Parameters:
   *   cb      - Returned to the driver.  The driver version of the callback
   *             strucure may include additional, driver-specific state
   *             data at the end of the structure.
   *   pmstate - Identifies the new PM state
   *
   * Returned Value:
   *   None.  The driver already agreed to transition to the low power
   *   consumption state when when it returned OK to the prepare() call.
   *   At that time it should have made all preprations necessary to enter
   *   the new state.  Now the driver must make the state transition.
   *
   **************************************************************************/

  void (*notify)(FAR struct pm_callback_s *cb, enum pm_state_e pmstate);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: pm_initialize
 *
 * Description:
 *   This function is called by MCU-specific logic at power-on reset in
 *   order to provide one-time initialization the power management subystem.
 *   This function must be called *very* early in the intialization sequence
 *   *before* any other device drivers are initialized (since they may
 *   attempt to register with the power management subsystem).
 *
 * Input parameters:
 *   None.
 *
 * Returned value:
 *    None.
 *
 ****************************************************************************/

EXTERN void pm_initialize(void);

/****************************************************************************
 * Name: pm_register
 *
 * Description:
 *   This function is called by a device driver in order to register to
 *   receive power management event callbacks.
 *
 * Input parameters:
 *   callbacks - An instance of struct pm_callback_s providing the driver
 *               callback functions.
 *
 * Returned value:
 *    Zero (OK) on success; otherwise a negater errno value is returned.
 *
 ****************************************************************************/

EXTERN int pm_register(FAR struct pm_callback_s *callbacks);

/****************************************************************************
 * Name: pm_activity
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle).  This increment an activty
 *   count and/or will restart a idle timer and prevent entering reduced
 *   power states.
 *
 * Input Parameters:
 *   priority - Activity priority, range 0-9.  Larger values correspond to
 *     higher priorities.  Higher priority activity can prevent the system
 *     from entering reduced power states for a longer period of time.
 *
 *     As an example, a button press might be higher priority activity because
 *     it means that the user is actively interacting with the device.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler (this is the ONLY
 *   PM function that may be called from an interrupt handler!).
 *
 ****************************************************************************/

EXTERN void pm_activity(int priority);

/****************************************************************************
 * Name: pm_checkstate
 *
 * Description:
 *   This function is called from the MCU-specific IDLE loop to monitor the
 *   the power management conditions.  This function returns the "recommended"
 *   power management state based on the PM configuration and activity
 *   reported in the last sampling periods.  The power management state is
 *   not automatically changed, however.  The IDLE loop must call
 *   pm_changestate() in order to make the state change.
 *
 *   These two steps are separated because the plaform-specific IDLE loop may
 *   have additional situational information that is not available to the
 *   the PM sub-system.  For example, the IDLE loop may know that the
 *   battery charge level is very low and may force lower power states
 *   even if there is activity.
 *
 *   NOTE: That these two steps are separated in time and, hence, the IDLE
 *   loop could be suspended for a long period of time between calling
 *   pm_checkstate() and pm_changestate().  The IDLE loop may need to make
 *   these calls atomic by either disabling interrupts until the state change
 *   is completed.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The recommended power management state.
 *
 ****************************************************************************/

EXTERN enum pm_state_e pm_checkstate(void);

/****************************************************************************
 * Name: pm_changestate
 *
 * Description:
 *   This function is used to platform-specific power managmeent logic.  It
 *   will announce the power management power management state change to all
 *   drivers that have registered for power management event callbacks.
 *
 * Input Parameters:
 *   newstate - Identifies the new PM state
 *
 * Returned Value:
 *   0 (OK) means that the callback function for all registered drivers
 *   returned OK (meaning that they accept the state change).  Non-zero
 *   means that one of the drivers refused the state change.  In this case,
 *   the system will revert to the preceding state.
 *
 * Assumptions:
 *   It is assumed that interrupts are disabled when this function is
 *   called.  This function is probably called from the IDLE loop... the
 *   lowest priority task in the system.  Changing driver power management
 *   states may result in renewed system activity and, as a result, can
 *   suspend the IDLE thread before it completes the entire state change
 *   unless interrupts are disabled throughout the state change.
 *
 ****************************************************************************/

EXTERN int pm_changestate(enum pm_state_e newstate);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Stubs
 ****************************************************************************/

#else /* CONFIG_PM */

/* Stubbed out versions of all of PM interface functions that may be used to
 * avoid so much conditional compilation in driver code when PM is disabled:
 */

#  define pm_initialize()
#  define pm_register(cb)       (0)
#  define pm_activity(prio)
#  define pm_checkstate()       (0)
#  define pm_changestate(state)

#endif /* CONFIG_PM */
#endif /* __INCLUDE_NUTTX_POWER_PM_H */
