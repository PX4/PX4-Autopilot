/****************************************************************************
 * include/nuttx/pwm.h
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

#ifndef __INCLUDE_NUTTX_PWM_H
#define __INCLUDE_NUTTX_PWM_H

/* For the purposes of this driver, a PWM device is any device that generates
 * periodic output pulses s of controlled frequency and pulse width.  Such a
 * device might be used, for example, to perform pulse-width modulated output or
 * frequency/pulse-count modulated output (such as might be needed to control
 * a stepper motor).
 *
 * The PWM driver is split into two parts:
 *
 * 1) An "upper half", generic driver that provides the comman PWM interface
 *    to application level code, and
 * 2) A "lower half", platform-specific driver that implements the low-level
 *    timer controls to implement the PWM functionality.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <fixedmath.h>

#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_PWM

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_PWM - Enables because PWM driver support
 * CONFIG_PWM_PULSECOUNT - Some hardware will support generation of a fixed
 *   number of pulses.  This might be used, for example to support a stepper
 *   motor.  If the hardware will support a fixed pulse count, then this
 *   configuration should be set to enable the capability.
 * CONFIG_DEBUG_PWM - If enabled (with CONFIG_DEBUG and, optionally,
 *   CONFIG_DEBUG_VERBOSE), this will generate output that can be use dto
 *   debug the PWM driver.
 */
 
/* IOCTL Commands ***********************************************************/
/* The PWM module uses a standard character driver framework.  However, since
 * the PWM driver is a device control interface and not a data transfer
 * interface, the majority of the functionality is implemented in driver
 * ioctl calls.  The PWM ioctl commands are lised below:
 *
 * PWMIOC_SETCHARACTERISTICS - Set the characteristics of the next pulsed
 *   output.  This command will neither start nor stop the pulsed output.
 *   It will either setup the configuration that will be used when the
 *   output is started; or it will change the characteristics of the pulsed
 *   output on the fly if the timer is already started.  This command will
 *   set the PWM characteristics and return immediately.
 *
 *   ioctl argument:  A read-only reference to struct pwm_info_s that provides
 *   the characteristics of the pulsed output.
 *
 * PWMIOC_GETCHARACTERISTICS - Get the currently selected characteristics of
 *   the pulsed output (independent of whether the output is start or stopped).
 *
 *   ioctl argument:  A reference to struct pwm_info_s to recevie the
 *   characteristics of the pulsed output.
 *
 * PWMIOC_START - Start the pulsed output.  The PWMIOC_SETCHARACTERISTICS
 *   command must have previously been sent. If CONFIG_PWM_PULSECOUNT is
 *   defined and the pulse count was configured to a non-zero value, then
 *   this ioctl call will, by default, block until the programmed pulse count
 *   completes.  That default blocking behavior can be overridden by using
 *   the O_NONBLOCK flag when the PWM driver is opened.
 *
 *   ioctl argument:  None
 *
 * PWMIOC_STOP - Stop the pulsed output.  This command will stop the PWM
 *   and return immediately.
 *
 *   ioctl argument:  None
 */

#define PWMIOC_SETCHARACTERISTICS _PWMIOC(1)
#define PWMIOC_GETCHARACTERISTICS _PWMIOC(2)
#define PWMIOC_START              _PWMIOC(3)
#define PWMIOC_STOP               _PWMIOC(4)

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This structure describes the characteristics of the pulsed output */

struct pwm_info_s
{
  uint32_t frequency; /* Frequency of the pulse train */
  ub16_t   duty;      /* Duty of the pulse train, "1"-to-"0" duration.
                       * Maximum: 65535/65536 (0x0000ffff)
                       * Minimum:     1/65536 (0x00000001) */
#ifdef CONFIG_PWM_PULSECOUNT
  uint32_t count;     /* The number of pulse to generate.  0 means to
                       * generate an indefinite number of pulses  */
#endif
};

/* This structure is a set a callback functions used to call from the upper-
 * half, generic PWM driver into lower-half, platform-specific logic that
 * supports the low-level timer outputs.
 */

struct pwm_lowerhalf_s;
struct pwm_ops_s
{
  /* This method is called when the driver is opened.  The lower half driver
   * should configure and initialize the device so that it is ready for use.
   * It should not, however, output pulses until the start method is called.
   */

  CODE int (*setup)(FAR struct pwm_lowerhalf_s *dev);

  /* This method is called when the driver is closed.  The lower half driver
   * should stop pulsed output, free any resources, disable the timer hardware, and
   * put the system into the lowest possible power usage state
   */

  CODE int (*shutdown)(FAR struct pwm_lowerhalf_s *dev);

  /* (Re-)initialize the timer resources and start the pulsed output. The
   * start method should return an error if it cannot start the timer with
   * the given parameter (frequency, duty, or optionally pulse count)
   */

#ifdef CONFIG_PWM_PULSECOUNT
  CODE int (*start)(FAR struct pwm_lowerhalf_s *dev,
                    FAR const struct pwm_info_s *info,
                    FAR void *handle);
#else
  CODE int (*start)(FAR struct pwm_lowerhalf_s *dev,
                    FAR const struct pwm_info_s *info);
#endif

  /* Stop the pulsed output and reset the timer resources*/

  CODE int (*stop)(FAR struct pwm_lowerhalf_s *dev);

  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct pwm_lowerhalf_s *dev,
                    int cmd, unsigned long arg);
};

/* This structure is the generic form of state structure used by lower half
 * timer driver.  This state structure is passed to the pwm driver when the
 * driver is initialized.  Then, on subsequent callbacks into the lower half
 * timer logic, this structure is provided so that the timer logic can
 * maintain state information.
 *
 * Normally that timer logic will have its own, custom state structure
 * that is simply cast to struct pwm_lowerhalf_s.  In order to perform such casts,
 * the initial fields of the custom state structure match the initial fields
 * of the following generic PWM state structure.
 */

struct pwm_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the PWM
   * callback structure:
   */

  FAR const struct pwm_ops_s *ops;

  /* The custom timer state structure may include additional fields after
   * the pointer to the PWM callback structure.
   */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * "Upper-Half" PWM Driver Interfaces
 ****************************************************************************/
/****************************************************************************
 * Name: pwm_register
 *
 * Description:
 *   This function binds an instance of a "lower half" timer driver with the
 *   "upper half" PWM device and registers that device so that can be used
 *   by application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   reset state (as if the shutdown() method had already been called).
 *
 * Input parameters:
 *   path - The full path to the driver to be registers in the NuttX pseudo-
 *     filesystem.  The recommended convention is to name all PWM drivers
 *     as "/dev/pwm0", "/dev/pwm1", etc.  where the driver path differs only
 *     in the "minor" number at the end of the device name.
 *   dev - A pointer to an instance of lower half timer driver.  This instance
 *     is bound to the PWM driver and must persists as long as the driver
 *     persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

EXTERN int pwm_register(FAR const char *path, FAR struct pwm_lowerhalf_s *dev);

/****************************************************************************
 * Name: pwm_expired
 *
 * Description:
 *   If CONFIG_PWM_PULSECOUNT is defined and the pulse count was configured
 *   to a non-zero value, then the "upper half" driver will wait for the
 *   pulse count to expire.  The sequence of expected events is as follows:
 *
 *   1. The upper half driver calls the start method, providing the lower
 *      half driver with the pulse train characteristics.  If a fixed
 *      number of pulses is required, the 'count' value will be nonzero.
 *   2. The lower half driver's start() methoc must verify that it can
 *      support the request pulse train (frequency, duty, AND pulse count).
 *      It it cannot, it should return an error.  If the pulse count is
 *      non-zero, it should set up the hardware for that number of pulses
 *      and return success.  NOTE:  That is CONFIG_PWM_PULSECOUNT is
 *      defined, the start() method receives an additional parameter
 *      that must be used in this callback.
 *   3. When the start() method returns success, the upper half driver
 *      will "sleep" until the pwm_expired method is called.
 *   4. When the lower half detects that the pulse count has expired
 *      (probably through an interrupt), it must call the pwm_expired
 *      interface using the handle that was previously passed to the
 *      start() method
 *
 * Input parameters:
 *   handle - This is the handle that was provided to the lower-half
 *     start() method.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM_PULSECOUNT
EXTERN void pwm_expired(FAR void *handle);
#endif

/****************************************************************************
 * Platform-Independent "Lower-Half" PWM Driver Interfaces
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_PWM */
#endif /* __INCLUDE_NUTTX_PWM_H */
