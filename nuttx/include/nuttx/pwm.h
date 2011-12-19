/****************************************************************************
 * include/nuttx/pwm.h
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_PWM_H
#define __INCLUDE_NUTTX_PWM_H

/* For the purposes of this driver, a PWM device is any devices and generates
 * periodic outputs of controlled frequency and pulse width.  Such is device
 * might be use, for example, to perform pulse-width modulated output or
 * frequency/pulse-count modulated output (such as might be needed to control
 * a stepper motor.
 *
 * The PWM driver is split into two parts:
 *
 * 1) An "upper half", generic driver that provides the comman PWM interface
 *    to application level code, and
 * 2) An "lower half" platform-specific driver that implements the low-level
 *    timer controls to implement the PWM functionality.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <fixedmath.h>

#include <nuttx/ioctl.h>

#ifdef CONFIG_PWM

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* The PWM module uses a standard character driver framework.  However, since
 * the PWM driver is a devices control interface and not a data transfer
 * interface, the majority of the functionality is implemented in driver
 * ioctl calls.  The PWM ioctal commands are lised below:
 *
 * PWMIOC_SETCHARACTERISTICS - Set the characteristics of the next pulsed
 *   output.  This command will neither start nor stop the pulsed output.
 *   It will either setup the configuration that will be used when the
 *   output is started; or it will change the characteristics of the pulsed
 *   output on the fly if the timer is already started.
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
 *   command must have previously been sent.
 *
 *   ioctl argument:  None
 *
 * PWMIOC_STOP - Stop the pulsed output.
 *
 *   ioctl argument:  None
 *
 * PWMIOC_GETPULSECOUNT - Return the number of pulses generated.
 *
 *   ioctl argument:  A pointer to a pwm_count_t variable that will be used to
 *     receive the pulse count
 */

#define PWMIOC_SETCHARACTERISTICS _PWMIOC(1)
#define PWMIOC_GETCHARACTERISTICS _PWMIOC(2)
#define PWMIOC_START              _PWMIOC(3)
#define PWMIOC_STOP               _PWMIOC(4)
#define PWMIOC_GETPULSECOUNT      _PWMIOC(5)

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This structure describes the characteristics of the pulsed output */

struct pwm_info_s
{
  uint32_t frequency; /* Frequency of the pulse train */
  ub16_t   duty;      /* Duty of the pulse train, "1" to "0" duration */
};

/* This type is used to return pulse counts */

#ifdef CONFIG_HAVE_LONG_LONG
typedef uint16_t pwm_count_t;
#else
struct pwm_count_s
{
  uint32_t ms;  /* Most significant 32-bits of the 64-count */
  uint32_t ls;  /* Least significant 32-bits of the 64-count */
};
typedef struct pwm_count_s pwm_count_t;
#endif

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
   * stop pulsed output, free any resources, disable the timer hardware, and
   * put the system into the lowest possible power usage state
   */

  CODE int (*shutdown)(FAR struct pwm_lowerhalf_s *dev);

  /* (Re-)initialize the timer resources and start the pulsed output */

  CODE int (*start)(FAR struct pwm_lowerhalf_s *dev, FAR const struct pwm_info_s *info);

  /* Stop the pulsed output and reset the timer resources*/

  CODE int (*stop)(FAR struct pwm_lowerhalf_s *dev);

  /* Get the number of pulses generated */

  CODE int (*pulsecount)(FAR struct pwm_lowerhalf_s *dev, FAR pwm_count_t *count);

  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg);
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
   * the pointer to the PWM callback structgure.
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

int pwm_register(FAR const char *path, FAR struct pwm_lowerhalf_s *dev);

/****************************************************************************
 * Platform-Independent "Lower-Half" PWM Driver Interfaces
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_PWM */
#endif /* __INCLUDE_NUTTX_PWM_H */
