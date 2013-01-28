/****************************************************************************
 * examples/examples/pwm/pwm.h
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

#ifndef __APPS_EXAMPLES_PWM_PWM_H
#define __APPS_EXAMPLES_PWM_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_NSH_BUILTIN_APPS - Build the PWM test as an NSH built-in function.
 *   Default: Not built!  The example can only be used as an NSH built-in
 *   application
 * CONFIG_EXAMPLES_PWM_DEVPATH - The path to the PWM device. Default: /dev/pwm0
 * CONFIG_EXAMPLES_PWM_FREQUENCY - The initial PWM frequency.  Default: 100 Hz
 * CONFIG_EXAMPLES_PWM_DUTYPCT - The initial PWM duty as a percentage.  Default: 50%
 * CONFIG_EXAMPLES_PWM_DURATION - The initial PWM pulse train duration in seconds.
 *   Used only if the current pulse count is zero (pulse count is only supported
 *   if CONFIG_PWM_PULSECOUNT is defined). Default: 5 seconds
 * CONFIG_EXAMPLES_PWM_PULSECOUNT - The initial PWM pulse count.  This option is
 *   only available if CONFIG_PWM_PULSECOUNT is defined. Default: 0 (i.e., use
 *   the duration, not the count).
 */

#ifndef CONFIG_PWM
#  error "PWM device support is not enabled (CONFIG_PWM)"
#endif

#ifndef CONFIG_NSH_BUILTIN_APPS
#  warning "The PWM example only works as an NSH built-in application (CONFIG_NSH_BUILTIN_APPS)"
#endif

#ifndef CONFIG_EXAMPLES_PWM_DEVPATH
#  define CONFIG_EXAMPLES_PWM_DEVPATH "/dev/pwm0"
#endif

#ifndef CONFIG_EXAMPLES_PWM_FREQUENCY
#  define CONFIG_EXAMPLES_PWM_FREQUENCY 100
#endif

#ifndef CONFIG_EXAMPLES_PWM_DUTYPCT
#  define CONFIG_EXAMPLES_PWM_DUTYPCT 50
#endif

#ifndef CONFIG_EXAMPLES_PWM_DURATION
#  define CONFIG_EXAMPLES_PWM_DURATION 5
#endif

#ifndef CONFIG_EXAMPLES_PWM_COUNT
#  define CONFIG_EXAMPLES_PWM_COUNT 0
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) syslog(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush() fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message syslog
#    define msgflush()
#  else
#    define message printf
#    define msgflush() fflush(stdout)
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_devinit()
 *
 * Description:
 *   Perform architecuture-specific initialization of the PWM hardware.  This
 *   interface must be provided by all configurations using apps/examples/pwm
 *
 ****************************************************************************/

int pwm_devinit(void);

#endif /* __APPS_EXAMPLES_PWM_PWM_H */
