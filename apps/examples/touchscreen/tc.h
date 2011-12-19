/****************************************************************************
 * examples/touchscreen/tc.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __APPS_EXAMPLES_TOUCHSCREEN_TC_H
#define __APPS_EXAMPLES_TOUCHSCREEN_TC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>


/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_EXAMPLES_TOUCHSCREEN_BUILTIN - Build the touchscreen test as
 *   an NSH built-in function.  Default: Built as a standalone problem
 * CONFIG_EXAMPLES_TOUCHSCREEN_MINOR - The minor device number.  Minor=N
 *   correspnds to touchscreen device /dev/input0.  Note this value must
 *   with CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH.  Default 0.
 * CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH - The path to the touchscreen
 *   device.  This must be consistent with CONFIG_EXAMPLES_TOUCHSCREEN_MINOR.
 *   Default: "/dev/input0"
 * CONFIG_EXAMPLES_TOUCHSCREEN_NSAMPLES - If CONFIG_EXAMPLES_TOUCHSCREEN_BUILTIN
 *   is defined, then the number of samples is provided on the command line
 *   and this value is ignored.  Otherwise, this number of samples is
 *   collected and the program terminates.  Default:  Samples are collected
 *   indefinitely.
 */

#ifndef CONFIG_INPUT
#  error "Input device support is not enabled (CONFIG_INPUT)"
#endif

#ifndef CONFIG_EXAMPLES_TOUCHSCREEN_MINOR
#  undef  CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH
#  define CONFIG_EXAMPLES_TOUCHSCREEN_MINOR 0
#  define CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH "/dev/input0"
#endif

#ifndef CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH
#  undef  CONFIG_EXAMPLES_TOUCHSCREEN_MINOR
#  define CONFIG_EXAMPLES_TOUCHSCREEN_MINOR 0
#  define CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH "/dev/input0"
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lib_rawprintf(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush() fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lib_rawprintf
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
 * Name: arch_tcinitialize()
 *
 * Description:
 *   Perform architecuture-specific initialization of the touchscreen
 *   hardware.  This interface must be provided by all configurations
 *   using apps/examples/touchscreen
 *
 ****************************************************************************/

int arch_tcinitialize(int minor);

/****************************************************************************
 * Name: arch_tcuninitialize()
 *
 * Description:
 *   Perform architecuture-specific un-initialization of the touchscreen
 *   hardware.  This interface must be provided by all configurations
 *   using apps/examples/touchscreen
 *
 ****************************************************************************/

void arch_tcuninitialize(void);

#endif /* __APPS_EXAMPLES_TOUCHSCREEN_TC_H */
