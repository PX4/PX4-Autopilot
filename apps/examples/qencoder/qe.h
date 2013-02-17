/****************************************************************************
 * examples/examples/qe/qe.h
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

#ifndef __APPS_EXAMPLES_QENCODER_QE_H
#define __APPS_EXAMPLES_QENCODER_QE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_NSH_BUILTIN_APPS - Build the QE test as an NSH built-in function.
 *  Default: Built as a standalone problem
 * CONFIG_EXAMPLES_QENCODER_DEVPATH - The path to the QE device. Default:
 *  /dev/qe0
 * CONFIG_EXAMPLES_QENCODER_NSAMPLES - If CONFIG_NSH_BUILTIN_APPS
 *   is defined, then the number of samples is provided on the command line
 *   and this value is ignored.  Otherwise, this number of samples is
 *   collected and the program terminates.  Default:  Samples are collected
 *   indefinitely.
 * CONFIG_EXAMPLES_QENCODER_DELAY - This value provides the delay (in
 *   milliseonds) between each sample.  If CONFIG_NSH_BUILTIN_APPS
 *   is defined, then this value is the default delay if no other delay is
 *   provided on the command line.  Default:  100 milliseconds
 */

#ifndef CONFIG_QENCODER
#  error "QE device support is not enabled (CONFIG_QENCODER)"
#endif

#ifndef CONFIG_EXAMPLES_QENCODER_DEVPATH
#  define CONFIG_EXAMPLES_QENCODER_DEVPATH "/dev/qe0"
#endif

#ifndef CONFIG_EXAMPLES_QENCODER_DELAY
#  define CONFIG_EXAMPLES_QENCODER_DELAY 100
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

#ifdef CONFIG_NSH_BUILTIN_APPS
struct qe_example_s
{
  bool         initialized; /* True: QE devices have been initialized */
  bool         reset;       /* True: set the count back to zero */
  FAR char    *devpath;     /* Path to the QE device */
  unsigned int nloops;      /* Collect this number of samples */
  unsigned int delay;       /* Delay this number of seconds between samples */
};
#endif

/****************************************************************************
 * Public Variables
 ****************************************************************************/

#ifdef CONFIG_NSH_BUILTIN_APPS
extern struct qe_example_s g_qeexample;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: qe_devinit()
 *
 * Description:
 *   Perform architecuture-specific initialization of the QE hardware.  This
 *   interface must be provided by all configurations using apps/examples/qe
 *
 ****************************************************************************/

int qe_devinit(void);

#endif /* __APPS_EXAMPLES_QENCODER_QE_H */
