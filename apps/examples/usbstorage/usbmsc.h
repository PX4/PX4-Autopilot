/****************************************************************************
 * examples/usbstorage/usbmsc.h
 *
 *   Copyright (C) 2008-2009, 2012 Gregory Nutt. All rights reserved.
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

#ifndef __EXAMPLES_USBSTORAGE_USBMSC_H
#define __EXAMPLES_USBSTORAGE_USBMSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_EXAMPLES_USBMSC_NLUNS
#  define CONFIG_EXAMPLES_USBMSC_NLUNS 1
#endif

#ifndef CONFIG_EXAMPLES_USBMSC_DEVMINOR1
#  define CONFIG_EXAMPLES_USBMSC_DEVMINOR1 0
#endif

#ifndef CONFIG_EXAMPLES_USBMSC_DEVPATH1
#  define CONFIG_EXAMPLES_USBMSC_DEVPATH1 "/dev/mmcsd0"
#endif

#if CONFIG_EXAMPLES_USBMSC_NLUNS > 1
#  ifndef CONFIG_EXAMPLES_USBMSC_DEVMINOR2
#    error "CONFIG_EXAMPLES_USBMSC_DEVMINOR2 for LUN=2"
#  endif
#  ifndef CONFIG_EXAMPLES_USBMSC_DEVPATH2
#    error "CONFIG_EXAMPLES_USBMSC_DEVPATH2 for LUN=2"
#  endif
#  if CONFIG_EXAMPLES_USBMSC_NLUNS > 2
#    ifndef CONFIG_EXAMPLES_USBMSC_DEVMINOR3
#      error "CONFIG_EXAMPLES_USBMSC_DEVMINOR2 for LUN=3"
#    endif
#    ifndef CONFIG_EXAMPLES_USBMSC_DEVPATH2
#      error "CONFIG_EXAMPLES_USBMSC_DEVPATH2 for LUN=3"
#    endif
#    if CONFIG_EXAMPLES_USBMSC_NLUNS > 3
#      error "CONFIG_EXAMPLES_USBMSC_NLUNS must be {1,2,3}"
#    endif
#  endif
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lib_lowprintf(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush() fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lib_lowprintf
#    define msgflush()
#  else
#    define message printf
#    define msgflush() fflush(stdout)
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* All global variables used by this example are packed into a structure in
 * order to avoid name collisions.
 */

#if defined(CONFIG_EXAMPLES_USBMSC_BUILTIN) || defined(CONFIG_EXAMPLES_USBMSC_DEBUGMM)
struct usbmsc_state_s
{
  /* This is the handle that references to this particular USB storage driver
   * instance.  It is only needed if the USB mass storage device example is
   * built using CONFIG_EXAMPLES_USBMSC_BUILTIN.  In this case, the value
   * of the driver handle must be remembered between the 'msconn' and 'msdis'
   * commands.
   */

#ifdef CONFIG_EXAMPLES_USBMSC_BUILTIN
  FAR void *mshandle;
#endif

  /* Heap usage samples.  These are useful for checking USB storage memory
   * usage and for tracking down memoryh leaks.
   */

#ifdef CONFIG_EXAMPLES_USBMSC_DEBUGMM
  struct mallinfo mmstart;    /* Memory usage before the connection */
  struct mallinfo mmprevious; /* The last memory usage sample */
  struct mallinfo mmcurrent;  /* The current memory usage sample */
#endif
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* All global variables used by this example are packed into a structure in
 * order to avoid name collisions.
 */

#if defined(CONFIG_EXAMPLES_USBMSC_BUILTIN) || defined(CONFIG_EXAMPLES_USBMSC_DEBUGMM)
extern struct usbmsc_state_s g_usbmsc;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbmsc_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization.  This function must
 *   configure the block device to export via USB.  This function must be
 *   provided by architecture-specific logic in order to use this example.
 *
 ****************************************************************************/

extern int usbmsc_archinitialize(void);

#endif /* __EXAMPLES_USBSTORAGE_USBMSC_H */
