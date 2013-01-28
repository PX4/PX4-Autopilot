/****************************************************************************
 * arch/z80/src/common/up_initialize.c
 *
 *   Copyright (C) 2007-2009, 2012-2013 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mm.h>
#include <arch/board/board.h>

#include "chip/switch.h"
#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Define to enable timing loop calibration */

#undef CONFIG_ARCH_CALIBRATION

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_calibratedelay
 *
 * Description:
 *   Delay loops are provided for short timing loops.  This function, if
 *   enabled, will just wait for 100 seconds.  Using a stopwatch, you can
 *   can then determine if the timing loops are properly calibrated.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_CALIBRATION) & defined(CONFIG_DEBUG)
static void up_calibratedelay(void)
{
  int i;
  lldbg("Beginning 100s delay\n");
  for (i = 0; i < 100; i++)
    {
      up_mdelay(1000);
    }
  lldbg("End 100s delay\n");
}
#else
# define up_calibratedelay()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initialize
 *
 * Description:
 *   up_initialize will be called once during OS initialization after the
 *   basic OS services have been initialized.  The architecture specific
 *   details of initializing the OS will be handled here.  Such things as
 *   setting up interrupt service routines, starting the clock, and
 *   registering device drivers are some of the things that are different
 *   for each processor and hardware platform.
 *
 *   up_initialize is called after the OS initialized but before the user
 *   initialization logic has been started and before the libraries have
 *   been initialized.  OS services and driver services are available.
 *
 ****************************************************************************/

void up_initialize(void)
{
  /* Initialize global variables */

  INIT_IRQCONTEXT();

  /* Calibrate the timing loop */

  up_calibratedelay();

  /* Add extra memory fragments to the memory manager */

#if CONFIG_MM_REGIONS > 1
  up_addregion();
#endif

  /* Initialize the interrupt subsystem */

  up_irqinitialize();

  /* Initialize the system timer interrupt */

#if !defined(CONFIG_SUPPRESS_INTERRUPTS) && !defined(CONFIG_SUPPRESS_TIMER_INTS)
  up_timerinit();
#endif

  /* Initialize the CPU for those that use it (only for the Z180).  This
   * needs to be done before any tasks are created).
   */

#if CONFIG_ADDRENV
  (void)up_mmuinit();
#endif

  /* Register devices */

#if CONFIG_NFILE_DESCRIPTORS > 0
  devnull_register();   /* Standard /dev/null */
#endif

  /* Initialize the serial device driver */

#ifdef USE_SERIALDRIVER
  up_serialinit();
#endif

  /* Initialize the console device driver (if it is other than the standard
   * serial driver).
   */

#if defined(CONFIG_DEV_LOWCONSOLE)
  lowconsole_init();
#elif defined(CONFIG_RAMLOG_CONSOLE)
  ramlog_consoleinit();
#endif

  /* Initialize the netwok */

  up_netinitialize();
  up_ledon(LED_IRQSENABLED);
}
