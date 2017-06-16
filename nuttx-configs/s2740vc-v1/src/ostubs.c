/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: David Sidrane <david_s5@nscdg.com>
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile dq_queue_t g_readytorun;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
void timer_init(void);
int __wrap_up_svcall(int irq, FAR void *context);

/****************************************************************************
 * Name: os_start
 *
 * Description:
 *   This function hijacks the entry point of the OS. Normally called by the
 *   statup code for a given architecture
 *
 ****************************************************************************/

void os_start(void)
{
  /* Initialize the timer software subsystem */

  timer_init();

  /* Initialize the interrupt subsystem */

  up_irqinitialize();

  /* Initialize the OS's timer subsystem */
#if !defined(CONFIG_SUPPRESS_INTERRUPTS) && !defined(CONFIG_SUPPRESS_TIMER_INTS) && \
    !defined(CONFIG_SYSTEMTICK_EXTCLK)
  up_timer_initialize();
#endif

  /* Keep the compiler happy for a no return function */

  while (1)
    {
      main(0, 0);
    }
}

/****************************************************************************
 * Name: malloc
 *
 * Description:
 *   This function hijacks the OS's malloc and provides no allocation
 *
 ****************************************************************************/

FAR void *malloc(size_t size)
{
  return NULL;
}

/****************************************************************************
 * Name: malloc
 *
 * Description:
 *   This function hijacks the systems exit
 *
 ****************************************************************************/
void exit(int status)
{
  while (1);
}

/****************************************************************************
 * Name: sched_ufree
 *
 * Description:
 *   This function hijacks the systems sched_ufree that my be called during
 *   exception processing.
 *
 ****************************************************************************/

void sched_ufree(FAR void *address)
{

}

/****************************************************************************
 * Name: up_svcall
 *
 * Description:
 *   This function hijacks by the way of a compile time wrapper the systems
 *   up_svcall
 *
 ****************************************************************************/

int __wrap_up_svcall(int irq, FAR void *context)
{
  return 0;
}
