/************************************************************************
 * sched/wd_initialize.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <queue.h>
#include <nuttx/kmalloc.h>

#include "os_internal.h"
#include "wd_internal.h"

/************************************************************************
 * Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Global Variables
 ************************************************************************/

/* The g_wdfreelist data structure is a singly linked list of watchdogs
 * available to the system for delayed function use.
 */

sq_queue_t g_wdfreelist;

/* g_wdpool is a pointer to a list of pre-allocated watchdogs. The number
 * of watchdogs in the pool is a configuration item.
 */

FAR wdog_t *g_wdpool;

/* The g_wdactivelist data structure is a singly linked list ordered by
 * watchdog expiration time. When watchdog timers expire,the functions on
 * this linked list are removed and the function is called.
 */

sq_queue_t g_wdactivelist;

/************************************************************************
 * Private Variables
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: wd_initialize
 *
 * Description:
 * This function initalized the watchdog data structures
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   This function must be called early in the initialization sequence
 *   before the timer interrupt is attached and before any watchdog
 *   services are used.
 *
 ************************************************************************/

void wd_initialize(void)
{
  /* Initialize the free watchdog list */

  sq_init(&g_wdfreelist);

  /* The g_wdfreelist must be loaded at initialization time to hold the
   * configured number of watchdogs.
   */

  g_wdpool = (FAR wdog_t*)kmalloc(sizeof(wdog_t) * CONFIG_PREALLOC_WDOGS);
  if (g_wdpool)
    {
      FAR wdog_t *wdog = g_wdpool;
      int i;

      for (i = 0; i < CONFIG_PREALLOC_WDOGS; i++)
        {
          sq_addlast((FAR sq_entry_t*)wdog++, &g_wdfreelist);
        }
    }

  /* The g_wdactivelist queue must be reset at initialization time. */

  sq_init(&g_wdactivelist);
}
