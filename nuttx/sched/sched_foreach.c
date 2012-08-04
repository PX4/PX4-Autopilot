/************************************************************************
 * sched/sched_foreach.c
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

#include <sched.h>
#include "os_internal.h"

/************************************************************************
 * Global Functions
 ************************************************************************/

/************************************************************************
 * Name: sched_foreach
 *
 * Description:
 *   Enumerate over each task and provide the TCB of each
 *   task to a user callback functions.  Interrupts will be
 *   disabled throughout this enumeration!
 *
 * Parameters:
 *   handler - The function to be called with the TCB of
 *     each task
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ************************************************************************/

void sched_foreach(sched_foreach_t handler, FAR void *arg)
{
  irqstate_t flags = irqsave();
  int ndx;

  /* Verify that the PID is within range */

  for (ndx = 0; ndx < CONFIG_MAX_TASKS; ndx++)
    {
       if (g_pidhash[ndx].tcb)
         {
           handler(g_pidhash[ndx].tcb, arg);
         }
    }

  irqrestore(flags);
}


