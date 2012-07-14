/****************************************************************************
 * sched/wd_delete.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#include <wdog.h>
#include <queue.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "wd_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wd_delete
 *
 * Description:  
 *   The wd_delete function will deallocate a watchdog by returning it to
 *   the free pool of watchdogs.  The watchdog will be removed from the timer
 *   queue if has been started.
 *
 * Parameters:
 *   wdId - The watchdog ID to delete.  This is actually a pointer to a
 *          watchdog structure.
 *
 * Return Value:
 *   Returns OK or ERROR
 *
 * Assumptions:
 *   The caller has assured that the watchdog is no longer in use.
 *
 ****************************************************************************/

int wd_delete(WDOG_ID wdId)
{
  irqstate_t saved_state;

  /* Verify that a valid watchdog was provided */

  if (!wdId)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* The following steps are atomic... the watchdog must not be active when
   * it is being deallocated.
   */

  saved_state = irqsave();

  /* Check if the watchdog has been started. */

  if (wdId->active)
    {
      wd_cancel(wdId);
    }

  /* Put the watchdog back on the free list */

  sq_addlast((FAR sq_entry_t*)wdId, &g_wdfreelist);
  irqrestore(saved_state);

  /* Return success */

  return OK;
}
