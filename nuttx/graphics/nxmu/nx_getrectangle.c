/****************************************************************************
 * graphics/nxmu/nx_getrectangle.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <mqueue.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>

#include "nxfe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_getrectangle
 *
 * Description:
 *  Get the raw contents of graphic memory within a rectangular region. NOTE:
 *  Since raw graphic memory is returned, the returned memory content may be
 *  the memory of windows above this one and may not necessarily belong to
 *  this window unless you assure that this is the top window.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be copied
 *   plane - Specifies the color plane to get from.
 *   dest - The location to copy the memory region
 *   deststride - The width, in bytes, the the dest memory
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_getrectangle(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                    unsigned int plane, FAR uint8_t *dest,
                    unsigned int deststride)
{
  FAR struct nxbe_window_s        *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_getrectangle_s  outmsg;
  int ret;
  sem_t sem_done;
  
#ifdef CONFIG_DEBUG
  if (!hwnd || !rect || !dest)
    {
      gvdbg("Invalid parameters\n");
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Format the fill command */

  outmsg.msgid      = NX_SVRMSG_GETRECTANGLE;
  outmsg.wnd        = wnd;
  outmsg.plane      = plane;
  outmsg.dest       = dest;
  outmsg.deststride = deststride;

  nxgl_rectcopy(&outmsg.rect, rect);

  /* Create a semaphore for tracking command completion */

  outmsg.sem_done = &sem_done;
  ret = sem_init(&sem_done, 0, 0);
  
  if (ret != OK)
    {
      gdbg("sem_init failed: %d\n", errno);
      return ret;
    }
  
  /* Forward the fill command to the server */

  ret = nxmu_sendwindow(wnd, &outmsg, sizeof(struct nxsvrmsg_getrectangle_s));
  
  /* Wait that the command is completed, so that caller can release the buffer. */
  
  if (ret == OK)
    {
      ret = sem_wait(&sem_done);
    }
  
  /* Destroy the semaphore and return. */
  
  sem_destroy(&sem_done);
  
  return ret;
}
