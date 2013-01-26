/****************************************************************************
 * sched_setuppthreadfiles.c
 *
 *   Copyright (C) 2007, 2009, 2012 Gregory Nutt. All rights reserved.
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

#include <sched.h>

#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>
#include <nuttx/lib.h>

#include "os_internal.h"

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_setuppthreadfiles
 *
 * Description:
 *   Configure a newly allocated TCB so that it will inherit file
 *   descriptors and streams from the parent pthread.
 *
 * Parameters:
 *   tcb - tcb of the new task.
 *
 * Return Value:
 *   OK (if an error were returned, it would need to be a non-negated
 *   errno value).
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_setuppthreadfiles(FAR _TCB *tcb)
{
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;

#if CONFIG_NSOCKET_DESCRIPTORS > 0
  /* The child thread inherits the parent socket descriptors */

  tcb->sockets = rtcb->sockets;
  net_addreflist(tcb->sockets);

#endif /* CONFIG_NSOCKET_DESCRIPTORS */

#if CONFIG_NFILE_STREAMS > 0
  /* The child thread inherits the parent streams */

  tcb->streams = rtcb->streams;
  lib_addreflist(tcb->streams);

#endif /* CONFIG_NFILE_STREAMS */
  return OK;
}

#endif /* CONFIG_NFILE_DESCRIPTORS || CONFIG_NSOCKET_DESCRIPTORS */
