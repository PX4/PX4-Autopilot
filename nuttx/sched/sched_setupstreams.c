/****************************************************************************
 * sched_setupstreams.c
 *
 *   Copyright (C) 2007-2008, 2010-2012 Gregory Nutt. All rights reserved.
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
#include <fcntl.h>

#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>
#include <nuttx/lib.h>

/* Make sure that there are file or socket descriptors in the system and
 * that some number of streams have been configured.
 */

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0
#if CONFIG_NFILE_STREAMS > 0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_setupstreams
 *
 * Description:
 *   Setup streams data structures that may be used for standard C buffered
 *   I/O with underlying socket or file desciptors
 *
 ****************************************************************************/

int sched_setupstreams(FAR _TCB *tcb)
{
  DEBUGASSERT(tcb && tcb->group);

  /* Initialize file streams for the task group */

  lib_streaminit(&tcb->group->tg_streamlist);

  /* fdopen to get the stdin, stdout and stderr streams. The following logic
   * depends on the fact that the library layer will allocate FILEs in order.
   *
   * fd = 0 is stdin  (read-only)
   * fd = 1 is stdout (write-only, append)
   * fd = 2 is stderr (write-only, append)
   */

  (void)fs_fdopen(0, O_RDONLY,       tcb);
  (void)fs_fdopen(1, O_WROK|O_CREAT, tcb);
  (void)fs_fdopen(2, O_WROK|O_CREAT, tcb);

  return OK;
}

#endif /* CONFIG_NFILE_STREAMS > 0 */
#endif /* CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0*/
