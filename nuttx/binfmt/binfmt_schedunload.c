/****************************************************************************
 * binfmt/binfmt_schedunload.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/binfmt/binfmt.h>

#include "binfmt_internal.h"

#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_SCHED_HAVE_PARENT)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

FAR struct binary_s *g_unloadhead;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unload_list_add
 *
 * Description:
 *   If CONFIG_SCHED_HAVE_PARENT is defined then schedul_unload() will
 *   manage instances of struct binary_s allocated with kmalloc.  It
 *   will keep the binary data in a link list and when SIGCHLD is received
 *   (meaning that the task has exit'ed, schedul_unload() will find the
 *   data, unload the module, and free the structure.
 *
 *   This function will add one structure to the linked list
 *
 * Input Parameter:
 *   pid - The task ID of the child task
 *   bin - This structure must have been allocated with kmalloc() and must
 *         persist until the task unloads

 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void unload_list_add(pid_t pid, FAR struct binary_s *bin)
{
  irqstate_t flags;

  /* Save the PID in the structure so that we recover it later */

  bin->pid = pid;

  /* Disable deliver of any signals while we muck with the list.  The graceful
   * way to do this would be block delivery of SIGCHLD would be with
   * sigprocmask.  Here we do it the quick'n'dirty way by just disabling
   * interrupts.
   */

  flags = irqsave();
  bin->flink = g_unloadhead;
  g_unloadhead = bin;
  irqrestore(flags);
}

/****************************************************************************
 * Name: unload_list_remove
 *
 * Description:
 *   If CONFIG_SCHED_HAVE_PARENT is defined then schedul_unload() will
 *   manage instances of struct binary_s allocated with kmalloc.  It
 *   will keep the binary data in a link list and when SIGCHLD is received
 *   (meaning that the task has exit'ed, schedul_unload() will find the
 *   data, unload the module, and free the structure.
 *
 *   This function will remove one structure to the linked list
 *
 * Input Parameter:
 *   pid - The task ID of the child task
 *
 * Returned Value:
 *   On success, the load structure is returned.  NULL is returned on
 *   failure.
 *
 ****************************************************************************/

static FAR struct binary_s *unload_list_remove(pid_t pid)
{
  FAR struct binary_s *curr;
  FAR struct binary_s *prev;

  /* Note the asymmetry.  We do not have to disable interrupts here because
   * the main thread cannot run while we are in the interrupt handler.  Here,
   * it should be sufficient to disable pre-emption so that no other thread
   * can run.
   */

  sched_lock();

  /* Find the structure in the unload list with the matching PID */

  for (prev = NULL, curr = g_unloadhead;
       curr && (curr->pid != pid);
       prev = curr, curr = curr->flink);

  /* Did we find it?  It must be there. Hmmm.. we should probably ASSERT if
   * we do not!
   */

  if (curr)
    {
      /* Was there another entry before this one? */

      if (prev)
        {
          /* Yes.. remove the current entry from after the previous entry */

          prev->flink = curr->flink;
        }
      else
        {
          /* No.. remove the current entry from the head of the list */

          g_unloadhead = curr->flink;
        }

      /* Nullify the forward link ... superstitious */

      curr->flink = NULL;
    }

  sched_unlock();
  return curr;
}

/****************************************************************************
 * Name: unload_callback
 *
 * Description:
 *   If CONFIG_SCHED_HAVE_PARENT is defined, this function may be called to
 *   automatically unload the module when task exits.  It assumes that
 *   bin was allocated with kmalloc() or friends and will also automatically
 *   free the structure with kfree() when the task exists.
 *
 * Input Parameter:
 *   pid - The ID of the task that just exited
 *   arg - A reference to the load structure cast to FAR void *
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void unload_callback(int signo, siginfo_t *info, void *ucontext)
{
  FAR struct binary_s *bin;
  int ret;

  /* Sanity checking */

  if (!info || signo != SIGCHLD)
    {
      blldbg("ERROR:Bad signal callback: signo=%d info=%p\n", signo, callback);
      return;
    }

  /* Get the load information for this pid */

  bin = unload_list_remove(info->si_pid);
  if (!bin)
    {
      blldbg("ERROR: Could not find load info for PID=%d\n", info->si_pid);
      return;
    }

  /* Unload the module */

  ret = unload_module(bin);
  if (ret < 0)
    {
      blldbg("ERROR: unload_module failed: %d\n", get_errno());
    }

  /* Free the load structure */

  kfree(bin);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: schedule_unload
 *
 * Description:
 *   If CONFIG_SCHED_HAVE_PARENT is defined, this function may be called by
 *   the parent of the the newly created task to automatically unload the
 *   module when the task exits.  This assumes that (1) the caller is the
 *   parent of the created task, (2) that bin was allocated with kmalloc()
 *   or friends.  It will also automatically free the structure with kfree()
 *   after unloading the module.
 *
 * Input Parameter:
 *   pid - The task ID of the child task
 *   bin - This structure must have been allocated with kmalloc() and must
 *         persist until the task unloads
 *
 * Returned Value:
 *   This is an end-user function, so it follows the normal convention:
 *   It returns 0 (OK) if the callback was successfully scheduled. On
 *   failure, it returns -1 (ERROR) and sets errno appropriately.
 *
 *   On failures, the 'bin' structure will not be deallocated and the
 *   module not not be unloaded.
 *
 ****************************************************************************/

int schedule_unload(pid_t pid, FAR struct binary_s *bin)
{
  struct sigaction act;
  struct sigaction oact;
  sigset_t sigset;
  irqstate_t flags;
  int errorcode;
  int ret;

  /* Make sure that SIGCHLD is unmasked */

  (void)sigemptyset(&sigset);
  (void)sigaddset(&sigset, SIGCHLD);
  ret = sigprocmask(SIG_UNBLOCK, &sigset, NULL);
  if (ret != OK)
    {
      /* The errno value will get trashed by the following debug output */

      errorcode = get_errno();
      bvdbg("ERROR: sigprocmask failed: %d\n", ret);
      goto errout;
    }

  /* Add the structure to the list.  We want to do this *before* connecting
   * the signal handler.  This does, however, make error recovery more
   * complex if sigaction() fails below because then we have to remove the
   * unload structure for the list in an unexpected context.
   */

  unload_list_add(pid, bin);

  /* Register the SIGCHLD handler */

  act.sa_sigaction = unload_callback;
  act.sa_flags     = SA_SIGINFO;

  (void)sigfillset(&act.sa_mask);
  (void)sigdelset(&act.sa_mask, SIGCHLD);

  ret = sigaction(SIGCHLD, &act, &oact);
  if (ret != OK)
    {
      /* The errno value will get trashed by the following debug output */

      errorcode = get_errno();
      bvdbg("ERROR: sigaction failed: %d\n" , ret);

      /* Emergency removal from the list */

      flags = irqsave();
      if (unload_list_remove(pid) != bin)
        {
          blldbg("ERROR: Failed to remove structure\n");
        }
      
      goto errout;
    }

  return OK;

errout:
  set_errno(errorcode);
  return ERROR;
}

#endif /* !CONFIG_BINFMT_DISABLE && CONFIG_SCHED_HAVE_PARENT */

