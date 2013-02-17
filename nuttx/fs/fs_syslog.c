/****************************************************************************
 * fs/syslog.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/syslog.h>

#include "fs_internal.h"

#if defined(CONFIG_SYSLOG) && defined(CONFIG_SYSLOG_CHAR)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 /* Open the device/file write-only, try to create (file) it if it doesn't
  * exist, if the file that already exists, then append the new log data to
  * end of the file.
  */

#define SYSLOG_OFLAGS (O_WRONLY | O_CREAT | O_APPEND)

/* An invalid thread ID */

#define NO_HOLDER     ((pid_t)-1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This enumeration represents the state of the SYSLOG device interface */

enum syslog_state_e
{
  SYSLOG_UNINITIALIZED = 0, /* SYSLOG has not been initialized */
  SYSLOG_INITIALIZING,      /* SYSLOG is being initialized */
  SYSLOG_REOPEN,            /* SYSLOG open failed... try again later */
  SYSLOG_FAILURE,           /* SYSLOG open failed... don't try again */
  SYSLOG_OPENED,            /* SYSLOG device is open and ready to use */
};

/* This structure contains all SYSLOGing state information */

struct syslog_dev_s
{
  uint8_t     sl_state;     /* See enum syslog_state_e */
  sem_t       sl_sem;       /* Enforces mutually exclusive access */
  pid_t       sl_holder;    /* PID of the thread that holds the semaphore */
  struct file sl_file;      /* The syslog file structure */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the device structure for the console or syslogging function. */

static struct syslog_dev_s g_sysdev;
static const uint8_t       g_syscrlf[2]  = { '\r', '\n' };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_takesem
 *
 * Description:
 *   Write to the syslog device
 *
 ****************************************************************************/

static inline int syslog_takesem(void)
{
  pid_t me = getpid();
  int ret;

  /* Does this thread already hold the semaphore?  That could happen if
   * we wer called recursively, i.e., if the logic kicked off by
   * syslog_write() where to generate more debug output.  Return an error
   * in that case.
   */

  if (g_sysdev.sl_holder == me)
    {
      /* Return an error (instead of deadlocking) */

      return -EWOULDBLOCK;
    }

  /* Either the semaphore is available or is currently held by another
   * thread.  Wait for it to become available.
   */

  ret = sem_wait(&g_sysdev.sl_sem);
  if (ret < 0)
    {
      return -get_errno();
    }

  /* We hold the semaphore.  We can safely mark ourself as the holder
   * of the semaphore.
   */

  g_sysdev.sl_holder = me;
  return OK;
}

/****************************************************************************
 * Name: syslog_givesem
 *
 * Description:
 *   Write to the syslog device
 *
 ****************************************************************************/

static inline void syslog_givesem(void)
{
#ifdef CONFIG_DEBUG
  pid_t me = getpid();
  DEBUGASSERT(g_sysdev.sl_holder == me);
#endif

  /* Relinquish the semaphore */

  g_sysdev.sl_holder = NO_HOLDER;
  sem_post(&g_sysdev.sl_sem);
}
  
/****************************************************************************
 * Name: syslog_write
 *
 * Description:
 *   Write to the syslog device
 *
 ****************************************************************************/

static inline ssize_t syslog_write(FAR const void *buf, size_t nbytes)
{
  FAR struct inode *inode;

  /* Let the driver perform the write */

  inode = g_sysdev.sl_file.f_inode;
  return inode->u.i_ops->write(&g_sysdev.sl_file, buf, nbytes);
}

/****************************************************************************
 * Name: syslog_flush
 *
 * Description:
 *   Flush any buffer data in the file system to media.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static inline void syslog_flush(void)
{
  FAR struct inode *inode = g_sysdev.sl_file.f_inode;

  /* Is this a mountpoint? Does it support the sync method? */

  if (INODE_IS_MOUNTPT(inode) && inode->u.i_mops->sync)
    {
      /* Yes... synchronize to the stream */
 
      (void)inode->u.i_mops->sync(&g_sysdev.sl_file);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_initialize
 *
 * Description:
 *   Initialize to use the character device (or file) at
 *   CONFIG_SYSLOG_DEVPATH as the SYSLOG sink.
 *
 *   NOTE that this implementation excludes using a network connection as
 *   SYSLOG device.  That would be a good extension.
 *
 ****************************************************************************/

int syslog_initialize(void)
{
  FAR struct inode    *inode;
  FAR const char      *relpath = NULL;
  int                  ret;

  /* At this point, the only expected states are SYSLOG_UNINITIALIZED or
   * SYSLOG_REOPEN..  Not SYSLOG_INITIALIZING, SYSLOG_FAILURE, SYSLOG_OPENED.
   */

  DEBUGASSERT(g_sysdev.sl_state == SYSLOG_UNINITIALIZED ||
              g_sysdev.sl_state == SYSLOG_REOPEN);

  g_sysdev.sl_state = SYSLOG_INITIALIZING;

  /* Try to open the device.
   *
   * Note that we cannot just call open.  The syslog device must work on all
   * threads.  Open returns a file descriptor that is valid only for the
   * task that opened the device (and its pthread children).  Instead, we
   * essentially re-implement the guts of open() here so that we can get to
   * the thread-independent structures of the inode.
   */

  /* Get an inode for this file/device */

  inode = inode_find(CONFIG_SYSLOG_DEVPATH, &relpath);
  if (!inode)
    {
      /* The inode was not found.  In this case, we will attempt to re-open
       * the device repeatedly.  The assumption is that the device path is
       * valid but that the driver has not yet been registered.
       */

      g_sysdev.sl_state = SYSLOG_REOPEN;
      return -ENOENT;
    }

  /* Verify that the inode is valid and either a character driver or a
   * mountpoint.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if ((!INODE_IS_DRIVER(inode) && !INODE_IS_MOUNTPT(inode)))
#else
  if (!INODE_IS_DRIVER(inode))
#endif
    {
      ret = -ENXIO;
      goto errout_with_inode;
    }

  /* Make sure that the "entity" at this inode supports write access */

  if (!inode->u.i_ops || !inode->u.i_ops->write)
    {
      ret = -EACCES;
      goto errout_with_inode;
    }

  /* Initialize the file structure */

  g_sysdev.sl_file.f_oflags = SYSLOG_OFLAGS;
  g_sysdev.sl_file.f_pos    = 0;
  g_sysdev.sl_file.f_inode  = inode;

  /* Perform the low-level open operation. */

  ret = OK;
  if (inode->u.i_ops->open)
    {
      /* Is the inode a mountpoint? */

#ifndef CONFIG_DISABLE_MOUNTPOINT
      if (INODE_IS_MOUNTPT(inode))
        {
          /* Yes.  Open the device write-only, try to create it if it
           * doesn't exist, if the file that already exists, then append the
           * new log data to end of the file.
           */

          ret = inode->u.i_mops->open(&g_sysdev.sl_file, relpath,
                                      SYSLOG_OFLAGS, 0666);
        }

      /* No... then it must be a character driver in the NuttX pseudo-
       * file system.
       */

      else
#endif
        {
          ret = inode->u.i_ops->open(&g_sysdev.sl_file);
        }
    }

  /* Was the file/device successfully opened? */

  if (ret < 0)
    {
      ret = -ret;
      goto errout_with_inode;
    }

  /* The SYSLOG device is open and ready for writing. */

  sem_init(&g_sysdev.sl_sem, 0, 1);
  g_sysdev.sl_holder = NO_HOLDER;
  g_sysdev.sl_state  = SYSLOG_OPENED;
  return OK;

 errout_with_inode:
  inode_release(inode);
  g_sysdev.sl_state = SYSLOG_FAILURE;
  return ret;
}

/****************************************************************************
 * Name: syslog_putc
 *
 * Description:
 *   This is the low-level system logging interface.  The debugging/syslogging
 *   interfaces are syslog() and lowsyslog().  The difference is is that
 *   the syslog() function writes to fd=1 (stdout) whereas lowsyslog() uses
 *   a lower level interface that works from interrupt handlers.  This
 *   function is a a low-level interface used to implement lowsyslog().
 *
 ****************************************************************************/

int syslog_putc(int ch)
{
  ssize_t nbytes;
  uint8_t uch;
  int errcode;
  int ret;

  /* Ignore any output:
   *
   * (1) Before the SYSLOG device has been initialized.  This could happen
   *     from debug output that occurs early in the boot sequence before
   *     syslog_initialize() is called (SYSLOG_UNINITIALIZED).
   * (2) While the device is being initialized.  The case could happen if
   *     debug output is generated while syslog_initialize() executes
   *     (SYSLOG_INITIALIZING).
   * (3) While we are generating SYSLOG output.  The case could happen if
   *     debug output is generated while syslog_putc() executes
   *     (This case is actually handled inside of syslog_semtake()).
   * (4) Any debug output generated from interrupt handlers.  A disadvantage
   *     of using the generic character device for the SYSLOG is that it
   *     cannot handle debug output generated from interrupt level handlers.
   * (5) Any debug output generated from the IDLE loop.  The character
   *     driver interface is blocking and the IDLE thread is not permitted
   *     to block.
   * (6) If an irrecoverable failure occurred during initialization.  In
   *     this case, we won't ever bother to try again (ever).
   *
   * NOTE: That the third case is different.  It applies only to the thread
   * that currently holds the sl_sem sempaphore.  Other threads should wait.
   * that is why that case is handled in syslog_semtake().
   */

  /* Cases (4) and (5) */

  if (up_interrupt_context() || getpid() == 0)
    {
      errcode = ENOSYS;
      goto errout_with_errcode;
    }

  /* We can save checks in the usual case:  That after the SYSLOG device
   * has been successfully opened.
   */

  if (g_sysdev.sl_state != SYSLOG_OPENED)
    {
      /* Case (1) and (2) */
 
      if (g_sysdev.sl_state == SYSLOG_UNINITIALIZED ||
          g_sysdev.sl_state == SYSLOG_INITIALIZING)
       {
         errcode = EAGAIN; /* Can't access the SYSLOG now... maybe next time? */
         goto errout_with_errcode;
       }

      /* Case (6) */

      if (g_sysdev.sl_state == SYSLOG_FAILURE)
        {
          errcode = ENXIO;  /* There is no SYSLOG device */
          goto errout_with_errcode;
        }

      /* syslog_initialize() is called as soon as enough of the operating
       * system is in place to support the open operation... but it is
       * possible that the SYSLOG device is not yet registered at that time.
       * In this case, we know that the system is sufficiently initialized
       * to support an attempt to re-open the SYSLOG device.
       *
       * NOTE that the scheduler is locked.  That is because we do not have
       * fully initialized semaphore capability until the SYSLOG device is
       * successfully initialized
       */

      sched_lock();
      if (g_sysdev.sl_state == SYSLOG_REOPEN)
        {
          /* Try again to initialize the device.  We may do this repeatedly
           * because the log device might be something that was not ready
           * the first time that syslog_intialize() was called (such as a
           * USB serial device that has not yet been connected or a file in
           * an NFS mounted file system that has not yet been mounted).
           */

          ret = syslog_initialize();
          if (ret < 0)
            {
              sched_unlock();
              errcode = -ret;
              goto errout_with_errcode;
            }
        }

      sched_unlock();
      DEBUGASSERT(g_sysdev.sl_state == SYSLOG_OPENED);
    }

  /* Ignore carriage returns */

  if (ch == '\r')
    {
      return ch;
    }

  /* The syslog device is ready for writing and we have something of
   * value to write.
   */

  ret = syslog_takesem();
  if (ret < 0)
    {
      /* We probably already hold the semaphore and were probably
       * re-entered by the logic kicked off by syslog_write().
       * We might also have been interrupted by a signal.  Either
       * way, we are outta here.
       */

      errcode = -ret;
      goto errout_with_errcode;
    }

  /* Pre-pend a newline with a carriage return. */

  if (ch == '\n')
    {
      /* Write the CR-LF sequence */

      nbytes = syslog_write(g_syscrlf, 2);

      /* Synchronize the file when each CR-LF is encountered (i.e.,
       * implements line buffering always).
       */

#ifndef CONFIG_DISABLE_MOUNTPOINT
      if (nbytes > 0)
        {
          syslog_flush();
        }
#endif
    }
  else
    {
      /* Write the non-newline character (and don't flush) */

      uch = (uint8_t)ch;
      nbytes = syslog_write(&uch, 1);
    }

  syslog_givesem();

  /* Check if the write was successful.  If not, nbytes will be
   * a negated errno value.
   */

  if (nbytes < 0)
    {
      errcode = -ret;
      goto errout_with_errcode;
    }

  return ch;

errout_with_errcode:
  set_errno(errcode);
  return EOF;
}

#endif /* CONFIG_SYSLOG && CONFIG_SYSLOG_CHAR */
