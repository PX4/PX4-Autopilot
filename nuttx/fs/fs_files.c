/****************************************************************************
 * fs/fs_files.c
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>

#include "fs_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _files_semtake
 ****************************************************************************/

static void _files_semtake(FAR struct filelist *list)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&list->fl_sem) != 0)
    {
      /* The only case that an error should occr here is if
       * the wait was awakened by a signal.
       */

      ASSERT(get_errno() == EINTR);
    }
}

/****************************************************************************
 * Name: _files_semgive
 ****************************************************************************/

#define _files_semgive(list) sem_post(&list->fl_sem)

/****************************************************************************
 * Name: _files_close
 *
 * Description:
 *   Close an inode (if open)
 *
 * Assumuptions:
 *   Caller holds the list semaphore because the file descriptor will be freed.
 *
 ****************************************************************************/

static int _files_close(FAR struct file *filep)
{
  struct inode *inode = filep->f_inode;
  int ret = OK;

  /* Check if the struct file is open (i.e., assigned an inode) */

  if (inode)
    {
      /* Close the file, driver, or mountpoint. */

      if (inode->u.i_ops && inode->u.i_ops->close)
        {
          /* Perform the close operation */

          ret = inode->u.i_ops->close(filep);
        }

      /* And release the inode */

      inode_release(inode);

      /* Release the file descriptor */

      filep->f_oflags  = 0;
      filep->f_pos     = 0;
      filep->f_inode = NULL;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: files_initialize
 *
 * Description:
 *   This is called from the FS initialization logic to configure the files.
 *
 ****************************************************************************/

void files_initialize(void)
{
}

/****************************************************************************
 * Name: files_alloclist
 *
 * Description: Allocate a list of files for a new task
 *
 ****************************************************************************/

FAR struct filelist *files_alloclist(void)
{
  FAR struct filelist *list;
  list = (FAR struct filelist*)kzalloc(sizeof(struct filelist));
  if (list)
    {
       /* Start with a reference count of one */

       list->fl_crefs = 1;

       /* Initialize the list access mutex */

      (void)sem_init(&list->fl_sem, 0, 1);
    }

  return list;
}

/****************************************************************************
 * Name: files_addreflist
 *
 * Description:
 *   Increase the reference count on a file list
 *
 ****************************************************************************/

int files_addreflist(FAR struct filelist *list)
{
  if (list)
    {
      /* Increment the reference count on the list.
       * NOTE: that we disable interrupts to do this
       * (vs. taking the list semaphore).  We do this
       * because file cleanup operations often must be
       * done from the IDLE task which cannot wait
       * on semaphores.
       */

      register irqstate_t flags = irqsave();
      list->fl_crefs++;
      irqrestore(flags);
    }

  return OK;
}

/****************************************************************************
 * Name: files_releaselist
 *
 * Description:
 *   Release a reference to the file list
 *
 ****************************************************************************/

int files_releaselist(FAR struct filelist *list)
{
  int crefs;
  if (list)
    {
      /* Decrement the reference count on the list.
       * NOTE: that we disable interrupts to do this
       * (vs. taking the list semaphore).  We do this
       * because file cleanup operations often must be
       * done from the IDLE task which cannot wait
       * on semaphores.
       */

      register irqstate_t flags = irqsave();
      crefs = --(list->fl_crefs);
      irqrestore(flags);

      /* If the count decrements to zero, then there is no reference
       * to the structure and it should be deallocated.  Since there
       * are references, it would be an error if any task still held
       * a reference to the list's semaphore.
       */

      if (crefs <= 0)
        {
          int i;

          /* Close each file descriptor .. Normally, you would need
           * take the list semaphore, but it is safe to ignore the
           * semaphore in this context because there are no references
           */

          for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
            {
              (void)_files_close(&list->fl_files[i]);
            }

          /* Destroy the semaphore and release the filelist */

          (void)sem_destroy(&list->fl_sem);
          sched_free(list);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: files_dup
 *
 * Description:
 *   Assign an inode to a specific files structure.  This is the heart of
 *   dup2.
 *
 ****************************************************************************/

int files_dup(FAR struct file *filep1, FAR struct file *filep2)
{
  FAR struct filelist *list;
  FAR struct inode *inode;
  int err;
  int ret;

  if (!filep1 || !filep1->f_inode || !filep2)
    {
      err = EBADF;
      goto errout;
    }

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (INODE_IS_MOUNTPT(filep1->f_inode))
    {
      err = ENOSYS; /* Not yet supported */
      goto errout;
    }
#endif

  list = sched_getfiles();
  if (!list)
    {
      err = EMFILE;
      goto errout;
    }

  _files_semtake(list);

  /* If there is already an inode contained in the new file structure,
   * close the file and release the inode.
   */

  ret = _files_close(filep2);
  if (ret < 0)
    {
      /* An error occurred while closing the driver */

      goto errout_with_ret;
    }

  /* Increment the reference count on the contained inode */

  inode = filep1->f_inode;
  inode_addref(inode);

  /* Then clone the file structure */

  filep2->f_oflags = filep1->f_oflags;
  filep2->f_pos    = filep1->f_pos;
  filep2->f_inode  = inode;

  /* Call the open method on the file, driver, mountpoint so that it
   * can maintain the correct open counts.
   */

  if (inode->u.i_ops && inode->u.i_ops->open)
    {
#ifndef CONFIG_DISABLE_MOUNTPOINT
#if 0 /* Not implemented */
      if (INODE_IS_MOUNTPT(inode))
        {
         /* Open a file on the mountpoint */

          ret = inode->u.i_mops->open(filep2, ?, filep2->f_oflags, ?);
        }
      else
#endif
#endif
        {
          /* Open the pseudo file or device driver */

          ret = inode->u.i_ops->open(filep2);
        }

      /* Handle open failures */

      if (ret < 0)
        {
          goto errout_with_inode;
        }
    }

  _files_semgive(list);
  return OK;

/* Handler various error conditions */

errout_with_inode:
  inode_release(filep2->f_inode);
  filep2->f_oflags = 0;
  filep2->f_pos    = 0;
  filep2->f_inode  = NULL;

errout_with_ret:
  err              = -ret;
  _files_semgive(list);

errout:
  set_errno(err);
  return ERROR;
}

/****************************************************************************
 * Name: files_allocate
 *
 * Description:
 *   Allocate a struct files instance and associate it with an inode instance. 
 *   Returns the file descriptor == index into the files array.
 *
 ****************************************************************************/

int files_allocate(FAR struct inode *inode, int oflags, off_t pos, int minfd)
{
  FAR struct filelist *list;
  int i;

  list = sched_getfiles();
  if (list)
    {
      _files_semtake(list);
      for (i = minfd; i < CONFIG_NFILE_DESCRIPTORS; i++)
        {
          if (!list->fl_files[i].f_inode)
            {
               list->fl_files[i].f_oflags = oflags;
               list->fl_files[i].f_pos    = pos;
               list->fl_files[i].f_inode  = inode;
               list->fl_files[i].f_priv   = NULL;
               _files_semgive(list);
               return i;
            }
        }

      _files_semgive(list);
    }

  return ERROR;
}

/****************************************************************************
 * Name: files_close
 *
 * Description:
 *   Close an inode (if open)
 *
 * Assumuptions:
 *   Caller holds the list semaphore because the file descriptor will be freed.
 *
 ****************************************************************************/

int files_close(int filedes)
{
  FAR struct filelist *list;
  int                  ret;

  /* Get the thread-specific file list */

  list = sched_getfiles();
  if (!list)
    {
      return -EMFILE;
    }

  /* If the file was properly opened, there should be an inode assigned */

  if (filedes < 0 || filedes >= CONFIG_NFILE_DESCRIPTORS || !list->fl_files[filedes].f_inode)
   {
     return -EBADF;
   }

  /* Perform the protected close operation */

  _files_semtake(list);
  ret = _files_close(&list->fl_files[filedes]);
  _files_semgive(list);
  return ret;
}

/****************************************************************************
 * Name: files_release
 *
 * Assumuptions:
 *   Similar to files_close().  Called only from open() logic on error
 *   conditions.
 *
 ****************************************************************************/

void files_release(int filedes)
{
  FAR struct filelist *list;

  list = sched_getfiles();
  if (list)
    {
      if (filedes >=0 && filedes < CONFIG_NFILE_DESCRIPTORS)
        {
          _files_semtake(list);
          list->fl_files[filedes].f_oflags  = 0;
          list->fl_files[filedes].f_pos     = 0;
          list->fl_files[filedes].f_inode = NULL;
          _files_semgive(list);
        }
    }
}

