/************************************************************
 * libc/misc/lib_init.c
 *
 *   Copyright (C) 2007, 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lib.h>

#include "lib_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * lib_initialize
 ************************************************************/

/* General library initialization hook */

void weak_const_function lib_initialize(void)
{
}

#if CONFIG_NFILE_STREAMS > 0
/* The following function is called when a new TCB is allocated.  It
 * creates the streamlist instance that is stored in the TCB.
 */

FAR struct streamlist *lib_alloclist(void)
{
  FAR struct streamlist *list;
  list = (FAR struct streamlist*)lib_zalloc(sizeof(struct streamlist));
  if (list)
    {
      int i;

      /* Start with a reference count of one */

      list->sl_crefs = 1;

      /* Initialize the list access mutex */

      (void)sem_init(&list->sl_sem, 0, 1);

      /* Initialize each FILE structure */

      for (i = 0; i < CONFIG_NFILE_STREAMS; i++)
       {
         /* Clear the IOB */

          memset(&list->sl_streams[i], 0, sizeof(FILE));

          /* Indicate not opened */

          list->sl_streams[i].fs_filedes = -1;

          /* Initialize the stream semaphore to one to support one-at-
           * a-time access to private data sets.
           */

          lib_sem_initialize(&list->sl_streams[i]);
        }
    }
  return list;

}

/* This function is called when a TCB is closed (such as with
 * pthread_create().  It increases the reference count on the stream
 * list.
 */

void lib_addreflist(FAR struct streamlist *list)
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
       list->sl_crefs++;
       irqrestore(flags);
    }
}

/* this function is called when a TCB is destroyed.  Note that is
 * does not close the file by release this inode.  This happens
 * separately when the file descriptor list is freed.
 */

void lib_releaselist(FAR struct streamlist *list)
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
       crefs = --(list->sl_crefs);
       irqrestore(flags);

       /* If the count decrements to zero, then there is no reference
        * to the structure and it should be deallocated.  Since there
        * are references, it would be an error if any task still held
        * a reference to the list's semaphore.
        */

       if (crefs <= 0)
          {
#if CONFIG_STDIO_BUFFER_SIZE > 0
            int i;
#endif
            /* Destroy the semaphore and release the filelist */

            (void)sem_destroy(&list->sl_sem);

            /* Release each stream in the list */

#if CONFIG_STDIO_BUFFER_SIZE > 0
            for (i = 0; i < CONFIG_NFILE_STREAMS; i++)
             {
               /* Destroy the semaphore that protects the IO buffer */

               (void)sem_destroy(&list->sl_streams[i].fs_sem);

               /* Release the IO buffer */
               if (list->sl_streams[i].fs_bufstart)
                 {
                   sched_free(list->sl_streams[i].fs_bufstart);
                 }
             }
#endif
           /* Finally, release the list itself */

            sched_free(list);
         }
     }
}

#endif /* CONFIG_NFILE_STREAMS */


