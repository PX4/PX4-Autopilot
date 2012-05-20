/****************************************************************************
 * nuttx/graphics/nxconsole/nxcon_kbdin.c
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

#include <fcntl.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "nxcon_internal.h"

#ifdef CONFIG_NXCONSOLE_NXKBDIN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxcon_pollnotify
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static void nxcon_pollnotify(FAR struct nxcon_state_s *priv, pollevent_t eventset)
{
  FAR struct pollfd *fds;
  irqstate_t flags;
  int i;

  /* This function may be called from an interrupt handler */

  for (i = 0; i < CONFIG_NXCONSOLE_NPOLLWAITERS; i++)
    {
      flags = irqsave();
      fds   = priv->fds[i];
      if (fds)
        {
          fds->revents |= (fds->events & eventset);
          if (fds->revents != 0)
            {
              sem_post(fds->sem);
            }
        }
      irqrestore(flags);
    }
}
#else
#  define nxcon_pollnotify(priv,event)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxcon_read
 *
 * Description:
 *   The optional NxConsole read method
 *
 ****************************************************************************/

ssize_t nxcon_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct nxcon_state_s *priv;
  ssize_t nread;
  char ch;
  int ret;

  /* Recover our private state structure */

  DEBUGASSERT(filep && filep->f_priv);
  priv = (FAR struct nxcon_state_s *)filep->f_priv;

  /* Get exclusive access to the driver structure */

  ret = nxcon_semwait(priv);
  if (ret < 0)
    {
      gdbg("ERROR: nxcon_semwait failed\n");
      return ret;
    }

  /* Loop until something is read */

  for (nread = 0; nread < len; )
    {
      /* Get the next byte from the buffer */

      if (priv->head == priv->tail)
        {
          /* The circular buffer is empty. Did we read anything? */

          if (nread > 0)
            {
              /* Yes.. break out to return what we have.  */

              break;
            }

          /* If the driver was opened with O_NONBLOCK option, then don't wait.
           * Just return EGAIN.
           */

          if (filep->f_oflags & O_NONBLOCK)
            {
              nread = -EAGAIN;
              break;
            }

          /* Otherwise, wait for something to be written to the circular
           * buffer. Increment the number of waiters so that the nxcon_write()
           * will not that it needs to post the semaphore to wake us up.
           */

          sched_lock();
          priv->nwaiters++;
          nxcon_sempost(priv);

          /* We may now be pre-empted!  But that should be okay because we
           * have already incremented nwaiters.  Pre-emption is disabled
           * but will be re-enabled while we are waiting.
           */

          ret = sem_wait(&priv->waitsem);

          /* Pre-emption will be disabled when we return.  So the decrementing
           * nwaiters here is safe.
           */

          priv->nwaiters--;
          sched_unlock();

          /* Did we successfully get the waitsem? */

          if (ret >= 0)
            {
              /* Yes... then retake the mutual exclusion semaphore */

              ret = nxcon_semwait(priv);
            }

          /* Was the semaphore wait successful? Did we successful re-take the
           * mutual exclusion semaphore?
           */

          if (ret < 0) 
            {
              /* No.. One of the two sem_wait's failed. */

              int errval = errno;

              gdbg("ERROR: nxcon_semwait failed\n");

              /* Were we awakened by a signal?  Did we read anything before
               * we received the signal?
               */

              if (errval != EINTR || nread >= 0)
                {
                  /* Yes.. return the error. */

                  nread = -errval;
                }

              /* Break out to return what we have.  Note, we can't exactly
               * "break" out because whichever error occurred, we do not hold
               * the exclusion semaphore.
               */

              goto errout_without_sem;
            }
        }
      else
        {
          /* The circular buffer is not empty, get the next byte from the
           * tail index.
           */

          ch = priv->rxbuffer[priv->tail];

          /* Increment the tail index and re-enable interrupts */

          if (++priv->tail >= CONFIG_NXCONSOLE_KBDBUFSIZE)
            {
              priv->tail = 0;
            }

          /* Add the character to the user buffer */

          buffer[nread] = ch;
          nread++;
        }
    }

  /* Relinquish the mutual exclusion semaphore */

  nxcon_sempost(priv);

  /* Notify all poll/select waiters that they can write to the FIFO */

errout_without_sem:

#ifndef CONFIG_DISABLE_POLL
  if (nread > 0)
    {
      nxcon_pollnotify(priv, POLLOUT);
    }
#endif

  /* Return the number of characters actually read */

  return nread;
}

/****************************************************************************
 * Name: nxcon_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
int nxcon_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct nxcon_state_s *priv;
  pollevent_t eventset;
  int ndx;
  int ret;
  int i;

  /* Some sanity checking */

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = nxcon_semwait(priv);
  if (ret < 0)
    {
      gdbg("ERROR: nxcon_semwait failed\n");
      return ret;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_NXCONSOLE_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv       = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_NXCONSOLE_NPOLLWAITERS)
        {
          gdbg("ERROR: Too many poll waiters\n");

          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should immediately notify on any of the requested events?
       * This driver is always available for transmission.
       */

      eventset = POLLOUT;

      /* Check if the receive buffer is empty */

      if (priv->head != priv->tail)
       {
         eventset |= POLLIN;
       }

      if (eventset)
        {
          nxcon_pollnotify(priv, eventset);
        }

    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;

#ifdef CONFIG_DEBUG
      if (!slot)
        {
          gdbg("ERROR: No slot\n");

          ret = -EIO;
          goto errout;
        }
#endif

      /* Remove all memory of the poll setup */

      *slot      = NULL;
      fds->priv  = NULL;
    }

errout:
  nxcon_sempost(priv);
  return ret;
}
#endif

/****************************************************************************
 * Name: nxcon_kbdin
 *
 * Description:
 *  This function should be driven by the window kbdin callback function
 *  (see nx.h).  When the NxConsole is the top window and keyboard input is
 *  received on the top window, that window callback should be directed to
 *  this function.  This function will buffer the keyboard data and may
 *  it available to the NxConsole as stdin.
 *
 *  If CONFIG_NXCONSOLE_NXKBDIN is not selected, then the NxConsole will
 *  receive its input from stdin (/dev/console).  This works great but
 *  cannot be shared between different windows.  Chaos will ensue if you
 *  try to support multiple NxConsole windows without CONFIG_NXCONSOLE_NXKBDIN
 *
 * Input Parameters:
 *   handle - A handle previously returned by nx_register, nxtk_register, or
 *     nxtool_register.
 *   buffer   - The array of characters
 *   buflen  - The number of characters that are available in buffer[]
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxcon_kbdin(NXCONSOLE handle, FAR const uint8_t *buffer, uint8_t buflen)
{
  FAR struct nxcon_state_s *priv;
  ssize_t nwritten;
  int nexthead;
  char ch;
  int ret;

  gvdbg("buflen=%d\n");
  DEBUGASSERT(handle);

  /* Get the reference to the driver structure from the handle */

  priv = (FAR struct nxcon_state_s *)handle;

  /* Get exclusive access to the driver structure */

  ret = nxcon_semwait(priv);
  if (ret < 0)
    {
      gdbg("ERROR: nxcon_semwait failed\n");
      return;
    }

 /* Loop until all of the bytes have been written.  This function may be
  * called from an interrupt handler!  Semaphores cannot be used!
  *
  * The write logic only needs to modify the head index.  Therefore,
  * there is a difference in the way that head and tail are protected:
  * tail is protected with a semaphore; tail is protected by disabling
  * interrupts.
  */

  for (nwritten = 0; nwritten < buflen; nwritten++)
    {
      /* Add the next character */

      ch = buffer[nwritten];

      /* Calculate the write index AFTER the next byte is add to the ring
       * buffer
       */

      nexthead = priv->head + 1;
      if (nexthead >= CONFIG_NXCONSOLE_KBDBUFSIZE)
        {
          nexthead = 0;
        }

      /* Would the next write overflow the circular buffer? */

      if (nexthead == priv->tail)
        {
          /* Yes... Return an indication that nothing was saved in the buffer. */

          gdbg("ERROR: Keyboard data overrun\n");
          break;
        }

      /* No... copy the byte */

      priv->rxbuffer[priv->head] = ch;
      priv->head = nexthead;
    }

  /* Was anything written? */

  if (nwritten > 0)
    {
      int i;

      /* Are there threads waiting for read data? */

      sched_lock();
      for (i = 0; i < priv->nwaiters; i++)
        {
          /* Yes.. Notify all of the waiting readers that more data is available */

          sem_post(&priv->waitsem);
        }

      /* Notify all poll/select waiters that they can write to the FIFO */

#ifndef CONFIG_DISABLE_POLL
      nxcon_pollnotify(priv, POLLIN);
#endif
      sched_unlock();
    }

  nxcon_sempost(priv);
}

#endif /* CONFIG_NXCONSOLE_NXKBDIN */
