/****************************************************************************
 * arch/sim/src/up_touchscreen.c
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

#include <sys/types.h>

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <semaphore.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/nx/nx.h>

#include <nuttx/input/touchscreen.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifdef CONFIG_DISABLE_POLL
#  undef CONFIG_SIM_TCNWAITERS
#else
#  ifndef CONFIG_SIM_TCNWAITERS
#    define CONFIG_SIM_TCNWAITERS 4
#  endif
#endif

/* Driver support ***********************************************************/
/* This format is used to construct the /dev/input[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/input%d"
#define DEV_NAMELEN  16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This describes the state of one contact */

enum up_contact_3
{
  CONTACT_NONE = 0,                    /* No contact */
  CONTACT_DOWN,                        /* First contact */
  CONTACT_MOVE,                        /* Same contact, possibly different position */
  CONTACT_UP,                          /* Contact lost */
};

/* This structure describes the results of one touchscreen sample */

struct up_sample_s
{
  uint8_t  id;                         /* Sampled touch point ID */
  uint8_t  contact;                    /* Contact state (see enum up_contact_e) */
  uint16_t x;                          /* Measured X position */
  uint16_t y;                          /* Measured Y position */
};

/* This structure describes the state of one touchscreen driver instance */

struct up_dev_s
{
  volatile uint8_t nwaiters;           /* Number of threads waiting for touchscreen data */
  uint8_t id;                          /* Current touch point ID */
  uint8_t minor;                       /* Minor device number */
  volatile bool penchange;             /* An unreported event is buffered */
  sem_t devsem;                        /* Manages exclusive access to this structure */
  sem_t waitsem;                       /* Used to wait for the availability of data */

  struct up_sample_s sample;           /* Last sampled touch point data */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_SIM_TCNWAITERS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void up_notify(FAR struct up_dev_s *priv);
static int up_sample(FAR struct up_dev_s *priv,
                     FAR struct up_sample_s *sample);
static int up_waitsample(FAR struct up_dev_s *priv,
                         FAR struct up_sample_s *sample);

/* Character driver methods */

static int up_open(FAR struct file *filep);
static int up_close(FAR struct file *filep);
static ssize_t up_read(FAR struct file *filep, FAR char *buffer, size_t len);
static int up_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int up_poll(FAR struct file *filep, struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the the vtable that supports the character driver interface */

static const struct file_operations up_fops =
{
  up_open,    /* open */
  up_close,   /* close */
  up_read,    /* read */
  0,          /* write */
  0,          /* seek */
  up_ioctl    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , up_poll   /* poll */
#endif
};

/* Only one simulated touchscreen is supported o the the driver state
 * structure may as well be pre-allocated.
 */

static struct up_dev_s g_simtouchscreen;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_notify
 ****************************************************************************/

static void up_notify(FAR struct up_dev_s *priv)
{
#ifndef CONFIG_DISABLE_POLL
  int i;
#endif

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  ivdbg("contact=%d nwaiters=%d\n", priv->sample.contact, priv->nwaiters);
  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the touchscreen
       * is no longer avaialable.
       */

      sem_post(&priv->waitsem); 
    }

  /* If there are threads waiting on poll() for touchscreen data to become availabe,
   * then wake them up now.  NOTE: we wake up all waiting threads because we
   * do not know that they are going to do.  If they all try to read the data,
   * then some make end up blocking after all.
   */

#ifndef CONFIG_DISABLE_POLL
  for (i = 0; i < CONFIG_SIM_TCNWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          ivdbg("Report events: %02x\n", fds->revents);
          sem_post(fds->sem);
        }
    }
#endif
}

/****************************************************************************
 * Name: up_sample
 ****************************************************************************/

static int up_sample(FAR struct up_dev_s *priv,
                     FAR struct up_sample_s *sample)
{
  int ret = -EAGAIN;

  /* Is there new touchscreen sample data available? */

  ivdbg("penchange=%d contact=%d id=%d\n",
        priv->penchange, sample->contact, priv->id);

  if (priv->penchange)
    {
      /* Yes.. the state has changed in some way.  Return a copy of the
       * sampled data.
       */

      memcpy(sample, &priv->sample, sizeof(struct up_sample_s ));

      /* Now manage state transitions */

      if (sample->contact == CONTACT_UP)
        {
          /* Next.. no contract.  Increment the ID so that next contact ID will be unique */

          priv->sample.contact = CONTACT_NONE;
          priv->id++;
        }
      else if (sample->contact == CONTACT_DOWN)
       {
          /* First report -- next report will be a movement */

         priv->sample.contact = CONTACT_MOVE;
       }

      priv->penchange = false;
      ivdbg("penchange=%d contact=%d id=%d\n",
             priv->penchange, priv->sample.contact, priv->id);

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: up_waitsample
 ****************************************************************************/

static int up_waitsample(FAR struct up_dev_s *priv,
                         FAR struct up_sample_s *sample)
{
  irqstate_t flags;
  int ret;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   *
   * In addition, we will also disable pre-emption to prevent other threads
   * from getting control while we muck with the semaphores.
   */

  sched_lock();
  flags = irqsave();

  /* Now release the semaphore that manages mutually exclusive access to
   * the device structure.  This may cause other tasks to become ready to
   * run, but they cannot run yet because pre-emption is disabled.
   */

  sem_post(&priv->devsem);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is available.
   */

  while (up_sample(priv, sample) < 0)
    {
      /* Wait for a change in the touchscreen state */

      ivdbg("Waiting...\n");
      priv->nwaiters++;
      ret = sem_wait(&priv->waitsem);
      priv->nwaiters--;
      ivdbg("Awakened...\n");

      if (ret < 0)
        {
          /* If we are awakened by a signal, then we need to return
           * the failure now.
           */

          DEBUGASSERT(errno == EINTR);
          ret = -EINTR;
          goto errout;
        }
    }

  /* Re-acquire the the semaphore that manages mutually exclusive access to
   * the device structure.  We may have to wait here.  But we have our sample.
   * Interrupts and pre-emption will be re-enabled while we wait.
   */

  ret = sem_wait(&priv->devsem);

errout:
  /* Then re-enable interrupts.  We might get interrupt here and there
   * could be a new sample.  But no new threads will run because we still
   * have pre-emption disabled.
   */

  irqrestore(flags);

  /* Restore pre-emption.  We might get suspended here but that is okay
   * because we already have our sample.  Note:  this means that if there
   * were two threads reading from the touchscreen for some reason, the data
   * might be read out of order.
   */

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: up_open
 ****************************************************************************/

static int up_open(FAR struct file *filep)
{
  ivdbg("Opening...\n");
  return OK;
}

/****************************************************************************
 * Name: up_close
 ****************************************************************************/

static int up_close(FAR struct file *filep)
{
  ivdbg("Closing...\n");
  return OK;
}

/****************************************************************************
 * Name: up_read
 ****************************************************************************/

static ssize_t up_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode          *inode;
  FAR struct up_dev_s       *priv;
  FAR struct touch_sample_s *report;
  struct up_sample_s         sample;
  int                        ret;

  ivdbg("len=%d\n", len);

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct up_dev_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the touch data.
   */

  if (len < SIZEOF_TOUCH_SAMPLE_S(1))
    {
      /* We could provide logic to break up a touch report into segments and
       * handle smaller reads... but why?
       */

      return -ENOSYS;
    }

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Try to read sample data. */

  ret = up_sample(priv, &sample);
  if (ret < 0)
    {
      /* Sample data is not available now.  We would ave to wait to get
       * receive sample data.  If the user has specified the O_NONBLOCK
       * option, then just return an error.
       */

      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
       }

      /* Wait for sample data */

      ret = up_waitsample(priv, &sample);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          goto errout;
        }
    }

  /* In any event, we now have sampled touchscreen data that we can report
   * to the caller.
   */

  report = (FAR struct touch_sample_s *)buffer;
  memset(report, 0, SIZEOF_TOUCH_SAMPLE_S(1));
  report->npoints            = 1;
  report->point[0].id        = priv->id;
  report->point[0].x         = sample.x;
  report->point[0].y         = sample.y;
  report->point[0].h         = 1;
  report->point[0].w         = 1;
  report->point[0].pressure  = 42;

  /* Report the appropriate flags */

  if (sample.contact == CONTACT_UP)
    {
      /* Pen is now up */

      report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
    }
  else if (sample.contact == CONTACT_DOWN)
    {
      /* First contact */

      report->point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID | TOUCH_PRESSURE_VALID;
    }
  else /* if (sample->contact == CONTACT_MOVE) */
    {
      /* Movement of the same contact */

      report->point[0].flags  = TOUCH_MOVE | TOUCH_ID_VALID | TOUCH_POS_VALID | TOUCH_PRESSURE_VALID;
    }

  ret = SIZEOF_TOUCH_SAMPLE_S(1);

errout:
  ivdbg("Returning %d\n", ret);
  sem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name:up_ioctl
 ****************************************************************************/

static int up_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode         *inode;
  FAR struct up_dev_s *priv;
  int                       ret;

  ivdbg("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct up_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      default:
        ret = -ENOTTY;
        break;
    }

  sem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: up_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int up_poll(FAR struct file *filep, FAR struct pollfd *fds,
                   bool setup)
{
  FAR struct inode    *inode;
  FAR struct up_dev_s *priv;
  int                  ret = OK;
  int                  i;

  ivdbg("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct up_dev_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->revents & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_SIM_TCNWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_SIM_TCNWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->penchange)
        {
          up_notify(priv);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  sem_post(&priv->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arch_tcinitialize
 *
 * Description:
 *   Configure the simulated touchscreen.  This will register the driver as
 *   /dev/inputN where N is the minor device number
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int arch_tcinitialize(int minor)
{
  FAR struct up_dev_s *priv = ( FAR struct up_dev_s *)&g_simtouchscreen;
  char devname[DEV_NAMELEN];
  int ret;

  ivdbg("minor: %d\n", minor);

  /* Debug-only sanity checks */

  DEBUGASSERT(minor >= 0 && minor < 100);

  /* Initialize the touchscreen device driver instance */

  memset(priv, 0, sizeof(struct up_dev_s));
  sem_init(&priv->devsem,  0, 1); /* Initialize device structure semaphore */
  sem_init(&priv->waitsem, 0, 0); /* Initialize pen event wait semaphore */

  priv->minor = minor;

  /* Register the device as an input device */

  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, minor);
  ivdbg("Registering %s\n", devname);

  ret = register_driver(devname, &up_fops, 0666, priv);
  if (ret < 0)
    {
      idbg("register_driver() failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* Enable X11 event processing from the IDLE loop */

  g_eventloop = 1;

  /* And return success */

  return OK;

errout_with_priv:
  sem_destroy(&priv->waitsem);
  sem_destroy(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: arch_tcuninitialize
 *
 * Description:
 *   Uninitialized the simulated touchscreen
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void arch_tcuninitialize(void)
{
  FAR struct up_dev_s *priv = ( FAR struct up_dev_s *)&g_simtouchscreen;
  char devname[DEV_NAMELEN];
  int ret;

  /* Get exclusive access */

  do
    {
      ret = sem_wait(&priv->devsem);
      if (ret < 0)
        {
          /* This should only happen if the wait was canceled by an signal */

          DEBUGASSERT(errno == EINTR);
        }
    }
  while (ret != OK);

  /* Stop the event loop (Hmm.. the caller must be sure that there are no
   * open references to the touchscreen driver.  This might better be
   * done in close() using a reference count).
   */

  g_eventloop = 0;

  /* Un-register the device*/

  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->minor);
  ivdbg("Un-registering %s\n", devname);

  ret = unregister_driver(devname);
  if (ret < 0)
    {
      idbg("uregister_driver() failed: %d\n", ret);
    }

  /* Clean up any resources.  Ouch!  While we are holding the semaphore? */

  sem_destroy(&priv->waitsem);
  sem_destroy(&priv->devsem);
}

/****************************************************************************
 * Name: up_buttonevent
 ****************************************************************************/

int up_buttonevent(int x, int y, int buttons)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)&g_simtouchscreen;
  bool                 pendown;  /* true: pen is down */

  ivdbg("x=%d y=%d buttons=%02x\n", x, y, buttons);
  ivdbg("contact=%d nwaiters=%d\n", priv->sample.contact, priv->nwaiters);

  /* Any button press will count as pendown. */

  pendown = (buttons != 0);

  /* Handle the change from pen down to pen up */

  if (!pendown)
    {
      /* Ignore the pend up if the pen was already up (CONTACT_NONE == pen up and
       * already reported.  CONTACT_UP == pen up, but not reported)
       */

      if (priv->sample.contact == CONTACT_NONE)
        {
          return OK;
        }

      /* Not yet reported */

      priv->sample.contact = CONTACT_UP;
    }
  else
    {
      /* Save the measurements */

      priv->sample.x = x;
      priv->sample.y = y;

      /* Note the availability of new measurements */
      /* If this is the first (acknowledged) pen down report, then report
       * this as the first contact.  If contact == CONTACT_DOWN, it will be
       * set to set to CONTACT_MOVE after the contact is first sampled.
       */

      if (priv->sample.contact != CONTACT_MOVE)
        {
          /* First contact */

          priv->sample.contact = CONTACT_DOWN;
        }
    }

  /* Indicate the availability of new sample data for this ID */

  priv->sample.id = priv->id;
  priv->penchange = true;

  /* Notify any waiters that new touchscreen data is available */

  up_notify(priv);
  return OK;
}

