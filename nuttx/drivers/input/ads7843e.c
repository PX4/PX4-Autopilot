/****************************************************************************
 * drivers/input/ads7843e.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Diego Sanchez <dsanchez@nx-engineering.com>
 *
 * References:
 *   "Touch Screen Controller, ADS7843," Burr-Brown Products from Texas
 *    Instruments, SBAS090B, September 2000, Revised May 2002"
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
#include <wdog.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi.h>
#include <nuttx/wqueue.h>

#include <nuttx/input/touchscreen.h>
#include <nuttx/input/ads7843e.h>

#include "ads7843e.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Low-level SPI helpers */

static inline void ads7843e_configspi(FAR struct spi_dev_s *spi);
#ifdef CONFIG_SPI_OWNBUS
static inline void ads7843e_select(FAR struct spi_dev_s *spi);
static inline void ads7843e_deselect(FAR struct spi_dev_s *spi);
#else
static void ads7843e_select(FAR struct spi_dev_s *spi);
static void ads7843e_deselect(FAR struct spi_dev_s *spi);
#endif

static inline void ads7843e_waitbusy(FAR struct ads7843e_dev_s *priv);
static uint16_t ads7843e_sendcmd(FAR struct ads7843e_dev_s *priv, uint8_t cmd);

/* Interrupts and data sampling */

static void ads7843e_notify(FAR struct ads7843e_dev_s *priv);
static int ads7843e_sample(FAR struct ads7843e_dev_s *priv,
                           FAR struct ads7843e_sample_s *sample);
static int ads7843e_waitsample(FAR struct ads7843e_dev_s *priv,
                               FAR struct ads7843e_sample_s *sample);
static void ads7843e_worker(FAR void *arg);
static int ads7843e_interrupt(int irq, FAR void *context);

/* Character driver methods */

static int ads7843e_open(FAR struct file *filep);
static int ads7843e_close(FAR struct file *filep);
static ssize_t ads7843e_read(FAR struct file *filep, FAR char *buffer, size_t len);
static int ads7843e_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int ads7843e_poll(FAR struct file *filep, struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the the vtable that supports the character driver interface */

static const struct file_operations ads7843e_fops =
{
  ads7843e_open,    /* open */
  ads7843e_close,   /* close */
  ads7843e_read,    /* read */
  0,                /* write */
  0,                /* seek */
  ads7843e_ioctl    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , ads7843e_poll   /* poll */
#endif
};

/* If only a single ADS7843E device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

#ifndef CONFIG_ADS7843E_MULTIPLE
static struct ads7843e_dev_s g_ads7843e;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct ads7843e_dev_s *g_ads7843elist;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: ads7843e_select
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary.  This function
 *   must be called before initiating any sequence of SPI operations. If we
 *   are sharing the SPI bus with other devices (CONFIG_SPI_OWNBUS undefined)
 *   then we need to lock and configure the SPI bus for each transfer.
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void ads7843e_select(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just select the chip */

  SPI_SELECT(spi, SPIDEV_TOUCHSCREEN, true);
}
#else
static void ads7843e_select(FAR struct spi_dev_s *spi)
{
  /* Select ADS7843 chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_TOUCHSCREEN, true);

  /* Now make sure that the SPI bus is configured for the ADS7843 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_ADS7843E_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, CONFIG_ADS7843E_FREQUENCY);
}
#endif

/****************************************************************************
 * Function: ads7843e_deselect
 *
 * Description:
 *   De-select the SPI, unlocking as necessary.  This function must be
 *   after completing a sequence of SPI operations. If we are sharing the SPI
 *   bus with other devices (CONFIG_SPI_OWNBUS undefined) then we need to
 *   un-lock the SPI bus for each transfer, possibly losing the current
 *   configuration.
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void ads7843e_deselect(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just de-select the chip */

  SPI_SELECT(spi, SPIDEV_TOUCHSCREEN, false);
}
#else
static void ads7843e_deselect(FAR struct spi_dev_s *spi)
{
  /* De-select ADS7843 chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_TOUCHSCREEN, false);
  SPI_LOCK(spi, false);
}
#endif

/****************************************************************************
 * Function: ads7843e_configspi
 *
 * Description:
 *   Configure the SPI for use with the ADS7843E.  This function should be
 *   called once during touchscreen initialization to configure the SPI
 *   bus.  Note that if CONFIG_SPI_OWNBUS is not defined, then this function
 *   does nothing.
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void ads7843e_configspi(FAR struct spi_dev_s *spi)
{
  idbg("Mode: %d Bits: 8 Frequency: %d\n",
       CONFIG_ADS7843E_SPIMODE, CONFIG_ADS7843E_FREQUENCY);

  /* Configure SPI for the ADS7843.  But only if we own the SPI bus.  Otherwise, don't
   * bother because it might change.
   */

#ifdef CONFIG_SPI_OWNBUS
  SPI_SELECT(spi, SPIDEV_TOUCHSCREEN, true);
  SPI_SETMODE(spi, CONFIG_ADS7843E_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, CONFIG_ADS7843E_FREQUENCY);
  SPI_SELECT(spi, SPIDEV_TOUCHSCREEN, false);
#endif
}

/****************************************************************************
 * Name: ads7843e_waitbusy
 ****************************************************************************/

static inline void ads7843e_waitbusy(FAR struct ads7843e_dev_s *priv)
{
  while (priv->config->busy(priv->config));
}

/****************************************************************************
 * Name: ads7843e_sendcmd
 ****************************************************************************/

static uint16_t ads7843e_sendcmd(FAR struct ads7843e_dev_s *priv, uint8_t cmd)
{
  uint8_t  buffer[2];
  uint16_t result;

  /* Select the ADS7843E */

  ads7843e_select(priv->spi);

  /* Send the command */

  (void)SPI_SEND(priv->spi, cmd);
  ads7843e_waitbusy(priv);

  /* Read the data */

  SPI_RECVBLOCK(priv->spi, buffer, 2);
  ads7843e_deselect(priv->spi);

  result = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
  result = result >> 4;

  ivdbg("cmd:%02x response:%04x\n", cmd, result);
  return result;
}

/****************************************************************************
 * Name: ads7843e_notify
 ****************************************************************************/

static void ads7843e_notify(FAR struct ads7843e_dev_s *priv)
{
#ifndef CONFIG_DISABLE_POLL
  int i;
#endif

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the ADS7843E
       * is no longer available.
       */

      sem_post(&priv->waitsem); 
    }

  /* If there are threads waiting on poll() for ADS7843E data to become available,
   * then wake them up now.  NOTE: we wake up all waiting threads because we
   * do not know that they are going to do.  If they all try to read the data,
   * then some make end up blocking after all.
   */

#ifndef CONFIG_DISABLE_POLL
  for (i = 0; i < CONFIG_ADS7843E_NPOLLWAITERS; i++)
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
 * Name: ads7843e_sample
 ****************************************************************************/

static int ads7843e_sample(FAR struct ads7843e_dev_s *priv,
                          FAR struct ads7843e_sample_s *sample)
{
  irqstate_t flags;
  int ret = -EAGAIN;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semaphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   */

  flags = irqsave();

  /* Is there new ADS7843E sample data available? */

  if (priv->penchange)
    {
      /* Yes.. the state has changed in some way.  Return a copy of the
       * sampled data.
       */

      memcpy(sample, &priv->sample, sizeof(struct ads7843e_sample_s ));

      /* Now manage state transitions */

      if (sample->contact == CONTACT_UP)
        {
          /* Next.. no contact.  Increment the ID so that next contact ID
           * will be unique.  X/Y positions are no longer valid.
           */

          priv->sample.contact = CONTACT_NONE;
          priv->sample.valid   = false;
          priv->id++;
        }
      else if (sample->contact == CONTACT_DOWN)
       {
          /* First report -- next report will be a movement */

         priv->sample.contact = CONTACT_MOVE;
       }

      priv->penchange = false;
      ret = OK;
    }

  irqrestore(flags);
  return ret;
}

/****************************************************************************
 * Name: ads7843e_waitsample
 ****************************************************************************/

static int ads7843e_waitsample(FAR struct ads7843e_dev_s *priv,
                               FAR struct ads7843e_sample_s *sample)
{
  irqstate_t flags;
  int ret;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semaphores from interrupt handlers, and (2) to prevent sampled data
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

  while (ads7843e_sample(priv, sample) < 0)
    {
      /* Wait for a change in the ADS7843E state */
 
      ivdbg("Waiting..\n");
      priv->nwaiters++;
      ret = sem_wait(&priv->waitsem);
      priv->nwaiters--;

      if (ret < 0)
        {
          /* If we are awakened by a signal, then we need to return
           * the failure now.
           */

          idbg("sem_wait: %d\n", errno);
          DEBUGASSERT(errno == EINTR);
          ret = -EINTR;
          goto errout;
        }
    }

  ivdbg("Sampled\n");

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
   * were two threads reading from the ADS7843E for some reason, the data
   * might be read out of order.
   */

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: ads7843e_schedule
 ****************************************************************************/

static int ads7843e_schedule(FAR struct ads7843e_dev_s *priv)
{
  FAR struct ads7843e_config_s *config;
  int                           ret;

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Disable further interrupts.  ADS7843E interrupts will be re-enabled
   * after the worker thread executes.
   */

  config->enable(config, false);

  /* Disable the watchdog timer.  It will be re-enabled in the worker thread
   * while the pen remains down.
   */

  wd_cancel(priv->wdog);

  /* Transfer processing to the worker thread.  Since ADS7843E interrupts are
   * disabled while the work is pending, no special action should be required
   * to protected the work queue.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(&priv->work, ads7843e_worker, priv, 0);
  if (ret != 0)
    {
      illdbg("Failed to queue work: %d\n", ret);
    }

  return OK;
}

/****************************************************************************
 * Name: ads7843e_wdog
 ****************************************************************************/

static void ads7843e_wdog(int argc, uint32_t arg1, ...)
{
  FAR struct ads7843e_dev_s *priv = (FAR struct ads7843e_dev_s *)((uintptr_t)arg1);
  (void)ads7843e_schedule(priv);
}

/****************************************************************************
 * Name: ads7843e_worker
 ****************************************************************************/

static void ads7843e_worker(FAR void *arg)
{
  FAR struct ads7843e_dev_s    *priv = (FAR struct ads7843e_dev_s *)arg;
  FAR struct ads7843e_config_s *config;
  bool                          pendown;

  ASSERT(priv != NULL);

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Disable the watchdog timer */

  wd_cancel(priv->wdog);

  /* Check for pen up or down by reading the PENIRQ GPIO. */

  pendown = config->pendown(config);

  /* Handle the change from pen down to pen up */

  if (!pendown)
    {
      /* Ignore the interrupt if the pen was already up (CONTACT_NONE == pen up and
       * already reported.  CONTACT_UP == pen up, but not reported)
       */

      if (priv->sample.contact == CONTACT_NONE)
        {
          goto errout;
        }

      /* The pen is up.  NOTE: We know from a previous test, that this is a
       * loss of contact condition.  This will be changed to CONTACT_NONE
       * after the loss of contact is sampled.
       */

       priv->sample.contact = CONTACT_UP;
    }

  /* It is a pen down event.  If the last loss-of-contact event has not been
   * processed yet, then we have to ignore the pen down event (or else it will
   * look like a drag event)
   */

  else if (priv->sample.contact == CONTACT_UP)
    {
       goto errout;
    }
  else
    {
      /* Handle pen down events.  First, sample positional values. */

      priv->sample.x = ads7843e_sendcmd(priv, ADS7843_CMD_XPOSITION);
      priv->sample.y = ads7843e_sendcmd(priv, ADS7843_CMD_YPOSITION);

      /* The X/Y positional data is now valid */

      priv->sample.valid = true;

      /* If this is the first (acknowledged) pen down report, then report
       * this as the first contact.  If contact == CONTACT_DOWN, it will be
       * set to set to CONTACT_MOVE after the contact is first sampled.
       */

      if (priv->sample.contact != CONTACT_MOVE)
        {
          /* First contact */

          priv->sample.contact = CONTACT_DOWN;
        }

      /* Continue to sample the position while the pen is down */

       wd_start(priv->wdog, ADS7843E_WDOG_DELAY, ads7843e_wdog, 1, (uint32_t)priv);
    }

  /* Indicate the availability of new sample data for this ID */

  priv->sample.id = priv->id;
  priv->penchange = true;

  /* Notify any waiters that new ADS7843E data is available */

  ads7843e_notify(priv);

  /* Exit, re-enabling ADS7843E interrupts */

errout:
  (void)ads7843e_sendcmd(priv, ADS7843_CMD_ENABPINIRQ);
  config->enable(config, true);
}

/****************************************************************************
 * Name: ads7843e_interrupt
 ****************************************************************************/

static int ads7843e_interrupt(int irq, FAR void *context)
{
  FAR struct ads7843e_dev_s    *priv;
  FAR struct ads7843e_config_s *config;
  int                           ret;

  /* Which ADS7843E device caused the interrupt? */

#ifndef CONFIG_ADS7843E_MULTIPLE
  priv = &g_ads7843e;
#else
  for (priv = g_ads7843elist;
       priv && priv->configs->irq != irq;
       priv = priv->flink);

  ASSERT(priv != NULL);
#endif

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Schedule sampling to occur on the worker thread */

  ret = ads7843e_schedule(priv);

  /* Clear any pending interrupts and return success */

  config->clear(config);
  return ret;
}

/****************************************************************************
 * Name: ads7843e_open
 ****************************************************************************/

static int ads7843e_open(FAR struct file *filep)
{
#ifdef CONFIG_ADS7843E_REFCNT
  FAR struct inode         *inode;
  FAR struct ads7843e_dev_s *priv;
  uint8_t                   tmp;
  int                       ret;

  ivdbg("Opening\n");

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ads7843e_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Increment the reference count */

  tmp = priv->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* When the reference increments to 1, this is the first open event
   * on the driver.. and an opportunity to do any one-time initialization.
   */

  /* Save the new open count on success */

  priv->crefs = tmp;

errout_with_sem:
  sem_post(&priv->devsem);
  return ret;
#else
  ivdbg("Opening\n");
  return OK;
#endif
}

/****************************************************************************
 * Name: ads7843e_close
 ****************************************************************************/

static int ads7843e_close(FAR struct file *filep)
{
#ifdef CONFIG_ADS7843E_REFCNT
  FAR struct inode         *inode;
  FAR struct ads7843e_dev_s *priv;
  int                       ret;

  ivdbg("Closing\n");
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ads7843e_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Decrement the reference count unless it would decrement a negative
   * value.  When the count decrements to zero, there are no further
   * open references to the driver.
   */

  if (priv->crefs >= 1)
    {
      priv->crefs--;
    }

  sem_post(&priv->devsem);
#endif
  ivdbg("Closing\n");
  return OK;
}

/****************************************************************************
 * Name: ads7843e_read
 ****************************************************************************/

static ssize_t ads7843e_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode          *inode;
  FAR struct ads7843e_dev_s *priv;
  FAR struct touch_sample_s *report;
  struct ads7843e_sample_s   sample;
  int                        ret;

  ivdbg("buffer:%p len:%d\n", buffer, len);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ads7843e_dev_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the touch data.
   */

  if (len < SIZEOF_TOUCH_SAMPLE_S(1))
    {
      /* We could provide logic to break up a touch report into segments and
       * handle smaller reads... but why?
       */

      idbg("Unsupported read size: %d\n", len);
      return -ENOSYS;
    }

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      idbg("sem_wait: %d\n", errno);
      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Try to read sample data. */

  ret = ads7843e_sample(priv, &sample);
  if (ret < 0)
    {
      /* Sample data is not available now.  We would ave to wait to get
       * receive sample data.  If the user has specified the O_NONBLOCK
       * option, then just return an error.
       */

      ivdbg("Sample data is not available\n");
      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
       }

      /* Wait for sample data */

      ret = ads7843e_waitsample(priv, &sample);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          idbg("ads7843e_waitsample: %d\n", ret);
          goto errout;
        }
    }

  /* In any event, we now have sampled ADS7843E data that we can report
   * to the caller.
   */

  report = (FAR struct touch_sample_s *)buffer;
  memset(report, 0, SIZEOF_TOUCH_SAMPLE_S(1));
  report->npoints            = 1;
  report->point[0].id        = priv->id;
  report->point[0].x         = sample.x;
  report->point[0].y         = sample.y;

  /* Report the appropriate flags */

  if (sample.contact == CONTACT_UP)
    {
       /* Pen is now up.  Is the positional data valid?  This is important to
        * know because the release will be sent to the window based on its
        * last positional data.
        */

      if (sample.valid)
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID |
                                    TOUCH_POS_VALID | TOUCH_PRESSURE_VALID;
        }
      else
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
        }
    }
  else if (sample.contact == CONTACT_DOWN)
    {
      /* First contact */

      report->point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID;
    }
  else /* if (sample->contact == CONTACT_MOVE) */
    {
      /* Movement of the same contact */

      report->point[0].flags  = TOUCH_MOVE | TOUCH_ID_VALID | TOUCH_POS_VALID;
    }

  ivdbg("  id:      %d\n", report->point[0].id);
  ivdbg("  flags:   %02x\n", report->point[0].flags);
  ivdbg("  x:       %d\n", report->point[0].x);
  ivdbg("  y:       %d\n", report->point[0].y);

  ret = SIZEOF_TOUCH_SAMPLE_S(1);

errout:
  sem_post(&priv->devsem);
  ivdbg("Returning: %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name:ads7843e_ioctl
 ****************************************************************************/

static int ads7843e_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode         *inode;
  FAR struct ads7843e_dev_s *priv;
  int                       ret;

  ivdbg("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ads7843e_dev_s *)inode->i_private;

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
      case TSIOC_SETFREQUENCY:  /* arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(priv->config != NULL && ptr != NULL);
          priv->config->frequency = SPI_SETFREQUENCY(priv->spi, *ptr);
        }
        break;

      case TSIOC_GETFREQUENCY:  /* arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(priv->config != NULL && ptr != NULL);
          *ptr = priv->config->frequency;
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  sem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: ads7843e_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int ads7843e_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode         *inode;
  FAR struct ads7843e_dev_s *priv;
  pollevent_t               eventset;
  int                       ndx;
  int                       ret = OK;
  int                       i;

  ivdbg("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ads7843e_dev_s *)inode->i_private;

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

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_ADS7843E_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_ADS7843E_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->penchange)
        {
          ads7843e_notify(priv);
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
 * Name: ads7843e_register
 *
 * Description:
 *   Configure the ADS7843E to use the provided SPI device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   dev     - An SPI driver instance
 *   config  - Persistent board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int ads7843e_register(FAR struct spi_dev_s *dev,
                      FAR struct ads7843e_config_s *config, int minor)
{
  FAR struct ads7843e_dev_s *priv;
  char devname[DEV_NAMELEN];
#ifdef CONFIG_ADS7843E_MULTIPLE
  irqstate_t flags;
#endif
  int ret;

  ivdbg("dev: %p minor: %d\n", dev, minor);

  /* Debug-only sanity checks */

  DEBUGASSERT(dev != NULL && config != NULL && minor >= 0 && minor < 100);

  /* Create and initialize a ADS7843E device driver instance */

#ifndef CONFIG_ADS7843E_MULTIPLE
  priv = &g_ads7843e;
#else
  priv = (FAR struct ads7843e_dev_s *)kmalloc(sizeof(struct ads7843e_dev_s));
  if (!priv)
    {
      idbg("kmalloc(%d) failed\n", sizeof(struct ads7843e_dev_s));
      return -ENOMEM;
    }
#endif

  /* Initialize the ADS7843E device driver instance */

  memset(priv, 0, sizeof(struct ads7843e_dev_s));
  priv->spi    = dev;             /* Save the SPI device handle */
  priv->config = config;          /* Save the board configuration */
  priv->wdog   = wd_create();     /* Create a watchdog timer */
  sem_init(&priv->devsem,  0, 1); /* Initialize device structure semaphore */
  sem_init(&priv->waitsem, 0, 0); /* Initialize pen event wait semaphore */

  /* Make sure that interrupts are disabled */

  config->clear(config);
  config->enable(config, false);

  /* Attach the interrupt handler */

  ret = config->attach(config, ads7843e_interrupt);
  if (ret < 0)
    {
      idbg("Failed to attach interrupt\n");
      goto errout_with_priv;
    }

  /* Configure the SPI interface */

  ads7843e_configspi(dev);

  /* Enable the PEN IRQ */
  
  ads7843e_sendcmd(priv, ADS7843_CMD_ENABPINIRQ);

  /* Register the device as an input device */

  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, minor);
  ivdbg("Registering %s\n", devname);

  ret = register_driver(devname, &ads7843e_fops, 0666, priv);
  if (ret < 0)
    {
      idbg("register_driver() failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* If multiple ADS7843E devices are supported, then we will need to add
   * this new instance to a list of device instances so that it can be
   * found by the interrupt handler based on the recieved IRQ number.
   */

#ifdef CONFIG_ADS7843E_MULTIPLE
  priv->flink    = g_ads7843elist;
  g_ads7843elist = priv;
  irqrestore(flags);
#endif

  /* Schedule work to perform the initial sampling and to set the data
   * availability conditions.
   */

  ret = work_queue(&priv->work, ads7843e_worker, priv, 0);
  if (ret != 0)
    {
      idbg("Failed to queue work: %d\n", ret);
      goto errout_with_priv;
    }

  /* And return success (?) */

  return OK;

errout_with_priv:
  sem_destroy(&priv->devsem);
#ifdef CONFIG_ADS7843E_MULTIPLE
  kfree(priv);
#endif
  return ret;
}
