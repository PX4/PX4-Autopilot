/****************************************************************************
 * drivers/watchdog.c
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
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/watchdog.h>

#ifdef CONFIG_WATCHDOG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the watchdog driver */

#ifdef CONFIG_DEBUG_WATCHDOG
#  define wddbg    dbg
#  define wdvdbg   vdbg
#  define wdlldbg  lldbg
#  define wdllvdbg llvdbg
#else
#  define wddbg(x...)
#  define wdvdbg(x...)
#  define wdlldbg(x...)
#  define wdllvdbg(x...)
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct watchdog_upperhalf_s
{
  uint8_t   crefs;    /* The number of times the device has been opened */
  sem_t     exclsem;  /* Supports mutual exclusion */
  FAR char *path;     /* Registration path */

  /* The contained lower-half driver */
 
  FAR struct watchdog_lowerhalf_s *lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     wdog_open(FAR struct file *filep);
static int     wdog_close(FAR struct file *filep);
static ssize_t wdog_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t wdog_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     wdog_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_wdogops =
{
  wdog_open,  /* open */
  wdog_close, /* close */
  wdog_read,  /* read */
  wdog_write, /* write */
  0,          /* seek */
  wdog_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0         /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: wdog_open
 *
 * Description:
 *   This function is called whenever the watchdog timer device is opened.
 *
 ************************************************************************************/

static int wdog_open(FAR struct file *filep)
{
  FAR struct inode                *inode = filep->f_inode;
  FAR struct watchdog_upperhalf_s *upper = inode->i_private;
  uint8_t                          tmp;
  int                              ret;

  wdvdbg("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -errno;
      goto errout;
    }

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Save the new open count */

  upper->crefs = tmp;
  ret = OK;

errout_with_sem:
  sem_post(&upper->exclsem);
  
errout:
  return ret;
}

/************************************************************************************
 * Name: wdog_close
 *
 * Description:
 *   This function is called when the watchdog timer device is closed.
 *
 ************************************************************************************/

static int wdog_close(FAR struct file *filep)
{
  FAR struct inode                *inode = filep->f_inode;
  FAR struct watchdog_upperhalf_s *upper = inode->i_private;
  int                              ret;

  wdvdbg("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -errno;
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 0)
    {
      upper->crefs--;
    }

  sem_post(&upper->exclsem);
  ret = OK;
  
errout:
  return ret;
}

/************************************************************************************
 * Name: wdog_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t wdog_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/************************************************************************************
 * Name: wdog_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t wdog_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  return 0;
}

/************************************************************************************
 * Name: wdog_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the watchdog timer work is
 *   done.
 *   
 ************************************************************************************/

static int wdog_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode                *inode = filep->f_inode;
  FAR struct watchdog_upperhalf_s *upper = inode->i_private;
  FAR struct watchdog_lowerhalf_s *lower = upper->lower;
  int                              ret;

  wdvdbg("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(upper && lower);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
    /* cmd:         WDIOC_START
     * Description: Start the watchdog timer
     * Argument:    Ignored
     */

    case WDIOC_START:
      {
        /* Start the watchdog timer, resetting the time to the current timeout */

        DEBUGASSERT(lower->ops->start); /* Required */
        ret = lower->ops->start(lower);
      }
      break;

    /* cmd:         WDIOC_STOP
     * Description: Stop the watchdog timer
     * Argument:    Ignored
     */

    case WDIOC_STOP:
      {
        /* Stop the watchdog timer */

        DEBUGASSERT(lower->ops->stop); /* Required */
        ret = lower->ops->stop(lower);
      }
      break;

    /* cmd:         WDIOC_GETSTATUS
     * Description: et the status of the watchdog timer.
     * Argument:    A writeable pointer to struct watchdog_status_s.
     */

    case WDIOC_GETSTATUS:
      {
        FAR struct watchdog_status_s *status;

        /* Get the current watchdog timer status */

        if (lower->ops->getstatus) /* Optional */
          {
            status = (FAR struct watchdog_status_s *)((uintptr_t)arg);
            if (status)
              {
                ret = lower->ops->getstatus(lower, status);
              }
            else
              {
                ret = -EINVAL;
              }
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;

    /* cmd:         WDIOC_SETTIMEOUT
     * Description: Reset the watchdog timeout to this value
     * Argument:    A 32-bit timeout value in milliseconds.
     */

    case WDIOC_SETTIMEOUT:
      {
        /* Set a new timeout value (and reset the watchdog timer) */

        if (lower->ops->settimeout) /* Optional */
          {
            ret = lower->ops->settimeout(lower, (uint32_t)arg);
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;

    /* cmd:         WDIOC_CAPTURE
     * Description: Do not reset.  Instead, called this handler.
     * Argument:    A pointer to struct watchdog_capture_s.
     */

    case WDIOC_CAPTURE:
      {
        FAR struct watchdog_capture_s *capture;

        /* Don't reset on watchdog timer timeout; instead, call this user
         * provider timeout handler.  NOTE:  Providing handler==NULL will
         * restore the reset behavior.
         */

        if (lower->ops->capture) /* Optional */
          {
            capture = (FAR struct watchdog_capture_s *)((uintptr_t)arg);
            if (capture)
              {
                capture->oldhandler =
                  lower->ops->capture(lower, capture->newhandler);
                ret = OK;
              }
            else
              {
                ret = -EINVAL;
              }
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;

    /* cmd:         WDIOC_KEEPALIVE
     * Description: Reset the watchdog timer ("ping", "pet the dog")
     * Argument:    Argument: Ignored
     */

    case WDIOC_KEEPALIVE:
      {
        /* Reset the watchdog timer to the current timeout value, prevent any
         * imminent watchdog timeouts.  This is sometimes referred as
         * "pinging" the watchdog timer or "petting the dog".
         */

        if (lower->ops->keepalive) /* Optional */
          {
            ret = lower->ops->keepalive(lower);
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;


    /* Any unrecognized IOCTL commands might be platform-specific ioctl commands */

    default:
      {
        wdvdbg("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);

        /* An ioctl commands that are not recognized by the "upper-half"
         * driver are forwarded to the lower half driver through this
         * method.
         */

        if (lower->ops->ioctl) /* Optional */
          {
            ret = lower->ops->ioctl(lower, cmd, arg);
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;
    }

  sem_post(&upper->exclsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: watchdog_register
 *
 * Description:
 *   This function binds an instance of a "lower half" watchdog driver with the
 *   "upper half" watchdog device and registers that device so that can be used
 *   by application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   disabled state (as if the stop() method had already been called).
 *
 * Input parameters:
 *   dev path - The full path to the driver to be registers in the NuttX
 *     pseudo-filesystem.  The recommended convention is to name all watchdog
 *     drivers as "/dev/watchdog0", "/dev/watchdog1", etc.  where the driver
 *     path differs only in the "minor" number at the end of the device name.
 *   lower - A pointer to an instance of lower half watchdog driver.  This
 *     instance is bound to the watchdog driver and must persists as long as
 *     the driver persists.
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned to the caller.  In the event
 *   of any failure, a NULL value is returned.
 *
 ****************************************************************************/

FAR void *watchdog_register(FAR const char *path,
                            FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct watchdog_upperhalf_s *upper;
  int ret;

  DEBUGASSERT(path && lower);
  wdvdbg("Entry: path=%s\n", path);

  /* Allocate the upper-half data structure */

  upper = (FAR struct watchdog_upperhalf_s *)
    kzalloc(sizeof(struct watchdog_upperhalf_s));
  if (!upper)
    {
      wddbg("Upper half allocation failed\n");
      goto errout;
    }

  /* Initialize the watchdog timer device structure (it was already zeroed
   * by kzalloc()).
   */

  sem_init(&upper->exclsem, 0, 1);
  upper->lower = lower;

  /* Copy the registration path */

  upper->path = strdup(path);
  if (!upper->path)
    {
      wddbg("Path allocation failed\n");
      goto errout_with_upper;
    }

  /* Register the watchdog timer device */

  ret = register_driver(path, &g_wdogops, 0666, upper);
  if (ret < 0)
    {
      wddbg("register_driver failed: %d\n", ret);
      goto errout_with_path;
    }

  return (FAR void *)upper;

errout_with_path:
  kfree(upper->path);

errout_with_upper:
  sem_destroy(&upper->exclsem);
  kfree(upper);

errout:
  return NULL;
}

/****************************************************************************
 * Name: watchdog_unregister
 *
 * Description:
 *   This function can be called to disable and unregister the watchdog
 *   device driver.
 *
 * Input parameters:
 *   handle - This is the handle that was returned by watchdog_register()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void watchdog_unregister(FAR void *handle)
{
  FAR struct watchdog_upperhalf_s *upper;
  FAR struct watchdog_lowerhalf_s *lower;

  /* Recover the pointer to the upper-half driver state */

  upper = (FAR struct watchdog_upperhalf_s *)handle;
  lower = upper->lower;
  DEBUGASSERT(upper && lower);

  wdvdbg("Unregistering: %s\n", upper->path);

  /* Disable the watchdog timer */

  DEBUGASSERT(lower->ops->stop); /* Required */
  (void)lower->ops->stop(lower);

  /* Unregister the watchdog timer device */

  (void)unregister_driver(upper->path);

  /* Then free all of the driver resources */

  kfree(upper->path);
  sem_destroy(&upper->exclsem);
  kfree(upper);
}

#endif /* CONFIG_WATCHDOG */
