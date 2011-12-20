/****************************************************************************
 * drivers/pwm.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs.h>
#include <nuttx/arch.h>
#include <nuttx/pwm.h>

#include <arch/irq.h>

#ifdef CONFIG_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing PWM */

#ifdef CONFIG_DEBUG_PWM
#  define pwmdbg    dbg
#  define pwmvdbg   vdbg
#  define pwmlldbg  lldbg
#  define pwmllvdbg llvdbg
#else
#  define pwmdbg(x...)
#  define pwmvdbg(x...)
#  define pwmlldbg(x...)
#  define pwmllvdbg(x...)
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half drivere */

struct pwm_upperhalf_s
{
  uint8_t           crefs;          /* The number of times the device has been opened */
  bool              started;        /* True: pulsed output is being generated */
  sem_t             sem;            /* Supports mutual exclusion */
  struct pwm_info_s info;           /* Pulsed output characteristics */
  FAR struct pwm_lowerhalf_s *dev;  /* lower-half state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     pwm_open(FAR struct file *filep);
static int     pwm_close(FAR struct file *filep);
static ssize_t pwm_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t pwm_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     pwm_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pwmops =
{
  pwm_open,  /* open */
  pwm_close, /* close */
  pwm_read,  /* read */
  pwm_write, /* write */
  0,         /* seek */
  pwm_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0        /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: pwm_open
 *
 * Description:
 *   This function is called whenever the PWM device is opened.
 *
 ************************************************************************************/

static int pwm_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct pwm_upperhalf_s *upper = inode->i_private;
  uint8_t                     tmp;
  int                         ret;

  pwmvdbg("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->sem);
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

  /* Check if this is the first time that the driver has been opened. */

  if (tmp == 1)
    {
      FAR struct pwm_lowerhalf_s *lower = upper->dev;

      /* Yes.. perform one time hardware initialization. */

      DEBUGASSERT(lower->ops->setup != NULL);
      pwmvdbg("calling setup\n");

      ret = lower->ops->setup(lower);
      if (ret < 0)
        {
          goto errout_with_sem;
        }
    }

  /* Save the new open count on success */

  upper->crefs = tmp;
  ret = OK;

errout_with_sem:
  sem_post(&upper->sem);
  
errout:
  return ret;
}

/************************************************************************************
 * Name: pwm_close
 *
 * Description:
 *   This function is called when the PWM device is closed.
 *
 ************************************************************************************/

static int pwm_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct pwm_upperhalf_s *upper = inode->i_private;
  int                         ret;

  pwmvdbg("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->sem);
  if (ret < 0)
    {
      ret = -errno;
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 1)
    {
      upper->crefs--;
    }
  else
    {
      FAR struct pwm_lowerhalf_s *lower = upper->dev;

      /* There are no more references to the port */

      upper->crefs = 0;

      /* Disable the PWM device */

      DEBUGASSERT(lower->ops->shutdown != NULL);
      pwmvdbg("calling shutdown: %d\n");

      lower->ops->shutdown(lower);
    }
  ret = OK;

//errout_with_sem:
  sem_post(&upper->sem);
  
errout:
  return ret;
}

/************************************************************************************
 * Name: pwm_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t pwm_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/************************************************************************************
 * Name: pwm_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t pwm_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  return 0;
}

/************************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the PWM work is done.
 *   
 ************************************************************************************/

static int pwm_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct pwm_upperhalf_s *upper = inode->i_private;
  FAR struct pwm_lowerhalf_s *lower = upper->dev;
  int                         ret   = OK;

  /* Handle built-in ioctl commands */

  pwmvdbg("cmd: %d arg: %ld\n", cmd, arg);

  switch (cmd)
    {
      /* PWMIOC_SETCHARACTERISTICS - Set the characteristics of the next pulsed
       *   output.  This command will neither start nor stop the pulsed output.
       *   It will either setup the configuration that will be used when the
       *   output is started; or it will change the characteristics of the pulsed
       *   output on the fly if the timer is already started.
       *
       *   ioctl argument:  A read-only reference to struct pwm_info_s that provides
       *   the characteristics of the pulsed output.
       */

      case PWMIOC_SETCHARACTERISTICS:
        {
          FAR const struct pwm_info_s *info = (FAR const struct pwm_info_s*)((uintptr_t)arg);
          DEBUGASSERT(info != NULL && lower->ops->start != NULL);

          pwmvdbg("PWMIOC_SETCHARACTERISTICS frequency: %d duty: %08x started: %d\n",
                  info->frequency, info->duty, upper->started);

          memcpy(&upper->info, info, sizeof(struct pwm_info_s));
          if (upper->started)
            {
              ret = lower->ops->start(lower, &upper->info);
            }
        }
        break;

      /* PWMIOC_GETCHARACTERISTICS - Get the currently selected characteristics of
       *   the pulsed output (independent of whether the output is start or stopped).
       *
       *   ioctl argument:  A reference to struct pwm_info_s to recevie the
       *   characteristics of the pulsed output.
       */

      case PWMIOC_GETCHARACTERISTICS:
        {
          FAR struct pwm_info_s *info = (FAR struct pwm_info_s*)((uintptr_t)arg);
          DEBUGASSERT(info != NULL);

          memcpy(info, &upper->info, sizeof(struct pwm_info_s));
          pwmvdbg("PWMIOC_GETCHARACTERISTICS frequency: %d duty: %08x\n",
                  info->frequency, info->duty);
        }
        break;

      /* PWMIOC_START - Start the pulsed output.  The PWMIOC_SETCHARACTERISTICS
       *   command must have previously been sent.
       *
       *   ioctl argument:  None
       */

      case PWMIOC_START:
        {
          pwmvdbg("PWMIOC_START frequency: %d duty: %08x started: %d\n",
                  upper->info.frequency, upper->info.duty, upper->started);

          DEBUGASSERT(lower->ops->start != NULL);

          if (!upper->started)
            {
              ret = lower->ops->start(lower, &upper->info);
              upper->started = true;
            }
        }
        break;

      /* PWMIOC_STOP - Stop the pulsed output.
       *
       *   ioctl argument:  None
       */

      case PWMIOC_STOP:
        {
          pwmvdbg("PWMIOC_STOP: started: %d\n", upper->started);
          DEBUGASSERT(lower->ops->stop != NULL);

          if (upper->started)
            {
              ret = lower->ops->stop(lower);
              upper->started = false;
            }
        }
        break;

      /* Any unrecognized IOCTL commands might be platform-specific ioctl commands */

      default:
        {
          pwmvdbg("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
          DEBUGASSERT(lower->ops->ioctl != NULL);
          ret = lower->ops->ioctl(lower, cmd, arg);
        }
        break;
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_register
 *
 * Description:
 *   This function binds an instance of a "lower half" timer driver with the
 *   "upper half" PWM device and registers that device so that can be used
 *   by application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   reset state (as if the shutdown() method had already been called).
 *
 * Input parameters:
 *   path - The full path to the driver to be registers in the NuttX pseudo-
 *     filesystem.  The recommended convention is to name all PWM drivers
 *     as "/dev/pwm0", "/dev/pwm1", etc.  where the driver path differs only
 *     in the "minor" number at the end of the device name.
 *   dev - A pointer to an instance of lower half timer driver.  This instance
 *     is bound to the PWM driver and must persists as long as the driver
 *     persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pwm_register(FAR const char *path, FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct pwm_upperhalf_s *upper;

  /* Allocate the upper-half data structure */

  upper = (FAR struct pwm_upperhalf_s *)zalloc(sizeof(struct pwm_upperhalf_s));
  if (!upper)
    {
      pwmdbg("Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the PWM device structure (it was already zeroed by zalloc()) */

  sem_init(&upper->sem, 0, 1);
  upper->dev = dev;

  /* Register the PWM device */

  pwmvdbg("Registering %s\n", path);
  return register_driver(path, &g_pwmops, 0666, upper);
}

#endif /* CONFIG_PWM */
