/****************************************************************************
 * drivers/sensors/qencoder.c
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
#include <nuttx/sensors/qencoder.h>

#include <arch/irq.h>

#ifdef CONFIG_QENCODER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing PWM */

#ifdef CONFIG_DEBUG_QENCODER
#  define qedbg    dbg
#  define qevdbg   vdbg
#  define qelldbg  lldbg
#  define qellvdbg llvdbg
#else
#  define qedbg(x...)
#  define qevdbg(x...)
#  define qelldbg(x...)
#  define qellvdbg(x...)
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half drivere */

struct qe_upperhalf_s
{
  uint8_t                    crefs;    /* The number of times the device has been opened */
  sem_t                      exclsem;  /* Supports mutual exclusion */
  FAR struct qe_lowerhalf_s *lower;    /* lower-half state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     qe_open(FAR struct file *filep);
static int     qe_close(FAR struct file *filep);
static ssize_t qe_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t qe_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     qe_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_qeops =
{
  qe_open,  /* open */
  qe_close, /* close */
  qe_read,  /* read */
  qe_write, /* write */
  0,         /* seek */
  qe_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0        /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: qe_open
 *
 * Description:
 *   This function is called whenever the PWM device is opened.
 *
 ************************************************************************************/

static int qe_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct qe_upperhalf_s *upper = inode->i_private;
  uint8_t                     tmp;
  int                         ret;

  qevdbg("crefs: %d\n", upper->crefs);

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

  /* Check if this is the first time that the driver has been opened. */

  if (tmp == 1)
    {
      FAR struct qe_lowerhalf_s *lower = upper->lower;

      /* Yes.. perform one time hardware initialization. */

      DEBUGASSERT(lower->ops->setup != NULL);
      qevdbg("calling setup\n");

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
  sem_post(&upper->exclsem);
  
errout:
  return ret;
}

/************************************************************************************
 * Name: qe_close
 *
 * Description:
 *   This function is called when the PWM device is closed.
 *
 ************************************************************************************/

static int qe_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct qe_upperhalf_s *upper = inode->i_private;
  int                         ret;

  qevdbg("crefs: %d\n", upper->crefs);

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

  if (upper->crefs > 1)
    {
      upper->crefs--;
    }
  else
    {
      FAR struct qe_lowerhalf_s *lower = upper->lower;

      /* There are no more references to the port */

      upper->crefs = 0;

      /* Disable the PWM device */

      DEBUGASSERT(lower->ops->shutdown != NULL);
      qevdbg("calling shutdown: %d\n");

      lower->ops->shutdown(lower);
    }
  ret = OK;

//errout_with_sem:
  sem_post(&upper->exclsem);
  
errout:
  return ret;
}

/************************************************************************************
 * Name: qe_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t qe_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/************************************************************************************
 * Name: qe_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t qe_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  /* Return a failure */

  return -EPERM;
}

/************************************************************************************
 * Name: qe_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the PWM work is done.
 *   
 ************************************************************************************/

static int qe_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode          *inode = filep->f_inode;
  FAR struct qe_upperhalf_s *upper = inode->i_private;
  FAR struct qe_lowerhalf_s *lower = upper->lower;
  int                        ret;

  qevdbg("cmd: %d arg: %ld\n", cmd, arg);
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
      /* QEIOC_POSITION - Get the current position from the encoder.
       *   Argument: int32_t pointer to the location to return the position.
       */

      case QEIOC_POSITION:
        {
          FAR int32_t *ptr = (FAR int32_t *)((uintptr_t)arg);
          DEBUGASSERT(lower->ops->position != NULL && ptr);
          ret = lower->ops->position(lower, ptr);
        }
        break;

      /* QEIOC_RESET - Reset the position to zero.
       *   Argument: None
       */

      case QEIOC_RESET:
        {
          DEBUGASSERT(lower->ops->reset != NULL);
          ret = lower->ops->reset(lower);
        }
        break;

      /* Any unrecognized IOCTL commands might be platform-specific ioctl commands */

      default:
        {
          qevdbg("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
          DEBUGASSERT(lower->ops->ioctl != NULL);
          ret = lower->ops->ioctl(lower, cmd, arg);
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
 * Name: qe_register
 *
 * Description:
 *   Register the Quadrature Encoder lower half device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   lower - An instance of the lower half interface
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.  The following
 *   possible error values may be returned (most are returned by
 *   register_driver()):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int qe_register(FAR const char *devpath, FAR struct qe_lowerhalf_s *lower)
{
  FAR struct qe_upperhalf_s *upper;

  /* Allocate the upper-half data structure */

  upper = (FAR struct qe_upperhalf_s *)zalloc(sizeof(struct qe_upperhalf_s));
  if (!upper)
    {
      qedbg("Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the PWM device structure (it was already zeroed by zalloc()) */

  sem_init(&upper->exclsem, 0, 1);
  upper->lower = lower;

  /* Register the PWM device */

  qevdbg("Registering %s\n", devpath);
  return register_driver(devpath, &g_qeops, 0666, upper);
}

#endif /* CONFIG_QENCODER */
