/****************************************************************************
 * drivers/power/battery.c
 * Upper-half, character driver for batteries.
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

#include <errno.h>
#include <debug.h>

#include <nuttx/power/battery.h>

/* This driver requires:
 *
 * CONFIG_BATTERY - Upper half battery driver support
 */

#if defined(CONFIG_BATTERY)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/

struct bat_dev_s
{
  FAR struct battery_lower_s *lower; /* The lower half battery driver */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int     bat_open(FAR struct file *filep);
static int     bat_close(FAR struct file *filep);
static ssize_t bat_read(FAR struct file *, FAR char *, size_t);
static ssize_t bat_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     bat_ioctl(FAR struct file *filep,int cmd,unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_operations_s g_max1704xfopg =
{
  bat_open,
  bat_close,
  bat_read,
  bat_write,
  0,
  bat_ioctl
#ifndef CONFIG_DISABLE_POLL
  , 0
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: bat_open
 *
 * Description:
 *   This function is called whenever theMAX1704x device is opened.
 *
 ****************************************************************************/

static int bat_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: bat_close
 *
 * Description:
 *   This routine is called when theMAX1704x device is closed.
 *
 ****************************************************************************/

static int bat_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: bat_read
 ****************************************************************************/

static ssize_t bat_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: bat_write
 ****************************************************************************/

static ssize_t bat_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -EACCESS;
}

/****************************************************************************
 * Name: bat_ioctl
 ****************************************************************************/

static int bat_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bat_dev_s *upper  = inode->i_private;
  int ret   = -EINVAL;

  switch (cmd)
    {
      case BATIOC_STATE:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg));
          if (ptr)
            {
              *ptr = upper->state(priv->lower);
              ret = OK;
            }
        }
        break;

      case BATIOC_ONLINE:
          FAR bool *ptr = (FAR bool *)((uintptr_t)arg));
          if (ptr)
            {
              *ptr = upper->online(priv->lower);
              ret = OK;
            }
        break;

      case BATIOC_VOLTAGE:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg));
          if (ptr)
            {
              *ptr = upper->voltage(priv->lower);
              ret = OK;
            }
        }
        break;

      case BATIOC_CAPACITY:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg));
          if (ptr)
            {
              *ptr = upper->capacity(priv->lower);
              ret = OK;
            }
        }
        break;

      default:
        i2cdbg("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: battery_register
 *
 * Description:
 *   Register a lower half battery driver with the common, upper-half
 *   battery driver.
 *
 * Input parameters:
 *   devpath - The location in the pseudo-filesystem to create the driver.
 *     Recommended standard is "/dev/bat0", "/dev/bat1", etc.
 *   lower - The lower half battery state.
 *
 * Returned value:
 *    Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int battery_register(FAR const char *devpath, FAR struct battery_lower_s *lower)
{
  FAR struct bat_dev_s *upper;
  int ret;

  /* Initialize theMAX1704x device structure */

  upper = (FAR struct bat_dev_s *)kzalloc(sizeof(struct bat_dev_s));
  if (!upper)
    {
      i2cdbg("Failed to allocate instance\n");
      return -ENOMEM;
    }

  upper->lower = lower;

  /* Register the character driver */

  ret = register_driver(devpath, &g_max1704xfopg, 0555, upper);
  if (ret < 0)
    {
      i2cdbg("Failed to register driver: %d\n", ret);
      free(upper);
    }
  return ret;
}
#endif /* CONFIG_BATTERY */
