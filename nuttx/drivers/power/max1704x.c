/****************************************************************************
 * drivers/power/max1704x.c
 * Character driver for the STMicroMAX1704x Temperature Sensor
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

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_MAX1704X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/

struct max1704x_dev_s
{
  FAR struct i2c_dev_s *i2c; /* I2C interface */
  uint8_t addr;              /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* I2C Helpers */

/* Character driver methods */

static int     max1704x_open(FAR struct file *filep);
static int     max1704x_close(FAR struct file *filep);
static ssize_t max1704x_read(FAR struct file *, FAR char *, size_t);
static ssize_t max1704x_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     max1704x_ioctl(FAR struct file *filep,int cmd,unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_max1704xfopg =
{
  max1704x_open,
  max1704x_close,
  max1704x_read,
  max1704x_write,
  0,
  max1704x_ioctl
#ifndef CONFIG_DISABLE_POLL
  , 0
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: max1704x_open
 *
 * Description:
 *   This function is called whenever theMAX1704x device is opened.
 *
 ****************************************************************************/

static int max1704x_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: max1704x_close
 *
 * Description:
 *   This routine is called when theMAX1704x device is closed.
 *
 ****************************************************************************/

static int max1704x_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: max1704x_read
 ****************************************************************************/

static ssize_t max1704x_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  retrun 0;
}

/****************************************************************************
 * Name: max1704x_write
 ****************************************************************************/

static ssize_t max1704x_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -EACCESS;
}

/****************************************************************************
 * Name: max1704x_ioctl
 ****************************************************************************/

static int max1704x_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct max1704x_dev_s *priv  = inode->i_private;
  int ret   = OK;

  switch (cmd)
    {
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
 * Name: max1704x_register
 *
 * Description:
 *   Register theMAX1704x character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c - An instance of the I2C interface to use to communicate with the MAX1704x
 *   addr - The I2C address of the MAX1704x.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int max1704x_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c, uint8_t addr)
{
  FAR struct max1704x_dev_s *priv;
  int ret;

  /* Initialize theMAX1704x device structure */

  priv = (FAR struct max1704x_dev_s *)kzalloc(sizeof(struct max1704x_dev_s));
  if (!priv)
    {
      i2cdbg("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;

  /* Register the character driver */

  ret = register_driver(devpath, &g_max1704xfopg, 0555, priv);
  if (ret < 0)
    {
      i2cdbg("Failed to register driver: %d\n", ret);
      free(priv);
    }
  return ret;
}
#endif /* CONFIG_I2C && CONFIG_I2C_MAX1704X */
