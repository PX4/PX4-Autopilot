/****************************************************************************
 * drivers/sensors/lm75.c
 * Character driver for the STMicro LM-75 Temperature Sensor
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs.h>
#include <nuttx/i2c.h>
#include <nuttx/sensors/lm75.h>

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_LM75)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Centigrade to Fahrenheit conversion:  F = 9*C/5 + 32 */

#define B16_9DIV5  (9 * 65536 / 5)
#define B16_32     (32 * 65536)

/* Debug for this file only */

#ifdef CONFIG_DEBUG_LM75
#  define lm75dbg dbg
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define lm75dbg(x...)
#  else
#    define lm75dbg (void)
#  endif
#endif

/****************************************************************************
 * Private
 ****************************************************************************/

struct lm75_dev_s
{
  FAR struct i2c_dev_s *i2c; /* I2C interface */
  uint8_t addr;              /* I2C address */
  bool fahrenheit;           /* true: temperature will be reported in fahrenheit */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* I2C Helpers */

static int lm75_readb16(FAR struct lm75_dev_s *priv, uint8_t regaddr,
                        FAR b16_t *regvalue);
static int lm75_writeb16(FAR struct lm75_dev_s *priv, uint8_t regaddr,
                         b16_t regval);
static int lm75_readtemp(FAR struct lm75_dev_s *priv, FAR b16_t *temp);
static int lm75_readconf(FAR struct lm75_dev_s *priv, FAR uint8_t *conf);
static int lm75_writeconf(FAR struct lm75_dev_s *priv, uint8_t conf);

/* Character driver methods */

static int     lm75_open(FAR struct file *filep);
static int     lm75_close(FAR struct file *filep);
static ssize_t lm75_read(FAR struct file *, FAR char *, size_t);
static ssize_t lm75_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     lm75_ioctl(FAR struct file *filep,int cmd,unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations lm75_fops =
{
  lm75_open,
  lm75_close,
  lm75_read,
  lm75_write,
  0,
  lm75_ioctl
#ifndef CONFIG_DISABLE_POLL
  , 0
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: lm75_readb16
 *
 * Description:
 *   Read a 16-bit register (LM75_TEMP_REG, LM75_THYS_REG, or LM75_TOS_REG)
 *
 ****************************************************************************/

static int lm75_readb16(FAR struct lm75_dev_s *priv, uint8_t regaddr,
                        FAR b16_t *regvalue)
{
  uint8_t buffer[2];
  int ret;

  /* Write the register address */

  I2C_SETADDRESS(priv->i2c, priv->addr, 7);
  ret = I2C_WRITE(priv->i2c, &regaddr, 1);
  if (ret < 0)
    {
      lm75dbg("I2C_WRITE failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 16-bits from the register (discarding 7) */

  ret = I2C_READ(priv->i2c, buffer, 2);
  if (ret < 0)
    {
      lm75dbg("I2C_READ failed: %d\n", ret);
      return ret;
    }

  /* Data format is:  TTTTTTTT Txxxxxxx where TTTTTTTTT is a nine-bit,
   * signed temperature value with LSB = 0.5 degrees centigrade.  So the
   * raw data is b8_t
   */

  *regvalue = b8tob16((b8_t)buffer[0] << 8 | (b8_t)buffer[1]);
  lm75dbg("addr: %02x value: %08x ret: %d\n", regaddr, *regvalue, ret);
  return OK;
}

/****************************************************************************
 * Name: lm75_writeb16
 *
 * Description:
 *   Write to a 16-bit register (LM75_TEMP_REG, LM75_THYS_REG, or LM75_TOS_REG)
 *
 ****************************************************************************/

static int lm75_writeb16(FAR struct lm75_dev_s *priv, uint8_t regaddr,
                         b16_t regval)
{
  uint8_t buffer[3];
  b8_t regb8;

  lm75dbg("addr: %02x value: %08x\n", regaddr, regval);

  /* Set up a 3 byte message to send */

  buffer[0] = regaddr;

  regb8 = b16tob8(regval);
  buffer[1] = (uint8_t)(regb8 >> 8);
  buffer[2] = (uint8_t)regb8;

  /* Write the register address followed by the data (no RESTART) */

  I2C_SETADDRESS(priv->i2c, priv->addr, 7);
  return I2C_WRITE(priv->i2c, buffer, 3);
}

/****************************************************************************
 * Name: lm75_readtemp
 *
 * Description:
 *   Read the temperature register with special scaling (LM75_TEMP_REG)
 *
 ****************************************************************************/

static int lm75_readtemp(FAR struct lm75_dev_s *priv, FAR b16_t *temp)
{
  b16_t temp16;
  int ret;

  /* Read the raw temperature data (b16_t) */

  ret = lm75_readb16(priv, LM75_TEMP_REG, &temp16);
  if (ret < 0)
    {
      lm75dbg("lm75_readb16 failed: %d\n", ret);
      return ret;
    }
  lm75dbg("Centigrade: %08x\n", temp16);

  /* Was fahrenheit requested? */

  if (priv->fahrenheit)
    {
      /* Centigrade to Fahrenheit conversion:  F = 9*C/5 + 32 */

      temp16 =  b16mulb16(temp16, B16_9DIV5) + B16_32;
      lm75dbg("Fahrenheit: %08x\n", temp16);
    }

  *temp = temp16;
  return OK;
}

/****************************************************************************
 * Name: lm75_readconf
 *
 * Description:
 *   Read the 8-bit LM75 configuration register
 *
 ****************************************************************************/

static int lm75_readconf(FAR struct lm75_dev_s *priv, FAR uint8_t *conf)
{
  uint8_t buffer;
  int ret;

  /* Write the configuration register address */

  I2C_SETADDRESS(priv->i2c, priv->addr, 7);

  buffer = LM75_CONF_REG;
  ret = I2C_WRITE(priv->i2c, &buffer, 1);
  if (ret < 0)
    {
      lm75dbg("I2C_WRITE failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8-bits from the register */

  ret = I2C_READ(priv->i2c, conf, 1);
  lm75dbg("conf: %02x ret: %d\n", *conf, ret);
  return ret;
}

/****************************************************************************
 * Name: lm75_writeconf
 *
 * Description:
 *   Write to a 8-bit LM75 configuration register.
 *
 ****************************************************************************/

static int lm75_writeconf(FAR struct lm75_dev_s *priv, uint8_t conf)
{
  uint8_t buffer[2];

  lm75dbg("conf: %02x\n", conf);

  /* Set up a 2 byte message to send */

  buffer[0] = LM75_CONF_REG;
  buffer[1] = conf;

  /* Write the register address followed by the data (no RESTART) */

  I2C_SETADDRESS(priv->i2c, priv->addr, 7);
  return I2C_WRITE(priv->i2c, buffer, 2);
}

/****************************************************************************
 * Name: lm75_open
 *
 * Description:
 *   This function is called whenever the LM-75 device is opened.
 *
 ****************************************************************************/

static int lm75_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: lm75_close
 *
 * Description:
 *   This routine is called when the LM-75 device is closed.
 *
 ****************************************************************************/

static int lm75_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: lm75_read
 ****************************************************************************/

static ssize_t lm75_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct lm75_dev_s *priv   = inode->i_private;
  FAR b16_t             *ptr;
  ssize_t                nsamples;
  int                    i;
  int                    ret;

  /* How many samples were requested to get? */

  nsamples = buflen / sizeof(b16_t);
  ptr      = (FAR b16_t *)buffer;

  lm75dbg("buflen: %d nsamples: %d\n", buflen, nsamples);

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      b16_t temp;

      /* Read the next b16_t temperature value */

      ret = lm75_readtemp(priv, &temp);
      if (ret < 0)
        {
          lm75dbg("lm75_readtemp failed: %d\n",ret);
          return (ssize_t)ret;
        }

      /* Save the temperature value in the user buffer */

      *ptr++ = temp;
    }

  return nsamples * sizeof(b16_t);
}

/****************************************************************************
 * Name: lm75_write
 ****************************************************************************/

static ssize_t lm75_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lm75_ioctl
 ****************************************************************************/

static int lm75_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct lm75_dev_s *priv  = inode->i_private;
  int                    ret   = OK;

  switch (cmd)
    {
      /* Read from the configuration register. Arg: uint8_t* pointer */

      case SNIOC_READCONF:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          ret = lm75_readconf(priv, ptr);
          lm75dbg("conf: %02x ret: %d\n", *ptr, ret);
        }
        break;

      /* Wrtie to the configuration register. Arg:  uint8_t value */

      case SNIOC_WRITECONF:
        ret = lm75_writeconf(priv, (uint8_t)arg);
        lm75dbg("conf: %02x ret: %d\n", *(uint8_t*)arg, ret);
        break;

      /* Shutdown the LM75, Arg: None */

      case SNIOC_SHUTDOWN:
        {
          uint8_t conf;
          ret = lm75_readconf(priv, &conf);
          if (ret == OK)
            {
              ret = lm75_writeconf(priv, conf | LM75_CONF_SHUTDOWN);
            }
          lm75dbg("conf: %02x ret: %d\n", conf | LM75_CONF_SHUTDOWN, ret);
        }
        break;

      /* Powerup the LM75, Arg: None */

      case SNIOC_POWERUP:
        {
          uint8_t conf;
          ret = lm75_readconf(priv, &conf);
          if (ret == OK)
            {
              ret = lm75_writeconf(priv, conf & ~LM75_CONF_SHUTDOWN);
            }
          lm75dbg("conf: %02x ret: %d\n", conf & ~LM75_CONF_SHUTDOWN, ret);
        }
        break;

      /* Report samples in Fahrenheit */

      case SNIOC_FAHRENHEIT:
        priv->fahrenheit = true;
        lm75dbg("Fahrenheit\n");
        break;

      /* Report Samples in Centigrade */

      case SNIOC_CENTIGRADE:
        priv->fahrenheit = false;
        lm75dbg("Centigrade\n");
        break;

      /* Read THYS temperature register.  Arg: b16_t* pointer */

      case SNIOC_READTHYS:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          ret = lm75_readb16(priv, LM75_THYS_REG, ptr);
          lm75dbg("THYS: %08x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write THYS temperature register. Arg: b16_t value */

      case SNIOC_WRITETHYS:
        ret = lm75_writeb16(priv, LM75_THYS_REG, (b16_t)arg);
        lm75dbg("THYS: %08x ret: %d\n", (b16_t)arg, ret);
        break;

      /* Read TOS (Over-temp Shutdown Threshold) Register. Arg: b16_t* pointer */

      case SNIOC_READTOS:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          ret = lm75_readb16(priv, LM75_TOS_REG, ptr);
          lm75dbg("TOS: %08x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write TOS (Over-temp Shutdown Threshold) Register. Arg: b16_t value */

      case SNIOC_WRITRETOS:
        ret = lm75_writeb16(priv, LM75_TOS_REG, (b16_t)arg);
        lm75dbg("TOS: %08x ret: %d\n", (b16_t)arg, ret);
        break;

      default:
        lm75dbg("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm75_register
 *
 * Description:
 *   Register the LM-75 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c - An instance of the I2C interface to use to communicate with LM75
 *   addr - The I2C address of the LM-75.  The base I2C address of the LM75
 *   is 0x48.  Bits 0-3 can be controlled to get 8 unique addresses from 0x48
 *   through 0x4f.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lm75_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c, uint8_t addr)
{
  FAR struct lm75_dev_s *priv;
  int ret;

  /* Initialize the LM-75 device structure */

  priv = (FAR struct lm75_dev_s *)malloc(sizeof(struct lm75_dev_s));
  if (!priv)
    {
      lm75dbg("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->fahrenheit = false;

  /* Register the character driver */

  ret = register_driver(devpath, &lm75_fops, 0555, priv);
  if (ret < 0)
    {
      lm75dbg("Failed to register driver: %d\n", ret);
      free(priv);
    }
  return ret;
}
#endif /* CONFIG_I2C && CONFIG_I2C_LM75 */
