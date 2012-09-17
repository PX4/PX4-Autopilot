/****************************************************************************
 * apps/system/i2c/i2c_get.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>

#include <nuttx/i2c.h>

#include "i2ctool.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2ccmd_get
 ****************************************************************************/

int i2ccmd_get(FAR struct i2ctool_s *i2ctool, int argc, FAR char **argv)
{
  FAR struct i2c_dev_s *dev;
  FAR char *ptr;
  uint16_t result;
  uint8_t regaddr;
  long repititions;
  int nargs;
  int argndx;
  int ret;
  int i;

  /* Parse any command line arguments */

  for (argndx = 1; argndx < argc; )
    {
      /* Break out of the look when the last option has been parsed */

      ptr = argv[argndx];
      if (*ptr != '-')
        {
          break;
        }

      /* Otherwise, check for common options */

      nargs = common_args(i2ctool, &argv[argndx]);
      if (nargs < 0)
        {
          return ERROR;
        }
      argndx += nargs;
    }

  /* There may be one more thing on the command line:  The repitition
   * count.
   */

  repititions = 1;
  if (argndx < argc)
    {
      repititions = strtol(argv[argndx], NULL, 16);
      if (repititions < 1)
        {
          i2ctool_printf(i2ctool, g_i2cargrange, argv[0]);
          return ERROR;
        }

      argndx++;
    }

  if (argndx != argc)
    {
      i2ctool_printf(i2ctool, g_i2ctoomanyargs, argv[0]);
      return ERROR;
    }

  /* Get a handle to the I2C bus */

  dev = up_i2cinitialize(i2ctool->bus);
  if (!dev)
    {
       i2ctool_printf(i2ctool, "Failed to get bus %d\n", i2ctool->bus);
       return ERROR;
    }

  /* Set the frequency and the address (NOTE:  Only 7-bit address supported now) */

  I2C_SETFREQUENCY(dev, i2ctool->freq);
  I2C_SETADDRESS(dev, i2ctool->addr, 7);

  /* Loop for the requested number of repititions */

  regaddr = i2ctool->regaddr;
  ret = OK;

  for (i = 0; i < repititions; i++)
    {
      /* Read from the I2C bus */

      ret = i2ctool_get(i2ctool, dev, regaddr, &result);

      /* Display the result */

      if (ret == OK)
        {
          i2ctool_printf(i2ctool, "READ Bus: %d Addr: %02x Subaddr: %02x Value: ",
                         i2ctool->bus, i2ctool->addr, i2ctool->regaddr);
          if (i2ctool->width == 8)
            {
              i2ctool_printf(i2ctool, "%02x\n", result);
            }
          else
            {
              i2ctool_printf(i2ctool, "%04x\n", result);
            }
        }
      else
        {
          i2ctool_printf(i2ctool, g_i2cxfrerror, argv[0], -ret);
          break;
        }

      /* Auto-increment the address if so configured */

      if (i2ctool->autoincr)
        {
          regaddr++;
        }
    }

  (void)up_i2cuninitialize(dev);
  return ret;
}

/****************************************************************************
 * Name: i2ctool_get
 ****************************************************************************/

int i2ctool_get(FAR struct i2ctool_s *i2ctool, FAR struct i2c_dev_s *dev,
                uint8_t regaddr, uint16_t *result)
{
  struct i2c_msg_s msg[2];
  union
  {
    uint16_t data16;
    uint8_t  data8;
  } u;
  int ret;

  /* Set up data structures */

  msg[0].addr   = i2ctool->addr;
  msg[0].flags  = 0;
  msg[0].buffer = &regaddr;
  msg[0].length = 1;

  msg[1].addr   = i2ctool->addr;
  msg[1].flags  = I2C_M_READ;
  if (i2ctool->width == 8)
    {
      msg[1].buffer = &u.data8;
      msg[1].length = 1;
    }
  else
    {
      msg[1].buffer = (uint8_t*)&u.data16;
      msg[1].length = 2;
    }

  if (i2ctool->start)
    {
      ret = I2C_TRANSFER(dev, &msg[0], 1);
      if (ret== OK)
        {
          ret = I2C_TRANSFER(dev, &msg[1], 1);
        }
    }
  else
    {
      ret = I2C_TRANSFER(dev, msg, 2);
    }

  /* Return the result of the read operation */

  if (ret == OK)
    {
      if (i2ctool->width == 8)
        {
          *result = (uint16_t)u.data8;
        }
      else
        {
          *result =  u.data16;
        }
    }
  return ret;
}
