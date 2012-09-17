/****************************************************************************
 * apps/system/i2c/i2c_verf.c
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
 * Name: i2ccmd_verf
 ****************************************************************************/

int i2ccmd_verf(FAR struct i2ctool_s *i2ctool, int argc, FAR char **argv)
{
  FAR struct i2c_dev_s *dev;
  FAR char *ptr;
  uint16_t rdvalue;
  uint8_t regaddr;
  bool addrinaddr;
  long wrvalue;
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

  /* The options may be followed by the optional wrvalue to be written.  If omitted, then
   * the register address will be used as the wrvalue, providing an address-in-address
   * test.
   */

  addrinaddr = true;
  wrvalue    = 0;

  if (argndx < argc)
    {
      wrvalue = strtol(argv[argndx], NULL, 16);
      if (i2ctool->width == 8)
        {
          if (wrvalue < 0 || wrvalue > 255)
            {
              i2ctool_printf(i2ctool, g_i2cargrange, argv[0]);
              return ERROR;
            }
        }
      else if (wrvalue < 0 || wrvalue > 65535)
        {
          i2ctool_printf(i2ctool, g_i2cargrange, argv[0]);
          return ERROR;
        }

      addrinaddr = false;
      argndx++;
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
      /* If we are performing an address-in-address test, then use the register
       * address as the value to write.
       */

      if (addrinaddr)
        {
          wrvalue = regaddr;
        }

      /* Write to the I2C bus */

      ret = i2ctool_set(i2ctool, dev, regaddr, (uint16_t)wrvalue);
      if (ret == OK)
        {
          /* Read the value back from the I2C bus */

          ret = i2ctool_get(i2ctool, dev, regaddr, &rdvalue);
        }

      /* Display the result */

      if (ret == OK)
        {
          i2ctool_printf(i2ctool, "VERIFY Bus: %d Addr: %02x Subaddr: %02x Wrote: ",
                         i2ctool->bus, i2ctool->addr, i2ctool->regaddr);

          if (i2ctool->width == 8)
            {
              i2ctool_printf(i2ctool, "%02x Read: %02x", (int)wrvalue, (int)rdvalue);
            }
          else
            {
              i2ctool_printf(i2ctool, "%04x Read: %04x", (int)wrvalue, (int)rdvalue);
            }

          if (wrvalue != rdvalue)
            {
              i2ctool_printf(i2ctool, "  <<< FAILURE\n");
            }
          else
            {
              i2ctool_printf(i2ctool, "\n");
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
