/****************************************************************************
 * apps/system/i2c/i2c_common.c
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
 * Name: common_args
 ****************************************************************************/

int common_args(FAR struct i2ctool_s *i2ctool, FAR char **arg)
{
  FAR char *ptr = *arg;
  long value;
  int ret;

  if (ptr[0] != '-')
    {
      goto invalid_argument;
    }

  switch (ptr[1])
    {
      case 'a':
        ret = arg_hex(arg, &value);
        if (value < CONFIG_I2CTOOL_MINADDR || value > CONFIG_I2CTOOL_MAXADDR)
          {
            goto out_of_range;
          }

        i2ctool->addr = (uint8_t) value;
        return ret;

      case 'b':
        ret = arg_decimal(arg, &value);
        if (value < CONFIG_I2CTOOL_MINBUS || value > CONFIG_I2CTOOL_MAXBUS)
          {
            goto out_of_range;
          }

        i2ctool->bus = (uint8_t) value;
        return ret;

      case 'f':
        ret = arg_decimal(arg, &value);
        if (value == 0)
          {
            goto out_of_range;
          }

        i2ctool->freq = value;
        return ret;

      case 'i':
        i2ctool->autoincr = true;
        return 1;

      case 'j':
        i2ctool->autoincr = false;
        return 1;

      case 'n':
        i2ctool->start = false;
        return 1;

      case 'r':
        ret = arg_hex(arg, &value);
        if (value < 0 || value > CONFIG_I2CTOOL_MAXREGADDR)
          {
            goto out_of_range;
          }

        i2ctool->regaddr = (uint8_t) value;
        return ret;

      case 's':
        i2ctool->start = true;
        return 1;

      case 'w':
        ret = arg_decimal(arg, &value);
        if (value != 8 && value != 16)
          {
            goto out_of_range;
          }

        i2ctool->width = (uint8_t) value;
        return ret;

      default:
        goto invalid_argument;
    }

invalid_argument:
  i2ctool_printf(i2ctool, g_i2carginvalid, ptr);
  return ERROR;

out_of_range:
  i2ctool_printf(i2ctool, g_i2cargrange, ptr);
  return ERROR;
}

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
    {
      *value = arg[1];
      return 2;
    }
  else
    {
      *value = &ptr[2];
      return 1;
    }
}

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/

int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}

/****************************************************************************
 * Name: arg_hex
 ****************************************************************************/

int arg_hex(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;
  
  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 16);
  return ret;
}
