/****************************************************************************
 * drivers/power/max1704x.c
 * Lower half driver for MAX1704x battery charger
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
 * CONFIG_I2C - I2C support
 * CONFIG_I2C_MAX1704X - And the driver must be explictly selected.
 */

#if defined(CONFIG_BATTERY) && defined(CONFIG_I2C) && defined(CONFIG_I2C_MAX1704X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/

struct max1704x_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  FAR const struct battery_operations_s *ops; /* Battery operations */
  sem_t batsem;  /* Enforce mutually exclusive access */

  /* Data fields specific to the lower half MAX1704x driver follow */

  FAR struct i2c_dev_s *i2c; /* I2C interface */
  uint8_t addr;              /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Battery driver lower half methods */

static enum battery_status_e mx1704x_state(struct battery_dev_s *lower);
static bool mx1704x_online(struct battery_dev_s *lower);
static int mx1704x_voltage(struct battery_dev_s *lower);
static int mx1704x_capacity(struct battery_dev_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_operations_s g_max1704xops =
{
  mx1704x_state,
  mx1704x_online,
  mx1704x_voltage,
  mx1704x_capacity
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max1704x_open
 *
 * Description:
 *   Return the current battery state
 *
 ****************************************************************************/

static enum battery_status_e state(struct battery_dev_s *lower)
{
#warning "Missing logic"
  return BATTERY_UNKNOWN;
}

/****************************************************************************
 * Name: max1704x_open
 *
 * Description:
 *   Return true if the batter is online
 *
 ****************************************************************************/

static bool online(struct battery_dev_s *lower)
{
#warning "Missing logic"
  return false;
}

/****************************************************************************
 * Name: max1704x_open
 *
 * Description:
 *   Current battery voltage
 *
 ****************************************************************************/

static int voltage(struct battery_dev_s *lower);
{
#warning "Missing logic"
  return 0;
}

/****************************************************************************
 * Name: max1704x_open
 *
 * Description:
 *   Battery capacity
 *
 ****************************************************************************/

static int capacity(struct battery_dev_s *lower);
{
#warning "Missing logic"
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max1704x_initialize
 *
 * Description:
 *   Initialize the MAX1704x battery driver and return an instance of the
 *   lower_half interface that may be used with battery_register();
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY - Upper half battery driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_I2C_MAX1704X - And the driver must be explictly selected.
 *
 * Input Parameters:
 *   i2c - An instance of the I2C interface to use to communicate with the MAX1704x
 *   addr - The I2C address of the MAX1704x.
 *
 * Returned Value:
 *   A pointer to the intialized lower-half driver instance.  A NULL pointer
 *   is returned on a failure to initialize the MAX1704x lower half.
 *
 ****************************************************************************/

FAR struct battery_dev_s *max1704x_initialize(FAR struct i2c_dev_s *i2c,
                            uint8_t addr)
{
  FAR struct max1704x_dev_s *priv;
  int ret;

  /* Initialize theMAX1704x device structure */

  priv = (FAR struct max1704x_dev_s *)kzalloc(sizeof(struct max1704x_dev_s));
  if (priv)
    {
      sem_init(&priv->batsem, 0, 1);
      priv->ops  = &g_max1704xops;
      priv->i2c  = i2c;
      priv->addr = addr;
    }
  return (FAR struct battery_dev_s *)priv;
}

#endif /* CONFIG_BATTERY && CONFIG_I2C && CONFIG_I2C_MAX1704X */
