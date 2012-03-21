/****************************************************************************
 * include/nuttx/power/battery.h
 * NuttX Battery Interfaces
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

#ifndef __INCLUDE_NUTTX_POWER_BATTERY_H
#define __INCLUDE_NUTTX_POWER_BATTERY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#include <stdbool.h>
#include <semaphore.h>
#include <fixedmath.h>

#ifdef CONFIG_BATTERY

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_BATTERY - Upper half battery driver support
 *
 * Specific, lower-half drivers will have other configuration requirements
 * such as:
 *
 *   CONFIG_I2C - I2C support *may* be needed
 *   CONFIG_I2C_MAX1704X - The MAX1704x driver must be explictly selected.
 */

/* IOCTL Commands ***********************************************************/
/* The upper-half battery driver provides a character driver "wrapper"
 * around the lower-half battery driver that does all of the real work.
 * Since there is no real data transfer to/or from a battery, all of the
 * driver interaction is through IOCTIL commands.  The IOCTL commands
 * supported by the upper-half driver simply provide calls into the the
 * lower half as summarized below:
 *
 * BATIOC_STATE - Return the current state of the battery (see
 *   enum battery_status_e).
 *   Input value:  A pointer to type int.
 * BATIOC_ONLINE - Return 1 if the battery is online; 0 if offline.
 *   Input value:  A pointer to type bool.
 * BATIOC_VOLTAGE - Return the current battery voltage.  The returned value
 *   is a fixed preceision number in units of volts.
 *   Input value:  A pointer to type b16_t.
 * BATIOC_CAPACITY - Return the current battery capacity or State of Charge
 *   (SoC).  The returned value is a fixed precision percentage of the
 *   batteries full capacity.
 *   Input value:  A pointer to type b16_t.
 */

#define BATIOC_STATE    _BATIOC(0x0001)
#define BATIOC_ONLINE   _BATIOC(0x0002)
#define BATIOC_VOLTAGE  _BATIOC(0x0003)
#define BATIOC_CAPACITY _BATIOC(0x0004)

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* Battery status */

enum battery_status_e
{
  BATTERY_UNKNOWN = 0, /* Battery state is not known */
  BATTERY_IDLE,        /* Not full, not charging, not discharging */
  BATTERY_FULL,        /* Full, not discharging */
  BATTERY_CHARGING,    /* Not full, charging */
  BATTERY_DISCHARGING  /* Probably not full, discharging */
};

 /* This structure defines the lower half battery interface */

struct battery_dev_s;
struct battery_operations_s
{
  /* Return the current battery state (see enum battery_status_e) */

  int (*state)(struct battery_dev_s *dev, int *status);

  /* Return true if the batter is online */

  int (*online)(struct battery_dev_s *dev, bool *status);

  /* Current battery voltage */

  int (*voltage)(struct battery_dev_s *dev, b16_t *value);

  /* Battery capacity */

  int (*capacity)(struct battery_dev_s *dev, b16_t *value);
};

/* This structure defines the battery driver state structure */

struct battery_dev_s
{
  /* Fields required by the upper-half driver */

  FAR const struct battery_operations_s *ops; /* Battery operations */
  sem_t batsem;  /* Enforce mutually exclusive access */

  /* Data fields specific to the lower-half driver may follow */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
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
 *   dev - An instance of the battery state structure .
 *
 * Returned value:
 *    Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

EXTERN int battery_register(FAR const char *devpath,
                            FAR struct battery_dev_s *dev);

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
 *   CONFIG_I2C_MAX17040 or CONFIG_I2C_MAX17041 - The driver must know which
 *     chip is on the board in order to scale the voltage correctly.
 *
 * Input Parameters:
 *   i2c - An instance of the I2C interface to use to communicate with the MAX1704x
 *   addr - The I2C address of the MAX1704x (Better be 0x36).
 *   frequency - The I2C frequency
 *
 * Returned Value:
 *   A pointer to the intialized battery driver instance.  A NULL pointer
 *   is returned on a failure to initialize the MAX1704x lower half.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_MAX1704X)
struct i2c_dev_s; /* Forward reference */

EXTERN FAR struct battery_dev_s *
  max1704x_initialize(FAR struct i2c_dev_s *i2c, uint8_t addr, uint32_t frequency);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_BATTERY */
#endif /* __INCLUDE_NUTTX_POWER_BATTERY_H */
