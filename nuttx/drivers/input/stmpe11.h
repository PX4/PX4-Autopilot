/********************************************************************************************
 * drivers/input/stmpe11.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "STMPE811 S-Touch® advanced resistive touchscreen controller with 8-bit
 *    GPIO expander," Doc ID 14489 Rev 6, CD00186725, STMicroelectronics"
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
 ********************************************************************************************/

#ifndef __DRIVERS_INPUT_STMPE11_H
#define __DRIVERS_INPUT_STMPE11_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <wdog.h>
#include <semaphore.h>

#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/input/stmpe11.h>

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_STMPE11)

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/
/* Configuration ****************************************************************************/
/* Reference counting is partially implemented, but not needed in the current design.
 */

#undef CONFIG_STMPE11_REFCNT

/* No support for the SPI interface yet */

#ifdef CONFIG_STMPE11_SPI
#  error "Only the STMPE11 I2C interface is supported by this driver"
#endif

/* Driver support ***************************************************************************/
/* This format is used to construct the /dev/input[n] device driver path.  It defined here
 * so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/input%d"
#define DEV_NAMELEN  16

/* STMPE11 Resources ************************************************************************/
#ifndef CONFIG_STMPE11_TSC_DISABLE
#  define SMTPE11_ADC_NPINS  4 /* Only pins 0-3 can be used for ADC */
#  define SMTPE11_GPIO_NPINS 4 /* Only pins 0-3 can be used as GPIOs */
#else
#  define SMTPE11_ADC_NPINS  8 /* All pins can be used for ADC */
#  define SMTPE11_GPIO_NPINS 8 /* All pins can be used as GPIOs */
#endif

/* Driver flags */

#define STMPE11_FLAGS_TSC_INITIALIZED  (1 << 0) /* 1: The TSC block has been initialized */
#define STMPE11_FLAGS_GPIO_INITIALIZED (1 << 1) /* 1: The GIO block has been initialized */
#define STMPE11_FLAGS_ADC_INITIALIZED  (1 << 2) /* 1: The ADC block has been initialized */
#define STMPE11_FLAGS_TS_INITIALIZED   (1 << 3) /* 1: The TS block has been initialized */

/* Timeout to detect missing pen up events */

#define STMPE11_PENUP_TICKS  ((100 + (MSEC_PER_TICK-1)) / MSEC_PER_TICK)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/
/* This describes the state of one contact */

enum stmpe11_contact_3
{
  CONTACT_NONE = 0,                    /* No contact */
  CONTACT_DOWN,                        /* First contact */
  CONTACT_MOVE,                        /* Same contact, possibly different position */
  CONTACT_UP,                          /* Contact lost */
};

/* This structure describes the results of one STMPE11 sample */

struct stmpe11_sample_s
{
  uint8_t  id;                         /* Sampled touch point ID */
  uint8_t  contact;                    /* Contact state (see enum stmpe11_contact_e) */
  bool     valid;                      /* True: x,y,z contain valid, sampled data */
  uint16_t x;                          /* Measured X position */
  uint16_t y;                          /* Measured Y position */
  uint8_t  z;                          /* Measured Z index */
};

/* This structure represents the state of the SMTPE11 driver */

struct stmpe11_dev_s
{
#ifdef CONFIG_STMPE11_MULTIPLE
  FAR struct stmpe11_dev_s *flink;      /* Supports a singly linked list of drivers */
#endif

  /* Common fields */

  FAR struct stmpe11_config_s *config; /* Board configuration data */
  sem_t exclsem;                       /* Manages exclusive access to this structure */
#ifdef CONFIG_STMPE11_SPI
  FAR struct spi_dev_s *spi;           /* Saved SPI driver instance */
#else
  FAR struct i2c_dev_s *i2c;           /* Saved I2C driver instance */
#endif

  uint8_t inuse;                       /* SMTPE11 pins in use */
  uint8_t flags;                       /* See SMTPE11_FLAGS_* definitions */
  struct work_s work;                  /* Supports the interrupt handling "bottom half" */

  /* Fields that may be disabled to save size if touchscreen support is not used. */

#ifndef CONFIG_STMPE11_TSC_DISABLE
#ifdef CONFIG_STMPE11_REFCNT
  uint8_t crefs;                       /* Number of times the device has been opened */
#endif
  uint8_t nwaiters;                    /* Number of threads waiting for STMPE11 data */
  uint8_t id;                          /* Current touch point ID */
  uint8_t minor;                       /* Touchscreen minor device number */
  volatile bool penchange;             /* An unreported event is buffered */

  uint16_t threshx;                    /* Thresholded X value */
  uint16_t threshy;                    /* Thresholded Y value */
  sem_t waitsem;                       /* Used to wait for the availability of data */

  struct work_s timeout;               /* Supports tiemeout work */
  WDOG_ID wdog;                        /* Timeout to detect missing pen down events */
  struct stmpe11_sample_s sample;      /* Last sampled touch point data */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_STMPE11_NPOLLWAITERS];
#endif
#endif

  /* Fields that may be disabled to save size of GPIO support is not used */

#if !defined(CONFIG_STMPE11_GPIO_DISABLE) && !defined(CONFIG_STMPE11_GPIOINT_DISABLE)
  stmpe11_handler_t handlers[SMTPE11_GPIO_NPINS]; /* GPIO "interrupt handlers" */
#endif
};

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

/********************************************************************************************
 * Name: stmpe11_getreg8
 *
 * Description:
 *   Read from an 8-bit STMPE11 register
 *
 ********************************************************************************************/

uint8_t stmpe11_getreg8(FAR struct stmpe11_dev_s *priv, uint8_t regaddr);

/********************************************************************************************
 * Name: stmpe11_putreg8
 *
 * Description:
 *   Write a value to an 8-bit STMPE11 register
 *
 ********************************************************************************************/

void stmpe11_putreg8(FAR struct stmpe11_dev_s *priv, uint8_t regaddr, uint8_t regval);

/********************************************************************************************
 * Name: stmpe11_getreg16
 *
 * Description:
 *   Read 16-bits of data from an STMPE-11 register
 *
 ********************************************************************************************/

uint16_t stmpe11_getreg16(FAR struct stmpe11_dev_s *priv, uint8_t regaddr);

/********************************************************************************************
 * Name: stmpe11_tscint
 *
 * Description:
 *   Handle touchscreen interrupt events (this function actually executes in the context of
 *   the worker thread).
 *
 ********************************************************************************************/

#ifndef CONFIG_STMPE11_TSC_DISABLE
void stmpe11_tscworker(FAR struct stmpe11_dev_s *priv, uint8_t intsta) weak_function;
#endif

/********************************************************************************************
 * Name: stmpe11_gpioworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the context of the
 *   worker thread).
 *
 ********************************************************************************************/

#if !defined(CONFIG_STMPE11_GPIO_DISABLE) && !defined(CONFIG_STMPE11_GPIOINT_DISABLE)
void stmpe11_gpioworker(FAR struct stmpe11_dev_s *priv) weak_function;
#endif

#endif /* CONFIG_INPUT && CONFIG_INPUT_STMPE11 */
#endif /* __DRIVERS_INPUT_STMPE11_H */
