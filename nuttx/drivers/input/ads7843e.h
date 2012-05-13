/********************************************************************************************
 * drivers/input/ads7843e.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Touch Screen Controller, ADS7843," Burr-Brown Products from Texas
 *    Instruments, SBAS090B, September 2000, Revised May 2002"
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

#ifndef __DRIVERS_INPUT_ADS7843E_H
#define __DRIVERS_INPUT_ADS7843E_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <semaphore.h>
#include <poll.h>
#include <wdog.h>
#include <nuttx/wqueue.h>

#include <nuttx/spi.h>
#include <nuttx/clock.h>
#include <nuttx/input/ads7843e.h>

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/
/* Configuration ****************************************************************************/
/* Reference counting is partially implemented, but not needed in the current design. */

#undef CONFIG_ADS7843E_REFCNT

/* ADS7843E Interfaces *********************************************************************/
/* ADS7843E command bit settings */

#define ADS7843E_CMD_PD0              (1 << 0)  /* PD0 */
#define ADS7843E_CMD_PD1              (1 << 1)  /* PD1 */
#define ADS7843E_CMD_DFR              (1 << 2)  /* SER/DFR */
#define ADS7843E_CMD_EIGHT_BITS_MOD   (1 << 3)  /* Mode */
#define ADS7843E_CMD_START            (1 << 7)  /* Start Bit */
#define ADS7843E_CMD_SWITCH_SHIFT     4         /* Address setting */

/* ADS7843E Commands */

#define ADS7843_CMD_YPOSITION \
  ((1 << ADS7843E_CMD_SWITCH_SHIFT)|ADS7843E_CMD_START|ADS7843E_CMD_PD0|ADS7843E_CMD_PD1)
#define ADS7843_CMD_XPOSITION \
  ((5 << ADS7843E_CMD_SWITCH_SHIFT)|ADS7843E_CMD_START|ADS7843E_CMD_PD0|ADS7843E_CMD_PD1)
#define ADS7843_CMD_ENABPINIRQ \
  ((1 << ADS7843E_CMD_SWITCH_SHIFT)|ADS7843E_CMD_START)

/* Driver support **************************************************************************/
/* This format is used to construct the /dev/input[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/input%d"
#define DEV_NAMELEN  16

/* Poll the pen position while the pen is down at this rate (50MS): */

#define ADS7843E_WDOG_DELAY  ((50 + (MSEC_PER_TICK-1))/ MSEC_PER_TICK)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/* This describes the state of one contact */

enum ads7843e_contact_3
{
  CONTACT_NONE = 0,                    /* No contact */
  CONTACT_DOWN,                        /* First contact */
  CONTACT_MOVE,                        /* Same contact, possibly different position */
  CONTACT_UP,                          /* Contact lost */
};

/* This structure describes the results of one ADS7843E sample */

struct ads7843e_sample_s
{
  uint8_t  id;                          /* Sampled touch point ID */
  uint8_t  contact;                     /* Contact state (see enum ads7843e_contact_e) */
  bool     valid;                       /* True: x,y contain valid, sampled data */
  uint16_t x;                           /* Measured X position */
  uint16_t y;                           /* Measured Y position */
};

/* This structure describes the state of one ADS7843E driver instance */

struct ads7843e_dev_s
{
#ifdef CONFIG_ADS7843E_MULTIPLE
  FAR struct ads7843e_dev_s *flink;     /* Supports a singly linked list of drivers */
#endif
#ifdef CONFIG_ADS7843E_REFCNT
  uint8_t crefs;                        /* Number of times the device has been opened */
#endif
  uint8_t nwaiters;                     /* Number of threads waiting for ADS7843E data */
  uint8_t id;                           /* Current touch point ID */
  volatile bool penchange;              /* An unreported event is buffered */
  sem_t devsem;                         /* Manages exclusive access to this structure */
  sem_t waitsem;                        /* Used to wait for the availability of data */

  FAR struct ads7843e_config_s *config; /* Board configuration data */
  FAR struct spi_dev_s *spi;            /* Saved SPI driver instance */
  struct work_s work;                   /* Supports the interrupt handling "bottom half" */
  struct ads7843e_sample_s sample;      /* Last sampled touch point data */
  WDOG_ID wdog;                         /* Poll the position while the pen is down */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_ADS7843E_NPOLLWAITERS];
#endif
};

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_INPUT_ADS7843E_H */
