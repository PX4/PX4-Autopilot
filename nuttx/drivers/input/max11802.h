/********************************************************************************************
 * drivers/input/max11802.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Petteri Aimonen <jpa@nx.mail.kapsi.fi>
 *
 * References:
 *   "Low-Power, Ultra-Small Resistive Touch-Screen Controllers
 *    with I2C/SPI Interface" Maxim IC, Rev 3, 10/2010
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

#ifndef __DRIVERS_INPUT_MAX11802_H
#define __DRIVERS_INPUT_MAX11802_H

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
#include <nuttx/input/max11802.h>

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/
/* Configuration ****************************************************************************/

/* MAX11802 Interfaces *********************************************************************/

/* LSB of register addresses specifies read (1) or write (0). */
#define MAX11802_CMD_XPOSITION ((0x52 << 1) | 1)
#define MAX11802_CMD_YPOSITION ((0x54 << 1) | 1)
#define MAX11802_CMD_MEASUREXY (0x70 << 1)
#define MAX11802_CMD_MODE_WR   (0x0B << 1)
#define MAX11802_CMD_MODE_RD   ((0x0B << 1) | 1)
#define MAX11802_CMD_AVG_WR    (0x03 << 1)
#define MAX11802_CMD_TIMING_WR (0x05 << 1)
#define MAX11802_CMD_DELAY_WR  (0x06 << 1)

/* Register values to set */
#define MAX11802_MODE    0x0E
#define MAX11802_AVG     0x55
#define MAX11802_TIMING  0x77
#define MAX11802_DELAY   0x55

/* Driver support **************************************************************************/
/* This format is used to construct the /dev/input[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/input%d"
#define DEV_NAMELEN  16

/* Poll the pen position while the pen is down at this rate (50MS): */

#define MAX11802_WDOG_DELAY  ((50 + (MSEC_PER_TICK-1))/ MSEC_PER_TICK)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/* This describes the state of one contact */

enum max11802_contact_3
{
  CONTACT_NONE = 0,                    /* No contact */
  CONTACT_DOWN,                        /* First contact */
  CONTACT_MOVE,                        /* Same contact, possibly different position */
  CONTACT_UP,                          /* Contact lost */
};

/* This structure describes the results of one MAX11802 sample */

struct max11802_sample_s
{
  uint8_t  id;                          /* Sampled touch point ID */
  uint8_t  contact;                     /* Contact state (see enum ads7843e_contact_e) */
  bool     valid;                       /* True: x,y contain valid, sampled data */
  uint16_t x;                           /* Measured X position */
  uint16_t y;                           /* Measured Y position */
};

/* This structure describes the state of one MAX11802 driver instance */

struct max11802_dev_s
{
#ifdef CONFIG_ADS7843E_MULTIPLE
  FAR struct ads7843e_dev_s *flink;     /* Supports a singly linked list of drivers */
#endif
  uint8_t nwaiters;                     /* Number of threads waiting for MAX11802 data */
  uint8_t id;                           /* Current touch point ID */
  volatile bool penchange;              /* An unreported event is buffered */
  uint16_t threshx;                     /* Thresholding X value */
  uint16_t threshy;                     /* Thresholding Y value */
  sem_t devsem;                         /* Manages exclusive access to this structure */
  sem_t waitsem;                        /* Used to wait for the availability of data */

  FAR struct max11802_config_s *config; /* Board configuration data */
  FAR struct spi_dev_s *spi;            /* Saved SPI driver instance */
  struct work_s work;                   /* Supports the interrupt handling "bottom half" */
  struct max11802_sample_s sample;      /* Last sampled touch point data */
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
