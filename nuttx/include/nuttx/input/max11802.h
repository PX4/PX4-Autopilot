/****************************************************************************
 * include/nuttx/input/max11802.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_INPUT_MAX11802_H
#define __INCLUDE_NUTTX_INPUT_MAX11802_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi.h>
#include <stdbool.h>
#include <nuttx/irq.h>

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_MAX11802)

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* SPI Frequency.  Default:  100KHz */

#ifndef CONFIG_MAX11802_FREQUENCY
#  define CONFIG_MAX11802_FREQUENCY 100000
#endif

/* Maximum number of threads than can be waiting for POLL events */

#ifndef CONFIG_MAX11802_NPOLLWAITERS
#  define CONFIG_MAX11802_NPOLLWAITERS 2
#endif

#ifndef CONFIG_MAX11802_SPIMODE
#  define CONFIG_MAX11802_SPIMODE SPIDEV_MODE0
#endif

/* Thresholds */

#ifndef CONFIG_MAX11802_THRESHX
#  define CONFIG_MAX11802_THRESHX 12
#endif

#ifndef CONFIG_MAX11802_THRESHY
#  define CONFIG_MAX11802_THRESHY 12
#endif

/* Check for some required settings.  This can save the user a lot of time
 * in getting the right configuration.
 */

#ifdef CONFIG_DISABLE_SIGNALS
#  error "Signals are required.  CONFIG_DISABLE_SIGNALS must not be selected."
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected."
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the MAX11802
 * driver.  This structure provides information about the configuration
 * of the MAX11802 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

struct max11802_config_s
{
  /* Device characterization */

  uint32_t frequency;  /* SPI frequency */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the MAX11802 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.  If possible,
   * interrupts should be configured on both rising and falling edges
   * so that contact and loss-of-contact events can be detected.
   *
   * attach  - Attach the MAX11802 interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   * clear   - Acknowledge/clear any pending GPIO interrupt
   * pendown - Return the state of the pen down GPIO input
   */

  int  (*attach)(FAR struct max11802_config_s *state, xcpt_t isr);
  void (*enable)(FAR struct max11802_config_s *state, bool enable);
  void (*clear)(FAR struct max11802_config_s *state);
  bool (*pendown)(FAR struct max11802_config_s *state);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: max11802_register
 *
 * Description:
 *   Configure the MAX11802 to use the provided SPI device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   spi     - An SPI driver instance
 *   config  - Persistent board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

EXTERN  int max11802_register(FAR struct spi_dev_s *spi,
                             FAR struct max11802_config_s *config,
                             int minor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT && CONFIG_INPUT_MAX11802 */
#endif /* __INCLUDE_NUTTX_INPUT_MAX11802_H */
