/************************************************************************************
 * configs/shenzhou/src/up_touchscreen.c
 * arch/arm/src/board/up_touchscreen.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/spi.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/ads7843e.h>

#include "stm32_internal.h"
#include "shenzhou-internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifdef CONFIG_INPUT_ADS7843E
#ifndef CONFIG_INPUT
#  error "Touchscreen support requires CONFIG_INPUT"
#endif

#ifndef CONFIG_STM32_SPI3
#  error "Touchscreen support requires CONFIG_STM32_SPI3"
#endif

#ifndef CONFIG_ADS7843E_FREQUENCY
#  define CONFIG_ADS7843E_FREQUENCY 500000
#endif

#ifndef CONFIG_ADS7843E_SPIDEV
#  define CONFIG_ADS7843E_SPIDEV 3
#endif

#if CONFIG_ADS7843E_SPIDEV != 3
#  error "CONFIG_ADS7843E_SPIDEV must be three"
#endif

#ifndef CONFIG_ADS7843E_DEVMINOR
#  define CONFIG_ADS7843E_DEVMINOR 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_config_s
{
  struct ads7843e_config_s dev;
  xcpt_t                   handler;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the ADS7843E driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 * attach  - Attach the ADS7843E interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 * pendown - Return the state of the pen down GPIO input
 */

static int  tsc_attach(FAR struct ads7843e_config_s *state, xcpt_t isr);
static void tsc_enable(FAR struct ads7843e_config_s *state, bool enable);
static void tsc_clear(FAR struct ads7843e_config_s *state);
static bool tsc_busy(FAR struct ads7843e_config_s *state);
static bool tsc_pendown(FAR struct ads7843e_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the ADS7843E
 * driver.  This structure provides information about the configuration
 * of the ADS7843E and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct stm32_config_s g_tscinfo =
{
  {
    .frequency = CONFIG_ADS7843E_FREQUENCY,
    .attach    = tsc_attach,
    .enable    = tsc_enable,
    .clear     = tsc_clear,
    .busy      = tsc_busy,
    .pendown   = tsc_pendown,
  },
  .handler     = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the ADS7843E driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 * attach  - Attach the ADS7843E interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 * pendown - Return the state of the pen down GPIO input
 */

static int tsc_attach(FAR struct ads7843e_config_s *state, xcpt_t handler)
{
  FAR struct stm32_config_s *priv = (FAR struct stm32_config_s *)state;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  return OK;
}

static void tsc_enable(FAR struct ads7843e_config_s *state, bool enable)
{
  FAR struct stm32_config_s *priv = (FAR struct stm32_config_s *)state;

  /* The caller should not attempt to enable interrupts if the handler
   * has not yet been 'attached'
   */

  DEBUGASSERT(priv->handler || !enable);

  /* Attach and enable, or detach and disable */

  ivdbg("enable:%d\n", enable);
  if (enable)
    {
      (void)stm32_gpiosetevent(GPIO_TP_INT, true, true, false,
                               priv->handler);
    }
  else
    {
      (void)stm32_gpiosetevent(GPIO_TP_INT, false, false, false, NULL);
    }
}

static void tsc_clear(FAR struct ads7843e_config_s *state)
{
  /* Does nothing */
}

static bool tsc_busy(FAR struct ads7843e_config_s *state)
{
  /* Hmmm... The ADS7843E BUSY pin is not brought out on the Shenzhou board.
   * We will most certainly have to revisit this.
   */

  return false;
}

static bool tsc_pendown(FAR struct ads7843e_config_s *state)
{
  /* XPT2046 uses an an internal pullup resistor.  The PENIRQ output goes low
   * due to the current path through the touch screen to ground, which
   * initiates an interrupt to the processor via TP_INT.
   */

  bool pendown = !stm32_gpioread(GPIO_TP_INT);
  ivdbg("pendown:%d\n", pendown);
  return pendown;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arch_tcinitialize
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   configure the touchscreen device.  This function will register the driver
 *   as /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int arch_tcinitialize(int minor)
{
  FAR struct spi_dev_s *dev;
  int ret;

  idbg("minor %d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Configure and enable the ADS7843E interrupt pin as an input. */

  (void)stm32_configgpio(GPIO_TP_INT);

  /* Get an instance of the SPI interface */

  dev = up_spiinitialize(CONFIG_ADS7843E_SPIDEV);
  if (!dev)
    {
      idbg("Failed to initialize SPI bus %d\n", CONFIG_ADS7843E_SPIDEV);
      return -ENODEV;
    }

  /* Initialize and register the SPI touschscreen device */

  ret = ads7843e_register(dev, &g_tscinfo.dev, CONFIG_ADS7843E_DEVMINOR);
  if (ret < 0)
    {
      idbg("Failed to initialize SPI bus %d\n", CONFIG_ADS7843E_SPIDEV);
      /* up_spiuninitialize(dev); */
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: arch_tcuninitialize
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   uninitialize the touchscreen device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void arch_tcuninitialize(void)
{
  /* No support for un-initializing the touchscreen ADS7843E device yet */
}

#endif /* CONFIG_INPUT_ADS7843E */

