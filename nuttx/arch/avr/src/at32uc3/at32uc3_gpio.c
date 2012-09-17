/**************************************************************************
 * arch/avr/src/at32uc3/at32uc3_gpio.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>

#include "at32uc3_config.h"
#include "up_internal.h"
#include "at32uc3_internal.h"

#include "up_arch.h"
#include "chip.h"
#include "at32uc3_gpio.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

/* How many GPIO ports are supported?  There are 32-pins per port and we
 * know he number of GPIO pins supported by the architecture:
 */

#define AVR32_NGPIO_PORTS ((AVR32_NGPIO+31) >> 5)

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

static uint32_t g_portmap[AVR32_NGPIO_PORTS] =
{
#if AVR32_NGPIO > 0
   AVR32_GPIO0_BASE
#endif
#if AVR32_NGPIO > 32
   , AVR32_GPIO1_BASE,
#endif
#if AVR32_NGPIO > 64
   , AVR32_GPIO2_BASE,
#endif
#if AVR32_NGPIO > 96
   , AVR32_GPIO3_BASE,
#endif
#if AVR32_NGPIO > 128
   , AVR32_GPIO4_BASE,
#endif
};

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Public Functions
 **************************************************************************/

/************************************************************************************
 * Name: at32uc3_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int at32uc3_configgpio(uint16_t cfgset)
{
  unsigned int port;
  unsigned int pin;
  uint32_t     pinmask;
  uint32_t     base;

  /* Extract then port number and the pin number from the configuration */

  port = ((unsigned int)cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin  = ((unsigned int)cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  DEBUGASSERT(port < AVR32_NGPIO_PORTS);

  /* Get pin mask and the GPIO base address */

  pinmask = (1 << pin);
  base    = g_portmap[port];

  /* First, just to be safe, disable the output driver, give GPIO control of
   * the pin, rese the peripheral mux, set the output low, remove the pull-up,
   * disable GPIO interrupts, reset the interrupt mode, and disable glitch
   * filtering, while we reconfigure the pin.
   */

  putreg32(pinmask, base + AVR32_GPIO_ODERC_OFFSET);
  putreg32(pinmask, base + AVR32_GPIO_GPERS_OFFSET);
  putreg32(pinmask, base + AVR32_GPIO_PMR0C_OFFSET);
  putreg32(pinmask, base + AVR32_GPIO_PMR1C_OFFSET);
  putreg32(pinmask, base + AVR32_GPIO_OVRC_OFFSET);
  putreg32(pinmask, base + AVR32_GPIO_PUERC_OFFSET);
  putreg32(pinmask, base + AVR32_GPIO_IERC_OFFSET);
  putreg32(pinmask, base + AVR32_GPIO_IMR0C_OFFSET);
  putreg32(pinmask, base + AVR32_GPIO_IMR1C_OFFSET);
  putreg32(pinmask, base + AVR32_GPIO_GFERC_OFFSET);

  /* Is this a GPIO?  Or a peripheral */

  if ((cfgset & GPIO_ENABLE) != 0)
    {
      /* Its a GPIO.  Input or output? */
 
      if ((cfgset & GPIO_OUTPUT) != 0)
        {
          /* Its a GPIO output. Set up the initial output value and enable
           * the output driver.
           */
 
          if ((cfgset & GPIO_VALUE) != 0)
            {
              putreg32(pinmask, base + AVR32_GPIO_OVRS_OFFSET);
            }
          putreg32(pinmask, base + AVR32_GPIO_ODERS_OFFSET);
        }
      else
        {
          /* Its a GPIO input.  There is nothing more to do here. */
        }
    }
  else
    {
      /* Its a peripheral.  Set the peripheral mux */

      if ((cfgset & GPIO_PMR0) != 0)
        {
          putreg32(pinmask, base + AVR32_GPIO_PMR0S_OFFSET);
        }

      if ((cfgset & GPIO_PMR1) != 0)
        {
          putreg32(pinmask, base + AVR32_GPIO_PMR1S_OFFSET);
        }

      /* And enable peripheral control of the pin */

      putreg32(pinmask, base + AVR32_GPIO_GPERC_OFFSET);
    }

  /* Then the "ornaments" tha do not depend on gpio/peripheral mode:
   * Pull-ups and glitch filering.
   */

  if ((cfgset & GPIO_PULLUP) != 0)
    {
      putreg32(pinmask, base + AVR32_GPIO_PUERS_OFFSET);
    }

  if ((cfgset & GPIO_PULLUP) != 0)
    {
      putreg32(pinmask, base + AVR32_GPIO_GFERS_OFFSET);
    }

  /* Check for GPIO interrupt */

  if ((cfgset & GPIO_INTR) != 0)
    {
      /* Set up the interrupt mode */

      if ((cfgset & GPIO_IMR0) != 0)
        {
          putreg32(pinmask, base + AVR32_GPIO_IMR0S_OFFSET);
        }

      if ((cfgset & GPIO_IMR1) != 0)
        {
          putreg32(pinmask, base + AVR32_GPIO_IMR1S_OFFSET);
        }

      /* Then enable the GPIO interrupt */

      putreg32(pinmask, base + AVR32_GPIO_IERS_OFFSET);
    }
  
  return OK;
}

/************************************************************************************
 * Name: at32uc3_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void at32uc3_gpiowrite(uint16_t pinset, bool value)
{
  unsigned int port;
  unsigned int pin;
  uint32_t     pinmask;
  uint32_t     base;

  /* Extract then port number and the pin number from the configuration */

  port = ((unsigned int)pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin  = ((unsigned int)pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  DEBUGASSERT(port < AVR32_NGPIO_PORTS);

  /* Get pin mask and the GPIO base address */

  pinmask = (1 << pin);
  base    = g_portmap[port];

  /* Now, set or clear the pin ouput value */

  if (value)
    {
      putreg32(pinmask, base + AVR32_GPIO_OVRS_OFFSET);
    }
  else
    {
      putreg32(pinmask, base + AVR32_GPIO_OVRC_OFFSET);
    }
}

/************************************************************************************
 * Name: at32uc3_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool at32uc3_gpioread(uint16_t pinset)
{
  unsigned int port;
  unsigned int pin;
  uint32_t     pinmask;
  uint32_t     base;

  /* Extract then port number and the pin number from the configuration */

  port = ((unsigned int)pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin  = ((unsigned int)pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  DEBUGASSERT(port < AVR32_NGPIO_PORTS);

  /* Get pin mask and the GPIO base address */

  pinmask = (1 << pin);
  base    = g_portmap[port];

  /* Now, return the current pin value */

  return (getreg32(base + AVR32_GPIO_PVR_OFFSET) & pinmask) != 0;
}
