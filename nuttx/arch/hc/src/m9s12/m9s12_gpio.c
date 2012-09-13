/****************************************************************************
 * arch/arm/src/m9s12/m9s12_gpio.c
 * arch/arm/src/chip/m9s12_gpio.c
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

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "up_arch.h"
#include "m9s12_internal.h"
#include "m9s12_pim.h"
#include "m9s12_mebi.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* GPIO management macros:
 *
 * The GPIO configuration is represented by a 16-bit value encoded as follows:
 *
 *   xIIO UURV DMGG GPPP
 *    ||| |||| |||    `-Pin number
 *    ||| |||| || `- Port number
 *    ||| |||| | `- PIM Ports
 *    ||| |||| `- Direction
 *    ||| |||`- Initial value of output
 *    ||| ||`- Reduced drive
 *    ||| |`- Polarity
 *    ||| `- Pull up (or down)
 *    ||`- Wired OR open drain
 *    |`- Interrupt or rising/falling (polarity)
 *    `- Interrupt
 *
 * NOTE: MEBI ports E and K can have special configurations as controlled by
 * the PEAR and MODE registers.  Those special configurations are not managed
 * by the logic below; that logic is only intended to support general GPIO
 * pin usage.
 */

/* PIM ports (T,S,G,H,J,L) */

#define HCS12_PIM_NPORTS 6

/* MEBI ports (A,B,E,K) */

#define HCS12_MEBI_NPORTS 4

/* Which ports have which registers? */

#define HCS12_PORT_T      (1 << 0)
#define HCS12_PORT_S      (1 << 1)
#define HCS12_PORT_G      (1 << 2)
#define HCS12_PORT_H      (1 << 3)
#define HCS12_PORT_J      (1 << 4)
#define HCS12_PORT_L      (1 << 5)
#define HCS12_PORT_ALL    0x3f

#define HCS12_IO_PORTS    HCS12_PORT_ALL
#define HCS12_INPUT_PORTS HCS12_PORT_ALL
#define HCS12_DDR_PORTS   HCS12_PORT_ALL
#define HCS12_RDR_PORTS   HCS12_PORT_ALL
#define HCS12_PER_PORTS   HCS12_PORT_ALL
#define HCS12_PS_PORTS    HCS12_PORT_ALL
#define HCS12_WOM_PORTS   (HCS12_PORT_S|HCS12_PORT_L)
#define HCS12_IE_PORTS    (HCS12_PORT_G|HCS12_PORT_H|HCS12_PORT_J)
#define HCS12_IF_PORTS    (HCS12_PORT_G|HCS12_PORT_H|HCS12_PORT_J)

/* Decoding helper macros */

#define HCS12_PIN(cfg)        (((cfg) & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT)
#define HCS12_PORTNDX(cfg)    (((cfg) >> GPIO_PORT_SHIFT) & 7)
#define HCS12_PIMPORT(cfg)    (((cfg) & GPIO_PORT_PIM) != 0)
#define HCS12_MEBIPORT(cfg)   (((cfg) & GPIO_PORT_PIM) == 0)
#define HCS12_OUTPUT(cfg)     (((cfg) & GPIO_DIRECTION) == GPIO_OUTPUT)

#define HCS12_PULL(cfg)       (((cfg) & GPIO_PULLUP_MASK) >> GPIO_PULLUP_SHIFT)
#  define HCS12_PULL_NONE     0
#  define HCS12_PULL_POLARITY 1
#  define HCS12_PULL_ENABLE   2
#  define HCS12_PULL_UP       2
#  define HCS12_PULL_DOWN     3

#define HCS12_INTERRUPT(cfg)  (((cfg) & GPIO_INT_MASK) >> GPIO_INT_SHIFT)
#  define HCS12_INT_NONE      0
#  define HCS12_INT_POLARITY  1
#  define HCS12_INT_ENABLE    2
#  define HCS12_INT_FALLING   2
#  define HCS12_INT_RISING    3

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mebi_portaddr_s
{
  uint16_t data;     /* Data register */
  uint16_t ddr;      /* Direction register */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct mebi_portaddr_s mebi_portaddr[HCS12_MEBI_NPORTS] =
{
  {HCS12_MEBI_PORTA, HCS12_MEBI_DDRA}, /* Port A */
  {HCS12_MEBI_PORTB, HCS12_MEBI_DDRB}, /* Port B */
  {HCS12_MEBI_PORTE, HCS12_MEBI_DDRE}, /* Port E */
  {HCS12_MEBI_PORTK, HCS12_MEBI_DDRK}  /* Port K */
};

static uint8_t mebi_bits[HCS12_MEBI_NPORTS] =
{
  (1 << 0), (1 << 1), (1 << 4), (1 << 7)
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Misc. Low-Level, Inline Helper Functions
 ****************************************************************************/

/* Set or clear a bit in a register */

static inline void gpio_writebit(uint16_t regaddr, uint8_t pin, bool set)
{
  uint8_t regval = getreg8(regaddr);
  if (set)
    {
      regval |= (1 << pin);
    }
  else
    {
      regval &= ~(1 << pin);
    }
  putreg8(regval, regaddr);
}

/* Return the value of a bit in a register */

static inline bool gpio_readbit(uint16_t regaddr, uint8_t pin)
{
  uint8_t regval = getreg8(regaddr);
  return ((regval & (1 << pin)) != 0);
}

/* Set the direction of a PIM port */

static inline void pim_direction(uint8_t portndx, uint8_t pin, bool output)
{
  gpio_writebit(HCS12_PIM_PORT_DDR(portndx), pin, output);
}

/* Set the direction of a MEBI port */

static inline void mebi_direction(uint8_t portndx, uint8_t pin, bool output)
{
  gpio_writebit(mebi_portaddr[portndx].ddr, pin, output);
}

/* Write to the Wired-OR register of a PIM port */

static inline void pim_opendrain(uint8_t portndx, uint8_t pin, bool opendrain)
{
  DEBUGASSERT(!opendrain || (HCS12_WOM_PORTS & (1 << pin)) != 0);
  gpio_writebit(HCS12_PIM_PORT_WOM(portndx), pin, opendrain);
}

/* Configure pull up resisters on on a PIM port pin */

static inline void pim_pullpin(uint8_t portndx, uint8_t pin, uint8_t pull)
{
  bool     enable   = false;
  bool     polarity = false;

  if ((pull & HCS12_PULL_ENABLE) != 0)
    {
      enable = true;
      if ((pull & HCS12_PULL_POLARITY) != 0)
        {
          polarity = true;
        }
    }

  gpio_writebit(HCS12_PIM_PORT_PER(portndx), pin, enable);
  gpio_writebit(HCS12_PIM_PORT_PS(portndx), pin, polarity);
}

/* Configure pull up resisters on on a while PIM port */

static inline void mebi_pullport(uint8_t portndx, uint8_t pull)
{
  uint8_t regval = getreg8(HCS12_MEBI_PUCR);
  if (pull == HCS12_PULL_UP)
    {
      regval |= mebi_bits[portndx];
    }
  else
    {
      regval &= ~mebi_bits[portndx];
    }
  putreg8(regval, HCS12_MEBI_PUCR);
}

/* Select/deselect reduced drive for a PIM port pin */

static inline void pim_rdpin(uint8_t portndx, uint8_t pin, bool rdenable)
{
  gpio_writebit(HCS12_PIM_PORT_RDR(portndx), pin, rdenable);
}

/* Select/deselect reduced drive for a whole MEBI port */

static inline void mebi_rdport(uint8_t portndx, bool rdenable)
{
  uint8_t regval = getreg8(HCS12_MEBI_RDRIV);
  if (rdenable)
    {
      regval |= mebi_bits[portndx];
    }
  else
    {
      regval &= ~mebi_bits[portndx];
    }
  putreg8(regval, HCS12_MEBI_RDRIV);
}

/* Configure the PIM port pin as a interrupt */

static inline void pim_interrupt(uint8_t portndx, unsigned pin, uint8_t type)
{
  if (type != HCS12_INT_NONE)
    {
      DEBUGASSERT((HCS12_IE_PORTS & (1 << pin)) != 0);
      gpio_writebit(HCS12_PIM_PORT_IE(portndx), pin, false);
      gpio_writebit(HCS12_PIM_PORT_PS(portndx), pin, ((type & GPIO_INT_POLARITY) != 0));
    }
  else if ((HCS12_IE_PORTS & (1 << pin)) != 0)
    {
      gpio_writebit(HCS12_PIM_PORT_IE(portndx), pin, false);
    }
} 

/****************************************************************************
 * Name: pim_configgpio
 *
 * Description:
 *   Configure a PIM pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void pim_configgpio(uint16_t cfgset, uint8_t portndx, uint8_t pin)
{
  /* Sanity checking -- Check if the pin will be enabled as an interrupt
   * (later)
   */

  DEBUGASSERT(portndx < HCS12_PIM_NPORTS);

#ifdef CONFIG_DEBUG
  if ((cfgset & GPIO_INT_ENABLE) != 0)
    {
      /* Yes.. then it must not be tagged as an output */
 
      ASSERT((cfgset & GPIO_DIRECTION) != GPIO_OUTPUT);

      /* If the pull-driver is also enabled, it must be enabled with a
       * compatible priority.
       */

      if ((cfgset & GPIO_PULL_ENABLE) != 0)
        {
          if ((cfgset & GPIO_INT_POLARITY) != 0)
            {
              ASSERT((cfgset & GPIO_PULL_POLARITY) != 0);
            }
          else
            {
              ASSERT((cfgset & GPIO_PULL_POLARITY) == 0);
            }
        }
    }
#endif

  pim_direction(portndx, pin, ((cfgset & GPIO_DIRECTION) == GPIO_OUTPUT));
  pim_opendrain(portndx, pin, ((cfgset & GPIO_OPENDRAIN) != 0));
  pim_pullpin(portndx, pin, HCS12_PULL(cfgset));
  pim_rdpin(portndx, pin, ((cfgset & GPIO_REDUCED) != 0));
  pim_interrupt(portndx, pin, HCS12_INTERRUPT(cfgset));
}

/****************************************************************************
 * Name: mebi_configgpio
 *
 * Description:
 *   Configure a MEBI pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void mebi_configgpio(uint16_t cfgset, uint8_t portndx, uint8_t pin)
{
  DEBUGASSERT(portndx < HCS12_MEBI_NPORTS);
  mebi_direction(portndx, pin, ((cfgset & GPIO_DIRECTION) == GPIO_OUTPUT));
  mebi_pullport(portndx, HCS12_PULL(cfgset));
  mebi_rdport(portndx, ((cfgset & GPIO_REDUCED) != 0));
}

/****************************************************************************
 * Read/Write Helpers
 ****************************************************************************/

/* Set the output state of a PIM port pin */

static inline void pim_gpiowrite(uint8_t portndx, uint8_t pin, bool value)
{
  uint16_t regaddr = HCS12_PIM_PORT_IO(portndx);
  DEBUGASSERT(portndx < HCS12_PIM_NPORTS);
  gpio_writebit(regaddr, pin, value);
}

/* Set the output state of a MEBI port pin */

static inline void mebi_gpiowrite(uint8_t portndx, uint8_t pin, bool value)
{
  uint16_t regaddr;
  DEBUGASSERT(portndx < HCS12_MEBI_NPORTS);
  regaddr = mebi_portaddr[portndx].data;
  gpio_writebit(regaddr, pin, value);
}

/* Get the current state of a PIM port pin */

static inline bool pim_gpioread(uint8_t portndx, uint8_t pin)
{
  uint16_t regaddr = HCS12_PIM_PORT_INPUT(portndx);
  DEBUGASSERT(portndx < HCS12_PIM_NPORTS);
  return gpio_readbit(regaddr, pin);
}

/* Get the current state of a MEBI port pin */

static inline bool mebi_gpioread(uint8_t portndx, uint8_t pin)
{
  uint16_t regaddr;
  DEBUGASSERT(portndx < HCS12_MEBI_NPORTS);
  regaddr = mebi_portaddr[portndx].data;
  return gpio_readbit(regaddr, pin);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hcs12_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int hcs12_configgpio(uint16_t cfgset)
{
  /* Get the port index and pin number */

  uint8_t portndx = HCS12_PORTNDX(cfgset);
  uint8_t pin     = HCS12_PIN(cfgset);

  /* Configure the pin */

  if (HCS12_PIMPORT(cfgset))
    {
      pim_configgpio(cfgset, portndx, pin);
    }
  else
    {
      mebi_configgpio(cfgset, portndx, pin);
    }

  /* If the pin is an output, then set the initial value of the output */

  if (HCS12_OUTPUT(cfgset))
    {
      hcs12_gpiowrite(cfgset, (cfgset & GPIO_OUTPUT_VALUE) == GPIO_OUTPUT_HIGH);
    }
  return OK;
}

/****************************************************************************
 * Name: hcs12_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void hcs12_gpiowrite(uint16_t pinset, bool value)
{
  uint8_t    portndx = HCS12_PORTNDX(pinset);
  uint8_t    pin     = HCS12_PIN(pinset);
  irqstate_t flags   = irqsave();

  DEBUGASSERT((pinset & GPIO_DIRECTION) == GPIO_OUTPUT);
  if (HCS12_PIMPORT(pinset))
    {
      pim_gpiowrite(portndx, pin, value);
    }
  else
    {
      mebi_gpiowrite(portndx, pin, value);
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: hcs12_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool hcs12_gpioread(uint16_t pinset)
{
  uint8_t portndx = HCS12_PORTNDX(pinset);
  uint8_t pin     = HCS12_PIN(pinset);

  if (HCS12_PIMPORT(pinset))
    {
      return pim_gpioread(portndx, pin);
    }
  else
    {
      return mebi_gpioread(portndx, pin);
    }
}
