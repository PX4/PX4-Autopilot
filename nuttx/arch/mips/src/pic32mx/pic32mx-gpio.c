/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx-gpio.c
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
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "pic32mx-ioport.h"
#include "pic32mx-internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uintptr_t g_gpiobase[CHIP_NPORTS] =
{
  PIC32MX_IOPORTA_K1BASE
#if CHIP_NPORTS > 1
  , PIC32MX_IOPORTB_K1BASE
#endif
#if CHIP_NPORTS > 2
  , PIC32MX_IOPORTC_K1BASE
#endif
#if CHIP_NPORTS > 3
  , PIC32MX_IOPORTD_K1BASE
#endif
#if CHIP_NPORTS > 4
  , PIC32MX_IOPORTE_K1BASE
#endif
#if CHIP_NPORTS > 5
  , PIC32MX_IOPORTF_K1BASE
#endif
#if CHIP_NPORTS > 6
  , PIC32MX_IOPORTG_K1BASE
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: Inline PIN set field extractors
 ****************************************************************************/

static inline bool pic32mx_output(uint16_t pinset)
{
  return ((pinset & GPIO_OUTPUT) != 0);
}

static inline bool pic32mx_opendrain(uint16_t pinset)
{
  return ((pinset & GPIO_MODE_MASK) == GPIO_OPENDRAN);
}

static inline bool pic32mx_outputhigh(uint16_t pinset)
{
  return ((pinset & GPIO_VALUE_MASK) != 0);
}

static inline bool pic32mx_value(uint16_t pinset)
{
  return ((pinset & GPIO_VALUE_MASK) != GPIO_VALUE_ZERO);
}

static inline unsigned int pic32mx_portno(uint16_t pinset)
{
  return ((pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT);
}

static inline unsigned int pic32mx_pinno(uint16_t pinset)
{
  return ((pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
static inline unsigned int pic32mx_analog(uint16_t pinset)
{
  return ((pinset & GPIO_ANALOG_MASK) != 0);
}
#else
#  define pic32mx_analog(pinset) (false)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin (the
 *   interrupt will be configured when pic32mx_attach() is called.
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int pic32mx_configgpio(uint16_t cfgset)
{
  unsigned int port = pic32mx_portno(cfgset);
  unsigned int pin  = pic32mx_pinno(cfgset);
  uint32_t     mask = (1 << pin);
  uintptr_t    base;

  /* Verify that the port number is within range */

  if (port < CHIP_NPORTS)
    {
      /* Get the base address of the ports */

      base = g_gpiobase[port];

      /* Is this an input or an output? */

      sched_lock();
      if (pic32mx_output(cfgset))
        {
          /* Not analog */

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
          putreg32(mask, base + PIC32MX_IOPORT_ANSELCLR_OFFSET);
#endif
          /* It is an output; clear the corresponding bit in the TRIS register */

          putreg32(mask, base + PIC32MX_IOPORT_TRISCLR_OFFSET);

          /* Is it an open drain output? */

          if (pic32mx_opendrain(cfgset))
            {
              /* It is an open drain output.  Set the corresponding bit in
               * the ODC register.
               */

              putreg32(mask, base + PIC32MX_IOPORT_ODCSET_OFFSET);
            }
          else
            {
              /* Is is a normal output.  Clear the corresponding bit in the
               * ODC register.
               */

              putreg32(mask, base + PIC32MX_IOPORT_ODCCLR_OFFSET);
            }

          /* Set the initial output value */

          pic32mx_gpiowrite(cfgset, pic32mx_outputhigh(cfgset));
        }
      else
        {
          /* It is an input; set the corresponding bit in the TRIS register. */

          putreg32(mask, base + PIC32MX_IOPORT_TRISSET_OFFSET);
          putreg32(mask, base + PIC32MX_IOPORT_ODCCLR_OFFSET);

          /* Is it an analog input? */

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
          if (pic32mx_analog(cfgset))
            {
              putreg32(mask, base + PIC32MX_IOPORT_ANSELSET_OFFSET);
            }
          else
            {
              putreg32(mask, base + PIC32MX_IOPORT_ANSELCLR_OFFSET);
            }
#endif
        }

      sched_unlock();
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: pic32mx_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void pic32mx_gpiowrite(uint16_t pinset, bool value)
{
  unsigned int port = pic32mx_portno(pinset);
  unsigned int pin  = pic32mx_pinno(pinset);
  uintptr_t    base;

  /* Verify that the port number is within range */

  if (port < CHIP_NPORTS)
    {
      /* Get the base address of the ports */

      base = g_gpiobase[port];

      /* Set or clear the output */

      if (value)
        {
          putreg32(1 << pin, base + PIC32MX_IOPORT_PORTSET_OFFSET);
        }
      else
        {
          putreg32(1 << pin, base + PIC32MX_IOPORT_PORTCLR_OFFSET);
        }
    }
}

/****************************************************************************
 * Name: pic32mx_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool pic32mx_gpioread(uint16_t pinset)
{
  unsigned int port = pic32mx_portno(pinset);
  unsigned int pin  = pic32mx_pinno(pinset);
  uintptr_t    base;

  /* Verify that the port number is within range */

  if (port < CHIP_NPORTS)
    {
      /* Get the base address of the ports */

      base = g_gpiobase[port];

      /* Get ane return the input value */

      return (getreg32(base + PIC32MX_IOPORT_PORT_OFFSET) & (1 << pin)) != 0;
    }

  return false;
}

/****************************************************************************
 * Function:  pic32mx_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_VERBOSE) && defined(CONFIG_DEBUG_GPIO)
void pic32mx_dumpgpio(uint32_t pinset, const char *msg)
{
  unsigned int port = pic32mx_portno(pinset);
  irqstate_t   flags;
  uintptr_t    base;

  /* Verify that the port number is within range */

  if (port < CHIP_NPORTS)
    {
      /* Get the base address of the ports */

      base = g_gpiobase[port];

      /* The following requires exclusive access to the GPIO registers */

      sched_lock();
      lldbg("IOPORT%c pinset: %04x base: %08x -- %s\n",
            'A'+port, pinset, base, msg);
      lldbg("   TRIS: %08x   PORT: %08x    LAT: %08x    ODC: %08x\n",
            getreg32(base + PIC32MX_IOPORT_TRIS_OFFSET),
            getreg32(base + PIC32MX_IOPORT_PORT_OFFSET),
            getreg32(base + PIC32MX_IOPORT_LAT_OFFSET),
            getreg32(base + PIC32MX_IOPORT_ODC_OFFSET));
      lldbg("  CNCON: %08x   CNEN: %08x  CNPUE: %08x\n",
            getreg32(PIC32MX_IOPORT_CNCON),
            getreg32(PIC32MX_IOPORT_CNEN),
            getreg32(PIC32MX_IOPORT_CNPUE));
      sched_unlock();
    }
}
#endif

