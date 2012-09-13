/****************************************************************************
 * arch/arm/src/m9s12/m9s12_dumpgpio.c
 * arch/arm/src/chip/m9s12_dumpgpio.c
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
#include <debug.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "m9s12_internal.h"
#include "m9s12_pim.h"
#include "m9s12_mebi.h"

#ifdef CONFIG_DEBUG_GPIO

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* PIM ports (T,S,G,H,J,L) */

#define HCS12_PIM_NPORTS 6

/* MEBI ports (A,B,E,K) */

#define HCS12_MEBI_NPORTS 4

/* Decoding helper macros */

#define HCS12_PIN(cfg)        (((cfg) & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT)
#define HCS12_PORTNDX(cfg)    (((cfg) >> GPIO_PORT_SHIFT) & 7)
#define HCS12_PIMPORT(cfg)    (((cfg) & GPIO_PORT_PIM) != 0)
#define HCS12_MEBIPORT(cfg)   (((cfg) & GPIO_PORT_PIM) == 0)

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

/* PIM ports have the following forms:
 *
 *   FORM 1: IO INPUT DDR RDR PER PS            Port T
 *   FORM 2: IO INPUT DDR RDR PER PS WOM        Port S,L
 *   FORM 3: IO INPUT DDR RDR PER PS     IE IF  Port G,H,J
 */

#define PIMPORT_FORM1 0
#define PIMPORT_FORM2 1
#define PIMPORT_FORM3 2

/* MEBI ports are pretty strange.  Most look the same, but E and K can have
 * some special attributes.
 */

#define MEBIPORT_AB  0
#define MEBIPORT_E   1
#define MEBIPORT_K   2

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct gpio_piminfo_s
{
  uint16_t base;  /* PIM GPIO block base address */
  char     name;  /* Port name */
  uint8_t  form;  /* Form of the PIM GPIO block registers */
};

struct gpio_mebiinfo_s
{
  uint16_t data;  /* MEBI data register address */
  uint16_t ddr;   /* MEBI ddr register address */
  char     name;  /* Port name */
  uint8_t  form;  /* Form of the MEBI port */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

static const struct gpio_piminfo_s piminfo[HCS12_PIM_NPORTS] =
{
 {HCS12_PIM_PORTT_BASE, 'T', PIMPORT_FORM1}, /* Port T */
 {HCS12_PIM_PORTS_BASE, 'S', PIMPORT_FORM2}, /* Port S */
 {HCS12_PIM_PORTG_BASE, 'G', PIMPORT_FORM3}, /* Port G */
 {HCS12_PIM_PORTH_BASE, 'H', PIMPORT_FORM3}, /* Port H */
 {HCS12_PIM_PORTJ_BASE, 'J', PIMPORT_FORM3}, /* Port J */
 {HCS12_PIM_PORTL_BASE, 'L', PIMPORT_FORM2}  /* Port L */
};

static const struct gpio_mebiinfo_s mebiinfo[HCS12_MEBI_NPORTS] =
{
 {HCS12_MEBI_PORTA, HCS12_MEBI_DDRA, 'A', MEBIPORT_AB}, /* Port A */
 {HCS12_MEBI_PORTB, HCS12_MEBI_DDRB, 'B', MEBIPORT_AB}, /* Port B */
 {HCS12_MEBI_PORTE, HCS12_MEBI_DDRE, 'E', MEBIPORT_E},  /* Port E */
 {HCS12_MEBI_PORTK, HCS12_MEBI_DDRK, 'K', MEBIPORT_K}   /* Port K */
};
 
/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hcs12_pimdump
 *
 * Description:
 *   PIM GPIO register block dump
 *
 ****************************************************************************/

static inline void hcs12_pimdump(uint8_t portndx)
{
  const struct gpio_piminfo_s *ptr;

  if (portndx >= HCS12_PIM_NPORTS)
    {
      lldbg("  Illegal PIM port index: %d\n", portndx);
      return;
    }

  ptr = &piminfo[portndx];
  lldbg(" PIM Port%c:\n", ptr->name);
  lldbg("   IO:%02x  INP:%02x DDR:%02x RDR:%02x\n",
        getreg8(ptr->base+HCS12_PIM_IO_OFFSET),
        getreg8(ptr->base+HCS12_PIM_INPUT_OFFSET),
        getreg8(ptr->base+HCS12_PIM_DDR_OFFSET),
        getreg8(ptr->base+HCS12_PIM_RDR_OFFSET));

  switch (ptr->form)
    {
    case PIMPORT_FORM1:
      lldbg("  PER:%02x  PS:%02x\n",
            getreg8(ptr->base+HCS12_PIM_PER_OFFSET),
            getreg8(ptr->base+HCS12_PIM_PS_OFFSET));
      break;

    case PIMPORT_FORM2:
      lldbg("  PER:%02x  PS:%02x WOM:%02x\n",
            getreg8(ptr->base+HCS12_PIM_PER_OFFSET),
            getreg8(ptr->base+HCS12_PIM_PS_OFFSET),
            getreg8(ptr->base+HCS12_PIM_WOM_OFFSET));
      break;

    case PIMPORT_FORM3:
      lldbg("  PER:%02x  PS:%02x  IE:%02x  IF:%02x\n",
            getreg8(ptr->base+HCS12_PIM_PER_OFFSET),
            getreg8(ptr->base+HCS12_PIM_PS_OFFSET),
            getreg8(ptr->base+HCS12_PIM_IE_OFFSET),
            getreg8(ptr->base+HCS12_PIM_IF_OFFSET));
      break;

    default:
      break;
    }
}

/****************************************************************************
 * Name: hcs12_mebidump
 *
 * Description:
 *   PIM GPIO register block dump
 *
 ****************************************************************************/

static inline void hcs12_mebidump(uint8_t portndx)
{
  const struct gpio_mebiinfo_s *ptr;

  if (portndx >= HCS12_MEBI_NPORTS)
    {
      lldbg("  Illegal MEBI port index: %d\n", portndx);
      return;
    }
    
  ptr = &mebiinfo[portndx];
  lldbg(" MEBI Port%c:\n", ptr->name);

  switch (ptr->form)
    {
    case MEBIPORT_AB:
      lldbg("   DATA:%02x DDR:%02x\n",
            getreg8(ptr->data), getreg8(ptr->ddr));
      break;

    case MEBIPORT_E:
      lldbg("   DATA:%02x DDR:%02x MODE:%02x PEAR:%02x\n",
            getreg8(ptr->data), getreg8(ptr->ddr),
            getreg8(HCS12_MEBI_MODE), getreg8(HCS12_MEBI_PEAR));
      break;

    case MEBIPORT_K:
      lldbg("   DATA:%02x DDR:%02x MODE:%02x\n",
            getreg8(ptr->data), getreg8(ptr->ddr),
            getreg8(HCS12_MEBI_MODE));
      break;

    default:
      break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Function:  hcs12_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

int hcs12_dumpgpio(uint16_t pinset, const char *msg)
{
  uint8_t portndx = HCS12_PORTNDX(pinset);
  irqstate_t flags = irqsave();

  lldbg("pinset: %08x -- %s\n", pinset, msg);

  if (HCS12_PIMPORT(pinset))
    {
      hcs12_pimdump(portndx);
    }
  else
    {
      hcs12_mebidump(portndx);
    }

  irqrestore(flags);
  return OK;
}

#endif /* CONFIG_DEBUG_GPIO */

