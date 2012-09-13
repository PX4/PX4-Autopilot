/************************************************************************
 * up_leds.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "up_internal.h"

/************************************************************************
 * Definitions
 ************************************************************************/

#define RESET_KLUDGE_NEEDED 1

/************************************************************************
 * Private Data
 ************************************************************************/

static uint8_t g_ledstate;

/************************************************************************
 * Private Functions
 ************************************************************************/

#if defined(CONFIG_LED_DEBUG) && defined(CONFIG_ARCH_LEDS)
static void _up_puthex(uint8_t hex) __naked
{
  hex; /* To avoid unreferenced argument warning */
  _asm
        mov     a, dpl
        ljmp    PM2_ENTRY_PHEX
  _endasm;
}

static void _up_putch(uint8_t ch) __naked
{
  _asm
        mov     a, dpl
        ljmp	PM2_ENTRY_COUT
  _endasm;
}

static void _up_putnl(void) __naked
{
  _asm
	ljmp	PM2_ENTRY_NEWLINE
  _endasm;
}

# define _up_showledinit() \
  _up_putch('I'); \
  _up_puthex(g_ledstate); _up_putch(':'); \
  _up_puthex(p82c55_port_e); _up_putnl();

# define _up_showledreset() \
  _up_putch('R'); \
  _up_puthex(led); _up_putch(':'); \
  _up_puthex(g_ledstate); _up_putch(':'); \
  _up_puthex(p82c55_port_e); _up_putnl();

# define _up_showledon() \
  _up_putch('+'); \
  _up_puthex(led); _up_putch(':'); \
  _up_puthex(g_ledstate); _up_putch(':'); \
  _up_puthex(p82c55_port_e); _up_putnl();

# define _up_showledoff() \
  _up_putch('-'); \
  _up_puthex(led); _up_putch(':'); \
  _up_puthex(g_ledstate); _up_putch(':'); \
  _up_puthex(p82c55_port_e); _up_putnl();

#else

# define _up_showledinit()
# define _up_showledreset()
# define _up_showledon()
# define _up_showledoff()

#endif

/************************************************************************
 * Public Funtions
 ************************************************************************/

/************************************************************************
 * Name: up_ledinit
 ************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledinit(void)
{
  /* Set all ports as outputs */

  p82c55_def_config = 128;

  /* Turn LED 1-7 off; turn LED 0 on */

  g_ledstate    = 0xfe;
  p82c55_port_e = g_ledstate;

  _up_showledinit();
}

/************************************************************************
 * Name: up_ledon
 ************************************************************************/

void up_ledon(uint8_t led)
{
  /* This may be called from an interrupt handler */

  irqstate_t flags = irqsave();

#ifdef RESET_KLUDGE_NEEDED
  /* I don't understand why this happens yet, but sometimes
   * it is necessary to reconfigure port E.
   */

  if (g_ledstate != p82c55_port_e)
    {
      _up_showledreset();
      p82c55_def_config = 128;
    }
#endif

  /* Clear the bit in port E corresponding to LED to turn it on */

  if (led < 8)
    {
      g_ledstate   &= ~(g_ntobit[led]);
      p82c55_port_e = g_ledstate;
    }

  _up_showledon();
  irqrestore(flags);
}

/************************************************************************
 * Name: up_ledoff
 ************************************************************************/

void up_ledoff(uint8_t led)
{
  /* This may be called from an interrupt handler */

  irqstate_t flags = irqsave();

#ifdef RESET_KLUDGE_NEEDED
  /* I don't understand why this happens yet, but sometimes
   * it is necessary to reconfigure port E.
   */

  if (g_ledstate != p82c55_port_e)
    {
      _up_showledreset();
      p82c55_def_config = 128;
    }
#endif

  /* Set the bit in port E corresponding to LED to turn it off */

  if (led < 8)
    {
      g_ledstate   |= g_ntobit[led];
      p82c55_port_e = g_ledstate;
    }

  _up_showledoff();
  irqrestore(flags);
}
#endif /* CONFIG_ARCH_LEDS */
