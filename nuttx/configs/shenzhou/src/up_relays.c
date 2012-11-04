/****************************************************************************
 * configs/shenzhou/src/up_relays.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Darcy Gong
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
#include <unistd.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "shenzhou-internal.h"

#ifdef CONFIG_ARCH_RELAYS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RELAYS_MIN_RESET_TIME 5
#define RELAYS_RESET_MTIME 5
#define RELAYS_POWER_MTIME 50

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_relays_stat = 0;
static bool g_relays_init = false;

static const uint16_t g_relays[NUM_RELAYS] =
{
  GPIO_RELAYS_R00
#ifdef GPIO_RELAYS_R01
  , GPIO_RELAYS_R01
#endif
#ifdef GPIO_RELAYS_R02
  , GPIO_RELAYS_R02
#endif
#ifdef GPIO_RELAYS_R03
  , GPIO_RELAYS_R03
#endif
#ifdef GPIO_RELAYS_R04
  , GPIO_RELAYS_R04
#endif
#ifdef GPIO_RELAYS_R05
  , GPIO_RELAYS_R05
#endif
#ifdef GPIO_RELAYS_R06
  , GPIO_RELAYS_R06
#endif
#ifdef GPIO_RELAYS_R07
  , GPIO_RELAYS_R07
#endif
#ifdef GPIO_RELAYS_R08
  , GPIO_RELAYS_R08
#endif
#ifdef GPIO_RELAYS_R09
  , GPIO_RELAYS_R09
#endif
#ifdef GPIO_RELAYS_R10
  , GPIO_RELAYS_R10
#endif
#ifdef GPIO_RELAYS_R11
  , GPIO_RELAYS_R11
#endif
#ifdef GPIO_RELAYS_R12
  , GPIO_RELAYS_R12
#endif
#ifdef GPIO_RELAYS_R13
  , GPIO_RELAYS_R13
#endif
#ifdef GPIO_RELAYS_R14
  , GPIO_RELAYS_R14
#endif
#ifdef GPIO_RELAYS_R15
  , GPIO_RELAYS_R15
#endif
#ifdef GPIO_RELAYS_R16
  , GPIO_RELAYS_R16
#endif
#ifdef GPIO_RELAYS_R17
  , GPIO_RELAYS_R17
#endif
#ifdef GPIO_RELAYS_R18
  , GPIO_RELAYS_R18
#endif
#ifdef GPIO_RELAYS_R19
  , GPIO_RELAYS_R19
#endif
#ifdef GPIO_RELAYS_R20
  , GPIO_RELAYS_R20
#endif
#ifdef GPIO_RELAYS_R21
  , GPIO_RELAYS_R21
#endif
#ifdef GPIO_RELAYS_R22
  , GPIO_RELAYS_R22
#endif
#ifdef GPIO_RELAYS_R23
  , GPIO_RELAYS_R23
#endif
#ifdef GPIO_RELAYS_R24
  , GPIO_RELAYS_R24
#endif
#ifdef GPIO_RELAYS_R25
  , GPIO_RELAYS_R25
#endif
#ifdef GPIO_RELAYS_R26
  , GPIO_RELAYS_R26
#endif
#ifdef GPIO_RELAYS_R27
  , GPIO_RELAYS_R27
#endif
#ifdef GPIO_RELAYS_R28
  , GPIO_RELAYS_R28
#endif
#ifdef GPIO_RELAYS_R29
  , GPIO_RELAYS_R29
#endif
#ifdef GPIO_RELAYS_R30
  , GPIO_RELAYS_R30
#endif
#ifdef GPIO_RELAYS_R31
  , GPIO_RELAYS_R31
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_relaysinit(void)
{
  int i;

  if (g_relays_init)
    {
      return;
    }

  /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are
   * configured for some pins but NOT used in this file
   */

  for (i = 0; i < NUM_RELAYS; i++)
    {
      stm32_configgpio(g_relays[i]);
      stm32_gpiowrite(g_relays[i], false);
    }

  g_relays_init = true;
}

void relays_setstat(int relays,bool stat)
{
  if ((unsigned)relays < NUM_RELAYS)
    {
      stm32_gpiowrite(g_relays[relays], stat);
      if (!stat)
        {
          g_relays_stat &= ~(1 << relays);
        }
      else
        {
          g_relays_stat |= (1 << relays);
        }
    }
}

bool relays_getstat(int relays)
{
  if ((unsigned)relays < NUM_RELAYS)
    {
      return (g_relays_stat & (1 << relays)) != 0;
    }

  return false;
}

void relays_setstats(uint32_t relays_stat)
{
  int i;

  for (i = 0; i < NUM_RELAYS; i++)
    {
      relays_setstat(i, (relays_stat & (1<<i))!=0);
    }
}

uint32_t relays_getstats(void)
{
  return (uint32_t)g_relays_stat;
}

void relays_onoff(int relays, uint32_t mdelay)
{
  if ((unsigned)relays < NUM_RELAYS)
    {
      if (mdelay>0)
        {
          if (relays_getstat(relays))
            {
              relays_setstat(relays, false);
              usleep(RELAYS_MIN_RESET_TIME*1000*1000);
            }

          relays_setstat(relays,true);
          usleep(mdelay*100*1000);
          relays_setstat(relays, false);
        }
    }
}

void relays_onoffs(uint32_t relays_stat, uint32_t mdelay)
{
  int i;

  for (i = 0; i < NUM_RELAYS; i++)
    {
      relays_onoff(i, mdelay);
    }
}

void relays_resetmode(int relays)
{
  relays_onoff(relays, RELAYS_RESET_MTIME);
}

void relays_powermode(int relays)
{
  relays_onoff(relays, RELAYS_POWER_MTIME);
}

void relays_resetmodes(uint32_t relays_stat)
{
  relays_onoffs(relays_stat, RELAYS_RESET_MTIME);
}

void relays_powermodes(uint32_t relays_stat)
{
  relays_onoffs(relays_stat, RELAYS_POWER_MTIME);
}

#endif /* CONFIG_ARCH_BUTTONS */
