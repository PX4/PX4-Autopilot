/****************************************************************************
 * arch/arm/src/stm32/stm32_rcc.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Author: Uros Platise <uros.platise@isotel.eu>
 *
 * This file is part of NuttX:
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Activity reference count, showing inactivity after start-up.
 * Device drivers increment this count using rcclock() and rccunlock()
 * 
 * If this value goes beyond the range [0, MAX_RCCs] indicates
 * reference count leakage (asymetric number of locks vs. unlocks) and
 * system enters permanent active state.
 */

static int stm32_rcclock_count = 0; 
 
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t stm32_rcclock(uint8_t domain_id)
{
  // THINK:
  // maybe just shift domain_id into 32-bit or 64-bit register 
  // and if there value of this var != 0, we are active...
  // increment some variable, so it is possible to test leakage
  // multiple locks or multiple unlocks
    
  if (stm32_rcclock_count >= 0)
    {
      stm32_rcclock_count++;
      if (stm32_rcclock_count > 64)
        {
          stm32_rcclock_count = -1; /* capture error */
        }
    }
    
  return 0;
}

uint32_t stm32_rccunlock(uint8_t domain_id)
{
  if (stm32_rcclock_count > -1)
    {
      stm32_rcclock_count--;
    }
  return 0;
}

uint32_t stm32_setrccoptions(uint8_t domain_id, uint32_t options)
{
  return 0;
}

int stm32_getrccactivity(void)
{
  return stm32_rcclock_count;
}
