/****************************************************************************
 * arch/arm/src/stm32/stm32_rng.c
 *
 *   Copyright (C) 2012 Max Holtzberg. All rights reserved.
 *   Author: Max Holtzberg <mh@uvc.de>
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

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "up_arch.h"
#include "chip/stm32_rng.h"
#include "up_internal.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_rnginitialize(void);
static int stm32_interrupt(int irq, void *context);
static void stm32_enable(void);
static void stm32_disable(void);
static ssize_t stm32_read(struct file *filep, char *buffer, size_t);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_dev_s
{
  sem_t rd_devsem;      /* Threads can only exclusively access the RNG */
  sem_t rd_readsem;     /* To block until the buffer is filled  */
  char *rd_buf;
  size_t rd_buflen;
  uint32_t rd_lastval;
  bool rd_first;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_dev_s g_rngdev;

static const struct file_operations g_rngops =
{
  0,               /* open */
  0,               /* close */
  stm32_read,      /* read */
  0,               /* write */
  0,               /* seek */
  0                /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  ,0               /* poll */
#endif
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

static int stm32_rnginitialize()
{
  uint32_t regval;

  vdbg("Initializing RNG\n");

  memset(&g_rngdev, 0, sizeof(struct rng_dev_s));

  sem_init(&g_rngdev.rd_devsem, 0, 1);

  if (irq_attach(STM32_IRQ_RNG, stm32_interrupt))
    {
      /* We could not attach the ISR to the interrupt */

      vdbg("Could not attach IRQ.\n");

      return -EAGAIN;
    }

  /* Enable interrupts */

  regval = getreg32(STM32_RNG_CR);
  regval |=  RNG_CR_IE;
  putreg32(regval, STM32_RNG_CR);

  up_enable_irq(STM32_IRQ_RNG);

  return OK;
}

static void stm32_enable()
{
  uint32_t regval;

  g_rngdev.rd_first = true;

  regval = getreg32(STM32_RNG_CR);
  regval |= RNG_CR_RNGEN;
  putreg32(regval, STM32_RNG_CR);
}

static void stm32_disable()
{
  uint32_t regval;
  regval = getreg32(STM32_RNG_CR);
  regval &= ~RNG_CR_RNGEN;
  putreg32(regval, STM32_RNG_CR);
}

static int stm32_interrupt(int irq, void *context)
{
  uint32_t rngsr;
  uint32_t data;

  rngsr = getreg32(STM32_RNG_SR);

  if ((rngsr & (RNG_SR_SEIS | RNG_SR_CEIS)) /* Check for error bits */
      || !(rngsr & RNG_SR_DRDY)) /* Data ready must be set */
    {
      /* This random value is not valid, we will try again. */

      return OK;
    }

  data = getreg32(STM32_RNG_DR);

  /* As required by the FIPS PUB (Federal Information Processing Standard
   * Publication) 140-2, the first random number generated after setting the
   * RNGEN bit should not be used, but saved for comparison with the next
   * generated random number. Each subsequent generated random number has to be
   * compared with the previously generated number. The test fails if any two
   * compared numbers are equal (continuous random number generator test).
   */

  if (g_rngdev.rd_first)
    {
      g_rngdev.rd_first = false;
      g_rngdev.rd_lastval = data;
      return OK;
    }

  if (g_rngdev.rd_lastval == data)
    {
      /* Two subsequent same numbers, we will try again. */

      return OK;
    }

  /* If we get here, the random number is valid. */

  g_rngdev.rd_lastval = data;

  if (g_rngdev.rd_buflen >= 4)
    {
      g_rngdev.rd_buflen -= 4;
      *(uint32_t*)&g_rngdev.rd_buf[g_rngdev.rd_buflen] = data;
    }
  else
    {
      while (g_rngdev.rd_buflen > 0)
        {
          g_rngdev.rd_buf[--g_rngdev.rd_buflen] = (char)data;
          data >>= 8;
        }
    }

  if (g_rngdev.rd_buflen == 0)
    {
      /* Buffer filled, stop further interrupts. */

      stm32_disable();
      sem_post(&g_rngdev.rd_readsem);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_read
 ****************************************************************************/

static ssize_t stm32_read(struct file *filep, char *buffer, size_t buflen)
{
  if (sem_wait(&g_rngdev.rd_devsem) != OK)
    {
      return -errno;
    }
  else
    {
      /* We've got the semaphore. */

      /* Initialize semaphore with 0 for blocking until the buffer is filled from
       * interrupts.
       */

      sem_init(&g_rngdev.rd_readsem, 0, 1);

      g_rngdev.rd_buflen = buflen;
      g_rngdev.rd_buf = buffer;

      /* Enable RNG with interrupts */

      stm32_enable();

      /* Wait until the buffer is filled */

      sem_wait(&g_rngdev.rd_readsem);

      /* Free RNG for next use */

      sem_post(&g_rngdev.rd_devsem);

      return buflen;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_rnginitialize()
{
  stm32_rnginitialize();
  register_driver("/dev/random", &g_rngops, 0444, NULL);
}
