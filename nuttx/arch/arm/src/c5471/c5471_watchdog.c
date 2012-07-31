/**************************************************************************
 * c5471/c5471_watchdog.c
 *
 *   Copyright (C) 2007, 2009, 2012 Gregory Nutt. All rights reserved.
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
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <arch/watchdog.h>

#include "chip.h"
#include "up_arch.h"

/**************************************************************************
 * Definitions
 **************************************************************************/

#undef CONFIG_SOFTWARE_TEST
#undef CONFIG_SOFTWARE_REBOOT
#undef CONFIG_WATCHDOG_STRICT

#define MAX_WDT_USEC           353200
#define MAX_PRESCALER          256
#define C5471_TIMER_STOP       0

#define C5471_TIMER_PRESCALER  0x07          /* Bits 0-2: Prescale value */
#define C5471_TIMER_STARTBIT   (1 << 3)      /* Bit 3: Start timer bit */
#define C5471_TIMER_AUTORELOAD (1 << 4)      /* Bit 4: Auto-reload timer */
#define C5471_TIMER_LOADTIM    (0xffff << 5) /* Bits 20-5: Load timer value */
#define C5471_TIMER_MODE       (1 << 21)     /* Bit 21: Timer mode */
#define C5471_DISABLE_VALUE1   (0xf5 << 22)  /* Bits 29-22: WD disable */
#define C5471_DISABLE_VALUE2   (0xa0 << 22)

#define CLOCK_KHZ              47500
#define CLOCK_MHZx2            95

/* Macros to manage access to to watchdog timer macros */

#define c5471_wdt_cntl  (*(volatile uint32_t*)C5471_TIMER0_CTRL)
#define c5471_wdt_count (*(volatile uint32_t*)C5471_TIMER0_CNT)

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/* Local implementation of timer interface */

static inline unsigned int wdt_prescaletoptv(unsigned int prescale);

static int     wdt_setusec(uint32_t usec);
static int     wdt_interrupt(int irq, void *context);

static int     wdt_open(struct file *filep);
static int     wdt_close(struct file *filep);
static ssize_t wdt_read(struct file *filep, char *buffer, size_t buflen);
static ssize_t wdt_write(struct file *filep, const char *buffer, size_t buflen);
static int     wdt_ioctl(FAR struct file *filp, int cmd, unsigned long arg);

/**************************************************************************
 * Private Data
 **************************************************************************/

static bool g_wdtopen;

static const struct file_operations g_wdtops =
{
  .open  = wdt_open,
  .close = wdt_close,
  .read  = wdt_read,
  .write = wdt_write,
  .ioctl = wdt_ioctl,
};

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Name: wdt_prescaletoptv
 **************************************************************************/

static inline unsigned int wdt_prescaletoptv(unsigned int prescale)
{
  unsigned int ptv = 0;

  if (prescale > 255)
    {
      ptv = 7;
    }
  else
    {
      unsigned int value = prescale >> 1;

      /* 0: 0-2
       * 1: 3-4
       * 2: 5-8
       * 3: 9-16
       * 4: 17-32
       * 5: 33-64
       * 6: 65-128
       * 7: 129-
       */

      while (value > 1)
        {
          value >>= 1;
          ptv++;
        }
    }

  dbg("prescale=%d -> ptv=%d\n", prescale, ptv);
  return ptv;
}

/**************************************************************************
 * Name: wdt_setusec
 **************************************************************************/

static int wdt_setusec(uint32_t usec)
{
  /* prescaler: clock / prescaler = #clock ticks per counter in ptv
   * divisor:   #counts until the interrupt comes.
   */

  uint32_t prescaler = MAX_PRESCALER;
  uint32_t divisor   = 1;
  uint32_t mode;

  dbg("usec=%d\n", usec);

  /* Calculate a value of prescaler and divisor that will be able
   * to count to the usec.  It may not be exact or the best
   * possible set, but it's a quick and simple algorithm.
   *
   *    divisor max = 0x10000
   *    prescaler max = MAX_PRESCALER
   */

  do
    {
      divisor = (CLOCK_MHZx2 * usec) / (prescaler * 2);
      dbg("divisor=0x%x prescaler=0x%x\n", divisor, prescaler);

      if (divisor >= 0x10000)
        {
          if (prescaler == MAX_PRESCALER)
            { 
              /* This is the max possible ~2.5 seconds. */

              dbg("prescaler=0x%x too big!\n", prescaler);
              return ERROR;
            }

          prescaler <<= 1;
          if (prescaler > MAX_PRESCALER)
            {
              prescaler = MAX_PRESCALER;
            }
        }
    }
  while (divisor >= 0x10000);

  dbg("prescaler=0x%x divisor=0x%x\n", prescaler, divisor);

  mode  = wdt_prescaletoptv(prescaler);
  mode &= ~C5471_TIMER_AUTORELOAD; /* One shot mode. */
  mode |= divisor << 5;
  dbg("mode=0x%x\n", mode);

  c5471_wdt_cntl = mode;

  /* Now start the watchdog */

  c5471_wdt_cntl |= C5471_TIMER_STARTBIT;
  dbg("cntl_timer=0x%x\n", c5471_wdt_cntl);

  return 0;
}

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Name: wdt_interrupt
 **************************************************************************/

static int wdt_interrupt(int irq, void *context)
{
  dbg("expired\n");

#if defined(CONFIG_SOFTWARE_REBOOT)
#  if defined(CONFIG_SOFTWARE_TEST)
  dbg("  Test only\n");
#  else
  dbg("  Re-booting\n");
#    warning "Add logic to reset CPU here"
#  endif
#else
  dbg("  No reboot\n");
#endif
  return OK;
}

/**************************************************************************
 * Name: wdt_read
 **************************************************************************/

static ssize_t wdt_read(struct file *filep, char *buffer, size_t buflen)
{
  /* We are going to return "NNNNNNNN NNNNNNNN."  The followig logic will
   * not work if the user provides a buffer smaller than 18 bytes.
   */

  dbg("buflen=%d\n", buflen);
  if (buflen >= 18)
    {
      sprintf(buffer, "#08x %08x\n", c5471_wdt_cntl, c5471_wdt_count);
      return 18;
    }
  return 0;
}

/**************************************************************************
 * Name: wdt_write
 **************************************************************************/

static ssize_t wdt_write(struct file *filep, const char *buffer, size_t buflen)
{
  dbg("buflen=%d\n", buflen);
  if (buflen)
    {
      /* Reset the timer to the maximum delay */

      wdt_setusec(MAX_WDT_USEC);
      return 1;
    }

  return 0;
}

/**************************************************************************
 * Name: wdt_ioctl
 **************************************************************************/

static int wdt_ioctl(FAR struct file *filp, int cmd, unsigned long arg)
{
  dbg("ioctl Call: cmd=0x%x arg=0x%x", cmd, arg);

  /* Process the IOCTL command (see arch/watchdog.h) */

  switch(cmd)
    {
    case WDIOC_KEEPALIVE:
      wdt_setusec(MAX_WDT_USEC);
      break;

    default:
      return -ENOTTY;
    }

  return OK;
}

/**************************************************************************
 * Name: wdt_open
 **************************************************************************/

static int wdt_open(struct file *filep)
{
  dbg("");

  if (g_wdtopen)
    {
      return -EBUSY;
    }

  /* This will automatically load the timer with its max
   * count and start it running.
   */

  c5471_wdt_cntl = C5471_DISABLE_VALUE1;
  c5471_wdt_cntl = C5471_DISABLE_VALUE2;

  g_wdtopen = true;
  return OK;
}

/**************************************************************************
 * Name: wdt_close
 **************************************************************************/

static int wdt_close(struct file *filep)
{
  dbg("");

  /* The task controlling the watchdog has terminated.  Take the timer
   *  the
   * watchdog in interrupt mode -- we are going to reset unless the
   * reopened again soon.
   */

#ifndef CONFIG_WATCHDOG_STRICT
  c5471_wdt_cntl = C5471_TIMER_MODE;
#endif

  g_wdtopen = false;
  return 0;
}

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_wdtinit
 **************************************************************************/

int up_wdtinit(void)
{
  int ret;

  dbg("C547x Watchdog Driver\n");

  /* Register as /dev/wdt */

  ret = register_driver("/dev/wdt", &g_wdtops, 0666, NULL);
  if (ret)
    {
      return ERROR;
    }

  /* Register for an interrupt level callback through wdt_interrupt */

  dbg("Attach to IRQ=%d\n", C5471_IRQ_WATCHDOG);

  /* Make sure that the timer is stopped */

  c5471_wdt_cntl = C5471_TIMER_STOP;

  /* Request the interrupt. */

  ret = irq_attach(C5471_IRQ_WATCHDOG, wdt_interrupt);
  if (ret)
    {
      unregister_driver("/dev/wdt");
      return ERROR;
    }

  return OK;
}
