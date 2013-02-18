/****************************************************************************
 * examples/buttons/buttons_main.c
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
 * NOTE: This test exercises internal button driver interfaces.  As such, it
 * it relies on internal OS interfaces that are not normally available to a
 * user-space program.  As a result, this example cannot be used if a
 * NuttX is built as a protected, supervisor kernel (CONFIG_NUTTX_KERNEL).
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <debug.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_ARCH_BUTTONS
#  error "CONFIG_ARCH_BUTTONS is not defined in the configuration"
#endif

#ifndef CONFIG_EXAMPLES_BUTTONS_NAME0
#  define CONFIG_EXAMPLES_BUTTONS_NAME0 "BUTTON0"
#endif
#ifndef CONFIG_EXAMPLES_BUTTONS_NAME1
#  define CONFIG_EXAMPLES_BUTTONS_NAME1 "BUTTON1"
#endif
#ifndef CONFIG_EXAMPLES_BUTTONS_NAME2
#  define CONFIG_EXAMPLES_BUTTONS_NAME2 "BUTTON2"
#endif
#ifndef CONFIG_EXAMPLES_BUTTONS_NAME3
#  define CONFIG_EXAMPLES_BUTTONS_NAME3 "BUTTON3"
#endif
#ifndef CONFIG_EXAMPLES_BUTTONS_NAME4
#  define CONFIG_EXAMPLES_BUTTONS_NAME4 "BUTTON4"
#endif
#ifndef CONFIG_EXAMPLES_BUTTONS_NAME5
#  define CONFIG_EXAMPLES_BUTTONS_NAME5 "BUTTON5"
#endif
#ifndef CONFIG_EXAMPLES_BUTTONS_NAME6
#  define CONFIG_EXAMPLES_BUTTONS_NAME6 "BUTTON6"
#endif
#ifndef CONFIG_EXAMPLES_BUTTONS_NAME7
#  define CONFIG_EXAMPLES_BUTTONS_NAME7 "BUTTON7"
#endif

#define BUTTON_MIN 0
#define BUTTON_MAX 7

#ifndef CONFIG_EXAMPLES_BUTTONS_MIN
#  define CONFIG_EXAMPLES_BUTTONS_MIN BUTTON_MIN
#endif
#ifndef CONFIG_EXAMPLES_BUTTONS_MAX
#  define CONFIG_EXAMPLES_BUTTONS_MAX BUTTON_MAX
#endif

#if CONFIG_EXAMPLES_BUTTONS_MIN > CONFIG_EXAMPLES_BUTTONS_MAX
#  error "CONFIG_EXAMPLES_BUTTONS_MIN > CONFIG_EXAMPLES_BUTTONS_MAX"
#endif
#if CONFIG_EXAMPLES_BUTTONS_MAX > 7
#  error "CONFIG_EXAMPLES_BUTTONS_MAX > 7"
#endif

#ifndef CONFIG_EXAMPLES_IRQBUTTONS_MIN
#  define CONFIG_EXAMPLES_IRQBUTTONS_MIN CONFIG_EXAMPLES_BUTTONS_MIN
#endif
#ifndef CONFIG_EXAMPLES_IRQBUTTONS_MAX
#  define CONFIG_EXAMPLES_IRQBUTTONS_MAX CONFIG_EXAMPLES_BUTTONS_MAX
#endif

#if CONFIG_EXAMPLES_IRQBUTTONS_MIN > CONFIG_EXAMPLES_IRQBUTTONS_MAX
#  error "CONFIG_EXAMPLES_IRQBUTTONS_MIN > CONFIG_EXAMPLES_IRQBUTTONS_MAX"
#endif
#if CONFIG_EXAMPLES_IRQBUTTONS_MAX > 7
#  error "CONFIG_EXAMPLES_IRQBUTTONS_MAX > 7"
#endif

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif
#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

#define MIN_BUTTON MIN(CONFIG_EXAMPLES_BUTTONS_MIN, CONFIG_EXAMPLES_IRQBUTTONS_MIN)
#define MAX_BUTTON MAX(CONFIG_EXAMPLES_BUTTONS_MAX, CONFIG_EXAMPLES_IRQBUTTONS_MAX)

#define NUM_BUTTONS     (MAX_BUTTON - MIN_BUTTON + 1)
#define BUTTON_INDEX(b) ((b)-MIN_BUTTON)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct button_info_s
{
  FAR const char *name; /* Name for the button */
#ifdef CONFIG_ARCH_IRQBUTTONS
  xcpt_t handler;       /* Button interrupt handler */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void show_buttons(uint8_t oldset, uint8_t newset);

#ifdef CONFIG_ARCH_IRQBUTTONS
static void button_handler(int id, int irq);

#if MIN_BUTTON < 1
static int button0_handler(int irq, FAR void *context);
#endif
#if MIN_BUTTON < 2 && MAX_BUTTON > 0
static int button1_handler(int irq, FAR void *context);
#endif
#if MIN_BUTTON < 3 && MAX_BUTTON > 1
static int button2_handler(int irq, FAR void *context);
#endif
#if MIN_BUTTON < 4 && MAX_BUTTON > 2
static int button3_handler(int irq, FAR void *context);
#endif
#if MIN_BUTTON < 5 && MAX_BUTTON > 3
static int button4_handler(int irq, FAR void *context);
#endif
#if MIN_BUTTON < 6 && MAX_BUTTON > 4
static int button5_handler(int irq, FAR void *context);
#endif
#if MIN_BUTTON < 7 && MAX_BUTTON > 5
static int button6_handler(int irq, FAR void *context);
#endif
#if MAX_BUTTON > 6
static int button7_handler(int irq, FAR void *context);
#endif
#endif /* CONFIG_ARCH_IRQBUTTONS */

/****************************************************************************
 * Private Data
 ****************************************************************************/

 /* Button Names */

static const struct button_info_s g_buttoninfo[NUM_BUTTONS] =
{
#if MIN_BUTTON < 1
  {
    CONFIG_EXAMPLES_BUTTONS_NAME0,
#ifdef CONFIG_ARCH_IRQBUTTONS
    button0_handler
#endif
  },
#endif
#if MIN_BUTTON < 2 && MAX_BUTTON > 0
  {
    CONFIG_EXAMPLES_BUTTONS_NAME1,
#ifdef CONFIG_ARCH_IRQBUTTONS
    button1_handler
#endif
  },
#endif
#if MIN_BUTTON < 3 && MAX_BUTTON > 1
  {
    CONFIG_EXAMPLES_BUTTONS_NAME2,
#ifdef CONFIG_ARCH_IRQBUTTONS
    button2_handler
#endif
  },
#endif
#if MIN_BUTTON < 4 && MAX_BUTTON > 2
  {
    CONFIG_EXAMPLES_BUTTONS_NAME3,
#ifdef CONFIG_ARCH_IRQBUTTONS
    button3_handler
#endif
  },
#endif
#if MIN_BUTTON < 5 && MAX_BUTTON > 3
  {
    CONFIG_EXAMPLES_BUTTONS_NAME4,
#ifdef CONFIG_ARCH_IRQBUTTONS
    button4_handler
#endif
  },
#endif
#if MIN_BUTTON < 6 && MAX_BUTTON > 4
  {
    CONFIG_EXAMPLES_BUTTONS_NAME5,
#ifdef CONFIG_ARCH_IRQBUTTONS
    button5_handler
#endif
  },
#endif
#if MIN_BUTTON < 7 && MAX_BUTTON > 5
  {
    CONFIG_EXAMPLES_BUTTONS_NAME6,
#ifdef CONFIG_ARCH_IRQBUTTONS
    button6_handler
#endif
  },
#endif
#if MAX_BUTTON > 6
  {
    CONFIG_EXAMPLES_BUTTONS_NAME7,
#ifdef CONFIG_ARCH_IRQBUTTONS
    button7_handler
#endif
  }
#endif
};

/* Last sampled button set */

static uint8_t g_oldset;

/* Used to limit the number of button presses */

#ifdef CONFIG_NSH_BUILTIN_APPS
static volatile long g_nbuttons;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_buttons(uint8_t oldset, uint8_t newset)
{
  uint8_t chgset = oldset ^ newset;
  int i;

  /* Update the count of button presses shown */

#ifdef CONFIG_NSH_BUILTIN_APPS
  if ((chgset & newset) != 0)
    {
      g_nbuttons++;
    }
#endif

  /* Show each button state change */

  for (i = MIN_BUTTON; i <= MAX_BUTTON; i++)
    {
      uint8_t mask = (1 << i);
      if ((chgset & mask) != 0)
        {
          FAR const char *state;

          /* Get the button state */

          if ((newset & mask) != 0)
            {
              state = "depressed";
            }
          else
            {
              state = "released";            
            }

          /* Use lowsyslog() because we make be executing from an
           * interrupt handler.
           */

          lowsyslog("  %s %s\n", g_buttoninfo[BUTTON_INDEX(i)].name, state);
        }
    }
}

#ifdef CONFIG_ARCH_IRQBUTTONS
static void button_handler(int id, int irq)
{
  uint8_t newset = up_buttons();

  lowsyslog("IRQ:%d Button %d:%s SET:%02x:\n",
            irq, id, g_buttoninfo[BUTTON_INDEX(id)].name, newset);
  show_buttons(g_oldset, newset);
  g_oldset = newset;
}

#if MIN_BUTTON < 1
static int button0_handler(int irq, FAR void *context)
{
  button_handler(0, irq);
  return OK;
}
#endif

#if MIN_BUTTON < 2 && MAX_BUTTON > 0
static int button1_handler(int irq, FAR void *context)
{
  button_handler(1, irq);
  return OK;
}
#endif

#if MIN_BUTTON < 3 && MAX_BUTTON > 1
static int button2_handler(int irq, FAR void *context)
{
  button_handler(2, irq);
  return OK;
}
#endif

#if MIN_BUTTON < 4 && MAX_BUTTON > 2
static int button3_handler(int irq, FAR void *context)
{
  button_handler(3, irq);
  return OK;
}
#endif

#if MIN_BUTTON < 5 && MAX_BUTTON > 3
static int button4_handler(int irq, FAR void *context)
{
  button_handler(4, irq);
  return OK;
}
#endif

#if MIN_BUTTON < 6 && MAX_BUTTON > 4
static int button5_handler(int irq, FAR void *context)
{
  button_handler(5, irq);
  return OK;
}
#endif

#if MIN_BUTTON < 7 && MAX_BUTTON > 5
static int button6_handler(int irq, FAR void *context)
{
  button_handler(6, irq);
  return OK;
}
#endif

#if MAX_BUTTON > 6
static int button7_handler(int irq, FAR void *context)
{
  button_handler(7, irq);
  return OK;
}
#endif
#endif /* CONFIG_ARCH_IRQBUTTONS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * buttons_main
 ****************************************************************************/

int buttons_main(int argc, char *argv[])
{
  uint8_t newset;
  irqstate_t flags;
  int i;

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

#ifdef CONFIG_NSH_BUILTIN_APPS
  long maxbuttons = 1;
  g_nbuttons      = 0;
  if (argc > 1)
    {
      maxbuttons = strtol(argv[1], NULL, 10);
    }
  lowsyslog("maxbuttons: %d\n", maxbuttons);
#endif

  /* Initialize the button GPIOs */

  up_buttoninit();

  /* Register to recieve button interrupts */

#ifdef CONFIG_ARCH_IRQBUTTONS
  for (i = CONFIG_EXAMPLES_IRQBUTTONS_MIN; i <= CONFIG_EXAMPLES_IRQBUTTONS_MAX; i++)
    {
      xcpt_t oldhandler = up_irqbutton(i, g_buttoninfo[BUTTON_INDEX(i)].handler);

      /* Use lowsyslog() for compatibility with interrrupt handler output. */

      lowsyslog("Attached handler at %p to button %d [%s], oldhandler:%p\n",
                g_buttoninfo[BUTTON_INDEX(i)].handler, i,
                g_buttoninfo[BUTTON_INDEX(i)].name, oldhandler);

      /* Some hardware multiplexes different GPIO button sources to the same
       * physical interrupt.  If we register multiple such multiplexed button
       * interrupts, then the second registration will overwrite the first.  In
       * this case, the first button interrupts may be aliased to the second
       * interrupt handler (or worse, could be lost).
       */

      if (oldhandler != NULL)
        {
          lowsyslog("WARNING: oldhandler:%p is not NULL!  "
                    "Button events may be lost or aliased!\n",
                    oldhandler);
        }
    }
#endif

  /* Poll button state */

  g_oldset = up_buttons();
#ifdef CONFIG_NSH_BUILTIN_APPS
  while (g_nbuttons < maxbuttons)
#else
  for (;;)
#endif
    {
      /* Get the set of pressed and release buttons. */

      newset = up_buttons();

      /* Any changes from the last sample? */

      if (newset != g_oldset)
        {
          /* Disable interrupts so that output here will not collide with
           * output from an interrupt handler.
           */

          flags = irqsave();

          /* Use lowsyslog() for compatibility with interrrupt handler
           * output.
           */

          lowsyslog("POLL SET:%02x:\n", newset);
          show_buttons(g_oldset, newset);
          g_oldset = newset;
          irqrestore(flags);
        }

      /* Sleep a little... but not long.  This will determine how fast we
       * poll for button changes.
       */

      usleep(150000); /* 150 Milliseconds */
    }

  /* Un-register button handlers */

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_NSH_BUILTIN_APPS)
  for (i = CONFIG_EXAMPLES_IRQBUTTONS_MIN; i <= CONFIG_EXAMPLES_IRQBUTTONS_MAX; i++)
    {
      (void)up_irqbutton(i, NULL);
    }
#endif

  return 0;
}

