/****************************************************************************
 * boards/arm/rp23xx/raspberrypi-pico-2/src/rp23xx_gpio.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "board_config.h"
#include <nuttx/config.h>

#include <sys/types.h>
#include <syslog.h>
#include <arch/irq.h>
#include <nuttx/irq.h>

#include <assert.h>
#include <debug.h>

#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "rp23xx_gpio.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/* Output pins. GPIO25 is onboard LED any other outputs could be used.
 */

#define GPIO_OUT1     25

/* Input pins.
 */

#define GPIO_IN1      6

/* Interrupt pins.
 */

#define GPIO_IRQPIN1  14

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp23xxgpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct rp23xxgpint_dev_s
{
  struct rp23xxgpio_dev_s rp23xxgpio;
  pin_interrupt_t callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);
#endif

#if BOARD_NGPIOIN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value);
#endif

#if BOARD_NGPIOINT > 0
static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_OUT1
};

static struct rp23xxgpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOIN > 0
static const struct gpio_operations_s gpin_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};

/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
  GPIO_IN1
};

static struct rp23xxgpio_dev_s g_gpin[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOINT > 0
static const struct gpio_operations_s gpint_ops =
{
  .go_read   = gpint_read,
  .go_write  = NULL,
  .go_attach = gpint_attach,
  .go_enable = gpint_enable,
};

/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
  GPIO_IRQPIN1,
};

static struct rp23xxgpint_dev_s g_gpint[BOARD_NGPIOINT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpout_read
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct rp23xxgpio_dev_s *rp23xxgpio =
    (struct rp23xxgpio_dev_s *)dev;

  DEBUGASSERT(rp23xxgpio != NULL && value != NULL);
  DEBUGASSERT(rp23xxgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = rp23xx_gpio_get(g_gpiooutputs[rp23xxgpio->id]);
  return OK;
}

/****************************************************************************
 * Name: gpout_write
 ****************************************************************************/

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct rp23xxgpio_dev_s *rp23xxgpio =
    (struct rp23xxgpio_dev_s *)dev;

  DEBUGASSERT(rp23xxgpio != NULL);
  DEBUGASSERT(rp23xxgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  rp23xx_gpio_put(g_gpiooutputs[rp23xxgpio->id], value);
  return OK;
}
#endif

/****************************************************************************
 * Name: gpin_read
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct rp23xxgpio_dev_s *rp23xxgpio =
    (struct rp23xxgpio_dev_s *)dev;

  DEBUGASSERT(rp23xxgpio != NULL && value != NULL);
  DEBUGASSERT(rp23xxgpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading... pin %d\n", (int)g_gpioinputs[rp23xxgpio->id]);

  *value = rp23xx_gpio_get(g_gpioinputs[rp23xxgpio->id]);
  return OK;
}
#endif

/****************************************************************************
 * Name: rp23xxgpio_interrupt
 ****************************************************************************/

#if BOARD_NGPIOINT > 0
static int rp23xxgpio_interrupt(int irq, void *context, void *arg)
{
  struct rp23xxgpint_dev_s *rp23xxgpint =
    (struct rp23xxgpint_dev_s *)arg;

  DEBUGASSERT(rp23xxgpint != NULL && rp23xxgpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", rp23xxgpint->callback);

  rp23xxgpint->callback(&rp23xxgpint->rp23xxgpio.gpio,
                       rp23xxgpint->rp23xxgpio.id);
  return OK;
}

/****************************************************************************
 * Name: gpint_read
 ****************************************************************************/

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct rp23xxgpint_dev_s *rp23xxgpint =
    (struct rp23xxgpint_dev_s *)dev;

  DEBUGASSERT(rp23xxgpint != NULL && value != NULL);
  DEBUGASSERT(rp23xxgpint->rp23xxgpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = rp23xx_gpio_get(g_gpiointinputs[rp23xxgpint->rp23xxgpio.id]);
  return OK;
}

/****************************************************************************
 * Name: gpint_attach
 ****************************************************************************/

static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
  struct rp23xxgpint_dev_s *rp23xxgpint =
    (struct rp23xxgpint_dev_s *)dev;
  int irq = g_gpiointinputs[rp23xxgpint->rp23xxgpio.id];
  int ret;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  rp23xx_gpio_disable_irq(irq);
  ret = rp23xx_gpio_irq_attach(irq,
                               RP23XX_GPIO_INTR_EDGE_LOW,
                               rp23xxgpio_interrupt,
                               &g_gpint[rp23xxgpint->rp23xxgpio.id]);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: gpint_attach() failed: %d\n", ret);
      return ret;
    }

  gpioinfo("Attach %p\n", callback);
  rp23xxgpint->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: gpint_enable
 ****************************************************************************/

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct rp23xxgpint_dev_s *rp23xxgpint =
    (struct rp23xxgpint_dev_s *)dev;
  int irq = g_gpiointinputs[rp23xxgpint->rp23xxgpio.id];

  if (enable)
    {
      if (rp23xxgpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for rising edge */

          rp23xx_gpio_enable_irq(irq);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      rp23xx_gpio_disable_irq(irq);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_dev_gpio_init
 ****************************************************************************/

int rp23xx_dev_gpio_init(void)
{
  int i;
  int pincount = 0;

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register(&g_gpout[i].gpio, g_gpiooutputs[i]);

      /* Configure the pins that will be used as output */

      rp23xx_gpio_init(g_gpiooutputs[i]);
      rp23xx_gpio_setdir(g_gpiooutputs[i], true);
      rp23xx_gpio_put(g_gpiooutputs[i], false);

      pincount++;
    }
#endif

  pincount = 0;

#if BOARD_NGPIOIN > 0
  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register(&g_gpin[i].gpio, g_gpioinputs[i]);

      /* Configure the pins that will be used as INPUT */

      rp23xx_gpio_init(g_gpioinputs[i]);

      pincount++;
    }
#endif

  pincount = 0;

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].rp23xxgpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].rp23xxgpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].rp23xxgpio.id              = i;
      gpio_pin_register(&g_gpint[i].rp23xxgpio.gpio, g_gpiointinputs[i]);

      /* Configure the pins that will be used as interrupt input */

      rp23xx_gpio_init(g_gpiointinputs[i]);

      /* pull-up = false : pull-down = true */

      rp23xx_gpio_set_pulls(g_gpiointinputs[i], false, true);

      pincount++;
    }
#endif

  return OK;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
