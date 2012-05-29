/****************************************************************************
 * drivers/input/stmpe811_gpio.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "STMPE811 S-Touch® advanced resistive touchscreen controller with 8-bit
 *    GPIO expander," Doc ID 14489 Rev 6, CD00186725, STMicroelectronics"
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/input/stmpe811.h>

#include "stmpe811.h"

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_STMPE811) && !defined(CONFIG_STMPE811_GPIO_DISABLE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe811_gpioinit
 *
 * Description:
 *  Initialize the GPIO interrupt subsystem
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static void stmpe811_gpioinit(FAR struct stmpe811_dev_s *priv)
{
  uint8_t regval;

  if ((priv->flags & STMPE811_FLAGS_GPIO_INITIALIZED) == 0)
    {
      /* Enable Clocking for GPIO */

      regval = stmpe811_getreg8(priv, STMPE811_SYS_CTRL2);
      regval &= ~SYS_CTRL2_GPIO_OFF;
      stmpe811_putreg8(priv, STMPE811_SYS_CTRL2, regval);

      /* Disable all GPIO interrupts */

      stmpe811_putreg8(priv, STMPE811_GPIO_EN, 0);

      /* Enable global GPIO interrupts */

#ifndef CONFIG_STMPE811_GPIOINT_DISABLE
      regval = stmpe811_getreg8(priv, STMPE811_INT_EN);
      regval |= INT_GPIO;
      stmpe811_putreg8(priv, STMPE811_INT_EN, regval);
#endif

      priv->flags |= STMPE811_FLAGS_GPIO_INITIALIZED;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe811_gpioconfig
 *
 * Description:
 *  Configure an STMPE811 GPIO pin
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *   pinconfig - Bit-encoded pin configuration
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stmpe811_gpioconfig(STMPE811_HANDLE handle, uint8_t pinconfig)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  int pin = (pinconfig & STMPE811_GPIO_PIN_MASK) >> STMPE811_GPIO_PIN_SHIFT;
  uint8_t pinmask = (1 << pin);
  uint8_t regval;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < STMPE811_GPIO_NPINS);

  /* Get exclusive access to the device structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      int errval = errno;
      idbg("sem_wait failed: %d\n", errval);
      return -errval;
    }

  /* Make sure that the pin is not already in use */

  if ((priv->inuse & pinmask) != 0)
    {
      idbg("PIN%d is already in-use\n", pin);
      sem_post(&priv->exclsem);
      return -EBUSY;
    }

  /* Make sure that the GPIO block has been initialized */

  stmpe811_gpioinit(priv);

  /* Set the alternate function bit for the pin, making it a GPIO */

  regval  = stmpe811_getreg8(priv, STMPE811_GPIO_AF);
  regval |= pinmask;
  stmpe811_putreg8(priv, STMPE811_GPIO_AF, regval);

  /* Is the pin an input or an output? */

  if ((pinconfig & STMPE811_GPIO_DIR) == STMPE811_GPIO_OUTPUT)
    {
      /* The pin is an output */

      regval  = stmpe811_getreg8(priv, STMPE811_GPIO_DIR);
      regval &= ~pinmask;
      stmpe811_putreg8(priv, STMPE811_GPIO_DIR, regval);

      /* Set its initial output value */

      stmpe811_gpiowrite(handle, pinconfig,
                        (pinconfig & STMPE811_GPIO_VALUE) != STMPE811_GPIO_ZERO);
    }
  else
    {
      /* It is an input */

      regval  = stmpe811_getreg8(priv, STMPE811_GPIO_DIR);
      regval |= pinmask;
      stmpe811_putreg8(priv, STMPE811_GPIO_DIR, regval);

      /* Set up the falling edge detection */

      regval = stmpe811_getreg8(priv, STMPE811_GPIO_FE);
      if ((pinconfig & STMPE811_GPIO_FALLING) != 0)
        {
          regval |= pinmask;
        }
      else
        {
          regval &= pinmask;
        }
      stmpe811_putreg8(priv, STMPE811_GPIO_FE, regval);

      /* Set up the rising edge detection */

     regval = stmpe811_getreg8(priv, STMPE811_GPIO_RE);
      if ((pinconfig & STMPE811_GPIO_FALLING) != 0)
        {
          regval |= pinmask;
        }
      else
        {
          regval &= pinmask;
        }
      stmpe811_putreg8(priv, STMPE811_GPIO_RE, regval);
 
      /* Disable interrupts for now */

      regval = stmpe811_getreg8(priv, STMPE811_GPIO_EN);
      regval &= ~pinmask;
      stmpe811_putreg8(priv, STMPE811_GPIO_EN, regval);
    }

  /* Mark the pin as 'in use' */

  priv->inuse |= pinmask;
  sem_post(&priv->exclsem);
  return OK;
}

/****************************************************************************
 * Name: stmpe811_gpiowrite
 *
 * Description:
 *  Set or clear the GPIO output
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *   pinconfig - Bit-encoded pin configuration
 *   value     = true: write logic '1'; false: write logic '0;
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stmpe811_gpiowrite(STMPE811_HANDLE handle, uint8_t pinconfig, bool value)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  int pin = (pinconfig & STMPE811_GPIO_PIN_MASK) >> STMPE811_GPIO_PIN_SHIFT;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < STMPE811_GPIO_NPINS);

  /* Get exclusive access to the device structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      idbg("sem_wait failed: %d\n", errno);
      return;
    }

  /* Are we setting or clearing outputs? */

  if (value)
    {
      /* Set the output valu(s)e by writing to the SET register */

      stmpe811_putreg8(priv, STMPE811_GPIO_SETPIN, (1 << pin));
    }
  else
    {
      /* Clear the output value(s) by writing to the CLR register */

      stmpe811_putreg8(priv, STMPE811_GPIO_CLRPIN, (1 << pin));
    }

  sem_post(&priv->exclsem);
}

/****************************************************************************
 * Name: stmpe811_gpioread
 *
 * Description:
 *  Set or clear the GPIO output
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *   pinconfig - Bit-encoded pin configuration
 *   value     - The location to return the state of the GPIO pin
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stmpe811_gpioread(STMPE811_HANDLE handle, uint8_t pinconfig, bool *value)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  int pin = (pinconfig & STMPE811_GPIO_PIN_MASK) >> STMPE811_GPIO_PIN_SHIFT;
  uint8_t regval;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < STMPE811_GPIO_NPINS);

  /* Get exclusive access to the device structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      int errval = errno;
      idbg("sem_wait failed: %d\n", errval);
      return -errval;
    }

  regval  = stmpe811_getreg8(priv, STMPE811_GPIO_MPSTA);
  *value = ((regval & GPIO_PIN(pin)) != 0);
  sem_post(&priv->exclsem);
  return OK;
}

/***********************************************************************************
 * Name: stmpe811_gpioattach
 *
 * Description:
 *  Attach to a GPIO interrupt input pin and enable interrupts on the pin.  Using
 *  the value NULL for the handler address will disable interrupts from the pin and
 *  detach the handler.
 *
 *  NOTE:  Callbacks do not occur from an interrupt handler but rather from the
 *  context of the worker thread.
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *   pinconfig - Bit-encoded pin configuration
 *   handler   - The handler that will be called when the interrupt occurs.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is returned
 *   to indicate the nature of the failure.
 *
 ************************************************************************************/

#ifndef CONFIG_STMPE811_GPIOINT_DISABLE
int stmpe811_gpioattach(STMPE811_HANDLE handle, uint8_t pinconfig,
                       stmpe811_handler_t handler)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  int pin = (pinconfig & STMPE811_GPIO_PIN_MASK) >> STMPE811_GPIO_PIN_SHIFT;
  uint8_t regval;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < STMPE811_GPIO_NPINS);

  /* Get exclusive access to the device structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      int errval = errno;
      idbg("sem_wait failed: %d\n", errval);
      return -errval;
    }

  /* Make sure that the GPIO interrupt system has been gpioinitialized */

  stmpe811_gpioinit(priv);

  /* Set/clear the handler */

  priv->handlers[pin] = handler;

  /* If an handler has provided, then we are enabling interrupts */

  regval = stmpe811_getreg8(priv, STMPE811_GPIO_EN);
  if (handler)
    {
      /* Enable interrupts for this GPIO */

      regval &= ~GPIO_PIN(pin);
    }
  else
    {
      /* Disable interrupts for this GPIO */

      regval &= ~GPIO_PIN(pin);
    }
  stmpe811_putreg8(priv, STMPE811_GPIO_EN, regval);

  sem_post(&priv->exclsem);
  return OK;
}
#endif

/****************************************************************************
 * Name: stmpe811_gpioworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

#ifndef CONFIG_STMPE811_GPIOINT_DISABLE
void stmpe811_gpioworker(FAR struct stmpe811_dev_s *priv)
{
  uint8_t regval;
  uint8_t pinmask;
  int pin;

  /* Get the set of pending GPIO interrupts */

  regval = stmpe811_getreg8(priv, STMPE811_GPIO_INTSTA);

  /* Look at each pin */

  for (pin = 0; pin < STMPE811_GPIO_NPINS; pin++)
    {
      pinmask = GPIO_INT(pin);
      if ((regval & pinmask) != 0)
        {
          /* Check if we have a handler for this interrupt (there should
           * be one)
           */

          if (priv->handlers[pin])
            {
              /* Interrupt is pending... dispatch the interrupt to the
               * callback
               */

              priv->handlers[pin](pin);
            }
          else
            {
              illdbg("No handler for PIN%d, GPIO_INTSTA: %02x\n", pin, regval);
            }

          /* Clear the pending GPIO interrupt by writing a '1' to the
           * pin position in the status register.
           */

          stmpe811_putreg8(priv, STMPE811_GPIO_INTSTA, pinmask);
        }
    }
}
#endif

#endif /* CONFIG_INPUT && CONFIG_INPUT_STMPE811 && !CONFIG_STMPE811_GPIO_DISABLE */

