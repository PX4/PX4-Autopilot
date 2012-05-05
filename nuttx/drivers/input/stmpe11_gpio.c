/****************************************************************************
 * drivers/input/stmpe11_gpio.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/input/stmpe11.h>

#include "stmpe11.h"

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_STMPE11) && !defined(CONFIG_STMPE11_GPIO_DISABLE)

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
 * Name: stmpe11_gpioinit
 *
 * Description:
 *  Initialize the GPIO interrupt subsystem
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe11_instantiate
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifndef CONFIG_STMPE11_GPIOINT_DISABLE
static inline void stmpe11_gpioinit(FAR struct stmpe11_dev_s *priv)
{
  uint8_t regval;

  if (!priv->initialized)
    {
      /* Disable all GPIO interrupts */

      stmpe11_putreg8(priv, STMPE11_GPIO_EN, 0);

      /* Enable global GPIO interrupts */

      regval = stmpe11_getreg8(priv, STMPE11_INT_EN);
      regval |= INT_GPIO;
      stmpe11_putreg8(priv, STMPE11_INT_EN, regval);

      priv->initialized = true;
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe11_gpioconfig
 *
 * Description:
 *  Configure an STMPE11 GPIO pin
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe11_instantiate
 *   pinconfig - Bit-encoded pin configuration
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stmpe11_gpioconfig(STMPE11_HANDLE handle, uint8_t pinconfig)
{
  FAR struct stmpe11_dev_s *priv = (FAR struct stmpe11_dev_s *)handle;
  int pin = (pinconfig & STMPE11_GPIO_PIN_MASK) >> STMPE11_GPIO_PIN_SHIFT;
  uint8_t pinmask = (1 << pin);
  uint8_t regval;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < 8);

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

  /* Is the pin an input or an output? */

  if ((pinconfig & STMPE11_GPIO_DIR) == STMPE11_GPIO_OUTPUT)
    {
      /* The pin is an output */

      regval  = stmpe11_getreg8(priv, STMPE11_GPIO_DIR);
      regval &= ~pinmask;
      stmpe11_putreg8(priv, STMPE11_GPIO_DIR, regval);

      /* Set its initial output value */

      stmpe11_gpiowrite(handle, pinconfig,
                        (pinconfig & STMPE11_GPIO_VALUE) != STMPE11_GPIO_ZERO);
    }
  else
    {
      /* It is an input */

      regval  = stmpe11_getreg8(priv, STMPE11_GPIO_DIR);
      regval |= pinmask;
      stmpe11_putreg8(priv, STMPE11_GPIO_DIR, regval);

      /* Set up the falling edge detection */

      regval = stmpe11_getreg8(priv, STMPE11_GPIO_FE);
      if ((pinconfig & STMPE11_GPIO_FALLING) != 0)
        {
          regval |= pinmask;
        }
      else
        {
          regval &= pinmask;
        }
      stmpe11_putreg8(priv, STMPE11_GPIO_FE, regval);

      /* Set up the rising edge detection */

     regval = stmpe11_getreg8(priv, STMPE11_GPIO_RE);
      if ((pinconfig & STMPE11_GPIO_FALLING) != 0)
        {
          regval |= pinmask;
        }
      else
        {
          regval &= pinmask;
        }
      stmpe11_putreg8(priv, STMPE11_GPIO_RE, regval);
 
      /* Disable interrupts for now */

      regval = stmpe11_getreg8(priv, STMPE11_GPIO_EN);
      regval &= ~pinmask;
      stmpe11_putreg8(priv, STMPE11_GPIO_EN, regval);
    }

  /* Mark the pin as 'in use' */

  priv->inuse |= pinmask;
  sem_post(&priv->exclsem);
  return OK;
}

/****************************************************************************
 * Name: stmpe11_gpiowrite
 *
 * Description:
 *  Set or clear the GPIO output
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe11_instantiate
 *   pinconfig - Bit-encoded pin configuration
 *   value     = true: write logic '1'; false: write logic '0;
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stmpe11_gpiowrite(STMPE11_HANDLE handle, uint8_t pinconfig, bool value)
{
  FAR struct stmpe11_dev_s *priv = (FAR struct stmpe11_dev_s *)handle;
  int pin = (pinconfig & STMPE11_GPIO_PIN_MASK) >> STMPE11_GPIO_PIN_SHIFT;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < 8);

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

      stmpe11_putreg8(priv, STMPE11_GPIO_SETPIN, (1 << pin));
    }
  else
    {
      /* Clear the output value(s) by writing to the CLR register */

      stmpe11_putreg8(priv, STMPE11_GPIO_CLRPIN, (1 << pin));
    }

  sem_post(&priv->exclsem);
}

/****************************************************************************
 * Name: stmpe11_gpioread
 *
 * Description:
 *  Set or clear the GPIO output
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe11_instantiate
 *   pinconfig - Bit-encoded pin configuration
 *   value     - The location to return the state of the GPIO pin
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stmpe11_gpioread(STMPE11_HANDLE handle, uint8_t pinconfig, bool *value)
{
  FAR struct stmpe11_dev_s *priv = (FAR struct stmpe11_dev_s *)handle;
  int pin = (pinconfig & STMPE11_GPIO_PIN_MASK) >> STMPE11_GPIO_PIN_SHIFT;
  uint8_t regval;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < 8);

  /* Get exclusive access to the device structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      int errval = errno;
      idbg("sem_wait failed: %d\n", errval);
      return -errval;
    }

  regval  = stmpe11_getreg8(priv, STMPE11_GPIO_MPSTA);
  *value = ((regval & GPIO_PIN(pin)) != 0);
  sem_post(&priv->exclsem);
  return OK;
}

/***********************************************************************************
 * Name: stmpe11_gpioattach
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
 *   handle    - The handle previously returned by stmpe11_instantiate
 *   pinconfig - Bit-encoded pin configuration
 *   handler   - The handler that will be called when the interrupt occurs.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is returned
 *   to indicate the nature of the failure.
 *
 ************************************************************************************/

#ifndef CONFIG_STMPE11_GPIOINT_DISABLE
int stmpe11_gpioattach(STMPE11_HANDLE handle, uint8_t pinconfig,
                       stmpe11_handler_t handler)
{
  FAR struct stmpe11_dev_s *priv = (FAR struct stmpe11_dev_s *)handle;
  int pin = (pinconfig & STMPE11_GPIO_PIN_MASK) >> STMPE11_GPIO_PIN_SHIFT;
  uint8_t regval;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < 8);

  /* Get exclusive access to the device structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      int errval = errno;
      idbg("sem_wait failed: %d\n", errval);
      return -errval;
    }

  /* Make sure that the GPIO interrupt system has been initialized */

  stmpe11_gpioinit(priv);

  /* Set/clear the handler */

  priv->handlers[pin] = handler;

  /* If an handler has provided, then we are enabling interrupts */

  regval = stmpe11_getreg8(priv, STMPE11_GPIO_EN);
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
  stmpe11_putreg8(priv, STMPE11_GPIO_EN, regval);

  sem_post(&priv->exclsem);
  return OK;
}
#endif

/****************************************************************************
 * Name: stmpe11_gpioint
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

#ifndef CONFIG_STMPE11_GPIOINT_DISABLE
void stmpe11_gpioint(FAR struct stmpe11_dev_s *priv)
{
  uint8_t regval;
  uint8_t pinmask;
  int pin;

  /* Get the set of pending GPIO interrupts */

  regval = stmpe11_getreg8(priv, STMPE11_GPIO_INTSTA);

  /* Look at each pin */

  for (pin = 0; pin < 8; pin++)
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

          stmpe11_putreg8(priv, STMPE11_GPIO_INTSTA, pinmask);
        }
    }
}
#endif

#endif /* CONFIG_INPUT && CONFIG_INPUT_STMPE11 */

