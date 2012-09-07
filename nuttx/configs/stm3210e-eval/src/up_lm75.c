/************************************************************************************
 * configs/stm3210e-eval/src/up_lm75.c
 * arch/arm/src/board/up_lm75.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>

#include <nuttx/i2c.h>
#include <nuttx/sensors/lm75.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "stm3210e-internal.h"

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_LM75) && defined(CONFIG_STM32_I2C1)

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_lm75initialize
 *
 * Description:
 *   Initialize and register the LM-75 Temperature Sensor driver.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

int stm32_lm75initialize(FAR const char *devpath)
{
  FAR struct i2c_dev_s *i2c;
  int ret;

  /* Configure PB.5 as Input pull-up.  This pin can be used as a temperature
   * sensor interrupt (not fully implemented).
   */

  stm32_configgpio(GPIO_LM75_OSINT);

  /* Get an instance of the I2C1 interface */

  i2c =  up_i2cinitialize(1);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the temperature sensor */

  ret = lm75_register(devpath, i2c, 0x48);
  if (ret < 0)
    {
      (void)up_i2cuninitialize(i2c);
    }
  return ret;
}

/************************************************************************************
 * Name: stm32_lm75attach
 *
 * Description:
 *   Attach the LM-75 interrupt handler
 *
 * Input parameters:
 *   irqhandler - the LM-75 interrupt handler
 *
 * Returned Value:
 *   The previous LM-75 interrupt handler
 *
 ************************************************************************************/

xcpt_t stm32_lm75attach(xcpt_t irqhandler)
{
  return stm32_gpiosetevent(GPIO_LM75_OSINT, true, true, true, irqhandler);
}

#endif /* CONFIG_I2C && CONFIG_I2C_LM75 && CONFIG_STM32_I2C1 */
