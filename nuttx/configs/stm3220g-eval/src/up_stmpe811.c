/************************************************************************************
 * configs/stm3220g-eval/src/up_touchscreen.c
 * arch/arm/src/board/up_touchscreen.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/i2c.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/stmpe811.h>

#include <arch/irq.h>

#include "stm32_internal.h"
#include "stm3220g-internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifdef CONFIG_INPUT_STMPE811
#ifndef CONFIG_INPUT
#  error "STMPE811 support requires CONFIG_INPUT"
#endif

#ifndef CONFIG_STM32_I2C1
#  error "STMPE811 support requires CONFIG_STM32_I2C1"
#endif

#ifndef CONFIG_STMPE811_I2C
#  error "Only the STMPE811 I2C interface is supported"
#endif

#ifdef CONFIG_STMPE811_SPI
#  error "Only the STMPE811 SPI interface is supported"
#endif

#ifndef CONFIG_STMPE811_FREQUENCY
#  define CONFIG_STMPE811_FREQUENCY 100000
#endif

#ifndef CONFIG_STMPE811_I2CDEV
#  define CONFIG_STMPE811_I2CDEV 1
#endif

#if CONFIG_STMPE811_I2CDEV != 1
#  error "CONFIG_STMPE811_I2CDEV must be one"
#endif

#ifndef CONFIG_STMPE811_DEVMINOR
#  define CONFIG_STMPE811_DEVMINOR 0
#endif

/* Board definitions ********************************************************/
/* The STM3220G-EVAL has two STMPE811QTR I/O expanders on board both connected
 * to the STM32 via I2C1.  They share a common interrupt line: PI2.
 * 
 * STMPE811 U24, I2C address 0x41 (7-bit)
 * ------ ---- ---------------- --------------------------------------------
 * STPE11 PIN  BOARD SIGNAL     BOARD CONNECTION
 * ------ ---- ---------------- --------------------------------------------
 *   Y-        TouchScreen_Y-   LCD Connector XL
 *   X-        TouchScreen_X-   LCD Connector XR
 *   Y+        TouchScreen_Y+   LCD Connector XD
 *   X+        TouchScreen_X+   LCD Connector XU
 *   IN3       EXP_IO9
 *   IN2       EXP_IO10
 *   IN1       EXP_IO11
 *   IN0       EXP_IO12
 * 
 * STMPE811 U29, I2C address 0x44 (7-bit)
 * ------ ---- ---------------- --------------------------------------------
 * STPE11 PIN  BOARD SIGNAL     BOARD CONNECTION
 * ------ ---- ---------------- --------------------------------------------
 *   Y-        EXP_IO1
 *   X-        EXP_IO2
 *   Y+        EXP_IO3
 *   X+        EXP_IO4
 *   IN3       EXP_IO5
 *   IN2       EXP_IO6
 *   IN1       EXP_IO7
 *   IN0       EXP_IO8
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_stmpe811config_s
{
  /* Configuration structure as seen by the STMPE811 driver */

  struct stmpe811_config_s config;

  /* Additional private definitions only known to this driver */

  STMPE811_HANDLE handle;   /* The STMPE811 driver handle */
  xcpt_t         handler;  /* The STMPE811 interrupt handler */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the STMPE811 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.* so that contact and loss-of-contact events can be detected.
 *
 * attach  - Attach the STMPE811 interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 */

static int  stmpe811_attach(FAR struct stmpe811_config_s *state, xcpt_t isr);
static void stmpe811_enable(FAR struct stmpe811_config_s *state, bool enable);
static void stmpe811_clear(FAR struct stmpe811_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the STMPE811
 * driver.  This structure provides information about the configuration
 * of the STMPE811 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

#ifndef CONFIG_STMPE811_TSC_DISABLE
static struct stm32_stmpe811config_s g_stmpe811config =
{
  .config =
  {
#ifdef CONFIG_STMPE811_I2C
    .address   = STMPE811_ADDR1,
#endif
    .frequency = CONFIG_STMPE811_FREQUENCY,

#ifdef CONFIG_STMPE811_MULTIPLE
    .irq       = STM32_IRQ_EXTI2,
#endif
    .ctrl1     = (ADC_CTRL1_SAMPLE_TIME_80 | ADC_CTRL1_MOD_12B),
    .ctrl2     = ADC_CTRL2_ADC_FREQ_3p25,

    .attach    = stmpe811_attach,
    .enable    = stmpe811_enable,
    .clear     = stmpe811_clear,
  },
  .handler     = NULL,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the STMPE811 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.
 *
 * attach  - Attach the STMPE811 interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 */

static int stmpe811_attach(FAR struct stmpe811_config_s *state, xcpt_t isr)
{
  FAR struct stm32_stmpe811config_s *priv = (FAR struct stm32_stmpe811config_s *)state;

  ivdbg("Saving handler %p\n", isr);
  DEBUGASSERT(priv);
  
  /* Just save the handler.  We will use it when EXTI interruptsare enabled */

  priv->handler = isr;
  return OK;
}

static void stmpe811_enable(FAR struct stmpe811_config_s *state, bool enable)
{
  FAR struct stm32_stmpe811config_s *priv = (FAR struct stm32_stmpe811config_s *)state;
  irqstate_t flags;

  /* Attach and enable, or detach and disable.  Enabling and disabling GPIO
   * interrupts is a multi-step process so the safest thing is to keep
   * interrupts disabled during the reconfiguratino.
   */

  flags = irqsave();
  if (enable)
    {
      /* Configure the EXTI interrupt using the SAVED handler */

      (void)stm32_gpiosetevent(GPIO_IO_EXPANDER, true, true, true, priv->handler);
    }
  else
    {
      /* Configure the EXTI interrupt with a NULL handler to disable it */

     (void)stm32_gpiosetevent(GPIO_IO_EXPANDER, false, false, false, NULL);
    }
  irqrestore(flags);
}

static void stmpe811_clear(FAR struct stmpe811_config_s *state)
{
  /* Does nothing */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arch_tcinitialize
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   configure the touchscreen device.  This function will register the driver
 *   as /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int arch_tcinitialize(int minor)
{
#ifndef CONFIG_STMPE811_TSC_DISABLE
  FAR struct i2c_dev_s *dev;
  int ret;

  idbg("minor %d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Check if we are already initialized */

  if (!g_stmpe811config.handle)
    {
      ivdbg("Initializing\n");

      /* Configure the STMPE811 interrupt pin as an input */

      (void)stm32_configgpio(GPIO_IO_EXPANDER);

      /* Get an instance of the I2C interface */

      dev = up_i2cinitialize(CONFIG_STMPE811_I2CDEV);
      if (!dev)
        {
          idbg("Failed to initialize I2C bus %d\n", CONFIG_STMPE811_I2CDEV);
          return -ENODEV;
        }

      /* Instantiate the STMPE811 driver */

      g_stmpe811config.handle =
        stmpe811_instantiate(dev, (FAR struct stmpe811_config_s *)&g_stmpe811config);
      if (!g_stmpe811config.handle)
        {
          idbg("Failed to instantiate the STMPE811 driver\n");
          return -ENODEV;
        }

      /* Initialize and register the I2C touchscreen device */

      ret = stmpe811_register(g_stmpe811config.handle, CONFIG_STMPE811_DEVMINOR);
      if (ret < 0)
        {
          idbg("Failed to register STMPE driver: %d\n", ret);
          /* up_i2cuninitialize(dev); */
          return -ENODEV;
        }
    }

  return OK;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: arch_tcuninitialize
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   uninitialize the touchscreen device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void arch_tcuninitialize(void)
{
  /* No support for un-initializing the touchscreen STMPE811 device yet */
}

#endif /* CONFIG_INPUT_STMPE811 */

