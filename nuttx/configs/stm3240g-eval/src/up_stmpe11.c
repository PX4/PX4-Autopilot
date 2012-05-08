/************************************************************************************
 * configs/stm3240g-eval/src/up_touchscreen.c
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
#include <nuttx/input/stmpe11.h>

#include "stm32_internal.h"
#include "stm3240g-internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifdef CONFIG_INPUT_STMPE11
#ifndef CONFIG_INPUT
#  error "STMPE11 support requires CONFIG_INPUT"
#endif

#ifndef CONFIG_STM32_I2C1
#  error "STMPE11 support requires CONFIG_STM32_I2C1"
#endif

#ifndef CONFIG_STMPE11_I2C
#  error "Only the STMPE11 I2C interface is supported"
#endif

#ifdef CONFIG_STMPE11_SPI
#  error "Only the STMPE11 SPI interface is supported"
#endif

#ifndef CONFIG_STMPE11_FREQUENCY
#  define CONFIG_STMPE11_FREQUENCY 100000
#endif

#ifndef CONFIG_STMPE11_I2CDEV
#  define CONFIG_STMPE11_I2CDEV 1
#endif

#if CONFIG_STMPE11_I2CDEV != 1
#  error "CONFIG_STMPE11_I2CDEV must be one"
#endif

#ifndef CONFIG_STMPE11_DEVMINOR
#  define CONFIG_STMPE11_DEVMINOR 0
#endif

/* Board definitions ********************************************************/
/* The STM3240G-EVAL has two STMPE11QTR I/O expanders on board both connected
 * to the STM32 via I2C1.  They share a common interrupt line: PI2.
 * 
 * STMPE11 U24, I2C address 0x41 (7-bit)
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
 * STMPE11 U29, I2C address 0x44 (7-bit)
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

struct stm32_stmpe11config_s
{
  /* Configuration structure as seen by the STMPE11 driver */

  struct stmpe11_config_s config;

  /* Additional private definitions only known to this driver */

  STMPE11_HANDLE handle;   /* The STMPE11 driver handle */
  xcpt_t         handler;  /* The STMPE11 interrupt handler */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the STMPE11 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.* so that contact and loss-of-contact events can be detected.
 *
 * attach  - Attach the STMPE11 interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 */

static int  stmpe11_attach(FAR struct stmpe11_config_s *state, xcpt_t isr);
static void stmpe11_enable(FAR struct stmpe11_config_s *state, bool enable);
static void stmpe11_clear(FAR struct stmpe11_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the STMPE11
 * driver.  This structure provides information about the configuration
 * of the STMPE11 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

#ifndef CONFIG_STMPE11_TSC_DISABLE
static struct stm32_stmpe11config_s g_stmpe11config =
{
  .config =
  {
#ifdef CONFIG_STMPE11_I2C
    .address   = STMPE11_ADDR1,
#endif
    .frequency = CONFIG_STMPE11_FREQUENCY,

#ifdef CONFIG_STMPE11_MULTIPLE
    .irq       = STM32_IRQ_EXTI2,
#endif
    .ctrl1     = (ADC_CTRL1_SAMPLE_TIME_80|ADC_CTRL1_MOD_12B|ADC_CTRL1_REF_SEL),
    .ctrl2     = ADC_CTRL2_ADC_FREQ_3p25,

    .attach    = stmpe11_attach,
    .enable    = stmpe11_enable,
    .clear     = stmpe11_clear,
  },
  .handler     = NULL,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the STMPE11 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.
 *
 * attach  - Attach the STMPE11 interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 */

static int stmpe11_attach(FAR struct stmpe11_config_s *state, xcpt_t isr)
{
  FAR struct stm32_stmpe11config_s *priv = (FAR struct stm32_stmpe11config_s *)state;

  ivdbg("Saving handler %p\n", isr);
  DEBUGASSERT(priv);
  
  /* Just save the handler.  We will use it when EXTI interruptsare enabled */

  priv->handler = isr;
  return OK;
}

static void stmpe11_enable(FAR struct stmpe11_config_s *state, bool enable)
{
  FAR struct stm32_stmpe11config_s *priv = (FAR struct stm32_stmpe11config_s *)state;

  /* Attach and enable, or detach and disable */

  ivdbg("IRQ:%d enable:%d\n", STM32_IRQ_EXTI2, enable);
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
}

static void stmpe11_clear(FAR struct stmpe11_config_s *state)
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
 *   Initialize the touchscreen device
 *
 ****************************************************************************/

int arch_tcinitialize(void)
{
#ifndef CONFIG_STMPE11_TSC_DISABLE
  FAR struct i2c_dev_s *dev;
  int ret;

  ivdbg("Initializing\n");

  /* Configure and enable the STMPE11 interrupt pin as an input */

  (void)stm32_configgpio(GPIO_IO_EXPANDER);

  /* Get an instance of the I2C interface */

  dev = up_i2cinitialize(CONFIG_STMPE11_I2CDEV);
  if (!dev)
    {
      idbg("Failed to initialize I2C bus %d\n", CONFIG_STMPE11_I2CDEV);
      return -ENODEV;
    }

  /* Instantiate the STMPE11 driver */

  g_stmpe11config.handle =
    stmpe11_instantiate(dev, (FAR struct stmpe11_config_s *)&g_stmpe11config);
  if (!g_stmpe11config.handle)
    {
      idbg("Failed to instantiate the STMPE11 driver\n");
      return -ENODEV;
    }

  /* Initialize and register the I2C touschscreen device */

  ret = stmpe11_register(dev, CONFIG_STMPE11_DEVMINOR);
  if (ret < 0)
    {
      idbg("Failed to register STMPE driver: %d\n", ret);
      /* up_i2cuninitialize(dev); */
      return -ENODEV;
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
 *   Un-initialize the touchscreen device
 *
 ****************************************************************************/

void arch_tcuninitialize(void)
{
  /* No support for un-initializing the touchscreen STMPE11 device yet */
}

#endif /* CONFIG_INPUT_STMPE11 */

