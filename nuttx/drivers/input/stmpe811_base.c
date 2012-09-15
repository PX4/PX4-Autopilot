/****************************************************************************
 * drivers/input/stmpe811_base.c
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

#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/input/stmpe811.h>

#include "stmpe811.h"

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_STMPE811)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* If only a single STMPE811 device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

#ifndef CONFIG_STMPE811_MULTIPLE
static struct stmpe811_dev_s g_stmpe811;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct stmpe811_dev_s *g_stmpe811list;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe811_worker
 *
 * Description:
 *   This is the "bottom half" of the STMPE811 interrupt handler
 *
 ****************************************************************************/

static void stmpe811_worker(FAR void *arg)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)arg;
  uint8_t regval;

  DEBUGASSERT(priv && priv->config);

  /* Get the global interrupt status */

  regval =  stmpe811_getreg8(priv, STMPE811_INT_STA);

  /* Check for a touchscreen interrupt */

#ifndef CONFIG_STMPE811_TSC_DISABLE
  if ((regval & (INT_TOUCH_DET|INT_FIFO_TH|INT_FIFO_OFLOW)) != 0)
    {
      /* Dispatch the touchscreen interrupt if it was brought into the link */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
      if (stmpe811_tscworker)
#endif
        {
           stmpe811_tscworker(priv, regval);
        }

      stmpe811_putreg8(priv, STMPE811_INT_STA, (INT_TOUCH_DET|INT_FIFO_TH|INT_FIFO_OFLOW));
      regval &= ~(INT_TOUCH_DET|INT_FIFO_TH|INT_FIFO_OFLOW);
    }
#endif

#if !defined(CONFIG_STMPE811_GPIO_DISABLE) && !defined(CONFIG_STMPE811_GPIOINT_DISABLE)
  if ((regval & INT_GPIO) != 0)
    {
      /* Dispatch the GPIO interrupt if it was brought into the link */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
      if (stmpe811_gpioworker)
#endif
        {
          stmpe811_gpioworker(priv);
        }

      stmpe811_putreg8(priv, STMPE811_INT_STA, INT_GPIO);
      regval &= ~INT_GPIO;
    }
#endif

  /* Clear any other residual, unhandled pending interrupt */

  if (regval != 0)
    {
      stmpe811_putreg8(priv, STMPE811_INT_STA, regval);
    }

  /* Re-enable the STMPE811 GPIO interrupt */

  priv->config->enable(priv->config, true);
}

/****************************************************************************
 * Name: stmpe811_interrupt
 *
 * Description:
 *  The STMPE811 interrupt handler
 *
 ****************************************************************************/

static int stmpe811_interrupt(int irq, FAR void *context)
{
  FAR struct stmpe811_dev_s    *priv;
  FAR struct stmpe811_config_s *config;
  int                          ret;

  /* Which STMPE811 device caused the interrupt? */

#ifndef CONFIG_STMPE811_MULTIPLE
  priv = &g_stmpe811;
#else
  for (priv = g_stmpe811list;
       priv && priv->config->irq != irq;
       priv = priv->flink);

  ASSERT(priv != NULL);
#endif

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Disable further interrupts */

  config->enable(config, false);

  /* Check if interrupt work is already queue.  If it is already busy, then
   * we already have interrupt processing in the pipeline and we need to do
   * nothing more.
   */

  if (work_available(&priv->work))
    {
      /* Yes.. Transfer processing to the worker thread.  Since STMPE811
       * interrupts are disabled while the work is pending, no special
       * action should be required to protect the work queue.
       */

      ret = work_queue(HPWORK, &priv->work, stmpe811_worker, priv, 0);
      if (ret != 0)
        {
          illdbg("Failed to queue work: %d\n", ret);
        }
    }

  /* Clear any pending interrupts and return success */

  config->clear(config);
  return OK;
}

/****************************************************************************
 * Name: stmpe811_checkid
 *
 * Description:
 *   Read and verify the STMPE811 chip ID
 *
 ****************************************************************************/

static int stmpe811_checkid(FAR struct stmpe811_dev_s *priv)
{
  uint16_t devid = 0;

  /* Read device ID  */

  devid = stmpe811_getreg8(priv, STMPE811_CHIP_ID);
  devid = (uint32_t)(devid << 8);
  devid |= (uint32_t)stmpe811_getreg8(priv, STMPE811_CHIP_ID+1);
  ivdbg("devid: %04x\n", devid);

  if (devid != (uint16_t)CHIP_ID)
    {
      /* ID is not Correct */

      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: stmpe811_reset
 *
 * Description:
 *  Reset the STMPE811
 *
 ****************************************************************************/

static void stmpe811_reset(FAR struct stmpe811_dev_s *priv)
{
  /* Power Down the STMPE811 */

  stmpe811_putreg8(priv, STMPE811_SYS_CTRL1, SYS_CTRL1_SOFTRESET);

  /* Wait a bit */

  usleep(20*1000);

  /* Then power on again.  All registers will be in their reset state. */

  stmpe811_putreg8(priv, STMPE811_SYS_CTRL1, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe811_instantiate
 *
 * Description:
 *   Instantiate and configure the STMPE811 device driver to use the provided
 *   I2C or SPIdevice instance.
 *
 * Input Parameters:
 *   dev     - An I2C or SPI driver instance
 *   config  - Persistant board configuration data
 *
 * Returned Value:
 *   A non-zero handle is returned on success.  This handle may then be used
 *   to configure the STMPE811 driver as necessary.  A NULL handle value is
 *   returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_STMPE811_SPI
STMPE811_HANDLE stmpe811_instantiate(FAR struct spi_dev_s *dev,
                                     FAR struct stmpe811_config_s *config)
#else
STMPE811_HANDLE stmpe811_instantiate(FAR struct i2c_dev_s *dev,
                                     FAR struct stmpe811_config_s *config)
#endif
{
  FAR struct stmpe811_dev_s *priv;
  uint8_t regval;
  int ret;

  /* Allocate the device state structure */

#ifdef CONFIG_STMPE811_MULTIPLE
  priv = (FAR struct stmpe811_dev_s *)kzalloc(sizeof(struct stmpe811_dev_s));
  if (!priv)
    {
      return NULL;
    }

  /* And save the device structure in the list of STMPE811 so that we can find it later */

  priv->flink   = g_stmpe811list;
  g_stmpe811list = priv;
#else

  /* Use the one-and-only STMPE811 driver instance */

  priv = &g_stmpe811;
#endif

  /* Initialize the device state structure */

  sem_init(&priv->exclsem, 0, 1);
  priv->config = config;

#ifdef CONFIG_STMPE811_SPI
  priv->spi = dev;
#else
  priv->i2c = dev;

  /* Set the I2C address and frequency.  REVISIT:  This logic would be
   * insufficient if we share the I2C bus with any other devices that also
   * modify the address and frequency.
   */

  I2C_SETADDRESS(dev, config->address, 7);
  I2C_SETFREQUENCY(dev, config->frequency);
#endif

  /* Read and verify the STMPE811 chip ID */

  ret = stmpe811_checkid(priv);
  if (ret < 0)
    {
#ifdef CONFIG_STMPE811_MULTIPLE
      kfree(priv);
#endif
      return NULL;
    }

  /* Generate STMPE811 Software reset */

  stmpe811_reset(priv);

  /* Configure the interrupt output pin to generate interrupts on high or low level. */

  regval  = stmpe811_getreg8(priv, STMPE811_INT_CTRL);
#ifdef CONFIG_STMPE811_ACTIVELOW
  regval &= ~INT_CTRL_INT_POLARITY; /* Pin polarity: Active low / falling edge */
#else
  regval |= INT_CTRL_INT_POLARITY;  /* Pin polarity: Active high / rising edge */
#endif
#ifdef CONFIG_STMPE811_EDGE
  regval |= INT_CTRL_INT_TYPE;      /* Edge interrupt */
#else
  regval &= ~INT_CTRL_INT_TYPE;     /* Level interrupt */
#endif
  stmpe811_putreg8(priv, STMPE811_INT_CTRL, regval);

  /* Attach the STMPE811 interrupt handler. */

  config->attach(config, stmpe811_interrupt);

  /* Clear any pending interrupts */

  stmpe811_putreg8(priv, STMPE811_INT_STA, INT_ALL);
  config->clear(config);
  config->enable(config, true);

  /* Enable global interrupts */

  regval  = stmpe811_getreg8(priv, STMPE811_INT_CTRL);
  regval |= INT_CTRL_GLOBAL_INT;
  stmpe811_putreg8(priv, STMPE811_INT_CTRL, regval);

  /* Return our private data structure as an opaque handle */

  return (STMPE811_HANDLE)priv;
}

/****************************************************************************
 * Name: stmpe811_getreg8
 *
 * Description:
 *   Read from an 8-bit STMPE811 register
 *
 ****************************************************************************/

#ifdef CONFIG_STMPE811_I2C
uint8_t stmpe811_getreg8(FAR struct stmpe811_dev_s *priv, uint8_t regaddr)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - STMPE811_Reg_Address - 
   *    Repeated_Start - I2C_Read_Address  - STMPE811_Read_Data - STOP
   */

  struct i2c_msg_s msg[2];
  uint8_t regval;
  int ret;

  /* Setup 8-bit STMPE811 address write message */

  msg[0].addr   = priv->config->address; /* 7-bit address */
  msg[0].flags  = 0;                     /* Write transaction, beginning with START */
  msg[0].buffer = &regaddr;              /* Transfer from this address */
  msg[0].length = 1;                     /* Send one byte following the address
                                          * (no STOP) */

  /* Set up the 8-bit STMPE811 data read message */
  
  msg[1].addr   = priv->config->address; /* 7-bit address */
  msg[1].flags  = I2C_M_READ;            /* Read transaction, beginning with Re-START */
  msg[1].buffer = &regval;               /* Transfer to this address */
  msg[1].length = 1;                     /* Receive one byte following the address
                                          * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      idbg("I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

#ifdef CONFIG_STMPE811_REGDEBUG
  dbg("%02x->%02x\n", regaddr, regval);
#endif
  return regval;
}
#endif

/****************************************************************************
 * Name: stmpe811_putreg8
 *
 * Description:
 *   Write a value to an 8-bit STMPE811 register
 *
 ****************************************************************************/

#ifdef CONFIG_STMPE811_I2C
void stmpe811_putreg8(FAR struct stmpe811_dev_s *priv,
                      uint8_t regaddr, uint8_t regval)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - STMPE811_Reg_Address - STMPE811_Write_Data - STOP
   */

  struct i2c_msg_s msg;
  uint8_t txbuffer[2];
  int ret;

#ifdef CONFIG_STMPE811_REGDEBUG
  dbg("%02x<-%02x\n", regaddr, regval);
#endif

  /* Setup to the data to be transferred.  Two bytes:  The STMPE811 register
   * address followed by one byte of data.
   */

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  /* Setup 8-bit STMPE811 address write message */

  msg.addr   = priv->config->address; /* 7-bit address */
  msg.flags  = 0;                     /* Write transaction, beginning with START */
  msg.buffer = txbuffer;              /* Transfer from this address */
  msg.length = 2;                     /* Send two byte following the address
                                       * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      idbg("I2C_TRANSFER failed: %d\n", ret);
    }
}
#endif

/****************************************************************************
 * Name: stmpe811_getreg16
 *
 * Description:
 *   Read 16-bits of data from an STMPE-11 register
 *
 ****************************************************************************/

#ifdef CONFIG_STMPE811_I2C
uint16_t stmpe811_getreg16(FAR struct stmpe811_dev_s *priv, uint8_t regaddr)
{
  /* 16-bit data read sequence:
   *
   *  Start - I2C_Write_Address - STMPE811_Reg_Address - 
   *    Repeated_Start - I2C_Read_Address  - STMPE811_Read_Data_1 -
   *      STMPE811_Read_Data_2 - STOP
   */


  struct i2c_msg_s msg[2];
  uint8_t rxbuffer[2];
  int ret;

  /* Setup 8-bit STMPE811 address write message */

  msg[0].addr   = priv->config->address; /* 7-bit address */
  msg[0].flags  = 0;                     /* Write transaction, beginning with START */
  msg[0].buffer = &regaddr;              /* Transfer from this address */
  msg[0].length = 1;                     /* Send one byte following the address
                                          * (no STOP) */

  /* Set up the 8-bit STMPE811 data read message */
  
  msg[1].addr   = priv->config->address; /* 7-bit address */
  msg[1].flags  = I2C_M_READ;            /* Read transaction, beginning with Re-START */
  msg[1].buffer = rxbuffer;              /* Transfer to this address */
  msg[1].length = 2;                     /* Receive two bytes following the address
                                          * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      idbg("I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

#ifdef CONFIG_STMPE811_REGDEBUG
  dbg("%02x->%02x%02x\n", regaddr, rxbuffer[0], rxbuffer[1]);
#endif
  return (uint16_t)rxbuffer[0] << 8 | (uint16_t)rxbuffer[1];
}
#endif

#endif /* CONFIG_INPUT && CONFIG_INPUT_STMPE811 */

