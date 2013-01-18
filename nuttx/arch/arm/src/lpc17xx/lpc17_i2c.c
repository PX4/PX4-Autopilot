/*******************************************************************************
 * arch/arm/src/lpc17xx/lpc17_i2c.c
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *   History: 0.1 2011-08-20 initial version
 * 
 * Derived from arch/arm/src/lpc31xx/lpc31_i2c.c
 *
 *   Author: David Hewson
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <wdog.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"

#include "chip.h"
#include "chip/lpc17_syscon.h"
#include "lpc17_gpio.h"
#include "lpc17_i2c.h"

#if defined(CONFIG_LPC17_I2C0) || defined(CONFIG_LPC17_I2C1) || defined(CONFIG_LPC17_I2C2)

#ifndef GPIO_I2C1_SCL
 #define GPIO_I2C1_SCL GPIO_I2C1_SCL_1
 #define GPIO_I2C1_SDA GPIO_I2C1_SDA_1
#endif

#ifndef CONFIG_I2C0_FREQ
 #define CONFIG_I2C0_FREQ 100000
#endif

#ifndef CONFIG_I2C1_FREQ
 #define CONFIG_I2C1_FREQ 100000
#endif

#ifndef CONFIG_I2C2_FREQ
 #define CONFIG_I2C2_FREQ 100000
#endif

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************/

#define I2C_TIMEOUT     ((20 * CLK_TCK) / 1000) /* 20 mS */

/*******************************************************************************
 * Private Data
 *******************************************************************************/

struct lpc17_i2cdev_s
{
  struct i2c_dev_s    dev;        /* Generic I2C device */
  struct i2c_msg_s    msg;        /* a single message for legacy read/write */
  unsigned int        base;       /* Base address of registers */
  uint16_t            irqid;      /* IRQ for this device */

  sem_t               mutex;      /* Only one thread can access at a time */
  sem_t               wait;       /* Place to wait for state machine completion */
  volatile uint8_t    state;      /* State of state machine */
  WDOG_ID             timeout;    /* watchdog to timeout when bus hung */

  uint16_t            wrcnt;      /* number of bytes sent to tx fifo */
  uint16_t            rdcnt;      /* number of bytes read from rx fifo */
};

static struct lpc17_i2cdev_s i2cdevices[3];

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

static int i2c_start(struct lpc17_i2cdev_s *priv);
static void i2c_stop(struct lpc17_i2cdev_s *priv);
static int  i2c_interrupt(int irq, FAR void *context);
static void i2c_timeout(int argc, uint32_t arg, ...);

/*******************************************************************************
 * I2C device operations
 *******************************************************************************/

static uint32_t i2c_setfrequency(FAR struct i2c_dev_s *dev, uint32_t frequency);
static int      i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits);
static int      i2c_write(FAR struct i2c_dev_s *dev, const uint8_t *buffer, int buflen);
static int      i2c_read(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen);
static int      i2c_transfer(FAR struct i2c_dev_s *dev, FAR struct i2c_msg_s *msgs, int count);

struct i2c_ops_s lpc17_i2c_ops =
{
  .setfrequency = i2c_setfrequency,
  .setaddress   = i2c_setaddress,
  .write        = i2c_write,
  .read         = i2c_read,
#ifdef CONFIG_I2C_TRANSFER
  .transfer     = i2c_transfer
#endif
};

/*******************************************************************************
 * Name: lpc17_i2c_setfrequency
 *
 * Description:
 *   Set the frequence for the next transfer
 *
 *******************************************************************************/

static uint32_t i2c_setfrequency(FAR struct i2c_dev_s *dev, uint32_t frequency)
{
  struct lpc17_i2cdev_s *priv = (struct lpc17_i2cdev_s *) dev;

  if (frequency > 100000)
    {
      /* asymetric per 400Khz I2C spec */

      putreg32(LPC17_CCLK / (83 + 47) * 47 / frequency, priv->base + LPC17_I2C_SCLH_OFFSET);
      putreg32(LPC17_CCLK / (83 + 47) * 83 / frequency, priv->base + LPC17_I2C_SCLL_OFFSET);
    }
  else
    {
      /* 50/50 mark space ratio */

      putreg32(LPC17_CCLK / 100 * 50 / frequency, priv->base + LPC17_I2C_SCLH_OFFSET);
      putreg32(LPC17_CCLK / 100 * 50 / frequency, priv->base + LPC17_I2C_SCLL_OFFSET);
    }

  /* FIXME: This function should return the actual selected frequency */

  return frequency;
}

/*******************************************************************************
 * Name: lpc17_i2c_setaddress
 *
 * Description:
 *   Set the I2C slave address for a subsequent read/write
 *
 *******************************************************************************/

static int i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits)
{
  struct lpc17_i2cdev_s *priv = (struct lpc17_i2cdev_s *) dev;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(nbits == 7 );

  priv->msg.addr  = addr<<1;
  priv->msg.flags = 0 ;

  return OK;
}

/*******************************************************************************
 * Name: lpc17_i2c_write
 *
 * Description:
 *   Send a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 *******************************************************************************/

static int i2c_write(FAR struct i2c_dev_s *dev, const uint8_t *buffer, int buflen)
{
  struct lpc17_i2cdev_s *priv = (struct lpc17_i2cdev_s *) dev;
  int ret;

  DEBUGASSERT(dev != NULL);

  priv->wrcnt = 0;
  priv->rdcnt = 0;
  priv->msg.addr &= ~0x01;
  priv->msg.buffer = (uint8_t*)buffer;
  priv->msg.length = buflen;
  
  ret = i2c_start(priv);

  return ret > 0 ? OK : -ETIMEDOUT;
}

/*******************************************************************************
 * Name: lpc17_i2c_read
 *
 * Description:
 *   Receive a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 *******************************************************************************/

static int i2c_read(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen)
{
  struct lpc17_i2cdev_s *priv = (struct lpc17_i2cdev_s *) dev;
  int ret;

  DEBUGASSERT(dev != NULL);

  priv->wrcnt = 0;
  priv->rdcnt = 0;
  priv->msg.addr |= 0x01;
  priv->msg.buffer = buffer;
  priv->msg.length = buflen;

  ret = i2c_start(priv);

  return ret > 0 ? OK : -ETIMEDOUT;
}

/*******************************************************************************
 * Name: i2c_start
 *
 * Description:
 *   Perform a I2C transfer start
 *
 *******************************************************************************/

static int i2c_start(struct lpc17_i2cdev_s *priv)
{
  int ret = -1;

  sem_wait(&priv->mutex);

  putreg32(I2C_CONCLR_STAC|I2C_CONCLR_SIC, priv->base+LPC17_I2C_CONCLR_OFFSET);
  putreg32(I2C_CONSET_STA, priv->base+LPC17_I2C_CONSET_OFFSET);

  wd_start(priv->timeout, I2C_TIMEOUT, i2c_timeout, 1, (uint32_t)priv);
  sem_wait(&priv->wait);
  wd_cancel(priv->timeout);
  sem_post(&priv->mutex);

  if (priv-> state == 0x18 || priv->state == 0x28)
    {
      ret = priv->wrcnt;
    }
  else if (priv-> state == 0x50 || priv->state == 0x58)
    {
      ret = priv->rdcnt;
    }

  return ret;
} 

/*******************************************************************************
 * Name: i2c_stop
 *
 * Description:
 *   Perform a I2C transfer stop
 *
 *******************************************************************************/

static void i2c_stop(struct lpc17_i2cdev_s *priv)
{
  if (priv->state != 0x38)
    {
      putreg32(I2C_CONSET_STO|I2C_CONSET_AA,priv->base+LPC17_I2C_CONSET_OFFSET);
    }

  sem_post(&priv->wait);
}

/*******************************************************************************
 * Name: i2c_timeout
 *
 * Description:
 *   Watchdog timer for timeout of I2C operation
 *
 *******************************************************************************/

static void i2c_timeout(int argc, uint32_t arg, ...)
{
  struct lpc17_i2cdev_s *priv = (struct lpc17_i2cdev_s *) arg;

  irqstate_t flags = irqsave();
  priv->state = 0xff;
  sem_post(&priv->wait);
  irqrestore(flags);
}

/*******************************************************************************
 * Name: i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 *******************************************************************************/

static int i2c_interrupt(int irq, FAR void *context)
{
  struct lpc17_i2cdev_s *priv;
  uint32_t state;

#ifdef CONFIG_LPC17_I2C0
  if (irq == LPC17_IRQ_I2C0)
    {
      priv=&i2cdevices[0];
    }
  else
#endif
#ifdef CONFIG_LPC17_I2C1
  if (irq == LPC17_IRQ_I2C1)
    {
      priv=&i2cdevices[1];
    }
  else
#endif
#ifdef CONFIG_LPC17_I2C2
  if (irq == LPC17_IRQ_I2C2)
    {
      priv=&i2cdevices[2];
    }
  else
#endif
    {
      PANIC(OSERR_INTERNAL);
    }

/* Reference UM10360 19.10.5 */    

  state = getreg32(priv->base+LPC17_I2C_STAT_OFFSET);
  putreg32(I2C_CONCLR_SIC, priv->base+LPC17_I2C_CONCLR_OFFSET);
  priv->state = state;
  state &= 0xf8;

  switch (state)
    {
    case 0x00:      // Bus Error
    case 0x20:     
    case 0x30:
    case 0x38:
    case 0x48:
      i2c_stop(priv);
      break;

    case 0x08:     // START 
    case 0x10:     // Repeat START 
      putreg32(priv->msg.addr, priv->base+LPC17_I2C_DAT_OFFSET);
      putreg32(I2C_CONCLR_STAC, priv->base+LPC17_I2C_CONCLR_OFFSET);
      break;

    case 0x18:
      priv->wrcnt = 0;
      putreg32(priv->msg.buffer[0], priv->base+LPC17_I2C_DAT_OFFSET);
      break; 

    case 0x28:
      priv->wrcnt++;
      if (priv->wrcnt<priv->msg.length)
        {
          putreg32(priv->msg.buffer[priv->wrcnt],priv->base+LPC17_I2C_DAT_OFFSET);
        }
      else
        {
          i2c_stop(priv);
        }
      break;

    case 0x40:
      priv->rdcnt = -1;
      putreg32(I2C_CONSET_AA, priv->base+LPC17_I2C_CONSET_OFFSET);
      break;

    case 0x50:
      priv->rdcnt++;
      if (priv->rdcnt < priv->msg.length)
        {
          priv->msg.buffer[priv->rdcnt] = getreg32(priv->base+LPC17_I2C_BUFR_OFFSET);
        }

      if (priv->rdcnt>=priv->msg.length-1)
        {
          putreg32(I2C_CONCLR_AAC|I2C_CONCLR_SIC, priv->base+LPC17_I2C_CONCLR_OFFSET);
        }
      break;

    case 0x58:
      i2c_stop(priv);
      break;

    default:
      i2c_stop(priv);
      break;
    }

  return OK;
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: up_i2cinitialize
 *
 * Description:
 *   Initialise an I2C device
 *
 *******************************************************************************/

struct i2c_dev_s *up_i2cinitialize(int port)
{
  struct lpc17_i2cdev_s *priv;
  irqstate_t flags;
  uint32_t regval;

  if (port > 2)
    {
      dbg("lpc I2C Only support 0,1,2\n");
      return NULL;
    }

  flags = irqsave();

  priv= &i2cdevices[port];
#ifdef CONFIG_LPC17_I2C0
  if (port == 0)
    {
      priv= (FAR struct lpc17_i2cdev_s *)&i2cdevices[0];
      priv->base  = LPC17_I2C0_BASE;
      priv->irqid = LPC17_IRQ_I2C0;

      regval  = getreg32(LPC17_SYSCON_PCONP);
      regval |= SYSCON_PCONP_PCI2C0;
      putreg32(regval, LPC17_SYSCON_PCONP);

      regval  = getreg32(LPC17_SYSCON_PCLKSEL0);
      regval &= ~SYSCON_PCLKSEL0_I2C0_MASK;
      regval |= (SYSCON_PCLKSEL_CCLK << SYSCON_PCLKSEL0_I2C0_SHIFT);
      putreg32(regval, LPC17_SYSCON_PCLKSEL0);
        
      lpc17_configgpio(GPIO_I2C0_SCL);
      lpc17_configgpio(GPIO_I2C0_SDA);
        
      putreg32(LPC17_CCLK/CONFIG_I2C0_FREQ/2, priv->base + LPC17_I2C_SCLH_OFFSET);
      putreg32(LPC17_CCLK/CONFIG_I2C0_FREQ/2, priv->base + LPC17_I2C_SCLL_OFFSET);
    }
  else
#endif
#ifdef CONFIG_LPC17_I2C1
  if (port == 1)
    {
      priv= (FAR struct lpc17_i2cdev_s *)&i2cdevices[1];
      priv->base  = LPC17_I2C1_BASE;
      priv->irqid = LPC17_IRQ_I2C1;

      regval  = getreg32(LPC17_SYSCON_PCONP);
      regval |= SYSCON_PCONP_PCI2C1;
      putreg32(regval, LPC17_SYSCON_PCONP);

      regval  = getreg32(LPC17_SYSCON_PCLKSEL1);
      regval &= ~SYSCON_PCLKSEL1_I2C1_MASK;
      regval |= (SYSCON_PCLKSEL_CCLK << SYSCON_PCLKSEL1_I2C1_SHIFT);
      putreg32(regval, LPC17_SYSCON_PCLKSEL1);

      lpc17_configgpio(GPIO_I2C1_SCL);
      lpc17_configgpio(GPIO_I2C1_SDA);

      putreg32(LPC17_CCLK/CONFIG_I2C1_FREQ/2, priv->base + LPC17_I2C_SCLH_OFFSET);
      putreg32(LPC17_CCLK/CONFIG_I2C1_FREQ/2, priv->base + LPC17_I2C_SCLL_OFFSET);
    }
  else
#endif
#ifdef CONFIG_LPC17_I2C2
  if (port == 2)
    {
      priv= (FAR struct lpc17_i2cdev_s *)&i2cdevices[2];
      priv->base  = LPC17_I2C2_BASE;
      priv->irqid = LPC17_IRQ_I2C2;

      regval  = getreg32(LPC17_SYSCON_PCONP);
      regval |= SYSCON_PCONP_PCI2C2;
      putreg32(regval, LPC17_SYSCON_PCONP);

      regval  = getreg32(LPC17_SYSCON_PCLKSEL1);
      regval &= ~SYSCON_PCLKSEL1_I2C2_MASK;
      regval |= (SYSCON_PCLKSEL_CCLK << SYSCON_PCLKSEL1_I2C2_SHIFT);
      putreg32(regval, LPC17_SYSCON_PCLKSEL1);
        
      lpc17_configgpio(GPIO_I2C2_SCL);
      lpc17_configgpio(GPIO_I2C2_SDA);
        
      putreg32(LPC17_CCLK/CONFIG_I2C2_FREQ/2, priv->base + LPC17_I2C_SCLH_OFFSET);
      putreg32(LPC17_CCLK/CONFIG_I2C2_FREQ/2, priv->base + LPC17_I2C_SCLL_OFFSET);
    }
  else
#endif
    {
      return NULL;
    }

  putreg32(I2C_CONSET_I2EN, priv->base+LPC17_I2C_CONSET_OFFSET);

  sem_init(&priv->mutex, 0, 1);
  sem_init(&priv->wait, 0, 0);

  /* Allocate a watchdog timer */

  priv->timeout = wd_create();
  DEBUGASSERT(priv->timeout != 0);

  /* Attach Interrupt Handler */

  irq_attach(priv->irqid, i2c_interrupt);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irqid);

  /* Install our operations */

  priv->dev.ops = &lpc17_i2c_ops;
  return &priv->dev;
}

/*******************************************************************************
 * Name: up_i2cuninitalize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 *******************************************************************************/

int up_i2cuninitialize(FAR struct i2c_dev_s * dev)
{
  struct lpc17_i2cdev_s *priv = (struct lpc17_i2cdev_s *) dev;
  
  /* Disable I2C */

  putreg32(I2C_CONCLRT_I2ENC, priv->base+LPC17_I2C_CONCLR_OFFSET);

  /* Reset data structures */

  sem_destroy(&priv->mutex);
  sem_destroy(&priv->wait);

  /* Free the watchdog timer */

  wd_delete(priv->timeout);
  priv->timeout = NULL;

  /* Disable interrupts */

  up_disable_irq(priv->irqid);

  /* Detach Interrupt Handler */

  irq_detach(priv->irqid);
  return OK;
}

#endif
