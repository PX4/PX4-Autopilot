/************************************************************************************
 * arch/arm/src/stm32/stm32_can.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/can.h>

#include "up_internal.h"
#include "up_arch.h"

#include "os_internal.h"

#include "chip.h"
#include "stm32_internal.h"
#include "stm32_can.h"

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Up to 2 CAN interfaces are supported */

#if STM32_NCAN < 2
#  undef CONFIG_STM32_CAN2
#endif

#if STM32_NCAN < 1
#  undef CONFIG_STM32_CAN1
#endif

#if defined(CONFIG_STM32_CAN1) || defined(CONFIG_STM32_CAN2)

/* CAN BAUD */

#if defined(CONFIG_STM32_CAN1) && !defined(CONFIG_CAN1_BAUD)
#  error "CONFIG_CAN1_BAUD is not defined"
#endif

#if defined(CONFIG_STM32_CAN2) && !defined(CONFIG_CAN2_BAUD)
#  error "CONFIG_CAN2_BAUD is not defined"
#endif

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing CAN */

#ifdef CONFIG_DEBUG_CAN
#  define candbg    dbg
#  define canvdbg   vdbg
#  define canlldbg  lldbg
#  define canllvdbg llvdbg
#else
#  define candbg(x...)
#  define canvdbg(x...)
#  define canlldbg(x...)
#  define canllvdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_can_s
{
  uint8_t  port;   /* CAN port number (1 or 2) */
  uint8_t  cantx;  /* CAN TX IRQ number */
  uint8_t  canrx0; /* CAN RX FIFO 0 IRQ number */
  uint8_t  canrx1; /* CAN RX FIFO 1 IRQ number */
  uint8_t  cansce; /* CAN RX0 Status change error (SCE) IRQ number */
  uint32_t baud;   /* Configured baud */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN driver methods */

static void can_reset(FAR struct can_dev_s *dev);
static int  can_setup(FAR struct can_dev_s *dev);
static void can_shutdown(FAR struct can_dev_s *dev);
static void can_rxint(FAR struct can_dev_s *dev, bool enable);
static void can_txint(FAR struct can_dev_s *dev, bool enable);
static int  can_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg);
static int  can_remoterequest(FAR struct can_dev_s *dev, uint16_t id);
static int  can_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg);
static bool can_txempty(FAR struct can_dev_s *dev);

/* CAN interrupt handling */

static int  can_txinterrupt(int irq, void *context);
static int  can_rx0interrupt(int irq, void *context);
static int  can_rx1interrupt(int irq, void *context);
static int  can_sceinterrupt(int irq, void *context);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_canops =
{
  .co_reset         = can_reset,
  .co_setup         = can_setup,
  .co_shutdown      = can_shutdown,
  .co_rxint         = can_rxint,
  .co_txint         = can_txint,
  .co_ioctl         = can_ioctl,
  .co_remoterequest = can_remoterequest,
  .co_send          = can_send,
  .co_txempty       = can_txempty,
};

#ifdef CONFIG_STM32_CAN1
static struct stm32_can_s g_can1priv =
{
  .port             = 1,
#if defined(CONFIG_STM32_STM32F10XX) && !defined(CONFIG_STM32_CONNECTIVITY_LINE)
  .cantx            = STM32_IRQ_USBHPCANTX,
  .canrx0           = STM32_IRQ_USBLPCANRX0,
#else
  .cantx            = STM32_IRQ_CAN1TX,
  .canrx0           = STM32_IRQ_CAN1RX0,
#endif
  .canrx1           = STM32_IRQ_CAN1RX1,
  .cansce           = STM32_IRQ_CAN1SCE,
  .baud             = CONFIG_CAN1_BAUD,
};

static struct can_dev_s g_can1dev =
{
  .cd_ops           = &g_canops,
  .cd_priv          = &g_can1priv,
};
#endif

#ifdef CONFIG_STM32_CAN2
static struct stm32_can_s g_can2priv =
{
  .port             = 2,
  .cantx            = STM32_IRQ_CAN2TX,
  .canrx0           = STM32_IRQ_CAN2RX0,
  .canrx1           = STM32_IRQ_CAN2RX1,
  .cansce           = STM32_IRQ_CAN2SCE,
  .baud             = CONFIG_CAN2_BAUD,
};

static struct can_dev_s g_can2dev =
{
  .cd_ops           = &g_canops,
  .cd_priv          = &g_can2priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_reset
 *
 * Description:
 *   Reset the CAN device.  Called early to initialize the hardware. This
 *   function is called, before can_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void can_reset(FAR struct can_dev_s *dev)
{
  FAR struct stm32_can_s *priv = dev->cd_priv;
  irqstate_t flags;

  /* Disable interrupts momentary to stop any ongoing CAN event processing */

  flags = irqsave();

#ifdef CONFIG_STM32_CAN1  
  if (priv->port == 1)
    {
#warning "Missing logic"
    }
  else
#endif
#ifdef CONFIG_STM32_CAN2 
  if (priv->port == 2)
    {
#warning "Missing logic"
    }
  else
#endif
    {
      candbg("Unsupport port %d\n", priv->port);
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: can_setup
 *
 * Description:
 *   Configure the CAN. This method is called the first time that the CAN
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching CAN interrupts.
 *   All CAN interrupts are disabled upon return.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_setup(FAR struct can_dev_s *dev)
{
  FAR struct stm32_can_s *priv = dev->cd_priv;
  int ret;

  /* Attach all CAN interrupts */

  ret = irq_attach(priv->cantx, can_txinterrupt);
  if (ret < 0)
    {
      candbg("Failed to attach CAN%d TX IRQ (%d)", priv->port, priv->cantx);
      return ret;
    }

  ret = irq_attach(priv->canrx0, can_rx0interrupt);
  if (ret < 0)
    {
      candbg("Failed to attach CAN%d RX0 IRQ (%d)", priv->port, priv->canrx0);
      return ret;
    }

  ret = irq_attach(priv->canrx1, can_rx1interrupt);
  if (ret < 0)
    {
      candbg("Failed to attach CAN%d RX1 IRQ (%d)", priv->port, priv->canrx1);
      return ret;
    }

  ret = irq_attach(priv->cansce, can_sceinterrupt);
  if (ret < 0)
    {
      candbg("Failed to attach CAN%d SCE IRQ (%d)", priv->port, priv->cansce);
      return ret;
    }

  /* Enable all interrupts at the NVIC.  Interrupts are still disabled in
   * the CAN module.  Since we coming out of reset here, there should be
   */

  up_enable_irq(priv->cantx);
  up_enable_irq(priv->canrx0);
  up_enable_irq(priv->canrx1);
  up_enable_irq(priv->cansce);
  return OK;
}

/****************************************************************************
 * Name: can_shutdown
 *
 * Description:
 *   Disable the CAN.  This method is called when the CAN device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_shutdown(FAR struct can_dev_s *dev)
{
  FAR struct stm32_can_s *priv = dev->cd_priv;

  /* Disable all interrupts */

  up_disable_irq(priv->cantx);
  up_disable_irq(priv->canrx0);
  up_disable_irq(priv->canrx1);
  up_disable_irq(priv->cansce);

  /* Detach all interrupts */

  irq_detach(priv->cantx);
  irq_detach(priv->canrx0);
  irq_detach(priv->canrx1);
  irq_detach(priv->cansce);

  /* And reset the hardware */

  can_reset(dev);
}

/****************************************************************************
 * Name: can_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_rxint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct stm32_can_s *priv = dev->cd_priv;

#ifdef CONFIG_STM32_CAN1  
  if (priv->port == 1)
    {
#warning "Missing logic"
    }
  else
#endif
#ifdef CONFIG_STM32_CAN2 
  if (priv->port == 2)
    {
#warning "Missing logic"
    }
  else
#endif
    {
      candbg("Unsupport port %d\n", priv->port);
    }
}

/****************************************************************************
 * Name: can_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_txint(FAR struct can_dev_s *dev, bool enable)
{
  FAR struct stm32_can_s *priv = dev->cd_priv;

#ifdef CONFIG_STM32_CAN1  
  if (priv->port == 1)
    {
#warning "Missing logic"
    }
  else
#endif
#ifdef CONFIG_STM32_CAN2 
  if (priv->port == 2)
    {
#warning "Missing logic"
    }
  else
#endif
    {
      candbg("Unsupport port %d\n", priv->port);
    }
}

/****************************************************************************
 * Name: can_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg)
{
  /* No CAN ioctls are supported */

  return -ENOTTY;
}

/****************************************************************************
 * Name: can_remoterequest
 *
 * Description:
 *   Send a remote request
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int  can_remoterequest(FAR struct can_dev_s *dev, uint16_t id)
{
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: can_send
 *
 * Description:
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int  can_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg)
{
  FAR struct stm32_can_s *priv = dev->cd_priv;

#ifdef CONFIG_STM32_CAN1  
  if (priv->port == 1)
    {
#warning "Missing logic"
    }
  else
#endif
#ifdef CONFIG_STM32_CAN2 
  if (priv->port == 2)
    {
#warning "Missing logic"
    }
  else
#endif
    {
      candbg("Unsupport port %d\n", priv->port);
      return -EINVAL;
    }

#warning "Missing logic"
  return OK;
}

/****************************************************************************
 * Name: can_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the CAN
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static bool can_txempty(FAR struct can_dev_s *dev)
{
  FAR struct stm32_can_s *priv = dev->cd_priv;

#ifdef CONFIG_STM32_CAN1  
  if (priv->port == 1)
    {
#warning "Missing logic"
    }
  else
#endif
#ifdef CONFIG_STM32_CAN2 
  if (priv->port == 2)
    {
#warning "Missing logic"
    }
  else
#endif
    {
      candbg("Unsupport port %d\n", priv->port);
      return true;
    }

#warning "Missing logic"
  return true;
}

/****************************************************************************
 * Name: can_txinterrupt
 *
 * Description:
 *   CAN TX interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_txinterrupt(int irq, void *context)
{
  FAR struct stm32_can_s *priv;

#if defined(CONFIG_STM32_CAN1) && defined(CONFIG_STM32_CAN2)
  if (g_can1priv.cantx == irq)
    {
      priv = &g_can1priv;
    }
  else if (g_can2priv.cantx == irq)
    {
      priv = &g_can2priv;
    }
  else
    {
      PANIC(OSERR_UNEXPECTEDISR);
    }
#elif defined(CONFIG_STM32_CAN1)
  priv = &g_can1priv;
#else /* defined(CONFIG_STM32_CAN2) */
  priv = &g_can2priv;
#endif

#warning "Missing logic"
    return OK;
}

/****************************************************************************
 * Name: can_rx0interrupt
 *
 * Description:
 *   CAN RX FIFO 0 interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_rx0interrupt(int irq, void *context)
{
  FAR struct stm32_can_s *priv;

#if defined(CONFIG_STM32_CAN1) && defined(CONFIG_STM32_CAN2)
  if (g_can1priv.canrx0 == irq)
    {
      priv = &g_can1priv;
    }
  else if (g_can2priv.canrx0 == irq)
    {
      priv = &g_can2priv;
    }
  else
    {
      PANIC(OSERR_UNEXPECTEDISR);
    }
#elif defined(CONFIG_STM32_CAN1)
  priv = &g_can1priv;
#else /* defined(CONFIG_STM32_CAN2) */
  priv = &g_can2priv;
#endif

#warning "Missing logic"
    return OK;
}

/****************************************************************************
 * Name: can_rx1interrupt
 *
 * Description:
 *   CAN FIFO 1 interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_rx1interrupt(int irq, void *context)
{
  FAR struct stm32_can_s *priv;

#if defined(CONFIG_STM32_CAN1) && defined(CONFIG_STM32_CAN2)
  if (g_can1priv.canrx1 == irq)
    {
      priv = &g_can1priv;
    }
  else if (g_can2priv.canrx1 == irq)
    {
      priv = &g_can2priv;
    }
  else
    {
      PANIC(OSERR_UNEXPECTEDISR);
    }
#elif defined(CONFIG_STM32_CAN1)
  priv = &g_can1priv;
#else /* defined(CONFIG_STM32_CAN2) */
  priv = &g_can2priv;
#endif

#warning "Missing logic"
    return OK;
}

/****************************************************************************
 * Name: can_sceinterrupt
 *
 * Description:
 *   CAN Status Change Error (SCE) interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_sceinterrupt(int irq, void *context)
{
  FAR struct stm32_can_s *priv;

#if defined(CONFIG_STM32_CAN1) && defined(CONFIG_STM32_CAN2)
  if (g_can1priv.cansce == irq)
    {
      priv = &g_can1priv;
    }
  else if (g_can2priv.cansce == irq)
    {
      priv = &g_can2priv;
    }
  else
    {
      PANIC(OSERR_UNEXPECTEDISR);
    }
#elif defined(CONFIG_STM32_CAN1)
  priv = &g_can1priv;
#else /* defined(CONFIG_STM32_CAN2) */
  priv = &g_can2priv;
#endif

#warning "Missing logic"
    return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_caninitialize
 *
 * Description:
 *   Initialize the selected CAN port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple CAN interfaces)
 *
 * Returned Value:
 *   Valid CAN device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct can_dev_s *up_caninitialize(int port)
{
  struct can_dev_s *dev = NULL;

#ifdef CONFIG_STM32_CAN1  
  if( port == 1 )
    {
      dev = &g_can1dev;
    }
  else
#endif  
#ifdef CONFIG_STM32_CAN2  
  if ( port ==2 )
    {
      dev = &g_can2dev;
    }
  else
#endif  
    {
      candbg("Unsupport port %d\n", priv->port);
      return NULL;
    }

#warning "Missing logic"
  return dev;
}

#endif /* CONFIG_STM32_CAN1 || CONFIG_STM32_CAN2 */
#endif /* CONFIG_CAN */

