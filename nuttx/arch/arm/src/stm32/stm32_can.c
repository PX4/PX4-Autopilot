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

#if defined(CONFIG_CAN) && (defined(CONFIG_STM32_CAN1) || defined(CONFIG_STM32_CAN2))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Delays *******************************************************************/
/* Time out for INAK bit */

#define INAK_TIMEOUT 65535

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

#if !defined(CONFIG_DEBUG) || !defined(CONFIG_DEBUG_CAN)
#  undef CONFIG_CAN_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_can_s
{
  uint8_t  port;   /* CAN port number (1 or 2) */
  uint8_t  canrx0; /* CAN RX FIFO 0 IRQ number */
  uint8_t  filter; /* Filter number */
  uint32_t base;   /* Base address of the CAN registers */
  uint32_t baud;   /* Configured baud */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN Register access */

static uint32_t can_getreg(struct stm32_can_s *priv, int offset);
static void can_putreg(struct stm32_can_s *priv, int offset, uint32_t value);
#ifdef CONFIG_CAN_REGDEBUG
static void can_dumpctrlregs(struct stm32_can_s *priv, FAR const char *msg);
static void can_dumpmbregs(struct stm32_can_s *priv, FAR const char *msg);
static void can_dumpfiltregs(struct stm32_can_s *priv, FAR const char *msg);
#else
#  define can_dumpctrlregs(priv,msg)
#  define can_dumpmbregs(priv,msg)
#  define can_dumpfiltregs(priv,msg)
#endif

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

static int  can_rx0interrupt(int irq, void *context);

/* Initialization */

static int  can_bittiming(struct stm32_can_s *priv);
static int  can_cellinit(struct stm32_can_s *priv);
static int  can_filterinit(struct stm32_can_s *priv);

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
  .canrx0           = STM32_IRQ_USBLPCANRX0,
#else
  .canrx0           = STM32_IRQ_CAN1RX0,
#endif
  .filter           = 0,
  .base             = STM32_CAN1_BASE,
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
  .canrx0           = STM32_IRQ_CAN2RX0,
  .filter           = CAN_NFILTERS / 2,
  .base             = STM32_CAN2_BASE,
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
 * Name: can_getreg
 *
 * Description:
 *   Read the value of an CAN register.
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *
 ****************************************************************************/

static uint32_t can_getreg(struct stm32_can_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: can_getreg
 *
 * Description:
 *   Read the value of an CAN register.
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void can_putreg(struct stm32_can_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: can_dumpctrlregs
 *
 * Description:
 *   Dump the contents of all CAN control registers
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_REGDEBUG
static void can_dumpctrlregs(struct stm32_can_s *priv, FAR const char *msg)
{
  if (msg)
    {
      canlldbg("Control Registers: %s\n", msg);
    }
  else
    {
      canlldbg("Control Registers:\n");
    }
 
  /* CAN control and status registers */

  lldbg("  MCR: %08x   MSR: %08x   TSR: %08x\n",
        can_getreg(priv, STM32_CAN_MCR_OFFSET),
        can_getreg(priv, STM32_CAN_MSR_OFFSET),
        can_getreg(priv, STM32_CAN_TSR_OFFSET));

  lldbg(" RF0R: %08x  RF1R: %08x\n",
        can_getreg(priv, STM32_CAN_RF0R_OFFSET),
        can_getreg(priv, STM32_CAN_RF1R_OFFSET));

  lldbg("  IER: %08x   ESR: %08x   BTR: %08x\n",
        can_getreg(priv, STM32_CAN_IER_OFFSET),
        can_getreg(priv, STM32_CAN_ESR_OFFSET),
        can_getreg(priv, STM32_CAN_BTR_OFFSET));
}
#endif

/****************************************************************************
 * Name: can_dumpmbregs
 *
 * Description:
 *   Dump the contents of all CAN mailbox registers
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_REGDEBUG
static void can_dumpmbregs(struct stm32_can_s *priv, FAR const char *msg)
{
  if (msg)
    {
      canlldbg("Mailbox Registers: %s\n", msg);
    }
  else
    {
      canlldbg("Mailbox Registers:\n");
    }

  /* CAN mailbox registers (3 TX and 2 RX) */

  lldbg(" TI0R: %08x TDT0R: %08x TDL0R: %08x TDH0R: %08x\n",
        can_getreg(priv, STM32_CAN_TI0R_OFFSET),
        can_getreg(priv, STM32_CAN_TDT0R_OFFSET),
        can_getreg(priv, STM32_CAN_TDL0R_OFFSET),
        can_getreg(priv, STM32_CAN_TDH0R_OFFSET));

  lldbg(" TI1R: %08x TDT1R: %08x TDL1R: %08x TDH1R: %08x\n",
        can_getreg(priv, STM32_CAN_TI1R_OFFSET),
        can_getreg(priv, STM32_CAN_TDT1R_OFFSET),
        can_getreg(priv, STM32_CAN_TDL1R_OFFSET),
        can_getreg(priv, STM32_CAN_TDH1R_OFFSET));

  lldbg(" TI2R: %08x TDT2R: %08x TDL2R: %08x TDH2R: %08x\n",
        can_getreg(priv, STM32_CAN_TI2R_OFFSET),
        can_getreg(priv, STM32_CAN_TDT2R_OFFSET),
        can_getreg(priv, STM32_CAN_TDL2R_OFFSET),
        can_getreg(priv, STM32_CAN_TDH2R_OFFSET));

  lldbg(" RI0R: %08x RDT0R: %08x RDL0R: %08x RDH0R: %08x\n",
        can_getreg(priv, STM32_CAN_RI0R_OFFSET),
        can_getreg(priv, STM32_CAN_RDT0R_OFFSET),
        can_getreg(priv, STM32_CAN_RDL0R_OFFSET),
        can_getreg(priv, STM32_CAN_RDH0R_OFFSET));

  lldbg(" RI1R: %08x RDT1R: %08x RDL1R: %08x RDH1R: %08x\n",
        can_getreg(priv, STM32_CAN_RI1R_OFFSET),
        can_getreg(priv, STM32_CAN_RDT1R_OFFSET),
        can_getreg(priv, STM32_CAN_RDL1R_OFFSET),
        can_getreg(priv, STM32_CAN_RDH1R_OFFSET));
}
#endif

/****************************************************************************
 * Name: can_dumpfiltregs
 *
 * Description:
 *   Dump the contents of all CAN filter registers
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_REGDEBUG
static void can_dumpfiltregs(struct stm32_can_s *priv, FAR const char *msg)
{
  int i;

  if (msg)
    {
      canlldbg("Filter Registers: %s\n", msg);
    }
  else
    {
      canlldbg("Filter Registers:\n");
    }

  lldbg(" FMR: %08x   FM1R: %08x  FS1R: %08x FSA1R: %08x  FA1R: %08x\n",
        can_getreg(priv, STM32_CAN_FMR_OFFSET),
        can_getreg(priv, STM32_CAN_FM1R_OFFSET),
        can_getreg(priv, STM32_CAN_FS1R_OFFSET),
        can_getreg(priv, STM32_CAN_FFA1R_OFFSET),
        can_getreg(priv, STM32_CAN_FA1R_OFFSET));

  for (i = 0; i < CAN_NFILTERS; i++)
    {
      lldbg(" F%dR1: %08x F%dR2: %08x\n",
            i, can_getreg(priv, STM32_CAN_FR_OFFSET(i,1)),
            i, can_getreg(priv, STM32_CAN_FR_OFFSET(i,2)));
    }
}
#endif

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
  uint32_t regval;
  uint32_t regbit = 0;
  irqstate_t flags;

  canllvdbg("CAN%d\n", priv->port);

  /* Get the bits in the AHB1RSTR register needed to reset this CAN device */

#ifdef CONFIG_STM32_CAN1  
  if (priv->port == 1)
    {
      regbit = RCC_APB1RSTR_CAN1RST;
    }
  else
#endif
#ifdef CONFIG_STM32_CAN2 
  if (priv->port == 2)
    {
      regbit = RCC_APB1RSTR_CAN2RST;
    }
  else
#endif
    {
      canlldbg("Unsupported port %d\n", priv->port);
      return;
    }

  /* Disable interrupts momentary to stop any ongoing CAN event processing and
   * to prevent any concurrent access to the AHB1RSTR register.
  */

  flags = irqsave();

  /* Reset the CAN */

  regval  = getreg32(STM32_RCC_APB1RSTR);
  regval |= regbit;
  putreg32(regval, STM32_RCC_APB1RSTR);

  regval &= ~regbit;
  putreg32(regval, STM32_RCC_APB1RSTR);
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

  canllvdbg("CAN%d irq: %d\n", priv->port, priv->canrx0);

  /* CAN cell initialization */

  ret = can_cellinit(priv);
  if (ret < 0)
    {
      canlldbg("CAN%d cell initialization failed: %d\n", priv->port, ret);
      return ret;
    }

  can_dumpctrlregs(priv, "After cell initialization");
  can_dumpmbregs(priv, NULL);

  /* CAN filter initialization */

  ret = can_filterinit(priv);
  if (ret < 0)
    {
      canlldbg("CAN%d filter initialization failed: %d\n", priv->port, ret);
      return ret;
    }
  can_dumpfiltregs(priv, "After filter initialization");

  /* Attach only the CAN RX FIFO 0 interrupts.  The others are not used */

  ret = irq_attach(priv->canrx0, can_rx0interrupt);
  if (ret < 0)
    {
      canlldbg("Failed to attach CAN%d RX0 IRQ (%d)", priv->port, priv->canrx0);
      return ret;
    }

  /* Enable only the CAN RX FIFO 0 interrupts at the NVIC.  Interrupts are
   * still disabled in the CAN module.  Since we coming out of reset here,
   * there should be no pending interrupts.
   */

  up_enable_irq(priv->canrx0);
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

  canllvdbg("CAN%d\n", priv->port);

  /* Disable the RX FIFO 0 interrupt */

  up_disable_irq(priv->canrx0);

  /* Detach the RX FIFO 0 interrupt */

  irq_detach(priv->canrx0);

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
  uint32_t regval;

  canllvdbg("CAN%d enable: %d\n", priv->port, enable);

  /* Enable/disable the FIFO 0 message pending interrupt */

  regval  = can_getreg(priv, STM32_CAN_IER_OFFSET);
  if (enable)
    {
      regval |= CAN_IER_FMPIE0;
    }
  else
    {
      regval &= ~CAN_IER_FMPIE0;
    }
  can_putreg(priv, STM32_CAN_IER_OFFSET, regval);
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
  /* This driver does not use the TX interrupt */
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
#warning "Remote request not implemented"
  return -ENOSYS;
}

/****************************************************************************
 * Name: can_send
 *
 * Description:
 *    Send one can message.
 *
 *    One CAN-message consists of a maximum of 10 bytes.  A message is
 *    composed of at least the first 2 bytes (when there are no data bytes).
 *
 *    Byte 0:      Bits 0-7: Bits 3-10 of the 11-bit CAN identifier
 *    Byte 1:      Bits 5-7: Bits 0-2 of the 11-bit CAN identifier
 *                 Bit 4:    Remote Tranmission Request (RTR)
 *                 Bits 0-3: Data Length Code (DLC)
 *    Bytes 2-10: CAN data
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg)
{
  FAR struct stm32_can_s *priv = dev->cd_priv;
  FAR uint8_t *ptr;
  uint32_t regval;
  uint32_t tmp;
  int dlc;
  int txmb;

  canllvdbg("CAN%d ID: %d DLC: %d\n",
            priv->port, CAN_ID(msg->cm_hdr), CAN_DLC(msg->cm_hdr));

  /* Select one empty transmit mailbox */

  regval = can_getreg(priv, STM32_CAN_TSR_OFFSET);
  if ((regval & CAN_TSR_TME0) != 0)
    {
      txmb = 0;
    }
  else if ((regval & CAN_TSR_TME1) != 0)
    {
      txmb = 1;
    }
  else if ((regval & CAN_TSR_TME2) != 0)
    {
      txmb = 2;
    }
  else
    {
      canlldbg("ERROR: No available mailbox\n");
      return -EBUSY;
    }

  /* Set up the Id */

  regval  = can_getreg(priv, STM32_CAN_TIR_OFFSET(txmb));
  regval &= ~CAN_TIR_TXRQ;  /* Transmit Mailbox Request */
  can_putreg(priv, STM32_CAN_TIR_OFFSET(txmb), regval);

  /* Only standard (11-bit) CAN identifiers are support (the stm 32
   * supports extended, 29-bit identifiers, but this method does not.
   *
   * Get the 11-bit identifier from the header bits 0-7 and 13-15.
   */

  regval &= ~CAN_TIR_STID_MASK;
  regval |= (uint32_t)CAN_ID(msg->cm_hdr) << CAN_TIR_STID_SHIFT;
  can_putreg(priv, STM32_CAN_TIR_OFFSET(txmb), regval);

  /* Set up the DLC */

  dlc     = CAN_DLC(msg->cm_hdr);
  regval  = can_getreg(priv, STM32_CAN_TDTR_OFFSET(txmb));
  regval &= ~CAN_TDTR_DLC_MASK;
  regval |= (uint32_t)dlc << CAN_TDTR_DLC_SHIFT;
  can_putreg(priv, STM32_CAN_TDTR_OFFSET(txmb), regval);

  /* Set up the data fields */

  ptr    = msg->cm_data;
  regval = 0;

  if (dlc > 0)
    {
      tmp    = (uint32_t)*ptr++;
      regval = tmp << CAN_TDLR_DATA0_SHIFT;

      if (dlc > 1)
       {
         tmp    = (uint32_t)*ptr++;
         regval |= tmp << CAN_TDLR_DATA1_SHIFT;

         if (dlc > 2)
           {
             tmp    = (uint32_t)*ptr++;
             regval |= tmp << CAN_TDLR_DATA2_SHIFT;

             if (dlc > 3)
               {
                 tmp    = (uint32_t)*ptr++;
                 regval |= tmp << CAN_TDLR_DATA3_SHIFT;
               }
           }
       }
    }
  can_putreg(priv, STM32_CAN_TDLR_OFFSET(txmb), regval);

  regval = 0;
  if (dlc > 4)
    {
      tmp    = (uint32_t)*ptr++;
      regval = tmp << CAN_TDHR_DATA4_SHIFT;

      if (dlc > 5)
       {
         tmp    = (uint32_t)*ptr++;
         regval |= tmp << CAN_TDHR_DATA5_SHIFT;

         if (dlc > 6)
           {
             tmp    = (uint32_t)*ptr++;
             regval |= tmp << CAN_TDHR_DATA6_SHIFT;

             if (dlc > 7)
               {
                 tmp    = (uint32_t)*ptr++;
                 regval |= tmp << CAN_TDHR_DATA7_SHIFT;
               }
           }
       }
    }
  can_putreg(priv, STM32_CAN_TDHR_OFFSET(txmb), regval);

  /* Request transmission */

  regval  = can_getreg(priv, STM32_CAN_TIR_OFFSET(txmb));
  regval |= CAN_TIR_TXRQ;  /* Transmit Mailbox Request */
  can_putreg(priv, STM32_CAN_TIR_OFFSET(txmb), regval);

  /* Tell the upper half that the tansfer is finished now (we would have to
   * take the transfer complete interrupt to know it "really" finished.  So
   * although the transfer is not finished, all of the upper half resources
   * are now available so makes sense to call can_txdone now.
   */
   
  (void)can_txdone(dev);
  can_dumpmbregs(priv, "After send");
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
  uint32_t regval;

  /* Return false if any mailbox is unavailable */

  regval = can_getreg(priv, STM32_CAN_TSR_OFFSET);
  canllvdbg("CAN%d TSR: %08x\n", priv->port, regval);

  if ((regval & CAN_TSR_TME0) == 0)
    {
      return false;
    }
  else if ((regval & CAN_TSR_TME1) == 0)
    {
      return false;
    }
  else if ((regval & CAN_TSR_TME2) == 0)
    {
      return false;
    }
  return true;
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
  FAR struct can_dev_s *dev = NULL;
  FAR struct stm32_can_s *priv;
  uint8_t data[CAN_MAXDATALEN];
  uint32_t regval;
  int npending;
  int id;
  int rtr;
  int dlc;
  int ret;

#if defined(CONFIG_STM32_CAN1) && defined(CONFIG_STM32_CAN2)
  if (g_can1priv.canrx0 == irq)
    {
      dev = &g_can1dev;
    }
  else if (g_can2priv.canrx0 == irq)
    {
      dev = &g_can2dev;
    }
  else
    {
      PANIC(OSERR_UNEXPECTEDISR);
    }
#elif defined(CONFIG_STM32_CAN1)
  dev = &g_can1dev;
#else /* defined(CONFIG_STM32_CAN2) */
  dev = &g_can2dev;
#endif
  priv = dev->cd_priv;

  /* Verify that a message is pending in FIFO 0 */

  regval   = can_getreg(priv, STM32_CAN_RF0R_OFFSET);
  npending = (regval & CAN_RFR_FMP_MASK) >> CAN_RFR_FMP_SHIFT;
  if (npending < 1)
    {
      canlldbg("WARNING: No messages pending\n");
      return OK;
    }

  can_dumpmbregs(priv, "RX0 interrupt");

  /* Get the CAN identifier.  Only standard 11-bit IDs are supported */

  regval = can_getreg(priv, STM32_CAN_RI0R_OFFSET);
  if ((regval & CAN_RIR_IDE) != 0)
    {
      canlldbg("ERROR: Received message with extended identifier.  Dropped\n");
      ret = -ENOSYS;
      goto errout;
    }

  id = (regval & CAN_RIR_STID_MASK) >> CAN_RIR_STID_SHIFT;

  /* Get the Remote Transmission Request (RTR) */

  rtr = (regval & CAN_RIR_RTR) != 0 ? 1 : 0;

  /* Get the DLC */

  regval = can_getreg(priv, STM32_CAN_RDT0R_OFFSET);
  dlc = (regval & CAN_RDTR_DLC_MASK) >> CAN_RDTR_DLC_SHIFT;

  /* Save the message data */

  regval  = can_getreg(priv, STM32_CAN_RDL0R_OFFSET);
  data[0] = (regval & CAN_RDLR_DATA0_MASK) >> CAN_RDLR_DATA0_SHIFT;
  data[1] = (regval & CAN_RDLR_DATA1_MASK) >> CAN_RDLR_DATA1_SHIFT;
  data[2] = (regval & CAN_RDLR_DATA2_MASK) >> CAN_RDLR_DATA2_SHIFT;
  data[3] = (regval & CAN_RDLR_DATA3_MASK) >> CAN_RDLR_DATA3_SHIFT;

  regval  = can_getreg(priv, STM32_CAN_RDH0R_OFFSET);
  data[4] = (regval & CAN_RDHR_DATA4_MASK) >> CAN_RDHR_DATA4_SHIFT;
  data[5] = (regval & CAN_RDHR_DATA5_MASK) >> CAN_RDHR_DATA4_SHIFT;
  data[6] = (regval & CAN_RDHR_DATA6_MASK) >> CAN_RDHR_DATA4_SHIFT;
  data[7] = (regval & CAN_RDHR_DATA7_MASK) >> CAN_RDHR_DATA4_SHIFT;

  /* Provide the data to the upper half driver */

  ret = can_receive(dev, (uint16_t)CAN_HDR(id, rtr, dlc), data);

  /* Release the FIFO0 */

errout:
  regval  = can_getreg(priv, STM32_CAN_RF0R_OFFSET);
  regval |= CAN_RFR_RFOM;
  can_putreg(priv, STM32_CAN_RF0R_OFFSET, regval);
  return ret;
}

/****************************************************************************
 * Name: can_bittiming
 *
 * Description:
 *   Set the CAN bit timing register (BTR) based on the configured BAUD.
 *
 * "The bit timing logic monitors the serial bus-line and performs sampling
 *  and adjustment of the sample point by synchronizing on the start-bit edge
 *  and resynchronizing on the following edges.
 *
 * "Its operation may be explained simply by splitting nominal bit time into
 *  three segments as follows:
 *
 * 1. "Synchronization segment (SYNC_SEG): a bit change is expected to occur
 *     within this time segment. It has a fixed length of one time quantum
 *     (1 x tCAN).
 * 2. "Bit segment 1 (BS1): defines the location of the sample point. It
 *     includes the PROP_SEG and PHASE_SEG1 of the CAN standard. Its duration
 *     is programmable between 1 and 16 time quanta but may be automatically
 *     lengthened to compensate for positive phase drifts due to differences
 *     in the frequency of the various nodes of the network.
 * 3. "Bit segment 2 (BS2): defines the location of the transmit point. It
 *     represents the PHASE_SEG2 of the CAN standard. Its duration is
 *     programmable between 1 and 8 time quanta but may also be automatically
 *     shortened to compensate for negative phase drifts."
 *
 * Pictorially:
 *
 *  |<----------------- NOMINAL BIT TIME ----------------->|
 *  |<- SYNC_SEG ->|<------ BS1 ------>|<------ BS2 ------>|
 *  |<---- Tq ---->|<----- Tbs1 ------>|<----- Tbs2 ------>|
 *
 * Where
 *   Tbs1 is the duration of the BS1 segment
 *   Tbs2 is the duration of the BS2 segment
 *   Tq is the "Time Quantum"
 *
 * Relationships:
 *
 *   baud = 1 / bit_time
 *   bit_time = Tq + Tbs1 + Tbs2
 *   Tbs1 = Tq * ts1
 *   Tbs2 = Tq * ts2
 *   Tq = brp * Tpclk1
 *
 * Where:
 *   Tpclk1 is the period of the APB1 clock (PCLK1).
 *
 * Input Parameter:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_bittiming(struct stm32_can_s *priv)
{
  uint32_t tmp;
  uint32_t brp;
  uint32_t ts1;
  uint32_t ts2;

  canllvdbg("CAN%d PCLK1: %d baud: %d\n", priv->port, STM32_PCLK1_FREQUENCY, priv->baud);

  /* Try to get 14 quanta in one bit_time.  That is based on the idea that the ideal
   * would be ts1=6 nd ts2=7 and (1 + ts1 + ts2) = 14.
   *
   *   bit_time = Tq*(1 +ts1 + ts2)
   *   nquanta = bit_time/Tq
   *   nquanta  = (1 +ts1 + ts2)
   *
   *   bit_time = brp * Tpclk1 * (1 + ts1 + ts2)
   *   nquanta  = bit_time / brp / Tpclk1
   *            = PCLK1 / baud / brp
   *   brp      = PCLK1 / baud / nquanta;
   *
   * Example:
   *   PCLK1 = 42,000,000 baud = 1,000,000 nquanta = 14 : brp = 3
   *   PCLK1 = 42,000,000 baud =   700,000 nquanta = 14 : brp = 4
   */

  tmp = STM32_PCLK1_FREQUENCY / priv->baud;
  if (tmp < 14)
    {
      /* At the smallest brp value (1), there are already fewer bit times
       * (PCLCK1 / baud) is already smaller than our goal.  brp must be one
       * and we need make some reasonalble guesses about ts1 and ts2.
       */

      brp = 1;

      /* In this case, we have to guess a good value for ts1 and ts2 */

      ts1 = (tmp - 1) >> 1;
      ts2 = tmp - ts1 - 1;
      if (ts1 == ts2 && ts1 > 1 && ts2 < 16)
        {
          ts1--;
          ts2++;          
        }
    }

  /* Otherwise, nquanta is 14, ts1 is 6, ts2 is 7 and we calculate brp to
   * achieve 14 quanta in the bit time 
   */

  else
    {
      ts1 = 6;
      ts2 = 7;
      brp = tmp / 14;
      DEBUGASSERT(brp >=1 && brp < 1024);
    }

  canllvdbg("TS1: %d TS2: %d BRP: %d\n", ts1, ts2, brp);

 /* Configure bit timing.  This also does the the following, less obvious
   * things.  Unless loopback mode is enabled, it:
   *
   * - Disables silent mode.
   * - Disables loopback mode.
   *
   * NOTE that for the time being, SJW is set to 1 just because I don't
   * know any better.
   */

  tmp = ((brp - 1) << CAN_BTR_BRP_SHIFT) | ((ts1 - 1) << CAN_BTR_TS1_SHIFT) |
        ((ts2 - 1) << CAN_BTR_TS2_SHIFT) | ((1 - 1) << CAN_BTR_SJW_SHIFT);
#ifdef CONFIG_CAN_LOOPBACK
//tmp |= (CAN_BTR_LBKM | CAN_BTR_SILM);
  tmp |= CAN_BTR_LBKM;
#endif

  can_putreg(priv, STM32_CAN_BTR_OFFSET, tmp);
  return OK;
}

/****************************************************************************
 * Name: can_cellinit
 *
 * Description:
 *   CAN cell initialization
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int can_cellinit(struct stm32_can_s *priv)
{
  volatile uint32_t timeout;
  uint32_t regval;
  int ret;

  canllvdbg("CAN%d\n", priv->port);

  /* Exit from sleep mode */

  regval  = can_getreg(priv, STM32_CAN_MCR_OFFSET);
  regval &= ~CAN_MCR_SLEEP;
  can_putreg(priv, STM32_CAN_MCR_OFFSET, regval);

  /* Enter initialization mode */

  regval  = can_getreg(priv, STM32_CAN_MCR_OFFSET);
  regval |= CAN_MCR_INRQ;
  can_putreg(priv, STM32_CAN_MCR_OFFSET, regval);

  /* Wait until initialization mode is acknowledged */

  for (timeout = INAK_TIMEOUT; timeout > 0; timeout--)
    {
      regval = can_getreg(priv, STM32_CAN_MSR_OFFSET);
      if ((regval & CAN_MSR_INAK) != 0)
        {
          /* We are in initialization mode */

          break;
        }
    }

  /* Check for a timeout */

  if (timeout < 1)
    {
      canlldbg("ERROR: Timed out waiting to enter initialization mode\n");
      return -ETIMEDOUT;
    }

  /* Disable the following modes:
   *
   *  - Time triggered communication mode
   *  - Automatic bus-off management
   *  - Automatic wake-up mode
   *  - No automatic retransmission
   *  - Receive FIFO locked mode
   *  - Transmit FIFO priority
   */

  regval   = can_getreg(priv, STM32_CAN_MCR_OFFSET);
  regval &= ~(CAN_MCR_TXFP | CAN_MCR_RFLM | CAN_MCR_NART | CAN_MCR_AWUM | CAN_MCR_ABOM | CAN_MCR_TTCM);
  can_putreg(priv, STM32_CAN_MCR_OFFSET, regval);

  /* Configure bit timing. */
 
  ret = can_bittiming(priv);
  if (ret < 0)
    {
      canlldbg("ERROR: Failed to set bit timing: %d\n", ret);
      return ret;
    }

  /* Exit initialization mode */

  regval  = can_getreg(priv, STM32_CAN_MCR_OFFSET);
  regval &= ~CAN_MCR_INRQ;
  can_putreg(priv, STM32_CAN_MCR_OFFSET, regval);

  /* Wait until initialization mode is acknowledged */

  for (timeout = INAK_TIMEOUT; timeout > 0; timeout--)
    {
      regval = can_getreg(priv, STM32_CAN_MSR_OFFSET);
      if ((regval & CAN_MSR_INAK) == 0)
        {
          /* We are out of initialization mode */

          break;
        }
    }

  /* Check for a timeout */

  if (timeout < 1)
    {
      canlldbg("ERROR: Timed out waiting to exit initialization mode: %08x\n", regval);
      return -ETIMEDOUT;
    }
  return OK;
}

/****************************************************************************
 * Name: can_filterinit
 *
 * Description:
 *   CAN filter initialization
 *
 * Input Parameter:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int can_filterinit(struct stm32_can_s *priv)
{
  uint32_t regval;
  uint32_t bitmask;

  canllvdbg("CAN%d filter: %d\n", priv->port, priv->filter);

  /* Get the bitmask associated with the filter used by this CAN block */

  bitmask = ((uint32_t)1) << priv->filter;

  /* Enter filter initialization mode */

  regval  = can_getreg(priv, STM32_CAN_FMR_OFFSET);
  regval |= CAN_FMR_FINIT;
  can_putreg(priv, STM32_CAN_FMR_OFFSET, regval);

  /* Disable the filter */

  regval  = can_getreg(priv, STM32_CAN_FA1R_OFFSET);
  regval &= ~bitmask;
  can_putreg(priv, STM32_CAN_FA1R_OFFSET, regval);

  /* Select the 32-bit scale for the filter */

  regval  = can_getreg(priv, STM32_CAN_FS1R_OFFSET);
  regval |= bitmask;
  can_putreg(priv, STM32_CAN_FS1R_OFFSET, regval);
 
  /* There are 14 or 28 filter banks (depending) on the device.  Each filter bank is
   * composed of two 32-bit registers, CAN_FiR:
   */

  can_putreg(priv,  STM32_CAN_FR_OFFSET(priv->filter, 1), 0);
  can_putreg(priv,  STM32_CAN_FR_OFFSET(priv->filter, 2), 0);

 /* Set Id/Mask mode for the filter */

  regval  = can_getreg(priv, STM32_CAN_FM1R_OFFSET);
  regval &= ~bitmask;
  can_putreg(priv, STM32_CAN_FM1R_OFFSET, regval);

 /* Assign FIFO 0 for the filter */

  regval  = can_getreg(priv, STM32_CAN_FFA1R_OFFSET);
  regval &= ~bitmask;
  can_putreg(priv, STM32_CAN_FFA1R_OFFSET, regval);
 
  /* Enable the filter */

  regval  = can_getreg(priv, STM32_CAN_FA1R_OFFSET);
  regval |= bitmask;
  can_putreg(priv, STM32_CAN_FA1R_OFFSET, regval);

  /* Exit filter initialization mode */

  regval  = can_getreg(priv, STM32_CAN_FMR_OFFSET);
  regval &= ~CAN_FMR_FINIT;
  can_putreg(priv, STM32_CAN_FMR_OFFSET, regval);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_caninitialize
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

FAR struct can_dev_s *stm32_caninitialize(int port)
{
  struct can_dev_s *dev = NULL;

  canvdbg("CAN%d\n", port);

  /* NOTE:  Peripherical clocking for CAN1 and/or CAN2 was already provided
   * by stm32_clockconfig() early in the reset sequence.
   */

#ifdef CONFIG_STM32_CAN1  
  if( port == 1 )
    {
      /* Select the CAN1 device structure */

      dev = &g_can1dev;

      /* Configure CAN1 pins.  The ambiguous settings in the stm32*_pinmap.h
       * file must have been disambiguated in the board.h file.
       */

      stm32_configgpio(GPIO_CAN1_RX);
      stm32_configgpio(GPIO_CAN1_TX);
    }
  else
#endif  
#ifdef CONFIG_STM32_CAN2  
  if ( port ==2 )
    {
      /* Select the CAN2 device structure */

      dev = &g_can2dev;

      /* Configure CAN2 pins.  The ambiguous settings in the stm32*_pinmap.h
       * file must have been disambiguated in the board.h file.
       */

      stm32_configgpio(GPIO_CAN2_RX);
      stm32_configgpio(GPIO_CAN2_TX);
    }
  else
#endif  
    {
      candbg("ERROR: Unsupported port %d\n", port);
      return NULL;
    }

  return dev;
}

#endif /* CONFIG_CAN && (CONFIG_STM32_CAN1 || CONFIG_STM32_CAN2) */

