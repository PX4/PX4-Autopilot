/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_can.c
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors:
 *     Li Zhuoyi <lzyy.cn@gmail.com>
 *     Gregory Nutt <gnutt@nuttx.org>
 *   History: 
 *     2011-07-12: Initial version (Li Zhuoyi)
 *     2011-08-03: Support CAN1/CAN2 (Li Zhuoyi)
 *     2012-01-02: Add support for CAN loopback mode (Gregory Nutt)
 * 
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#include "chip.h"
#include "lpc17_internal.h"
#include "lpc17_syscon.h"
#include "lpc17_pinconn.h"
#include "lpc17_can.h"

#if defined(CONFIG_LPC17_CAN1) || defined(CONFIG_LPC17_CAN2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing CAN */

#if !defined(CONFIG_DEBUG) || !defined(CONFIG_DEBUG_CAN)
#  undef CONFIG_CAN_REGDEBUG
#endif

#ifdef CONFIG_DEBUG_CAN
#  ifdef CONFIG_CAN_REGDEBUG
#    define candbg  lldbg
#    define canvdbg llvdbg
#  else
#    define candbg  dbg
#    define canvdbg vdbg
#  endif
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

struct up_dev_s
{
  uint8_t  port; /* CAN port number */
  uint32_t baud; /* Configured baud */
  uint32_t base; /* CAN register base address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* CAN Register access */

#ifdef CONFIG_CAN_REGDEBUG
static void can_printreg(uint32_t addr, uint32_t value);
#endif

static uint32_t can_getreg(struct up_dev_s *priv, int offset);
static void can_putreg(struct up_dev_s *priv, int offset, uint32_t value);

#ifdef CONFIG_CAN_REGDEBUG
static uint32_t can_getcommon(uint32_t addr);
static void can_putcommon(uint32_t addr, uint32_t value);
#else
#  define can_getcommon(addr)        getreg32(addr)
#  define can_putcommon(addr, value) putreg32(value, addr)
#endif

/* CAN methods */

static void can_reset(FAR struct can_dev_s *dev);
static int  can_setup(FAR struct can_dev_s *dev);
static void can_shutdown(FAR struct can_dev_s *dev);
static void can_rxint(FAR struct can_dev_s *dev, bool enable);
static void can_txint(FAR struct can_dev_s *dev, bool enable);
static int  can_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg);
static int  can_remoterequest(FAR struct can_dev_s *dev, uint16_t id);
static int  can_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg);
static bool can_txempty(FAR struct can_dev_s *dev);

static void can_interrupt(FAR struct can_dev_s *dev);
static int  can12_interrupt(int irq, void *context);

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

#ifdef CONFIG_LPC17_CAN1
static struct up_dev_s g_can1priv =
{
  .port    = 1,
  .baud    = CONFIG_CAN1_BAUD,
  .base    = LPC17_CAN1_BASE,
};

static struct can_dev_s g_can1dev =
{
  .cd_ops  = &g_canops,
  .cd_priv = &g_can1priv,
};
#endif

#ifdef CONFIG_LPC17_CAN2
static struct up_dev_s g_can2priv =
{
  .port    = 2,
  .baud    = CONFIG_CAN2_BAUD,
  .base    = LPC17_CAN2_BASE,
};

static struct can_dev_s g_can2dev =
{
  .cd_ops  = &g_canops,
  .cd_priv = &g_can2priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: can_printreg
 *
 * Description:
 *   Print the value read from a register.
 *
 * Input Parameters:
 *   addr - The register address
 *   value - The register value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_REGDEBUG
static void can_printreg(uint32_t addr, uint32_t value)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && value == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
           if (count == 4)
             {
               lldbg("...\n");
             }
          return;
        }
    }

  /* No this is a new address or value */

  else
    {
       /* Did we print "..." for the previous value? */

       if (count > 3)
         {
           /* Yes.. then show how many times the value repeated */

           lldbg("[repeats %d more times]\n", count-3);
         }

       /* Save the new address, value, and count */

       prevaddr = addr;
       preval   = value;
       count    = 1;
    }

  /* Show the register value read */

  lldbg("%08x->%08x\n", addr, value);
}
#endif

/****************************************************************************
 * Name: can_getreg
 *
 * Description:
 *   Read the value of an CAN1/2 register.
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_REGDEBUG
static uint32_t can_getreg(struct up_dev_s *priv, int offset)
{
  uint32_t addr;
  uint32_t value;

  /* Read the value from the register */

  addr  = priv->base + offset;
  value = getreg32(addr);
  can_printreg(addr, value);
  return value;
}
#else
static uint32_t can_getreg(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}
#endif

/****************************************************************************
 * Name: can_putreg
 *
 * Description:
 *   Set the value of an CAN1/2 register.
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *   offset - The offset to the register to write
 *   value - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_REGDEBUG
static void can_putreg(struct up_dev_s *priv, int offset, uint32_t value)
{
  uint32_t addr = priv->base + offset;

  /* Show the register value being written */

  lldbg("%08x<-%08x\n", addr, value);

  /* Write the value */

  putreg32(value, addr);
}
#else
static void can_putreg(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}
#endif

/****************************************************************************
 * Name: can_getcommon
 *
 * Description:
 *   Get the value of common register.
 *
 * Input Parameters:
 *   addr - The address of the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_REGDEBUG
static uint32_t can_getcommon(uint32_t addr)
{
  uint32_t value;

  /* Read the value from the register */

  value = getreg32(addr);
  can_printreg(addr, value);
  return value;
}
#endif

/****************************************************************************
 * Name: can_putcommon
 *
 * Description:
 *   Set the value of common register.
 *
 * Input Parameters:
 *   addr - The address of the register to write
 *   value - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_REGDEBUG
static void can_putcommon(uint32_t addr, uint32_t value)
{
  /* Show the register value being written */

  lldbg("%08x<-%08x\n", addr, value);

  /* Write the value */

  putreg32(value, addr);
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
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  uint32_t baud;
  irqstate_t flags;

  canvdbg("CAN%d\n", priv->port);

#warning "BTR setting must be calculated from priv->baud"
  baud  = 0x25c003;
  flags = irqsave();

  can_putreg(priv, LPC17_CAN_MOD_OFFSET, CAN_MOD_RM);  /* Enter Reset Mode */
  can_putreg(priv, LPC17_CAN_IER_OFFSET, 0);           /* Disable interrupts */
  can_putreg(priv, LPC17_CAN_GSR_OFFSET, 0);           /* Clear status bits */
  can_putreg(priv, LPC17_CAN_CMR_OFFSET, CAN_CMR_AT);  /* Abort transmission */
  can_putreg(priv, LPC17_CAN_BTR_OFFSET, baud);        /* Set bit timing */
#ifdef CONFIG_CAN_LOOPBACK
  can_putreg(priv, LPC17_CAN_MOD_OFFSET, CAN_MOD_STM); /* Leave Reset Mode, enter Test Mode */
#else
  can_putreg(priv, LPC17_CAN_MOD_OFFSET, 0);           /* Leave Reset Mode */
#endif
  can_putcommon(LPC17_CANAF_AFMR, CANAF_AFMR_ACCBP);   /* All RX messages accepted */
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
#ifdef CONFIG_DEBUG_CAN
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
#endif
  int ret;

  canvdbg("CAN%d\n", priv->port);

  ret = irq_attach(LPC17_IRQ_CAN, can12_interrupt);
  if (ret == OK)
    {
      up_enable_irq(LPC17_IRQ_CAN);
    }
  return ret;
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
#ifdef CONFIG_DEBUG_CAN
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;

  canvdbg("CAN%d\n", priv->port);
#endif

  up_disable_irq(LPC17_IRQ_CAN);
  irq_detach(LPC17_IRQ_CAN);
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
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  uint32_t regval;
  irqstate_t flags;

  canvdbg("CAN%d enable: %d\n", priv->port, enable);

   /* The EIR register is also modifed from the interrupt handler, so we have
    * to protect this code section.
    */

  flags = irqsave();
  regval = can_getreg(priv, LPC17_CAN_IER_OFFSET);
  if (enable)
    {
      regval |= CAN_IER_RIE;
    }
  else
    {
      regval &= ~CAN_IER_RIE;
    }
  can_putreg(priv, LPC17_CAN_IER_OFFSET, regval);
  irqrestore(flags);
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
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  uint32_t regval;
  irqstate_t flags;

  canvdbg("CAN%d enable: %d\n", priv->port, enable);

  /* Only disabling of the TX interrupt is supported here.  The TX interrupt
   * is automatically enabled just before a message is sent in order to avoid
   * lost TX interrupts.
   */

  if (!enable)
    {
      /* TX interrupts are also disabled from the interrupt handler, so we have
       * to protect this code section.
       */

      flags = irqsave();

      /* Disable all TX interrupts */

      regval = can_getreg(priv, LPC17_CAN_IER_OFFSET);
      regval &= ~(CAN_IER_TIE1 | CAN_IER_TIE2 | CAN_IER_TIE3);
      can_putreg(priv, LPC17_CAN_IER_OFFSET, regval);
      irqrestore(flags);
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
  dbg("Fix me:Not Implemented\n");
  return 0;
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

static int can_remoterequest(FAR struct can_dev_s *dev, uint16_t id)
{
  dbg("Fix me:Not Implemented\n");
  return 0;
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
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  uint32_t tid = CAN_ID(msg->cm_hdr);
  uint32_t tfi = CAN_DLC(msg->cm_hdr) << 16;
  uint32_t regval;
  irqstate_t flags;
  int ret = OK;

  canvdbg("CAN%d ID: %d DLC: %d\n",
          priv->port, CAN_ID(msg->cm_hdr), CAN_DLC(msg->cm_hdr));

  if (CAN_RTR(msg->cm_hdr))
    {
      tfi |= CAN_TFI_RTR;
    }

  flags = irqsave();

  /* Pick a transmit buffer */

  regval = can_getreg(priv, LPC17_CAN_SR_OFFSET);
  if ((regval & CAN_SR_TBS1) != 0)
    {
      /* Set up the transfer */

      can_putreg(priv, LPC17_CAN_TFI1_OFFSET, tfi);
      can_putreg(priv, LPC17_CAN_TID1_OFFSET, tid);
      can_putreg(priv, LPC17_CAN_TDA1_OFFSET, *(uint32_t *)&msg->cm_data[0]);
      can_putreg(priv, LPC17_CAN_TDB1_OFFSET, *(uint32_t *)&msg->cm_data[4]);

      /* Send the message */

#ifdef CONFIG_CAN_LOOPBACK
      can_putreg(priv, LPC17_CAN_CMR_OFFSET, CAN_CMR_STB1 | CAN_CMR_SRR);
#else
      can_putreg(priv, LPC17_CAN_CMR_OFFSET, CAN_CMR_STB1 | CAN_CMR_TR);
#endif

      /* Tell the caller that the transfer is done.  It isn't, but we have
       * more transmit buffers and this can speed things up.
       */

      can_txdone(dev);
    }
  else if ((regval & CAN_SR_TBS2) != 0)
    {
      /* Set up the transfer */

      can_putreg(priv, LPC17_CAN_TFI2_OFFSET, tfi);
      can_putreg(priv, LPC17_CAN_TID2_OFFSET, tid);
      can_putreg(priv, LPC17_CAN_TDA2_OFFSET, *(uint32_t *)&msg->cm_data[0]);
      can_putreg(priv, LPC17_CAN_TDB2_OFFSET, *(uint32_t *)&msg->cm_data[4]);

      /* Send the message */

#ifdef CONFIG_CAN_LOOPBACK
      can_putreg(priv, LPC17_CAN_CMR_OFFSET, CAN_CMR_STB2 | CAN_CMR_SRR);
#else
      can_putreg(priv, LPC17_CAN_CMR_OFFSET, CAN_CMR_STB2 | CAN_CMR_TR);
#endif

      /* Tell the caller that the transfer is done.  It isn't, but we have
       * more transmit buffers and this can speed things up.
       */

      can_txdone(dev);
    }
  else if ((regval & CAN_SR_TBS3) != 0)
    {
      /* We have no more buffers.  We will make the caller wait.  First, make
       * sure that all buffer 3 interrupts are enabled BEFORE sending the
       * message. The TX interrupt is generated TBSn bit in CANxSR goes from 0
       * to 1 when the TIEn bit in CANxIER is 1.  If we don't enable it now,
       * we will miss the TIE3 interrupt.  We enable ALL TIE interrupts here
       * because we don't care which one finishes:  When first one finishes it
       * means that a transmit buffer is again available.
       *
       * NOTE: The IER is also modified from the interrupt handler, but the
       * following is safe because interrupts are disabled here.
       */

      regval  = can_getreg(priv, LPC17_CAN_IER_OFFSET);
      regval |= (CAN_IER_TIE1 | CAN_IER_TIE2 | CAN_IER_TIE3);
      can_putreg(priv, LPC17_CAN_IER_OFFSET, regval);

      /* Set up the transfer */

      can_putreg(priv, LPC17_CAN_TFI3_OFFSET, tfi);
      can_putreg(priv, LPC17_CAN_TID3_OFFSET, tid);
      can_putreg(priv, LPC17_CAN_TDA3_OFFSET, *(uint32_t *)&msg->cm_data[0]);
      can_putreg(priv, LPC17_CAN_TDB3_OFFSET, *(uint32_t *)&msg->cm_data[4]);

      /* Send the message */

#ifdef CONFIG_CAN_LOOPBACK
      can_putreg(priv, LPC17_CAN_CMR_OFFSET, CAN_CMR_STB3 | CAN_CMR_SRR);
#else
      can_putreg(priv, LPC17_CAN_CMR_OFFSET, CAN_CMR_STB3 | CAN_CMR_TR);
#endif
    }
  else
    {
      candbg("No available transmission buffer, SR: %08x\n", regval);
      ret = -EBUSY;
    }

  irqrestore(flags);
  return ret;
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
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  uint32_t regval = can_getreg(priv, LPC17_CAN_GSR_OFFSET);
  return ((regval & CAN_GSR_TBS) != 0);
}

/****************************************************************************
 * Name: can_interrupt
 *
 * Description:
 *   CAN1/2 RX/TX interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static void can_interrupt(FAR struct can_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  uint32_t data[2];
  uint32_t rfs;
  uint32_t rid;
  uint32_t regval;
  uint16_t hdr;
  uint16_t id;
  uint16_t dlc;
  uint16_t rtr;

  /* Read the interrupt and capture register (also clearing most status bits) */

  regval = can_getreg(priv, LPC17_CAN_ICR_OFFSET);
  canllvdbg("CAN%d ICR: %08x\n",  priv->port, regval);

  /* Check for a receive interrupt */

  if ((regval & CAN_ICR_RI) != 0)
    {
      rfs     = can_getreg(priv, LPC17_CAN_RFS_OFFSET);
      rid     = can_getreg(priv, LPC17_CAN_RID_OFFSET);
      data[0] = can_getreg(priv, LPC17_CAN_RDA_OFFSET);
      data[1] = can_getreg(priv, LPC17_CAN_RDB_OFFSET);

      /* Release the receive buffer */

      can_putreg(priv, LPC17_CAN_CMR_OFFSET, CAN_CMR_RRB);

      /* Construct the CAN header */

      id      = rid & CAN_RID_ID11_MASK;
      dlc     = (rfs & CAN_RFS_DLC_MASK) >> CAN_RFS_DLC_SHIFT;
      rtr     = ((rfs & CAN_RFS_RTR) != 0);
      hdr     = CAN_HDR(id, rtr, dlc);

      /* Process the received CAN packet */

      can_receive(dev, hdr, (uint8_t *)data);
    }

  /* Check for a transmit interrupt from buffer 1, 2, or 3 meaning that at
   * least one TX is complete and that at least one TX buffer is available.
   */

  if ((regval & (CAN_ICR_TI1 | CAN_ICR_TI2 |CAN_ICR_TI3)) != 0)
    {
      /* Disable all further TX interrupts */

      regval = can_getreg(priv, LPC17_CAN_IER_OFFSET);
      regval &= ~(CAN_IER_TIE1 | CAN_IER_TIE2 | CAN_IER_TIE3);
      can_putreg(priv, LPC17_CAN_IER_OFFSET, regval);

      /* Indicate that the TX is done and a new TX buffer is available */

      can_txdone(&g_can1dev);
    }
}

/****************************************************************************
 * Name: can12_interrupt
 *
 * Description:
 *   CAN interrupt handler.  There is a single interrupt for both CAN1 and
 *   CAN2.
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can12_interrupt(int irq, void *context)
{
  /* Handle CAN1/2 interrupts */

  canllvdbg("irq: %d\n",  irq);

#ifdef CONFIG_LPC17_CAN1
  can_interrupt(&g_can1dev);
#endif
#ifdef CONFIG_LPC17_CAN2
  can_interrupt(&g_can2dev);
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: lpc17_caninitialize
 *
 * Description:
 *   Initialize the selected can port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple can interfaces)
 *
 * Returned Value:
 *   Valid can device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct can_dev_s *lpc17_caninitialize(int port)
{
  FAR struct can_dev_s *candev;
  irqstate_t flags;
  uint32_t regval;

  canllvdbg("CAN%d\n",  port);

  flags = irqsave();

#ifdef CONFIG_LPC17_CAN1  
  if (port == 1)
    {
      /* Enable power to the CAN module */

      regval  = can_getcommon(LPC17_SYSCON_PCONP);
      regval |= SYSCON_PCONP_PCCAN1;
      can_putcommon(LPC17_SYSCON_PCONP, regval);

      /* Enable clocking to the CAN module (not necessary... already done
       * in low level clock configuration logic.
       */

      regval  = can_getcommon(LPC17_SYSCON_PCLKSEL0);
      regval &= ~SYSCON_PCLKSEL0_CAN1_MASK;
      regval |= (SYSCON_PCLKSEL_CCLK4 << SYSCON_PCLKSEL0_CAN1_SHIFT);
      can_putcommon(LPC17_SYSCON_PCLKSEL0, regval);

      /* Configure CAN GPIO pins */

      lpc17_configgpio(GPIO_CAN1_RD);
      lpc17_configgpio(GPIO_CAN1_TD);

      candev = &g_can1dev;
    }
  else
#endif
#ifdef CONFIG_LPC17_CAN2  
  if (port == 2)
    {
      /* Enable power to the CAN module */

      regval  = can_getcommon(LPC17_SYSCON_PCONP);
      regval |= SYSCON_PCONP_PCCAN2;
      can_putcommon(LPC17_SYSCON_PCONP, regval);

      /* Enable clocking to the CAN module (not necessary... already done
       * in low level clock configuration logic.
       */

      regval  = can_getcommon(LPC17_SYSCON_PCLKSEL0);
      regval &= ~SYSCON_PCLKSEL0_CAN2_MASK;
      regval |= (SYSCON_PCLKSEL_CCLK4 << SYSCON_PCLKSEL0_CAN2_SHIFT);
      can_putcommon(LPC17_SYSCON_PCLKSEL0, regval);

      /* Configure CAN GPIO pins */

      lpc17_configgpio(GPIO_CAN2_RD);
      lpc17_configgpio(GPIO_CAN2_TD);

      candev = &g_can2dev;
   }
 else
#endif
  {
    candbg("Unsupported port: %d\n", port);
    irqrestore(flags);
    return NULL;
  }

  /* Then just perform a CAN reset operation */

  can_reset(candev);
  irqrestore(flags);
  return candev;
}
#endif

