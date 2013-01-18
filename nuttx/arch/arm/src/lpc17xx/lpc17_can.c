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
#include "chip/lpc17_syscon.h"
#include "lpc17_gpio.h"
#include "lpc17_can.h"

#if defined(CONFIG_LPC17_CAN1) || defined(CONFIG_LPC17_CAN2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifdef CONFIG_LPC17_CAN1

   /* A CAN bit rate must be provided */

#  ifndef CONFIG_CAN1_BAUD
#    error "CONFIG_CAN1_BAUD is not defined"
#  endif

   /* If no divsor is provided, use a divisor of 4 */

#  ifndef CONFIG_CAN1_DIVISOR
#    define CONFIG_CAN1_DIVISOR 4
#  endif

   /* Get the SYSCON_PCLKSEL value for CAN1 the implements this divisor */

#  if CONFIG_CAN1_DIVISOR == 1
#    define CAN1_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK
#  elif CONFIG_CAN1_DIVISOR == 2
#    define CAN1_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK2
#  elif CONFIG_CAN1_DIVISOR == 4
#    define CAN1_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK4
#  elif CONFIG_CAN1_DIVISOR == 6
#    define CAN1_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK6
#  else
#    error "Unsupported value of CONFIG_CAN1_DIVISOR"
#  endif
#endif

#ifdef CONFIG_LPC17_CAN2

   /* A CAN bit rate must be provided */

#  ifndef CONFIG_CAN2_BAUD
#    error "CONFIG_CAN2_BAUD is not defined"
#  endif

   /* If no divsor is provided, use a divisor of 4 */

#  ifndef CONFIG_CAN2_DIVISOR
#    define CONFIG_CAN2_DIVISOR 4
#  endif

   /* Get the SYSCON_PCLKSEL value for CAN2 the implements this divisor */

#  if CONFIG_CAN2_DIVISOR == 1
#    define CAN2_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK
#  elif CONFIG_CAN2_DIVISOR == 2
#    define CAN2_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK2
#  elif CONFIG_CAN2_DIVISOR == 4
#    define CAN2_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK4
#  elif CONFIG_CAN2_DIVISOR == 6
#    define CAN2_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK6
#  else
#    error "Unsupported value of CONFIG_CAN2_DIVISOR"
#  endif
#endif

/* User-defined TSEG1 and TSEG2 settings may be used.
 *
 * CONFIG_CAN_TSEG1 = the number of CAN time quanta in segment 1
 * CONFIG_CAN_TSEG2 = the number of CAN time quanta in segment 2
 * CAN_BIT_QUANTA   = The number of CAN time quanta in on bit time
 */

#ifndef CONFIG_CAN_TSEG1
#  define CONFIG_CAN_TSEG1 6
#endif

#if CONFIG_CAN_TSEG1 < 1 || CONFIG_CAN_TSEG1 > CAN_BTR_TSEG1_MAX
#  errror "CONFIG_CAN_TSEG1 is out of range"
#endif

#ifndef CONFIG_CAN_TSEG2
#  define CONFIG_CAN_TSEG2 7
#endif

#if CONFIG_CAN_TSEG2 < 1 || CONFIG_CAN_TSEG2 > CAN_BTR_TSEG2_MAX
#  errror "CONFIG_CAN_TSEG2 is out of range"
#endif

#define CAN_BIT_QUANTA (CONFIG_CAN_TSEG1 + CONFIG_CAN_TSEG2 + 1)

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

/* Timing *******************************************************************/
/* CAN clocking is provided at CCLK divided by the configured divisor */

#define CAN_CLOCK_FREQUENCY(d) ((uint32_t)LPC17_CCLK / (uint32_t)(d))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint8_t  port;    /* CAN port number */
  uint8_t  divisor; /* CCLK divisor (numeric value) */
  uint32_t baud;    /* Configured baud */
  uint32_t base;    /* CAN register base address */
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
static bool can_txready(FAR struct can_dev_s *dev);
static bool can_txempty(FAR struct can_dev_s *dev);

/* CAN interrupts */

static void can_interrupt(FAR struct can_dev_s *dev);
static int  can12_interrupt(int irq, void *context);

/* Initialization */

static int can_bittiming(struct up_dev_s *priv);

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
  .co_txready       = can_txready,
  .co_txempty       = can_txempty,
};

#ifdef CONFIG_LPC17_CAN1
static struct up_dev_s g_can1priv =
{
  .port    = 1,
  .divisor = CONFIG_CAN1_DIVISOR,
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
  .divisor = CONFIG_CAN2_DIVISOR,
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
  irqstate_t flags;
  int ret;

  canvdbg("CAN%d\n", priv->port);

  flags = irqsave();

  /* Disable the CAN and stop ongong transmissions */

  can_putreg(priv, LPC17_CAN_MOD_OFFSET, CAN_MOD_RM);  /* Enter Reset Mode */
  can_putreg(priv, LPC17_CAN_IER_OFFSET, 0);           /* Disable interrupts */
  can_putreg(priv, LPC17_CAN_GSR_OFFSET, 0);           /* Clear status bits */
  can_putreg(priv, LPC17_CAN_CMR_OFFSET, CAN_CMR_AT);  /* Abort transmission */

  /* Set bit timing */

  ret = can_bittiming(priv);
  if (ret != OK)
    {
      candbg("ERROR: Failed to set bit timing: %d\n", ret);
    }

  /* Restart the CAN */

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
  uint32_t tid = (uint32_t)msg->cm_hdr.ch_id;
  uint32_t tfi = (uint32_t)msg->cm_hdr.ch_dlc << 16;
  uint32_t regval;
  irqstate_t flags;
  int ret = OK;

  canvdbg("CAN%d ID: %d DLC: %d\n", priv->port, msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  if (msg->cm_hdr.ch_rtr)
    {
      tfi |= CAN_TFI_RTR;
    }

  /* Set the FF bit in the TFI register if this message should be sent with
   * the extended frame format (and 29-bit extened ID).
   */

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      /* The provided ID should be 29 bits */

      DEBUGASSERT((tid & ~CAN_TID_ID29_MASK) == 0);
      tfi |= CAN_TFI_FF;
    }
  else
#endif
    {
      /* The provided ID should be 11 bits */

      DEBUGASSERT((tid & ~CAN_TID_ID11_MASK) == 0);
    }

  flags = irqsave();

  /* Pick a transmit buffer */

  regval = can_getreg(priv, LPC17_CAN_SR_OFFSET);
  if ((regval & CAN_SR_TBS1) != 0)
    {
      /* Make sure that buffer 1 TX interrupts are enabled BEFORE sending the
       * message. The TX interrupt is generated when the TBSn bit in CANxSR
       * goes from 0 to 1 when the TIEn bit in CANxIER is 1.  If we don't
       * enable it now, we may miss the TIE1 interrupt.
       *
       * NOTE: The IER is also modified from the interrupt handler, but the
       * following is safe because interrupts are disabled here.
       */

      regval  = can_getreg(priv, LPC17_CAN_IER_OFFSET);
      regval |= CAN_IER_TIE1;
      can_putreg(priv, LPC17_CAN_IER_OFFSET, regval);

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
    }
  else if ((regval & CAN_SR_TBS2) != 0)
    {
      /* Make sure that buffer 2 TX interrupts are enabled BEFORE sending the
       * message. The TX interrupt is generated when the TBSn bit in CANxSR
       * goes from 0 to 1 when the TIEn bit in CANxIER is 1.  If we don't
       * enable it now, we may miss the TIE2 interrupt.
       *
       * NOTE: The IER is also modified from the interrupt handler, but the
       * following is safe because interrupts are disabled here.
       */

      regval  = can_getreg(priv, LPC17_CAN_IER_OFFSET);
      regval |= CAN_IER_TIE2;
      can_putreg(priv, LPC17_CAN_IER_OFFSET, regval);

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
    }
  else if ((regval & CAN_SR_TBS3) != 0)
    {
      /* Make sure that buffer 3 TX interrupts are enabled BEFORE sending the
       * message. The TX interrupt is generated when the TBSn bit in CANxSR
       * goes from 0 to 1 when the TIEn bit in CANxIER is 1.  If we don't
       * enable it now, we may miss the TIE3 interrupt.
       *
       * NOTE: The IER is also modified from the interrupt handler, but the
       * following is safe because interrupts are disabled here.
       */

      regval  = can_getreg(priv, LPC17_CAN_IER_OFFSET);
      regval |= CAN_IER_TIE3;
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
 * Name: can_txready
 *
 * Description:
 *   Return true if the CAN hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if the CAN hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool can_txready(FAR struct can_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  uint32_t regval = can_getreg(priv, LPC17_CAN_SR_OFFSET);
  return ((regval & (CAN_SR_TBS1 | CAN_SR_TBS2 | CAN_SR_TBS3)) != 0);
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
 *   True if there are no pending TX transfers in the CAN hardware.
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
  struct can_hdr_s hdr;
  uint32_t data[2];
  uint32_t rfs;
  uint32_t rid;
  uint32_t regval;

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

      hdr.ch_id    = rid;
      hdr.ch_rtr   = ((rfs & CAN_RFS_RTR) != 0);
      hdr.ch_dlc   = (rfs & CAN_RFS_DLC_MASK) >> CAN_RFS_DLC_SHIFT;
#ifdef CONFIG_CAN_EXTID
      hdr.ch_extid = ((rfs & CAN_RFS_FF) != 0);
#else
      if ((rfs & CAN_RFS_FF) != 0)
        {
          canlldbg("ERROR: Received message with extended identifier.  Dropped\n");
        }
      else
#endif
        {
          /* Process the received CAN packet */

          can_receive(dev, &hdr, (uint8_t *)data);
        }
    }

  /* Check for TX buffer 1 complete */

  if ((regval & CAN_ICR_TI1) != 0)
    {
      /* Disable all further TX buffer 1 interrupts */

      regval  = can_getreg(priv, LPC17_CAN_IER_OFFSET);
      regval &= ~CAN_IER_TIE1;
      can_putreg(priv, LPC17_CAN_IER_OFFSET, regval);

      /* Indicate that the TX is done and a new TX buffer is available */

      can_txdone(dev);
    }

  /* Check for TX buffer 2 complete */

  if ((regval & CAN_ICR_TI2) != 0)
    {
      /* Disable all further TX buffer 2 interrupts */

      regval  = can_getreg(priv, LPC17_CAN_IER_OFFSET);
      regval &= ~CAN_IER_TIE2;
      can_putreg(priv, LPC17_CAN_IER_OFFSET, regval);

      /* Indicate that the TX is done and a new TX buffer is available */

      can_txdone(dev);
    }

  /* Check for TX buffer 3 complete */

  if ((regval & CAN_ICR_TI3) != 0)
    {
      /* Disable all further TX buffer 3 interrupts */

      regval = can_getreg(priv, LPC17_CAN_IER_OFFSET);
      regval &= ~CAN_IER_TIE3;
      can_putreg(priv, LPC17_CAN_IER_OFFSET, regval);

      /* Indicate that the TX is done and a new TX buffer is available */

      can_txdone(dev);
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
 * Name: can_bittiming
 *
 * Description:
 *   Set the CAN bit timing register (BTR) based on the configured BAUD.
 *
 * The bit timing logic monitors the serial bus-line and performs sampling
 * and adjustment of the sample point by synchronizing on the start-bit edge
 * and resynchronizing on the following edges.
 *
 * Its operation may be explained simply by splitting nominal bit time into
 * three segments as follows:
 *
 * 1. Synchronization segment (SYNC_SEG): a bit change is expected to occur
 *    within this time segment. It has a fixed length of one time quantum
 *    (1 x tCAN).
 * 2. Bit segment 1 (BS1): defines the location of the sample point. It
 *    includes the PROP_SEG and PHASE_SEG1 of the CAN standard. Its duration
 *    is programmable between 1 and 16 time quanta but may be automatically
 *    lengthened to compensate for positive phase drifts due to differences
 *    in the frequency of the various nodes of the network.
 * 3. Bit segment 2 (BS2): defines the location of the transmit point. It
 *    represents the PHASE_SEG2 of the CAN standard. Its duration is
 *    programmable between 1 and 8 time quanta but may also be automatically
 *    shortened to compensate for negative phase drifts.
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
 *   Tq = brp * Tcan
 *
 * Where:
 *   Tcan is the period of the APB clock (PCLK = CCLK / CONFIG_CAN1_DIVISOR).
 *
 * Input Parameter:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_bittiming(struct up_dev_s *priv)
{
  uint32_t btr;
  uint32_t nclks;
  uint32_t brp;
  uint32_t ts1;
  uint32_t ts2;
  uint32_t sjw;

  canllvdbg("CAN%d PCLK: %d baud: %d\n", priv->port,
            CAN_CLOCK_FREQUENCY(priv->divisor), priv->baud);

  /* Try to get CAN_BIT_QUANTA quanta in one bit_time.
   *
   *   bit_time = Tq*(ts1 + ts2 + 1)
   *   nquanta  = bit_time/Tq
   *   Tq       = brp * Tcan
   *   nquanta  = (ts1 + ts2 + 1)
   *
   *   bit_time = brp * Tcan * (ts1 + ts2 + 1)
   *   nquanta  = bit_time / brp / Tcan
   *   brp      = Fcan / baud / nquanta;
   *
   * First, calculate the number of CAN clocks in one bit time: Fcan / baud
   */

  nclks = CAN_CLOCK_FREQUENCY(priv->divisor) / priv->baud;
  if (nclks < CAN_BIT_QUANTA)
    {
      /* At the smallest brp value (1), there are already too few bit times
       * (CAN_CLOCK / baud) to meet our goal.  brp must be one and we need
       * make some reasonable guesses about ts1 and ts2.
       */

      brp = 1;

      /* In this case, we have to guess a good value for ts1 and ts2 */

      ts1 = (nclks - 1) >> 1;
      ts2 = nclks - ts1 - 1;
      if (ts1 == ts2 && ts1 > 1 && ts2 < CAN_BTR_TSEG2_MAX)
        {
          ts1--;
          ts2++;          
        }
    }

  /* Otherwise, nquanta is CAN_BIT_QUANTA, ts1 is CONFIG_CAN_TSEG1, ts2 is
   * CONFIG_CAN_TSEG2 and we calculate brp to achieve CAN_BIT_QUANTA quanta
   * in the bit time 
   */

  else
    {
      ts1 = CONFIG_CAN_TSEG1;
      ts2 = CONFIG_CAN_TSEG2;
      brp = (nclks + (CAN_BIT_QUANTA/2)) / CAN_BIT_QUANTA;
      DEBUGASSERT(brp >=1 && brp <= CAN_BTR_BRP_MAX);
    }
    
  sjw = 1;

  canllvdbg("TS1: %d TS2: %d BRP: %d SJW= %d\n", ts1, ts2, brp, sjw);

 /* Configure bit timing */

  btr = (((brp - 1) << CAN_BTR_BRP_SHIFT)   |
         ((ts1 - 1) << CAN_BTR_TSEG1_SHIFT) |
         ((ts2 - 1) << CAN_BTR_TSEG2_SHIFT) |
         ((sjw - 1) << CAN_BTR_SJW_SHIFT));

#ifdef CONFIG_CAN_SAM
  /* The bus is sampled 3 times (recommended for low to medium speed buses
   * to spikes on the bus-line).
   */

  btr |= CAN_BTR_SAM;
#endif

  canllvdbg("Setting CANxBTR= 0x%08x\n", btr);
  can_putreg(priv, LPC17_CAN_BTR_OFFSET, btr);        /* Set bit timing */
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
       * in low level clock configuration logic).
       */

      regval  = can_getcommon(LPC17_SYSCON_PCLKSEL0);
      regval &= ~SYSCON_PCLKSEL0_CAN1_MASK;
      regval |= (CAN1_CCLK_DIVISOR << SYSCON_PCLKSEL0_CAN1_SHIFT);
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
       * in low level clock configuration logic).
       */

      regval  = can_getcommon(LPC17_SYSCON_PCLKSEL0);
      regval &= ~SYSCON_PCLKSEL0_CAN2_MASK;
      regval |= (CAN2_CCLK_DIVISOR << SYSCON_PCLKSEL0_CAN2_SHIFT);
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

