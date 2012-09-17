/****************************************************************************
 * drivers/net/m9s12_ethernet.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_HCS12_EMAC)

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <wdog.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arp.h>
#include <nuttx/net/uip/uip-arch.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* CONFIG_HCS12_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_HCS12_NINTERFACES
# define CONFIG_HCS12_NINTERFACES 1
#endif

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define HCS12_WDDELAY   (1*CLK_TCK)
#define HCS12_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define HCS12_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)priv->d_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The emac_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct emac_driver_s
{
  bool    d_bifup;            /* true:ifup false:ifdown */
  WDOG_ID d_txpoll;           /* TX poll timer */
  WDOG_ID d_txtimeout;        /* TX timeout timer */

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s d_dev;  /* Interface understood by uIP */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct emac_driver_s g_emac[CONFIG_HCS12_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  emac_transmit(FAR struct emac_driver_s *priv);
static int  emac_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static void emac_receive(FAR struct emac_driver_s *priv);
static void emac_txdone(FAR struct emac_driver_s *priv);
static int  emac_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void emac_polltimer(int argc, uint32_t arg, ...);
static void emac_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int emac_ifup(struct uip_driver_s *dev);
static int emac_ifdown(struct uip_driver_s *dev);
static int emac_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int emac_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
static int emac_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: emac_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int emac_transmit(FAR struct emac_driver_s *priv)
{
  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is not transmission in progress.
   */

  /* Increment statistics */

  /* Send the packet: address=priv->d_dev.d_buf, length=priv->d_dev.d_len */

  /* Enable Tx interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->d_txtimeout, HCS12_TXTIMEOUT, emac_txtimeout, 1, (uint32_t)priv);
  return OK;
}

/****************************************************************************
 * Function: emac_uiptxpoll
 *
 * Description:
 *   The transmitter is available, check if uIP has any outgoing packets ready
 *   to send.  This is a callback from uip_poll().  uip_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int emac_uiptxpoll(struct uip_driver_s *dev)
{
  FAR struct emac_driver_s *priv = (FAR struct emac_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->d_dev.d_len > 0)
    {
      uip_arp_out(&priv->d_dev);
      emac_transmit(priv);

      /* Check if there is room in the device to hold another packet. If not,
       * return a non-zero value to terminate the poll.
       */
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: emac_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void emac_receive(FAR struct emac_driver_s *priv)
{
  do
    {
      /* Check for errors and update statistics */

      /* Check if the packet is a valid size for the uIP buffer configuration */

      /* Copy the data data from the hardware to priv->d_dev.d_buf.  Set
       * amount of data in priv->d_dev.d_len
       */

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
      if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
        {
          uip_arp_ipin(&priv->d_dev);
          uip_input(&priv->d_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->d_dev.d_len > 0)
           {
             uip_arp_out(&priv->d_dev);
             emac_transmit(priv);
           }
        }
      else if (BUF->type == htons(UIP_ETHTYPE_ARP))
        {
          uip_arp_arpin(&priv->d_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->d_dev.d_len > 0)
            {
              emac_transmit(priv);
            }
        }
    }
  while (); /* While there are more packets to be processed */
}

/****************************************************************************
 * Function: emac_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void emac_txdone(FAR struct emac_driver_s *priv)
{
  /* Check for errors and update statistics */

  /* If no further xmits are pending, then cancel the TX timeout and
   * disable further Tx interrupts.
   */

  wd_cancel(priv->d_txtimeout);

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->d_dev, emac_uiptxpoll);
}

/****************************************************************************
 * Function: emac_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int emac_interrupt(int irq, FAR void *context)
{
  register FAR struct emac_driver_s *priv = &g_emac[0];

  /* Get and clear interrupt status bits */

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call emac_receive() */

  emac_receive(priv);

  /* Check is a packet transmission just completed.  If so, call emac_txdone.
   * This may disable further Tx interrupts if there are no pending
   * tansmissions.
   */

  emac_txdone(priv);

  return OK;
}

/****************************************************************************
 * Function: emac_txtimeout
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void emac_txtimeout(int argc, uint32_t arg, ...)
{
  FAR struct emac_driver_s *priv = (FAR struct emac_driver_s *)arg;

  /* Increment statistics and dump debug info */

  /* Then reset the hardware */

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->d_dev, emac_uiptxpoll);
}

/****************************************************************************
 * Function: emac_polltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void emac_polltimer(int argc, uint32_t arg, ...)
{
  FAR struct emac_driver_s *priv = (FAR struct emac_driver_s *)arg;

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  /* If so, update TCP timing states and poll uIP for new XMIT data. Hmmm..
   * might be bug here.  Does this mean if there is a transmit in progress,
   * we will missing TCP time state updates?
   */

  (void)uip_timer(&priv->d_dev, emac_uiptxpoll, HCS12_POLLHSEC);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->d_txpoll, HCS12_WDDELAY, emac_polltimer, 1, arg);
}

/****************************************************************************
 * Function: emac_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided 
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int emac_ifup(struct uip_driver_s *dev)
{
  FAR struct emac_driver_s *priv = (FAR struct emac_driver_s *)dev->d_private;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Initialize PHYs, the Ethernet interface, and setup up Ethernet interrupts */

  /* Set and activate a timer process */

  (void)wd_start(priv->d_txpoll, HCS12_WDDELAY, emac_polltimer, 1, (uint32_t)priv);

  /* Enable the Ethernet interrupt */

  priv->d_bifup = true;
  up_enable_irq(CONFIG_HCS12_IRQ);
  return OK;
}

/****************************************************************************
 * Function: emac_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int emac_ifdown(struct uip_driver_s *dev)
{
  FAR struct emac_driver_s *priv = (FAR struct emac_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = irqsave();
  up_disable_irq(CONFIG_HCS12_IRQ);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->d_txpoll);
  wd_cancel(priv->d_txtimeout);

  /* Put the the EMAC is its reset, non-operational state.  This should be
   * a known configuration that will guarantee the emac_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->d_bifup = false;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: emac_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a 
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int emac_txavail(struct uip_driver_s *dev)
{
  FAR struct emac_driver_s *priv = (FAR struct emac_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable interrupts because this function may be called from interrupt
   * level processing.
   */

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (priv->d_bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */

      /* If so, then poll uIP for new XMIT data */

      (void)uip_poll(&priv->d_dev, emac_uiptxpoll);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: emac_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int emac_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct emac_driver_s *priv = (FAR struct emac_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: emac_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int emac_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct emac_driver_s *priv = (FAR struct emac_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: emac_initialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int emac_initialize(int intf)
{
  struct lpc17_driver_s *priv;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(inf <  CONFIG_HCS12_NINTERFACES);
  priv = &g_ethdrvr[intf];

   /* Check if a Ethernet chip is recognized at its I/O base */

  /* Attach the IRQ to the driver */

  if (irq_attach(CONFIG_HCS12_IRQ, emac_interrupt))
    {
      /* We could not attach the ISR to the the interrupt */

      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct emac_driver_s));
  priv->d_dev.d_ifup    = emac_ifup;     /* I/F down callback */
  priv->d_dev.d_ifdown  = emac_ifdown;   /* I/F up (new IP address) callback */
  priv->d_dev.d_txavail = emac_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->d_dev.d_addmac  = emac_addmac;   /* Add multicast MAC address */
  priv->d_dev.d_rmmac   = emac_rmmac;    /* Remove multicast MAC address */
#endif
  priv->d_dev.d_private = (void*)g_emac; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->d_txpoll       = wd_create();   /* Create periodic poll timer */
  priv->d_txtimeout    = wd_create();   /* Create TX timeout timer */

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling emac_ifdown().
   */

  /* Read the MAC address from the hardware into priv->d_dev.d_mac.ether_addr_octet */

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->d_dev);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_HCS12_EMAC */
