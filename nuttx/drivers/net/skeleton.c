/****************************************************************************
 * drivers/net/skeleton.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_skeleton)

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

/* CONFIG_skeleton_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_skeleton_NINTERFACES
# define CONFIG_skeleton_NINTERFACES 1
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define skeleton_WDDELAY   (1*CLK_TCK)
#define skeleton_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define skeleton_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)skel->sk_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The skel_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct skel_driver_s
{
  bool sk_bifup;               /* true:ifup false:ifdown */
  WDOG_ID sk_txpoll;           /* TX poll timer */
  WDOG_ID sk_txtimeout;        /* TX timeout timer */

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s sk_dev;  /* Interface understood by uIP */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct skel_driver_s g_skel[CONFIG_skeleton_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  skel_transmit(FAR struct skel_driver_s *skel);
static int  skel_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static void skel_receive(FAR struct skel_driver_s *skel);
static void skel_txdone(FAR struct skel_driver_s *skel);
static int  skel_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void skel_polltimer(int argc, uint32_t arg, ...);
static void skel_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int skel_ifup(struct uip_driver_s *dev);
static int skel_ifdown(struct uip_driver_s *dev);
static int skel_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int skel_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
static int skel_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: skel_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   skel  - Reference to the driver state structure
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

static int skel_transmit(FAR struct skel_driver_s *skel)
{
  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  /* Increment statistics */

  /* Send the packet: address=skel->sk_dev.d_buf, length=skel->sk_dev.d_len */

  /* Enable Tx interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(skel->sk_txtimeout, skeleton_TXTIMEOUT, skel_txtimeout, 1, (uint32_t)skel);
  return OK;
}

/****************************************************************************
 * Function: skel_uiptxpoll
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

static int skel_uiptxpoll(struct uip_driver_s *dev)
{
  FAR struct skel_driver_s *skel = (FAR struct skel_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (skel->sk_dev.d_len > 0)
    {
      uip_arp_out(&skel->sk_dev);
      skel_transmit(skel);

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
 * Function: skel_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   skel  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void skel_receive(FAR struct skel_driver_s *skel)
{
  do
    {
      /* Check for errors and update statistics */

      /* Check if the packet is a valid size for the uIP buffer configuration */

      /* Copy the data data from the hardware to skel->sk_dev.d_buf.  Set
       * amount of data in skel->sk_dev.d_len
       */

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
      if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
        {
          uip_arp_ipin(&skel->sk_dev);
          uip_input(&skel->sk_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (skel->sk_dev.d_len > 0)
           {
             uip_arp_out(&skel->sk_dev);
             skel_transmit(skel);
           }
        }
      else if (BUF->type == htons(UIP_ETHTYPE_ARP))
        {
          uip_arp_arpin(&skel->sk_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (skel->sk_dev.d_len > 0)
            {
              skel_transmit(skel);
            }
        }
    }
  while (); /* While there are more packets to be processed */
}

/****************************************************************************
 * Function: skel_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   skel  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void skel_txdone(FAR struct skel_driver_s *skel)
{
  /* Check for errors and update statistics */

  /* If no further xmits are pending, then cancel the TX timeout and
   * disable further Tx interrupts.
   */

  wd_cancel(skel->sk_txtimeout);

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&skel->sk_dev, skel_uiptxpoll);
}

/****************************************************************************
 * Function: skel_interrupt
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

static int skel_interrupt(int irq, FAR void *context)
{
  register FAR struct skel_driver_s *skel = &g_skel[0];

  /* Get and clear interrupt status bits */

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call skel_receive() */

  skel_receive(skel);

  /* Check if a packet transmission just completed.  If so, call skel_txdone.
   * This may disable further Tx interrupts if there are no pending
   * tansmissions.
   */

  skel_txdone(skel);

  return OK;
}

/****************************************************************************
 * Function: skel_txtimeout
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

static void skel_txtimeout(int argc, uint32_t arg, ...)
{
  FAR struct skel_driver_s *skel = (FAR struct skel_driver_s *)arg;

  /* Increment statistics and dump debug info */

  /* Then reset the hardware */

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&skel->sk_dev, skel_uiptxpoll);
}

/****************************************************************************
 * Function: skel_polltimer
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

static void skel_polltimer(int argc, uint32_t arg, ...)
{
  FAR struct skel_driver_s *skel = (FAR struct skel_driver_s *)arg;

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  /* If so, update TCP timing states and poll uIP for new XMIT data. Hmmm..
   * might be bug here.  Does this mean if there is a transmit in progress,
   * we will missing TCP time state updates?
   */

  (void)uip_timer(&skel->sk_dev, skel_uiptxpoll, skeleton_POLLHSEC);

  /* Setup the watchdog poll timer again */

  (void)wd_start(skel->sk_txpoll, skeleton_WDDELAY, skel_polltimer, 1, arg);
}

/****************************************************************************
 * Function: skel_ifup
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

static int skel_ifup(struct uip_driver_s *dev)
{
  FAR struct skel_driver_s *skel = (FAR struct skel_driver_s *)dev->d_private;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Initialize PHYs, the Ethernet interface, and setup up Ethernet interrupts */

  /* Set and activate a timer process */

  (void)wd_start(skel->sk_txpoll, skeleton_WDDELAY, skel_polltimer, 1, (uint32_t)skel);

  /* Enable the Ethernet interrupt */

  skel->sk_bifup = true;
  up_enable_irq(CONFIG_skeleton_IRQ);
  return OK;
}

/****************************************************************************
 * Function: skel_ifdown
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

static int skel_ifdown(struct uip_driver_s *dev)
{
  FAR struct skel_driver_s *skel = (FAR struct skel_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = irqsave();
  up_disable_irq(CONFIG_skeleton_IRQ);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(skel->sk_txpoll);
  wd_cancel(skel->sk_txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the skel_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  skel->sk_bifup = false;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: skel_txavail
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

static int skel_txavail(struct uip_driver_s *dev)
{
  FAR struct skel_driver_s *skel = (FAR struct skel_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable interrupts because this function may be called from interrupt
   * level processing.
   */

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (skel->sk_bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */

      /* If so, then poll uIP for new XMIT data */

      (void)uip_poll(&skel->sk_dev, skel_uiptxpoll);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: skel_addmac
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
static int skel_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct skel_driver_s *skel = (FAR struct skel_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: skel_rmmac
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
static int skel_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct skel_driver_s *skel = (FAR struct skel_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: skel_initialize
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

int skel_initialize(int intf)
{
  struct skel_driver_s *priv;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(inf < CONFIG_skeleton_NINTERFACES);
  priv = &g_skel[intf];

  /* Check if a Ethernet chip is recognized at its I/O base */

  /* Attach the IRQ to the driver */

  if (irq_attach(CONFIG_skeleton_IRQ, skel_interrupt))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct skel_driver_s));
  priv->sk_dev.d_ifup    = skel_ifup;     /* I/F up (new IP address) callback */
  priv->sk_dev.d_ifdown  = skel_ifdown;   /* I/F down callback */
  priv->sk_dev.d_txavail = skel_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->sk_dev.d_addmac  = skel_addmac;   /* Add multicast MAC address */
  priv->sk_dev.d_rmmac   = skel_rmmac;    /* Remove multicast MAC address */
#endif
  priv->sk_dev.d_private = (void*)g_skel; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->sk_txpoll       = wd_create();   /* Create periodic poll timer */
  priv->sk_txtimeout    = wd_create();   /* Create TX timeout timer */

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling skel_ifdown().
   */

  /* Read the MAC address from the hardware into priv->sk_dev.d_mac.ether_addr_octet */

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->sk_dev);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_skeleton */
