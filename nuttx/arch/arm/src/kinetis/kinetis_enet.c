/****************************************************************************
 * drivers/net/kinetis_enet.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_KINETIS_ENET)

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <wdog.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/net/mii.h>

#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arp.h>
#include <nuttx/net/uip/uip-arch.h>

#include "up_arch.h"
#include "chip.h"
#include "kinetis_internal.h"
#include "kinetis_config.h"
#include "kinetis_pinmux.h"
#include "kinetis_sim.h"
#include "kinetis_mpu.h"
#include "kinetis_enet.h"

#if defined(KINETIS_NENET) && KINETIS_NENET > 0

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* CONFIG_ENET_NETHIFS determines the number of physical interfaces
 * that will be supported.
 */

#if CONFIG_ENET_NETHIFS != 1
#  error "CONFIG_ENET_NETHIFS must be one for now"
#endif

#if CONFIG_ENET_NTXBUFFERS < 1
#  error "Need at least one TX buffer"
#endif

#if CONFIG_ENET_NRXBUFFERS < 1
#  error "Need at least one RX buffer"
#endif

#define NENET_NBUFFERS (CONFIG_ENET_NTXBUFFERS+CONFIG_ENET_NRXBUFFERS)

#ifndef CONFIG_NET_MULTIBUFFER
#  errror "CONFIG_NET_MULTIBUFFER is required in the configuration"
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define KINETIS_WDDELAY   (1*CLK_TCK)
#define KINETIS_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define KINETIS_TXTIMEOUT (60*CLK_TCK)
#define MII_MAXPOLLS      (0x1ffff)
#define LINK_WAITUS       (500*1000)

/* PHY hardware specifics.  This was copied from the FreeScale code examples.
 * this is a vendor specific register and bit settings.  I really should
 * do the research and find out what this really is.
 */

#define PHY_STATUS         (0x1f)
#define PHY_DUPLEX_STATUS  (4 << 2)
#define PHY_SPEED_STATUS   (1 << 2)

/* Estimate the hold time to use based on the peripheral (bus) clock:
 *
 *   HOLD_TIME = (2*BUS_FREQ_MHZ)/5 + 1
 *             = (BUS_FREQ)/2500000 + 1
 *
 * For example, if BUS_FREQ_MHZ=48 (MHz):
 *
 *   HOLD_TIME = 48Mhz, hold time clocks
 *             = 48000000/2500000 + 1
 *             = 20
 */

#define KINETIS_MII_SPEED  (BOARD_BUS_FREQ/2500000 + 1)
#if KINETIS_MII_SPEED > 63
#  error "KINETIS_MII_SPEED is out-of-range"
#endif

/* Interrupt groups */

#define RX_INTERRUPTS     (ENET_INT_RXF | ENET_INT_RXB)
#define TX_INTERRUPTS      ENET_INT_TXF
#define ERROR_INTERRUPTS  (ENET_INT_UN    | ENET_INT_RL   | ENET_INT_LC | \
                           ENET_INT_EBERR | ENET_INT_BABT | ENET_INT_BABR)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* EMAC statistics (debug only) */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
struct kinetis_statistics_s
{
  uint32_t rx_packets;     /* Number of packets received */
  uint32_t tx_packets;     /* Number of Tx packets queued */
  unit32_t tx_done;        /* Number of packets completed */
  uint32_t tx_timeouts;    /* Number of Tx timeout errors */
  uint32_t errors;         /* Number of error interrupts */
};
#  define EMAC_STAT(priv,name) priv->stats.name++
#else
#  define EMAC_STAT(priv,name)
#endif

/* The kinetis_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct kinetis_driver_s
{
  bool bifup;                  /* true:ifup false:ifdown */
  uint8_t txtail;              /* The oldest busy TX descriptor */
  uint8_t txhead;              /* The next TX descriptor to use */
  uint8_t rxtail;              /* The next RX descriptor to use */
  WDOG_ID txpoll;              /* TX poll timer */
  WDOG_ID txtimeout;           /* TX timeout timer */
  struct enet_desc_s *txdesc;  /* A pointer to the list of TX descriptor */
  struct enet_desc_s *rxdesc;  /* A pointer to the list of RX descriptors */

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s dev;     /* Interface understood by uIP */

  /* Statistics */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
  struct kinetis_statistics_s stats;
#endif

  /* The DMA descriptors.  A unaligned uint8_t is used to allocate the
   * memory; 16 is added to assure that we can meet the desciptor alignment
   * requirements.
   */

 uint8_t desc[NENET_NBUFFERS * sizeof(struct enet_desc_s) + 16];

  /* The DMA buffers.  Again, A unaligned uint8_t is used to allocate the
   * memory; 16 is added to assure that we can meet the desciptor alignment
   * requirements.
   */

  uint8_t buffers[NENET_NBUFFERS * CONFIG_NET_BUFSIZE + 16];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct kinetis_driver_s g_enet[CONFIG_ENET_NETHIFS];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Utility functions */

#ifdef CONFIG_ENDIAN_BIG
#  define kinesis_swap32(value) (value)
#  define kinesis_swap16(value) (value)
#else
static inline uint32_t kinesis_swap32(uint32_t value);
static inline uint16_t kinesis_swap16(uint16_t value);
#endif

/* Common TX logic */

static bool kinetics_txringfull(FAR struct kinetis_driver_s *priv);
static int  kinetis_transmit(FAR struct kinetis_driver_s *priv);
static int  kinetis_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static void kinetis_receive(FAR struct kinetis_driver_s *priv);
static void kinetis_txdone(FAR struct kinetis_driver_s *priv);
static int  kinetis_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void kinetis_polltimer(int argc, uint32_t arg, ...);
static void kinetis_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int kinetis_ifup(struct uip_driver_s *dev);
static int kinetis_ifdown(struct uip_driver_s *dev);
static int kinetis_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int kinetis_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
static int kinetis_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
#endif

/* PHY/MII support */

static inline void kinetis_initmii(struct kinetis_driver_s *priv);
static inline void kinetis_initphy(struct kinetis_driver_s *priv);

/* Initialization */

static void kinetis_initbuffers(struct kinetis_driver_s *priv);
static void kinetis_reset(struct kinetis_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: kinetis_swap16/32
 *
 * Description:
 *   The descriptors are represented by structures  Unfortunately, when the
 *   structures are overlayed on the data, the bytes are reversed because
 *   the underlying hardware writes the data in big-endian byte order.
 *
 * Parameters:
 *   value  - The value to be byte swapped
 *
 * Returned Value:
 *   The byte swapped value
 *
 ****************************************************************************/

#ifndef CONFIG_ENDIAN_BIG
static inline uint32_t kinesis_swap32(uint32_t value)
{
  uint32_t result = 0;

  __asm__ __volatile__
  (
    "rev %0, %1"
    :"=r" (result)
    : "r"(value)
  );
  return result;
}

static inline uint16_t kinesis_swap16(uint16_t value)
{
  uint16_t result = 0;

  __asm__ __volatile__
  (
    "revsh %0, %1"
    :"=r" (result)
    : "r"(value)
  );
  return result;
}
#endif

/****************************************************************************
 * Function: kinetics_txringfull
 *
 * Description:
 *   Check if all of the TX descriptors are in use.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   true is the TX ring is full; false if there are free slots at the
 *   head index.
 *
 ****************************************************************************/

static bool kinetics_txringfull(FAR struct kinetis_driver_s *priv)
{
  uint8_t txnext;

  /* Check if there is room in the hardware to hold another outgoing
   * packet.  The ring is full if incrementing the head pointer would
   * collide with the tail pointer.
   */

  txnext = priv->txhead + 1;
  if (txnext > CONFIG_ENET_NTXBUFFERS)
    {
      txnext = 0;
    }

  return priv->txtail == txnext;
}

/****************************************************************************
 * Function: kinetis_transmit
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

static int kinetis_transmit(FAR struct kinetis_driver_s *priv)
{
  struct enet_desc_s *txdesc;
  uint32_t regval;

  /* When we get here the TX descriptor should show that the previous
   * transfer has  completed.  If we get here, then we are committed to
   * sending a packet; Higher level logic must have assured that there is
   * no transmission in progress.
   */

  txdesc = &priv->txdesc[priv->txhead];
  priv->txhead++;
  if (priv->txhead > CONFIG_ENET_NTXBUFFERS)
    {
      priv->txhead = 0;
    }

  DEBUGASSERT(priv->txtail != priv->txhead &&
             (txdesc->status1 & TXDESC_R) == 0);

  /* Increment statistics */

  EMAC_STAT(priv, tx_packets);

  /* Setup the buffer descriptor for transmission: address=priv->dev.d_buf,
   * length=priv->dev.d_len
   */

  txdesc->length  = kinesis_swap16(priv->dev.d_len);
#ifdef CONFIG_ENET_ENHANCEDBD
  txdesc->bdu     = 0x00000000;
  txdesc->status2 = TXDESC_INT | TXDESC_TS; // | TXDESC_IINS | TXDESC_PINS;
#endif
  txdesc->status1 = (TXDESC_R | TXDESC_L | TXDESC_TC | TXDESC_W);

  /* Start the TX transfer (if it was not already waiting for buffers) */

  putreg32(ENET_TDAR, KINETIS_ENET_TDAR);

  /* Enable TX interrupts */

  regval  = getreg32(KINETIS_ENET_EIMR);
  regval |= TX_INTERRUPTS;
  putreg32(regval, KINETIS_ENET_EIMR);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->txtimeout, KINETIS_TXTIMEOUT, kinetis_txtimeout, 1, (uint32_t)priv);
  return OK;
}

/****************************************************************************
 * Function: kinetis_uiptxpoll
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

static int kinetis_uiptxpoll(struct uip_driver_s *dev)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      uip_arp_out(&priv->dev);
      kinetis_transmit(priv);

      /* Check if there is room in the device to hold another packet. If not,
       * return a non-zero value to terminate the poll.
       */

      if (kinetics_txringfull(priv))
        {
           return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: kinetis_receive
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

static void kinetis_receive(FAR struct kinetis_driver_s *priv)
{
  /* Loop while there are received packets to be processed */

  while ((priv->rxdesc[priv->rxtail].status1 & RXDESC_E) != 0)
    {
      /* Update statistics */

      EMAC_STAT(priv, rx_packets);

      /* Copy the buffer pointer to priv->dev.d_buf.  Set amount of data in
       * priv->dev.d_len
       */

      priv->dev.d_len = kinesis_swap16(priv->rxdesc[priv->rxtail].length);
      priv->dev.d_buf = (uint8_t*)kinesis_swap32((uint32_t)priv->rxdesc[priv->rxtail].data);

      /* Doing this here could cause corruption! */

      priv->rxdesc[priv->rxtail].status1 |= RXDESC_E;

      /* Update the index to the next descriptor */

      priv->rxtail++;
      if (priv->rxtail >= CONFIG_ENET_NTXBUFFERS)
        {
          priv->rxtail = 0;
        }

      /* Indicate that there have been empty receive buffers produced */

      putreg32(ENET_RDAR, KINETIS_ENET_RDAR);

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
      if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
        {
          uip_arp_ipin(&priv->dev);
          uip_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              uip_arp_out(&priv->dev);
              kinetis_transmit(priv);
            }
        }
      else if (BUF->type == htons(UIP_ETHTYPE_ARP))
        {
          uip_arp_arpin(&priv->dev);

          /* If the above function invocation resulted in data that should
           * be sent out on the network, the field  d_len will set to a
           * value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              kinetis_transmit(priv);
            }
        }
    }
}

/****************************************************************************
 * Function: kinetis_txdone
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

static void kinetis_txdone(FAR struct kinetis_driver_s *priv)
{
  struct enet_desc_s *txdesc;
  uint32_t regval;

  /* Verify that the oldest descriptor descriptor completed */

  txdesc = &priv->txdesc[priv->txtail];
  if ((txdesc->status1 & TXDESC_R) == 0)
    {
      /* Yes.. bump up the tail pointer, making space for a new TX descriptor */

      priv->txtail++;
      if (priv->txtail > CONFIG_ENET_NTXBUFFERS)
        {
          priv->txtail = 0;
        }

        /* Update statistics */

      EMAC_STAT(priv, tx_done);
    }

  /* Are there other transmissions queued? */

  if (priv->txtail == priv->txhead)
    {
      /* No.. Cancel the TX timeout and disable further Tx interrupts. */

      wd_cancel(priv->txtimeout);

      regval  = getreg32(KINETIS_ENET_EIMR);
      regval &= ~TX_INTERRUPTS;
      putreg32(regval, KINETIS_ENET_EIMR);
    }

  /* There should be space for a new TX in any event.  Poll uIP for new XMIT
   * data
   */

  (void)uip_poll(&priv->dev, kinetis_uiptxpoll);
}

/****************************************************************************
 * Function: kinetis_interrupt
 *
 * Description:
 *   Three interrupt sources will vector this this function:
 *   1. Ethernet MAC transmit interrupt handler
 *   2. Ethernet MAC receive interrupt handler
 *   3. 
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

static int kinetis_interrupt(int irq, FAR void *context)
{
  register FAR struct kinetis_driver_s *priv = &g_enet[0];
  uint32_t pending;

  /* Get the set of unmasked, pending interrupt. */

  pending = getreg32(KINETIS_ENET_EIR) & getreg32(KINETIS_ENET_EIMR);

  /* Clear the pending interrupts */

  putreg32(pending, KINETIS_ENET_EIR);

  /* Check for the receipt of a packet */

  if ((pending & ENET_INT_RXF) != 0)
    {
      /* A packet has been received, call kinetis_receive() to handle the packet */

      kinetis_receive(priv);
    }

  /* Check if a packet transmission has completed */

  if ((pending & ENET_INT_TXF) != 0)
    {
      /* Call kinetis_txdone to handle the end of transfer even.  NOTE that
       * this may disable further Tx interrupts if there are no pending
       * tansmissions.
       */

      kinetis_txdone(priv);
    }

  /* Check for errors */

  if (pending & ERROR_INTERRUPTS)
    {
      /* An error has occurred, update statistics */

      EMAC_STAT(priv, errors);

      /* Reinitialize all buffers. */

      kinetis_initbuffers(priv);
 
      /* Indicate that there have been empty receive buffers produced */

      putreg32(ENET_RDAR, KINETIS_ENET_RDAR);
    }

  return OK;
}

/****************************************************************************
 * Function: kinetis_txtimeout
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

static void kinetis_txtimeout(int argc, uint32_t arg, ...)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)arg;

  /* Increment statistics and dump debug info */

  EMAC_STAT(priv, tx_timeout);

  /* Take the interface down and bring it back up.  The is the most agressive
   * hardware reset.
   */

  (void)kinetis_ifup(&priv->dev);
  (void)kinetis_ifdown(&priv->dev);

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->dev, kinetis_uiptxpoll);
}

/****************************************************************************
 * Function: kinetis_polltimer
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

static void kinetis_polltimer(int argc, uint32_t arg, ...)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)arg;

  /* Check if there is there is a transmission in progress.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  if (!kinetics_txringfull(priv))
    {
      /* If so, update TCP timing states and poll uIP for new XMIT data. Hmmm..
       * might be bug here.  Does this mean if there is a transmit in progress,
       * we will missing TCP time state updates?
       */

      (void)uip_timer(&priv->dev, kinetis_uiptxpoll, KINETIS_POLLHSEC);
    }

  /* Setup the watchdog poll timer again in any case */

  (void)wd_start(priv->txpoll, KINETIS_WDDELAY, kinetis_polltimer, 1, arg);
}

/****************************************************************************
 * Function: kinetis_ifup
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

static int kinetis_ifup(struct uip_driver_s *dev)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)dev->d_private;
  uint32_t regval;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Initialize ENET buffers */

  kinetis_initbuffers(priv);

  /* Reset and disable the interface */

  kinetis_reset(priv);

  /* Configure the MII interface */

  kinetis_initmii(priv);

  /* Clear the Individual and Group Address Hash registers */

  putreg32(0, KINETIS_ENET_IALR);
  putreg32(0, KINETIS_ENET_IAUR);
  putreg32(0, KINETIS_ENET_GALR);
  putreg32(0, KINETIS_ENET_GAUR);

  /* Configure the PHY */

  kinetis_initphy(priv);

  /* Handle promiscuous mode */

#ifdef CONFIG_NET_PROMISCUOUS
  regval = getreg32(KINETIS_ENET_RCR);
  regval |= ENET_RCR_PROM;
  putreg32(regval, KINETIS_ENET_RCR);
#endif

  /* Select legacy of enhanced buffer descriptor format */

#ifdef CONFIG_ENET_ENHANCEDBD
  putreg32(ENET_ECR_EN1588, KINETIS_ENET_ECR);
#else
  putreg32(0, KINETIS_ENET_ECR);
#endif

  /* Set the RX buffer size */

  putreg32(CONFIG_NET_BUFSIZE, KINETIS_ENET_MRBR);

  /* Point to the start of the circular RX buffer descriptor queue */

  putreg32((uint32_t)priv->rxdesc, KINETIS_ENET_RDSR);

  /* Point to the start of the circular TX buffer descriptor queue */

  putreg32((uint32_t)priv->txdesc, KINETIS_ENET_TDSR);

  /* Indicate that there have been empty receive buffers produced */

  putreg32(ENET_RDAR, KINETIS_ENET_RDAR);

  /* And enable the MAC itself */

  regval = getreg32(KINETIS_ENET_ECR);
  regval |= ENET_ECR_ETHEREN;
  putreg32(regval, KINETIS_ENET_ECR);

  /* Set and activate a timer process */

  (void)wd_start(priv->txpoll, KINETIS_WDDELAY, kinetis_polltimer, 1, (uint32_t)priv);

  /* Clear all pending ENET interrupt */

  putreg32(0xffffffff, KINETIS_ENET_EIR);

  /* Enable RX and error interrupts at the controller (TX interrupts are
   * still disabled).
   */

  putreg32(RX_INTERRUPTS | ERROR_INTERRUPTS,
           KINETIS_ENET_EIMR);

  /* Mark the interrupt "up" and enable interrupts at the NVIC */

  priv->bifup = true;
#if 0
  up_enable_irq(KINETIS_IRQ_EMACTMR);
#endif
  up_enable_irq(KINETIS_IRQ_EMACTX);
  up_enable_irq(KINETIS_IRQ_EMACRX);
  up_enable_irq(KINETIS_IRQ_EMACMISC);
  return OK;
}

/****************************************************************************
 * Function: kinetis_ifdown
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

static int kinetis_ifdown(struct uip_driver_s *dev)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupts at the NVIC */

  flags = irqsave();
  up_disable_irq(KINETIS_IRQ_EMACTMR);
  up_disable_irq(KINETIS_IRQ_EMACTX);
  up_disable_irq(KINETIS_IRQ_EMACRX);
  up_disable_irq(KINETIS_IRQ_EMACMISC);
  putreg32(0, KINETIS_ENET_EIMR);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->txpoll);
  wd_cancel(priv->txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the kinetis_ifup() always
   * successfully brings the interface back up.
   */

  kinetis_reset(priv);

  /* Mark the device "down" */

  priv->bifup = false;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: kinetis_txavail
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

static int kinetis_txavail(struct uip_driver_s *dev)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable interrupts because this function may be called from interrupt
   * level processing.
   */

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

       if (!kinetics_txringfull(priv))
         {
           /* No, there is space for another transfer.  Poll uIP for new
            * XMIT data.
            */

           (void)uip_poll(&priv->dev, kinetis_uiptxpoll);
        }
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: kinetis_addmac
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
static int kinetis_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: kinetis_rmmac
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
static int kinetis_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct kinetis_driver_s *priv = (FAR struct kinetis_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: kinetis_initmii
 *
 * Description:
 *   Configure the MII interface
 *
 * Parameters:
 *   priv - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void kinetis_initmii(struct kinetis_driver_s *priv)
{
  /* Speed is based on the peripheral (bus) clock; hold time is 1 module
   * clock.  This hold time value may need to be increased on some platforms
   */

  putreg32(ENET_MSCR_HOLDTIME_1CYCLE |
           KINETIS_MII_SPEED << ENET_MSCR_MII_SPEED_SHIFT,
           KINETIS_ENET_MSCR);
}

/****************************************************************************
 * Function: kinetis_writemii
 *
 * Description:
 *   Write a 16-bit value to a PHY register. 
 *
 * Parameters:
 *   priv - Reference to the private ENET driver state structure
 *   phyaddr - The PHY address
 *   regaddr - The PHY register address
 *   data    - The data to write to the PHY register
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int kinetis_writemii(struct kinetis_driver_s *priv, uint8_t phyaddr,
                            uint8_t regaddr, uint16_t data)
{
  int timeout;

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, KINETIS_ENET_EIR);

  /* Initiatate the MII Management write */

  putreg32(data |
           2 << ENET_MMFR_TA_SHIFT |
           (uint32_t)regaddr << ENET_MMFR_PA_SHIFT |
           (uint32_t)phyaddr << ENET_MMFR_PA_SHIFT |
           ENET_MMFR_OP_WRMII |
           1 << ENET_MMFR_ST_SHIFT,
           KINETIS_ENET_MMFR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((getreg32(KINETIS_ENET_EIR) & ENET_INT_MII) != 0)
        {
          break;
        }
    }

  /* Check for a timeout */

  if(timeout == MII_MAXPOLLS) 
    {
      return -ETIMEDOUT;
    }

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, KINETIS_ENET_EIR);
  return OK;
}

/****************************************************************************
 * Function: kinetis_writemii
 *
 * Description:
 *   Read a 16-bit value from a PHY register. 
 *
 * Parameters:
 *   priv    - Reference to the private ENET driver state structure
 *   phyaddr - The PHY address
 *   regaddr - The PHY register address
 *   data    - A pointer to the location to return the data
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int kinetis_readmii(struct kinetis_driver_s *priv, uint8_t phyaddr,
                           uint8_t regaddr, uint16_t *data)
{
  int timeout;

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, KINETIS_ENET_EIR);

  /* Initiatate the MII Management read */
 
  putreg32(2 << ENET_MMFR_TA_SHIFT |
           (uint32_t)regaddr << ENET_MMFR_PA_SHIFT |
           (uint32_t)phyaddr << ENET_MMFR_PA_SHIFT |
           ENET_MMFR_OP_RDMII |
           1 << ENET_MMFR_ST_SHIFT,
           KINETIS_ENET_MMFR);

  /* Wait for the transfer to complete */

  for (timeout = 0; timeout < MII_MAXPOLLS; timeout++)
    {
      if ((getreg32(KINETIS_ENET_EIR) & ENET_INT_MII) != 0)
        {
          break;
        }
    }

  /* Check for a timeout */

  if (timeout >= MII_MAXPOLLS) 
    {
      return -ETIMEDOUT;
    }

  /* Clear the MII interrupt bit */

  putreg32(ENET_INT_MII, KINETIS_ENET_EIR);

  /* And return the MII data */

  *data = (uint16_t)(getreg32(KINETIS_ENET_MMFR) & ENET_MMFR_DATA_MASK);
  return OK;
}

/****************************************************************************
 * Function: kinetis_initphy
 *
 * Description:
 *   Configure the PHY
 *
 * Parameters:
 *   priv - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void kinetis_initphy(struct kinetis_driver_s *priv)
{
  uint32_t rcr;
  uint32_t tcr;
  uint16_t phydata;

  /* Loop (potentially infinitely?) until we successfully communicate with
   * the PHY.
   */

  do
    {
      usleep(LINK_WAITUS);
      phydata = 0xffff;
      kinetis_readmii(priv, CONFIG_ENET_PHYADDR, MII_PHYID1, &phydata);
    }
  while (phydata == 0xffff);

  /* Start auto negotiation */

  kinetis_writemii(priv, CONFIG_ENET_PHYADDR, MII_MCR,
                  (MII_MCR_ANRESTART | MII_MCR_ANENABLE));

  /* Wait (potentially forever) for auto negotiation to complete */

  do
    {
      usleep(LINK_WAITUS);
      kinetis_readmii(priv, CONFIG_ENET_PHYADDR, MII_MSR, &phydata);

    }
  while ((phydata & MII_MSR_ANEGCOMPLETE) == 0);

  /* When we get here we have a link - Find the negotiated speed and duplex. */

  phydata = 0;
  kinetis_readmii(priv, CONFIG_ENET_PHYADDR, PHY_STATUS, &phydata);

  /* Set up the transmit and receive contrel registers based on the 
   * configuration and the auto negotiation results.
   */

#if CONFIG_ENET_USEMII
  rcr = ENET_RCR_MII_MODE | ENET_RCR_CRCFWD |
        CONFIG_NET_BUFSIZE << ENET_RCR_MAX_FL_SHIFT;
#else
  rcr = ENET_RCR_RMII_MODE | ENET_RCR_CRCFWD |
        CONFIG_NET_BUFSIZE << ENET_RCR_MAX_FL_SHIFT;
#endif
  tcr = 0;

  putreg32(rcr, KINETIS_ENET_RCR);
  putreg32(tcr, KINETIS_ENET_TCR);
  
  /* Setup half or full duplex */

  if ((phydata & PHY_DUPLEX_STATUS) != 0)
    {
      /* Full duplex */

      tcr |= ENET_TCR_FDEN;
    }
  else
    {
      /* Half duplex */

      rcr |= ENET_RCR_DRT;
    }
 
  if ((phydata & PHY_SPEED_STATUS) != 0)
    {
      /* 10Mbps */

      rcr |= ENET_RCR_RMII_10T;
    }

  putreg32(rcr, KINETIS_ENET_RCR);
  putreg32(tcr, KINETIS_ENET_TCR);
}

/****************************************************************************
 * Function: kinetis_initbuffers
 *
 * Description:
 *   Initialize ENET buffers and descriptors
 *
 * Parameters:
 *   priv - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void kinetis_initbuffers(struct kinetis_driver_s *priv)
{
  uintptr_t addr;
  int i;

  /* Get an aligned TX descriptor (array)address */

  addr         = ((uintptr_t)priv->desc + 0x0f) & ~0x0f;
  priv->txdesc = (struct enet_desc_s *)addr;

  /* Get an aligned RX descriptor (array) address */

  addr        +=  CONFIG_ENET_NTXBUFFERS * sizeof(struct enet_desc_s);
  priv->txdesc = (struct enet_desc_s *)addr;

  /* Get the beginning of the first aligned buffer */

  addr        = ((uintptr_t)priv->buffers + 0x0f) & ~0x0f;

  /* Then fill in the TX descriptors */

  for (i = 0; i < CONFIG_ENET_NTXBUFFERS; i++)
    {
      priv->txdesc[i].status1 = 0;
      priv->txdesc[i].length  = 0;
      priv->txdesc[i].data    = (uint8_t*)kinesis_swap32((uint32_t)addr);
#ifdef CONFIG_ENET_ENHANCEDBD
      priv->txdesc[i].status2 = TXDESC_IINS | TXDESC_PINS;
#endif
      addr                   += CONFIG_NET_BUFSIZE;
    }

  /* Then fill in the RX descriptors */

  for (i = 0; i < CONFIG_ENET_NRXBUFFERS; i++)
    {
      priv->rxdesc[i].status1 = RXDESC_E;
      priv->rxdesc[i].length  = 0;
      priv->rxdesc[i].data    = (uint8_t*)kinesis_swap32((uint32_t)addr);
#ifdef CONFIG_ENET_ENHANCEDBD
      priv->rxdesc[i].bdu     = 0;
      priv->rxdesc[i].status2 = RXDESC_INT;
#endif
      addr                   += CONFIG_NET_BUFSIZE;
    }

  /* Set the wrap bit in the last descriptors to form a ring */

  priv->txdesc[CONFIG_ENET_NTXBUFFERS-1].status1 |= TXDESC_W;
  priv->rxdesc[CONFIG_ENET_NRXBUFFERS-1].status1 |= RXDESC_W;

  /* We start with RX descriptor 0 and with no TX descriptors in use */

  priv->txtail = 0;
  priv->txhead = 0;
  priv->rxtail = 0;
}

/****************************************************************************
 * Function: kinetis_reset
 *
 * Description:
 *   Put the EMAC in the non-operational, reset state
 *
 * Parameters:
 *   priv - Reference to the private ENET driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void kinetis_reset(struct kinetis_driver_s *priv)
{
  unsigned int i;

  /* Set the reset bit and clear the enable bit */

  putreg32(ENET_ECR_RESET, KINETIS_ENET_ECR);

  /* Wait at least 8 clock cycles */

  for (i = 0; i < 10; i++)
    {
      asm volatile ("nop");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: kinetis_netinitialize
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

int kinetis_netinitialize(int intf)
{
  struct kinetis_driver_s *priv;
  uint32_t regval;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(inf < CONFIG_ENET_NETHIFS);
  priv = &g_enet[intf];

  /* Enable the ENET clock */

  regval  = getreg32(KINETIS_SIM_SCGC2);
  regval |= SIM_SCGC2_ENET;
  putreg32(regval, KINETIS_SIM_SCGC2);

  /* Allow concurrent access to MPU controller. Example: ENET uDMA to SRAM,
   * otherwise a bus error will result.
   */

  putreg32(0, KINETIS_MPU_CESR);

  /* Configure all ENET/MII pins */

#if CONFIG_ENET_USEMII
  kinetis_pinconfig(PIN_MII0_MDIO);
  kinetis_pinconfig(PIN_MII0_MDC);
  kinetis_pinconfig(PIN_MII0_RXDV);
  kinetis_pinconfig(PIN_MII0_RXER);
  kinetis_pinconfig(PIN_MII0_TXER);
  kinetis_pinconfig(PIN_MII0_RXD0);
  kinetis_pinconfig(PIN_MII0_RXD1);
  kinetis_pinconfig(PIN_MII0_RXD2);
  kinetis_pinconfig(PIN_MII0_RXD3);
  kinetis_pinconfig(PIN_MII0_TXD0);
  kinetis_pinconfig(PIN_MII0_TXD1);
  kinetis_pinconfig(PIN_MII0_TXD3);
  kinetis_pinconfig(PIN_MII0_TXD2);
  kinetis_pinconfig(PIN_MII0_TXEN);
  kinetis_pinconfig(PIN_MII0_RXCLK);
  kinetis_pinconfig(PIN_MII0_TXCLK);
  kinetis_pinconfig(PIN_MII0_CRS);
  kinetis_pinconfig(PIN_MII0_COL);
#else
  kinetis_pinconfig(PIN_RMII0_MDIO);
  kinetis_pinconfig(PIN_RMII0_MDC);
  kinetis_pinconfig(PIN_RMII0_CRS_DV);
  kinetis_pinconfig(PIN_RMII0_RXER);
  kinetis_pinconfig(PIN_RMII0_RXD0);
  kinetis_pinconfig(PIN_RMII0_RXD1);
  kinetis_pinconfig(PIN_RMII0_TXD0);
  kinetis_pinconfig(PIN_RMII0_TXD1);
  kinetis_pinconfig(PIN_RMII0_TXEN);
#endif   

  /* Set interrupt priority levels */

  up_prioritize_irq(KINETIS_IRQ_EMACTMR, CONFIG_KINETIS_EMACTMR_PRIO);
  up_prioritize_irq(KINETIS_IRQ_EMACTX, CONFIG_KINETIS_EMACTX_PRIO);
  up_prioritize_irq(KINETIS_IRQ_EMACRX, CONFIG_KINETIS_EMACRX_PRIO);
  up_prioritize_irq(KINETIS_IRQ_EMACMISC, CONFIG_KINETIS_EMACMISC_PRIO);

  /* Attach the Ethernet MAC IEEE 1588 timer interrupt handler */

#if 0
  if (irq_attach(KINETIS_IRQ_EMACTMR, kinetis_tmrinterrupt))
    {
      /* We could not attach the ISR to the interrupt */

      ndbg("Failed to attach EMACTMR IRQ\n");
      return -EAGAIN;
    }
#endif

  /* Attach the Ethernet MAC transmit interrupt handler */

  if (irq_attach(KINETIS_IRQ_EMACTX, kinetis_interrupt))
    {
      /* We could not attach the ISR to the interrupt */

      ndbg("Failed to attach EMACTX IRQ\n");
      return -EAGAIN;
    }

  /* Attach the Ethernet MAC receive interrupt handler */

  if (irq_attach(KINETIS_IRQ_EMACRX, kinetis_interrupt))
    {
      /* We could not attach the ISR to the interrupt */

      ndbg("Failed to attach EMACRX IRQ\n");
      return -EAGAIN;
    }

  /* Attach the Ethernet MAC error and misc interrupt handler */

  if (irq_attach(KINETIS_IRQ_EMACMISC, kinetis_interrupt))
    {
      /* We could not attach the ISR to the interrupt */

      ndbg("Failed to attach EMACMISC IRQ\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct kinetis_driver_s));
  priv->dev.d_ifup    = kinetis_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = kinetis_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = kinetis_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->dev.d_addmac  = kinetis_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = kinetis_rmmac;    /* Remove multicast MAC address */
#endif
  priv->dev.d_private = (void*)g_enet;    /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->txpoll        = wd_create();      /* Create periodic poll timer */
  priv->txtimeout     = wd_create();      /* Create TX timeout timer */

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling kinetis_ifdown().
   */

  (void)kinetis_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->dev);
  return OK;
}

/****************************************************************************
 * Name: up_netinitialize
 *
 * Description:
 *   Initialize the first network interface.  If there are more than one
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, Ethernet controllers should
 *   be initialized.
 *
 ****************************************************************************/

#if CONFIG_ENET_NETHIFS == 1
void up_netinitialize(void)
{
  (void)kinetis_netinitialize(0);
}
#endif

#endif /* KINETIS_NENET > 0 */
#endif /* CONFIG_NET && CONFIG_KINETIS_ENET */
