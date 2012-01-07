/****************************************************************************
 * arch/arm/src/pic32mx/pic32mx_ethernet.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This driver derives from the PIC32MX Ethernet Driver
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
#if defined(CONFIG_NET) && defined(CONFIG_PIC32MX_ETHERNET)

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <wdog.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mii.h>

#include <net/uip/uip.h>
#include <net/uip/uipopt.h>
#include <net/uip/uip-arp.h>
#include <net/uip/uip-arch.h>

#include "chip.h"
#include "up_arch.h"
#include "pic32mx-ethernet.h"
#include "pic32mx-internal.h"

#include <arch/board/board.h>

/* Does this chip have and ethernet controller? */

#if CHIP_NETHERNET > 0

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* CONFIG_PIC32MX_NINTERFACES determines the number of physical interfaces
 * that will be supported -- unless it is more than actually supported by the
 * hardware!
 */

#if !defined(CONFIG_PIC32MX_NINTERFACES) || CONFIG_PIC32MX_NINTERFACES > CHIP_NETHERNET
#  undef CONFIG_PIC32MX_NINTERFACES
#  define CONFIG_PIC32MX_NINTERFACES CHIP_NETHERNET
#endif

/* The logic here has a few hooks for support for multiple interfaces, but
 * that capability is not yet in place (and I won't worry about it until I get
 * the first multi-interface PIC32MX).
 */

#if CONFIG_PIC32MX_NINTERFACES > 1
#  warning "Only a single ethernet controller is supported"
#  undef CONFIG_PIC32MX_NINTERFACES
#  define CONFIG_PIC32MX_NINTERFACES 1
#endif

/* If IGMP is enabled, then accept multi-cast frames. */

#if defined(CONFIG_NET_IGMP) && !defined(CONFIG_NET_MULTICAST)
#  define CONFIG_NET_MULTICAST 1
#endif

/* If the user did not specify a priority for Ethernet interrupts, set the
 * interrupt priority to the maximum.
 */

#ifndef CONFIG_NET_PRIORITY
#  define CONFIG_NET_PRIORITY NVIC_SYSH_PRIORITY_MAX
#endif

/* Debug Configuration *****************************************************/
/* Register debug -- can only happen of CONFIG_DEBUG is selected */

#ifndef CONFIG_DEBUG
#  undef  CONFIG_NET_REGDEBUG
#endif

/* CONFIG_NET_DUMPPACKET will dump the contents of each packet to the
 * console.
 */

#ifndef CONFIG_DEBUG
#  undef  CONFIG_NET_DUMPPACKET
#endif

#ifdef CONFIG_NET_DUMPPACKET
#  define pic32mx_dumppacket(m,a,n) lib_dumpbuffer(m,a,n)
#else
#  define pic32mx_dumppacket(m,a,n)
#endif

/* Timing *******************************************************************/

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define PIC32MX_WDDELAY        (1*CLK_TCK)
#define PIC32MX_POLLHSEC       (1*2)

/* TX timeout = 1 minute */

#define PIC32MX_TXTIMEOUT      (60*CLK_TCK)

/* Interrupts ***************************************************************/

#define ETH_RXINTS           (ETH_INT_RXOVR | ETH_INT_RXERR | ETH_INT_RXFIN | ETH_INT_RXDONE)
#define ETH_TXINTS           (ETH_INT_TXUNR | ETH_INT_TXERR | ETH_INT_TXFIN | ETH_INT_TXDONE)

/* Misc. Helpers ***********************************************************/

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)priv->pd_dev.d_buf)

/* This is the number of ethernet GPIO pins that must be configured */

#define GPIO_NENET_PINS      10

/* PHYs *********************************************************************/
/* Select PHY-specific values.  Add more PHYs as needed. */

#if defined(CONFIG_PHY_KS8721)
#  define PIC32MX_PHYNAME      "KS8721"
#  define PIC32MX_PHYID1       MII_PHYID1_KS8721
#  define PIC32MX_PHYID2       MII_PHYID2_KS8721
#  define PIC32MX_HAVE_PHY     1
#elif defined(CONFIG_PHY_DP83848C)
#  define PIC32MX_PHYNAME      "DP83848C"
#  define PIC32MX_PHYID1       MII_PHYID1_DP83848C
#  define PIC32MX_PHYID2       MII_PHYID2_DP83848C
#  define PIC32MX_HAVE_PHY     1
#elif defined(CONFIG_PHY_LAN8720)
#  define PIC32MX_PHYNAME      "LAN8720"
#  define PIC32MX_PHYID1       MII_PHYID1_LAN8720
#  define PIC32MX_PHYID2       MII_PHYID2_LAN8720
#  define PIC32MX_HAVE_PHY     1
#else
#  warning "No PHY specified!"
#  undef PIC32MX_HAVE_PHY
#endif

#define MII_BIG_TIMEOUT      666666

/* These definitions are used to remember the speed/duplex settings */

#define PIC32MX_SPEED_MASK     0x01
#define PIC32MX_SPEED_100      0x01
#define PIC32MX_SPEED_10       0x00

#define PIC32MX_DUPLEX_MASK    0x02
#define PIC32MX_DUPLEX_FULL    0x02
#define PIC32MX_DUPLEX_HALF    0x00

#define PIC32MX_10BASET_HD     (PIC32MX_SPEED_10  | PIC32MX_DUPLEX_HALF)
#define PIC32MX_10BASET_FD     (PIC32MX_SPEED_10  | PIC32MX_DUPLEX_FULL)
#define PIC32MX_100BASET_HD    (PIC32MX_SPEED_100 | PIC32MX_DUPLEX_HALF)
#define PIC32MX_100BASET_FD    (PIC32MX_SPEED_100 | PIC32MX_DUPLEX_FULL)

#ifdef CONFIG_PHY_SPEED100
#  ifdef CONFIG_PHY_FDUPLEX
#    define PIC32MX_MODE_DEFLT PIC32MX_100BASET_FD
#  else
#    define PIC32MX_MODE_DEFLT PIC32MX_100BASET_HD
#  endif
#else
#  ifdef CONFIG_PHY_FDUPLEX
#    define PIC32MX_MODE_DEFLT PIC32MX_10BASET_FD
#  else
#    define PIC32MX_MODE_DEFLT PIC32MX_10BASET_HD
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* EMAC statistics (debug only) */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
struct pic32mx_statistics_s
{
#ifdef ENABLE_WOL
  uint32_t wol;            /* Wake-up interrupts */
#endif
  uint32_t rx_finished;    /* Rx finished interrupts */
  uint32_t rx_done;        /* Rx done interrupts */
  uint32_t rx_ovrerrors;   /* Number of Rx overrun error interrupts */
  uint32_t rx_errors;      /* Number of Rx error interrupts (OR of other errors) */
  uint32_t rx_packets;     /* Number of packets received (sum of the following): */
  uint32_t rx_ip;          /*   Number of Rx IP packets received */
  uint32_t rx_arp;         /*   Number of Rx ARP packets received */
  uint32_t rx_dropped;     /*   Number of dropped, unsupported Rx packets */
  uint32_t rx_pkterr;      /*   Number of dropped, error in Rx descriptor */
  uint32_t rx_pktsize;     /*   Number of dropped, too small or too big */
  uint32_t rx_fragment;    /*   Number of dropped, packet fragments */

  uint32_t tx_packets;     /* Number of Tx packets queued */
  uint32_t tx_pending;     /* Number of Tx packets that had to wait for a TxDesc */
  uint32_t tx_unpend;      /* Number of pending Tx packets that were sent */
  uint32_t tx_finished;    /* Tx finished interrupts */
  uint32_t tx_done;        /* Tx done interrupts */
  uint32_t tx_underrun;    /* Number of Tx underrun error interrupts */
  uint32_t tx_errors;      /* Number of Tx error inerrupts (OR of other errors) */
  uint32_t tx_timeouts;    /* Number of Tx timeout errors */
};
#  define EMAC_STAT(priv,name) priv->pd_stat.name++
#else
#  define EMAC_STAT(priv,name)
#endif

/* The pic32mx_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct pic32mx_driver_s
{
  /* The following fields would only be necessary on chips that support
   * multiple Ethernet controllers.
   */

#if CONFIG_PIC32MX_NINTERFACES > 1
  uint32_t pd_base;             /* Ethernet controller base address */
  int      pd_irq;              /* Ethernet controller IRQ vector number */
  int      pd_irqsrc;           /* Ethernet controller IRQ source number */
#endif

  bool     pd_ifup;             /* true:ifup false:ifdown */
  bool     pd_mode;             /* speed/duplex */
  bool     pd_txpending;        /* There is a pending Tx in pd_dev */
#ifdef PIC32MX_HAVE_PHY
  uint8_t  pd_phyaddr;          /* PHY device address */
#endif
  uint32_t pd_inten;            /* Shadow copy of INTEN register */
  WDOG_ID  pd_txpoll;           /* TX poll timer */
  WDOG_ID  pd_txtimeout;        /* TX timeout timer */
  
#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
  struct pic32mx_statistics_s pd_stat;
#endif

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s pd_dev;  /* Interface understood by uIP */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Array of ethernet driver status structures */

static struct pic32mx_driver_s g_ethdrvr[CONFIG_PIC32MX_NINTERFACES];

/* ENET pins are on P1[0,1,4,6,8,9,10,14,15] + MDC on P1[16] or P2[8] and
 * MDIO on P1[17] or P2[9].  The board.h file will define GPIO_ENET_MDC and
 * PGIO_ENET_MDIO to selec which pin setting to use.
 *
 * On older Rev '-' devices, P1[6] ENET-TX_CLK would also have be to configured.
 */

static const uint16_t g_enetpins[GPIO_NENET_PINS] =
{
  GPIO_ENET_TXD0, GPIO_ENET_TXD1, GPIO_ENET_TXEN,   GPIO_ENET_CRS, GPIO_ENET_RXD0,
  GPIO_ENET_RXD1, GPIO_ENET_RXER, GPIO_ENET_REFCLK, GPIO_ENET_MDC, GPIO_ENET_MDIO
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations */

#ifdef CONFIG_NET_REGDEBUG
static void pic32mx_printreg(uint32_t addr, uint32_t val, bool iswrite);
static void pic32mx_checkreg(uint32_t addr, uint32_t val, bool iswrite);
static uint32_t pic32mx_getreg(uint32_t addr);
static void pic32mx_putreg(uint32_t val, uint32_t addr);
#else
# define pic32mx_getreg(addr)     getreg32(addr)
# define pic32mx_putreg(val,addr) putreg32(val,addr)
#endif

/* Common TX logic */

static int  pic32mx_txdesc(struct pic32mx_driver_s *priv);
static int  pic32mx_transmit(struct pic32mx_driver_s *priv);
static int  pic32mx_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static void pic32mx_response(struct pic32mx_driver_s *priv);
static void pic32mx_rxdone(struct pic32mx_driver_s *priv);
static void pic32mx_txdone(struct pic32mx_driver_s *priv);
static int  pic32mx_interrupt(int irq, void *context);

/* Watchdog timer expirations */

static void pic32mx_polltimer(int argc, uint32_t arg, ...);
static void pic32mx_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int pic32mx_ifup(struct uip_driver_s *dev);
static int pic32mx_ifdown(struct uip_driver_s *dev);
static int pic32mx_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int pic32mx_addmac(struct uip_driver_s *dev, const uint8_t *mac);
static int pic32mx_rmmac(struct uip_driver_s *dev, const uint8_t *mac);
#endif

/* Initialization functions */

#if defined(CONFIG_NET_REGDEBUG) && defined(CONFIG_DEBUG_GPIO)
static void pic32mx_showpins(void);
#else
#  define pic32mx_showpins()
#endif

/* PHY initialization functions */

#ifdef PIC32MX_HAVE_PHY
#  ifdef CONFIG_NET_REGDEBUG
static void pic32mx_showmii(uint8_t phyaddr, const char *msg);
#  else
#    define pic32mx_showmii(phyaddr,msg)
#  endif

static void pic32mx_phywrite(uint8_t phyaddr, uint8_t regaddr,
                           uint16_t phydata);
static uint16_t pic32mx_phyread(uint8_t phyaddr, uint8_t regaddr);
static inline int pic32mx_phyreset(uint8_t phyaddr);
#  ifdef CONFIG_PHY_AUTONEG
static inline int pic32mx_phyautoneg(uint8_t phyaddr);
#  endif
static int pic32mx_phymode(uint8_t phyaddr, uint8_t mode);
static inline int pic32mx_phyinit(struct pic32mx_driver_s *priv);
#else
#  define pic32mx_phyinit(priv)
#endif

/* EMAC Initialization functions */

static inline void pic32mx_txdescinit(struct pic32mx_driver_s *priv);
static inline void pic32mx_rxdescinit(struct pic32mx_driver_s *priv);
static void pic32mx_macmode(uint8_t mode);
static void pic32mx_ethreset(struct pic32mx_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*******************************************************************************
 * Name: pic32mx_printreg
 *
 * Description:
 *   Print the contents of an PIC32MX register operation
 *
 *******************************************************************************/

#ifdef CONFIG_NET_REGDEBUG
static void pic32mx_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  dbg("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/*******************************************************************************
 * Name: pic32mx_checkreg
 *
 * Description:
 *   Get the contents of an PIC32MX register
 *
 *******************************************************************************/

#ifdef CONFIG_NET_REGDEBUG
static void pic32mx_checkreg(uint32_t addr, uint32_t val, bool iswrite)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;
  static bool     prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register last time?
   * Are we polling the register?  If so, suppress the output.
   */

  if (addr == prevaddr && val == preval && prevwrite == iswrite)
    {
      /* Yes.. Just increment the count */

      count++;
    }
  else
    {
      /* No this is a new address or value or operation. Were there any
       * duplicate accesses before this one?
       */

      if (count > 0)
        {
          /* Yes.. Just one? */

          if (count == 1)
            {
              /* Yes.. Just one */

              pic32mx_printreg(prevaddr, preval, prevwrite);
            }
          else
            {
              /* No.. More than one. */

              dbg("[repeats %d more times]\n", count);
            }
        }

      /* Save the new address, value, count, and operation for next time */

      prevaddr  = addr;
      preval    = val;
      count     = 0;
      prevwrite = iswrite;

      /* Show the new regisgter access */

      pic32mx_printreg(addr, val, iswrite);
    }
}
#endif

/*******************************************************************************
 * Name: pic32mx_getreg
 *
 * Description:
 *   Get the contents of an PIC32MX register
 *
 *******************************************************************************/

#ifdef CONFIG_NET_REGDEBUG
static uint32_t pic32mx_getreg(uint32_t addr)
{
  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Check if we need to print this value */

  pic32mx_checkreg(addr, val, false);
  return val;
}
#endif

/*******************************************************************************
 * Name: pic32mx_putreg
 *
 * Description:
 *   Set the contents of an PIC32MX register to a value
 *
 *******************************************************************************/

#ifdef CONFIG_NET_REGDEBUG
static void pic32mx_putreg(uint32_t val, uint32_t addr)
{
  /* Check if we need to print this value */

  pic32mx_checkreg(addr, val, true);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Function: pic32mx_txdesc
 *
 * Description:
 *   Check if a free TX descriptor is available.
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

static int pic32mx_txdesc(struct pic32mx_driver_s *priv)
{
  unsigned int prodidx;
  unsigned int considx;

  /* Get the next producer index */

  prodidx = pic32mx_getreg(PIC32MX_ETH_TXPRODIDX) & ETH_TXPRODIDX_MASK;
  if (++prodidx >= CONFIG_NET_NTXDESC)
    {
     /* Wrap back to index zero */

      prodidx = 0;
    }

  /* If the next producer index would overrun the consumer index, then there
   * are no available Tx descriptors.
   */

  considx = pic32mx_getreg(PIC32MX_ETH_TXCONSIDX) & ETH_TXCONSIDX_MASK;
  return prodidx != considx ? OK : -EAGAIN;
}

/****************************************************************************
 * Function: pic32mx_transmit
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

static int pic32mx_transmit(struct pic32mx_driver_s *priv)
{
  uint32_t *txdesc;
  void     *txbuffer;
  unsigned int prodidx;

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  DEBUGASSERT(pic32mx_txdesc(priv) == OK);

  /* Increment statistics and dump the packet *if so configured) */

  EMAC_STAT(priv, tx_packets);
  pic32mx_dumppacket("Transmit packet",
                   priv->pd_dev.d_buf, priv->pd_dev.d_len);

  /* Get the current producer index */

  prodidx = pic32mx_getreg(PIC32MX_ETH_TXPRODIDX) & ETH_TXPRODIDX_MASK;

  /* Get the packet address from the descriptor and set the descriptor control
   * fields.
   */

  txdesc   = (uint32_t*)(PIC32MX_TXDESC_BASE + (prodidx << 3));
  txbuffer = (void*)*txdesc++;
  *txdesc  = TXDESC_CONTROL_INT | TXDESC_CONTROL_LAST | TXDESC_CONTROL_CRC |
             (priv->pd_dev.d_len - 1);

  /* Copy the packet data into the Tx buffer assignd to this descriptor.  It
   * should fit because each packet buffer is the MTU size and breaking up
   * largerTCP messasges is handled by higher level logic.  The hardware
   * does, however, support breaking up larger messages into many fragments,
   * however, that capability is not exploited here.
   *
   * This would be a great performance improvement:  Remove the buffer from
   * the pd_dev structure and replace it a pointer directly into the EMAC
   * DMA memory.  This could eliminate the following, costly memcpy.
   */

  DEBUGASSERT(priv->pd_dev.d_len <= PIC32MX_MAXPACKET_SIZE);
  memcpy(txbuffer, priv->pd_dev.d_buf, priv->pd_dev.d_len);

  /* Bump the producer index, making the packet available for transmission. */
  
  if (++prodidx >= CONFIG_NET_NTXDESC)
    {
     /* Wrap back to index zero */

      prodidx = 0;
    }
  pic32mx_putreg(prodidx, PIC32MX_ETH_TXPRODIDX);

  /* Enable Tx interrupts */

  priv->pd_inten |= ETH_TXINTS;
  pic32mx_putreg(priv->pd_inten, PIC32MX_ETH_IEN);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->pd_txtimeout, PIC32MX_TXTIMEOUT, pic32mx_txtimeout,
                 1, (uint32_t)priv);
  return OK;
}

/****************************************************************************
 * Function: pic32mx_uiptxpoll
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

static int pic32mx_uiptxpoll(struct uip_driver_s *dev)
{
  struct pic32mx_driver_s *priv = (struct pic32mx_driver_s *)dev->d_private;
  int ret = OK;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->pd_dev.d_len > 0)
    {
      /* Send this packet.  In this context, we know that there is space for
       * at least one more packet in the descriptor list.
       */

      uip_arp_out(&priv->pd_dev);
      pic32mx_transmit(priv);

      /* Check if there is room in the device to hold another packet. If not,
       * return any non-zero value to terminate the poll.
       */

      ret = pic32mx_txdesc(priv);
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return ret;
}

/****************************************************************************
 * Function: pic32mx_response
 *
 * Description:
 *   While processing an RxDone event, higher logic decides to send a packet,
 *   possibly a response to the incoming packet (but probably not, in reality).
 *   However, since the Rx and Tx operations are decoupled, there is no
 *   guarantee that there will be a Tx descriptor available at that time.
 *   This function will perform that check and, if no Tx descriptor is 
 *   available, this function will (1) stop incoming Rx processing (bad), and
 *   (2) hold the outgoing packet in a pending state until the next Tx
 *   interrupt occurs. 
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

static void pic32mx_response(struct pic32mx_driver_s *priv)
{
  int ret;

  /* Check if there is room in the device to hold another packet. */

  ret = pic32mx_txdesc(priv);
  if (ret == OK)
    {
       /* Yes.. queue the packet now. */

       pic32mx_transmit(priv);
    }
  else
    {
       /* No.. mark the Tx as pending and halt further Tx interrupts */

       DEBUGASSERT((priv->pd_inten & ETH_INT_TXDONE) != 0);
       
       priv->pd_txpending = true;
       priv->pd_inten    &= ~ETH_RXINTS;
       pic32mx_putreg(priv->pd_inten, PIC32MX_ETH_IEN);
       EMAC_STAT(priv, tx_pending);
    }
}

/****************************************************************************
 * Function: pic32mx_rxdone
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

static void pic32mx_rxdone(struct pic32mx_driver_s *priv)
{
  uint32_t    *rxstat;
  bool         fragment;
  unsigned int prodidx;
  unsigned int considx;
  unsigned int pktlen;

  /* Get the current producer and consumer indices */

  considx = pic32mx_getreg(PIC32MX_ETH_RXCONSIDX) & ETH_RXCONSIDX_MASK;
  prodidx = pic32mx_getreg(PIC32MX_ETH_RXPRODIDX) & ETH_RXPRODIDX_MASK;

  /* Loop while there are incoming packets to be processed, that is, while
   * the producer index is not equal to the consumer index.
   */

  fragment = false;
  while (considx != prodidx)
    {
      /* Update statistics */

      EMAC_STAT(priv, rx_packets);

      /* Get the Rx status and packet length (-4+1) */
    
      rxstat   = (uint32_t*)(PIC32MX_RXSTAT_BASE + (considx << 3));
      pktlen   = (*rxstat & RXSTAT_INFO_RXSIZE_MASK) - 3;

      /* Check for errors.  NOTE:  The DMA engine reports bogus length errors,
       * making this a pretty useless check.
       */

      if ((*rxstat & RXSTAT_INFO_ERROR) != 0)
        {
          nlldbg("Error. considx: %08x prodidx: %08x rxstat: %08x\n",
                 considx, prodidx, *rxstat);
          EMAC_STAT(priv, rx_pkterr);
        }

      /* If the pktlen is greater then the buffer, then we cannot accept
       * the packet.  Also, since the DMA packet buffers are set up to
       * be the same size as our max packet size, any fragments also
       * imply that the packet is too big.
       */
 
      /* else */ if (pktlen > CONFIG_NET_BUFSIZE + CONFIG_NET_GUARDSIZE)
        {
          nlldbg("Too big. considx: %08x prodidx: %08x pktlen: %d rxstat: %08x\n",
                 considx, prodidx, pktlen, *rxstat);
          EMAC_STAT(priv, rx_pktsize);
        }
      else if ((*rxstat & RXSTAT_INFO_LASTFLAG) == 0)
        {
          nlldbg("Fragment. considx: %08x prodidx: %08x pktlen: %d rxstat: %08x\n",
                 considx, prodidx, pktlen, *rxstat);
          EMAC_STAT(priv, rx_fragment);
          fragment = true;
        }
      else if (fragment)
        {
          nlldbg("Last fragment. considx: %08x prodidx: %08x pktlen: %d rxstat: %08x\n",
                 considx, prodidx, pktlen, *rxstat);
          EMAC_STAT(priv, rx_fragment);
          fragment = false;
        }
      else
        {
          uint32_t *rxdesc;
          void     *rxbuffer;

          /* Get the Rx buffer address from the Rx descriptor */
 
          rxdesc   = (uint32_t*)(PIC32MX_RXDESC_BASE + (considx << 3));
          rxbuffer = (void*)*rxdesc;

          /* Copy the data data from the EMAC DMA RAM to priv->pd_dev.d_buf. 
           * Set amount of data in priv->pd_dev.d_len
           *
           * Here would be a great performance improvement:  Remove the
           * buffer from the pd_dev structure and replace it with a pointer
           * directly into the EMAC DMA memory.  This could eliminate the
           * following, costly memcpy.
           */

          memcpy(priv->pd_dev.d_buf, rxbuffer, pktlen);
          priv->pd_dev.d_len = pktlen;

          pic32mx_dumppacket("Received packet",
                           priv->pd_dev.d_buf, priv->pd_dev.d_len);

          /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
          if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
          if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
            {
              /* Handle the incoming Rx packet */

              EMAC_STAT(priv, rx_ip);
              uip_arp_ipin(&priv->pd_dev);
              uip_input(&priv->pd_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the field  d_len will
               * set to a value > 0.
               */

              if (priv->pd_dev.d_len > 0)
                {
                  uip_arp_out(&priv->pd_dev);
                  pic32mx_response(priv);
                }
            }
          else if (BUF->type == htons(UIP_ETHTYPE_ARP))
            {
              EMAC_STAT(priv, rx_arp);
              uip_arp_arpin(&priv->pd_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the field  d_len will
               * set to a value > 0.
               */

              if (priv->pd_dev.d_len > 0)
                {
                  pic32mx_response(priv);
                }
            }
          else
            {
              /* Unrecognized... drop it. */

              EMAC_STAT(priv, rx_dropped);
            }
        }

      /* Bump up the consumer index and resample the producer index (which
       * might also have gotten bumped up by the hardware).
       */

      if (++considx >= CONFIG_NET_NRXDESC)
       {
         /* Wrap back to index zero */

          considx = 0;
        }

      pic32mx_putreg(considx, PIC32MX_ETH_RXCONSIDX);
      prodidx = pic32mx_getreg(PIC32MX_ETH_RXPRODIDX) & ETH_RXPRODIDX_MASK;
    }
}

/****************************************************************************
 * Function: pic32mx_txdone
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
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void pic32mx_txdone(struct pic32mx_driver_s *priv)
{
  /* Cancel the pending Tx timeout */

  wd_cancel(priv->pd_txtimeout);

  /* Disable further Tx interrupts.  Tx interrupts may be re-enabled again
   * depending upon the result of the poll.
   */

  priv->pd_inten &= ~ETH_TXINTS;
  pic32mx_putreg(priv->pd_inten, PIC32MX_ETH_IEN);

  /* Verify that the hardware is ready to send another packet.  Since a Tx
   * just completed, this must be the case.
   */

  DEBUGASSERT(pic32mx_txdesc(priv) == OK);

  /* Check if there is a pending Tx transfer that was scheduled by Rx handling
   * while the Tx logic was busy.  If so, processing that pending Tx now.
   */

  if (priv->pd_txpending)
    {
      /* Clear the pending condition, send the packet, and restore Rx interrupts */

      priv->pd_txpending = false;
      EMAC_STAT(priv, tx_unpend);

      pic32mx_transmit(priv);

      priv->pd_inten    |= ETH_RXINTS;
      pic32mx_putreg(priv->pd_inten, PIC32MX_ETH_IEN);
    }

  /* Otherwise poll uIP for new XMIT data */

  else
    {
      (void)uip_poll(&priv->pd_dev, pic32mx_uiptxpoll);
    }
}

/****************************************************************************
 * Function: pic32mx_interrupt
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

static int pic32mx_interrupt(int irq, void *context)
{
  register struct pic32mx_driver_s *priv;
  uint32_t status;

#if CONFIG_PIC32MX_NINTERFACES > 1
# error "A mechanism to associate and interface with an IRQ is needed"
#else
  priv = &g_ethdrvr[0];
#endif

  /* Get the interrupt status (zero means no interrupts pending). */

  status = pic32mx_getreg(PIC32MX_ETH_INTST);
  if (status != 0)
    {
      /* Clear all pending interrupts */

      pic32mx_putreg(status, PIC32MX_ETH_INTCLR);
      
      /* Handle each pending interrupt **************************************/
      /* Check for Wake-Up on Lan *******************************************/

#ifdef CONFIG_NET_WOL
      if ((status & ETH_INT_WKUP) != 0)
        {
          EMAC_STAT(priv, wol);
#         warning "Missing logic"
        }
      else
#endif
      /* Fatal Errors *******************************************************/
      /* RX OVERRUN -- Fatal overrun error in the receive queue. The fatal
       * interrupt should be resolved by a Rx soft-reset. The bit is not
       * set when there is a nonfatal overrun error.
       *
       * TX UNDERRUN -- Interrupt set on a fatal underrun error in the
       * transmit queue. The fatal interrupt should be resolved by a Tx
       * soft-reset. The bit is not set when there is a nonfatal underrun
       * error.
       */

      if ((status & (ETH_INT_RXOVR|ETH_INT_TXUNR)) != 0)
        {
          if ((status & ETH_INT_RXOVR) != 0)
            {
              nlldbg("RX Overrun. status: %08x\n", status);
              EMAC_STAT(priv, rx_ovrerrors);
            }

          if ((status & ETH_INT_TXUNR) != 0)
            {
              nlldbg("TX Underrun. status: %08x\n", status);
              EMAC_STAT(priv, tx_underrun);
            }

           /* ifup() will reset the EMAC and bring it back up */

           (void)pic32mx_ifup(&priv->pd_dev);
        }
      else
        {      
          /* Check for receive events ***************************************/
          /* RX ERROR -- Triggered on receive errors: AlignmentError,
           * RangeError, LengthError, SymbolError, CRCError or NoDescriptor
           * or Overrun.  NOTE:  (1) We will still need to call pic32mx_rxdone
           * on RX errors to bump the considx over the bad packet.  (2) The
           * DMA engine reports bogus length errors, making this a pretty
           * useless check anyway.
           */

          if ((status & ETH_INT_RXERR) != 0)
            {
              nlldbg("RX Error. status: %08x\n", status);
              EMAC_STAT(priv, rx_errors);
            }

          /* RX FINISHED -- Triggered when all receive descriptors have
           * been processed i.e. on the transition to the situation
           * where ProduceIndex == ConsumeIndex.
           */

          if ((status & ETH_INT_RXFIN) != 0)
            {
              EMAC_STAT(priv, rx_finished);
              DEBUGASSERT(pic32mx_getreg(PIC32MX_ETH_RXPRODIDX) == pic32mx_getreg(PIC32MX_ETH_RXCONSIDX));
            }

          /* RX DONE -- Triggered when a receive descriptor has been
           * processed while the Interrupt bit in the Control field of
           * the descriptor was set.
           */

          if ((status & ETH_INT_RXDONE) != 0)
            {
              EMAC_STAT(priv, rx_done);

              /* We have received at least one new incoming packet. */

              pic32mx_rxdone(priv);
            }
 
          /* Check for Tx events ********************************************/
          /* TX ERROR -- Triggered on transmit errors: LateCollision,
           * ExcessiveCollision and ExcessiveDefer, NoDescriptor or Underrun.
           * NOTE: We will still need to call pic32mx_txdone() in order to
           * clean up after the failed transmit.
           */

          if ((status & ETH_INT_TXERR) != 0)
            {
              nlldbg("TX Error. status: %08x\n", status);
              EMAC_STAT(priv, tx_errors);
            }

          /* TX FINISHED -- Triggered when all transmit descriptors have
           * been processed i.e. on the transition to the situation
           * where ProduceIndex == ConsumeIndex.
           */

          if ((status & ETH_INT_TXFIN) != 0)
            {
              EMAC_STAT(priv, tx_finished);
            }

          /* TX DONE -- Triggered when a descriptor has been transmitted
           * while the Interrupt bit in the Control field of the
           * descriptor was set.
           */

          if ((status & ETH_INT_TXDONE) != 0)
            {
              EMAC_STAT(priv, tx_done);

              /* A packet transmission just completed */

              pic32mx_txdone(priv);
            }
        }
    }

  /* Clear the pending interrupt */

#if 0 /* Apparently not necessary */
# if CONFIG_PIC32MX_NINTERFACES > 1
  pic32mx_clrpend(priv->pd_irqsrc);
# else
  pic32mx_clrpend(PIC32MX_IRQSRC_ETH);
# endif
#endif

  return OK;
}

/****************************************************************************
 * Function: pic32mx_txtimeout
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

static void pic32mx_txtimeout(int argc, uint32_t arg, ...)
{
  struct pic32mx_driver_s *priv = (struct pic32mx_driver_s *)arg;

  /* Increment statistics and dump debug info */

  EMAC_STAT(priv, tx_timeouts);
  if (priv->pd_ifup)
    {
      /* Then reset the hardware. ifup() will reset the interface, then bring
       * it back up.
       */

      (void)pic32mx_ifup(&priv->pd_dev);

      /* Then poll uIP for new XMIT data */

      (void)uip_poll(&priv->pd_dev, pic32mx_uiptxpoll);
    }
}

/****************************************************************************
 * Function: pic32mx_polltimer
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

static void pic32mx_polltimer(int argc, uint32_t arg, ...)
{
  struct pic32mx_driver_s *priv = (struct pic32mx_driver_s *)arg;

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  if (pic32mx_txdesc(priv) == OK)
    {
      /* If so, update TCP timing states and poll uIP for new XMIT data. Hmmm..
       * might be bug here.  Does this mean if there is a transmit in progress,
       * we will missing TCP time state updates?
       */

      (void)uip_timer(&priv->pd_dev, pic32mx_uiptxpoll, PIC32MX_POLLHSEC);
    }

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->pd_txpoll, PIC32MX_WDDELAY, pic32mx_polltimer, 1, arg);
}

/****************************************************************************
 * Function: pic32mx_ifup
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

static int pic32mx_ifup(struct uip_driver_s *dev)
{
  struct pic32mx_driver_s *priv = (struct pic32mx_driver_s *)dev->d_private;
  uint32_t regval;
  int ret;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);

  /* Reset the Ethernet controller (again) */

  pic32mx_ethreset(priv);

  /* Initialize the PHY and wait for the link to be established */

  ret = pic32mx_phyinit(priv);
  if (ret != 0)
    {
      ndbg("pic32mx_phyinit failed: %d\n", ret);
      return ret;
    }

  /* Configure the MAC station address */

  regval = (uint32_t)priv->pd_dev.d_mac.ether_addr_octet[5] << 8 |
           (uint32_t)priv->pd_dev.d_mac.ether_addr_octet[4];
  pic32mx_putreg(regval, PIC32MX_EMAC1_SA0);

  regval = (uint32_t)priv->pd_dev.d_mac.ether_addr_octet[3] << 8 |
           (uint32_t)priv->pd_dev.d_mac.ether_addr_octet[2];
  pic32mx_putreg(regval, PIC32MX_EMAC1_SA1);

  regval = (uint32_t)priv->pd_dev.d_mac.ether_addr_octet[1] << 8 |
           (uint32_t)priv->pd_dev.d_mac.ether_addr_octet[0];
  pic32mx_putreg(regval, PIC32MX_EMAC1_SA2);

  /* Initialize Ethernet interface for the PHY setup */

  pic32mx_macmode(priv->pd_mode);

  /* Initialize EMAC DMA memory -- descriptors, status, packet buffers, etc. */

  pic32mx_txdescinit(priv);
  pic32mx_rxdescinit(priv);

  /* Configure to pass all received frames */

  regval = pic32mx_getreg(PIC32MX_EMAC1_CFG1);
  regval |= EMAC1_CFG1_PASSALL;
  pic32mx_putreg(regval, PIC32MX_EMAC1_CFG1);

  /* Set up RX filter and configure to accept broadcast addresses, multicast
   * addresses, and perfect station address matches.  We should also accept
   * perfect matches and, most likely, broadcast (for example, for ARP requests).
   * Other RX filter options will only be enabled if so selected.  NOTE: There
   * is a selection CONFIG_NET_BROADCAST, but this enables receipt of UDP
   * broadcast packets inside of the stack.
   */

  regval = ETH_RXFC_PERFEN | ETH_RXFC_BCEN;
#ifdef CONFIG_NET_MULTICAST
  regval |= (ETH_RXFC_MCEN | ETH_RXFC_UCEN);
#endif
  pic32mx_putreg(regval, PIC32MX_ETH_RXFC);

  /* Clear any pending interrupts (shouldn't be any) */

  pic32mx_putreg(0xffffffff, PIC32MX_ETH_INTCLR);

  /* Configure interrupts.  The Ethernet interrupt was attached during one-time
   * initialization, so we only need to set the interrupt priority, configure
   * interrupts, and enable them.
   */

  /* Set the interrupt to the highest priority */

#ifdef CONFIG_ARCH_IRQPRIO
#if CONFIG_PIC32MX_NINTERFACES > 1
  (void)up_prioritize_irq(priv->pd_irq, CONFIG_NET_PRIORITY);
#else
  (void)up_prioritize_irq(PIC32MX_IRQ_ETH, CONFIG_NET_PRIORITY);
#endif
#endif

  /* Enable Ethernet interrupts.  The way we do this depends on whether or
   * not Wakeup on Lan (WoL) has been configured.
   */

#ifdef CONFIG_NET_WOL
  /* Configure WoL: Clear all receive filter WoLs and enable the perfect
   * match WoL interrupt.  We will wait until the Wake-up to finish
   * bringing things up.
   */

  pic32mx_putreg(0xffffffff, PIC32MX_ETH_RXFLWOLCLR);
  pic32mx_putreg(ETH_RXFC_RXFILEN, PIC32MX_ETH_RXFC);

  priv->pd_inten = ETH_INT_WKUP;
  pic32mx_putreg(ETH_INT_WKUP, PIC32MX_ETH_IEN);
#else
  /* Otherwise, enable all Rx interrupts.  Tx interrupts, SOFTINT and WoL are
   * excluded.  Tx interrupts will not be enabled until there is data to be
   * sent.
   */

  priv->pd_inten = ETH_RXINTS;
  pic32mx_putreg(ETH_RXINTS, PIC32MX_ETH_IEN);
#endif

  /* Enable Rx. "Enabling of the receive function is located in two places.
   * The receive DMA manager needs to be enabled and the receive data path
   * of the MAC needs to be enabled. To prevent overflow in the receive
   * DMA engine the receive DMA engine should be enabled by setting the
   * RxEnable bit in the Command register before enabling the receive data
   * path in the MAC by setting the RECEIVE ENABLE bit in the MAC1 register."
   */

  regval  = pic32mx_getreg(PIC32MX_ETH_CMD);
  regval |= ETH_CMD_RXEN;
  pic32mx_putreg(regval, PIC32MX_ETH_CMD);

  regval  = pic32mx_getreg(PIC32MX_EMAC1_CFG1);
  regval |= EMAC1_CFG1_RE;
  pic32mx_putreg(regval, PIC32MX_EMAC1_CFG1);

  /* Enable Tx */

  regval  = pic32mx_getreg(PIC32MX_ETH_CMD);
  regval |= ETH_CMD_TXEN;
  pic32mx_putreg(regval, PIC32MX_ETH_CMD);

  /* Set and activate a timer process */

  (void)wd_start(priv->pd_txpoll, PIC32MX_WDDELAY, pic32mx_polltimer, 1,
                (uint32_t)priv);

  /* Finally, make the interface up and enable the Ethernet interrupt at
   * the interrupt controller
   */

  priv->pd_ifup = true;
#if CONFIG_PIC32MX_NINTERFACES > 1
  up_enable_irq(priv->pd_irqsrc);
#else
  up_enable_irq(PIC32MX_IRQSRC_ETH);
#endif
  return OK;
}

/****************************************************************************
 * Function: pic32mx_ifdown
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

static int pic32mx_ifdown(struct uip_driver_s *dev)
{
  struct pic32mx_driver_s *priv = (struct pic32mx_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = irqsave();
#if CONFIG_PIC32MX_NINTERFACES > 1
  up_disable_irq(priv->pd_irqsrc);
#else
  up_disable_irq(PIC32MX_IRQSRC_ETH);
#endif

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->pd_txpoll);
  wd_cancel(priv->pd_txtimeout);

  /* Reset the device and mark it as down. */

  pic32mx_ethreset(priv);
  priv->pd_ifup = false;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: pic32mx_txavail
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

static int pic32mx_txavail(struct uip_driver_s *dev)
{
  struct pic32mx_driver_s *priv = (struct pic32mx_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable interrupts because this function may be called from interrupt
   * level processing.
   */

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (priv->pd_ifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */

      if (pic32mx_txdesc(priv) == OK)
        {
          /* If so, then poll uIP for new XMIT data */

          (void)uip_poll(&priv->pd_dev, pic32mx_uiptxpoll);
        }
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: pic32mx_addmac
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
static int pic32mx_addmac(struct uip_driver_s *dev, const uint8_t *mac)
{
  struct pic32mx_driver_s *priv = (struct pic32mx_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

#warning "Not implemented"
  return OK;
}
#endif

/****************************************************************************
 * Function: pic32mx_rmmac
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
static int pic32mx_rmmac(struct uip_driver_s *dev, const uint8_t *mac)
{
  struct pic32mx_driver_s *priv = (struct pic32mx_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

#warning "Not implemented"
  return OK;
}
#endif

/*******************************************************************************
 * Name: pic32mx_showpins
 *
 * Description:
 *   Dump GPIO registers
 *
 * Parameters:
 *   None 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 *******************************************************************************/

#if defined(CONFIG_NET_REGDEBUG) && defined(CONFIG_DEBUG_GPIO)
static void pic32mx_showpins(void)
{
  pic32mx_dumpgpio(GPIO_PORT1|GPIO_PIN0, "P1[1-15]");
  pic32mx_dumpgpio(GPIO_PORT1|GPIO_PIN16, "P1[16-31]");
}
#endif

/*******************************************************************************
 * Name: pic32mx_showmii
 *
 * Description:
 *   Dump PHY MII registers
 *
 * Parameters:
 *   phyaddr - The device address where the PHY was discovered
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 *******************************************************************************/

#if defined(CONFIG_NET_REGDEBUG) && defined(PIC32MX_HAVE_PHY)
static void pic32mx_showmii(uint8_t phyaddr, const char *msg)
{
  dbg("PHY " PIC32MX_PHYNAME ": %s\n", msg);
  dbg("  MCR:       %04x\n", pic32mx_phyread(phyaddr, MII_MCR));
  dbg("  MSR:       %04x\n", pic32mx_phyread(phyaddr, MII_MSR));
  dbg("  ADVERTISE: %04x\n", pic32mx_phyread(phyaddr, MII_ADVERTISE));
  dbg("  LPA:       %04x\n", pic32mx_phyread(phyaddr, MII_LPA));
  dbg("  EXPANSION: %04x\n", pic32mx_phyread(phyaddr, MII_EXPANSION));
#ifdef CONFIG_PHY_KS8721
  dbg("  10BTCR:    %04x\n", pic32mx_phyread(phyaddr, MII_KS8721_10BTCR));
#endif
}
#endif

/****************************************************************************
 * Function: pic32mx_phywrite
 *
 * Description:
 *   Write a value to an MII PHY register
 *
 * Parameters:
 *   phyaddr - The device address where the PHY was discovered
 *   regaddr - The address of the PHY register to be written
 *   phydata - The data to write to the PHY register 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef PIC32MX_HAVE_PHY
static void pic32mx_phywrite(uint8_t phyaddr, uint8_t regaddr, uint16_t phydata)
{
  uint32_t regval;

  /* Set PHY address and PHY register address */

  regval = ((uint32_t)phyaddr << EMAC1_MADR_PHYADDR_SHIFT) |
           ((uint32_t)regaddr << EMAC1_MADR_REGADDR_SHIFT);
  pic32mx_putreg(regval, PIC32MX_EMAC1_MADR);

  /* Set up to write */

  pic32mx_putreg(EMAC1_MCMD_WRITE, PIC32MX_EMAC1_MCMD);

  /* Write the register data to the PHY */

  pic32mx_putreg((uint32_t)phydata, PIC32MX_EMAC1_MWTD);

  /* Wait for the PHY command to complete */

  while ((pic32mx_getreg(PIC32MX_EMAC1_MIND) & EMAC1_MIND_MIIMBUSY) != 0);
}
#endif

/****************************************************************************
 * Function: pic32mx_phyread
 *
 * Description:
 *   Read a value from an MII PHY register
 *
 * Parameters:
 *   phyaddr - The device address where the PHY was discovered
 *   regaddr - The address of the PHY register to be written
 *
 * Returned Value:
 *   Data read from the PHY register
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef PIC32MX_HAVE_PHY
static uint16_t pic32mx_phyread(uint8_t phyaddr, uint8_t regaddr)
{
  uint32_t regval;

  pic32mx_putreg(0, PIC32MX_EMAC1_MCMD);

  /* Set PHY address and PHY register address */

  regval = ((uint32_t)phyaddr << EMAC1_MADR_PHYADDR_SHIFT) |
           ((uint32_t)regaddr << EMAC1_MADR_REGADDR_SHIFT);
  pic32mx_putreg(regval, PIC32MX_EMAC1_MADR);

  /* Set up to read */

  pic32mx_putreg(EMAC1_MCMD_READ, PIC32MX_EMAC1_MCMD);

  /* Wait for the PHY command to complete */

  while ((pic32mx_getreg(PIC32MX_EMAC1_MIND) & (EMAC1_MIND_MIIMBUSY|EMAC1_MIND_NOTVALID)) != 0);
  pic32mx_putreg(0, PIC32MX_EMAC1_MCMD);

  /* Return the PHY register data */

  return (uint16_t)(pic32mx_getreg(PIC32MX_EMAC1_MRDD) & EMAC1_MRDD_MASK);
}
#endif

/****************************************************************************
 * Function: pic32mx_phyreset
 *
 * Description:
 *   Reset the PHY
 *
 * Parameters:
 *   phyaddr - The device address where the PHY was discovered
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef PIC32MX_HAVE_PHY
static inline int pic32mx_phyreset(uint8_t phyaddr)
{
  int32_t timeout;
  uint16_t phyreg;

  /* Reset the PHY.  Needs a minimal 50uS delay after reset. */

  pic32mx_phywrite(phyaddr, MII_MCR, MII_MCR_RESET);

  /* Wait for a minimum of 50uS no matter what */

  up_udelay(50);

  /* The MCR reset bit is self-clearing.  Wait for it to be clear indicating
   * that the reset is complete.
   */

  for (timeout = MII_BIG_TIMEOUT; timeout > 0; timeout--)
    {
      phyreg = pic32mx_phyread(phyaddr, MII_MCR);
      if ((phyreg & MII_MCR_RESET) == 0)
        {
          return OK;
        }
    }

  ndbg("Reset failed. MCR: %04x\n", phyreg);
  return -ETIMEDOUT;
}
#endif

/****************************************************************************
 * Function: pic32mx_phyautoneg
 *
 * Description:
 *   Enable auto-negotiation.
 *
 * Parameters:
 *   phyaddr - The device address where the PHY was discovered
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The adverisement regiser has already been configured.
 *
 ****************************************************************************/

#if defined(PIC32MX_HAVE_PHY) && defined(CONFIG_PHY_AUTONEG)
static inline int pic32mx_phyautoneg(uint8_t phyaddr)
{
  int32_t timeout;
  uint16_t phyreg;

  /* Start auto-negotiation */

  pic32mx_phywrite(phyaddr, MII_MCR, MII_MCR_ANENABLE | MII_MCR_ANRESTART);

  /* Wait for autonegotiation to complete */

  for (timeout = MII_BIG_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if auto-negotiation has completed */

      phyreg = pic32mx_phyread(phyaddr, MII_MSR);
      if ((phyreg & MII_MSR_ANEGCOMPLETE) != 0)
        {
          /* Yes.. return success */

          return OK;
        }
    }

  ndbg("Auto-negotiation failed. MSR: %04x\n", phyreg);
  return -ETIMEDOUT;
}
#endif

/****************************************************************************
 * Function: pic32mx_phymode
 *
 * Description:
 *   Set the PHY to operate at a selected speed/duplex mode.
 *
 * Parameters:
 *   phyaddr - The device address where the PHY was discovered
 *   mode - speed/duplex mode
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef PIC32MX_HAVE_PHY
static int pic32mx_phymode(uint8_t phyaddr, uint8_t mode)
{
  int32_t timeout;
  uint16_t phyreg;

  /* Disable auto-negotiation and set fixed Speed and Duplex settings:
   *
   *   MII_MCR_UNIDIR      0=Disable unidirectional enable
   *   MII_MCR_SPEED1000   0=Reserved on 10/100
   *   MII_MCR_CTST        0=Disable collision test
   *   MII_MCR_FULLDPLX    ?=Full duplex
   *   MII_MCR_ANRESTART   0=Don't restart auto negotiation
   *   MII_MCR_ISOLATE     0=Don't electronically isolate PHY from MII
   *   MII_MCR_PDOWN       0=Don't powerdown the PHY
   *   MII_MCR_ANENABLE    0=Disable auto negotiation
   *   MII_MCR_SPEED100    ?=Select 100Mbps
   *   MII_MCR_LOOPBACK    0=Disable loopback mode
   *   MII_MCR_RESET       0=No PHY reset
   */

  phyreg = 0;
  if ((mode & PIC32MX_SPEED_MASK) ==  PIC32MX_SPEED_100)
    {
      phyreg = MII_MCR_SPEED100;
    }

  if ((mode & PIC32MX_DUPLEX_MASK) == PIC32MX_DUPLEX_FULL)
    {
      phyreg |= MII_MCR_FULLDPLX;
    }

  pic32mx_phywrite(phyaddr, MII_MCR, phyreg);

  /* Then wait for the link to be established */

  for (timeout = MII_BIG_TIMEOUT; timeout > 0; timeout--)
    {
#ifdef CONFIG_PHY_DP83848C
      phyreg = pic32mx_phyread(phyaddr, MII_DP83848C_STS);
      if ((phyreg & 0x0001) != 0)
        {
          /* Yes.. return success */

          return OK;
        }
#else
      phyreg = pic32mx_phyread(phyaddr, MII_MSR);
      if ((phyreg & MII_MSR_LINKSTATUS) != 0)
        {
          /* Yes.. return success */

          return OK;
        }
#endif
    }

  ndbg("Link failed. MSR: %04x\n", phyreg);
  return -ETIMEDOUT;
}
#endif

/****************************************************************************
 * Function: pic32mx_phyinit
 *
 * Description:
 *   Initialize the PHY
 *
 * Parameters:
 *   priv - Pointer to EMAC device driver structure 
 *
 * Returned Value:
 *   None directly.  As a side-effect, it will initialize priv->pd_phyaddr
 *   and priv->pd_phymode.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef PIC32MX_HAVE_PHY
static inline int pic32mx_phyinit(struct pic32mx_driver_s *priv)
{
  unsigned int phyaddr;
  uint16_t phyreg;
  uint32_t regval;
  int ret;

  /* MII configuration: host clocked divided per board.h, no suppress
   * preamble, no scan increment.
   */

  pic32mx_putreg(EMAC1_MCFG_CLKSEL_DIV, PIC32MX_EMAC1_MCFG);
  pic32mx_putreg(0, PIC32MX_EMAC1_MCMD);

  /* Enter RMII mode and select 100 MBPS support */

  pic32mx_putreg(ETH_CMD_RMII, PIC32MX_ETH_CMD);
  pic32mx_putreg(EMAC1_SUPP_SPEEDRMII:, PIC32MX_ETH_SUPP);

  /* Find PHY Address.  Because the controller has a pull-up and the
   * PHY has pull-down resistors on RXD lines some times the PHY
   * latches different at different addresses.
   */

  for (phyaddr = 1; phyaddr < 32; phyaddr++)
    {
       /* Check if we can see the selected device ID at this
        * PHY address.
        */

       phyreg = (unsigned int)pic32mx_phyread(phyaddr, MII_PHYID1);
       nvdbg("Addr: %d PHY ID1: %04x\n", phyaddr, phyreg);

       if (phyreg == PIC32MX_PHYID1)
        {
          phyreg = pic32mx_phyread(phyaddr, MII_PHYID2);
          nvdbg("Addr: %d PHY ID2: %04x\n", phyaddr, phyreg);

          if (phyreg  == PIC32MX_PHYID2)
            {
              break;
            }
        }
    }

  /* Check if the PHY device address was found */

  if (phyaddr > 31)
    {
      /* Failed to find PHY at any location */

      ndbg("No PHY detected\n");
      return -ENODEV;
    }
  nvdbg("phyaddr: %d\n", phyaddr);

  /* Save the discovered PHY device address */

  priv->pd_phyaddr = phyaddr;

  /* Reset the PHY */

  ret = pic32mx_phyreset(phyaddr);
  if (ret < 0)
    {
      return ret;
    }
  pic32mx_showmii(phyaddr, "After reset");

  /* Check for preamble suppression support */

  phyreg = pic32mx_phyread(phyaddr, MII_MSR);
  if ((phyreg & MII_MSR_MFRAMESUPPRESS) != 0)
    {
      /* The PHY supports preamble suppression */

      regval  = pic32mx_getreg(PIC32MX_EMAC1_MCFG);
      regval |= EMAC1_MCFG_NOPRE;
      pic32mx_putreg(regval, PIC32MX_EMAC1_MCFG);
    }

  /* Are we configured to do auto-negotiation? */

#ifdef CONFIG_PHY_AUTONEG
  /* Setup the Auto-negotiation advertisement: 100 or 10, and HD or FD */

  pic32mx_phywrite(phyaddr, MII_ADVERTISE, 
                 (MII_ADVERTISE_100BASETXFULL | MII_ADVERTISE_100BASETXHALF |
                  MII_ADVERTISE_10BASETXFULL  | MII_ADVERTISE_10BASETXHALF  |
                  MII_ADVERTISE_CSMA));

  /* Then perform the auto-negotiation */

  ret = pic32mx_phyautoneg(phyaddr);
  if (ret < 0)
    {
      return ret;
    }
#else
  /* Set up the fixed PHY configuration */

  ret = pic32mx_phymode(phyaddr, PIC32MX_MODE_DEFLT);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* The link is established */

  pic32mx_showmii(phyaddr, "After link established");

  /* Check configuration */

#if defined(CONFIG_PHY_KS8721)
  phyreg = pic32mx_phyread(phyaddr, MII_KS8721_10BTCR);

  switch (phyreg & KS8721_10BTCR_MODE_MASK)
    {
      case KS8721_10BTCR_MODE_10BTHD:  /* 10BASE-T half duplex */
        priv->pd_mode = PIC32MX_10BASET_HD;
        pic32mx_putreg(0, PIC32MX_ETH_SUPP);
        break;
      case KS8721_10BTCR_MODE_100BTHD: /* 100BASE-T half duplex */
        priv->pd_mode = PIC32MX_100BASET_HD;
        break;
      case KS8721_10BTCR_MODE_10BTFD: /* 10BASE-T full duplex */
        priv->pd_mode = PIC32MX_10BASET_FD;
        pic32mx_putreg(0, PIC32MX_ETH_SUPP);
        break;
      case KS8721_10BTCR_MODE_100BTFD: /* 100BASE-T full duplex */
        priv->pd_mode = PIC32MX_100BASET_FD;
        break;
      default:
        ndbg("Unrecognized mode: %04x\n", phyreg);
        return -ENODEV;
    }
#elif defined(CONFIG_PHY_DP83848C)
  phyreg = pic32mx_phyread(phyaddr, MII_DP83848C_STS);

  /* Configure for full/half duplex mode and speed */

  switch (phyreg & 0x0006)
    {
      case 0x0000:
        priv->pd_mode = PIC32MX_100BASET_HD;
        break;
      case 0x0002:
        priv->pd_mode = PIC32MX_10BASET_HD;
        break;
      case 0x0004:
        priv->pd_mode = PIC32MX_100BASET_FD;
        break;
      case 0x0006:
        priv->pd_mode = PIC32MX_10BASET_FD;
        break;
      default:
        ndbg("Unrecognized mode: %04x\n", phyreg);
        return -ENODEV;
    }
#elif defined(CONFIG_PHY_LAN8720)
  {
    uint16_t advertise;
    uint16_t lpa;

    up_udelay(500);
    advertise = pic32mx_phyread(phyaddr, MII_ADVERTISE);
    lpa       = pic32mx_phyread(phyaddr, MII_LPA);

    /* Check for 100BASETX full duplex */

    if ((advertise & MII_ADVERTISE_100BASETXFULL) != 0 &&
        (lpa & MII_LPA_100BASETXFULL) != 0)
      {
        priv->pd_mode = PIC32MX_100BASET_FD;
      }

    /* Check for 100BASETX half duplex */

    else if ((advertise & MII_ADVERTISE_100BASETXHALF) != 0 &&
        (lpa & MII_LPA_100BASETXHALF) != 0)
      {
        priv->pd_mode = PIC32MX_100BASET_HD;
      }

    /* Check for 10BASETX full duplex */

    else if ((advertise & MII_ADVERTISE_10BASETXFULL) != 0 &&
        (lpa & MII_LPA_10BASETXFULL) != 0)
      {
        priv->pd_mode = PIC32MX_10BASET_FD;
      }

    /* Check for 10BASETX half duplex */

    else if ((advertise & MII_ADVERTISE_10BASETXHALF) != 0 &&
        (lpa & MII_LPA_10BASETXHALF) != 0)
      {
        priv->pd_mode = PIC32MX_10BASET_HD;
      }
    else
      {
        ndbg("Unrecognized mode: %04x\n", phyreg);
        return -ENODEV;
      }
  }
#else
#  warning "PHY Unknown: speed and duplex are bogus"
#endif

  ndbg("%dBase-T %s duplex\n",
       (priv->pd_mode & PIC32MX_SPEED_MASK) ==  PIC32MX_SPEED_100 ? 100 : 10,
       (priv->pd_mode & PIC32MX_DUPLEX_MASK) == PIC32MX_DUPLEX_FULL ?"full" : "half");

  /* Disable auto-configuration.  Set the fixed speed/duplex mode.
   * (probably more than little redundant).
   */

  ret = pic32mx_phymode(phyaddr, priv->pd_mode);
  pic32mx_showmii(phyaddr, "After final configuration");
  return ret;
}
#else
static inline int pic32mx_phyinit(struct pic32mx_driver_s *priv)
{
  priv->pd_mode = PIC32MX_MODE_DEFLT;
  return OK;
}
#endif

/****************************************************************************
 * Function: pic32mx_txdescinit
 *
 * Description:
 *   Initialize the EMAC Tx descriptor table
 *
 * Parameters:
 *   priv - Pointer to EMAC device driver structure 
 *
 * Returned Value:
 *   None directory.
 *   As a side-effect, it will initialize priv->pd_phyaddr and
 *   priv->pd_phymode.
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void pic32mx_txdescinit(struct pic32mx_driver_s *priv)
{
  uint32_t *txdesc;
  uint32_t *txstat;
  uint32_t pktaddr;
  int i;

  /* Configure Tx descriptor and status tables */

  pic32mx_putreg(PIC32MX_TXDESC_BASE, PIC32MX_ETH_TXST);
  pic32mx_putreg(PIC32MX_TXSTAT_BASE, PIC32MX_ETH_TXSTAT);
  pic32mx_putreg(CONFIG_NET_NTXDESC-1, PIC32MX_ETH_TXDESCRNO);

  /* Initialize Tx descriptors and link to packet buffers */

  txdesc  = (uint32_t*)PIC32MX_TXDESC_BASE;
  pktaddr = PIC32MX_TXBUFFER_BASE;

  for (i = 0; i < CONFIG_NET_NTXDESC; i++)
    {
      *txdesc++ = pktaddr;
      *txdesc++ = (TXDESC_CONTROL_INT | (PIC32MX_MAXPACKET_SIZE - 1));
      pktaddr  += PIC32MX_MAXPACKET_SIZE;
    }

  /* Initialize Tx status */

  txstat  = (uint32_t*)PIC32MX_TXSTAT_BASE;
  for (i = 0; i < CONFIG_NET_NTXDESC; i++)
    {
      *txstat++ = 0;
    }

  /* Point to first Tx descriptor */

  pic32mx_putreg(0, PIC32MX_ETH_TXPRODIDX);
}

/****************************************************************************
 * Function: pic32mx_rxdescinit
 *
 * Description:
 *   Initialize the EMAC Rx descriptor table
 *
 * Parameters:
 *   priv - Pointer to EMAC device driver structure 
 *
 * Returned Value:
 *   None directory.
 *   As a side-effect, it will initialize priv->pd_phyaddr and
 *   priv->pd_phymode.
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void pic32mx_rxdescinit(struct pic32mx_driver_s *priv)
{
  uint32_t *rxdesc;
  uint32_t *rxstat;
  uint32_t pktaddr;
  int i;

  /* Configure Rx descriptor and status tables */

  pic32mx_putreg(PIC32MX_RXDESC_BASE, PIC32MX_ETH_RXST);
  pic32mx_putreg(PIC32MX_RXSTAT_BASE, PIC32MX_ETH_RXSTAT);
  pic32mx_putreg(CONFIG_NET_NRXDESC-1, PIC32MX_ETH_RXDESCNO);

  /* Initialize Rx descriptors and link to packet buffers */

  rxdesc  = (uint32_t*)PIC32MX_RXDESC_BASE;
  pktaddr = PIC32MX_RXBUFFER_BASE;

  for (i = 0; i < CONFIG_NET_NRXDESC; i++)
    {
      *rxdesc++ = pktaddr;
      *rxdesc++ = (RXDESC_CONTROL_INT | (PIC32MX_MAXPACKET_SIZE - 1));
      pktaddr  += PIC32MX_MAXPACKET_SIZE;
    }

  /* Initialize Rx status */

  rxstat  = (uint32_t*)PIC32MX_RXSTAT_BASE;
  for (i = 0; i < CONFIG_NET_NRXDESC; i++)
    {
      *rxstat++ = 0;
      *rxstat++ = 0;
    }

  /* Point to first Rx descriptor */

  pic32mx_putreg(0, PIC32MX_ETH_RXCONSIDX);
}

/****************************************************************************
 * Function: pic32mx_macmode
 *
 * Description:
 *   Set the MAC to operate at a selected speed/duplex mode.
 *
 * Parameters:
 *   mode - speed/duplex mode
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef PIC32MX_HAVE_PHY
static void pic32mx_macmode(uint8_t mode)
{
  uint32_t regval;

  /* Set up for full or half duplex operation */

  if ((mode & PIC32MX_DUPLEX_MASK) == PIC32MX_DUPLEX_FULL)
    {
      /* Set the back-to-back inter-packet gap */
 
      pic32mx_putreg(21, PIC32MX_EMAC1_IPGT);

      /* Set MAC to operate in full duplex mode with CRC and Pad enabled */

      regval = pic32mx_getreg(PIC32MX_EMAC1_CFG2);
      regval |= (EMAC1_CFG2_FULLDPLX | EMAC1_CFG2_CRCEN | EMAC1_CFG2_PADCRCEN);
      pic32mx_putreg(regval, PIC32MX_EMAC1_CFG2);

      /* Select full duplex operation for ethernet controller */

      regval = pic32mx_getreg(PIC32MX_ETH_CMD);
      regval |= (ETH_CMD_FD | ETH_CMD_RMII | ETH_CMD_PRFRAME);
      pic32mx_putreg(regval, PIC32MX_ETH_CMD);
    }
  else
    {
      /* Set the back-to-back inter-packet gap */
 
      pic32mx_putreg(18, PIC32MX_EMAC1_IPGT);

      /* Set MAC to operate in half duplex mode with CRC and Pad enabled */

      regval = pic32mx_getreg(PIC32MX_EMAC1_CFG2);
      regval &= ~EMAC1_CFG2_FULLDPLX;
      regval |= (EMAC1_CFG2_CRCEN | EMAC1_CFG2_PADCRCEN);
      pic32mx_putreg(regval, PIC32MX_EMAC1_CFG2);

      /* Select half duplex operation for ethernet controller */

      regval = pic32mx_getreg(PIC32MX_ETH_CMD);
      regval &= ~ETH_CMD_FD;
      regval |= (ETH_CMD_RMII | ETH_CMD_PRFRAME);
      pic32mx_putreg(regval, PIC32MX_ETH_CMD);
    }

  /* This is currently done in pic32mx_phyinit().  That doesn't
   * seem like the right place. It should be done here.
   */

#if 0
  regval = pic32mx_getreg(PIC32MX_ETH_SUPP);
  if ((mode & PIC32MX_SPEED_MASK) == PIC32MX_SPEED_100)
    {
      regval |= EMAC1_SUPP_SPEEDRMII:;
    }
  else
    {
      regval &= ~EMAC1_SUPP_SPEEDRMII:;
    }
  pic32mx_putreg(regval, PIC32MX_ETH_SUPP);
#endif
}
#endif

/****************************************************************************
 * Function: pic32mx_ethreset
 *
 * Description:
 *   Configure and reset the Ethernet module, leaving it in a disabled state.
 *
 * Parameters:
 *   priv   - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static void pic32mx_ethreset(struct pic32mx_driver_s *priv)
{
  irqstate_t flags;

  /* Reset the MAC */

  flags = irqsave();

  /* Put the MAC into the reset state */

  pic32mx_putreg((EMAC1_CFG1_TXRST    | EMAC1_CFG1_MCSTXRST | EMAC1_CFG1_RXRST |
                EMAC1_CFG1_MCSRXRST | EMAC1_CFG1_SIMRST   | EMAC1_CFG1_SOFTRST),
               PIC32MX_EMAC1_CFG1);

  /* Disable RX/RX, clear modes, reset all control registers */

  pic32mx_putreg((ETH_CMD_REGRST | ETH_CMD_TXRST | ETH_CMD_RXRST),
               PIC32MX_ETH_CMD);

  /* Take the MAC out of the reset state */

  up_udelay(50);
  pic32mx_putreg(0, PIC32MX_EMAC1_CFG1);

  /* The RMII bit must be set on initialization (I'm not sure this needs
   * to be done here but... oh well).
   */

  pic32mx_putreg(ETH_CMD_RMII, PIC32MX_ETH_CMD);

  /* Set other misc configuration-related registers to default values */

  pic32mx_putreg(0, PIC32MX_EMAC1_CFG2);
  pic32mx_putreg(0, PIC32MX_ETH_SUPP);
  pic32mx_putreg(0, PIC32MX_EMAC1_TEST);

  pic32mx_putreg(((12 << EMAC1_IPGR_GAP1_SHIFT) | (12 << EMAC1_IPGR_GAP2_SHIFT)),
                 PIC32MX_EMAC1_IPGR);
  pic32mx_putreg(((15 << EMAC1_CLRT_RETX_SHIFT) | (55 << EMAC1_CLRT_CWINDOW_SHIFT)),
                 PIC32MX_EMAC1_CLRT);

  /* Set the Maximum Frame size register. "This field resets to the value
   * 0x0600, which represents a maximum receive frame of 1536 octets. An
   * untagged maximum size Ethernet frame is 1518 octets. A tagged frame adds
   * four octets for a total of 1522 octets. If a shorter maximum length
   * restriction is desired, program this 16-bit field."
   */

  pic32mx_putreg(PIC32MX_MAXPACKET_SIZE, PIC32MX_EMAC1_MAXF);

  /* Disable all Ethernet controller interrupts */

  pic32mx_putreg(0, PIC32MX_ETH_IEN);

  /* Clear any pending interrupts (shouldn't be any) */

  pic32mx_putreg(0xffffffff, PIC32MX_ETH_INTCLR);
  irqrestore(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: pic32mx_ethinitialize
 *
 * Description:
 *   Initialize one Ethernet controller and driver structure.
 *
 * Parameters:
 *   intf - Selects the interface to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if CONFIG_PIC32MX_NINTERFACES > 1
int pic32mx_ethinitialize(int intf)
#else
static inline int pic32mx_ethinitialize(int intf)
#endif
{
  struct pic32mx_driver_s *priv;
  uint32_t regval;
  int ret;
  int i;

  DEBUGASSERT(intf < CONFIG_PIC32MX_NINTERFACES);
  priv = &g_ethdrvr[intf];

  /* Turn on the ethernet MAC clock */

  regval  = pic32mx_getreg(PIC32MX_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCENET;
  pic32mx_putreg(regval, PIC32MX_SYSCON_PCONP);

  /* Configure all GPIO pins needed by ENET */

  for (i = 0; i < GPIO_NENET_PINS; i++)
    {
      (void)pic32mx_configgpio(g_enetpins[i]);
    }
  pic32mx_showpins();

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct pic32mx_driver_s));
  priv->pd_dev.d_ifup    = pic32mx_ifup;    /* I/F down callback */
  priv->pd_dev.d_ifdown  = pic32mx_ifdown;  /* I/F up (new IP address) callback */
  priv->pd_dev.d_txavail = pic32mx_txavail; /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->pd_dev.d_addmac  = pic32mx_addmac;  /* Add multicast MAC address */
  priv->pd_dev.d_rmmac   = pic32mx_rmmac;   /* Remove multicast MAC address */
#endif
  priv->pd_dev.d_private = (void*)priv;   /* Used to recover private state from dev */

#if CONFIG_PIC32MX_NINTERFACES > 1
# error "A mechanism to associate base address an IRQ with an interface is needed"
  priv->pd_base          = ??;            /* Ethernet controller base address */
  priv->pd_irq           = ??;            /* Ethernet controller IRQ vector number */
  priv->pd_irqsrc        = ??;            /* Ethernet controller IRQ source number */
#endif

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->pd_txpoll        = wd_create();   /* Create periodic poll timer */
  priv->pd_txtimeout     = wd_create();   /* Create TX timeout timer */

  /* Reset the Ethernet controller and leave in the ifdown statue.  The
   * Ethernet controller will be properly re-initialized each time
   * pic32mx_ifup() is called.
   */

  pic32mx_ifdown(&priv->pd_dev);

  /* Attach the IRQ to the driver */

#if CONFIG_PIC32MX_NINTERFACES > 1
  ret = irq_attach(priv->pd_irq, pic32mx_interrupt);
#else
  ret = irq_attach(PIC32MX_IRQ_ETH, pic32mx_interrupt);
#endif
  if (ret != 0)
    {
      /* We could not attach the ISR to the the interrupt */

      return -EAGAIN;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->pd_dev);
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

#if CONFIG_PIC32MX_NINTERFACES == 1
void up_netinitialize(void)
{
  (void)pic32mx_ethinitialize(0);
}
#endif
#endif /* CHIP_NETHERNET > 0 */
#endif /* CONFIG_NET && CONFIG_PIC32MX_ETHERNET */
