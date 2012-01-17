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

#include <arch/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "pic32mx-config.h"
#include "pic32mx-ethernet.h"
#include "pic32mx-internal.h"

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

/* CONFIG_NET_MULTIBUFFER is required */

#ifndef CONFIG_NET_MULTIBUFFER
#  error "CONFIG_NET_MULTIBUFFER=y is required"
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

/* Use defaults if the number of discriptors is not provided */

#ifndef CONFIG_NET_NTXDESC
#  define CONFIG_NET_NTXDESC 2
#endif

#ifndef CONFIG_NET_NRXDESC
#  define CONFIG_NET_NRXDESC 4
#endif

/* The number of buffers will, then, be one for each descriptor plus one extra */

#define PIC32MX_NBUFFERS (CONFIG_NET_NRXDESC + CONFIG_NET_NTXDESC + 1)

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

#define ETH_RXINTS             (ETH_INT_RXOVFLW | ETH_INT_RXBUFNA | ETH_INT_RXDONE | ETH_INT_RXBUSE)
#define ETH_TXINTS             (ETH_INT_TXABORT | ETH_INT_TXDONE | ETH_INT_TXBUSE)

/* Misc. Helpers ***********************************************************/

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)priv->pd_dev.d_buf)

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

/* Misc Helper Macros *******************************************************/

#define PHYS_ADDR(va) ((uint32_t)(va) & 0x1fffffff)
#define VIRT_ADDR(pa) (KSEG1_BASE | (uint32_t)(pa))

/* Ever-present MIN and MAX macros */

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* EMAC statistics (debug only) */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
struct pic32mx_statistics_s
{
  uint32_t rx_done;        /* Rx done interrupts */
  uint32_t rx_errors;      /* Number of Rx error interrupts */
  uint32_t rx_ovflw;       /*   Number of Rx overflow error interrupts */
  uint32_t rx_bufna;       /*   Number of Rx buffer not available errors */
  uint32_t rx_buse;        /*   Number of Rx BVCI bus errors */
  uint32_t rx_packets;     /* Number of packets received (sum of the following): */
  uint32_t rx_ip;          /*   Number of Rx IP packets received */
  uint32_t rx_arp;         /*   Number of Rx ARP packets received */
  uint32_t rx_dropped;     /*   Number of dropped, unsupported Rx packets */
  uint32_t rx_pkterr;      /*   Number of dropped, error in Rx descriptor */
  uint32_t rx_pktsize;     /*   Number of dropped, too small or too big */
  uint32_t rx_fragment;    /*   Number of dropped, packet fragments */
  uint32_t tx_done;        /* Tx done interrupts */
  uint32_t tx_errors;      /* Number of Tx error interrupts (OR of other errors) */
  uint32_t tx_abort;       /*   Number of Tx abort interrupts */
  uint32_t tx_buse;        /*   Number of Tx bus errors */
  uint32_t tx_packets;     /* Number of Tx packets queued */
  uint32_t tx_pending;     /* Number of Tx packets that had to wait for a TxDesc */
  uint32_t tx_unpend;      /* Number of pending Tx packets that were sent */
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
  uint32_t   pd_base;           /* Ethernet controller base address */
  int        pd_irq;            /* Ethernet controller IRQ vector number */
  int        pd_irqsrc;         /* Ethernet controller IRQ source number */
#endif

  bool       pd_ifup;           /* true:ifup false:ifdown */
  bool       pd_mode;           /* speed/duplex */
  bool       pd_txpending;      /* There is a pending Tx in pd_dev */
#ifdef PIC32MX_HAVE_PHY
  uint8_t    pd_phyaddr;        /* PHY device address */
#endif
  uint32_t   pd_inten;          /* Shadow copy of INTEN register */
  WDOG_ID    pd_txpoll;         /* TX poll timer */
  WDOG_ID    pd_txtimeout;      /* TX timeout timer */

  sq_queue_t pd_freebuffers;    /* The free buffer list */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
  struct pic32mx_statistics_s pd_stat;
#endif

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s pd_dev;  /* Interface understood by uIP */

  /* Descriptors and packet buffers */

  struct pic32mx_rxdesc_s pd_rxdesc[CONFIG_NET_NRXDESC];
  struct pic32mx_txdesc_s pd_txdesc[CONFIG_NET_NTXDESC];
  uint8_t  pd_buffers[PIC32MX_NBUFFERS * CONFIG_NET_BUFSIZE];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Array of ethernet driver status structures */

static struct pic32mx_driver_s g_ethdrvr[CONFIG_PIC32MX_NINTERFACES];

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

/* Buffer and descriptor management */

static inline void pic32mx_bufferinit(struct pic32mx_driver_s *priv);
static uint8_t *pic32mx_allocbuffer(struct pic32mx_driver_s *priv);
static void pic32mx_freebuffer(struct pic32mx_driver_s *priv, uint8_t *buffer);

static inline void pic32mx_txdescinit(struct pic32mx_driver_s *priv);
static inline void pic32mx_rxdescinit(struct pic32mx_driver_s *priv);
static struct pic32mx_txdesc_s *pic32mx_txdesc(struct pic32mx_driver_s *priv);
static struct pic32mx_rxdesc_s *pic32mx_rxdesc(struct pic32mx_driver_s *priv);

/* Common TX logic */

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
 * Function: pic32mx_bufferinit
 *
 * Description:
 *   Initialize the buffers by placing them all in a free list
 *
 * Parameters:
 *   priv - Pointer to EMAC device driver structure 
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void pic32mx_bufferinit(struct pic32mx_driver_s *priv)
{
  uint8_t *buffer;
  int i;

  for (i = 0, buffer = priv->pd_buffers; i < PIC32MX_NBUFFERS; i++)
   {
     sq_addlast((sq_entry_t*)buffer, &priv->pd_freebuffers);
     buffer += CONFIG_NET_BUFSIZE;
   }
}

/****************************************************************************
 * Function: pic32mx_allocbuffer
 *
 * Description:
 *   Allocate one buffer by removing it from the free list
 *
 * Parameters:
 *   priv - Pointer to EMAC device driver structure 
 *
 * Returned Value:
 *   Pointer to the allocated buffer (or NULL on failure)
 *
 ****************************************************************************/

static uint8_t *pic32mx_allocbuffer(struct pic32mx_driver_s *priv)
{
  return (uint8_t*)sq_remfirst(&priv->pd_freebuffers);
}

/****************************************************************************
 * Function: pic32mx_freebuffer
 *
 * Description:
 *   Free one buffer by returning it to the free list
 *
 * Parameters:
 *   priv - Pointer to EMAC device driver structure 
 *
 * Returned Value:
 *   Pointer to the allocated buffer (or NULL on failure)
 *
 ****************************************************************************/

static void pic32mx_freebuffer(struct pic32mx_driver_s *priv, uint8_t *buffer)
{
   sq_addlast((sq_entry_t*)buffer, &priv->pd_freebuffers);
}

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
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void pic32mx_txdescinit(struct pic32mx_driver_s *priv)
{
  struct pic32mx_txdesc_s *txdesc;
  int i;

  /* Assign a buffer to each TX descriptor.  For now, just mark each TX
   * descriptor as owned by softare andnot linked.
   */

  for (i = 0; i < CONFIG_NET_NTXDESC; i++)
    {
      txdesc          = &priv->pd_txdesc[i];
      txdesc->status  = TXDESC_STATUS_SOWN;
      txdesc->address = 0;
      txdesc->tsv1    = 0;
      txdesc->tsv2    = 0;
      txdesc->nexted  = (uint32_t)&priv->pd_txdesc[i+1];
    }

  /* Terminate the last entry with a null NEXTED pointer */

  txdesc[CONFIG_NET_NTXDESC-1].nexted = 0;

  /* Update the ETHTXST register with the physical address of the head of
   * the TX descriptors list.
   */

  pic32mx_putreg(PHYS_ADDR(priv->pd_txdesc), PIC32MX_ETH_TXST);
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
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void pic32mx_rxdescinit(struct pic32mx_driver_s *priv)
{
  struct pic32mx_rxdesc_s *rxdesc;
  int i;

  /* Prepare a list of RX descriptors populated with valid buffers for
   * messages to be received. Properly update the NPV, EOWN = 1 and
   * DATA_BUFFER_ADDRESS fields in the RX descriptors. The
   * DATA_BUFFER_ADDRESS should contain the physical address of the
   * corresponding RX buffer.
   */

  rxdesc  = priv->pd_rxdesc;

  for (i = 0; i < (CONFIG_NET_NRXDESC-1); i++)
    {
      rxdesc          = &priv->pd_rxdesc[i];
      rxdesc->status  = RXDESC_STATUS_EOWN | TXDESC_STATUS_NPV;
      rxdesc->address = PHYS_ADDR(pic32mx_allocbuffer(priv));
      rxdesc->rsv1    = 0;
      rxdesc->rsv2    = 0;
      rxdesc->nexted  = (uint32_t)&priv->pd_rxdesc[i+1];
    }

  rxdesc              = &priv->pd_rxdesc[CONFIG_NET_NRXDESC-1];
  rxdesc->status      = RXDESC_STATUS_EOWN;
  rxdesc->address     = PHYS_ADDR(pic32mx_allocbuffer(priv));
  rxdesc->rsv1        = 0;
  rxdesc->rsv2        = 0;
  rxdesc->nexted      = 0;

  /* Update the ETHRXST register with the physical address of the head of the
   * RX descriptors list.
   */

  pic32mx_putreg(PHYS_ADDR(priv->pd_rxdesc), PIC32MX_ETH_RXST);
}

/****************************************************************************
 * Function: pic32mx_txdesc
 *
 * Description:
 *   Check if a free TX descriptor is available.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   A pointer to an available TX descriptor on success; NULL on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static struct pic32mx_txdesc_s *pic32mx_txdesc(struct pic32mx_driver_s *priv)
{
  struct pic32mx_txdesc_s *txdesc;
  int i;

  /* Inspect the list of TX descriptors to see if the EOWN bit is cleared. If it
   * is, this descriptor is now under software control and the message was
   * transmitted.
   */
 
  for (i = 0; i < CONFIG_NET_NTXDESC; i++)
    {
      /* Check if software owns this descriptor */

      txdesc = &priv->pd_txdesc[i];
      if ((txdesc->status & TXDESC_STATUS_EOWN) == 0)
        {
          /* Yes.. return a pointer to the desciptor */

          return txdesc;
        }
    }

  /* All descriptors are owned by the Ethernet controller.. return NULL */

  return NULL;
}

/****************************************************************************
 * Function: pic32mx_rxdesc
 *
 * Description:
 *   Check if a RX descriptor is owned by the software.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   A pointer to the RX descriptor on success; NULL on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static struct pic32mx_rxdesc_s *pic32mx_rxdesc(struct pic32mx_driver_s *priv)
{
  struct pic32mx_rxdesc_s *rxdesc;
  int i;

  /* Inspect the list of RX descriptors to see if the EOWN bit is cleared.
   * If it is, this descriptor is now under software control and a message was
   * received. Use SOP and EOP to extract the message, use BYTE_COUNT, RXF_RSV,
   * RSV and PKT_CHECKSUM to get the message characteristics.
   */

  for (i = 0; i < CONFIG_NET_NRXDESC; i++)
    {
      /* Check if software owns this descriptor */

      rxdesc = &priv->pd_rxdesc[i];
      if ((rxdesc->status & RXDESC_STATUS_EOWN) == 0)
        {
          /* Yes.. return a pointer to the desciptor */

          return rxdesc;
        }
    }

  /* All descriptors are owned by the Ethernet controller.. return NULL */

  return NULL;
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
  struct pic32mx_txdesc_s *txdesc;

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  DEBUGASSERT(priv->pd_dev.d_buf && priv->pd_dev.d_len < CONFIG_NET_BUFSIZE);

  /* Increment statistics and dump the packet *if so configured) */

  EMAC_STAT(priv, tx_packets);
  pic32mx_dumppacket("Transmit packet", priv->pd_dev.d_buf, priv->pd_dev.d_len);

  /* In order to transmit a message:
   *
   * The SOP, EOP, DATA_BUFFER_ADDRESS and BYTE_COUNT will be updated when a
   * particular message has to be transmitted. The DATA_BUFFER_ADDRESS will
   * contain the physical address of the message, the BYTE_COUNT message size.
   * SOP and EOP are set depending on how many packets are needed to transmit
   * the message.
   */

  /* Find the first available TX descriptor.  We are guaranteed that is will
   * not fail by upstream logic that assures that a TX packet is available
   * before polling uIP.
   */

  txdesc = pic32mx_txdesc(priv);
  DEBUGASSERT(txdesc != NULL);

  /* Remove the transmit buffer from the device structure and assign it to
   * the TX descriptor.
   */

  txdesc->address    = PHYS_ADDR(priv->pd_dev.d_buf);
  priv->pd_dev.d_buf = NULL;

  /* Set the BYTE_COUNT for in the TX descriptor with the number of bytes
   * contained in the buffer.
   */

  txdesc->tsv2      &= TXDESC_TSV2_BYTECOUNT_MASK;
  txdesc->tsv2      |= ((uint32_t)priv->pd_dev.d_len << TXDESC_TSV2_BYTECOUNT_SHIFT);
  priv->pd_dev.d_len = 0;

  /* Set EOWN = 1 to indicate that the packet belongs to Ethernet and set both
   * SOP and EOP to indicate that the packet both begins and ends with this
   * frame.
   */

  txdesc->status |= (TXDESC_STATUS_EOWN | TXDESC_STATUS_EOP | TXDESC_STATUS_SOP);

  /* Enable the transmission of the message by setting the TXRTS bit (ETHCON1:9). */

  pic32mx_putreg(ETH_CON1_TXRTS | ETH_CON1_ON, PIC32MX_ETH_CON1SET);

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

      if (pic32mx_txdesc(priv) == NULL || sq_empty(&priv->pd_freebuffers))
        {
          /* There are no more TX descriptors/buffers available.. stop the poll */

          return -EAGAIN;
        }
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
  struct pic32mx_txdesc_s *txdesc;

  /* Check if there is room in the device to hold another packet. */

  txdesc = pic32mx_txdesc(priv);
  if (txdesc != NULL)
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
  struct pic32mx_rxdesc_s *rxdesc;

  /* Loop while there are incoming packets to be processed, that is, while
   * the producer index is not equal to the consumer index.
   */

  for (;;)
    {
      /* Check if any RX descriptor has the EOWN bit cleared meaning that the
       * this descriptor is now under software control and a message was
       * received. 
       */

      rxdesc = pic32mx_rxdesc(priv);
      if (rxdesc == NULL)
        {
          /* All RX descriptors are owned by the Ethernet controller... we
           * are finished here.
           */

          return;
        }

      /* Update statistics */

      EMAC_STAT(priv, rx_packets);

      /* Get the packet length */

      priv->pd_dev.d_len = (rxdesc->rsv1 & RXDESC_RSV1_BYTECOUNT_MASK) >> RXDESC_RSV1_BYTECOUNT_SHIFT;

      /* Check for errors */

      if ((rxdesc->status & RXDESC_RSV1_OK) == 0)
        {
          nlldbg("Error. rxdesc: %08x\n", rxdesc->status);
          EMAC_STAT(priv, rx_pkterr);
        }

      /* If the packet length is greater then the buffer, then we cannot accept
       * the packet.  Also, since the DMA packet buffers are set up to
       * be the same size as our max packet size, any fragments also
       * imply that the packet is too big.
       */
 
      else if (priv->pd_dev.d_len > CONFIG_NET_BUFSIZE)
        {
          nlldbg("Too big. packet length: %d rxdesc: %08x\n", priv->pd_dev.d_len, rxdesc->status);
          EMAC_STAT(priv, rx_pktsize);
        }

      /* We don't have any logic here for reassembling packets from fragments. */

      else if ((rxdesc->status & (RXDESC_STATUS_EOP|RXDESC_STATUS_SOP)) != (RXDESC_STATUS_EOP|RXDESC_STATUS_SOP))
        {
          nlldbg("Fragment. packet length: %d rxdesc: %08x\n", priv->pd_dev.d_len, rxdesc->status);
          EMAC_STAT(priv, rx_fragment);
        }
      else
        {
          uint8_t *rxbuffer;

          /* Get the Rx buffer address from the Rx descriptor */
 
          priv->pd_dev.d_buf = (uint8_t*)VIRT_ADDR(rxdesc->address);

          /* Replace the buffer in the RX descriptor with a new one */

          rxbuffer = pic32mx_allocbuffer(priv);
          DEBUGASSERT(rxbuffer != NULL);
          rxdesc->address = PHYS_ADDR(rxbuffer);

          /* And give the RX descriptor back to the hardware */

          rxdesc->status = RXDESC_STATUS_EOWN | TXDESC_STATUS_NPV;
          rxdesc->rsv1   = 0;
          rxdesc->rsv1   = 0;

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

          /* Discard any buffers still attached to the device structure */

          priv->pd_dev.d_len = 0;
          if (priv->pd_dev.d_buf)
            {
              pic32mx_freebuffer(priv, priv->pd_dev.d_buf);
              priv->pd_dev.d_buf = NULL;
            }
        }
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
  struct pic32mx_txdesc_s *txdesc;
  int i;

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

  DEBUGASSERT(pic32mx_txdesc(priv) != NULL);

  /* Inspect the list of TX descriptors to see if the EOWN bit is cleared. If it
   * is, this descriptor is now under software control and the message was
   * transmitted. Use TSV to check for the transmission result.
   */

  for (i = 0; i < CONFIG_NET_NTXDESC; i++)
    {
      txdesc = &priv->pd_txdesc[i];

      /* Check if software owns this descriptor */

      if ((txdesc->status & TXDESC_STATUS_EOWN) == 0)
        {
          /* Yes.. Check if there is a buffer attached? */

          if (txdesc->address != 0)
            {
              /* Reset status */

              txdesc->status  = TXDESC_STATUS_SOWN;
              txdesc->tsv1    = 0;
              txdesc->tsv2    = 0;

              /* Free the TX buffer */

              pic32mx_freebuffer(priv, (uint8_t *)VIRT_ADDR(txdesc->address));
              txdesc->address = 0;
            }
        }
      txdesc  += TXDESC_SIZE;
    }

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
      /* Assign a buffer for the poll */

      DEBUGASSERT(priv->pd_dev.d_buf == NULL);
      priv->pd_dev.d_buf = pic32mx_allocbuffer(priv);
      if (priv->pd_dev.d_buf)
        {
          /* And perform the poll */

          (void)uip_poll(&priv->pd_dev, pic32mx_uiptxpoll);

          /* Free any buffer left attached after the poll */

          if (priv->pd_dev.d_buf != NULL)
            {
              pic32mx_freebuffer(priv, priv->pd_dev.d_buf);
              priv->pd_dev.d_buf = NULL;
            }
        }
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

  status = pic32mx_getreg(PIC32MX_ETH_IRQ);
  if (status != 0)
    {
      /* Clear all pending interrupts */

      pic32mx_putreg(status, PIC32MX_ETH_IRQCLR);

      /* Handle each pending interrupt **************************************/
      /* Receive Errors *****************************************************/
      /* RXOVFLW: Receive FIFO Over Flow Error.  RXOVFLW is set by the RXBM
       * Logic for an RX FIFO Overflow condition. It is cleared by either a
       * Reset or CPU write of a ‘1’ to the CLR register.
       */

      if ((status & ETH_INT_RXOVFLW) != 0)
        {
           nlldbg("RX Overrun. status: %08x\n", status);
           EMAC_STAT(priv, rx_errors);
           EMAC_STAT(priv, rx_ovflw);
        }

      /* RXBUFNA: Receive Buffer Not Available Interrupt.  This bit is set by
       * a RX Buffer Descriptor Overrun condition. It is cleared by either a
       * Reset or a CPU write of a ‘1’ to the CLR register.
       */

      if ((status & ETH_INT_RXBUFNA) != 0)
        {
          nlldbg("RX buffer descriptor overrun. status: %08x\n", status);
          EMAC_STAT(priv, rx_errors);
          EMAC_STAT(priv, rx_bufna);
        }

      /* RXBUSE: Receive BVCI Bus Error Interrupt.  This bit is set when the
       * RX DMA encounters a BVCI Bus error during a memory access. It is
       * cleared by either a Reset or CPU write of a ‘1’ to the CLR register.
       */

      if ((status & ETH_INT_RXBUSE) != 0)
        {
          nlldbg("RX BVCI bus error. status: %08x\n", status);
          EMAC_STAT(priv, rx_errors);
          EMAC_STAT(priv, rx_buse);
        }

      /* Receive Normal Events **********************************************/
      /* RXACT: Receive Activity Interrupt.  This bit is set whenever RX packet
       * data is stored in the RXBM FIFO. It is cleared by either a Reset or CPU
       * write of a ‘1’ to the CLR register.
       */

      /* PKTPEND: Packet Pending Interrupt.  This bit is set when the BUFCNT
       * counter has a value other than ‘0’. It is cleared by either a Reset
       * or by writing the BUFCDEC bit to decrement the BUFCNT counter.
       * Writing a ‘0’ or a ‘1’ has no effect.
       */

      /* RXDONE: Receive Done Interrupt.  This bit is set whenever an RX packet
       * is successfully received. It is cleared by either a Reset or CPU
       * write of a ‘1’ to the CLR register.
       */

      if ((status & ETH_INT_RXDONE) != 0)
        {
          EMAC_STAT(priv, rx_done);

          /* We have received at least one new incoming packet. */

          pic32mx_rxdone(priv);
        }

      /* Transmit Errors ****************************************************/
      /* TXABORT: Transmit Abort Condition Interrupt.  This bit is set when
       * the MAC aborts the transmission of a TX packet for one of the
       * following reasons:
       * - Jumbo TX packet abort
       * - Underrun abort
       * - Excessive defer abort
       * - Late collision abort
       * - Excessive collisions abort
       * This bit is cleared by either a Reset or CPU write of a ‘1’ to the
       * CLR register.
       */

      if ((status & ETH_INT_TXABORT) != 0)
        {
          nlldbg("TX abort. status: %08x\n", status);
          EMAC_STAT(priv, tx_errors);
          EMAC_STAT(priv, tx_abort);
        }

      /* TXBUSE: Transmit BVCI Bus Error Interrupt. This bit is set when the
       * TX DMA encounters a BVCI Bus error during a memory access. It is
       * cleared by either a Reset or CPU write of a ‘1’ to the CLR register.
       */

      if ((status & ETH_INT_TXBUSE) != 0)
        {
          nlldbg("TX BVCI bus error. status: %08x\n", status);
          EMAC_STAT(priv, tx_errors);
          EMAC_STAT(priv, tx_buse);
        }

      /* TXDONE: Transmit Done Interrupt.  This bit is set when the currently
       * transmitted TX packet completes transmission, and the Transmit
       * Status Vector is loaded into the first descriptor used for the
       * packet. It is cleared by either a Reset or CPU write of a ‘1’ to
       * the CLR register.
       */

      if ((status & ETH_INT_TXDONE) != 0)
        {
          EMAC_STAT(priv, tx_done);

          /* A packet transmission just completed */

          pic32mx_txdone(priv);
        }

      /* Watermark Events ***************************************************/
      /* EWMARK: Empty Watermark Interrupt.  This bit is set when the RX
       * Descriptor Buffer Count is less than or equal to the value in the
       * RXEWM bit (ETHRXWM:0-7) value. It is cleared by BUFCNT bit
       * (ETHSTAT:16-23) being incremented by hardware. Writing a ‘0’ or a ‘1’
       * has no effect.
       */

      /* FWMARK: Full Watermark Interrupt.  This bit is set when the RX 
       * escriptor Buffer Count is greater than or equal to the value in the
       * RXFWM bit (ETHRXWM:16-23) field. It is cleared by writing the BUFCDEC
       * (ETHCON1:0) bit to decrement the BUFCNT counter. Writing a ‘0’ or a
       * ‘1’ has no effect.
       */

    }

  /* Clear the pending interrupt */

# if CONFIG_PIC32MX_NINTERFACES > 1
  up_clrpend_irq(priv->pd_irqsrc);
# else
  up_clrpend_irq(PIC32MX_IRQSRC_ETH);
# endif

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

      /* Then poll uIP for new XMIT data (We are guaranteed to have a free
       * buffer here).
       */

      priv->pd_dev.d_buf = pic32mx_allocbuffer(priv);
      (void)uip_poll(&priv->pd_dev, pic32mx_uiptxpoll);

      /* Free any buffer left attached after the poll */

      if (priv->pd_dev.d_buf != NULL)
        {
          pic32mx_freebuffer(priv, priv->pd_dev.d_buf);
          priv->pd_dev.d_buf = NULL;
        }
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

  if (pic32mx_txdesc(priv) != NULL)
    {
      /* If so, update TCP timing states and poll uIP for new XMIT data. Hmmm..
       * might be bug here.  Does this mean if there is a transmit in progress,
       * we will missing TCP time state updates?
       */

      DEBUGASSERT(priv->pd_dev.d_buf == NULL);
      priv->pd_dev.d_buf = pic32mx_allocbuffer(priv);
      if (priv->pd_dev.d_buf != NULL)
        {
          /* And perform the poll */

          (void)uip_timer(&priv->pd_dev, pic32mx_uiptxpoll, PIC32MX_POLLHSEC);

          /* Free any buffer left attached after the poll */

          if (priv->pd_dev.d_buf != NULL)
            {
              pic32mx_freebuffer(priv, priv->pd_dev.d_buf);
              priv->pd_dev.d_buf = NULL;
            }
        }
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

  /* MAC Initialization *****************************************************/
  /* Use the configuration fuse setting FETHIO bit (DEVCFG3:25) to detect
   * the alternate/default I/O configuration
   */
#warning "Missing logic"

  /* Use the configuration fuse setting FMIIEN (DEVCFG3:24) to detect the
   * MII/RMII operation mode.
   */

#if CONFIG_PIC32MX_FMIIEN
#endif

  /* No GPIO pin configuration is required.  Enabling the Ethernet Controller
   * will configure the I/O pin direction as defined by the Ethernet Controller
   * control bits. The port TRIS and LATCH registers will be overridden.
   *
   * I/O Pin    MII     RMII   Pin  Description
   *   Name   Required Required Type
   * EMDC     Yes      Yes      O    Ethernet MII Management Clock
   * EMDIO    Yes      Yes      I/O  Ethernet MII Management IO
   * ETXCLK   Yes      No       I    Ethernet MII TX Clock
   * ETXEN    Yes      Yes      O    Ethernet Transmit Enable
   * ETXD0    Yes      Yes      O    Ethernet Data Transmit 0
   * ETXD1    Yes      Yes      O    Ethernet Data Transmit 1
   * ETXD2    Yes      No       O    Ethernet Data Transmit 2
   * ETXD3    Yes      No       O    Ethernet Data Transmit 3
   * ETXERR   Yes      No       O    Ethernet Transmit Error
   * ERXCLK   Yes      No       I    Ethernet MII RX Clock
   * EREF_CLK No       Yes      I    Ethernet RMII Ref Clock
   * ERXDV    Yes      No       I    Ethernet MII Receive Data Valid
   * ECRS_DV  No       Yes      I    Ethernet RMII Carrier Sense/Receive Data Valid
   * ERXD0    Yes      Yes      I    Ethernet Data Receive 0
   * ERXD1    Yes      Yes      I    Ethernet Data Receive 1
   * ERXD2    Yes      No       I    Ethernet Data Receive 2
   * ERXD3    Yes      No       I    Ethernet Data Receive 3
   * ERXERR   Yes      Yes      I    Ethernet Receive Error
   * ECRS     Yes      No       I    Ethernet Carrier Sense
   * ECOL     Yes      No       I    Ethernet Collision Detected
   *
   * All that is required is to assure that the pins are initialized as
   * digital, all the pins used by the (normally only those pins that
   * have shared analog functionality need to be configured).
   */
#warning "Missing logic"

  /* Initialize the MIIM interface
   *
   * If the RMII operation is selected, reset the RMII module by using the
   * RESETRMII (EMAC1SUPP:11) bit and set the proper speed in the SPEEDRMII
   * bit (EMAC1SUPP:8) bit.
   */

#if CONFIG_PIC32MX_FMIIEN == 0
#warning "Missing logic"
#endif

   /* Issue an MIIM block reset, by setting the RESETMGMT (EMAC1MCFG:15) bit,
    * and then clear the reset bit.
    */
#warning "Missing logic"

   /* Select a proper divider in the CLKSEL bit (EMAC1CFG:2-5) for the MIIM
    * PHY communication based on the system running clock frequency and the
    * external PHY supported clock.
    *
    * MII configuration: host clocked divided per board.h, no suppress
    * preamble, no scan increment.
    */

  pic32mx_putreg(EMAC1_MCFG_CLKSEL_DIV, PIC32MX_EMAC1_MCFG);

  /* PHY Initialization *****************************************************/
  /* Initialize the PHY and wait for the link to be established */

  ret = pic32mx_phyinit(priv);
  if (ret != 0)
    {
      ndbg("pic32mx_phyinit failed: %d\n", ret);
      return ret;
    }

  /* MAC Configuration ******************************************************/
  /* Set other misc configuration-related registers to default values */

  pic32mx_putreg(0, PIC32MX_EMAC1_CFG2);
  pic32mx_putreg(0, PIC32MX_ETH_SUPP);
  pic32mx_putreg(0, PIC32MX_EMAC1_TEST);

  /* Having available the Duplex and Speed settings, configure the MAC
   * accordingly, using the following steps:
   *
   * Enable the RXENABLE bit (EMAC1CFG1:0), selecting both the TXPAUSE and
   * RXPAUSE bit (EMAC1CFG1:2-3) (the PIC32 MAC supports both).
   */
#warning "Missing logic"

   /* Select the desired auto-padding and CRC capabilities, and the enabling
    * of the huge frames and the Duplex type in the EMAC1CFG2 register.
    */
#warning "Missing logic"

   /* Program EMAC1IPGT with the back-to-back inter-packet gap */

   /* Use EMAC1IPGR for setting the non back-to-back inter-packet gap */

    pic32mx_putreg(((12 << EMAC1_IPGR_GAP1_SHIFT) | (12 << EMAC1_IPGR_GAP2_SHIFT)),
                   PIC32MX_EMAC1_IPGR);

   /* Set the collision window and the maximum number of retransmissions in
    * EMAC1CLRT.
    */

  pic32mx_putreg(((15 << EMAC1_CLRT_RETX_SHIFT) | (55 << EMAC1_CLRT_CWINDOW_SHIFT)),
                 PIC32MX_EMAC1_CLRT);

   /* Set the maximum frame length in EMAC1MAXF.  "This field resets to
    * 0x05EE, which represents a maximum receive frame of 1518 octets. An
    * untagged maximum size Ethernet frame is 1518 octets. A tagged frame adds
    * four octets for a total of 1522 octets. If a shorter/longer maximum
    * length restriction is desired, program this 16-bit field.
    */

  pic32mx_putreg(CONFIG_NET_BUFSIZE, PIC32MX_EMAC1_MAXF);

   /*  Configure the MAC station address in the EMAC1SA0, EMAC1SA1 and
    * EMAC1SA2 registers (these registers are loaded at reset from the
    * factory preprogrammed station address).
    */

#if 0
  regval = (uint32_t)priv->pd_dev.d_mac.ether_addr_octet[5] << 8 |
           (uint32_t)priv->pd_dev.d_mac.ether_addr_octet[4];
  pic32mx_putreg(regval, PIC32MX_EMAC1_SA0);

  regval = (uint32_t)priv->pd_dev.d_mac.ether_addr_octet[3] << 8 |
           (uint32_t)priv->pd_dev.d_mac.ether_addr_octet[2];
  pic32mx_putreg(regval, PIC32MX_EMAC1_SA1);

  regval = (uint32_t)priv->pd_dev.d_mac.ether_addr_octet[1] << 8 |
           (uint32_t)priv->pd_dev.d_mac.ether_addr_octet[0];
  pic32mx_putreg(regval, PIC32MX_EMAC1_SA2);
#else
  regval = pic32mx_getreg(PIC32MX_EMAC1_SA0);
  priv->pd_dev.d_mac.ether_addr_octet[4] = (uint32_t)(regval & 0xff);
  priv->pd_dev.d_mac.ether_addr_octet[5] = (uint32_t)((regval >> 8) & 0xff);

  regval = pic32mx_getreg(PIC32MX_EMAC1_SA1);
  priv->pd_dev.d_mac.ether_addr_octet[2] = (uint32_t)(regval & 0xff);
  priv->pd_dev.d_mac.ether_addr_octet[3] = (uint32_t)((regval >> 8) & 0xff);

  regval = pic32mx_getreg(PIC32MX_EMAC1_SA2);
  priv->pd_dev.d_mac.ether_addr_octet[0] = (uint32_t)(regval & 0xff);
  priv->pd_dev.d_mac.ether_addr_octet[1] = (uint32_t)((regval >> 8) & 0xff);
#endif

  /* Continue Ethernet Controller Initialization ****************************/
  /* If planning to turn on the flow control, update the PTV value
   *(ETHCON1:16-31).
   */

  /* If using the auto-flow control, set the full and empty watermarks: RXFWM
   * and RXEWM (ETHRXWM:16-23 and ETHRXWM:0-7).
   */

  /* If needed, enable the auto-flow control by setting AUTOFC (ETHCON1:7). */

  /* Set the RX filters by updating the ETHHT0, ETHHT1, ETHPMM0, ETHPMM1,
   * ETHPMCS and ETHRXFC registers.
   */
#warning "Missing logic"

  /* Set the size of the RX buffers in the RXBUFSZ bit (ETHCON2:4-10) (all
   * receive descriptors use the same buffer size). Keep in mind that using
   * packets that are too small leads to packet fragmentation and has a
   * noticeable impact on the performance.
   */
#warning "Missing logic"

  /* Initialize the buffer list */

  pic32mx_bufferinit(priv);

  /* Initialize the TX descriptor list */

  pic32mx_txdescinit(priv);

  /* Initialize the RX descriptor list */

  pic32mx_rxdescinit(priv);

  /* Enable the Ethernet Controller by setting the ON bit (ETHCON1:15).
   * Enable the receiving of messages by setting the RXEN bit (ETHCON1:8).
   */

  pic32mx_putreg(ETH_CON1_RXEN | ETH_CON1_ON, PIC32MX_ETH_CON1SET);

  /* Initialize Ethernet interface for the PHY setup */

  pic32mx_macmode(priv->pd_mode);

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

  pic32mx_putreg(0xffffffff, PIC32MX_ETH_IRQCLR);

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

  /* Otherwise, enable all Rx interrupts.  Tx interrupts, SOFTINT and WoL are
   * excluded.  Tx interrupts will not be enabled until there is data to be
   * sent.
   */

  priv->pd_inten = ETH_RXINTS;
  pic32mx_putreg(ETH_RXINTS, PIC32MX_ETH_IENSET);

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

      if (pic32mx_txdesc(priv) != NULL)
        {
          /* If so, then poll uIP for new XMIT data.  First allocate a buffer
           * to perform the poll
           */

          DEBUGASSERT(priv->pd_dev.d_buf == NULL);
          priv->pd_dev.d_buf = pic32mx_allocbuffer(priv);
          if (priv->pd_dev.d_buf)
            {
              /* And perform the poll */

              (void)uip_poll(&priv->pd_dev, pic32mx_uiptxpoll);

              /* Free any buffer left attached after the poll */

              if (priv->pd_dev.d_buf != NULL)
                {
                  pic32mx_freebuffer(priv, priv->pd_dev.d_buf);
                  priv->pd_dev.d_buf = NULL;
                }
            }
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

  /* Clear any ongoing PHY command bits */

  pic32mx_putreg(0, PIC32MX_EMAC1_MCMD);

  /* Enter RMII mode and select 100 MBPS support */

  pic32mx_putreg(ETH_CMD_RMII, PIC32MX_ETH_CMD);
  pic32mx_putreg(EMAC1_SUPP_SPEEDRMII, PIC32MX_ETH_SUPP);

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

  /* Set the MII/RMII operation mode. This usually requires access to a
   * vendor-specific control register.
   */

  /* Set the normal, swapped or auto (preferred) MDIX. This usually requires
   * access to a vendor-specific control register.
   */

  /* Check the PHY capabilities by investigating the Status Register 1. */

  /* Check for preamble suppression support */

  phyreg = pic32mx_phyread(phyaddr, MII_MSR);
  if ((phyreg & MII_MSR_MFRAMESUPPRESS) != 0)
    {
      /* The PHY supports preamble suppression */

      regval  = pic32mx_getreg(PIC32MX_EMAC1_MCFG);
      regval |= EMAC1_MCFG_NOPRE;
      pic32mx_putreg(regval, PIC32MX_EMAC1_MCFG);
    }

  /* Are we configured to do auto-negotiation?
   * 
   * Preferably the auto-negotiation should be selected if the PHY supports
   * it. Expose the supported capabilities: Half/Full Duplex, 10BaseT/100Base
   * TX, etc. (Extended Register 4). Start the negotiation (Control Register
   * 0) and wait for the negotiation complete and get the link partner
   * capabilities (Extended Register 5) and negotiation result (vendor-
   * specific register).
   */

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
  /* Set up the fixed PHY configuration
   *
   * If auto-negotiation is not supported/selected, update the PHY Duplex and
   * Speed settings directly (use Control Register 0 and possibly some vendor-
   * pecific registers).
   */

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

  /* Ethernet Controller Initialization *************************************/
  /* Disable Ethernet interrupts in the EVIC */

#if CONFIG_PIC32MX_NINTERFACES > 1
  up_disable_irq(priv->pd_irqsrc);
#else
  up_disable_irq(PIC32MX_IRQSRC_ETH);
#endif

  /* Turn the Ethernet Controller off: Clear the ON, RXEN and TXRTS bits */

  pic32mx_putreg(ETH_CON1_RXEN | ETH_CON1_TXRTS | ETH_CON1_ON, PIC32MX_ETH_CON1CLR);

  /* Wait activity abort by polling the ETHBUSY bit */

  while ((pic32mx_getreg(PIC32MX_ETH_STAT) & ETH_STAT_ETHBUSY) != 0);

  /* Clear the Ethernet Interrupt Flag (ETHIF) bit in the Interrupts module */

#if CONFIG_PIC32MX_NINTERFACES > 1
  up_pending_irq(priv->pd_irqsrc);
#else
  up_pending_irq(PIC32MX_IRQSRC_ETH);
#endif

  /* Disable any Ethernet Controller interrupt generation by clearing the IEN
   * register.
   */

  pic32mx_putreg(ETH_INT_ALLINTS, PIC32MX_ETH_IENCLR);

  /* Clear the TX and RX start addresses by using ETHTXSTCLR and ETHRXSTCLR */

  pic32mx_putreg(0xffffffff, PIC32MX_ETH_TXSTCLR);
  pic32mx_putreg(0xffffffff, PIC32MX_ETH_RXSTSET);

  /* MAC Initialization *****************************************************/
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
  int ret;

  DEBUGASSERT(intf < CONFIG_PIC32MX_NINTERFACES);
  priv = &g_ethdrvr[intf];

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

  /* Reset the Ethernet controller and leave in the ifdown state.  The
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
