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
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/net/mii.h>

#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uipopt.h>
#include <nuttx/net/uip/uip-arp.h>
#include <nuttx/net/uip/uip-arch.h>

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

/* Use defaults if the number of discriptors is not provided */

#ifndef CONFIG_NET_NTXDESC
#  define CONFIG_NET_NTXDESC 2
#endif

#if CONFIG_NET_NTXDESC > 255
#  error "The number of TX descriptors exceeds the range of a uint8_t index"
#endif

#ifndef CONFIG_NET_NRXDESC
#  define CONFIG_NET_NRXDESC 4
#endif

/* Make sure that the size of each buffer is a multiple of 4 bytes.  This
 * will force alignment of all buffers to 4-byte boundaries (this is needed
 * by the queuing logic which will cast each buffer address to a pointer
 * type).
 */

#define PIC32MX_ALIGNED_BUFSIZE ((CONFIG_NET_BUFSIZE + 3) & ~3)

/* The number of buffers will, then, be one for each descriptor plus one extra */

#define PIC32MX_NBUFFERS (CONFIG_NET_NRXDESC + CONFIG_NET_NTXDESC + 1)

/* Debug Configuration *****************************************************/
/* Register/Descriptor debug -- can only happen of CONFIG_DEBUG is selected.
 * This will probably generate much more output than you care to see.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_NET_REGDEBUG
#  undef CONFIG_NET_DESCDEBUG
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

/* PHY timout = 1 minute */

#define PIC32MX_MIITIMEOUT     (666666)

/* Ethernet MII clocking.
 *
 * The clock divider used to create the MII Management Clock (MDC).  The MIIM
 * module uses the SYSCLK as an input clock.  According to the IEEE 802.3
 * Specification this should be no faster than 2.5 MHz. However, some PHYs
 * support clock rates up to 12.5 MHz.
 *
 * The board.h file provides the "ideal" divisor as BOARD_EMAC_MIIM_DIV.  We
 * pick the closest, actual divisor greater than or equal to this.
 */

#if BOARD_EMAC_MIIM_DIV <= 4
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV4
#elif BOARD_EMAC_MIIM_DIV <= 6
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV6
#elif BOARD_EMAC_MIIM_DIV <= 8
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV8
#elif BOARD_EMAC_MIIM_DIV <= 10
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV10
#elif BOARD_EMAC_MIIM_DIV <= 14
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV14
#elif BOARD_EMAC_MIIM_DIV <= 20
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV20
#elif BOARD_EMAC_MIIM_DIV <= 40
#  define EMAC1_MCFG_CLKSEL_DIV EMAC1_MCFG_CLKSEL_DIV40
#else
#  error "MIIM divider cannot be realized"
#endif

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
  bool       pd_txpending;      /* There is a pending Tx in pd_dev */
  bool       pd_polling;        /* Avoid concurrent attempts to poll */
  uint8_t    pd_mode;           /* Speed/duplex */
#ifdef PIC32MX_HAVE_PHY
  uint8_t    pd_phyaddr;        /* PHY device address */
#endif
  uint8_t    pd_txnext;         /* Index to the next Tx descriptor */
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
  uint8_t pd_buffers[PIC32MX_NBUFFERS * PIC32MX_ALIGNED_BUFSIZE];
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

#ifdef CONFIG_NET_DESCDEBUG
static void pic32mx_dumptxdesc(struct pic32mx_txdesc_s *txdesc, const char *msg);
static void pic32mx_dumprxdesc(struct pic32mx_rxdesc_s *rxdesc, const char *msg);
#else
# define pic32mx_dumptxdesc(txdesc,msg)
# define pic32mx_dumprxdesc(rxdesc,msg)
#endif

static inline void pic32mx_bufferinit(struct pic32mx_driver_s *priv);
static uint8_t *pic32mx_allocbuffer(struct pic32mx_driver_s *priv);
static void pic32mx_freebuffer(struct pic32mx_driver_s *priv, uint8_t *buffer);

static inline void pic32mx_txdescinit(struct pic32mx_driver_s *priv);
static inline void pic32mx_rxdescinit(struct pic32mx_driver_s *priv);
static inline struct pic32mx_txdesc_s *pic32mx_txdesc(struct pic32mx_driver_s *priv);
static inline void pic32mx_txnext(struct pic32mx_driver_s *priv);
static inline void pic32mx_rxreturn(struct pic32mx_rxdesc_s *rxdesc);
static struct pic32mx_rxdesc_s *pic32mx_rxdesc(struct pic32mx_driver_s *priv);

/* Common TX logic */

static int  pic32mx_transmit(struct pic32mx_driver_s *priv);
static int  pic32mx_uiptxpoll(struct uip_driver_s *dev);
static void pic32mx_poll(struct pic32mx_driver_s *priv);
static void pic32mx_timerpoll(struct pic32mx_driver_s *priv);

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

static void pic32mx_phybusywait(void);
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
  lldbg("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
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

              lldbg("[repeats %d more times]\n", count);
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
 * Function: pic32mx_dumptxdesc
 *
 * Description:
 *   Dump the contents of the specified TX descriptor
 *
 * Parameters:
 *   txdesc - Pointer to the TX descriptor to dump
 *   msg    - Annotation for the TX descriptor
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_DESCDEBUG
static void pic32mx_dumptxdesc(struct pic32mx_txdesc_s *txdesc, const char *msg)
{
  lldbg("TX Descriptor [%p]: %s\n", txdesc, msg);
  lldbg("   status: %08x\n", txdesc->status);
  lldbg("  address: %08x [%08x]\n", txdesc->address, VIRT_ADDR(txdesc->address));
  lldbg("     tsv1: %08x\n", txdesc->tsv1);
  lldbg("     tsv2: %08x\n", txdesc->tsv2);
  lldbg("   nexted: %08x [%08x]\n", txdesc->nexted, VIRT_ADDR(txdesc->nexted));
}
#endif

/****************************************************************************
 * Function: pic32mx_dumprxdesc
 *
 * Description:
 *   Dump the contents of the specified RX descriptor
 *
 * Parameters:
 *   txdesc - Pointer to the RX descriptor to dump
 *   msg    - Annotation for the RX descriptor
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_DESCDEBUG
static void pic32mx_dumprxdesc(struct pic32mx_rxdesc_s *rxdesc, const char *msg)
{
  lldbg("RX Descriptor [%p]: %s\n", rxdesc, msg);
  lldbg("   status: %08x\n", rxdesc->status);
  lldbg("  address: %08x [%08x]\n", rxdesc->address, VIRT_ADDR(rxdesc->address));
  lldbg("     rsv1: %08x\n", rxdesc->rsv1);
  lldbg("     rsv2: %08x\n", rxdesc->rsv2);
  lldbg("   nexted: %08x [%08x]\n", rxdesc->nexted, VIRT_ADDR(rxdesc->nexted));
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
     /* Add the buffer to the end of the list of free buffers */

     sq_addlast((sq_entry_t*)buffer, &priv->pd_freebuffers);

     /* Get the address of the next buffer */
 
     buffer += PIC32MX_ALIGNED_BUFSIZE;
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
  /* Return the next free buffer from the head of the free buffer list */

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
  /* Add the buffer to the end of the free buffer list */

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
      /* Point to the next entry */

      txdesc          = &priv->pd_txdesc[i];

      /* Initialize the buffer.  It is idle, owned by software and has
       * no buffer assigned to it.
       */

      txdesc->status  = TXDESC_STATUS_SOWN | TXDESC_STATUS_NPV;
      txdesc->address = 0;
      txdesc->tsv1    = 0;
      txdesc->tsv2    = 0;

      /* Set the NEXTED pointer.  If this is the last descriptor in the
       * list, then set the NEXTED pointer back to the first entry,
       * creating a ring.
       */

      if (i == (CONFIG_NET_NRXDESC-1))
        {
          txdesc->nexted = PHYS_ADDR(priv->pd_txdesc);
        }
      else
        {
          txdesc->nexted = PHYS_ADDR(&priv->pd_txdesc[i+1]);
        }

      pic32mx_dumptxdesc(txdesc, "Initial");
    }

  /* Position the Tx index to the first descriptor in the ring */

  priv->pd_txnext = 0;

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

  for (i = 0; i < CONFIG_NET_NRXDESC; i++)
    {
      /* Point to the next entry */

      rxdesc = &priv->pd_rxdesc[i];

      /* Initialize the descriptor.  Assign it a buffer and make it ready
       * for reception.
       */

      rxdesc->rsv1    = 0;
      rxdesc->rsv2    = 0;
      rxdesc->address = PHYS_ADDR(pic32mx_allocbuffer(priv));
      rxdesc->status  = RXDESC_STATUS_EOWN | TXDESC_STATUS_NPV;

      /* Set the NEXTED pointer.  If this is the last descriptor in the
       * list, then set the NEXTED pointer back to the first entry,
       * creating a ring.
       */

      if (i == (CONFIG_NET_NRXDESC-1))
        {
          rxdesc->nexted = PHYS_ADDR(priv->pd_rxdesc);
        }
      else
        {
          rxdesc->nexted = PHYS_ADDR(&priv->pd_rxdesc[i+1]);
        }

      pic32mx_dumprxdesc(rxdesc, "Initial");
    }

  /* Update the ETHRXST register with the physical address of the head of the
   * RX descriptors list.
   */

  pic32mx_putreg(PHYS_ADDR(priv->pd_rxdesc), PIC32MX_ETH_RXST);
}

/****************************************************************************
 * Function: pic32mx_txdesc
 *
 * Description:
 *   Check if the next Tx descriptor is available.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   A pointer to the next available Tx descriptor on success; NULL if the
 *   next Tx dscriptor is not available.
 *
 ****************************************************************************/

static inline struct pic32mx_txdesc_s *pic32mx_txdesc(struct pic32mx_driver_s *priv)
{
  struct pic32mx_txdesc_s *txdesc;

  /* Get a reference to the next Tx descriptor in the ring */

  txdesc = &priv->pd_txdesc[priv->pd_txnext];

  /* Check if the EOWN bit is cleared. If it is, this descriptor is now under
   * software control and the message has been transmitted.
   *
   * Also check that the buffer address is NULL.  There is a race condition
   * in that the hardware may have completed the transfer, but there may
   * still be a valid buffer attached to the Tx descriptor because we have
   * not yet processed the Tx done condition.  We will know that the Tx
   * done condition has been processed when the buffer has been freed and
   * reset to zero.
   */
 
  if ((txdesc->status & TXDESC_STATUS_EOWN) == 0 && txdesc->address == 0)
    {
      /* Yes.. return a pointer to the descriptor */

      return txdesc;
    }

  /* The next Tx descriptor is still owned by the Ethernet controller.. the
   * Tx ring if full and cannot be used now.  Return NULL.
   */

  return NULL;
}

/****************************************************************************
 * Function: pic32mx_txnext
 *
 * Description:
 *   After the next Tx descriptor has been given to the hardware, update the
 *   index to the next Tx descriptor in the ring.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void pic32mx_txnext(struct pic32mx_driver_s *priv)
{
  /* Increment the index to the next Tx descriptor in the ring */

  int txnext = priv->pd_txnext + 1;

  /* If the new index would go beyond the end of the allocated descriptors
   * for the Tx ring, then reset to first descriptor.
   */

  if (txnext >= CONFIG_NET_NTXDESC)
    {
      txnext = 0;
    }

  /* Save the index to the next Tx descriptor */

  priv->pd_txnext = txnext;
}

/****************************************************************************
 * Function: pic32mx_rxreturn
 *
 * Description:
 *   Return an RX descriptor to the hardware.
 *
 * Parameters:
 *   rxdesc - Reference to the RX descriptor to be returned
 *
 * Returned Value:
 *    None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void pic32mx_rxreturn(struct pic32mx_rxdesc_s *rxdesc)
{
  rxdesc->rsv1   = 0;
  rxdesc->rsv2   = 0;
  rxdesc->status = RXDESC_STATUS_EOWN | TXDESC_STATUS_NPV;
  pic32mx_dumprxdesc(rxdesc, "Returned to hardware");
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
  uint32_t status;

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  DEBUGASSERT(priv->pd_dev.d_buf != NULL &&
              priv->pd_dev.d_len < CONFIG_NET_BUFSIZE);

  /* Increment statistics and dump the packet (if so configured) */

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

  /* Find the next available TX descriptor.  We are guaranteed that is will
   * not fail by upstream logic that assures that a TX packet is available
   * before polling uIP.
   */

  txdesc = pic32mx_txdesc(priv);
  DEBUGASSERT(txdesc != NULL);
  pic32mx_dumptxdesc(txdesc, "Before transmit setup");

  /* Remove the transmit buffer from the device structure and assign it to
   * the TX descriptor.
   */

  txdesc->address    = PHYS_ADDR(priv->pd_dev.d_buf);
  priv->pd_dev.d_buf = NULL;

  /* Set the BYTE_COUNT for in the TX descriptor with the number of bytes
   * contained in the buffer.
   */

  status             = ((uint32_t)priv->pd_dev.d_len << TXDESC_STATUS_BYTECOUNT_SHIFT);
  priv->pd_dev.d_len = 0;

  /* Set EOWN = 1 to indicate that the packet belongs to Ethernet and set both
   * SOP and EOP to indicate that the packet both begins and ends with this
   * frame.
   */

  status        |= (TXDESC_STATUS_EOWN | TXDESC_STATUS_NPV |
                    TXDESC_STATUS_EOP | TXDESC_STATUS_SOP);
  txdesc->status = status;
  pic32mx_dumptxdesc(txdesc, "After transmit setup");

  /* Update the index to the next descriptor to use in the Tx ring */

  pic32mx_txnext(priv);

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

      /* Check if the next TX descriptor is available. If not, return a
       * non-zero value to terminate the poll.
       */

      if (pic32mx_txdesc(priv) == NULL)
        {
          /* There are no more TX descriptors/buffers available.. stop the poll */

          return -EAGAIN;
        }

      /* Get the next Tx buffer needed in order to continue the poll */

      priv->pd_dev.d_buf = pic32mx_allocbuffer(priv);
      if (priv->pd_dev.d_buf == NULL)
        {
          /* We have no more buffers available for the nex Tx.. stop the poll */

          return -ENOMEM;
        }
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return ret;
}

/****************************************************************************
 * Function: pic32mx_poll
 *
 * Description:
 *   Perform the uIP poll.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pic32mx_poll(struct pic32mx_driver_s *priv)
{
  /* Is there already a poll in progress.  This happens, for example, when
   * debugging output is enabled.  Interrupts may be re-enabled while debug
   * output is performed and a timer expiration could attempt a concurrent
   * poll.
   */

  if (!priv->pd_polling)
    {
      /* Assign a buffer for the poll */

      DEBUGASSERT(priv->pd_dev.d_buf == NULL);
      priv->pd_dev.d_buf = pic32mx_allocbuffer(priv);
      if (priv->pd_dev.d_buf != NULL)
        {
          /* And perform the poll */

          priv->pd_polling = true;
          (void)uip_poll(&priv->pd_dev, pic32mx_uiptxpoll);

          /* Free any buffer left attached after the poll */

          if (priv->pd_dev.d_buf != NULL)
            {
              pic32mx_freebuffer(priv, priv->pd_dev.d_buf);
              priv->pd_dev.d_buf = NULL;
            }
          priv->pd_polling = false;
        }
    }

}

/****************************************************************************
 * Function: pic32mx_timerpoll
 *
 * Description:
 *   Perform the uIP timer poll.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pic32mx_timerpoll(struct pic32mx_driver_s *priv)
{
  /* Is there already a poll in progress.  This happens, for example, when
   * debugging output is enabled.  Interrupts may be re-enabled while debug
   * output is performed and a timer expiration could attempt a concurrent
   * poll.
   */

  if (!priv->pd_polling)
    {
      DEBUGASSERT(priv->pd_dev.d_buf == NULL);
      priv->pd_dev.d_buf = pic32mx_allocbuffer(priv);
      if (priv->pd_dev.d_buf != NULL)
        {
          /* And perform the poll */

          priv->pd_polling = true;
          (void)uip_timer(&priv->pd_dev, pic32mx_uiptxpoll, PIC32MX_POLLHSEC);

          /* Free any buffer left attached after the poll */

          if (priv->pd_dev.d_buf != NULL)
            {
              pic32mx_freebuffer(priv, priv->pd_dev.d_buf);
              priv->pd_dev.d_buf = NULL;
            }
          priv->pd_polling = false;
        }
    }
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

  /* Check if the next TX descriptor is available. */

  txdesc = pic32mx_txdesc(priv);
  if (txdesc != NULL)
    {
       /* Yes.. queue the packet now. */

       pic32mx_transmit(priv);
    }
  else
    {
       /* No.. mark the Tx as pending and halt further Rx interrupts */

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
      pic32mx_dumprxdesc(rxdesc, "RX Complete");

      /* Update statistics */

      EMAC_STAT(priv, rx_packets);

      /* Get the packet length */

      priv->pd_dev.d_len = (rxdesc->rsv2 & RXDESC_RSV2_BYTECOUNT_MASK) >> RXDESC_RSV2_BYTECOUNT_SHIFT;

      /* Check for errors */

      if ((rxdesc->rsv2 & RXDESC_RSV2_OK) == 0)
        {
          nlldbg("ERROR. rsv1: %08x rsv2: %08x\n", rxdesc->rsv1, rxdesc->rsv2);
          EMAC_STAT(priv, rx_pkterr);
          pic32mx_rxreturn(rxdesc);
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
          pic32mx_rxreturn(rxdesc);
        }

      /* We don't have any logic here for reassembling packets from fragments. */

      else if ((rxdesc->status & (RXDESC_STATUS_EOP|RXDESC_STATUS_SOP)) != (RXDESC_STATUS_EOP|RXDESC_STATUS_SOP))
        {
          nlldbg("Fragment. packet length: %d rxdesc: %08x\n", priv->pd_dev.d_len, rxdesc->status);
          EMAC_STAT(priv, rx_fragment);
          pic32mx_rxreturn(rxdesc);
        }
      else
        {
          uint8_t *rxbuffer;

          /* Get the Rx buffer address from the Rx descriptor */
 
          priv->pd_dev.d_buf = (uint8_t*)VIRT_ADDR(rxdesc->address);
          DEBUGASSERT(priv->pd_dev.d_buf != NULL);

          /* Replace the buffer in the RX descriptor with a new one */

          rxbuffer = pic32mx_allocbuffer(priv);
          DEBUGASSERT(rxbuffer != NULL);
          rxdesc->address = PHYS_ADDR(rxbuffer);

          /* And give the RX descriptor back to the hardware */

          pic32mx_rxreturn(rxdesc);
          pic32mx_dumppacket("Received packet",
                             priv->pd_dev.d_buf, priv->pd_dev.d_len);

          /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
          if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
          if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
            {
              /* Handle the incoming IP packet */

              EMAC_STAT(priv, rx_ip);
              uip_arp_ipin(&priv->pd_dev);
              uip_input(&priv->pd_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the field d_len will
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
              /* Handle the incoming ARP packet */

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

              nlldbg("Unrecognized packet type dropped: %04x\n", ntohs(BUF->type));
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
              pic32mx_dumptxdesc(txdesc, "Freeing TX buffer");

              /* Free the TX buffer */

              pic32mx_freebuffer(priv, (uint8_t *)VIRT_ADDR(txdesc->address));
              txdesc->address = 0;

              /* Reset status */

              txdesc->tsv1    = 0;
              txdesc->tsv2    = 0;
              txdesc->status  = TXDESC_STATUS_SOWN | TXDESC_STATUS_NPV;
              pic32mx_dumptxdesc(txdesc, "TX buffer freed");
            }
        }
    }

  /* Check if there is a pending Tx transfer that was deferred by Rx handling
   * because there were no available Tx descriptors.  If so, process that
   * pending Tx now.
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
      /* Perform the uIP poll */

      pic32mx_poll(priv);
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

      pic32mx_poll(priv);
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

  /* Check if the next Tx descriptor is available.  We cannot perform the Tx
   * poll if we are unable to accept another packet for transmission.
   */

  if (pic32mx_txdesc(priv) != NULL)
    {
      /* If so, update TCP timing states and poll uIP for new XMIT data. Hmmm..
       * might be bug here.  Does this mean if there is a transmit in progress,
       * we will missing TCP time state updates?
       */

      pic32mx_timerpoll(priv);
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
  /* Configuration:
   * - Use the configuration fuse setting FETHIO bit (DEVCFG3:25) to detect
   *   the alternate/default I/O configuration
   * - Use the configuration fuse setting FMIIEN (DEVCFG3:24) to detect the
   *   MII/RMII operation mode.
   */

  /* Pin Configuration:
   *
   * No GPIO pin configuration is required.  Enabling the Ethernet Controller
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
   * digital (normally only those pins that have shared analog functionality
   * need to be configured).
   */

  /* Initialize the MIIM interface
   *
   * If the RMII operation is selected, reset the RMII module by using the
   * RESETRMII (EMAC1SUPP:11) bit and set the proper speed in the SPEEDRMII
   * bit (EMAC1SUPP:8) bit.
   */

#if CONFIG_PIC32MX_FMIIEN == 0
  pic32mx_putreg(EMAC1_SUPP_RESETRMII, PIC32MX_EMAC1_SUPPSET);
  pic32mx_putreg((EMAC1_SUPP_RESETRMII | EMAC1_SUPP_SPEEDRMII), PIC32MX_EMAC1_SUPPCLR);
#endif

   /* Issue an MIIM block reset, by setting the RESETMGMT (EMAC1MCFG:15) bit,
    * and then clear the reset bit.
    */

  regval = pic32mx_getreg(PIC32MX_EMAC1_MCFG);
  pic32mx_putreg(EMAC1_MCFG_MGMTRST, PIC32MX_EMAC1_MCFGSET);

  regval &= ~EMAC1_MCFG_MGMTRST;
  pic32mx_putreg(regval, PIC32MX_EMAC1_MCFG);

  /* Select a proper divider in the CLKSEL bit (EMAC1CFG:2-5) for the MIIM
   * PHY communication based on the system running clock frequency and the
   * external PHY supported clock.
   *
   * MII configuration: host clocked divider per board.h, no suppress
   * preamble, no scan increment.
   */

  regval &= ~(EMAC1_MCFG_CLKSEL_MASK | EMAC1_MCFG_NOPRE | EMAC1_MCFG_SCANINC);
  regval |= EMAC1_MCFG_CLKSEL_DIV;
  pic32mx_putreg(regval, PIC32MX_EMAC1_MCFG);

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
  pic32mx_putreg(0, PIC32MX_EMAC1_TEST);

  /* Having available the Duplex and Speed settings, configure the MAC
   * accordingly, using the following steps:
   *
   * Enable the RXENABLE bit (EMAC1CFG1:0), selecting both the TXPAUSE and
   * RXPAUSE bit (EMAC1CFG1:2-3) (the PIC32 MAC supports both).
   */

  pic32mx_putreg(EMAC1_CFG1_RXEN | EMAC1_CFG1_RXPAUSE | EMAC1_CFG1_TXPAUSE,
                 PIC32MX_EMAC1_CFG1SET);

  /* Select the desired auto-padding and CRC capabilities, and the enabling
   * of the huge frames and the Duplex type in the EMAC1CFG2 register.
   * (This was done in the PHY initialization logic).
   */

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
   *
   * Set up RX filter and configure to accept broadcast addresses and multicast
   * addresses (if so configured).   NOTE: There is a selection
   * CONFIG_NET_BROADCAST, but this enables receipt of UDP broadcast packets
   * inside of the stack.
   */

  regval  = ETH_RXFC_BCEN | ETH_RXFC_UCEN | ETH_RXFC_PMMODE_DISABLED;
#ifdef CONFIG_NET_MULTICAST
  regval |= ETH_RXFC_MCEN;
#endif
  pic32mx_putreg(regval, PIC32MX_ETH_RXFC);

  /* Set the size of the RX buffers in the RXBUFSZ bit (ETHCON2:4-10) (all
   * receive descriptors use the same buffer size). Keep in mind that using
   * packets that are too small leads to packet fragmentation and has a
   * noticeable impact on the performance.
   */

  pic32mx_putreg(ETH_CON2_RXBUFSZ(CONFIG_NET_BUFSIZE), PIC32MX_ETH_CON2);

  /* Reset state varialbes */

  priv->pd_polling   = false;
  priv->pd_txpending = false;

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

  regval  = pic32mx_getreg(PIC32MX_EMAC1_CFG1);
  regval |= EMAC1_CFG1_PASSALL;
  pic32mx_putreg(regval, PIC32MX_EMAC1_CFG1);

  /* Clear any pending interrupts (shouldn't be any) */

  pic32mx_putreg(0xffffffff, PIC32MX_ETH_IRQCLR);

  /* Configure interrupts.  The Ethernet interrupt was attached during one-time
   * initialization, so we only need to set the interrupt priority, configure
   * interrupts, and enable them.
   */

  /* If the user provided an interrupt priority, then set the interrupt to that
   * priority
   */

#if defined(CONFIG_NET_PRIORITY) && defined(CONFIG_ARCH_IRQPRIO)
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

  /* Set and activate a timer process */

  (void)wd_start(priv->pd_txpoll, PIC32MX_WDDELAY, pic32mx_polltimer, 1,
                (uint32_t)priv);

  /* Finally, enable the Ethernet interrupt at the interrupt controller */

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
      /* Check if the next Tx descriptor is available. */

      if (pic32mx_txdesc(priv) != NULL)
        {
          /* If so, then poll uIP for new XMIT data.  First allocate a buffer
           * to perform the poll
           */

          pic32mx_poll(priv);
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
 * Function: pic32mx_phybusywait
 *
 * Description:
 *   Wait until the PHY is no longer busy
 *
 * Parameters:
 *  None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void pic32mx_phybusywait(void)
{
  while ((pic32mx_getreg(PIC32MX_EMAC1_MIND) & EMAC1_MIND_MIIMBUSY) != 0);
}

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

  /* Make sure that the PHY is not still busy from the last command */

  pic32mx_phybusywait();

  /* Set PHY address and PHY register address */

  regval = ((uint32_t)phyaddr << EMAC1_MADR_PHYADDR_SHIFT) |
           ((uint32_t)regaddr << EMAC1_MADR_REGADDR_SHIFT);
  pic32mx_putreg(regval, PIC32MX_EMAC1_MADR);

  /* Write the register data to the PHY */

  pic32mx_putreg((uint32_t)phydata, PIC32MX_EMAC1_MWTD);

  /* Two clock cycles until busy is set from the write operation */

  __asm__ __volatile__ ("nop; nop;");
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

  /* Make sure that the PHY is not still busy from the last command */

  pic32mx_phybusywait();

  /* Set PHY address and PHY register address */

  regval = ((uint32_t)phyaddr << EMAC1_MADR_PHYADDR_SHIFT) |
           ((uint32_t)regaddr << EMAC1_MADR_REGADDR_SHIFT);
  pic32mx_putreg(regval, PIC32MX_EMAC1_MADR);

  /* Set up to read */

  pic32mx_putreg(EMAC1_MCMD_READ, PIC32MX_EMAC1_MCMD);

  /* Four clock cycles until busy is set from the write operation */

  __asm__ __volatile__ ("nop; nop; nop; nop;");

  /* Wait for the PHY command to complete */

  pic32mx_phybusywait();
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
  int32_t  timeout;
  uint16_t phyreg;

  /* Reset the PHY.  Needs a minimal 50uS delay after reset. */

  pic32mx_phywrite(phyaddr, MII_MCR, MII_MCR_RESET);

  /* Wait for a minimum of 50uS no matter what */

  up_udelay(50);

  /* The MCR reset bit is self-clearing.  Wait for it to be clear indicating
   * that the reset is complete.
   */

  for (timeout = PIC32MX_MIITIMEOUT; timeout > 0; timeout--)
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

  for (timeout = PIC32MX_MIITIMEOUT; timeout > 0; timeout--)
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

  for (timeout = PIC32MX_MIITIMEOUT; timeout > 0; timeout--)
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

#if CONFIG_PIC32MX_FMIIEN == 0
  /* Set the RMII operation mode. This usually requires access to a vendor
   * specific control register.
   */

#ifdef CONFIG_PHY_DP83848C
 /* The RMII/MII of operation can be selected by strap options or register
  * control (using the RBR register). For RMII mode, it is required to use the
  * strap option, since it requires a 50 MHz clock instead of the normal 25 MHz.
  */
#endif

#else
  /* Set the MII/ operation mode. This usually requires access to a vendor-
   * specific control register.
   */

#ifdef CONFIG_PHY_DP83848C
#  warning "Missing logic"
#endif

#endif

  /* Find PHY Address.  Because the controller has a pull-up and the
   * PHY has pull-down resistors on RXD lines some times the PHY
   * latches different at different addresses.
   */

  for (phyaddr = 0; phyaddr < 32; phyaddr++)
    {
      /* Clear any ongoing PHY command bits */

      pic32mx_putreg(0, PIC32MX_EMAC1_MCMD);

      /* Reset the PHY (use Control Register 0). */

      ret = pic32mx_phyreset(phyaddr);
      if (ret < 0)
        {
          ndbg("Failed to reset PHY at address %d\n", phyaddr);
          continue;
        }

      /* Set the normal, swapped or auto (preferred) MDIX. This usually
       * requires access to a vendor-specific control register.
       */

      /* Check if we can see the selected device ID at this
       * PHY address.
       */

       phyreg = (unsigned int)pic32mx_phyread(phyaddr, MII_PHYID1);
       nvdbg("Addr: %d PHY ID1: %04x\n", phyaddr, phyreg);

       if (phyreg == PIC32MX_PHYID1)
        {
          phyreg = pic32mx_phyread(phyaddr, MII_PHYID2);
          nvdbg("Addr: %d PHY ID2: %04x\n", phyaddr, phyreg);

          if (phyreg == PIC32MX_PHYID2)
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
        break;
      case KS8721_10BTCR_MODE_100BTHD: /* 100BASE-T half duplex */
        priv->pd_mode = PIC32MX_100BASET_HD;
        break;
      case KS8721_10BTCR_MODE_10BTFD: /* 10BASE-T full duplex */
        priv->pd_mode = PIC32MX_10BASET_FD;
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
  /* Set up for full or half duplex operation */

  if ((mode & PIC32MX_DUPLEX_MASK) == PIC32MX_DUPLEX_FULL)
    {
      /* Set the back-to-back inter-packet gap */
 
      pic32mx_putreg(21, PIC32MX_EMAC1_IPGT);

      /* Set MAC to operate in full duplex mode with CRC and Pad enabled */

      pic32mx_putreg((EMAC1_CFG2_FULLDPLX | EMAC1_CFG2_CRCEN | EMAC1_CFG2_PADCRCEN),
                     PIC32MX_EMAC1_CFG2SET);
    }
  else
    {
      /* Set the back-to-back inter-packet gap */
 
      pic32mx_putreg(18, PIC32MX_EMAC1_IPGT);

      /* Set MAC to operate in half duplex mode with CRC and Pad enabled */

      pic32mx_putreg(EMAC1_CFG2_FULLDPLX, PIC32MX_EMAC1_CFG2CLR);
      pic32mx_putreg((EMAC1_CFG2_CRCEN | EMAC1_CFG2_PADCRCEN), PIC32MX_EMAC1_CFG2SET);
    }

  /* Set the RMII MAC speed. */

#if CONFIG_PIC32MX_FMIIEN == 0
  if ((mode & PIC32MX_SPEED_MASK) == PIC32MX_SPEED_100)
    {
      pic32mx_putreg(EMAC1_SUPP_SPEEDRMII, PIC32MX_EMAC1_SUPPSET);
    }
  else
    {
      pic32mx_putreg(EMAC1_SUPP_SPEEDRMII, PIC32MX_EMAC1_SUPPCLR);
    }
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

  /* Turn the Ethernet controller on. */

  pic32mx_putreg(ETH_CON1_ON, PIC32MX_ETH_CON1SET);

  /* Clear the Ethernet STAT BUFCNT */

  while ((pic32mx_getreg(PIC32MX_ETH_STAT) & ETH_STAT_BUFCNT_MASK) != 0)
    {
      pic32mx_putreg(ETH_CON1_BUFCDEC, PIC32MX_ETH_CON1SET);
    }

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
  pic32mx_putreg(0xffffffff, PIC32MX_ETH_RXSTCLR);

  /* MAC Initialization *****************************************************/
  /* Put the MAC into the reset state */

  pic32mx_putreg((EMAC1_CFG1_TXRST    | EMAC1_CFG1_MCSTXRST | EMAC1_CFG1_RXRST |
                 EMAC1_CFG1_MCSRXRST | EMAC1_CFG1_SIMRST   | EMAC1_CFG1_SOFTRST),
                 PIC32MX_EMAC1_CFG1);

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
