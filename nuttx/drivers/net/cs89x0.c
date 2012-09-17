/****************************************************************************
 * drivers/net/cs89x0.c
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_CS89x0)

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

#error "Under construction -- do not use"

/* CONFIG_CS89x0_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_CS89x0_NINTERFACES
# define CONFIG_CS89x0_NINTERFACES 1
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define CS89x0_WDDELAY   (1*CLK_TCK)
#define CS89x0_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define CS89x0_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)cs89x0->cs_dev.d_buf)

/* If there is only one CS89x0 instance, then mapping the CS89x0 IRQ to
 * a driver state instance is trivial.
 */

#if CONFIG_CS89x0_NINTERFACES == 1
#  define cs89x0_mapirq(irq) g_cs89x0[0]
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct cs89x0_driver_s *g_cs89x0[CONFIG_CS89x0_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CS89x0 register access */

static uint16_t cs89x0_getreg(struct cs89x0_driver_s *cs89x0, int offset);
static void cs89x0_putreg(struct cs89x0_driver_s *cs89x0, int offset,
                          uint16_t value);
static uint16_t cs89x0_getppreg(struct cs89x0_driver_s *cs89x0, int addr);
static void cs89x0_putppreg(struct cs89x0_driver_s *cs89x0, int addr,
                            uint16_t value);

/* Common TX logic */

static int  cs89x0_transmit(struct cs89x0_driver_s *cs89x0);
static int  cs89x0_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static void cs89x0_receive(struct cs89x0_driver_s *cs89x0);
static void cs89x0_txdone(struct cs89x0_driver_s *cs89x0, uint16_t isq);
#if CONFIG_CS89x0_NINTERFACES > 1
static inline FAR struct cs89x0_driver_s *cs89x0_mapirq(int irq);
#endif
static int  cs89x0_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void cs89x0_polltimer(int argc, uint32_t arg, ...);
static void cs89x0_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int cs89x0_ifup(struct uip_driver_s *dev);
static int cs89x0_ifdown(struct uip_driver_s *dev);
static int cs89x0_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int cs89x0_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
static int cs89x0_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: cs89x0_getreg and cs89x0_putreg
 *
 * Description:
 *   Read from and write to a CS89x0 register
 *
 * Parameters:
 *   cs89x0 - Reference to the driver state structure
 *   offset - Offset to the CS89x0 register
 *   value  - Value to be written (cs89x0_putreg only)
 *
 * Returned Value:
 *   cs89x0_getreg: The 16-bit value of the register
 *   cs89x0_putreg: None
 *
 ****************************************************************************/

static uint16_t cs89x0_getreg(struct cs89x0_driver_s *cs89x0, int offset)
{
#ifdef CONFIG_CS89x0_ALIGN16
  return getreg16(s89x0->cs_base + offset);
#else
  return (uint16_t)getreg32(s89x0->cs_base + offset);
#endif
}

static void cs89x0_putreg(struct cs89x0_driver_s *cs89x0, int offset, uint16_t value)
{
#ifdef CONFIG_CS89x0_ALIGN16
  return putreg16(value, s89x0->cs_base + offset);
#else
  return (uint16_t)putreg32((uint32_t)value, s89x0->cs_base + offset);
#endif
}

/****************************************************************************
 * Function: cs89x0_getppreg and cs89x0_putppreg
 *
 * Description:
 *   Read from and write to a CS89x0 page packet register
 *
 * Parameters:
 *   cs89x0 - Reference to the driver state structure
 *   addr   - Address of the CS89x0 page packet register
 *   value  - Value to be written (cs89x0_putppreg only)
 *
 * Returned Value:
 *   cs89x0_getppreg: The 16-bit value of the page packet register
 *   cs89x0_putppreg: None
 *
 ****************************************************************************/

static uint16_t cs89x0_getppreg(struct cs89x0_driver_s *cs89x0, int addr)
{
  /* In memory mode, the CS89x0's internal registers and frame buffers are mapped
   * into a contiguous 4kb block providing direct access to the internal registers
   * and frame buffers.
   */

#ifdef CONFIG_CS89x0_MEMMODE
  if (cs89x0->cs_memmode)
    {
#ifdef CONFIG_CS89x0_ALIGN16
      return getreg16(s89x0->cs_ppbase + (CS89x0_PDATA_OFFSET << ??));
#else
      return (uint16_t)getreg32(s89x0->cs_ppbase + (CS89x0_PDATA_OFFSET << ??));
#endif
    }

    /* When configured in I/O mode, the CS89x0 is accessed through eight, 16-bit
     * I/O ports that in the host system's I/O space.
     */

  else
#endif
    {
#ifdef CONFIG_CS89x0_ALIGN16
      putreg16((uint16_t)addr, cs89x0->cs_base + CS89x0_PPTR_OFFSET);
      return getreg16(s89x0->cs_base + CS89x0_PDATA_OFFSET);
#else
      putreg32((uint32_t)addr, cs89x0->cs_base + CS89x0_PPTR_OFFSET);
      return (uint16_t)getreg32(s89x0->cs_base + CS89x0_PDATA_OFFSET);
#endif
    }
}

static void cs89x0_putppreg(struct cs89x0_driver_s *cs89x0, int addr, uint16_t value)
{
  /* In memory mode, the CS89x0's internal registers and frame buffers are mapped
   * into a contiguous 4kb block providing direct access to the internal registers
   * and frame buffers.
   */

#ifdef CONFIG_CS89x0_MEMMODE
  if (cs89x0->cs_memmode)
    {
#ifdef CONFIG_CS89x0_ALIGN16
      putreg16(value), cs89x0->cs_ppbase + (CS89x0_PDATA_OFFSET << ??));
#else
      putreg32((uint32_t)value, cs89x0->cs_ppbase + (CS89x0_PDATA_OFFSET << ??));
#endif
    }

    /* When configured in I/O mode, the CS89x0 is accessed through eight, 16-bit
     * I/O ports that in the host system's I/O space.
     */

  else
#endif
    {
#ifdef CONFIG_CS89x0_ALIGN16
      putreg16((uint16_t)addr, cs89x0->cs_base + CS89x0_PPTR_OFFSET);
      putreg16(value, cs89x0->cs_base + CS89x0_PDATA_OFFSET);
#else
      putreg32((uint32_t)addr, cs89x0->cs_base + CS89x0_PPTR_OFFSET);
      putreg32((uint32_t)value, cs89x0->cs_base + CS89x0_PDATA_OFFSET);
#endif
    }
}

/****************************************************************************
 * Function: cs89x0_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   cs89x0  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int cs89x0_transmit(struct cs89x0_driver_s *cs89x0)
{
  /* Verify that the hardware is ready to send another packet */
#warning "Missing logic"

  /* Increment statistics */
#warning "Missing logic"

  /* Disable Ethernet interrupts */
#warning "Missing logic"

  /* Send the packet: address=cs89x0->cs_dev.d_buf, length=cs89x0->cs_dev.d_len */
#warning "Missing logic"

  /* Restore Ethernet interrupts */
#warning "Missing logic"

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(cs89x0->cs_txtimeout, CS89x0_TXTIMEOUT, cs89x0_txtimeout, 1, (uint32_t)cs89x0);
  return OK;
}

/****************************************************************************
 * Function: cs89x0_uiptxpoll
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
 *
 ****************************************************************************/

static int cs89x0_uiptxpoll(struct uip_driver_s *dev)
{
  struct cs89x0_driver_s *cs89x0 = (struct cs89x0_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (cs89x0->cs_dev.d_len > 0)
    {
      uip_arp_out(&cs89x0->cs_dev);
      cs89x0_transmit(cs89x0);

      /* Check if there is room in the CS89x0 to hold another packet. If not,
       * return a non-zero value to terminate the poll.
       */
#warning "Missing logic"
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: cs89x0_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   cs89x0  - Reference to the driver state structure
 *   isq     - Interrupt status queue value read by interrupt handler
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void cs89x0_receive(struct cs89x0_driver_s *cs89x0, uint16_t isq)
{
  uint16_t *dest;
  uint16_t rxlength;
  int nbytes;

  /* Check for errors and update statistics */

  rxlength = cs89x0_getreg(PPR_RXLENGTH);
  if ((isq & RX_OK) == 0)
    {
#ifdef CONFIG_C89x0_STATISTICS
      cd89x0->cs_stats.rx_errors++;
      if ((isq & RX_RUNT) != 0)
        {
          cd89x0->cs_stats.rx_lengtherrors++;
        }
      if ((isq & RX_EXTRA_DATA) != 0)
        {
          cd89x0->cs_stats.rx_lengtherrors++;
        }
      if (isq & RX_CRC_ERROR) != 0)
        {
          if (!(isq & (RX_EXTRA_DATA|RX_RUNT)))
            {
              cd89x0->cs_stats.rx_crcerrors++;
            }
        }
      if ((isq & RX_DRIBBLE) != 0)
        {
          cd89x0->cs_stats.rx_frameerrors++;
        }
#endif
      return;
    }

  /* Check if the packet is a valid size for the uIP buffer configuration */

  if (rxlength > ???)
    {
#ifdef CONFIG_C89x0_STATISTICS
      cd89x0->cs_stats.rx_errors++;
      cd89x0->cs_stats.rx_lengtherrors++;
#endif
      return;    
    }
    
  /* Copy the data data from the hardware to cs89x0->cs_dev.d_buf.  Set
   * amount of data in cs89x0->cs_dev.d_len
   */

  dest = (uint16_t*)cs89x0->cs_dev.d_buf;
  for (nbytes = 0; nbytes < rxlength; nbytes += sizeof(uint16_t))
    {
      *dest++ = cs89x0_getreg(PPR_RXFRAMELOCATION);
    }  

#ifdef CONFIG_C89x0_STATISTICS
  cd89x0->cs_stats.rx_packets++;
#endif
  /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
  if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
  if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
    {
      uip_arp_ipin(&cs89x0->cs_dev);
      uip_input(&cs89x0->cs_dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, the field  d_len will set to a value > 0.
       */

      if (cs89x0->cs_dev.d_len > 0)
        {
          uip_arp_out(&cs89x0->cs_dev);
          cs89x0_transmit(cs89x0);
        }
    }
  else if (BUF->type == htons(UIP_ETHTYPE_ARP))
    {
       uip_arp_arpin(&cs89x0->cs_dev);

       /* If the above function invocation resulted in data that should be
        * sent out on the network, the field  d_len will set to a value > 0.
        */

       if (cs89x0->cs_dev.d_len > 0)
         {
           cs89x0_transmit(cs89x0);
         }
    }
}

/****************************************************************************
 * Function: cs89x0_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   cs89x0  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void cs89x0_txdone(struct cs89x0_driver_s *cs89x0, uint16_t isq)
{
  /* Check for errors and update statistics.  The lower 6-bits of the ISQ
   * hold the register address causing the interrupt.  We got here because
   * those bits indicated */

#ifdef CONFIG_C89x0_STATISTICS
  cd89x0->cs_stats.tx_packets++;
  if ((isq & ISQ_TXEVENT_TXOK) == 0)
    {
      cd89x0->cs_stats.tx_errors++;
    }
  if ((isq & ISQ_TXEVENT_LOSSOFCRS) != 0)
    {
      cd89x0->cs_stats.tx_carriererrors++;
    }
  if ((isq & ISQ_TXEVENT_SQEERROR) != 0)
    {
      cd89x0->cs_stats.tx_heartbeaterrors++;
    }
  if (i(sq & ISQ_TXEVENT_OUTWINDOW) != 0)
    {
      cd89x0->cs_stats.tx_windowerrors++;
    }
  if (isq & TX_16_COL)
    {
      cd89x0->cs_stats.tx_abortederrors++;
    }
#endif

  /* If no further xmits are pending, then cancel the TX timeout */

  wd_cancel(cs89x0->cs_txtimeout);

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&cs89x0->cs_dev, cs89x0_uiptxpoll);
}

/****************************************************************************
 * Function: cs89x0_mapirq
 *
 * Description:
 *   Map an IRQ number to a CS89x0 device state instance.  This is only
 *   necessary to handler the case where the architecture includes more than
 *   on CS89x0 chip.
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *
 * Returned Value:
 *   A reference to device state structure (NULL if irq does not correspond
 *   to any CS89x0 device).
 *
 * Assumptions:
 *
 ****************************************************************************/

#if CONFIG_CS89x0_NINTERFACES > 1
static inline FAR struct cs89x0_driver_s *cs89x0_mapirq(int irq)
{
  int i;
  for (i = 0; i < CONFIG_CS89x0_NINTERFACES; i++)
    {
      if (g_cs89x0[i] && g_cs89x0[i].irq == irq)
        {
          return g_cs89x0[i];
        }
    }
  return NULL;
}
#endif

/****************************************************************************
 * Function: cs89x0_interrupt
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

static int cs89x0_interrupt(int irq, FAR void *context)
{
  register struct cs89x0_driver_s *cs89x0 = s89x0_mapirq(irq);
  uint16_t isq;
  
#ifdef CONFIG_DEBUG
  if (!cs89x0)
    {
      return -ENODEV;
    }
#endif

  /* Read and process all of the events from the ISQ */

  while ((isq = cs89x0_getreg(dev, CS89x0_ISQ_OFFSET)) != 0)
    {
      nvdbg("ISQ: %04x\n", isq);
      switch (isq & ISQ_EVENTMASK)
        {
        case ISQ_RXEVENT:
            cs89x0_receive(cs89x0);
            break;

        case ISQ_TXEVENT:
            cs89x0_txdone(cs89x0, isq);
            break;

        case ISQ_BUFEVENT:
            if ((isq & ISQ_BUFEVENT_TXUNDERRUN) != 0)
              {
                ndbg("Transmit underrun\n");
#ifdef CONFIG_CS89x0_XMITEARLY
                cd89x0->cs_txunderrun++;
                if (cd89x0->cs_txunderrun == 3)
                  {
                    cd89x0->cs_txstart = PPR_TXCMD_TXSTART381;
                  }
                else if (cd89x0->cs_txunderrun == 6)
                  {
                    cd89x0->cs_txstart = PPR_TXCMD_TXSTARTFULL;
                  }
#endif
              }
            break;

        case ISQ_RXMISSEVENT:
#ifdef CONFIG_C89x0_STATISTICS
            cd89x0->cs_stats.rx_missederrors += (isq >>6);
#endif
            break;

        case ISQ_TXCOLEVENT:
#ifdef CONFIG_C89x0_STATISTICS
            cd89x0->cs_stats.collisions += (isq >>6);
#endif
            break;
        }
    }
  return OK;
}

/****************************************************************************
 * Function: cs89x0_txtimeout
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
 *
 ****************************************************************************/

static void cs89x0_txtimeout(int argc, uint32_t arg, ...)
{
  struct cs89x0_driver_s *cs89x0 = (struct cs89x0_driver_s *)arg;

  /* Increment statistics and dump debug info */
#warning "Missing logic"

  /* Then reset the hardware */
#warning "Missing logic"

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&cs89x0->cs_dev, cs89x0_uiptxpoll);
}

/****************************************************************************
 * Function: cs89x0_polltimer
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
 *
 ****************************************************************************/

static void cs89x0_polltimer(int argc, uint32_t arg, ...)
{
  struct cs89x0_driver_s *cs89x0 = (struct cs89x0_driver_s *)arg;

  /* Check if there is room in the send another TXr packet.  */
#warning "Missing logic"

  /* If so, update TCP timing states and poll uIP for new XMIT data */

  (void)uip_timer(&cs89x0->cs_dev, cs89x0_uiptxpoll, CS89x0_POLLHSEC);

  /* Setup the watchdog poll timer again */

  (void)wd_start(cs89x0->cs_txpoll, CS89x0_WDDELAY, cs89x0_polltimer, 1, arg);
}

/****************************************************************************
 * Function: cs89x0_ifup
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

static int cs89x0_ifup(struct uip_driver_s *dev)
{
  struct cs89x0_driver_s *cs89x0 = (struct cs89x0_driver_s *)dev->d_private;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Initialize the Ethernet interface */
#warning "Missing logic"

  /* Set and activate a timer process */

  (void)wd_start(cs89x0->cs_txpoll, CS89x0_WDDELAY, cs89x0_polltimer, 1, (uint32_t)cs89x0);

  /* Enable the Ethernet interrupt */

  cs89x0->cs_bifup = true;
  up_enable_irq(CONFIG_CS89x0_IRQ);
  return OK;
}

/****************************************************************************
 * Function: cs89x0_ifdown
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

static int cs89x0_ifdown(struct uip_driver_s *dev)
{
  struct cs89x0_driver_s *cs89x0 = (struct cs89x0_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = irqsave();
  up_disable_irq(CONFIG_CS89x0_IRQ);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(cs89x0->cs_txpoll);
  wd_cancel(cs89x0->cs_txtimeout);

  /* Reset the device */

  cs89x0->cs_bifup = false;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: cs89x0_txavail
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

static int cs89x0_txavail(struct uip_driver_s *dev)
{
  struct cs89x0_driver_s *cs89x0 = (struct cs89x0_driver_s *)dev->d_private;
  irqstate_t flags;

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (cs89x0->cs_bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */
#warning "Missing logic"

      /* If so, then poll uIP for new XMIT data */

      (void)uip_poll(&cs89x0->cs_dev, cs89x0_uiptxpoll);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: cs89x0_addmac
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
static int cs89x0_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct cs89x0_driver_s *priv = (FAR struct cs89x0_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

#warning "Multicast MAC support not implemented"
  return OK;
}
#endif

/****************************************************************************
 * Function: cs89x0_rmmac
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
static int cs89x0_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct cs89x0_driver_s *priv = (FAR struct cs89x0_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

#warning "Multicast MAC support not implemented"
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: cs89x0_initialize
 *
 * Description:
 *   Initialize the Ethernet driver
 *
 * Parameters:
 *   impl - decribes the implementation of the cs89x00 implementation.
 *     This reference is retained so so must remain stable throughout the
 *     life of the driver instance.
 *   devno - Identifies the device number.  This must be a number between
 *     zero CONFIG_CS89x0_NINTERFACES and the same devno must not be
 *     initialized twice.  The associated network device will be referred
 *     to with the name "eth" followed by this number (eth0, eth1, etc).
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

/* Initialize the CS89x0 chip and driver */

int cs89x0_initialize(FAR const cs89x0_driver_s *cs89x0, int devno)
{
  /* Sanity checks -- only performed with debug enabled */

#ifdef CONFIG_DEBUG
  if (!cs89x0 || (unsigned)devno > CONFIG_CS89x0_NINTERFACES || g_cs89x00[devno])
    {
      return -EINVAL;
    }
#endif

  /* Check if a Ethernet chip is recognized at its I/O base */

#warning "Missing logic"

  /* Attach the IRQ to the driver */

  if (irq_attach(cs89x0->irq, cs89x0_interrupt))
    {
      /* We could not attach the ISR to the ISR */

      return -EAGAIN;
    }

  /* Initialize the driver structure */

  g_cs89x[devno]           = cs89x0;          /* Used to map IRQ back to instance */
  cs89x0->cs_dev.d_ifup    = cs89x0_ifup;     /* I/F down callback */
  cs89x0->cs_dev.d_ifdown  = cs89x0_ifdown;   /* I/F up (new IP address) callback */
  cs89x0->cs_dev.d_txavail = cs89x0_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  cs89x0->cs_dev.d_addmac  = cs89x0_addmac;   /* Add multicast MAC address */
  cs89x0->cs_dev.d_rmmac   = cs89x0_rmmac;    /* Remove multicast MAC address */
#endif
  cs89x0->cs_dev.d_private = (void*)cs89x0;   /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  cs89x0->cs_txpoll       = wd_create();   /* Create periodic poll timer */
  cs89x0->cs_txtimeout    = wd_create();   /* Create TX timeout timer */

  /* Read the MAC address from the hardware into cs89x0->cs_dev.d_mac.ether_addr_octet */

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&cs89x0->cs_dev);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_CS89x0 */

