/****************************************************************************
 * drivers/net/e1000.c
 *
 *   Copyright (C) 2011 Yu Qiang. All rights reserved.
 *   Author: Yu Qiang <yuq825@gmail.com>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include <nuttx/kmalloc.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <debug.h>
#include <wdog.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arp.h>
#include <nuttx/net/uip/uip-arch.h>

#include <rgmp/pmap.h>
#include <rgmp/string.h>
#include <rgmp/stdio.h>
#include <rgmp/utils.h>
#include <rgmp/arch/pci.h>
#include <rgmp/memio.h>
#include "e1000.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define E1000_WDDELAY   (1*CLK_TCK)
#define E1000_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define E1000_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)e1000->uip_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tx_ring {
    struct tx_desc *desc;
    char *buf;
    int tail;      // where to write desc
};

struct rx_ring {
    struct rx_desc *desc;
    char *buf;
    int head;      // where to read
    int tail;      // where to release free desc
    int free;      // number of freed desc
};

struct e1000_dev {
    uint32_t phy_mem_base;
    uint32_t io_mem_base;
    uint32_t mem_size;
    int pci_dev_id;
	uint16_t pci_addr;
    unsigned char src_mac[6];
    unsigned char dst_mac[6];
    struct irq_action int_desc;
    struct tx_ring tx_ring;
    struct rx_ring rx_ring;
    struct e1000_dev *next;

    // NuttX net data
    bool bifup;               /* true:ifup false:ifdown */
    WDOG_ID txpoll;           /* TX poll timer */
    WDOG_ID txtimeout;        /* TX timeout timer */

    /* This holds the information visible to uIP/NuttX */

    struct uip_driver_s uip_dev;  /* Interface understood by uIP */
};

struct e1000_dev_head {
    struct e1000_dev *next;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct e1000_dev_head e1000_list = {0};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  e1000_transmit(struct e1000_dev *e1000);
static int  e1000_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static void e1000_receive(struct e1000_dev *e1000);

/* Watchdog timer expirations */

static void e1000_polltimer(int argc, uint32_t arg, ...);
static void e1000_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int e1000_ifup(struct uip_driver_s *dev);
static int e1000_ifdown(struct uip_driver_s *dev);
static int e1000_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int e1000_addmac(struct uip_driver_s *dev, const uint8_t *mac);
static int e1000_rmmac(struct uip_driver_s *dev, const uint8_t *mac);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
 
static inline void e1000_outl(struct e1000_dev *dev, int reg, uint32_t val)
{
    writel(dev->io_mem_base+reg, val);
}

static inline uint32_t e1000_inl(struct e1000_dev *dev, int reg)
{
    return readl(dev->io_mem_base+reg);
}

/****************************** e1000 driver ********************************/

void e1000_reset(struct e1000_dev *dev)
{
    uint32_t dev_control;

    // Reset the network controller hardware
    dev_control = 0;
    dev_control |= (1<<0);   // FD-bit (Full Duplex)
    dev_control |= (0<<2);   // GIOMD-bit (GIO Master Disable)
    dev_control |= (1<<3);   // LRST-bit (Link Reset)
    dev_control |= (1<<6);   // SLU-bit (Set Link Up)	
    dev_control |= (2<<8);   // SPEED=2 (1000Mbps)
    dev_control |= (0<<11);  // FRCSPD-bit (Force Speed)
    dev_control |= (0<<12);  // FRCDPLX-bit (Force Duplex)
    dev_control |= (0<<20);  // ADVD3WUC-bit (Advertise D3 Wake Up Cap)
    dev_control |= (1<<26);  // RST-bit (Device Reset)
    dev_control |= (1<<27);  // RFCE-bit (Receive Flow Control Enable)
    dev_control |= (1<<28);  // TFCE-bit (Transmit Flow Control Enable) 
    dev_control |= (0<<30);  // VME-bit (VLAN Mode Enable) 
    dev_control |= (0<<31);  // PHY_RST-bit (PHY Reset)

    e1000_outl(dev, E1000_IMC, 0xFFFFFFFF);
    e1000_outl(dev, E1000_STATUS, 0x00000000);
    e1000_outl(dev, E1000_CTRL, dev_control);
    dev_control &= ~(1<<26);  // clear RST-bit (Device Reset)
    e1000_outl(dev, E1000_CTRL, dev_control);	
    up_mdelay(10);
    e1000_outl(dev, E1000_CTRL_EXT, 0x001401C0);
    e1000_outl(dev, E1000_IMC, 0xFFFFFFFF);
}

void e1000_turn_on(struct e1000_dev *dev)
{
    int	tx_control, rx_control;
    uint32_t ims = 0;

    // turn on the controller's receive engine
    rx_control = e1000_inl(dev, E1000_RCTL);
    rx_control |= (1<<1);
    e1000_outl(dev, E1000_RCTL, rx_control);	

    // turn on the controller's transmit engine
    tx_control = e1000_inl(dev, E1000_TCTL);
    tx_control |= (1<<1);
    e1000_outl(dev, E1000_TCTL, tx_control);	

    // enable the controller's interrupts
    e1000_outl(dev, E1000_ICR, 0xFFFFFFFF);
    e1000_outl(dev, E1000_IMC, 0xFFFFFFFF);

    ims |= 1<<0;      // TXDW
    ims |= 1<<1;      // TXQE
    ims |= 1<<2;      // LSC
    ims |= 1<<4;      // RXDMT0 
    ims |= 1<<7;      // RXT0 
    e1000_outl(dev, E1000_IMS, ims);
}

void e1000_turn_off(struct e1000_dev *dev)
{
    int	tx_control, rx_control;

    // turn off the controller's receive engine
    rx_control = e1000_inl(dev, E1000_RCTL);
    rx_control &= ~(1<<1);
    e1000_outl(dev, E1000_RCTL, rx_control);	

    // turn off the controller's transmit engine
    tx_control = e1000_inl(dev, E1000_TCTL);
    tx_control &= ~(1<<1);
    e1000_outl(dev, E1000_TCTL, tx_control);	

    // turn off the controller's interrupts
    e1000_outl(dev, E1000_IMC, 0xFFFFFFFF);
}

void e1000_init(struct e1000_dev *dev)
{
    uint32_t rxd_phys, txd_phys, kmem_phys;
    uint32_t rx_control, tx_control;
    uint32_t pba;
    int i;

    e1000_reset(dev);

    // configure the controller's 'receive' engine
    rx_control = 0;
    rx_control |= (0<<1);	  // EN-bit (Enable)
    rx_control |= (0<<2);	  // SPB-bit (Store Bad Packets) 	
    rx_control |= (0<<3);	  // UPE-bit (Unicast Promiscuous Mode)
    rx_control |= (1<<4);	  // MPE-bit (Multicast Promiscuous Mode)
    rx_control |= (0<<5);	  // LPE-bit (Long Packet Enable)
    rx_control |= (0<<6);	  // LBM=0 (Loop-Back Mode)
    rx_control |= (0<<8);	  // RDMTS=0 (Rx Descriptor Min Threshold Size)
    rx_control |= (0<<10);  // DTYPE=0 (Descriptor Type)
    rx_control |= (0<<12);  // MO=0 (Multicast Offset)
    rx_control |= (1<<15);  // BAM-bit (Broadcast Address Mode)
    rx_control |= (0<<16);  // BSIZE=0 (Buffer Size = 2048) 	
    rx_control |= (0<<18);  // VLE-bit (VLAN filter Enable)
    rx_control |= (0<<19);  // CFIEN-bit (Canonical Form Indicator Enable)	
    rx_control |= (0<<20);  // CFI-bit (Canonical Form Indicator)
    rx_control |= (1<<22);  // DPF-bit (Discard Pause Frames)	
    rx_control |= (0<<23);  // PMCF-bit (Pass MAC Control Frames)
    rx_control |= (0<<25);  // BSEX=0 (Buffer Size EXtension)
    rx_control |= (1<<26);  // SECRC-bit (Strip Ethernet CRC)
    rx_control |= (0<<27);  // FLEXBUF=0 (Flexible Buffer size)	
    e1000_outl(dev, E1000_RCTL, rx_control);

    // configure the controller's 'transmit' engine
    tx_control = 0;
    tx_control |= (0<<1);	   // EN-bit (Enable)
    tx_control |= (1<<3);	   // PSP-bit (Pad Short Packets)
    tx_control |= (15<<4);   // CT=15 (Collision Threshold)
    tx_control |= (63<<12);  // COLD=63 (Collision Distance)
    tx_control |= (0<<22);   // SWXOFF-bit (Software XOFF)
    tx_control |= (1<<24);   // RTLC-bit (Re-Transmit on Late Collision)
    tx_control |= (0<<25);   // UNORTX-bit (Underrun No Re-Transmit)
    tx_control |= (0<<26);   // TXCSCMT=0 (TxDesc Mininum Threshold)
    tx_control |= (0<<28);   // MULR-bit (Multiple Request Support)
    e1000_outl(dev, E1000_TCTL, tx_control);

    // hardware flow control
    pba = e1000_inl(dev, E1000_PBA);
    // get receive FIFO size
    pba = (pba & 0x000000ff)<<10;
    e1000_outl(dev, E1000_FCAL, 0x00C28001);
    e1000_outl(dev, E1000_FCAH, 0x00000100);
    e1000_outl(dev, E1000_FCT, 0x00008808);
    e1000_outl(dev, E1000_FCTTV, 0x00000680);
    e1000_outl(dev, E1000_FCRTL, (pba*8/10)|0x80000000);
    e1000_outl(dev, E1000_FCRTH, pba*9/10);

    // setup tx rings
    txd_phys = PADDR((uintptr_t)dev->tx_ring.desc);
    kmem_phys = PADDR((uintptr_t)dev->tx_ring.buf);
    for (i=0; i<CONFIG_E1000_N_TX_DESC; i++,kmem_phys+=CONFIG_E1000_BUFF_SIZE) {
		dev->tx_ring.desc[i].base_address = kmem_phys;
		dev->tx_ring.desc[i].packet_length = 0;
		dev->tx_ring.desc[i].cksum_offset = 0;
		dev->tx_ring.desc[i].cksum_origin = 0;
		dev->tx_ring.desc[i].desc_status = 1;
		dev->tx_ring.desc[i].desc_command = (1<<0)|(1<<1)|(1<<3);
		dev->tx_ring.desc[i].special_info = 0;
    }
    dev->tx_ring.tail = 0;
    e1000_outl(dev, E1000_TDT, 0);
    e1000_outl(dev, E1000_TDH, 0);
    // tell controller the location, size, and fetch-policy for Tx queue
    e1000_outl(dev, E1000_TDBAL, txd_phys);
    e1000_outl(dev, E1000_TDBAH, 0x00000000);
    e1000_outl(dev, E1000_TDLEN, CONFIG_E1000_N_TX_DESC*16);
    e1000_outl(dev, E1000_TXDCTL, 0x01010000);

    // setup rx rings
    rxd_phys = PADDR((uintptr_t)dev->rx_ring.desc);
    kmem_phys = PADDR((uintptr_t)dev->rx_ring.buf);
    for (i=0; i<CONFIG_E1000_N_RX_DESC; i++,kmem_phys+=CONFIG_E1000_BUFF_SIZE) {
		dev->rx_ring.desc[i].base_address = kmem_phys;
		dev->rx_ring.desc[i].packet_length = 0;
		dev->rx_ring.desc[i].packet_cksum = 0;
		dev->rx_ring.desc[i].desc_status = 0;
		dev->rx_ring.desc[i].desc_errors = 0;
		dev->rx_ring.desc[i].vlan_tag = 0;
    }
    dev->rx_ring.head = 0;
    dev->rx_ring.tail = CONFIG_E1000_N_RX_DESC-1;
    dev->rx_ring.free = 0;
    // give the controller ownership of all receive descriptors
    e1000_outl(dev, E1000_RDH, 0);
    e1000_outl(dev, E1000_RDT, CONFIG_E1000_N_RX_DESC-1);
    // tell controller the location, size, and fetch-policy for RX queue
    e1000_outl(dev, E1000_RDBAL, rxd_phys);
    e1000_outl(dev, E1000_RDBAH, 0x00000000);
    e1000_outl(dev, E1000_RDLEN, CONFIG_E1000_N_RX_DESC*16);
    e1000_outl(dev, E1000_RXDCTL, 0x01010000);

    e1000_turn_on(dev);
}

/****************************************************************************
 * Function: e1000_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   e1000  - Reference to the driver state structure
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

static int e1000_transmit(struct e1000_dev *e1000)
{
    int tail = e1000->tx_ring.tail;
    unsigned char *cp = (unsigned char *)
		(e1000->tx_ring.buf + tail * CONFIG_E1000_BUFF_SIZE);
    int count = e1000->uip_dev.d_len;

    /* Verify that the hardware is ready to send another packet.  If we get
     * here, then we are committed to sending a packet; Higher level logic
     * must have assured that there is not transmission in progress.
     */

    if (!e1000->tx_ring.desc[tail].desc_status)
		return -1;

    /* Increment statistics */

    /* Send the packet: address=skel->sk_dev.d_buf, length=skel->sk_dev.d_len */
    memcpy(cp, e1000->uip_dev.d_buf, e1000->uip_dev.d_len);

    // prepare the transmit-descriptor
    e1000->tx_ring.desc[tail].packet_length = count<60 ? 60:count;
    e1000->tx_ring.desc[tail].desc_status = 0;

    // give ownership of this descriptor to the network controller
    tail = (tail + 1) % CONFIG_E1000_N_TX_DESC;
    e1000->tx_ring.tail = tail;
    e1000_outl(e1000, E1000_TDT, tail);

    /* Enable Tx interrupts */

    /* Setup the TX timeout watchdog (perhaps restarting the timer) */

    wd_start(e1000->txtimeout, E1000_TXTIMEOUT, e1000_txtimeout, 1, (uint32_t)e1000);
    return OK;
}

/****************************************************************************
 * Function: e1000_uiptxpoll
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

static int e1000_uiptxpoll(struct uip_driver_s *dev)
{
    struct e1000_dev *e1000 = (struct e1000_dev *)dev->d_private;
    int tail = e1000->tx_ring.tail;

    /* If the polling resulted in data that should be sent out on the network,
     * the field d_len is set to a value > 0.
     */

    if (e1000->uip_dev.d_len > 0) {
		uip_arp_out(&e1000->uip_dev);
		e1000_transmit(e1000);

		/* Check if there is room in the device to hold another packet. If not,
		 * return a non-zero value to terminate the poll.
		 */
		if (!e1000->tx_ring.desc[tail].desc_status)
			return -1;
    }

    /* If zero is returned, the polling will continue until all connections have
     * been examined.
     */

    return 0;
}

/****************************************************************************
 * Function: e1000_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   e1000  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void e1000_receive(struct e1000_dev *e1000)
{
    int head = e1000->rx_ring.head;
    unsigned char *cp = (unsigned char *)
		(e1000->rx_ring.buf + head * CONFIG_E1000_BUFF_SIZE);
    int cnt;

    while (e1000->rx_ring.desc[head].desc_status) {

		/* Check for errors and update statistics */
	
		// Here we do not handle packets that exceed packet-buffer size
		if ((e1000->rx_ring.desc[head].desc_status & 3) == 1) {
			cprintf("NIC READ: Oversized packet\n");
			goto next;
		}
	
		/* Check if the packet is a valid size for the uIP buffer configuration */
	
		// get the number of actual data-bytes in this packet
		cnt = e1000->rx_ring.desc[head].packet_length;
	
		if (cnt > CONFIG_NET_BUFSIZE || cnt < 14) {
			cprintf("NIC READ: invalid package size\n");
			goto next;
		}
    
		/* Copy the data data from the hardware to e1000->uip_dev.d_buf.  Set
		 * amount of data in e1000->uip_dev.d_len
		 */
    
		// now we try to copy these data-bytes to the UIP buffer
		memcpy(e1000->uip_dev.d_buf, cp, cnt);
		e1000->uip_dev.d_len = cnt;

		/* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
		if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
			if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
			{
				uip_arp_ipin(&e1000->uip_dev);
				uip_input(&e1000->uip_dev);

				/* If the above function invocation resulted in data that should be
				 * sent out on the network, the field  d_len will set to a value > 0.
				 */

				if (e1000->uip_dev.d_len > 0) {
					uip_arp_out(&e1000->uip_dev);
					e1000_transmit(e1000);
				}
			}
			else if (BUF->type == htons(UIP_ETHTYPE_ARP)) {
				uip_arp_arpin(&e1000->uip_dev);

				/* If the above function invocation resulted in data that should be
				 * sent out on the network, the field  d_len will set to a value > 0.
				 */

				if (e1000->uip_dev.d_len > 0) {
					e1000_transmit(e1000);
				}
			}

    next:
		e1000->rx_ring.desc[head].desc_status = 0;
		e1000->rx_ring.head = (head + 1) % CONFIG_E1000_N_RX_DESC;
		e1000->rx_ring.free++;
		head = e1000->rx_ring.head;
		cp = (unsigned char *)(e1000->rx_ring.buf + head * CONFIG_E1000_BUFF_SIZE);
    }
}

/****************************************************************************
 * Function: e1000_txtimeout
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

static void e1000_txtimeout(int argc, uint32_t arg, ...)
{
    struct e1000_dev *e1000 = (struct e1000_dev *)arg;

    /* Increment statistics and dump debug info */

    /* Then reset the hardware */
    e1000_init(e1000);

    /* Then poll uIP for new XMIT data */

    (void)uip_poll(&e1000->uip_dev, e1000_uiptxpoll);
}

/****************************************************************************
 * Function: e1000_polltimer
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

static void e1000_polltimer(int argc, uint32_t arg, ...)
{
    struct e1000_dev *e1000 = (struct e1000_dev *)arg;
    int tail = e1000->tx_ring.tail;

    /* Check if there is room in the send another TX packet.  We cannot perform
     * the TX poll if he are unable to accept another packet for transmission.
     */
    if (!e1000->tx_ring.desc[tail].desc_status)
		return;

    /* If so, update TCP timing states and poll uIP for new XMIT data. Hmmm..
     * might be bug here.  Does this mean if there is a transmit in progress,
     * we will missing TCP time state updates?
     */

    (void)uip_timer(&e1000->uip_dev, e1000_uiptxpoll, E1000_POLLHSEC);

    /* Setup the watchdog poll timer again */

    (void)wd_start(e1000->txpoll, E1000_WDDELAY, e1000_polltimer, 1, arg);
}

/****************************************************************************
 * Function: e1000_ifup
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

static int e1000_ifup(struct uip_driver_s *dev)
{
    struct e1000_dev *e1000 = (struct e1000_dev *)dev->d_private;

    ndbg("Bringing up: %d.%d.%d.%d\n",
		 dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
		 (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

    /* Initialize PHYs, the Ethernet interface, and setup up Ethernet interrupts */
    e1000_init(e1000);

    /* Set and activate a timer process */

    (void)wd_start(e1000->txpoll, E1000_WDDELAY, e1000_polltimer, 1, (uint32_t)e1000);

    if (e1000_inl(e1000, E1000_STATUS) & 2)
		e1000->bifup = true;
    else
		e1000->bifup = false;

    return OK;
}

/****************************************************************************
 * Function: e1000_ifdown
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

static int e1000_ifdown(struct uip_driver_s *dev)
{
    struct e1000_dev *e1000 = (struct e1000_dev *)dev->d_private;
    irqstate_t flags;

    /* Disable the Ethernet interrupt */

    flags = irqsave();

    e1000_turn_off(e1000);

    /* Cancel the TX poll timer and TX timeout timers */

    wd_cancel(e1000->txpoll);
    wd_cancel(e1000->txtimeout);

    /* Put the the EMAC is its reset, non-operational state.  This should be
     * a known configuration that will guarantee the skel_ifup() always
     * successfully brings the interface back up.
     */
    //e1000_reset(e1000);

    /* Mark the device "down" */

    e1000->bifup = false;
    irqrestore(flags);

    return OK;
}

/****************************************************************************
 * Function: e1000_txavail
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

static int e1000_txavail(struct uip_driver_s *dev)
{
    struct e1000_dev *e1000 = (struct e1000_dev *)dev->d_private;
    int tail = e1000->tx_ring.tail;
    irqstate_t flags;

    /* Disable interrupts because this function may be called from interrupt
     * level processing.
     */

    flags = irqsave();

    /* Ignore the notification if the interface is not yet up */

    if (e1000->bifup) {
		/* Check if there is room in the hardware to hold another outgoing packet. */
		if (e1000->tx_ring.desc[tail].desc_status)
			(void)uip_poll(&e1000->uip_dev, e1000_uiptxpoll);
    }

    irqrestore(flags);
    return OK;
}

/****************************************************************************
 * Function: e1000_addmac
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
static int e1000_addmac(struct uip_driver_s *dev, const uint8_t *mac)
{
	struct e1000_dev *e1000 = (struct e1000_dev *)dev->d_private;

	/* Add the MAC address to the hardware multicast routing table */

	return OK;
}
#endif

/****************************************************************************
 * Function: e1000_rmmac
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
static int e1000_rmmac(struct uip_driver_s *dev, const uint8_t *mac)
{
	struct e1000_dev *e1000 = (struct e1000_dev *)dev->d_private;

	/* Add the MAC address to the hardware multicast routing table */

	return OK;
}
#endif

static irqreturn_t e1000_interrupt_handler(int irq, void *dev_id)
{
    struct e1000_dev *e1000 = (struct e1000_dev *)dev_id;
    
    /* Get and clear interrupt status bits */
    int intr_cause = e1000_inl(e1000, E1000_ICR);
    e1000_outl(e1000, E1000_ICR, intr_cause);

    // not for me
    if (intr_cause == 0) 
		return IRQ_NONE;

    /* Handle interrupts according to status bit settings */

    // Link status change
    if (intr_cause & (1<<2)) {
		if (e1000_inl(e1000, E1000_STATUS) & 2)
			e1000->bifup = true;
		else
			e1000->bifup = false;
    }
    
    /* Check if we received an incoming packet, if so, call skel_receive() */

    // Rx-descriptor Timer expired
    if (intr_cause & (1<<7))
		e1000_receive(e1000);

    // Tx queue empty
    if (intr_cause & (1<<1))
		wd_cancel(e1000->txtimeout);

    /* Check is a packet transmission just completed.  If so, call skel_txdone.
     * This may disable further Tx interrupts if there are no pending
     * tansmissions.
     */

    // Tx-descriptor Written back
    if (intr_cause & (1<<0))
		uip_poll(&e1000->uip_dev, e1000_uiptxpoll);
  

    // Rx-Descriptors Low
    if (intr_cause & (1<<4)) {
		int tail;
		tail = e1000->rx_ring.tail + e1000->rx_ring.free;
		tail %= CONFIG_E1000_N_RX_DESC;
		e1000->rx_ring.tail = tail;
		e1000->rx_ring.free = 0;
		e1000_outl(e1000, E1000_RDT, tail);
    }

    return IRQ_HANDLED;
}

/******************************* PCI driver *********************************/

static pci_id_t e1000_id_table[] = {
    {.sep = {INTEL_VENDERID, E1000_82573L}},
    {.sep = {INTEL_VENDERID, E1000_82540EM}},
    {.sep = {INTEL_VENDERID, E1000_82574L}},
    {.sep = {INTEL_VENDERID, E1000_82567LM}},
    {.sep = {INTEL_VENDERID, E1000_82541PI}},
    {.sep = {0,0}}	
};

static int e1000_probe(uint16_t addr, pci_id_t id)
{
    uint32_t mmio_base, mmio_size;
    uint32_t size;
    int err;
    void *kmem, *omem;
    struct e1000_dev *dev;

    // alloc e1000_dev memory
    if ((dev = kzalloc(sizeof(struct e1000_dev))) == NULL)
		return -1;

	// save pci addr
	dev->pci_addr = addr;

    // enable device
	if ((err = pci_enable_device(addr, PCI_BUS_MASTER)) < 0)
		goto error;

    // get e1000 device type
    dev->pci_dev_id = id.join;

    // remap the controller's i/o-memory into kernel's address-space
    mmio_base = pci_resource_start(addr, 0);
    mmio_size = pci_resource_len(addr, 0);
    err = rgmp_memmap_nocache(mmio_base, mmio_size, mmio_base);
    if (err) 
		goto error;
    dev->phy_mem_base = mmio_base;
    dev->io_mem_base = mmio_base;
    dev->mem_size = mmio_size;

    // MAC address
    memset(dev->dst_mac, 0xFF, 6);
    memcpy(dev->src_mac, (void *)(dev->io_mem_base+E1000_RA), 6);

    // IRQ setup
    dev->int_desc.handler = e1000_interrupt_handler;
    dev->int_desc.dev_id = dev;
	if ((err = pci_request_irq(addr, &dev->int_desc, 0)) < 0)
		goto err0;

    // Here we alloc a big block of memory once and make it
    // aligned to page boundary and multiple of page size. This
    // is because the memory can be modified by E1000 DMA and
    // should be mapped no-cache which will hugely reduce memory 
    // access performance. The page size alloc will restrict
    // this bad effect only within the memory we alloc here.
	//
	// NEED FIX: the memalign may alloc memory continous in
	// virtual address but dis-continous in physical address
	// due to RGMP memory setup.
    size = CONFIG_E1000_N_TX_DESC * sizeof(struct tx_desc) +
		CONFIG_E1000_N_TX_DESC * CONFIG_E1000_BUFF_SIZE +
		CONFIG_E1000_N_RX_DESC * sizeof(struct rx_desc) + 
		CONFIG_E1000_N_RX_DESC * CONFIG_E1000_BUFF_SIZE;
    size = ROUNDUP(size, PGSIZE);
    omem = kmem = memalign(PGSIZE, size);
    if (kmem == NULL) {
		err = -ENOMEM;
		goto err1;
    }
    rgmp_memremap_nocache((uintptr_t)kmem, size);

    // alloc memory for tx ring
    dev->tx_ring.desc = (struct tx_desc*)kmem;
    kmem += CONFIG_E1000_N_TX_DESC * sizeof(struct tx_desc);
    dev->tx_ring.buf = kmem;
    kmem += CONFIG_E1000_N_TX_DESC * CONFIG_E1000_BUFF_SIZE;

    // alloc memory for rx rings
    dev->rx_ring.desc = (struct rx_desc*)kmem;
    kmem += CONFIG_E1000_N_RX_DESC * sizeof(struct rx_desc);
    dev->rx_ring.buf = kmem;

    /* Initialize the driver structure */

    dev->uip_dev.d_ifup    = e1000_ifup;     /* I/F up (new IP address) callback */
    dev->uip_dev.d_ifdown  = e1000_ifdown;   /* I/F down callback */
    dev->uip_dev.d_txavail = e1000_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
    dev->uip_dev.d_addmac  = e1000_addmac;   /* Add multicast MAC address */
    dev->uip_dev.d_rmmac   = e1000_rmmac;    /* Remove multicast MAC address */
#endif
    dev->uip_dev.d_private = dev;            /* Used to recover private state from dev */

    /* Create a watchdog for timing polling for and timing of transmisstions */

    dev->txpoll       = wd_create();         /* Create periodic poll timer */
    dev->txtimeout    = wd_create();         /* Create TX timeout timer */

    // Put the interface in the down state.
    // e1000 reset
    e1000_reset(dev);

    /* Read the MAC address from the hardware */
    memcpy(dev->uip_dev.d_mac.ether_addr_octet, (void *)(dev->io_mem_base+E1000_RA), 6);

    /* Register the device with the OS so that socket IOCTLs can be performed */
    err = netdev_register(&dev->uip_dev);
    if (err)
		goto err2;

    // insert into e1000_list
    dev->next = e1000_list.next;
    e1000_list.next = dev;
    cprintf("bring up e1000 device: %04x %08x\n", addr, id.join);

    return 0;

err2:
    rgmp_memremap((uintptr_t)omem, size);
    free(omem);
err1:
    pci_free_irq(addr);
err0:
    rgmp_memunmap(mmio_base, mmio_size);
error:
    kfree(dev);
    cprintf("e1000 device probe fail: %d\n", err);
    return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void e1000_mod_init(void)
{
    pci_probe_device(e1000_id_table, e1000_probe);
}

void e1000_mod_exit(void)
{
    uint32_t size;
    struct e1000_dev *dev;

    size = CONFIG_E1000_N_TX_DESC * sizeof(struct tx_desc) +
		CONFIG_E1000_N_TX_DESC * CONFIG_E1000_BUFF_SIZE +
		CONFIG_E1000_N_RX_DESC * sizeof(struct rx_desc) + 
		CONFIG_E1000_N_RX_DESC * CONFIG_E1000_BUFF_SIZE;
    size = ROUNDUP(size, PGSIZE);

    for (dev=e1000_list.next; dev!=NULL; dev=dev->next) {
		netdev_unregister(&dev->uip_dev);
		e1000_reset(dev);
		wd_delete(dev->txpoll);
		wd_delete(dev->txtimeout);
		rgmp_memremap((uintptr_t)dev->tx_ring.desc, size);
		free(dev->tx_ring.desc);
		pci_free_irq(dev->pci_addr);
		rgmp_memunmap((uintptr_t)dev->io_mem_base, dev->mem_size);
		kfree(dev);
    }

    e1000_list.next = NULL;
}
