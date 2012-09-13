/****************************************************************************
 * drivers/usbhost/rtl8187.c
 *
 *   Copyright (C) 2011, 2012 Gregory Nutt. All rights reserved.
 *   Authors: Rafael Noronha <rafael@pdsolucoes.com.br>
 *            Gregory Nutt <gnutt@nuttx.org>
 *
 * Portions of the logic in this file derives from the KisMAC RTL8187x driver
 *
 *    Created by pr0gg3d on 02/24/08.
 *
 * Which, in turn, came frm the SourceForge rt2x00 project:
 *
 *   Copyright (C) 2004 - 2006 rt2x00 SourceForge Project
 *   <http://rt2x00.serialmonkey.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * There are probably also pieces from the Linux RTL8187x driver
 * 
 *   Copyright 2007 Michael Wu <flamingice@sourmilk.net>
 *   Copyright 2007 Andrea Merello <andreamrl@tiscali.it>
 *
 *   Based on the r8187 driver, which is:
 *   Copyright 2004-2005 Andrea Merello <andreamrl@tiscali.it>, et al.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <time.h>
#include <wdog.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/irq.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>

#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arp.h>
#include <nuttx/net/uip/uip-arch.h>

#include "rtl8187x.h"

#if defined(CONFIG_USBHOST) && defined(CONFIG_NET) && defined(CONFIG_NET_WLAN)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NET_NOINTS
#  warning "CONFIG_NET_NOINTS must be set"
#endif

#ifndef CONFIG_NET_MULTIBUFFER
#  warning "CONFIG_NET_MULTIBUFFER must be set"
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* Driver support ***********************************************************/
/* This format is used to construct the /dev/wlan[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "wlan%d"

/* Used in rtl8187x_cfgdesc() */

#define USBHOST_IFFOUND     0x01
#define USBHOST_BINFOUND    0x02
#define USBHOST_BOUTFOUND   0x04
#define USBHOST_ALLFOUND    0x07

#define USBHOST_MAX_CREFS   0x7fff

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define RTL8187X_TXDELAY    (1*CLK_TCK)
#define RTL8187X_RETRYDELAY (CLK_TCK/2)

/* RX poll delay = 100 millseconds. */

#define RTL8187X_RXDELAY    (CLK_TCK / 10)

/* TX timeout = 1 minute */

#define RTL8187X_TXTIMEOUT  (60*CLK_TCK)

/* Statistics helper */

#ifdef CONFIG_NET_STATISTICS
#  define RTL8187X_STATS(p,f) (p->stats.f)++
#else
#  define RTL8187X_STATS(p,f)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Describes one IEEE 802.11 Channel */

struct ieee80211_channel_s
{
  uint16_t                  chan;        /* Channel number (IEEE 802.11) */
  uint16_t                  freq;        /* Frequency in MHz */
  uint32_t                  val;         /* HW specific value for the channel */
  uint32_t                  flag;        /* Flag for hostapd use (IEEE80211_CHAN_*) */
  uint8_t                   pwrlevel;
  uint8_t                   antmax;
};

/* Driver statistics */

#ifdef CONFIG_NET_STATISTICS
struct rtl8187x_statistics_s
{
  uint32_t transmitted;     /* Number of packets transmitted */
  uint32_t txfailed;        /* - Number of failed packet transmissions */
  uint32_t received;        /* Number of packets received: */
  uint32_t rxdropped;       /* RX Dropped: */
  uint32_t rxtoosmall;      /* - Number of bad, small packets received */
  uint32_t rxtoobig;        /* - Number of bad, big packets received */
  uint32_t rxcrcerr;        /* - Number of packets received with a CRC error */
  uint32_t rxbadproto;      /* - Number of dropped packets with bad protocol */
                            /* RX Good: received - rxdropped */
  uint32_t rxippackets;     /* - Number of good IP packets */
  uint32_t rxarppackets;    /* - Number of good ARP packets */
};
#endif

/* This structure contains the internal, private state of the USB host class
 * driver.
 */

struct rtl8187x_state_s
{
  /* This is the externally visible portion of the USB class state.  This must
   * be the first thing in the structure so that 'struct rtl8187x_state_s' can
   * be obtained from the class instance by a simple cast.
   */

  struct usbhost_class_s     class;

  /* This is an instance of the USB host controller driver bound to this class instance */

  struct usbhost_driver_s   *hcd;

  /* The following fields support the USB class driver */

  volatile bool              disconnected; /* TRUE: Device has been disconnected */
  bool                       bifup;        /* TRUE: Ethernet interface is up */
  bool                       silence;      /* TRUE: Packets are being received */
  uint8_t                    ifno;         /* Interface number */
  uint8_t                    asicrev;      /* ASIC revision number */
  uint8_t                    rate;         /* RX rate parameter */
  uint8_t                    width;        /* EEPROM width (see PCI_EEPROM_WIDTH_* defines) */
  uint8_t                    datain;       /* EEPROM data input */
  uint8_t                    dataout;      /* EEPROM data output */
  uint8_t                    dataclk;      /* EEPROM data clock */
  uint8_t                    chipsel;      /* EEPROM chip select */
  uint8_t                    signal;       /* Estimated signal strength */
  int8_t                     crefs;        /* >0: The driver is busy and cannot be destoryed */
  uint16_t                   rxpwrbase;    /* RX power base */
  uint32_t                   lastpoll;     /* Time of last poll */
  sem_t                      exclsem;      /* Used to maintain mutual exclusive access */
  struct work_s              wkdisconn;    /* For performing disconnect on the worker thread */
  struct work_s              wktxpoll;     /* Perform TX poll on work thread */
  struct work_s              wkrxpoll;     /* Perform RX poll on work thread */
  FAR struct usb_ctrlreq_s  *ctrlreq;      /* The allocated request buffer */
  FAR uint8_t               *tbuffer;      /* The allocated transfer buffer */
  size_t                     tbuflen;      /* Size of the allocated transfer buffer */
  usbhost_ep_t               epin;         /* IN endpoint */
  usbhost_ep_t               epout;        /* OUT endpoint */
  WDOG_ID                    wdtxpoll;     /* TX poll timer */
  WDOG_ID                    wdrxpoll;     /* RX poll timer */

  /* Chip-specific function pointers  */

  void                       (*rfinit)(FAR struct rtl8187x_state_s *);
  void                       (*settxpower)(FAR struct rtl8187x_state_s *priv, int channel);

  /* Channel configuration */

  struct ieee80211_channel_s channels[RTL8187X_NCHANNELS];

  /* Statistics */

#ifdef CONFIG_NET_STATISTICS
  struct rtl8187x_statistics_s stats;
#endif

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s        ethdev;       /* Interface understood by uIP */
  FAR uint8_t               *txbuffer;     /* The allocated TX I/O buffer */
  FAR uint8_t               *rxbuffer;     /* The allocated RX I/O buffer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* General Utility Functions ************************************************/
/* Semaphores */

static void rtl8187x_takesem(sem_t *sem);
#define rtl8187x_givesem(s) sem_post(s);

/* Memory allocation services */

static inline FAR struct rtl8187x_state_s *rtl8187x_allocclass(void);
static inline void rtl8187x_freeclass(FAR struct rtl8187x_state_s *class);

/* Standard USB host class functions ****************************************/
/* Worker thread actions */

static void rtl8187x_destroy(FAR void *arg);

/* Helpers for rtl8187x_connect() */

static inline int rtl8187x_cfgdesc(FAR struct rtl8187x_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen,
                                  uint8_t funcaddr);
static inline int rtl8187x_devinit(FAR struct rtl8187x_state_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t rtl8187x_host2le16(uint16_t val);
static inline uint16_t rtl8187x_le2host16(uint16_t val);
static inline uint32_t rtl8187x_host2le32(uint32_t val);
static inline uint32_t rtl8187x_le2host32(uint32_t val);
static inline uint16_t rtl8187x_getle16(const uint8_t *val);
static inline uint32_t rtl8187x_getle32(const uint8_t *val);
static inline void rtl8187x_putle16(uint8_t *dest, uint16_t val);
static void rtl8187x_putle32(uint8_t *dest, uint32_t val);

/* Transfer descriptor memory management */

static inline int rtl8187x_allocbuffers(FAR struct rtl8187x_state_s *priv);
static inline int rtl8187x_freebuffers(FAR struct rtl8187x_state_s *priv);

/* struct usbhost_registry_s methods */
 
static struct usbhost_class_s *rtl8187x_create(FAR struct usbhost_driver_s *hcd,
                                               FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int rtl8187x_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr);
static int rtl8187x_disconnected(FAR struct usbhost_class_s *class);

/* Vendor-Specific USB host support *****************************************/

static uint8_t rtl8187x_ioread8(struct rtl8187x_state_s *priv, uint16_t addr);
static uint16_t rtl8187x_ioread16(struct rtl8187x_state_s *priv, uint16_t addr);
static uint32_t rtl8187x_ioread32(struct rtl8187x_state_s *priv, uint16_t addr);

static int rtl8187x_iowrite8(struct rtl8187x_state_s *priv, uint16_t addr,
                             uint8_t val);
static int rtl8187x_iowrite16(struct rtl8187x_state_s *priv, uint16_t addr,
                              uint16_t val);
static int rtl8187x_iowrite32(struct rtl8187x_state_s *priv, uint16_t addr,
                              uint32_t val);

static uint16_t rtl8187x_read(FAR struct rtl8187x_state_s *priv, uint8_t addr);
static inline void rtl8187x_write_8051(FAR struct rtl8187x_state_s *priv,
                                       uint8_t addr, uint16_t data);
static inline void rtl8187x_write_bitbang(struct rtl8187x_state_s *priv,
                                          uint8_t addr, uint16_t data);
static void rtl8187x_write(FAR struct rtl8187x_state_s *priv, uint8_t addr,
                           uint16_t data);

/* Ethernet driver methods **************************************************/
/* TX logic */

static int rtl8187x_transmit(FAR struct rtl8187x_state_s *priv);
static int rtl8187x_uiptxpoll(struct uip_driver_s *dev);
static void rtl8187x_txpollwork(FAR void *arg);
static void rtl8187x_txpolltimer(int argc, uint32_t arg, ...);

/* RX logic */

static inline int rtl8187x_receive(FAR struct rtl8187x_state_s *priv, unsigned int iolen, unsigned int *pktlen);
static inline void rtl8187x_rxdispatch(FAR struct rtl8187x_state_s *priv, unsigned int pktlen);
static void rtl8187x_rxpollwork(FAR void *arg);
static void rtl8187x_rxpolltimer(int argc, uint32_t arg, ...);

/* Network callback functions */

static int rtl8187x_ifup(struct uip_driver_s *dev);
static int rtl8187x_ifdown(struct uip_driver_s *dev);
static int rtl8187x_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int rtl8187x_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
static int rtl8187x_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
#endif

/* EEPROM support */

static inline void rtl8187x_eeprom_pulsehigh(FAR struct rtl8187x_state_s *priv);
static inline void rtl8187x_eeprom_pulselow(FAR struct rtl8187x_state_s *priv);
static void rtl8187x_eeprom_rdsetup(FAR struct rtl8187x_state_s *priv);
static void rtl8187x_eeprom_wrsetup(FAR struct rtl8187x_state_s *priv);
static void rtl8187x_eeprom_cleanup(FAR struct rtl8187x_state_s *priv);
static void rtl8187x_eeprom_wrbits(FAR struct rtl8187x_state_s *priv,
                                   uint16_t data, uint16_t count);
static void rtl8187x_eeprom_rdbits(FAR struct rtl8187x_state_s *priv,
                                   FAR uint16_t * data, uint16_t count);
static void rtl8187x_eeprom_read(FAR struct rtl8187x_state_s *priv,
                                 uint8_t word, uint16_t *data);
static void rtl8187x_eeprom_multiread(FAR struct rtl8187x_state_s *priv,
                                      uint8_t word, FAR uint16_t *data,
                                      uint16_t nwords);

/* PHY support */

static void rtl8187x_wrphy(FAR struct rtl8187x_state_s *priv, uint8_t addr,
                           uint32_t data);
static inline void rtl8187x_wrphyofdm(FAR struct rtl8187x_state_s *priv,
                                      uint8_t addr, uint32_t data);
static inline void rtl8187x_wrphycck(FAR struct rtl8187x_state_s *priv,
                                     uint8_t addr, uint32_t data);

/* Chip-specific RF initialization and TX power setup */

static void rtl8225_rfinit(FAR struct rtl8187x_state_s *priv);
static void rtl8225z2_rfinit(FAR struct rtl8187x_state_s *priv);
static void rtl8225_settxpower(FAR struct rtl8187x_state_s *priv, int channel);
static void rtl8225z2_settxpower(FAR struct rtl8187x_state_s *priv, int channel);

/* RTL8187 Ethernet initialization and registration */

static int rtl8187x_reset(struct rtl8187x_state_s *priv);
static void rtl8187x_setchannel(FAR struct rtl8187x_state_s *priv, int channel);
static int rtl8187x_start(FAR struct rtl8187x_state_s *priv);
static void rtl8187x_stop(FAR struct rtl8187x_state_s *priv);
static inline int rtl8187x_setup(FAR struct rtl8187x_state_s *priv);
static int rtl8187x_netinitialize(FAR struct rtl8187x_state_s *priv);
static int rtl8187x_netuninitialize(FAR struct rtl8187x_state_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID informatino that will  be 
 * used to associate the USB class driver to a connected USB device.
 */

static const const struct usbhost_id_s g_id[2] =
{
  {
    USB_CLASS_VENDOR_SPEC,  /* base */
    0xff,                   /* subclass */
    0xff,                   /* proto */
    CONFIG_USB_WLAN_VID,    /* vid */
    CONFIG_USB_WLAN_PID     /* pid */
  },
  {
    0,                      /* base */
    0,                      /* subclass */
    0,                      /* proto */
    CONFIG_USB_WLAN_VID,    /* vid */
    CONFIG_USB_WLAN_PID     /* pid */  
  }
};

/* This is the USB host wireless LAN class's registry entry */

static struct usbhost_registry_s g_wlan =
{
  NULL,                   /* flink    */
  rtl8187x_create,        /* create   */
  2,                      /* nids     */
  g_id                    /* id[]     */
};

/* This is a bitmap that is used to allocate device names /dev/wlana-z. */

static uint32_t g_devinuse;

/* Default values for IEEE 802.11 channels */

static const struct ieee80211_channel_s g_channels[RTL8187X_NCHANNELS] =
{
  { 1, 2412, 0, 0, 0, 0}, { 2, 2417, 0, 0, 0, 0},
  { 3, 2422, 0, 0, 0, 0}, { 4, 2427, 0, 0, 0, 0},
  { 5, 2432, 0, 0, 0, 0}, { 6, 2437, 0, 0, 0, 0},
  { 7, 2442, 0, 0, 0, 0}, { 8, 2447, 0, 0, 0, 0},
  { 9, 2452, 0, 0, 0, 0}, {10, 2457, 0, 0, 0, 0},
  {11, 2462, 0, 0, 0, 0}, {12, 2467, 0, 0, 0, 0},
  {13, 2472, 0, 0, 0, 0}, {14, 2484, 0, 0, 0, 0}
};

static const uint32_t g_chanselect[RTL8187X_NCHANNELS] =
{
  0x085c, 0x08dc, 0x095c, 0x09dc, 0x0a5c, 0x0adc, 0x0b5c,
  0x0bdc, 0x0c5c, 0x0cdc, 0x0d5c, 0x0ddc, 0x0e5c, 0x0f72
};

/* RTL8225-specific settings */

static const uint16_t g_rtl8225bcd_rxgain[] =
{
  0x0400, 0x0401, 0x0402, 0x0403, 0x0404, 0x0405, 0x0408, 0x0409,
  0x040a, 0x040b, 0x0502, 0x0503, 0x0504, 0x0505, 0x0540, 0x0541,
  0x0542, 0x0543, 0x0544, 0x0545, 0x0580, 0x0581, 0x0582, 0x0583,
  0x0584, 0x0585, 0x0588, 0x0589, 0x058a, 0x058b, 0x0643, 0x0644,
  0x0645, 0x0680, 0x0681, 0x0682, 0x0683, 0x0684, 0x0685, 0x0688,
  0x0689, 0x068a, 0x068b, 0x068c, 0x0742, 0x0743, 0x0744, 0x0745,
  0x0780, 0x0781, 0x0782, 0x0783, 0x0784, 0x0785, 0x0788, 0x0789,
  0x078a, 0x078b, 0x078c, 0x078d, 0x0790, 0x0791, 0x0792, 0x0793,
  0x0794, 0x0795, 0x0798, 0x0799, 0x079a, 0x079b, 0x079c, 0x079d,
  0x07a0, 0x07a1, 0x07a2, 0x07a3, 0x07a4, 0x07a5, 0x07a8, 0x07a9,
  0x07aa, 0x07ab, 0x07ac, 0x07ad, 0x07b0, 0x07b1, 0x07b2, 0x07b3,
  0x07b4, 0x07b5, 0x07b8, 0x07b9, 0x07ba, 0x07bb, 0x07bb
};

static const uint8_t g_rtl8225_agc[] =
{
  0x9e, 0x9e, 0x9e, 0x9e, 0x9e, 0x9e, 0x9e, 0x9e,
  0x9d, 0x9c, 0x9b, 0x9a, 0x99, 0x98, 0x97, 0x96,
  0x95, 0x94, 0x93, 0x92, 0x91, 0x90, 0x8f, 0x8e,
  0x8d, 0x8c, 0x8b, 0x8a, 0x89, 0x88, 0x87, 0x86,
  0x85, 0x84, 0x83, 0x82, 0x81, 0x80, 0x3f, 0x3e,
  0x3d, 0x3c, 0x3b, 0x3a, 0x39, 0x38, 0x37, 0x36,
  0x35, 0x34, 0x33, 0x32, 0x31, 0x30, 0x2f, 0x2e,
  0x2d, 0x2c, 0x2b, 0x2a, 0x29, 0x28, 0x27, 0x26,
  0x25, 0x24, 0x23, 0x22, 0x21, 0x20, 0x1f, 0x1e,
  0x1d, 0x1c, 0x1b, 0x1a, 0x19, 0x18, 0x17, 0x16,
  0x15, 0x14, 0x13, 0x12, 0x11, 0x10, 0x0f, 0x0e,
  0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06,
  0x05, 0x04, 0x03, 0x02, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01
};
static const uint8_t g_rtl8225_gain[] =
{
  0x23, 0x88, 0x7c, 0xa5,       /* -82dBm */
  0x23, 0x88, 0x7c, 0xb5,       /* -82dBm */
  0x23, 0x88, 0x7c, 0xc5,       /* -82dBm */
  0x33, 0x80, 0x79, 0xc5,       /* -78dBm */
  0x43, 0x78, 0x76, 0xc5,       /* -74dBm */
  0x53, 0x60, 0x73, 0xc5,       /* -70dBm */
  0x63, 0x58, 0x70, 0xc5,       /* -66dBm */
};

static const uint8_t g_rtl8225_threshold[] =
{
  0x8d, 0x8d, 0x8d, 0x8d, 0x9d, 0xad, 0xbd
};

static const uint8_t g_rtl8225_txgaincckofdm[] =
{
  0x02, 0x06, 0x0e, 0x1e, 0x3e, 0x7e
};

static const uint8_t g_rtl8225_txpowercck[] =
{
  0x18, 0x17, 0x15, 0x11, 0x0c, 0x08, 0x04, 0x02,
  0x1b, 0x1a, 0x17, 0x13, 0x0e, 0x09, 0x04, 0x02,
  0x1f, 0x1e, 0x1a, 0x15, 0x10, 0x0a, 0x05, 0x02,
  0x22, 0x21, 0x1d, 0x18, 0x11, 0x0b, 0x06, 0x02,
  0x26, 0x25, 0x21, 0x1b, 0x14, 0x0d, 0x06, 0x03,
  0x2b, 0x2a, 0x25, 0x1e, 0x16, 0x0e, 0x07, 0x03
};

static const uint8_t g_rtl8225_txpowercckch14[] =
{
  0x18, 0x17, 0x15, 0x0c, 0x00, 0x00, 0x00, 0x00,
  0x1b, 0x1a, 0x17, 0x0e, 0x00, 0x00, 0x00, 0x00,
  0x1f, 0x1e, 0x1a, 0x0f, 0x00, 0x00, 0x00, 0x00,
  0x22, 0x21, 0x1d, 0x11, 0x00, 0x00, 0x00, 0x00,
  0x26, 0x25, 0x21, 0x13, 0x00, 0x00, 0x00, 0x00,
  0x2b, 0x2a, 0x25, 0x15, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t g_rtl8225_txpowerofdm[] =
{
  0x80, 0x90, 0xa2, 0xb5, 0xcb, 0xe4
};

/* RTL8225z2-Specific settings */

static const uint8_t  g_rtl8225z2_txpowercckch14[] =
{
  0x36, 0x35, 0x2e, 0x1b, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t g_rtl8225z2_txpowercck[] =
{
  0x36, 0x35, 0x2e, 0x25, 0x1c, 0x12, 0x09, 0x04
};

static const uint8_t g_rtl8225z2_txpowerofdm[] =
{
  0x42, 0x00, 0x40, 0x00, 0x40
};

static const uint8_t g_rtl8225z2_txgaincckofdm[] =
{
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
  0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11,
  0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
  0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
  0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23
};

static const uint16_t g_rtl8225z2_rxgain[] =
{
  0x0400, 0x0401, 0x0402, 0x0403, 0x0404, 0x0405, 0x0408, 0x0409,
  0x040a, 0x040b, 0x0502, 0x0503, 0x0504, 0x0505, 0x0540, 0x0541,
  0x0542, 0x0543, 0x0544, 0x0545, 0x0580, 0x0581, 0x0582, 0x0583,
  0x0584, 0x0585, 0x0588, 0x0589, 0x058a, 0x058b, 0x0643, 0x0644,
  0x0645, 0x0680, 0x0681, 0x0682, 0x0683, 0x0684, 0x0685, 0x0688,
  0x0689, 0x068a, 0x068b, 0x068c, 0x0742, 0x0743, 0x0744, 0x0745,
  0x0780, 0x0781, 0x0782, 0x0783, 0x0784, 0x0785, 0x0788, 0x0789,
  0x078a, 0x078b, 0x078c, 0x078d, 0x0790, 0x0791, 0x0792, 0x0793,
  0x0794, 0x0795, 0x0798, 0x0799, 0x079a, 0x079b, 0x079c, 0x079d,
  0x07a0, 0x07a1, 0x07a2, 0x07a3, 0x07a4, 0x07a5, 0x07a8, 0x07a9,
  0x03aa, 0x03ab, 0x03ac, 0x03ad, 0x03b0, 0x03b1, 0x03b2, 0x03b3,
  0x03b4, 0x03b5, 0x03b8, 0x03b9, 0x03ba, 0x03bb, 0x03bb
};

static const uint8_t g_rtl8225z2_gainbg[] =
{
  0x23, 0x15, 0xa5,             /* -82-1dBm */
  0x23, 0x15, 0xb5,             /* -82-2dBm */
  0x23, 0x15, 0xc5,             /* -82-3dBm */
  0x33, 0x15, 0xc5,             /* -78dBm */
  0x43, 0x15, 0xc5,             /* -74dBm */
  0x53, 0x15, 0xc5,             /* -70dBm */
  0x63, 0x15, 0xc5              /* -66dBm */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8187x_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static void rtl8187x_takesem(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: rtl8187x_allocclass
 *
 * Description:
 *   This is really part of the logic that implements the create() method
 *   of struct usbhost_registry_s.  This function allocates memory for one
 *   new class instance.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s.  NULL is returned on failure; this function will
 *   will fail only if there are insufficient resources to create another
 *   USB host class instance.
 *
 ****************************************************************************/

static inline FAR struct rtl8187x_state_s *rtl8187x_allocclass(void)
{
  FAR struct rtl8187x_state_s *priv;

  DEBUGASSERT(!up_interrupt_context());
  priv = (FAR struct rtl8187x_state_s *)kmalloc(sizeof(struct rtl8187x_state_s));
  uvdbg("Allocated: %p\n", priv);;
  return priv;
}

/****************************************************************************
 * Name: rtl8187x_freeclass
 *
 * Description:
 *   Free a class instance previously allocated by rtl8187x_allocclass().
 *
 * Input Parameters:
 *   class - A reference to the class instance to be freed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline void rtl8187x_freeclass(FAR struct rtl8187x_state_s *class)
{
  DEBUGASSERT(class != NULL);

  /* Free the class instance (calling sched_free() in case we are executing
   * from an interrupt handler.
   */

  uvdbg("Freeing: %p\n", class);
  kfree(class);
}

/****************************************************************************
 * Name: rtl8187x_destroy
 *
 * Description:
 *   The USB device has been disconnected and the refernce count on the USB
 *   host class instance has gone to 1.. Time to destroy the USB host class
 *   instance.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rtl8187x_destroy(FAR void *arg)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)arg;

  DEBUGASSERT(priv != NULL);
  uvdbg("crefs: %d\n", priv->crefs);
 
  /* Unregister WLAN network interface */

  rtl8187x_netuninitialize(priv);

  /* Free the endpoints */

  if (priv->epout)
    {
      DRVR_EPFREE(priv->hcd, priv->epout);
    }

  if (priv->epin)
    {
      DRVR_EPFREE(priv->hcd, priv->epin);
    }

  /* Free any transfer buffers */

  rtl8187x_freebuffers(priv);

  /* Destroy the semaphores */
  
  sem_destroy(&priv->exclsem);

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(priv->hcd);

  /* And free the class instance.  Hmmm.. this may execute on the worker
   * thread and the work structure is part of what is getting freed.  That
   * should be okay because once the work contained is removed from the
   * queue, it should not longer be accessed by the worker thread.
   */

  rtl8187x_freeclass(priv);
}

/****************************************************************************
 * Name: rtl8187x_cfgdesc
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   priv - The USB host class instance.
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int rtl8187x_cfgdesc(FAR struct rtl8187x_state_s *priv,
                                   FAR const uint8_t *configdesc, int desclen,
                                   uint8_t funcaddr)
{
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usbhost_epdesc_s bindesc;
  FAR struct usbhost_epdesc_s boutdesc;
  int remaining;
  uint8_t found = 0;
  int ret;

  uvdbg("desclen:%d funcaddr:%d\n", desclen, funcaddr);
  DEBUGASSERT(priv != NULL && 
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));
  
  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (FAR struct usb_cfgdesc_s *)configdesc;
  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  /* Get the total length of the configuration descriptor (little endian).
   * It might be a good check to get the number of interfaces here too.
  */

  remaining = (int)rtl8187x_getle16(cfgdesc->totallen);

  /* Skip to the next entry descriptor */

  configdesc += cfgdesc->len;
  remaining  -= cfgdesc->len;

  /* Loop while there are more descriptors to examine */

  while (remaining >= sizeof(struct usb_desc_s))
    {
      /* What is the next descriptor? */

      desc = (FAR struct usb_desc_s *)configdesc;
      switch (desc->type)
        {
        /* Interface descriptor. We really should get the number of endpoints
         * from this descriptor too.
         */

        case USB_DESC_TYPE_INTERFACE:
          {
            FAR struct usb_ifdesc_s *ifdesc = (FAR struct usb_ifdesc_s *)configdesc;
 
            uvdbg("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Save the interface number and mark ONLY the interface found */

            priv->ifno = ifdesc->ifno;
            found      = USBHOST_IFFOUND;
          }
          break;

        /* Endpoint descriptor.  Here, we expect two bulk endpoints, an IN
         * and an OUT.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)configdesc;

            uvdbg("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for a bulk endpoint. */

#warning "Review needed"
/* For RTL8187B, the Linux driver hardcodes EP 3 for receiving and EP 2 for transmitting.
 * Otherwise, it uses EP 1 receiving and some other EP for transmitting (maybe 12).
 */
            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_BULK)
              {
                /* Yes.. it is a bulk endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an OUT bulk endpoint.  There should be only one
                     * bulk OUT endpoint.
                     */

                    if ((found & USBHOST_BOUTFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }
                    found |= USBHOST_BOUTFOUND;

                    /* Save the bulk OUT endpoint information */

                    boutdesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    boutdesc.in           = false;
                    boutdesc.funcaddr     = funcaddr;
                    boutdesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    boutdesc.interval     = epdesc->interval;
                    boutdesc.mxpacketsize = rtl8187x_getle16(epdesc->mxpacketsize);
                    uvdbg("Bulk OUT EP addr:%d mxpacketsize:%d\n",
                          boutdesc.addr, boutdesc.mxpacketsize);
                  }
                else
                  {
                    /* It is an IN bulk endpoint.  There should be only one
                     * bulk IN endpoint.
                     */

                    if ((found & USBHOST_BINFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }
                    found |= USBHOST_BINFOUND;

                    /* Save the bulk IN endpoint information */
                    
                    bindesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    bindesc.in           = 1;
                    bindesc.funcaddr     = funcaddr;
                    bindesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    bindesc.interval     = epdesc->interval;
                    bindesc.mxpacketsize = rtl8187x_getle16(epdesc->mxpacketsize);
                    uvdbg("Bulk IN EP addr:%d mxpacketsize:%d\n",
                          bindesc.addr, bindesc.mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          break;
        }

      /* If we found everything we need with this interface, then break out
       * of the loop early.
       */

      if (found == USBHOST_ALLFOUND)
        {
          break;
        }

      /* Increment the address of the next descriptor */
 
      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? */
    
  if (found != USBHOST_ALLFOUND)
    {
      udbg("ERROR: Found IF:%s BIN:%s BOUT:%s\n",
           (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
           (found & USBHOST_BINFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_BOUTFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the endpoints */

  ret = DRVR_EPALLOC(priv->hcd, &boutdesc, &priv->epout);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate Bulk OUT endpoint\n");
      return ret;
    }

  ret = DRVR_EPALLOC(priv->hcd, &bindesc, &priv->epin);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate Bulk IN endpoint\n");
      (void)DRVR_EPFREE(priv->hcd, priv->epout);
      return ret;
    }

  uvdbg("Endpoints allocated\n");
  return OK;
}

/****************************************************************************
 * Name: rtl8187x_devinit
 *
 * Description:
 *   The USB device has been successfully connected.  This completes the
 *   initialization operations.  It is first called after the
 *   configuration descriptor has been received.
 *
 *   This function is called from the connect() method.  This function always
 *   executes on the thread of the caller of connect().
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline int rtl8187x_devinit(FAR struct rtl8187x_state_s *priv)
{
  int ret = OK;

  /* Set aside a transfer buffer for exclusive use by the class driver */

  /* Increment the reference count.  This will prevent rtl8187x_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Configure the device and register the network driver */

  uvdbg("Register ethernet device\n");
  ret = rtl8187x_netinitialize(priv);

  /* Check if we successfully initialized. We now have to be concerned
   * about asynchronous modification of crefs because the network
   * driver has been registered.
   */

  if (ret == OK)
    {
      rtl8187x_takesem(&priv->exclsem);

      /* Decrement the reference count */

      priv->crefs--;

      /* Handle a corner case where (1) open() has been called so the
       * reference count was > 2, but the device has been disconnected.
       * In this case, the class instance needs to persist until close()
       * is called.
       */

      if (priv->crefs <= 1 && priv->disconnected)
        {
          /* The will cause the enumeration logic to disconnect
           * the class driver.
           */

          ret = -ENODEV;
        }

      /* Release the semaphore... there is a race condition here.
       * Decrementing the reference count and releasing the semaphore
       * allows usbhost_destroy() to execute (on the worker thread);
       * the class driver instance could get destoryed before we are
       * ready to handle it!
       */

      rtl8187x_givesem(&priv->exclsem);
    }

  return ret;
}

/****************************************************************************
 * Name: rtl8187x_host2le16 and rtl8187x_host2le32
 *
 * Description:
 *   Convert a 16/32-bit value in host byte order to little endian byte order.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t rtl8187x_host2le16(uint16_t val)
{
#ifdef CONFIG_ENDIAN_BIG
  uint16_t ret = ((val & 0x00ff) << 8) |
                 ((val)) >> 8) & 0x00ff))
  return ret
#else
  return val;
#endif
}

static inline uint16_t rtl8187x_le2host16(uint16_t val)
{
#ifdef CONFIG_ENDIAN_BIG
  uint16_t ret = ((val & 0x00ff) << 8) |
                 ((val)) >> 8) & 0x00ff))
  return ret
#else
  return val;
#endif
}

static inline uint32_t rtl8187x_host2le32(uint32_t val)
{
#ifdef CONFIG_ENDIAN_BIG
  uint32_t ret = ((val & 0x000000ffL) << 24) |
                 ((val & 0x0000ff00L) <<  8) |
                 ((val & 0x00ff0000L) >>  8) |
                 ((val & 0xff000000L) >> 24))
  return ret
#else
  return val;
#endif
}

static inline uint32_t rtl8187x_le2host32(uint32_t val)
{
#ifdef CONFIG_ENDIAN_BIG
  uint32_t ret = ((val & 0x000000ffL) << 24) |
                 ((val & 0x0000ff00L) <<  8) |
                 ((val & 0x00ff0000L) >>  8) |
                 ((val & 0xff000000L) >> 24))
  return ret
#else
  return val;
#endif
}

/****************************************************************************
 * Name: rtl8187x_getle16 and rtl8187x_getle32
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t rtl8187x_getle16(const uint8_t *val)
{
  /* Little endian means LS byte first in byte stream */

  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

static inline uint32_t rtl8187x_getle32(const uint8_t *val)
{
 /* Little endian means LS halfword first in byte stream */

  return (uint32_t)rtl8187x_getle16(&val[2]) << 16 | (uint32_t)rtl8187x_getle16(val);
}

/****************************************************************************
 * Name: rtl8187x_putle16 and  rtl8187x_putle32
 *
 * Description:
 *   Put a (possibly unaligned) 16/32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rtl8187x_putle16(uint8_t *dest, uint16_t val)
{
  /* Little endian means LS byte first in byte stream */

  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

static void rtl8187x_putle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  rtl8187x_putle16(dest, (uint16_t)(val & 0xffff));
  rtl8187x_putle16(dest+2, (uint16_t)(val >> 16));
}

/****************************************************************************
 * Name: rtl8187x_allocbuffers
 *
 * Description:
 *   Allocate transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int rtl8187x_allocbuffers(FAR struct rtl8187x_state_s *priv)
{
  size_t buflen;
  int ret;

  DEBUGASSERT(priv && priv->ctrlreq == NULL && priv->tbuffer == NULL);

  /* Allocate TD buffers for use in this driver.  We will need two:  One for
   * the request and one for the data buffer.
   */

  ret = DRVR_ALLOC(priv->hcd, (FAR uint8_t **)&priv->ctrlreq, &buflen);
  if (ret != OK)
    {
      uvdbg("DRVR_ALLOC(ctrlreq) failed: %d\n", ret);
      return ret;
    }

  ret = DRVR_ALLOC(priv->hcd, &priv->tbuffer, &priv->tbuflen);
  if (ret != OK)
    {
      uvdbg("DRVR_ALLOC(tbuffer) failed: %d\n", ret);
      return ret;
    }

  /* Allocate one TX I/O buffer big enough to hold one full packet plus
   * the TX descriptor.
   */

  buflen = CONFIG_NET_BUFSIZE + 2 + SIZEOF_TXDESC;
  ret = DRVR_IOALLOC(priv->hcd, &priv->txbuffer, buflen);
  if (ret != OK)
    {
      uvdbg("DRVR_ALLOC(txbuffer) failed: %d\n", ret);
    }

  /* Allocate one RX I/O buffer big enough to hold one full packet plus
   * the RX descriptor.
   */

  buflen = CONFIG_NET_BUFSIZE + 2 + SIZEOF_RXDESC;
  ret = DRVR_IOALLOC(priv->hcd, &priv->rxbuffer, buflen);
  if (ret != OK)
    {
      uvdbg("DRVR_ALLOC(rxbuffer) failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: rtl8187x_freebuffers
 *
 * Description:
 *   Free allocated buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int rtl8187x_freebuffers(FAR struct rtl8187x_state_s *priv)
{
  DEBUGASSERT(priv);

  if (priv->ctrlreq)
    {
      DEBUGASSERT(priv->hcd && priv->tbuffer);

      /* Free the allocated control request */

      (void)DRVR_FREE(priv->hcd, (FAR uint8_t *)priv->ctrlreq);
      priv->ctrlreq = NULL;
      
      /* Free the allocated buffer */

      (void)DRVR_FREE(priv->hcd, priv->tbuffer);
      priv->tbuffer = NULL;
      priv->tbuflen = 0;
    }
  return OK;
}

/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8187x_create
 *
 * Description:
 *   This function implements the create() method of struct usbhost_registry_s. 
 *   The create() method is a callback into the class implementation.  It is
 *   used to (1) create a new instance of the USB host class state and to (2)
 *   bind a USB host driver "session" to the class instance.  Use of this
 *   create() method will support environments where there may be multiple
 *   USB ports and multiple USB devices simultaneously connected.
 *
 * Input Parameters:
 *   hcd - An instance of struct usbhost_driver_s that the class
 *     implementation will "bind" to its state structure and will
 *     subsequently use to communicate with the USB host driver.
 *   id - In the case where the device supports multiple base classes,
 *     subclasses, or protocols, this specifies which to configure for.
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate
 *   with the USB host class.  NULL is returned on failure; this function
 *   will fail only if the hcd input parameter is NULL or if there are
 *   insufficient resources to create another USB host class instance.
 *
 ****************************************************************************/

static FAR struct usbhost_class_s *rtl8187x_create(FAR struct usbhost_driver_s *hcd,
                                                   FAR const struct usbhost_id_s *id)
{
  FAR struct rtl8187x_state_s *priv;
  int ret;

  DEBUGASSERT(hcd && id);

  /* Allocate a USB host class instance */

  uvdbg("vid:%04x pid:%04x\n", id->vid, id->pid);
  priv = rtl8187x_allocclass();
  if (priv)
    {
      /* Initialize the allocated storage class instance */

      memset(priv, 0, sizeof(struct rtl8187x_state_s));

      /* Initialize networking method function pointers */

      priv->ethdev.d_ifup      = rtl8187x_ifup;     /* I/F down callback */
      priv->ethdev.d_ifdown    = rtl8187x_ifdown;   /* I/F up (new IP address) callback */
      priv->ethdev.d_txavail   = rtl8187x_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
      priv->ethdev.d_addmac    = rtl8187x_addmac;   /* Add multicast MAC address */
      priv->ethdev.d_rmmac     = rtl8187x_rmmac;    /* Remove multicast MAC address */
#endif
      priv->ethdev.d_private   = (void*)priv;       /* Used to recover private state from dev */

      /* Initialize class method function pointers */

      priv->class.connect      = rtl8187x_connect;
      priv->class.disconnected = rtl8187x_disconnected;

      /* Bind the host controller driver to the storage class instance */

      priv->hcd                = hcd;

      /* The initial reference count is 1... One reference is held by the
       * USB host controller driver
       */

      priv->crefs              = 1;

     /* Allocate buffering */

      ret = rtl8187x_allocbuffers(priv);
      if (ret != OK)
        {
          udbg("ERROR: Failed to allocate buffers: %d\n", ret);
          goto errout;
        }

      /* Create a watchdog for timed polling for and timing of transfers */

      priv->wdtxpoll            = wd_create();       /* Create periodic TX poll timer */
      priv->wdrxpoll            = wd_create();       /* Create periodic RX poll timer */

      /* Initialize semphores (this works okay in the interrupt context) */

      sem_init(&priv->exclsem, 0, 1);

      /* Return the instance of the USB class driver */
 
      return &priv->class;
    }

  /* An error occurred. Free the allocation and return NULL on all failures */

errout:
  if (priv)
    {
      rtl8187x_freeclass(priv);
    }
  return NULL;
}

/****************************************************************************
 * struct usbhost_class_s methods
 ****************************************************************************/
/****************************************************************************
 * Name: rtl8187x_connect
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 *   NOTE that the class instance remains valid upon return with a failure.  It is
 *   the responsibility of the higher level enumeration logic to call
 *   CLASS_DISCONNECTED to free up the class driver resources.
 *
 * Assumptions:
 *   - This function will *not* be called from an interrupt handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ****************************************************************************/

static int rtl8187x_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)class;
  int ret;

  DEBUGASSERT(priv != NULL && 
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = rtl8187x_cfgdesc(priv, configdesc, desclen, funcaddr);
  if (ret != OK)
    {
      udbg("rtl8187x_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      /* Now configure the device and register the NuttX driver */

      ret = rtl8187x_devinit(priv);
      if (ret != OK)
        {
          udbg("rtl8187x_devinit() failed: %d\n", ret);
        }
    }
 
  return ret;
}

/****************************************************************************
 * Name: rtl8187x_disconnected
 *
 * Description:
 *   This function implements the disconnected() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to inform the class that the USB device has
 *   been disconnected.
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to
 *     create().
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

static int rtl8187x_disconnected(struct usbhost_class_s *class)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)class;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  /* Set an indication to any users of the device that the device is no
   * longer available.
   */

  flags              = irqsave();
  priv->disconnected = true;

  /* Now check the number of references on the class instance.  The USB host
   * controller driver has just reliquished its reference.  If the reference
   * count would go to zero, then we can free the class instance now.  Otherwise,
   * we will have to wait until the holders of the references free them.
   */

  ullvdbg("crefs: %d\n", priv->crefs);
  if (--priv->crefs <= 0)
    {
      /* Destroy the class instance.  Defer the destruction to the worker thread.
       * (in case we were callded from an interrupt handler).
       */

      ullvdbg("Queuing destruction: worker %p->%p\n", priv->wkdisconn.worker, rtl8187x_destroy);
      DEBUGASSERT(priv->wkdisconn.worker == NULL);
      (void)work_queue(&priv->wkdisconn, rtl8187x_destroy, priv, 0);
    }

  irqrestore(flags);  
  return OK;
}

/****************************************************************************
 * Function: rtl8187x_ioread8/16/32
 *
 * Description:
 *   Read 8, 16, or 32 bits from the RTL8187x.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   addr  - Device addresses
 *
 * Returned Value:
 *   The value read from the the RTL8187x (no error indication returned).
 *
 * Assumptions:
 *   Not called from an interrupt handler.
 *
 ****************************************************************************/

static uint8_t rtl8187x_ioread8(struct rtl8187x_state_s *priv, uint16_t addr)
{
  FAR struct usb_ctrlreq_s *ctrlreq = priv->ctrlreq;
  int ret;

  DEBUGASSERT(ctrlreq && priv->tbuffer);
  ctrlreq->type = (USB_REQ_DIR_IN | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE);
  ctrlreq->req  = RTL8187X_REQ_GETREG;
  rtl8187x_putle16(ctrlreq->value, addr);
  rtl8187x_putle16(ctrlreq->index, 0);
  rtl8187x_putle16(ctrlreq->len, sizeof(uint8_t));

  ret = DRVR_CTRLIN(priv->hcd, priv->ctrlreq, priv->tbuffer);
  if (ret != OK)
    {
      udbg("ERROR: DRVR_CTRLIN returned %d\n", ret);
      return 0;
    }

  return *((uint8_t*)priv->tbuffer);
}

static uint16_t rtl8187x_ioread16(struct rtl8187x_state_s*priv, uint16_t addr)
{
  FAR struct usb_ctrlreq_s *ctrlreq = priv->ctrlreq;
  int ret;

  DEBUGASSERT(ctrlreq && priv->tbuffer);
  ctrlreq->type = (USB_REQ_DIR_IN | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE);
  ctrlreq->req  = RTL8187X_REQ_GETREG;
  rtl8187x_putle16(ctrlreq->value, addr);
  rtl8187x_putle16(ctrlreq->index, 0);
  rtl8187x_putle16(ctrlreq->len, sizeof(uint16_t));

  ret = DRVR_CTRLIN(priv->hcd, priv->ctrlreq, priv->tbuffer);
  if (ret != OK)
    {
      udbg("ERROR: DRVR_CTRLIN returned %d\n", ret);
      return 0;
    }

  return rtl8187x_getle16(priv->tbuffer);
}

static uint32_t rtl8187x_ioread32(struct rtl8187x_state_s*priv, uint16_t addr)
{
  FAR struct usb_ctrlreq_s *ctrlreq = priv->ctrlreq;
  int ret;

  DEBUGASSERT(ctrlreq && priv->tbuffer);
  ctrlreq->type = (USB_REQ_DIR_IN | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE);
  ctrlreq->req  = RTL8187X_REQ_GETREG;
  rtl8187x_putle16(ctrlreq->value, addr);
  rtl8187x_putle16(ctrlreq->index, 0);
  rtl8187x_putle16(ctrlreq->len, sizeof(uint32_t));

  ret = DRVR_CTRLIN(priv->hcd, priv->ctrlreq, priv->tbuffer);
  if (ret != OK)
    {
      udbg("ERROR: DRVR_CTRLIN returned %d\n", ret);
      return 0;
    }

  return rtl8187x_getle32(priv->tbuffer);
}

/****************************************************************************
 * Function: rtl8187x_iowrite8/16/32
 *
 * Description:
 *   Write a 8, 16, or 32 bits to the RTL8187x.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   addr  - Device addresses
 *   val   - The value to write
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   Not called from an interrupt handler.
 *
 ****************************************************************************/

static int rtl8187x_iowrite8(struct rtl8187x_state_s *priv, uint16_t addr, uint8_t val)
{
  FAR struct usb_ctrlreq_s *ctrlreq = priv->ctrlreq;

  DEBUGASSERT(ctrlreq && priv->tbuffer);
  ctrlreq->type = (USB_REQ_DIR_OUT | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE);
  ctrlreq->req  = RTL8187X_REQ_SETREG;
  rtl8187x_putle16(ctrlreq->value, addr);
  rtl8187x_putle16(ctrlreq->index, 0);
  rtl8187x_putle16(ctrlreq->len, sizeof(uint8_t));

  priv->tbuffer[0] = val;
  return DRVR_CTRLOUT(priv->hcd, priv->ctrlreq, priv->tbuffer);
}

static int rtl8187x_iowrite16(struct rtl8187x_state_s *priv, uint16_t addr, uint16_t val)
{
  FAR struct usb_ctrlreq_s *ctrlreq = priv->ctrlreq;

  DEBUGASSERT(ctrlreq && priv->tbuffer);
  ctrlreq->type = (USB_REQ_DIR_OUT | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE);
  ctrlreq->req  = RTL8187X_REQ_SETREG;
  rtl8187x_putle16(ctrlreq->value, addr);
  rtl8187x_putle16(ctrlreq->index, 0);
  rtl8187x_putle16(ctrlreq->len, sizeof(uint16_t));

  rtl8187x_putle16(priv->tbuffer, val);
  return DRVR_CTRLOUT(priv->hcd, priv->ctrlreq, priv->tbuffer);
}

static int rtl8187x_iowrite32(struct rtl8187x_state_s *priv, uint16_t addr, uint32_t val)
{
  FAR struct usb_ctrlreq_s *ctrlreq = priv->ctrlreq;

  DEBUGASSERT(ctrlreq && priv->tbuffer);
  ctrlreq->type = (USB_REQ_DIR_OUT | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE);
  ctrlreq->req  = RTL8187X_REQ_SETREG;
  rtl8187x_putle16(ctrlreq->value, addr);
  rtl8187x_putle16(ctrlreq->index, 0);
  rtl8187x_putle16(ctrlreq->len, sizeof(uint32_t));

  rtl8187x_putle32(priv->tbuffer, val);
  return DRVR_CTRLOUT(priv->hcd, priv->ctrlreq, priv->tbuffer);
}

/****************************************************************************
 * Function: rtl8187x_read
 *
 * Description:
 *   Read RTL register value
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   addr  - Register address
 *
 * Returned Value:
 *   Value
 *
 ****************************************************************************/

static uint16_t rtl8187x_read(FAR struct rtl8187x_state_s *priv, uint8_t addr)
{
  uint16_t reg80;
  uint16_t reg82;
  uint16_t reg84;
  uint16_t ret;
  int i;

  reg80 = rtl8187x_ioread16(priv, RTL8187X_ADDR_RFPINSOUTPUT);
  reg82 = rtl8187x_ioread16(priv, RTL8187X_ADDR_RFPINSENABLE);
  reg84 = rtl8187x_ioread16(priv, RTL8187X_ADDR_RFPINSSELECT);

  reg80 &= ~0xf;

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSENABLE, reg82 | 0x000f);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSSELECT, reg84 | 0x000f);

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80 | (1 << 2));
  usleep(4);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80);
  usleep(5);

  for (i = 4; i >= 0; i--)
    {
      uint16_t reg = reg80 | ((addr >> i) & 1);

      if (!(i & 1))
        {
          rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg);
          usleep(1);
        }

      rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg | (1 << 1));
      usleep(2);
      rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg | (1 << 1));
      usleep(2);

      if (i & 1)
        {
          rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg);
          usleep(1);
        }
    }

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT,
                    reg80 | (1 << 3) | (1 << 1));
  usleep(2);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80 | (1 << 3));
  usleep(2);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80 | (1 << 3));
  usleep(2);

  ret = 0;
  for (i = 11; i >= 0; i--)
    {
      rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80 | (1 << 3));
      usleep(1);
      rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT,
                        reg80 | (1 << 3) | (1 << 1));
      usleep(2);
      rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT,
                        reg80 | (1 << 3) | (1 << 1));
      usleep(2);
      rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT,
                        reg80 | (1 << 3) | (1 << 1));
      usleep(2);

      if (rtl8187x_ioread16(priv, RTL8187X_ADDR_RFPINSINPUT) & (1 << 1))
        ret |= 1 << i;

      rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80 | (1 << 3));
      usleep(2);
    }

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT,
                    reg80 | (1 << 3) | (1 << 2));
  usleep(2);

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSENABLE, reg82);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSSELECT, reg84);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, 0x03a0);

  return ret;
}

/****************************************************************************
 * Function: rtl8187x_write
 *
 * Description:
 *   Write RTL register value
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   addr  - Register address
 *
 * Returned Value:
 *   Value
 *
 ****************************************************************************/

static inline void rtl8187x_write_8051(FAR struct rtl8187x_state_s *priv,
                                       uint8_t addr, uint16_t data)
{
  struct usb_ctrlreq_s *ctrlreq;
  uint16_t reg80;
  uint16_t reg82;
  uint16_t reg84;
  int ret;

  reg80 = rtl8187x_ioread16(priv, RTL8187X_ADDR_RFPINSOUTPUT);
  reg82 = rtl8187x_ioread16(priv, RTL8187X_ADDR_RFPINSENABLE);
  reg84 = rtl8187x_ioread16(priv, RTL8187X_ADDR_RFPINSSELECT);

  reg80 &= ~(0x3 << 2);
  reg84 &= ~0xf;

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSENABLE, reg82 | 0x0007);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSSELECT, reg84 | 0x0007);
  usleep(10);

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80 | (1 << 2));
  usleep(2);

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80);
  usleep(10);

  ctrlreq = priv->ctrlreq;
  ctrlreq->type = (USB_REQ_DIR_OUT | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE);
  ctrlreq->req  = RTL8187X_REQ_GETREG;
  rtl8187x_putle16(ctrlreq->value, addr);
  rtl8187x_putle16(ctrlreq->index, 0x8225);
  rtl8187x_putle16(ctrlreq->len, sizeof(uint16_t));

  rtl8187x_putle16(priv->tbuffer, data);
  ret = DRVR_CTRLOUT(priv->hcd, priv->ctrlreq, priv->tbuffer);

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80 | (1 << 2));
  usleep(10);

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80 | (1 << 2));
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSSELECT, reg84);
  usleep(2000);
}

static inline void rtl8187x_write_bitbang(struct rtl8187x_state_s *priv,
                                          uint8_t addr, uint16_t data)
{
  uint16_t reg80, reg84, reg82;
  uint32_t bangdata;
  int i;

  bangdata = (data << 4) | (addr & 0xf);

  reg80 = rtl8187x_ioread16(priv, RTL8187X_ADDR_RFPINSOUTPUT) & 0xfff3;
  reg82 = rtl8187x_ioread16(priv, RTL8187X_ADDR_RFPINSENABLE);

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSENABLE, reg82 | 0x7);

  reg84 = rtl8187x_ioread16(priv, RTL8187X_ADDR_RFPINSSELECT);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSSELECT, reg84 | 0x7);
  usleep(10);

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80 | (1 << 2));
  usleep(2);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80);
  usleep(10);

  for (i = 15; i >= 0; i--)
    {
      uint16_t reg = reg80 | (bangdata & (1 << i)) >> i;

      if (i & 1)
        rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg);

      rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg | (1 << 1));
      rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg | (1 << 1));

      if (!(i & 1))
        rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg);
    }

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80 | (1 << 2));
  usleep(10);

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, reg80 | (1 << 2));
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSSELECT, reg84);
  usleep(2);
}

static void rtl8187x_write(FAR struct rtl8187x_state_s *priv, uint8_t addr, uint16_t data)
{
  if (priv->asicrev)
    {
      rtl8187x_write_8051(priv, addr, rtl8187x_host2le16(data));
    }
  else
    {
      rtl8187x_write_bitbang(priv, addr, data);
    }
}

/****************************************************************************
 * Function: rtl8187x_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the watchdog based
 *   polling or to send a response to an incoming packet.
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

static int rtl8187x_transmit(FAR struct rtl8187x_state_s *priv)
{
  int ret;

  /* Get exclusive access to the USB controller interface */

  DEBUGASSERT(priv && priv->txbuffer);
  rtl8187x_takesem(&priv->exclsem);

  /* Check if the RTL8187 is still connected */

  if (priv->disconnected)
    {
      /* No... the wan driver is no longer bound to the class.  That means that
       * the USB device is no longer connected.  Refuse any attempt to write to
       * the device.
       */

      ret = -ENODEV;
    }
  else if (!priv->bifup)
    {
      /* The interface is not up.
       */

      ret = -EAGAIN;
    }
  else
    {
      FAR struct rtl8187x_txdesc_s *txdesc = (FAR struct rtl8187x_txdesc_s *)priv->txbuffer;
      unsigned int datlen = priv->ethdev.d_len;
      uint32_t flags;
      uint32_t retry;

      /* Increment statistics */

      RTL8187X_STATS(priv, transmitted);

      /* Construct the TX descriptor at the beginning of the IO buffer.  This
       * memory was previously reserved just for this use.
       */

      flags               = datlen | RTL8187X_TXDESC_FLAG_NOENC | RTL8187X_RATE_11 << 24;
      txdesc->flags       = rtl8187x_host2le32(flags);
      txdesc->rtsduration = 0;
      txdesc->len         = 0;
      retry               = rtl8187x_host2le32(3) | /* CWMIN */
                            (7 << 4) |              /* CMAX */
                            (0 << 8);               /* retry lim */
      txdesc->retry       = rtl8187x_host2le32(retry);

#ifdef CONFIG_RTL8187B
#warning "This number is bogus"
      txdesc->txduration  = 40;
#endif

      /* And transfer the packet */

      ret = DRVR_TRANSFER(priv->hcd, priv->epout, priv->txbuffer, datlen + SIZEOF_TXDESC);
      if (ret != OK)
        {
          RTL8187X_STATS(priv, txfailed);
        }
    }
  rtl8187x_givesem(&priv->exclsem);

  return ret;
}

/****************************************************************************
 * Function: rtl8187x_uiptxpoll
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
 *  Never called from an interrupt handler.  The polling process was
 *  initiated by a normal thread (possibly the worker thread).  The initiator
 *  has called uip_lock() to assure that we have exclusive access to uIP.
 *
 ****************************************************************************/

static int rtl8187x_uiptxpoll(struct uip_driver_s *dev)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->ethdev.d_len > 0)
    {
      uip_arp_out(&priv->ethdev);
      rtl8187x_transmit(priv);
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: rtl8187x_txpollwork
 *
 * Description:
 *   Periodic timer handler.  The poll occurs on the worker thread. The
 *   poll work was scheduled by rtl8187x_txpolltimer when the poll timer
 *   expired.
 *
 * Parameters:
 *   arg  - The passed argument (priv)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void rtl8187x_txpollwork(FAR void *arg)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)arg;
  
  /* Verify that the RTL8187 is still connected and that the interface is up */

  if (!priv->disconnected && priv->bifup)
    {
      uip_lock_t lock;
      uint32_t   now;
      uint32_t   hsecs;

      /* Get exclusive access to uIP */

      lock = uip_lock();

      /* Estimate the elapsed time in hsecs since the last poll */

      now            = g_system_timer;
      hsecs          = (priv->lastpoll - now + CLK_TCK / 4) / (CLK_TCK / 2);
      priv->lastpoll = now;

      /* Update TCP timing states and poll uIP for new XMIT data. Pass an
       * offset address into the txbuffer, reserving space for the TX
       * descriptor at the beginning of the buffer.
       */

      priv->ethdev.d_buf = &priv->txbuffer[SIZEOF_TXDESC];
      (void)uip_timer(&priv->ethdev, rtl8187x_uiptxpoll, (int)hsecs);
      uip_unlock(lock);
    }
}

/****************************************************************************
 * Function: rtl8187x_txpolltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler. The
 *   actually polling is performed by on the work threader thread by
 *   rtl8187x_txpollwork().
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

static void rtl8187x_txpolltimer(int argc, uint32_t arg, ...)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)arg;
  uint32_t delay = RTL8187X_TXDELAY;

  /* Verify that the RTL8187 is still connected and that the interface is up */

  if (!priv->disconnected && priv->bifup)
    {
      /* Check for over-run... What if the last queued poll work has not yet ran?
       * That could be the case if the system were too busy, if we are polling to
       * rapidly, or if something is hung.
       */

      if (priv->wktxpoll.worker != NULL)
        {
          ulldbg("ERROR: TX work overrun!\n");
          delay = RTL8187X_RETRYDELAY;
        }
      else
        {
          (void)work_queue(&priv->wktxpoll, rtl8187x_txpollwork, priv, 0);
        }
    }

  /* Setup the watchdog poll timer again -- possibly using a shorter retry delay */

  (void)wd_start(priv->wdtxpoll, delay, rtl8187x_txpolltimer, 1, arg);
}

/****************************************************************************
 * Function: rtl8187x_receive
 *
 * Description:
 *   Called upon receipt of a new USB packet on the epin endpoint.  Analyzes
 *   the RX header in priv->rxbuffer and returns the packet length
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   iolen - The size of the received USB packet
 *   pktlen - The returned size of the packet
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller holds the exclsem and has exclusive access to the USB
 *   interface.
 *
 ****************************************************************************/

static inline int rtl8187x_receive(FAR struct rtl8187x_state_s *priv,
                                   unsigned int iolen, unsigned int *pktlen)
{
  struct rtl8187x_rxdesc_s *rxdesc;
  uint32_t flags;
  uint16_t rxlen;
  int      signal;

  /* Increment statistics */

  RTL8187X_STATS(priv, received);

  /* Make sure that the packet was large enough to contain an RX descriptor
   * and an Ethernet header
   */

  if (iolen < UIP_LLH_LEN + SIZEOF_RXDESC)
    {
      RTL8187X_STATS(priv, rxtoosmall);
      RTL8187X_STATS(priv, rxdropped);
      return -EINVAL;
    }

  /* The RX descriptor lies at the end of the IO buffer */

  rxdesc = (struct rtl8187x_rxdesc_s *)(priv->rxbuffer + (iolen - SIZEOF_RXDESC));
  flags  = rtl8187x_le2host32(rxdesc->flags);
  if (flags & RTL8187X_RXDESC_FLAG_CRC32ERR)
    {
      udbg("Bad CRC\n");
      RTL8187X_STATS(priv, rxcrcerr);
      RTL8187X_STATS(priv, rxdropped);
      return -EINVAL;
    }

  /* Get the actual packet length and rate from the RX descriptor flags */

  rxlen     = (flags & 0xfff) - 4;
  priv->rate = (flags >> 20) & 0xf;

  /* Perform signal strength calculation */

#ifdef CONFIG_RTL8187B
/* Linux has this:
 *      signal = -4 - ((27 * hdr->agc) >> 6);
 *      antenna = (hdr->signal >> 7) & 1;
 *      mactime = le64_to_cpu(hdr->mac_time)
 * Otherwise
 *      signal = 14 - hdr->agc / 2;
 *      antenna = (hdr->rssi >> 7) & 1;
 *      mactime = le64_to_cpu(hdr->mac_time
 */
#warning "Signal computations must change for RTL8187B"
#endif

  signal = rxdesc->agc >> 1;
  if (priv->rate)
    {
      /* OFDM rate */

      if (signal > 90)
        {
          signal = 90;
        }
      else if (signal < 25)
        {
          signal = 25;
        }
      signal = 90 - signal;
    }
  else
    {
      /* CCK rate */

      if (signal > 95)
        {
          signal = 95;
        }
      else if (signal < 30)
        {
          signal = 30;
        }
      signal = 95 - signal;
    }

  /* Signal strength: (100.0 / 65.0) * signal */

  priv->signal  = (uint8_t) (20 * signal + 7) / 13;
  priv->silence = false;

  /* Check if uIP is configured to handle a packet of this size */

  if (iolen > CONFIG_NET_BUFSIZE + SIZEOF_RXDESC + 2)
    {
      RTL8187X_STATS(priv, rxtoobig);
      RTL8187X_STATS(priv, rxdropped);
      return -EINVAL;
    }

  /* Return the packet length.  This is the length of the value packet
   * data at the beginning of the buffer (we no longer need the RX descriptor)
   */

  *pktlen = rxlen;
  return OK;
}

/****************************************************************************
 * Function: rtl8187x_rxdispatch
 *
 * Description:
 *   Analyzes the ethernet header of the received packet (in priv->rxbuffer)
 *   and fowards valid Ethernet packets to uIP.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   pktlen - Returned size of the packet
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static inline void rtl8187x_rxdispatch(FAR struct rtl8187x_state_s *priv,
                                       unsigned int pktlen)
{
  FAR struct uip_eth_hdr *ethhdr = (FAR struct uip_eth_hdr *)priv->rxbuffer;
  uip_lock_t lock;

  /* Get exclusive access to uIP */

  lock               = uip_lock();
  priv->ethdev.d_buf = priv->rxbuffer;
  priv->ethdev.d_len = pktlen;

  /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
  if (ethhdr->type == HTONS(UIP_ETHTYPE_IP6))
#else
  if (ethhdr->type == HTONS(UIP_ETHTYPE_IP))
#endif
    {
      RTL8187X_STATS(priv, rxippackets);
      uip_arp_ipin(&priv->ethdev);
      uip_input(&priv->ethdev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, the field  d_len will set to a value > 0.
       */

      if (priv->ethdev.d_len > 0)
        {
          uip_arp_out(&priv->ethdev);
          rtl8187x_transmit(priv);
        }
    }
  else if (ethhdr->type == htons(UIP_ETHTYPE_ARP))
    {
      RTL8187X_STATS(priv, rxarppackets);   
      uip_arp_arpin(&priv->ethdev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, the field  d_len will set to a value > 0.
       */

      if (priv->ethdev.d_len > 0)
        {
          rtl8187x_transmit(priv);
        }
    }
  else
    {
      RTL8187X_STATS(priv, rxbadproto);
      RTL8187X_STATS(priv, rxdropped);
    }

  uip_unlock(lock);
}

/****************************************************************************
 * Function: rtl8187x_rxpollwork
 *
 * Description:
 *   Periodic timer handler.  The poll occurs on the worker thread. The
 *   poll work was scheduled by rtl8187x_rxpolltimer when the poll timer
 *   expired.
 *
 * Parameters:
 *   arg  - The passed argument (priv)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void rtl8187x_rxpollwork(FAR void *arg)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)arg;
  
  /* Get exclusive access to the USB controller interface and the device
   * structure
   */

  DEBUGASSERT(priv && priv->rxbuffer);
  rtl8187x_takesem(&priv->exclsem);

  /* Verify that the RTL8187 is still connected and that the interface is up */

  if (!priv->disconnected && priv->bifup)
    {
      unsigned int iolen;
      int ret;

      /* Attempt to read from the bulkin endpoint */

      ret = DRVR_TRANSFER(priv->hcd, priv->epin, priv->rxbuffer,
                          CONFIG_NET_BUFSIZE + SIZEOF_RXDESC + 2);

      /* How dow we get the length of the transfer? */
#warning "Missing logic"
      iolen = 0;

      if (ret == OK)
        {
          unsigned int pktlen;

          /* Analyze the packet and copy it into the RX buffer */

          ret = rtl8187x_receive(priv, iolen, &pktlen);
          if (ret == OK)
            {
              /* Now we can relinquish the USB interface and device
               * structure.
               */

              rtl8187x_givesem(&priv->exclsem);

              /* Send the packet to uIP */

              rtl8187x_rxdispatch(priv, pktlen);
              return;
            }
        }
    }

  rtl8187x_givesem(&priv->exclsem);
}

/****************************************************************************
 * Function: rtl8187x_rxpolltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler. The
 *   actually polling is performed by on the work threader thread by
 *   rtl8187x_rxpollwork().
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

static void rtl8187x_rxpolltimer(int argc, uint32_t arg, ...)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)arg;

  /* Verify that the RTL8187 is still connected and that the interface is up */

  if (!priv->disconnected && priv->bifup)
    {
      /* Check for over-run... What if the last queued poll work has not yet ran?
       * That could be the case if the system were too busy, if we are polling to
       * rapidly, or if something is hung.
       */

      if (priv->wkrxpoll.worker != NULL)
        {
          ulldbg("ERROR: RX work overrun!\n");
        }
      else
        {
          (void)work_queue(&priv->wkrxpoll, rtl8187x_rxpollwork, priv, 0);
        }
    }

  /* Setup the watchdog poll timer again -- possibly using a shorter retry delay */

  (void)wd_start(priv->wdrxpoll, RTL8187X_RXDELAY, rtl8187x_rxpolltimer, 1, arg);
}

/****************************************************************************
 * Function: rtl8187x_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the WLAN interface when an IP address is
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

static int rtl8187x_ifup(struct uip_driver_s *dev)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)dev->d_private;
  int ret;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Initialize PHYs and the WLAN interface */

  ret = rtl8187x_start(priv);
  if (ret == OK)
    {
      /* Set up and activate TX timer processes */

      (void)wd_start(priv->wdtxpoll, RTL8187X_TXDELAY, rtl8187x_txpolltimer, 1, (uint32_t)priv);
      priv->lastpoll = g_system_timer;

      /* Set up and activate RX timer processes */

      (void)wd_start(priv->wdrxpoll, RTL8187X_RXDELAY, rtl8187x_rxpolltimer, 1, (uint32_t)priv);

      /* Mark the interface as up. */

      priv->bifup = true;
    }

  return ret;
}

/****************************************************************************
 * Function: rtl8187x_ifdown
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

static int rtl8187x_ifdown(struct uip_driver_s *dev)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)dev->d_private;
  irqstate_t flags;

  /* Cancel the TX and RX poll timer. */

  flags = irqsave();
  wd_cancel(priv->wdtxpoll);
  wd_cancel(priv->wdrxpoll);

  /* Put the the EMAC is its non-operational state.  This should be a known
   * configuration that will guarantee the rtl8187x_ifup() always successfully
   * successfully brings the interface back up.
   */

  rtl8187x_stop(priv);

  /* Mark the device "down" */

  priv->bifup = false;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: rtl8187x_txavail
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

static int rtl8187x_txavail(struct uip_driver_s *dev)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)dev->d_private;

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {
      uip_lock_t lock;

      /* If so, then poll uIP for new XMIT data.  Pass the offset address
       * into the txbuffer, reserving space for the TX descriptor at the
       * beginning of the buffer.
       */

      lock = uip_lock();
      priv->ethdev.d_buf = &priv->txbuffer[SIZEOF_TXDESC];
      (void)uip_poll(&priv->ethdev, rtl8187x_uiptxpoll);
      uip_unlock(lock);
    }

  return OK;
}

/****************************************************************************
 * Function: rtl8187x_addmac
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
static int rtl8187x_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: rtl8187x_rmmac
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
static int rtl8187x_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: Various low-level EEPROM Support functions
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   priv  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void rtl8187x_eeprom_pulsehigh(FAR struct rtl8187x_state_s *priv)
{
  priv->dataclk = 1;
  rtl8187x_eeprom_wrsetup(priv);

  /* Add a short delay for the pulse to work. According to the specifications
   * the "maximum minimum" time should be 450ns.
   */

  usleep(1);
}

static inline void rtl8187x_eeprom_pulselow(FAR struct rtl8187x_state_s *priv)
{
  priv->dataclk = 0;
  rtl8187x_eeprom_wrsetup(priv);

  /* Add a short delay for the pulse to work. According to the specifications
   * the "maximum minimum" time should be 450ns.
   */

  usleep(1);
}

static void rtl8187x_eeprom_rdsetup(FAR struct rtl8187x_state_s *priv)
{
  uint8_t reg = rtl8187x_ioread8(priv, RTL8187X_ADDR_EEPROMCMD);

  priv->datain  = reg & RTL8187X_EEPROMCMD_WRITE;
  priv->dataout = reg & RTL8187X_EEPROMCMD_READ;
  priv->dataclk = reg & RTL8187X_EEPROMCMD_CK;
  priv->chipsel = reg & RTL8187X_EEPROMCMD_CS;
}

static void rtl8187x_eeprom_wrsetup(FAR struct rtl8187x_state_s *priv)
{
  uint8_t reg = RTL8187X_EEPROMCMD_PROGRAM;

  if (priv->datain)
    {
        reg |= RTL8187X_EEPROMCMD_WRITE;
    }

  if (priv->dataout)
    {
        reg |= RTL8187X_EEPROMCMD_READ;
    }

  if (priv->dataclk)
    {
      reg |= RTL8187X_EEPROMCMD_CK;
    }

  if (priv->chipsel)
    {
      reg |= RTL8187X_EEPROMCMD_CS;
    }

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, reg);
  usleep(10);
}

static void rtl8187x_eeprom_cleanup(FAR struct rtl8187x_state_s *priv)
{
  /* Clear chip_select and data_in flags. */

  rtl8187x_eeprom_rdsetup(priv);
  priv->datain = 0;
  priv->chipsel = 0;
  rtl8187x_eeprom_wrsetup(priv);

  /* Kick a pulse. */

  rtl8187x_eeprom_pulsehigh(priv);
  rtl8187x_eeprom_pulselow(priv);
}

static void rtl8187x_eeprom_wrbits(FAR struct rtl8187x_state_s *priv,
                                   uint16_t data, uint16_t count)
{
  unsigned int i;

  rtl8187x_eeprom_rdsetup(priv);

  /* Clear data flags. */

  priv->datain = 0;
  priv->dataout = 0;

  /* Start writing all bits. */

  for (i = count; i > 0; i--)
    {
      /* Check if this bit needs to be set. */

      priv->datain = ! !(data & (1 << (i - 1)));

      /* Write the bit to the eeprom register. */

      rtl8187x_eeprom_wrsetup(priv);

      /* Kick a pulse. */

      rtl8187x_eeprom_pulsehigh(priv);
      rtl8187x_eeprom_pulselow(priv);
    }

  priv->datain = 0;
  rtl8187x_eeprom_wrsetup(priv);
}

static void rtl8187x_eeprom_rdbits(FAR struct rtl8187x_state_s *priv,
                                   FAR uint16_t * data, uint16_t count)
{
  unsigned int i;
  uint16_t buf = 0;

  rtl8187x_eeprom_rdsetup(priv);

  /* Clear data flags. */

  priv->datain  = 0;
  priv->dataout = 0;

  /* Start reading all bits. */

  for (i = count; i > 0; i--)
    {
      rtl8187x_eeprom_pulsehigh(priv);

      rtl8187x_eeprom_rdsetup(priv);

      /* Clear data_in flag. */

      priv->datain = 0;

      /* Read if the bit has been set. */

      if (priv->dataout)
        {
          buf |= (1 << (i - 1));
        }

      rtl8187x_eeprom_pulselow(priv);
    }

  *data = buf;
}

/****************************************************************************
 * Function: rtl8187x_eeprom_read
 *
 * Description:
 *   Read data from EEPROM
 *
 * Parameters:
 *   priv  - Reference to the NuttX driver state structure
 *   word  -
 *   data  - Location for data to be written
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rtl8187x_eeprom_read(FAR struct rtl8187x_state_s *priv,
                                 uint8_t word, uint16_t *data)
{
  uint16_t command;

  /* Clear all flags, and enable chip select. */

  rtl8187x_eeprom_rdsetup(priv);
  priv->datain  = 0;
  priv->dataout = 0;
  priv->dataclk = 0;
  priv->chipsel = 1;
  rtl8187x_eeprom_wrsetup(priv);

  /* Kick a pulse. */

  rtl8187x_eeprom_pulsehigh(priv);
  rtl8187x_eeprom_pulselow(priv);

  /* Select the read opcode and the word to be read. */

  command = (PCI_EEPROM_READ_OPCODE << priv->width) | word;
  rtl8187x_eeprom_wrbits(priv, command, PCI_EEPROM_WIDTH_OPCODE + priv->width);

  /* Read the requested 16 bits. */

  rtl8187x_eeprom_rdbits(priv, data, 16);

  /* Cleanup eeprom register. */

  rtl8187x_eeprom_cleanup(priv);
}

/****************************************************************************
 * Function: rtl8187x_eeprom_multiread
 *
 * Description:
 *   A convenience wrapper around  rtl8187x_eeprom_read to obtain multiple
 *   words from the EEPROM.
 *
 * Parameters:
 *   priv   - Reference to the NuttX driver state structure
 *   word   -
 *   data   - Location for data to be written
 *   nwords - The number of words to be read
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rtl8187x_eeprom_multiread(FAR struct rtl8187x_state_s *priv,
                                      uint8_t word, FAR uint16_t *data,
                                      uint16_t nwords)
{
  unsigned int i;
  uint16_t tmp;

  for (i = 0; i < nwords; i++)
    {
      tmp = 0;
      rtl8187x_eeprom_read(priv, word + i, &tmp);
      data[i] = rtl8187x_host2le16(tmp);
    }
}

/****************************************************************************
 * Function: PHY support functions
 *
 * Description:
 *   Configure PHY
 *
 * Parameters:
 *   priv - Private driver state information
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rtl8187x_wrphy(FAR struct rtl8187x_state_s *priv, uint8_t addr,
                           uint32_t data)
{
  data <<= 8;
  data |= addr | 0x80;

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_PHY3, (data >> 24) & 0xff);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_PHY2, (data >> 16) & 0xff);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_PHY1, (data >> 8) & 0xff);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_PHY0, data & 0xff);

  usleep(1000);
}

static inline void rtl8187x_wrphyofdm(FAR struct rtl8187x_state_s *priv,
                                      uint8_t addr, uint32_t data)
{
  rtl8187x_wrphy(priv, addr, data);
}

static inline void rtl8187x_wrphycck(FAR struct rtl8187x_state_s *priv,
                                     uint8_t addr, uint32_t data)
{
  rtl8187x_wrphy(priv, addr, data | 0x10000);
}

/****************************************************************************
 * Function: rtl8225_rfinit and rtl8225z2_rfinit
 *
 * Description:
 *   Chip-specific RF initialization
 *
 * Parameters:
 *   priv - Private driver state information
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rtl8225_rfinit(FAR struct rtl8187x_state_s *priv)
{
  unsigned int i;

  uvdbg("rfinit");
  rtl8187x_write(priv, 0x0, 0x067);
  usleep(1000);
  rtl8187x_write(priv, 0x1, 0xFE0);
  usleep(1000);
  rtl8187x_write(priv, 0x2, 0x44D);
  usleep(1000);
  rtl8187x_write(priv, 0x3, 0x441);
  usleep(1000);
  rtl8187x_write(priv, 0x4, 0x486);
  usleep(1000);
  rtl8187x_write(priv, 0x5, 0xBC0);
  usleep(1000);
  rtl8187x_write(priv, 0x6, 0xAE6);
  usleep(1000);
  rtl8187x_write(priv, 0x7, 0x82A);
  usleep(1000);
  rtl8187x_write(priv, 0x8, 0x01F);
  usleep(1000);
  rtl8187x_write(priv, 0x9, 0x334);
  usleep(1000);
  rtl8187x_write(priv, 0xA, 0xFD4);
  usleep(1000);
  rtl8187x_write(priv, 0xB, 0x391);
  usleep(1000);
  rtl8187x_write(priv, 0xC, 0x050);
  usleep(1000);
  rtl8187x_write(priv, 0xD, 0x6DB);
  usleep(1000);
  rtl8187x_write(priv, 0xE, 0x029);
  usleep(1000);
  rtl8187x_write(priv, 0xF, 0x914);
  usleep(100000);

  rtl8187x_write(priv, 0x2, 0xC4D);
  usleep(200000);
  rtl8187x_write(priv, 0x2, 0x44D);
  usleep(200000);

  if (!(rtl8187x_read(priv, 6) & (1 << 7)))
    {
      rtl8187x_write(priv, 0x02, 0x0c4d);
      usleep(200000);
      rtl8187x_write(priv, 0x02, 0x044d);
      usleep(100000);
      if (!(rtl8187x_read(priv, 6) & (1 << 7)))
        {
          udbg("RF Calibration Failed! %x\n", rtl8187x_read(priv, 6));
        }
    }

  rtl8187x_write(priv, 0x0, 0x127);

  for (i = 0; i < ARRAY_SIZE(g_rtl8225bcd_rxgain); i++)
    {
      rtl8187x_write(priv, 0x1, i + 1);
      rtl8187x_write(priv, 0x2, g_rtl8225bcd_rxgain[i]);
    }

  rtl8187x_write(priv, 0x0, 0x027);
  rtl8187x_write(priv, 0x0, 0x22F);

  for (i = 0; i < ARRAY_SIZE(g_rtl8225_agc); i++)
    {
      rtl8187x_wrphyofdm(priv, 0xB, g_rtl8225_agc[i]);
      usleep(1000);
      rtl8187x_wrphyofdm(priv, 0xA, 0x80 + i);
      usleep(1000);
    }

  usleep(1000);

  rtl8187x_wrphyofdm(priv, 0x00, 0x01);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x01, 0x02);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x02, 0x42);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x03, 0x00);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x04, 0x00);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x05, 0x00);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x06, 0x40);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x07, 0x00);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x08, 0x40);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x09, 0xfe);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x0a, 0x09);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x0b, 0x80);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x0c, 0x01);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x0e, 0xd3);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x0f, 0x38);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x10, 0x84);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x11, 0x06);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x12, 0x20);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x13, 0x20);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x14, 0x00);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x15, 0x40);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x16, 0x00);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x17, 0x40);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x18, 0xef);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x19, 0x19);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x1a, 0x20);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x1b, 0x76);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x1c, 0x04);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x1e, 0x95);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x1f, 0x75);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x20, 0x1f);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x21, 0x27);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x22, 0x16);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x24, 0x46);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x25, 0x20);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x26, 0x90);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x27, 0x88);
  usleep(1000);

  rtl8187x_wrphyofdm(priv, 0x0d, g_rtl8225_gain[2 * 4]);
  rtl8187x_wrphyofdm(priv, 0x1b, g_rtl8225_gain[2 * 4 + 2]);
  rtl8187x_wrphyofdm(priv, 0x1d, g_rtl8225_gain[2 * 4 + 3]);
  rtl8187x_wrphyofdm(priv, 0x23, g_rtl8225_gain[2 * 4 + 1]);

  rtl8187x_wrphycck(priv, 0x00, 0x98);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x03, 0x20);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x04, 0x7e);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x05, 0x12);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x06, 0xfc);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x07, 0x78);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x08, 0x2e);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x10, 0x9b);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x11, 0x88);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x12, 0x47);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x13, 0xd0);
  rtl8187x_wrphycck(priv, 0x19, 0x00);
  rtl8187x_wrphycck(priv, 0x1a, 0xa0);
  rtl8187x_wrphycck(priv, 0x1b, 0x08);
  rtl8187x_wrphycck(priv, 0x40, 0x86);
  rtl8187x_wrphycck(priv, 0x41, 0x8d);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x42, 0x15);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x43, 0x18);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x44, 0x1f);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x45, 0x1e);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x46, 0x1a);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x47, 0x15);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x48, 0x10);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x49, 0x0a);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x4a, 0x05);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x4b, 0x02);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x4c, 0x05);
  usleep(1000);

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_TESTR, 0x0D);

  rtl8225_settxpower(priv, 1);

  /* RX antenna default to A */

  rtl8187x_wrphycck(priv, 0x10, 0x9b);
  usleep(1000);                 /* B: 0xDB */
  rtl8187x_wrphyofdm(priv, 0x26, 0x90);
  usleep(1000);                 /* B: 0x10 */

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_TXANTENNA, 0x03);        /* B: 0x00 */
  usleep(1000);
  rtl8187x_iowrite32(priv, 0xff94, 0x3dc00002);

  /* Set sensitivity */

  rtl8187x_write(priv, 0x0c, 0x50);
  rtl8187x_wrphyofdm(priv, 0x0d, g_rtl8225_gain[2 * 4]);
  rtl8187x_wrphyofdm(priv, 0x1b, g_rtl8225_gain[2 * 4 + 2]);
  rtl8187x_wrphyofdm(priv, 0x1d, g_rtl8225_gain[2 * 4 + 3]);
  rtl8187x_wrphyofdm(priv, 0x23, g_rtl8225_gain[2 * 4 + 1]);
  rtl8187x_wrphycck(priv, 0x41, g_rtl8225_threshold[2]);
}

static void rtl8225z2_rfinit(FAR struct rtl8187x_state_s *priv)
{
  unsigned int i;

  rtl8187x_write(priv, 0x0, 0x2BF);
  usleep(1000);
  rtl8187x_write(priv, 0x1, 0xEE0);
  usleep(1000);
  rtl8187x_write(priv, 0x2, 0x44D);
  usleep(1000);
  rtl8187x_write(priv, 0x3, 0x441);
  usleep(1000);
  rtl8187x_write(priv, 0x4, 0x8C3);
  usleep(1000);
  rtl8187x_write(priv, 0x5, 0xC72);
  usleep(1000);
  rtl8187x_write(priv, 0x6, 0x0E6);
  usleep(1000);
  rtl8187x_write(priv, 0x7, 0x82A);
  usleep(1000);
  rtl8187x_write(priv, 0x8, 0x03F);
  usleep(1000);
  rtl8187x_write(priv, 0x9, 0x335);
  usleep(1000);
  rtl8187x_write(priv, 0xa, 0x9D4);
  usleep(1000);
  rtl8187x_write(priv, 0xb, 0x7BB);
  usleep(1000);
  rtl8187x_write(priv, 0xc, 0x850);
  usleep(1000);
  rtl8187x_write(priv, 0xd, 0xCDF);
  usleep(1000);
  rtl8187x_write(priv, 0xe, 0x02B);
  usleep(1000);
  rtl8187x_write(priv, 0xf, 0x114);
  usleep(100000);

  rtl8187x_write(priv, 0x0, 0x1B7);

  for (i = 0; i < ARRAY_SIZE(g_rtl8225z2_rxgain); i++)
    {
      rtl8187x_write(priv, 0x1, i + 1);
      rtl8187x_write(priv, 0x2, g_rtl8225z2_rxgain[i]);
    }

  rtl8187x_write(priv, 0x3, 0x080);
  rtl8187x_write(priv, 0x5, 0x004);
  rtl8187x_write(priv, 0x0, 0x0B7);
  rtl8187x_write(priv, 0x2, 0xc4D);

  usleep(200000);
  rtl8187x_write(priv, 0x2, 0x44D);
  usleep(100000);

  if (!(rtl8187x_read(priv, 6) & (1 << 7)))
    {
      rtl8187x_write(priv, 0x02, 0x0C4D);
      usleep(200000);
      rtl8187x_write(priv, 0x02, 0x044D);
      usleep(100000);
      if (!(rtl8187x_read(priv, 6) & (1 << 7)))
        {
          udbg("RF Calibration Failed! %x\n", rtl8187x_read(priv, 6));
        }
    }

  usleep(200000);

  rtl8187x_write(priv, 0x0, 0x2BF);

  for (i = 0; i < ARRAY_SIZE(g_rtl8225_agc); i++)
    {
      rtl8187x_wrphyofdm(priv, 0xB, g_rtl8225_agc[i]);
      usleep(1000);
      rtl8187x_wrphyofdm(priv, 0xA, 0x80 + i);
      usleep(1000);
    }

  usleep(1000);

  rtl8187x_wrphyofdm(priv, 0x00, 0x01);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x01, 0x02);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x02, 0x42);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x03, 0x00);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x04, 0x00);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x05, 0x00);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x06, 0x40);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x07, 0x00);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x08, 0x40);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x09, 0xfe);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x0a, 0x08);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x0b, 0x80);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x0c, 0x01);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x0d, 0x43);
  rtl8187x_wrphyofdm(priv, 0x0e, 0xd3);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x0f, 0x38);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x10, 0x84);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x11, 0x07);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x12, 0x20);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x13, 0x20);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x14, 0x00);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x15, 0x40);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x16, 0x00);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x17, 0x40);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x18, 0xef);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x19, 0x19);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x1a, 0x20);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x1b, 0x15);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x1c, 0x04);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x1d, 0xc5);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x1e, 0x95);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x1f, 0x75);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x20, 0x1f);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x21, 0x17);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x22, 0x16);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x23, 0x80);
  usleep(1000);                 // FIXME: not needed?
  rtl8187x_wrphyofdm(priv, 0x24, 0x46);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x25, 0x00);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x26, 0x90);
  usleep(1000);
  rtl8187x_wrphyofdm(priv, 0x27, 0x88);
  usleep(1000);

  rtl8187x_wrphyofdm(priv, 0x0b, g_rtl8225z2_gainbg[4 * 3]);
  rtl8187x_wrphyofdm(priv, 0x1b, g_rtl8225z2_gainbg[4 * 3 + 1]);
  rtl8187x_wrphyofdm(priv, 0x1d, g_rtl8225z2_gainbg[4 * 3 + 2]);
  rtl8187x_wrphyofdm(priv, 0x21, 0x37);

  rtl8187x_wrphycck(priv, 0x00, 0x98);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x03, 0x20);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x04, 0x7e);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x05, 0x12);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x06, 0xfc);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x07, 0x78);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x08, 0x2e);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x10, 0x9b);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x11, 0x88);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x12, 0x47);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x13, 0xd0);
  rtl8187x_wrphycck(priv, 0x19, 0x00);
  rtl8187x_wrphycck(priv, 0x1a, 0xa0);
  rtl8187x_wrphycck(priv, 0x1b, 0x08);
  rtl8187x_wrphycck(priv, 0x40, 0x86);
  rtl8187x_wrphycck(priv, 0x41, 0x8d);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x42, 0x15);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x43, 0x18);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x44, 0x36);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x45, 0x35);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x46, 0x2e);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x47, 0x25);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x48, 0x1c);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x49, 0x12);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x4a, 0x09);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x4b, 0x04);
  usleep(1000);
  rtl8187x_wrphycck(priv, 0x4c, 0x05);
  usleep(1000);

  rtl8187x_iowrite8(priv, 0xff5B, 0x0D);
  usleep(1000);

  rtl8225z2_settxpower(priv, 1);

  /* RX antenna default to A */

  rtl8187x_wrphycck(priv, 0x10, 0x9b);
  usleep(1000);                 /* B: 0xDB */
  rtl8187x_wrphyofdm(priv, 0x26, 0x90);
  usleep(1000);                 /* B: 0x10 */

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_TXANTENNA, 0x03);        /* B: 0x00 */
  usleep(1000);
  rtl8187x_iowrite32(priv, 0xff94, 0x3dc00002);
}

/****************************************************************************
 * Function: rtl8187x_anaparam2on and rtl8187x_anaparamon
 *
 * Description:
 *   Chip-specific TX power configuration
 *
 * Parameters:
 *   priv - Private driver state information
 *   channel - The selected channel
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rtl8187x_anaparam2on(FAR struct rtl8187x_state_s *priv)
{
  uint8_t regval;

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_CONFIG);
  regval = rtl8187x_ioread8(priv, RTL8187X_ADDR_CONFIG3);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CONFIG3, regval | RTL8187X_CONFIG3_ANAPARAMWRITE);
#ifdef CONFIG_RTL8187B
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_ANAPARAM2, RTL8187B_RTL8225_ANAPARAM2_ON);
#else
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_ANAPARAM2, RTL8187X_RTL8225_ANAPARAM2_ON);
#endif
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CONFIG3, regval & ~RTL8187X_CONFIG3_ANAPARAMWRITE);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_NORMAL);
}

static void rtl8187x_anaparamon(FAR struct rtl8187x_state_s *priv)
{
  uint8_t regval;

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_CONFIG);
  regval = rtl8187x_ioread8(priv, RTL8187X_ADDR_CONFIG3);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CONFIG3, regval | RTL8187X_CONFIG3_ANAPARAMWRITE);

#ifdef CONFIG_RTL8187B
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_ANAPARAM, RTL8187B_RTL8225_ANAPARAM_ON);
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_ANAPARAM2, RTL8187B_RTL8225_ANAPARAM2_ON);
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_ANAPARAM3, RTL8187B_RTL8225_ANAPARAM3_ON);
#else
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_ANAPARAM, RTL8187X_RTL8225_ANAPARAM_ON);
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_ANAPARAM2, RTL8187X_RTL8225_ANAPARAM2_ON);
#endif

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CONFIG3, regval & ~RTL8187X_CONFIG3_ANAPARAMWRITE);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_NORMAL);
}

static void rtl8187x_anaparamoff(FAR struct rtl8187x_state_s *priv)
{
  uint8_t regval;

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_CONFIG);
  regval = rtl8187x_ioread8(priv, RTL8187X_ADDR_CONFIG3);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CONFIG3, regval | RTL8187X_CONFIG3_ANAPARAMWRITE);

#ifdef CONFIG_RTL8187B
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_ANAPARAM, RTL8187B_RTL8225_ANAPARAM_OFF);
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_ANAPARAM2, RTL8187B_RTL8225_ANAPARAM2_OFF);
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_ANAPARAM3, RTL8187B_RTL8225_ANAPARAM3_OFF);
#else
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_ANAPARAM, RTL8187X_RTL8225_ANAPARAM_OFF);
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_ANAPARAM2, RTL8187X_RTL8225_ANAPARAM2_OFF);
#endif

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CONFIG3, regval & ~RTL8187X_CONFIG3_ANAPARAMWRITE);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_NORMAL);
}

/****************************************************************************
 * Function: rtl8225_settxpower and 
 *
 * Description:
 *   Chip-specific TX power configuration
 *
 * Parameters:
 *   priv - Private driver state information
 *   channel - The selected channel
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rtl8225_settxpower(FAR struct rtl8187x_state_s *priv, int channel)
{
  uint8_t cck_power, ofdm_power;
  const uint8_t *tmp;
  int i;

  cck_power = priv->channels[channel - 1].val & 0xf;
  ofdm_power = priv->channels[channel - 1].val >> 4;

  cck_power = MIN(cck_power, (uint8_t) 11);
  ofdm_power = MIN(ofdm_power, (uint8_t) 35);

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_TXGAINCCK,
                   g_rtl8225_txgaincckofdm[cck_power / 6] >> 1);

  if (channel == 14)
    {
      tmp = &g_rtl8225_txpowercckch14[(cck_power % 6) * 8];
    }
  else
    {
      tmp = &g_rtl8225_txpowercck[(cck_power % 6) * 8];
    }

  for (i = 0; i < 8; i++)
    {
        rtl8187x_wrphycck(priv, 0x44 + i, *tmp++);
    }
  usleep(1000);                 // FIXME: optional?

  /* anaparam2 on */

  rtl8187x_anaparam2on(priv);

  rtl8187x_wrphyofdm(priv, 2, 0x42);
  rtl8187x_wrphyofdm(priv, 6, 0x00);
  rtl8187x_wrphyofdm(priv, 8, 0x00);

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_TXGAINOFDM,
                   g_rtl8225_txgaincckofdm[ofdm_power / 6] >> 1);

  tmp = &g_rtl8225_txpowerofdm[ofdm_power % 6];

  rtl8187x_wrphyofdm(priv, 5, *tmp);
  rtl8187x_wrphyofdm(priv, 7, *tmp);

  usleep(1000);
}

static void rtl8225z2_settxpower(FAR struct rtl8187x_state_s *priv, int channel)
{
  uint8_t cck_power;
  uint8_t ofdm_power;
  const uint8_t *tmp;
  int i;

  cck_power  = priv->channels[channel - 1].val & 0xF;
  ofdm_power = priv->channels[channel - 1].val >> 4;

  cck_power  = MIN(cck_power, (uint8_t) 15);
  cck_power += priv->rxpwrbase & 0xF;
  cck_power  = MIN(cck_power, (uint8_t) 35);

  ofdm_power  = MIN(ofdm_power, (uint8_t) 15);
  ofdm_power += priv->rxpwrbase >> 4;
  ofdm_power  = MIN(ofdm_power, (uint8_t) 35);

  if (channel == 14)
    {
      tmp =  g_rtl8225z2_txpowercckch14;
    }
  else
    {
      tmp = g_rtl8225z2_txpowercck;
    }

  for (i = 0; i < 8; i++)
    {
      rtl8187x_wrphycck(priv, 0x44 + i, *tmp++);
    }

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_TXGAINCCK,
                   g_rtl8225z2_txgaincckofdm[cck_power]);
  usleep(1000);

  /* anaparam2 on */

  rtl8187x_anaparam2on(priv);

  rtl8187x_wrphyofdm(priv, 2, 0x42);
  rtl8187x_wrphyofdm(priv, 5, 0x00);
  rtl8187x_wrphyofdm(priv, 6, 0x40);
  rtl8187x_wrphyofdm(priv, 7, 0x00);
  rtl8187x_wrphyofdm(priv, 8, 0x40);

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_TXGAINOFDM,
                    g_rtl8225z2_txgaincckofdm[ofdm_power]);
  usleep(1000);
}

/****************************************************************************
 * Function: rtl8187x_reset
 *
 * Description:
 *   Reset and initialize hardware
 *
 * Parameters:
 *   priv - Private driver state information
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int rtl8187x_reset(struct rtl8187x_state_s *priv)
{
  uint8_t regval;
  int i;

#ifdef CONFIG_RTL8187B
  int ret;

  /* Turn on ANAPARAM */

  rtl8187x_anaparamon(priv)

  /* Reset PLL sequence on 8187B. Realtek note: reduces power
   * consumption about 30 mA
   */

  rtl8187x_iowrite8(priv, (uint8_t*)0xff61, 0x10);
  regval = rtl818x_ioread8(priv, (uint8_t*)0xff62);
  rtl8187x_iowrite8(priv, (uint8_t*)0xff62, regval & ~(1 << 5));
  rtl8187x_iowrite8(priv, (uint8_t*)0xff62, regval | (1 << 5));

  ret = rtl8187_cmd_reset(dev);
  if (ret != 0)
    {
      return ret;
    }
  rtl8187x_anaparamon(priv)

  /* BRSR (Basic Rate Set Register) on 8187B looks to be the same as
   * RESP_RATE on 8187L in Realtek sources: each bit should be each
   * one of the 12 rates, all are enabled
   */

  rtl8187x_iowrite16(priv, (uint16_t*)0xff34, 0x0fff);

  regval = rtl818x_ioread8(priv, RTL8187X_ADDR_CWCONF);
  regval |= RTL818X_CW_CONF_PERPACKET_RETRY_SHIFT;
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CWCONF, regval);

  /* Auto Rate Fallback Register (ARFR): 1M-54M setting */

  rtl8187x_iowrite16_idx(priv, (uint16_t*)0xffe0, 0x0fff, 1);
  rtl8187x_iowrite8_idx(priv, (uint8_t*)0xffe2, 0x00, 1);
  rtl8187x_iowrite16_idx(priv, (uint16_t*)0xffd4, 0xffff, 1);

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL818X_EEPROMCMD_CONFIG);
  regval = rtl818x_ioread8(priv, RTL8187X_ADDR_CONFIG1);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CONFIG1, (regval & 0x3f) | 0x80);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL818X_EEPROMCMD_NORMAL);

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_WPACONF, 0);
  for (i = 0; i < ARRAY_SIZE(rtl8187b_reg_table); i++) {
          rtl8187x_iowrite8_idx(priv,
                               (uint8_t*)(uintptr_t)
                               (rtl8187b_reg_table[i][0] | 0xff00),
                               rtl8187b_reg_table[i][1],
                               rtl8187b_reg_table[i][2]);
  }

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_TIDACMAP, 0xfa50);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_INTMIG, 0);

  rtl8187x_iowrite32_idx(priv, (uint32_t*)0xfff0, 0, 1);
  rtl8187x_iowrite32_idx(priv, (uint32_t*)0xfff4, 0, 1);
  rtl8187x_iowrite8_idx(priv, (uint8_t*)0xfff8, 0, 1);

  rtl8187x_iowrite32(priv, RTL8187X_ADDR_RFTIMING, 0x00004001);

  /* RFSW_CTRL register */

  rtl8187x_iowrite16_idx(priv, (uint16_t*)0xff72, 0x569a, 2);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, 0x0480);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSSELECT, 0x2488);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSENABLE, 0x1fff);
  msleep(100);

  priv->rf->init(dev);

  regval = RTL818X_CMD_TX_ENABLE | RTL818X_CMD_RX_ENABLE;
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CMD, regval);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_INTMASK, 0xffff);

  rtl8187x_iowrite8(priv, (uint8_t*)0xfe41, 0xf4);
  rtl8187x_iowrite8(priv, (uint8_t*)0xfe40, 0x00);
  rtl8187x_iowrite8(priv, (uint8_t*)0xfe42, 0x00);
  rtl8187x_iowrite8(priv, (uint8_t*)0xfe42, 0x01);
  rtl8187x_iowrite8(priv, (uint8_t*)0xfe40, 0x0f);
  rtl8187x_iowrite8(priv, (uint8_t*)0xfe42, 0x00);
  rtl8187x_iowrite8(priv, (uint8_t*)0xfe42, 0x01);

  regval = rtl818x_ioread8(priv, (uint8_t*)0xffdb);
  rtl8187x_iowrite8(priv, (uint8_t*)0xffdb, regval | (1 << 2));
  rtl8187x_iowrite16_idx(priv, (uint16_t*)0xff72, 0x59fa, 3);
  rtl8187x_iowrite16_idx(priv, (uint16_t*)0xff74, 0x59d2, 3);
  rtl8187x_iowrite16_idx(priv, (uint16_t*)0xff76, 0x59d2, 3);
  rtl8187x_iowrite16_idx(priv, (uint16_t*)0xff78, 0x19fa, 3);
  rtl8187x_iowrite16_idx(priv, (uint16_t*)0xff7a, 0x19fa, 3);
  rtl8187x_iowrite16_idx(priv, (uint16_t*)0xff7c, 0x00d0, 3);
  rtl8187x_iowrite8(priv, (uint8_t*)0xff61, 0);
  rtl8187x_iowrite8_idx(priv, (uint8_t*)0xff80, 0x0f, 1);
  rtl8187x_iowrite8_idx(priv, (uint8_t*)0xff83, 0x03, 1);
  rtl8187x_iowrite8(priv, (uint8_t*)0xffda, 0x10);
  rtl8187x_iowrite8_idx(priv, (uint8_t*)0xff4d, 0x08, 2);

  rtl8187x_iowrite32(priv, rtl8187x_addr_hssipara, 0x0600321b);
  rtl8187x_iowrite16_idx(priv, (uint16_t*)0xffec, 0x0800, 1);

  priv->slot_time = 0x9;
  priv->aifsn[0] = 2; /* AIFSN[AC_VO] */
  priv->aifsn[1] = 2; /* AIFSN[AC_VI] */
  priv->aifsn[2] = 7; /* AIFSN[AC_BK] */
  priv->aifsn[3] = 3; /* AIFSN[AC_BE] */
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_ACMCONTROL, 0);

  /* ENEDCA flag must always be set, transmit issues? */

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_MSR, RTL818X_MSR_ENEDCA);
#else

  /* reset */

  rtl8187x_anaparamon(priv);

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_INTMASK, 0);

  usleep(200000);
  rtl8187x_iowrite8(priv, 0xfe18, 0x10);
  rtl8187x_iowrite8(priv, 0xfe18, 0x11);
  rtl8187x_iowrite8(priv, 0xfe18, 0x00);
  usleep(200000);

  regval = rtl8187x_ioread8(priv, RTL8187X_ADDR_CMD);
  regval &= (1 << 1);
  regval |= RTL8187X_CMD_RESET;
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CMD, regval);

  i = 10;
  do
    {
      usleep(2000);
      if (!(rtl8187x_ioread8(priv, RTL8187X_ADDR_CMD) & RTL8187X_CMD_RESET))
        break;
    }
  while (--i);

  if (!i)
    {
      udbg("Reset timeout!\n");
      return -ETIMEDOUT;
    }

  /* Reload registers from eeprom */

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_LOAD);

  i = 10;
  do
    {
      usleep(4000);
      if (!(rtl8187x_ioread8(priv, RTL8187X_ADDR_EEPROMCMD) &
            RTL8187X_EEPROMCMD_CONFIG))
        break;
    }
  while (--i);

  if (!i)
    {
      udbg("%s: eeprom reset timeout!\n");
      return -ETIMEDOUT;
    }

  rtl8187x_anaparamon(priv);

  /* Setup card */

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSSELECT, 0);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_GPIO, 0);

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSSELECT, (4 << 8));
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_GPIO, 1);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_GPENABLE, 0);

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_CONFIG);

  rtl8187x_iowrite16(priv, 0xffF4, 0xffFF);
  regval = rtl8187x_ioread8(priv, RTL8187X_ADDR_CONFIG1);
  regval &= 0x3F;
  regval |= 0x80;
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CONFIG1, regval);

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_NORMAL);

  rtl8187x_iowrite32(priv, RTL8187X_ADDR_INTTIMEOUT, 0);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_WPACONF, 0);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_RATEFALLBACK, 0x81);

  // TODO: set RESP_RATE and BRSR properly
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_RESPRATE, (8 << 4) | 0);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_BRSR, 0x01F3);

  /* host_usb_init */

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSSELECT, 0);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_GPIO, 0);
  regval = rtl8187x_ioread8(priv, 0xfe53);
  rtl8187x_iowrite8(priv, 0xfe53, regval | (1 << 7));
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSSELECT, (4 << 8));
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_GPIO, 0x20);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_GPENABLE, 0);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSOUTPUT, 0x80);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSSELECT, 0x80);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSENABLE, 0x80);

  usleep(100000);

  rtl8187x_iowrite32(priv, RTL8187X_ADDR_RFTIMING, 0x000a8008);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_BRSR, 0xffFF);
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_RFPARA, 0x00100044);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_CONFIG);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CONFIG3, 0x44);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_NORMAL);
  rtl8187x_iowrite16(priv, RTL8187X_ADDR_RFPINSENABLE, 0x1FF7);
  usleep(100000);

  priv->rfinit(priv);

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_BRSR, 0x01F3);
  regval = rtl8187x_ioread8(priv, RTL8187X_ADDR_PGSELECT) & ~1;
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_PGSELECT, regval | 1);
  rtl8187x_iowrite16(priv, 0xffFE, 0x10);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_TALLYSEL, 0x80);
  rtl8187x_iowrite8(priv, 0xffFF, 0x60);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_PGSELECT, regval);
#endif

  return OK;
}

/****************************************************************************
 * Function: rtl8187x_setchannel
 *
 * Description:
 *   Select the specified channel
 *
 * Parameters:
 *   priv - Private driver state information
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rtl8187x_setchannel(FAR struct rtl8187x_state_s *priv, int channel)
{
  uint32_t regval;

  regval = rtl8187x_ioread32(priv, RTL8187X_ADDR_TXCONF);

  /* Enable TX loopback on MAC level to avoid TX during channel changes, as
   * this has be seen to causes problems and the card will stop work until next 
   * reset
   */

  rtl8187x_iowrite32(priv, RTL8187X_ADDR_TXCONF,
                     regval | RTL8187X_TXCONF_LOOPBACKMAC);
  usleep(10000);

  priv->settxpower(priv, channel);

  rtl8187x_write(priv, 0x7, g_chanselect[channel - 1]);
  usleep(20000);

  rtl8187x_iowrite32(priv, RTL8187X_ADDR_TXCONF, regval);
}

/****************************************************************************
 * Function: rtl8187_start
 *
 * Description:
 *   Bring up the RTL8187
 *
 * Parameters:
 *   priv - Private driver state information
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int rtl8187x_start(FAR struct rtl8187x_state_s *priv)
{
  uint32_t regval;
  int ret;

  /* Reset and initialize the hardware */

  ret = rtl8187x_reset(priv);
  if (ret != OK)
    {
      return ret;
    }

#ifdef CONFIG_RTL8187B

  regval = RTL818X_RXCONF_MGMT | RTL818X_RXCONF_DATA | RTL818X_RXCONF_BROADCAST |
           RTL818X_RXCONF_NICMAC | RTL818X_RX_ONF_BSSID |
           (7 << 13 /* RX FIFO threshold NONE */) |
           (7 << 10 /* MAX RX DMA */) |
           RTL818X_RXCONF_RX_AUTORESETPHY | RTL818X_RXCONF_ONLYERLPKT | RTL818X_RXCONF_MULTICAST;
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_RXCONF, regval);

  regval  = rtl8187x_ioread8(priv, RTL8187X_ADDR_TXAGCCTL);
  regval &= ~RTL8187X_TXAGCCTL_PERPACKETGAINSHIFT;
  regval &= ~RTL8187X_TXAGCCTL_PERPACKETANTSELSHIFT;
  regval &= ~RTL8187X_TXAGCCTL_FEEDBACKANT;
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_TXAGCCTL, regval);

  regval = RTL818X_TXCONF_HWSEQNUM | RTL818X_TXCONF_DISREQQSIZE |
           (7 << 8  /* short retry limit */) |
           (7 << 0  /* long retry limit */) |
           (7 << 21 /* MAX TX DMA */);
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_TXCONF, regval);

#else

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_INTMASK, 0xffff);

  rtl8187x_iowrite32(priv, RTL8187X_ADDR_MAR0, ~0);
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_MAR1, ~0);

  regval = RTL8187X_RXCONF_ONLYERLPKT | RTL8187X_RXCONF_RXAUTORESETPHY | RTL8187X_RXCONF_BSSID |
        RTL8187X_RXCONF_MGMT | RTL8187X_RXCONF_DATA | RTL8187X_RXCONF_CTRL |
        (7 << 13 /* RX fifo threshold none */ ) |
        (7 << 10 /* MAX RX DMA */ ) |
        RTL8187X_RXCONF_BROADCAST | RTL8187X_RXCONF_NICMAC | RTL8187X_RXCONF_MONITOR;
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_RXCONF, regval);

  regval  = rtl8187x_ioread8(priv, RTL8187X_ADDR_CWCONF);
  regval &= ~RTL8187X_CWCONF_PERPACKETCWSHIFT;
  regval |= RTL8187X_CWCONF_PERPACKETRETRYSHIFT;
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CWCONF, regval);

  regval  = rtl8187x_ioread8(priv, RTL8187X_ADDR_TXAGCCTL);
  regval &= ~RTL8187X_TXAGCCTL_PERPACKETGAINSHIFT;
  regval &= ~RTL8187X_TXAGCCTL_PERPACKETANTSELSHIFT;
  regval &= ~RTL8187X_TXAGCCTL_FEEDBACKANT;
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_TXAGCCTL, regval);

  regval = RTL8187X_TXCONF_CWMIN | (7 << 21 /* MAX TX DMA */ ) | RTL8187X_TXCONF_NOICV;
  rtl8187x_iowrite32(priv, RTL8187X_ADDR_TXCONF, regval);

  regval  = rtl8187x_ioread8(priv, RTL8187X_ADDR_CMD);
  regval |= RTL8187X_CMD_TXENABLE;
  regval |= RTL8187X_CMD_RXENABLE;
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CMD, regval);
#endif
  return OK;
}

/****************************************************************************
 * Function: rtl8187x_stop
 *
 * Description:
 *   Bring down the RTL8187
 *
 * Parameters:
 *   priv - Private driver state information
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void rtl8187x_stop(FAR struct rtl8187x_state_s *priv)
{
  uint32_t regval;

  rtl8187x_iowrite16(priv, RTL8187X_ADDR_INTMASK, 0);

  regval  = rtl8187x_ioread8(priv, RTL8187X_ADDR_CMD);
  regval &= ~RTL8187X_CMD_TXENABLE;
  regval &= ~RTL8187X_CMD_RXENABLE;
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CMD, regval);

  rtl8187x_write(priv, 0x4, 0x1f);
  usleep(1000);

  /* RF stop */

  rtl8187x_anaparamoff(priv);

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_CONFIG);
  regval = rtl8187x_ioread8(priv, RTL8187X_ADDR_CONFIG4);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_CONFIG4, regval | RTL8187X_CONFIG4_VCOOFF);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_NORMAL);
}

/****************************************************************************
 * Function: rtl8187_setup
 *
 * Description:
 *   Configure the RTL8187
 *
 * Parameters:
 *   priv - Private driver state information
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int rtl8187x_setup(FAR struct rtl8187x_state_s *priv)
{
  struct ieee80211_channel_s *channel;
  uint16_t permaddr[3];
  uint16_t txpwr, regval;
  int i;

  /* Copy the default channel information */

  memcpy(priv->channels, g_channels, RTL8187X_NCHANNELS*sizeof(struct ieee80211_channel_s));

  /* Get the EEPROM width */

  if (rtl8187x_ioread32(priv, RTL8187X_ADDR_RXCONF) & (1 << 6))
    {
      priv->width = PCI_EEPROM_WIDTH_93C66;
    }
  else
    {
      priv->width = PCI_EEPROM_WIDTH_93C46;
    }

  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_CONFIG);
  usleep(10);

  rtl8187x_eeprom_multiread(priv, RTL8187X_EEPROM_MACADDR, permaddr, 3);
  udbg("MAC address: %04x.%04x.%04x", permaddr[0], permaddr[1], permaddr[2]);

  channel = priv->channels;
  for (i = 0; i < 3; i++)
    {
      rtl8187x_eeprom_read(priv, RTL8187X_EEPROM_TXPWRCHAN1 + i, &txpwr);
      (*channel++).val = txpwr & 0xff;
      (*channel++).val = txpwr >> 8;
    }

  for (i = 0; i < 2; i++)
    {
      rtl8187x_eeprom_read(priv, RTL8187X_EEPROM_TXPWRCHAN4 + i, &txpwr);
      (*channel++).val = txpwr & 0xff;
      (*channel++).val = txpwr >> 8;
    }

#ifdef CONFIG_RTL8187B

  rtl8187x_eeprom_read(&priv, RTL8187_EEPROM_TXPWR_CHAN_6, &txpwr);
  (*channel++).val = txpwr & 0xff;

  rtl8187x_eeprom_read(&priv, 0x0a, &txpwr);
  (*channel++).val = txpwr & 0xff;

  rtl8187x_eeprom_read(&priv, 0x1c, &txpwr);
  (*channel++).val = txpwr & 0xff;
  (*channel++).val= txpwr

#else

  for (i = 0; i < 2; i++)
    {
      rtl8187x_eeprom_read(priv, RTL8187X_EEPROM_TXPWRCHAN6 + i, &txpwr);
      (*channel++).val = txpwr & 0xff;
      (*channel++).val = txpwr >> 8;
    }

#endif

  rtl8187x_eeprom_read(priv, RTL8187X_EEPROM_TXPWRBASE, &priv->rxpwrbase);

  regval = rtl8187x_ioread8(priv, RTL8187X_ADDR_PGSELECT) & ~1;
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_PGSELECT, regval | 1);

  /* 0 means asic B-cut, we should use SW 3 wire bit-by-bit banging for radio.
   * 1 means we can use USB specific request to write radio registers
   */

  priv->asicrev = rtl8187x_ioread8(priv, 0xffFE) & 0x3;
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_PGSELECT, regval);
  rtl8187x_iowrite8(priv, RTL8187X_ADDR_EEPROMCMD, RTL8187X_EEPROMCMD_NORMAL);

  rtl8187x_write(priv, 0, 0x1b7);

  if (rtl8187x_read(priv, 8) != 0x588 || rtl8187x_read(priv, 9) != 0x700)
    {
      priv->rfinit     = rtl8225_rfinit;
      priv->settxpower = rtl8225_settxpower;
    }
  else
    {
      priv->rfinit     = rtl8225z2_rfinit;
      priv->settxpower = rtl8225z2_settxpower;

    }

  rtl8187x_write(priv, 0, 0x0b7);
  
  /* Save the MAC address in the device structure */

  priv->ethdev.d_mac.ether_addr_octet[0] = permaddr[0] & 0xff;
  priv->ethdev.d_mac.ether_addr_octet[1] = permaddr[0] >> 8;
  priv->ethdev.d_mac.ether_addr_octet[2] = permaddr[1] & 0xff;
  priv->ethdev.d_mac.ether_addr_octet[3] = permaddr[1] >> 8;
  priv->ethdev.d_mac.ether_addr_octet[4] = permaddr[2] & 0xff;
  priv->ethdev.d_mac.ether_addr_octet[5] = permaddr[2] >> 8;

  /* Provide information about the RTL device */

  udbg("hwaddr %02x.%02x.%02x.%02x.%02x.%02x, rtl8187 V%d + %s\n",
       priv->ethdev.d_mac.ether_addr_octet[0], priv->ethdev.d_mac.ether_addr_octet[1],
       priv->ethdev.d_mac.ether_addr_octet[2], priv->ethdev.d_mac.ether_addr_octet[3],
       priv->ethdev.d_mac.ether_addr_octet[4], priv->ethdev.d_mac.ether_addr_octet[5],
       priv->asicrev,
       priv->rfinit == rtl8225_rfinit ? "rtl8225" : "rtl8225z2");

  return 0;
}

/****************************************************************************
 * Function: rtl8187x_netinitialize
 *
 * Description:
 *   Initialize the WLAN controller and driver
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

static int rtl8187x_netinitialize(FAR struct rtl8187x_state_s *priv)
{
  int ret;

  /* Initialize the RTL8187x */

  ret = rtl8187x_setup(priv);
  if (ret == OK)
    {
      /* Put the interface in the down state. */

      rtl8187x_ifdown(&priv->ethdev);

      /* Register the device with the OS so that socket IOCTLs can be performed */

      (void)netdev_register(&priv->ethdev);
    }
  return OK;
}

/****************************************************************************
 * Function: rtl8187x_netuninitialize
 *
 * Description:
 *   Un-initialize the RTL8187x.  This only happens when the RTL8187x device
 *   is removed from the USB slot.
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

static int rtl8187x_netuninitialize(FAR struct rtl8187x_state_s *priv)
{
  irqstate_t flags;

  /* Cancel the TX and RX poll timers */

  flags = irqsave();
  wd_cancel(priv->wdtxpoll);
  wd_cancel(priv->wdrxpoll);

  /* Mark the device "down" */

  priv->bifup = false;
  irqrestore(flags);

  /* Unregister the device */

  (void)netdev_unregister(&priv->ethdev);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_wlaninit
 *
 * Description:
 *   Initialize the USB class driver.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host class device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_wlaninit(void)
{
  /* Advertise our availability to support RTL8187x devices */

  uvdbg("Register RTL8187x driver\n");
  return usbhost_registerclass(&g_wlan);
}

#endif /* CONFIG_USBHOST && CONFIG_NET && CONFIG_NET_WLAN */


