/*******************************************************************************
 * arch/arm/src/stm32/stm32_usbdev.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "stm32_otgfs.h"

#if defined(CONFIG_USBDEV) && defined(CONFIG_STM32_OTGFS)

/*******************************************************************************
 * Definitions
 *******************************************************************************/
/* Configuration ***************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef CONFIG_USBDEV_SETUP_MAXDATASIZE
#  define CONFIG_USBDEV_SETUP_MAXDATASIZE CONFIG_USBDEV_EP0_MAXSIZE
#endif

#ifndef CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100  /* mA */
#endif

/* There is 1.25Kb of FIFO memory.  The default partitions this memory
 * so that there is a TxFIFO allocated for each endpoint and with more
 * memory provided for the common RxFIFO.  A more knowledge-able
 * configuration would not allocate any TxFIFO space to OUT endpoints.
 */

#ifndef CONFIG_USBDEV_RXFIFO_SIZE
#  define CONFIG_USBDEV_RXFIFO_SIZE 512
#endif

#ifndef CONFIG_USBDEV_EP0_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP0_TXFIFO_SIZE 192
#endif

#ifndef CONFIG_USBDEV_EP1_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP1_TXFIFO_SIZE 192
#endif

#ifndef CONFIG_USBDEV_EP2_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP2_TXFIFO_SIZE 192
#endif

#ifndef CONFIG_USBDEV_EP3_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP3_TXFIFO_SIZE 192
#endif

#if (CONFIG_USBDEV_RXFIFO_SIZE + CONFIG_USBDEV_EP0_TXFIFO_SIZE + \
     CONFIG_USBDEV_EP2_TXFIFO_SIZE + CONFIG_USBDEV_EP3_TXFIFO_SIZE) > 1280
#  error "FIFO allocations exceed FIFO memory size"
#endif

/* The actual FIFO addresses that we use must be aligned to 4-byte boundaries;
 * FIFO sizes must be provided in units of 32-bit words.
 */

#define STM32_RXFIFO_BYTES     ((CONFIG_USBDEV_RXFIFO_SIZE + 3) & ~3)
#define STM32_RXFIFO_WORDS     ((CONFIG_USBDEV_RXFIFO_SIZE + 3) >> 2)

#define STM32_EP0_TXFIFO_BYTES ((CONFIG_USBDEV_EP0_TXFIFO_SIZE + 3) & ~3)
#define STM32_EP0_TXFIFO_WORDS ((CONFIG_USBDEV_EP0_TXFIFO_SIZE + 3) >> 2)

#if STM32_EP0_TXFIFO_WORDS < 16 || STM32_EP0_TXFIFO_WORDS > 256
#  error "CONFIG_USBDEV_EP0_TXFIFO_SIZE is out of range"
#endif

#define STM32_EP1_TXFIFO_BYTES ((CONFIG_USBDEV_EP1_TXFIFO_SIZE + 3) & ~3)
#define STM32_EP1_TXFIFO_WORDS ((CONFIG_USBDEV_EP1_TXFIFO_SIZE + 3) >> 2)

#if STM32_EP1_TXFIFO_WORDS < 16
#  error "CONFIG_USBDEV_EP1_TXFIFO_SIZE is out of range"
#endif

#define STM32_EP2_TXFIFO_BYTES ((CONFIG_USBDEV_EP2_TXFIFO_SIZE + 3) & ~3)
#define STM32_EP2_TXFIFO_WORDS ((CONFIG_USBDEV_EP2_TXFIFO_SIZE + 3) >> 2)

#if STM32_EP2_TXFIFO_WORDS < 16
#  error "CONFIG_USBDEV_EP2_TXFIFO_SIZE is out of range"
#endif

#define STM32_EP3_TXFIFO_BYTES ((CONFIG_USBDEV_EP3_TXFIFO_SIZE + 3) & ~3)
#define STM32_EP3_TXFIFO_WORDS ((CONFIG_USBDEV_EP3_TXFIFO_SIZE + 3) >> 2)

#if STM32_EP3_TXFIFO_WORDS < 16
#  error "CONFIG_USBDEV_EP3_TXFIFO_SIZE is out of range"
#endif

/* Debug ***********************************************************************/
/* Trace error codes */

#define STM32_TRACEERR_ALLOCFAIL            0x01
#define STM32_TRACEERR_BADCLEARFEATURE      0x02
#define STM32_TRACEERR_BADDEVGETSTATUS      0x03
#define STM32_TRACEERR_BADEPNO              0x04
#define STM32_TRACEERR_BADEPGETSTATUS       0x05
#define STM32_TRACEERR_BADGETCONFIG         0x06
#define STM32_TRACEERR_BADGETSETDESC        0x07
#define STM32_TRACEERR_BADGETSTATUS         0x08
#define STM32_TRACEERR_BADSETADDRESS        0x09
#define STM32_TRACEERR_BADSETCONFIG         0x0a
#define STM32_TRACEERR_BADSETFEATURE        0x0b
#define STM32_TRACEERR_BADTESTMODE          0x0c
#define STM32_TRACEERR_BINDFAILED           0x0d
#define STM32_TRACEERR_DISPATCHSTALL        0x0e
#define STM32_TRACEERR_DRIVER               0x0f
#define STM32_TRACEERR_DRIVERREGISTERED     0x10
#define STM32_TRACEERR_EP0NOSETUP           0x11
#define STM32_TRACEERR_EP0SETUPSTALLED      0x12
#define STM32_TRACEERR_EPINNULLPACKET       0x13
#define STM32_TRACEERR_EPOUTNULLPACKET      0x14
#define STM32_TRACEERR_INVALIDCTRLREQ       0x15
#define STM32_TRACEERR_INVALIDPARMS         0x16
#define STM32_TRACEERR_IRQREGISTRATION      0x17
#define STM32_TRACEERR_NOEP                 0x18
#define STM32_TRACEERR_NOTCONFIGURED        0x19
#define STM32_TRACEERR_EPOUTQEMPTY          0x1a
#define STM32_TRACEERR_EPINREQEMPTY         0x1b
#define STM32_TRACEERR_NOOUTSETUP           0x1c
#define STM32_TRACEERR_POLLTIMEOUT          0x1d

/* Trace interrupt codes */

#define STM32_TRACEINTID_USB                1       /* USB Interrupt entry/exit */
#define STM32_TRACEINTID_INTPENDING         2       /* On each pass through the loop */

#define STM32_TRACEINTID_EPOUT              (10 + 0) /* First level interrupt decode */
#define STM32_TRACEINTID_EPIN               (10 + 1)
#define STM32_TRACEINTID_MISMATCH           (10 + 2)
#define STM32_TRACEINTID_WAKEUP             (10 + 3)
#define STM32_TRACEINTID_SUSPEND            (10 + 4)
#define STM32_TRACEINTID_SOF                (10 + 5)
#define STM32_TRACEINTID_RXFIFO             (10 + 6)
#define STM32_TRACEINTID_DEVRESET           (10 + 7)
#define STM32_TRACEINTID_ENUMDNE            (10 + 8)
#define STM32_TRACEINTID_IISOIXFR           (10 + 9)
#define STM32_TRACEINTID_IISOOXFR           (10 + 10)
#define STM32_TRACEINTID_SRQ                (10 + 11)
#define STM32_TRACEINTID_OTG                (10 + 12)

#define STM32_TRACEINTID_EPOUT_XFRC         (40 + 0) /* EPOUT second level decode */
#define STM32_TRACEINTID_EPOUT_EPDISD       (40 + 1)
#define STM32_TRACEINTID_EPOUT_SETUP        (40 + 2)
#define STM32_TRACEINTID_DISPATCH           (40 + 3)

#define STM32_TRACEINTID_GETSTATUS          (50 + 0) /* EPOUT third level decode */
#define STM32_TRACEINTID_EPGETSTATUS        (50 + 1)
#define STM32_TRACEINTID_DEVGETSTATUS       (50 + 2)
#define STM32_TRACEINTID_IFGETSTATUS        (50 + 3)
#define STM32_TRACEINTID_CLEARFEATURE       (50 + 4)
#define STM32_TRACEINTID_SETFEATURE         (50 + 5)
#define STM32_TRACEINTID_SETADDRESS         (50 + 6)
#define STM32_TRACEINTID_GETSETDESC         (50 + 7)
#define STM32_TRACEINTID_GETCONFIG          (50 + 8)
#define STM32_TRACEINTID_SETCONFIG          (50 + 9)
#define STM32_TRACEINTID_GETSETIF           (50 + 10)
#define STM32_TRACEINTID_SYNCHFRAME         (50 + 11)

#define STM32_TRACEINTID_EPIN_XFRC          (70 + 0) /* EPIN second level decode */
#define STM32_TRACEINTID_EPIN_TOC           (70 + 1)
#define STM32_TRACEINTID_EPIN_ITTXFE        (70 + 2)
#define STM32_TRACEINTID_EPIN_EPDISD        (70 + 3)
#define STM32_TRACEINTID_EPIN_TXFE          (70 + 4)

#define STM32_TRACEINTID_EPIN_EMPWAIT       (80 + 0) /* EPIN second level decode */

#define STM32_TRACEINTID_OUTNAK             (90 + 0) /* RXFLVL second level decode */
#define STM32_TRACEINTID_OUTRECVD           (90 + 1)
#define STM32_TRACEINTID_OUTDONE            (90 + 2)
#define STM32_TRACEINTID_SETUPDONE          (90 + 3)
#define STM32_TRACEINTID_SETUPRECVD         (90 + 4)

/* Endpoints ******************************************************************/

/* Number of endpoints */

#define STM32_NENDPOINTS             (4)          /* ep0-3 x 2 for IN and OUT */

/* Odd physical endpoint numbers are IN; even are OUT */

#define STM32_EPPHYIN2LOG(epphy)     ((uint8_t)(epphy)|USB_DIR_IN)
#define STM32_EPPHYOUT2LOG(epphy)    ((uint8_t)(epphy)|USB_DIR_OUT)

/* Endpoint 0 */

#define EP0                          (0)

/* The set of all enpoints available to the class implementation (1-3) */

#define STM32_EP_AVAILABLE           (0x0e)       /* All available endpoints */

/* Maximum packet sizes for full speed endpoints */

#define STM32_MAXPACKET              (64)         /* Max packet size (1-64) */

/* Delays **********************************************************************/

#define STM32_READY_DELAY            200000
#define STM32_FLUSH_DELAY            200000

/* Request queue operations ****************************************************/

#define stm32_rqempty(ep)            ((ep)->head == NULL)
#define stm32_rqpeek(ep)             ((ep)->head)

/* Standard stuff **************************************************************/

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/* Overall device state */

enum stm32_devstate_e
{
  DEVSTATE_DEFAULT = 0,    /* Power-up, unconfigured state.  This state simply
                            * means that the device is not yet been given an
                            * address.
                            *   SET:    At initialization, uninitialization,
                            *           reset, and whenever the device address
                            *           is set to zero
                            *   TESTED: Never
                            */
  DEVSTATE_ADDRESSED,      /* Device address has been assigned, not no
                            * configuration has yet been selected.
                            *   SET:    When either a non-zero device address
                            *           is first assigned or when the device
                            *           is unconfigured (with configuration == 0)
                            *   TESTED: never
                            */
  DEVSTATE_CONFIGURED,     /* Address assigned and configured:
                            *   SET:    When the device has been addressed and
                            *           an non-zero configuration has been selected.
                            *   TESTED: In many places to assure that the USB device
                            *           has been properly configured by the host.
                            */
};

/* Endpoint 0 states */

enum stm32_ep0state_e
{
  EP0STATE_IDLE = 0,       /* Idle State, leave on receiving a SETUP packet or
                            * epsubmit:
                            *   SET:    In stm32_epin() and stm32_epout() when
                            *           we revert from request processing to
                            *           SETUP processing.
                            *   TESTED: Never
                            */
  EP0STATE_SETUP_OUT,      /* OUT SETUP packet received.  Waiting for the DATA
                            * OUT phase of SETUP Packet to complete before
                            * processing a SETUP command (without a USB request):
                            *   SET:    Set in stm32_rxinterrupt() when SETUP OUT
                            *           packet is received.
                            *   TESTED: In stm32_ep0out_receive()
                            */
  EP0STATE_SETUP_READY,    /* IN SETUP packet received -OR- OUT SETUP packet and
                            * accompanying data have been received.  Processing
                            * of SETUP command will happen soon.
                            *   SET:    (1) stm32_ep0out_receive() when the OUT
                            *           SETUP data phase completes, or (2)
                            *           stm32_rxinterrupt() when an IN SETUP is
                            *           packet received.
                            *   TESTED: Tested in stm32_epout_interrupt() when
                            *           SETUP phase is done to see if the SETUP
                            *           command is ready to be processed.  Also
                            *           tested in stm32_ep0out_setup() just to
                            *           double-check that we have a SETUP request
                            *           and any accompanying data.
                            */
  EP0STATE_SETUP_PROCESS,  /* SETUP Packet is being processed by stm32_ep0out_setup():
                            *   SET:    When SETUP packet received in EP0 OUT
                            *   TESTED: Never
                            */
  EP0STATE_SETUPRESPONSE,  /* Short SETUP response write (without a USB request):
                            *   SET:    When SETUP response is sent by
                            *           stm32_ep0in_setupresponse()
                            *   TESTED: Never
                            */
  EP0STATE_DATA_IN,        /* Waiting for data out stage (with a USB request):
                            *   SET:    In stm32_epin_request() when a write
                            *           request is processed on EP0.
                            *   TESTED: In stm32_epin() to see if we should
                            *           revert to SETUP processing.
                            */
  EP0STATE_DATA_OUT        /* Waiting for data in phase to complete ( with a
                            * USB request)
                            *   SET:    In stm32_epout_request() when a read
                            *           request is processed on EP0.
                            *   TESTED: In stm32_epout() to see if we should
                            *           revert to SETUP processing
                            */
};

/* Parsed control request */

struct stm32_ctrlreq_s
{
  uint8_t  type;
  uint8_t  req;
  uint16_t value;
  uint16_t index;
  uint16_t len;
};

/* A container for a request so that the request may be retained in a list */

struct stm32_req_s
{
  struct usbdev_req_s  req;           /* Standard USB request */
  struct stm32_req_s  *flink;         /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct stm32_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct stm32_ep_s.
   */

  struct usbdev_ep_s      ep;          /* Standard endpoint structure */

  /* STM32-specific fields */

  struct stm32_usbdev_s *dev;          /* Reference to private driver data */
  struct stm32_req_s    *head;         /* Request list for this endpoint */
  struct stm32_req_s    *tail;
  uint8_t                epphy;        /* Physical EP address */
  uint8_t                eptype:2;     /* Endpoint type */
  uint8_t                configured:1; /* 1: Endpoint has been configured */
  uint8_t                active:1;     /* 1: A request is being processed */
  uint8_t                stalled:1;    /* 1: Endpoint is stalled */
  uint8_t                isin:1;       /* 1: IN Endpoint */
  uint8_t                odd:1;        /* 1: Odd frame */
  uint8_t                zlp:1;        /* 1: Transmit a zero-length-packet (IN EPs only) */
};

/* This structure retains the state of the USB device controller */

struct stm32_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to struct stm32_usbdev_s.
   */

  struct usbdev_s         usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* STM32-specific fields */

  uint8_t                 stalled:1;     /* 1: Protocol stalled */
  uint8_t                 selfpowered:1; /* 1: Device is self powered */
  uint8_t                 addressed:1;   /* 1: Peripheral address has been set */
  uint8_t                 configured:1;  /* 1: Class driver has been configured */
  uint8_t                 wakeup:1;      /* 1: Device remote wake-up */
  uint8_t                 dotest:1;      /* 1: Test mode selected */

  uint8_t                 devstate:4;    /* See enum stm32_devstate_e */
  uint8_t                 ep0state:4;    /* See enum stm32_ep0state_e */
  uint8_t                 testmode:4;    /* Selected test mode */
  uint8_t                 epavail:4;     /* Bitset of available endpoints */

  /* E0 SETUP data buffering.
   *
   * ctrlreq:
   *   The 8-byte SETUP request is received on the EP0 OUT endpoint and is
   *   saved.
   *
   * ep0data
   *   For OUT SETUP requests, the SETUP data phase must also complete before
   *   the SETUP command can be processed.  The pack receipt logic will save
   *   the accompanying EP0 IN data in ep0data[] before the SETUP command is
   *   processed.
   *
   *   For IN SETUP requests, the DATA phase will occurr AFTER the SETUP
   *   control request is processed.  In that case, ep0data[] may be used as
   *   the response buffer.
   *
   * ep0datlen
   *   Lenght of OUT DATA received in ep0data[] (Not used with OUT data)
   */

  struct usb_ctrlreq_s    ctrlreq;
  uint8_t                 ep0data[CONFIG_USBDEV_SETUP_MAXDATASIZE];
  uint16_t                ep0datlen;

  /* The endpoint lists */

  struct stm32_ep_s       epin[STM32_NENDPOINTS];
  struct stm32_ep_s       epout[STM32_NENDPOINTS];
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Register operations ********************************************************/

#if defined(CONFIG_STM32_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t    stm32_getreg(uint32_t addr);
static void        stm32_putreg(uint32_t val, uint32_t addr);
#else
# define stm32_getreg(addr)     getreg32(addr)
# define stm32_putreg(val,addr) putreg32(val,addr)
#endif

/* Request queue operations ****************************************************/

static FAR struct stm32_req_s *stm32_req_remfirst(FAR struct stm32_ep_s *privep);
static bool       stm32_req_addlast(FAR struct stm32_ep_s *privep,
                    FAR struct stm32_req_s *req);

/* Low level data transfers and request operations *****************************/
/* Special endpoint 0 data transfer logic */

static void        stm32_ep0in_setupresponse(FAR struct stm32_usbdev_s *priv,
                     FAR uint8_t *data, uint32_t nbytes);
static inline void stm32_ep0in_transmitzlp(FAR struct stm32_usbdev_s *priv);
static void        stm32_ep0in_activate(void);

static void        stm32_ep0out_ctrlsetup(FAR struct stm32_usbdev_s *priv);

/* IN request and TxFIFO handling */

static void        stm32_txfifo_write(FAR struct stm32_ep_s *privep,
                     FAR uint8_t *buf, int nbytes);
static void        stm32_epin_transfer(FAR struct stm32_ep_s *privep,
                     FAR uint8_t *buf, int nbytes);
static void        stm32_epin_request(FAR struct stm32_usbdev_s *priv,
                     FAR struct stm32_ep_s *privep);

/* OUT request and RxFIFO handling */

static void        stm32_rxfifo_read(FAR struct stm32_ep_s *privep,
                     FAR uint8_t *dest, uint16_t len);
static void        stm32_rxfifo_discard(FAR struct stm32_ep_s *privep, int len);
static void        stm32_epout_complete(FAR struct stm32_usbdev_s *priv,
                     FAR struct stm32_ep_s *privep);
static inline void stm32_ep0out_receive(FAR struct stm32_ep_s *privep, int bcnt);
static inline void stm32_epout_receive(FAR struct stm32_ep_s *privep, int bcnt);
static void        stm32_epout_request(FAR struct stm32_usbdev_s *priv,
                     FAR struct stm32_ep_s *privep);

/* General request handling */

static void        stm32_ep_flush(FAR struct stm32_ep_s *privep);
static void        stm32_req_complete(FAR struct stm32_ep_s *privep,
                     int16_t result);
static void        stm32_req_cancel(FAR struct stm32_ep_s *privep,
                     int16_t status);

/* Interrupt handling **********************************************************/

static struct      stm32_ep_s *stm32_ep_findbyaddr(struct stm32_usbdev_s *priv,
                     uint16_t eplog);
static int         stm32_req_dispatch(FAR struct stm32_usbdev_s *priv,
                     FAR const struct usb_ctrlreq_s *ctrl);
static void        stm32_usbreset(FAR struct stm32_usbdev_s *priv);

/* Second level OUT endpoint interrupt processing */

static inline void stm32_ep0out_testmode(FAR struct stm32_usbdev_s *priv,
                     uint16_t index);
static inline void stm32_ep0out_stdrequest(struct stm32_usbdev_s *priv,
                     FAR struct stm32_ctrlreq_s *ctrlreq);
static inline void stm32_ep0out_setup(struct stm32_usbdev_s *priv);
static inline void stm32_epout(FAR struct stm32_usbdev_s *priv,
                     uint8_t epno);
static inline void stm32_epout_interrupt(FAR struct stm32_usbdev_s *priv);

/* Second level IN endpoint interrupt processing */

static inline void stm32_epin_runtestmode(FAR struct stm32_usbdev_s *priv);
static inline void stm32_epin(FAR struct stm32_usbdev_s *priv, uint8_t epno);
static inline void stm32_epin_txfifoempty(FAR struct stm32_usbdev_s *priv, int epno);
static inline void stm32_epin_interrupt(FAR struct stm32_usbdev_s *priv);

/* Other second level interrupt processing */

static inline void stm32_resumeinterrupt(FAR struct stm32_usbdev_s *priv);
static inline void stm32_suspendinterrupt(FAR struct stm32_usbdev_s *priv);
static inline void stm32_rxinterrupt(FAR struct stm32_usbdev_s *priv);
static inline void stm32_enuminterrupt(FAR struct stm32_usbdev_s *priv);
#ifdef CONFIG_USBDEV_ISOCHRONOUS
static inline void stm32_isocininterrupt(FAR struct stm32_usbdev_s *priv);
static inline void stm32_isocoutinterrupt(FAR struct stm32_usbdev_s *priv);
#endif
#ifdef CONFIG_USBDEV_VBUSSENSING
static inline void stm32_sessioninterrupt(FAR struct stm32_usbdev_s *priv);
static inline void stm32_otginterrupt(FAR struct stm32_usbdev_s *priv);
#endif

/* First level interrupt processing */

static int         stm32_usbinterrupt(int irq, FAR void *context);

/* Endpoint operations *********************************************************/
/* Global OUT NAK controls */

static void        stm32_enablegonak(FAR struct stm32_ep_s *privep);
static void        stm32_disablegonak(FAR struct stm32_ep_s *privep);

/* Endpoint configuration */

static int         stm32_epout_configure(FAR struct stm32_ep_s *privep,
                     uint8_t eptype, uint16_t maxpacket);
static int         stm32_epin_configure(FAR struct stm32_ep_s *privep,
                     uint8_t eptype, uint16_t maxpacket);
static int         stm32_ep_configure(FAR struct usbdev_ep_s *ep,
                     FAR const struct usb_epdesc_s *desc, bool last);
static void        stm32_ep0_configure(FAR struct stm32_usbdev_s *priv);

/* Endpoint disable */

static void        stm32_epout_disable(FAR struct stm32_ep_s *privep);
static void        stm32_epin_disable(FAR struct stm32_ep_s *privep);
static int         stm32_ep_disable(FAR struct usbdev_ep_s *ep);

/* Endpoint request management */

static FAR struct usbdev_req_s *stm32_ep_allocreq(FAR struct usbdev_ep_s *ep);
static void        stm32_ep_freereq(FAR struct usbdev_ep_s *ep,
                     FAR struct usbdev_req_s *);

/* Endpoint buffer management */

#ifdef CONFIG_USBDEV_DMA
static void       *stm32_ep_allocbuffer(FAR struct usbdev_ep_s *ep, unsigned bytes);
static void        stm32_ep_freebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf);
#endif

/* Endpoint request submission */

static int         stm32_ep_submit(FAR struct usbdev_ep_s *ep,
                     struct usbdev_req_s *req);

/* Endpoint request cancellation */

static int         stm32_ep_cancel(FAR struct usbdev_ep_s *ep,
                     struct usbdev_req_s *req);

/* Stall handling */

static int         stm32_epout_setstall(FAR struct stm32_ep_s *privep);
static int         stm32_epin_setstall(FAR struct stm32_ep_s *privep);
static int         stm32_ep_setstall(FAR struct stm32_ep_s *privep);
static int         stm32_ep_clrstall(FAR struct stm32_ep_s *privep);
static int         stm32_ep_stall(FAR struct usbdev_ep_s *ep, bool resume);
static void        stm32_ep0_stall(FAR struct stm32_usbdev_s *priv);

/* Endpoint allocation */

static FAR struct usbdev_ep_s *stm32_ep_alloc(FAR struct usbdev_s *dev,
                     uint8_t epno, bool in, uint8_t eptype);
static void        stm32_ep_free(FAR struct usbdev_s *dev,
                     FAR struct usbdev_ep_s *ep);

/* USB device controller operations ********************************************/

static int         stm32_getframe(struct usbdev_s *dev);
static int         stm32_wakeup(struct usbdev_s *dev);
static int         stm32_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int         stm32_pullup(struct usbdev_s *dev, bool enable);
static void        stm32_setaddress(struct stm32_usbdev_s *priv,
                     uint16_t address);
static int         stm32_txfifo_flush(uint32_t txfnum);
static int         stm32_rxfifo_flush(void);

/* Initialization **************************************************************/

static void        stm32_swinitialize(FAR struct stm32_usbdev_s *priv);
static void        stm32_hwinitialize(FAR struct stm32_usbdev_s *priv);

/*******************************************************************************
 * Private Data
 *******************************************************************************/
/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct stm32_usbdev_s g_otgfsdev;

static const struct usbdev_epops_s g_epops =
{
  .configure   = stm32_ep_configure,
  .disable     = stm32_ep_disable,
  .allocreq    = stm32_ep_allocreq,
  .freereq     = stm32_ep_freereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = stm32_ep_allocbuffer,
  .freebuffer  = stm32_ep_freebuffer,
#endif
  .submit      = stm32_ep_submit,
  .cancel      = stm32_ep_cancel,
  .stall       = stm32_ep_stall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = stm32_ep_alloc,
  .freeep      = stm32_ep_free,
  .getframe    = stm32_getframe,
  .wakeup      = stm32_wakeup,
  .selfpowered = stm32_selfpowered,
  .pullup      = stm32_pullup,
};

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: stm32_getreg
 *
 * Description:
 *   Get the contents of an STM32 register
 *
 *******************************************************************************/

#if defined(CONFIG_STM32_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t stm32_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same registe last time?  Are
   * we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
           if (count == 4)
             {
               lldbg("...\n");
             }
          return val;
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
       preval   = val;
       count    = 1;
    }

  /* Show the register value read */

  lldbg("%08x->%08x\n", addr, val);
  return val;
}
#endif

/*******************************************************************************
 * Name: stm32_putreg
 *
 * Description:
 *   Set the contents of an STM32 register to a value
 *
 *******************************************************************************/

#if defined(CONFIG_STM32_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static void stm32_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/*******************************************************************************
 * Name: stm32_req_remfirst
 *
 * Description:
 *   Remove a request from the head of an endpoint request queue
 *
 *******************************************************************************/

static FAR struct stm32_req_s *stm32_req_remfirst(FAR struct stm32_ep_s *privep)
{
  FAR struct stm32_req_s *ret = privep->head;

  if (ret)
    {
      privep->head = ret->flink;
      if (!privep->head)
        {
          privep->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}

/*******************************************************************************
 * Name: stm32_req_addlast
 *
 * Description:
 *   Add a request to the end of an endpoint request queue
 *
 *******************************************************************************/

static bool stm32_req_addlast(FAR struct stm32_ep_s *privep,
                              FAR struct stm32_req_s *req)
{
  bool is_empty = !privep->head;

  req->flink = NULL;
  if (is_empty)
    {
      privep->head = req;
      privep->tail = req;
    }
  else
    {
      privep->tail->flink = req;
      privep->tail        = req;
    }
  return is_empty;
}

/*******************************************************************************
 * Name: stm32_ep0in_setupresponse
 *
 * Description:
 *   Schedule a short transfer on Endpoint 0 (IN or OUT)
 *
 *******************************************************************************/

static void stm32_ep0in_setupresponse(FAR struct stm32_usbdev_s *priv,
                                      FAR uint8_t *buf, uint32_t nbytes)
{
  stm32_epin_transfer(&priv->epin[EP0], buf, nbytes);
  priv->ep0state = EP0STATE_SETUPRESPONSE;
  stm32_ep0out_ctrlsetup(priv);
}

/****************************************************************************
 * Name: stm32_ep0in_transmitzlp
 *
 * Description:
 *   Send a zero length packet (ZLP) on endpoint 0 IN
 *
 ****************************************************************************/

static inline void stm32_ep0in_transmitzlp(FAR struct stm32_usbdev_s *priv)
{
  stm32_ep0in_setupresponse(priv, NULL, 0);
}

/*******************************************************************************
 * Name: stm32_ep0in_activate
 *
 * Description:
 *   Activate the endpoint 0 IN endpoint.
 *
 *******************************************************************************/

static void stm32_ep0in_activate(void)
{
  uint32_t regval;

  /* Set the max packet size  of the IN EP. */

  regval  = stm32_getreg(STM32_OTGFS_DIEPCTL0);
  regval &= ~OTGFS_DIEPCTL0_MPSIZ_MASK;

#if CONFIG_USBDEV_EP0_MAXSIZE == 8
  regval |= OTGFS_DIEPCTL0_MPSIZ_8;
#elif CONFIG_USBDEV_EP0_MAXSIZE == 16
  regval |= OTGFS_DIEPCTL0_MPSIZ_16;
#elif CONFIG_USBDEV_EP0_MAXSIZE == 32
  regval |= OTGFS_DIEPCTL0_MPSIZ_32;
#elif CONFIG_USBDEV_EP0_MAXSIZE == 64
  regval |= OTGFS_DIEPCTL0_MPSIZ_64;
#else
#  error "Unsupported value of CONFIG_USBDEV_EP0_MAXSIZE"
#endif

  stm32_putreg(regval, STM32_OTGFS_DIEPCTL0);

  /* Clear global IN NAK */

  regval  = stm32_getreg(STM32_OTGFS_DCTL);
  regval |= OTGFS_DCTL_CGINAK;
  stm32_putreg(regval, STM32_OTGFS_DCTL);
}

/*******************************************************************************
 * Name: stm32_ep0out_ctrlsetup
 *
 * Description:
 *   Setup to receive a SETUP packet.
 *
 *******************************************************************************/

static void stm32_ep0out_ctrlsetup(FAR struct stm32_usbdev_s *priv)
{
  uint32_t regval;

  /* Setup the hardware to perform the SETUP transfer */

  regval = (USB_SIZEOF_CTRLREQ * 3 << OTGFS_DOEPTSIZ0_XFRSIZ_SHIFT) |
           (OTGFS_DOEPTSIZ0_PKTCNT) |
           (3 << OTGFS_DOEPTSIZ0_STUPCNT_SHIFT);
  stm32_putreg(regval, STM32_OTGFS_DOEPTSIZ0);

  /* Then clear NAKing and enable the transfer */

  regval  = stm32_getreg(STM32_OTGFS_DOEPCTL0);
  regval |= (OTGFS_DOEPCTL0_CNAK | OTGFS_DOEPCTL0_EPENA);
  stm32_putreg(regval, STM32_OTGFS_DOEPCTL0);
}

/****************************************************************************
 * Name: stm32_txfifo_write
 *
 * Description:
 *   Send data to the endpoint's TxFIFO.
 *
 ****************************************************************************/

static void stm32_txfifo_write(FAR struct stm32_ep_s *privep,
                               FAR uint8_t *buf, int nbytes)
{
  uint32_t regaddr;
  uint32_t regval;
  int nwords;
  int i;

  /* Convert the number of bytes to words */

  nwords = (nbytes + 3) >> 2;

  /* Get the TxFIFO for this endpoint (same as the endpoint number) */

  regaddr = STM32_OTGFS_DFIFO_DEP(privep->epphy);

  /* Then transfer each word to the TxFIFO */

  for (i = 0; i < nwords; i++)
    {
      /* Read four bytes from the source buffer (to avoid unaligned accesses)
       * and pack these into one 32-bit word (little endian).
       */

      regval  = (uint32_t)*buf++;
      regval |= ((uint32_t)*buf++) << 8;
      regval |= ((uint32_t)*buf++) << 16;
      regval |= ((uint32_t)*buf++) << 24;

      /* Then write the packet data to the TxFIFO */

      stm32_putreg(regval, regaddr);
    }
}

/****************************************************************************
 * Name: stm32_epin_transfer
 *
 * Description:
 *   Start the Tx data transfer
 *
 ****************************************************************************/

static void stm32_epin_transfer(FAR struct stm32_ep_s *privep,
                                FAR uint8_t *buf, int nbytes)
{
  uint32_t pktcnt;
  uint32_t regval;

  /* Read the DIEPSIZx register */

  regval = stm32_getreg(STM32_OTGFS_DIEPTSIZ(privep->epphy));

  /* Clear the XFRSIZ, PKTCNT, and MCNT field of the DIEPSIZx register */

  regval &= ~(OTGFS_DIEPTSIZ_XFRSIZ_MASK | OTGFS_DIEPTSIZ_PKTCNT_MASK |
              OTGFS_DIEPTSIZ_MCNT_MASK);

  /* Are we sending a zero length packet (ZLP) */

  if (nbytes == 0)
    {
      /* Yes.. leave the transfer size at zero and set the packet count to 1 */

      pktcnt = 1;
    }
  else
    {
      /* No.. Program the transfer size and packet count .  First calculate:
       *
       * xfrsize = The total number of bytes to be sent.
       * pktcnt  = the number of packets (of maxpacket bytes) required to
       *   perform the transfer.
       */

      pktcnt  = ((uint32_t)nbytes + (privep->ep.maxpacket - 1)) / privep->ep.maxpacket;
    }

  /* Set the XFRSIZ and PKTCNT */

  regval |= (pktcnt << OTGFS_DIEPTSIZ_PKTCNT_SHIFT);
  regval |= ((uint32_t)nbytes << OTGFS_DIEPTSIZ_XFRSIZ_SHIFT);

  /* If this is an isconchronous endpoint, then set the multi-count field to
   * the PKTCNT as well.
   */

  if (privep->eptype == USB_EP_ATTR_XFER_ISOC)
    {
      regval |= (pktcnt << OTGFS_DIEPTSIZ_MCNT_SHIFT);
    }

  /* Save DIEPSIZx register value */

  stm32_putreg(regval, STM32_OTGFS_DIEPTSIZ(privep->epphy));

  /* Read the DIEPCTLx register */

  regval = stm32_getreg(STM32_OTGFS_DIEPCTL(privep->epphy));

  /* If this is an isochronous endpoint, then set the even/odd frame bit
   * the DIEPCTLx register.
   */

  if (privep->eptype == USB_EP_ATTR_XFER_ISOC)
    {
      /* Check bit 0 of the frame number of the received SOF and set the
       * even/odd frame to match.
       */

      uint32_t status = stm32_getreg(STM32_OTGFS_DSTS);
      if ((status & OTGFS_DSTS_SOFFN0) == OTGFS_DSTS_SOFFN_EVEN)
        {
          regval |= OTGFS_DIEPCTL_SEVNFRM;
        }
      else
        {
          regval |= OTGFS_DIEPCTL_SODDFRM;
        }
    }

  /* EP enable, IN data in FIFO */

  regval &= ~OTGFS_DIEPCTL_EPDIS;
  regval |= (OTGFS_DIEPCTL_CNAK | OTGFS_DIEPCTL_EPENA);
  stm32_putreg(regval, STM32_OTGFS_DIEPCTL(privep->epphy));

  /* Transfer the data to the TxFIFO.  At this point, the caller has already
   * assured that there is sufficient space in the TxFIFO to hold the transfer
   * we can just blindly continue.
   */

  stm32_txfifo_write(privep, buf, nbytes);
}

/****************************************************************************
 * Name: stm32_epin_request
 *
 * Description:
 *   Begin or continue write request processing.
 *
 ****************************************************************************/

static void stm32_epin_request(FAR struct stm32_usbdev_s *priv,
                               FAR struct stm32_ep_s *privep)
{
  struct stm32_req_s *privreq;
  uint32_t regaddr;
  uint32_t regval;
  uint8_t *buf;
  int nbytes;
  int nwords;
  int bytesleft;

  /* We get here in one of four possible ways.  From three interrupting
   * events:
   *
   * 1. From stm32_epin as part of the transfer complete interrupt processing
   *    This interrupt indicates that the last transfer has completed.
   * 2. As part of the ITTXFE interrupt processing.  That interrupt indicates
   *    that an IN token was received when the associated TxFIFO was empty.
   * 3. From stm32_epin_txfifoempty as part of the TXFE interrupt processing.
   *    The TXFE interrupt is only enabled when the TxFIFO is full and the
   *    software must wait for space to become available in the TxFIFO.
   *
   * And this function may be called immediately when the write request is
   * queue to start up the next transaction.
   *
   * 4. From stm32_ep_submit when a new write request is received WHILE the
   *    endpoint is not active (privep->active == false).
   */

  /* Check the request from the head of the endpoint request queue */

  privreq = stm32_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EPINREQEMPTY), privep->epphy);

      /* There is no TX transfer in progress and no new pending TX
       * requests to send.  To stop transmitting any data on a particular
       * IN endpoint, the application must set the IN NAK bit. To set this
       * bit, the following field must be programmed.
       */

      regaddr = STM32_OTGFS_DIEPCTL(privep->epphy);
      regval  = stm32_getreg(regaddr);
      regval |= OTGFS_DIEPCTL_SNAK;
      stm32_putreg(regval, regaddr);

      /* The endpoint is no longer active */

      privep->active = false;
      return;
    }

  ullvdbg("EP%d req=%p: len=%d xfrd=%d zlp=%d\n",
          privep->epphy, privreq, privreq->req.len,
          privreq->req.xfrd, privep->zlp);

  /* Check for a special case:  If we are just starting a request (xfrd==0) and
   * the class driver is trying to send a zero-length packet (len==0).  Then set
   * the ZLP flag so that the packet will be sent.
   */

  if (privreq->req.len == 0)
    {
       /* The ZLP flag is set TRUE whenever we want to force the driver to
        * send a zero-length-packet on the next pass through the loop (below).
        * The flag is cleared whenever a packet is sent in the loop below.
        */

       privep->zlp = true;
    }

  /* Add one more packet to the TxFIFO.  We will wait for the transfer
   * complete event before we add the next packet (or part of a packet
   * to the TxFIFO).
   * 
   * The documentation says that we can can multiple packets to the TxFIFO,
   * but it seems that we need to get the transfer complete event before
   * we can add the next (or maybe I have got something wrong?)
   */

#if 0
  while (privreq->req.xfrd < privreq->req.len || privep->zlp)
#else
  if (privreq->req.xfrd < privreq->req.len || privep->zlp)
#endif
    {
      /* Get the number of bytes left to be sent in the request */

      bytesleft = privreq->req.len - privreq->req.xfrd;
      nbytes    = bytesleft;

      /* Assume no zero-length-packet on the next pass through this loop */

      privep->zlp = false;

      /* Limit the size of the transfer to one full packet and handle
       * zero-length packets (ZLPs).
       */

      if (nbytes > 0)
        {
          /* Either send the maxpacketsize or all of the remaining data in
           * the request.
           */

          if (nbytes >= privep->ep.maxpacket)
            {
              nbytes =  privep->ep.maxpacket;

              /* Handle the case where this packet is exactly the
               * maxpacketsize.  Do we need to send a zero-length packet
               * in this case?
               */

              if (bytesleft ==  privep->ep.maxpacket &&
                 (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0)
                {
                  /* The ZLP flag is set TRUE whenever we want to force
                   * the driver to send a zero-length-packet on the next
                   * pass through this loop. The flag is cleared (above)
                   * whenever we are committed to sending any packet and
                   * set here when we want to force one more pass through
                   * the loop.
                   */

                  privep->zlp = true;
                }
            }
        }

      /* Get the transfer size in 32-bit words */

      nwords = (nbytes + 3) >> 2;

      /* Get the number of 32-bit words available in the TxFIFO. The
       * DXTFSTS indicates the amount of free space available in the
       * endpoint TxFIFO. Values are in terms of 32-bit words:
       *
       *   0: Endpoint TxFIFO is full
       *   1: 1 word available
       *   2: 2 words available
       *   n: n words available
       */

      regaddr = STM32_OTGFS_DTXFSTS(privep->epphy);

      /* Check for space in the TxFIFO.  If space in the TxFIFO is not
       * available, then set up an interrupt to resume the transfer when
       * the TxFIFO is empty.
       */

      regval = stm32_getreg(regaddr);
      if ((regval & OTGFS_DTXFSTS_MASK) < nwords)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN_EMPWAIT), (uint16_t)regval);

          /* There is insufficent space in the TxFIFO.  Wait for a TxFIFO
           * empty interrupt and try again.
           */

          uint32_t empmsk = stm32_getreg(STM32_OTGFS_DIEPEMPMSK);
          empmsk |= OTGFS_DIEPEMPMSK(privep->epphy);
          stm32_putreg(empmsk, STM32_OTGFS_DIEPEMPMSK);

          /* Terminate the transfer.  We will try again when the TxFIFO empty
           * interrupt is received.
           */

          return;
        }

      /* Transfer data to the TxFIFO */

      buf = privreq->req.buf + privreq->req.xfrd;
      stm32_epin_transfer(privep, buf, nbytes);

      /* If it was not before, the OUT endpoint is now actively transferring
       * data.
       */

      privep->active = true;

      /* EP0 is a special case */

      if (privep->epphy == EP0)
        {
          priv->ep0state = EP0STATE_DATA_IN;
        }

      /* Update for the next time through the loop */

      privreq->req.xfrd += nbytes;
    }

  /* Note that the ZLP, if any, must be sent as a separate transfer.  The need
   * for a ZLP is indicated by privep->zlp.  If all of the bytes were sent
   * (including any final null packet) then we are finished with the transfer
   */

  if (privreq->req.xfrd >= privreq->req.len && !privep->zlp)
    {
      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);

      /* We are finished with the request (although the transfer has not
       * yet completed).
       */

      stm32_req_complete(privep, OK);
    }
}

/*******************************************************************************
 * Name: stm32_rxfifo_read
 *
 * Description:
 *   Read packet from the RxFIFO into a read request.
 *
 *******************************************************************************/

static void stm32_rxfifo_read(FAR struct stm32_ep_s *privep,
                              FAR uint8_t *dest, uint16_t len)
{
  uint32_t regaddr;
  int i;

  /* Get the address of the RxFIFO.  Note:  there is only one RxFIFO so
   * we might as well use the addess associated with EP0.
   */

  regaddr = STM32_OTGFS_DFIFO_DEP(EP0);

  /* Read 32-bits and write 4 x 8-bits at time (to avoid unaligned accesses) */

  for (i = 0; i < len; i += 4)
    {
      union
      {
        uint32_t w;
        uint8_t  b[4];
      } data;

      /* Read 1 x 32-bits of EP0 packet data */

      data.w = stm32_getreg(regaddr);

      /* Write 4 x 8-bits of EP0 packet data */

      *dest++ = data.b[0];
      *dest++ = data.b[1];
      *dest++ = data.b[2];
      *dest++ = data.b[3];
    }
}

/*******************************************************************************
 * Name: stm32_rxfifo_discard
 *
 * Description:
 *   Discard packet data from the RxFIFO.
 *
 *******************************************************************************/

static void stm32_rxfifo_discard(FAR struct stm32_ep_s *privep, int len)
{
  if (len > 0)
    {
      uint32_t regaddr;
      int i;

      /* Get the address of the RxFIFO  Note:  there is only one RxFIFO so
       * we might as well use the addess associated with EP0.
       */

      regaddr = STM32_OTGFS_DFIFO_DEP(EP0);

      /* Read 32-bits at time */

      for (i = 0; i < len; i += 4)
        {
          volatile uint32_t data = stm32_getreg(regaddr);
          (void)data;
        }
    }
}

/*******************************************************************************
 * Name: stm32_epout_complete
 *
 * Description:
 *   This function is called when an OUT transfer complete interrupt is
 *   received.  It completes the read request at the head of the endpoint's
 *   request queue.
 *
 *******************************************************************************/

static void stm32_epout_complete(FAR struct stm32_usbdev_s *priv,
                                 FAR struct stm32_ep_s *privep)
{
  struct stm32_req_s *privreq;

  /* Since a transfer just completed, there must be a read request at the head of
   * the endpoint request queue.
   */

  privreq = stm32_rqpeek(privep);
  DEBUGASSERT(privreq);

  if (!privreq)
    {
      /* An OUT transfer completed, but no packet to receive the data.  This
       * should not happen.
       */

      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EPOUTQEMPTY), privep->epphy);
      privep->active = false;
      return;
    }

  ullvdbg("EP%d: len=%d xfrd=%d\n",
          privep->epphy, privreq->req.len, privreq->req.xfrd);

  /* Return the completed read request to the class driver and mark the state
   * IDLE.
   */

  usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
  stm32_req_complete(privep, OK);
  privep->active = false;

  /* Now set up the next read request (if any) */

  stm32_epout_request(priv, privep);
}

/*******************************************************************************
 * Name: stm32_ep0out_receive
 *
 * Description:
 *   This function is called from the RXFLVL interrupt handler when new incoming
 *   data is available in the endpoint's RxFIFO.  This function will simply
 *   copy the incoming data into pending request's data buffer.
 *
 *******************************************************************************/

static inline void stm32_ep0out_receive(FAR struct stm32_ep_s *privep, int bcnt)
{
  FAR struct stm32_usbdev_s *priv;

  /* Sanity Checking */

  DEBUGASSERT(privep && privep->ep.priv);
  priv = (FAR struct stm32_usbdev_s *)privep->ep.priv;
  DEBUGASSERT(priv->ep0state == EP0STATE_SETUP_OUT);

  ullvdbg("EP0: bcnt=%d\n", bcnt);
  usbtrace(TRACE_READ(EP0), bcnt);

  /* Verify that an OUT SETUP request as received before this data was
   * received in the RxFIFO.
   */

  if (priv->ep0state == EP0STATE_SETUP_OUT)
    {
      /* Read the data into our special buffer for SETUP data */

      int readlen = MIN(CONFIG_USBDEV_SETUP_MAXDATASIZE, bcnt);
      stm32_rxfifo_read(privep, priv->ep0data, readlen);

      /* Do we have to discard any excess bytes? */

      stm32_rxfifo_discard(privep,  bcnt - readlen);

      /* Now we can process the setup command */

      privep->active  = false;
      priv->ep0state  = EP0STATE_SETUP_READY;
      priv->ep0datlen = readlen;

      stm32_ep0out_setup(priv);
    }
  else
    {
      /* This is an error.  We don't have any idea what to do with the EP0
       * data in this case.  Just read and discard it so that the RxFIFO
       * does not become constipated.
       */

      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_NOOUTSETUP), priv->ep0state);
      stm32_rxfifo_discard(privep, bcnt);
      privep->active = false;
    }
}

/*******************************************************************************
 * Name: stm32_epout_receive
 *
 * Description:
 *   This function is called from the RXFLVL interrupt handler when new incoming
 *   data is available in the endpoint's RxFIFO.  This function will simply
 *   copy the incoming data into pending request's data buffer.
 *
 *******************************************************************************/

static inline void stm32_epout_receive(FAR struct stm32_ep_s *privep, int bcnt)
{
  struct stm32_req_s *privreq;
  uint8_t *dest;
  int buflen;
  int readlen;

  /* Get a reference to the request at the head of the endpoint's request
   * queue.
   */

  privreq = stm32_rqpeek(privep);
  if (!privreq)
    {
      /* Incoming data is available in the RxFIFO, but there is no read setup
       * to receive the receive the data.  This should not happen for data
       * endpoints; those endpoints should have been NAKing any OUT data tokens.
       *
       * We should get here normally on OUT data phase following an OUT
       * SETUP command.  EP0 data will still receive data in this case and it
       * should not be NAKing.
       */

      if (privep->epphy == 0)
        {
          stm32_ep0out_receive(privep, bcnt);
        }
      else
        {
          /* Otherwise, the data is lost. This really should not happen if
           * NAKing is working as expected.
           */

          usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EPOUTQEMPTY), privep->epphy);

          /* Discard the data in the RxFIFO */

          stm32_rxfifo_discard(privep, bcnt);
        }

      privep->active = false;
      return;
    }

  ullvdbg("EP%d: len=%d xfrd=%d\n", privep->epphy, privreq->req.len, privreq->req.xfrd);
  usbtrace(TRACE_READ(privep->epphy), bcnt);

  /* Get the number of bytes to transfer from the RxFIFO */

  buflen  = privreq->req.len - privreq->req.xfrd;
  DEBUGASSERT(buflen > 0 && buflen >= bcnt);
  readlen = MIN(buflen, bcnt);

  /* Get the destination of the data transfer */

  dest = privreq->req.buf + privreq->req.xfrd;

  /* Transfer the data from the RxFIFO to the request's data buffer */

  stm32_rxfifo_read(privep, dest, readlen);

  /* If there were more bytes in the RxFIFO than could be held in the read
   * request, then we will have to discard those.
   */

  stm32_rxfifo_discard(privep, bcnt - readlen);

  /* Update the number of bytes transferred */

  privreq->req.xfrd += readlen;
}

/*******************************************************************************
 * Name: stm32_epout_request
 *
 * Description:
 *   This function is called when either (1) new read request is received, or
 *   (2) a pending receive request completes.  If there is no read in pending,
 *   then this function will initiate the next OUT (read) operation.
 *
 *******************************************************************************/

static void stm32_epout_request(FAR struct stm32_usbdev_s *priv,
                                FAR struct stm32_ep_s *privep)
{
  struct stm32_req_s *privreq;
  uint32_t regaddr;
  uint32_t regval;
  uint32_t xfrsize;
  uint32_t pktcnt;

  /* Make sure that there is not already a pending request request.  If there is,
   * just return, leaving the newly received request in the request queue.
   */

  if (!privep->active)
    {
      /* Loop until a valid request is found (or the request queue is empty).
       * The loop is only need to look at the request queue again is an invalid
       * read request is encountered.
       */

      for (;;)
        {
          /* Get a reference to the request at the head of the endpoint's request queue */

          privreq = stm32_rqpeek(privep);
          if (!privreq)
            {
              usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EPOUTQEMPTY), privep->epphy);

              /* There are no read requests to be setup.  Configure the hardware to
               * NAK any incoming packets.  (This should already be the case.  I
               * think that the hardware will automatically NAK after a transfer is
               * completed until SNAK is cleared).
               */

              regaddr = STM32_OTGFS_DOEPCTL(privep->epphy);
              regval  = stm32_getreg(regaddr);
              regval |= OTGFS_DOEPCTL_SNAK;
              stm32_putreg(regval, regaddr);

              /* This endpoint is no longer actively transferring */

              privep->active = false;
              return;
            }

          ullvdbg("EP%d: len=%d\n", privep->epphy, privreq->req.len);

          /* Ignore any attempt to receive a zero length packet (this really
           * should not happen.
           */

          if (privreq->req.len <= 0)
            {
              usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EPOUTNULLPACKET), 0);
              stm32_req_complete(privep, OK);
            }

          /* Otherwise, we have a usable read request... break out of the loop */

          else
            {
              break;
            }
        }

      /* Setup the pending read into the request buffer.  First calculate:
       *
       * pktcnt  = the number of packets (of maxpacket bytes) required to
       *   perform the transfer.
       * xfrsize = The total number of bytes required (in units of
       *   maxpacket bytes).
       */

      pktcnt  = (privreq->req.len + (privep->ep.maxpacket - 1)) / privep->ep.maxpacket;
      xfrsize = pktcnt * privep->ep.maxpacket;

      /* Then setup the hardware to perform this transfer */

      regaddr = STM32_OTGFS_DOEPTSIZ(privep->epphy);
      regval  = stm32_getreg(regaddr);
      regval &= ~(OTGFS_DOEPTSIZ_XFRSIZ_MASK | OTGFS_DOEPTSIZ_PKTCNT_MASK);
      regval |= (xfrsize << OTGFS_DOEPTSIZ_XFRSIZ_SHIFT);
      regval |= (pktcnt  << OTGFS_DOEPTSIZ_PKTCNT_SHIFT);
      stm32_putreg(regval, regaddr);

      /* Then enable the transfer */

      regaddr = STM32_OTGFS_DOEPCTL(privep->epphy);
      regval  = stm32_getreg(regaddr);

      /* When an isochronous transfer is enabled the Even/Odd frame bit must
       * also be set appropriately.
       */

#ifdef CONFIG_USBDEV_ISOCHRONOUS
      if (privep->eptype == USB_EP_ATTR_XFER_ISOC)
        {
          if (privep->odd)
            {
              regval |= OTGFS_DOEPCTL_SODDFRM;
            }
          else
            {
              regval |= OTGFS_DOEPCTL_SEVNFRM;
            }
        }
#endif

      /* Clearing NAKing and enable the transfer. */

      regval |= (OTGFS_DOEPCTL_CNAK | OTGFS_DOEPCTL_EPENA);
      stm32_putreg(regval, regaddr);

      /* A transfer is now active on this endpoint */

      privep->active = true;

      /* EP0 is a special case.  We need to know when to switch back to
       * normal SETUP processing.
       */

      if (privep->epphy == EP0)
        {
          priv->ep0state = EP0STATE_DATA_OUT;
        }
    }
}

/*******************************************************************************
 * Name: stm32_ep_flush
 *
 * Description:
 *   Flush any primed descriptors from this ep
 *
 *******************************************************************************/

static void stm32_ep_flush(struct stm32_ep_s *privep)
{
  if (privep->isin)
    {
      stm32_txfifo_flush(OTGFS_GRSTCTL_TXFNUM_D(privep->epphy));
    }
  else
    {
      stm32_rxfifo_flush();
    }
}

/*******************************************************************************
 * Name: stm32_req_complete
 *
 * Description:
 *   Handle termination of the request at the head of the endpoint request queue.
 *
 *******************************************************************************/

static void stm32_req_complete(struct stm32_ep_s *privep, int16_t result)
{
  FAR struct stm32_req_s *privreq;

  /* Remove the request at the head of the request list */

  privreq = stm32_req_remfirst(privep);
  DEBUGASSERT(privreq != NULL);

  /* If endpoint 0, temporarily reflect the state of protocol stalled
   * in the callback.
   */

  bool stalled = privep->stalled;
  if (privep->epphy == EP0)
    {
      privep->stalled = privep->dev->stalled;
    }

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);

  /* Restore the stalled indication */

  privep->stalled = stalled;
}

/*******************************************************************************
 * Name: stm32_req_cancel
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 *******************************************************************************/

static void stm32_req_cancel(struct stm32_ep_s *privep, int16_t status)
{
  if (!stm32_rqempty(privep))
    {
      stm32_ep_flush(privep);
    }

  while (!stm32_rqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(privep->epphy),
               (stm32_rqpeek(privep))->req.xfrd);
      stm32_req_complete(privep, status);
    }
}

/*******************************************************************************
 * Name: stm32_ep_findbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 *******************************************************************************/

static struct stm32_ep_s *stm32_ep_findbyaddr(struct stm32_usbdev_s *priv,
                                              uint16_t eplog)
{
  struct stm32_ep_s *privep;
  uint8_t epphy = USB_EPNO(eplog);

  if (epphy >= STM32_NENDPOINTS)
    {
      return NULL;
    }

  /* Is this an IN or an OUT endpoint? */

  if (USB_ISEPIN(eplog))
    {
      privep = &priv->epin[epphy];
    }
  else
    {
      privep = &priv->epout[epphy];
    }

  /* Verify that the endpoint has been configured */

  if (!privep->configured)
    {
      return NULL;
    }

  /* Return endpoint reference */

  DEBUGASSERT(privep->epphy == epphy);
  return privep;
}

/*******************************************************************************
 * Name: stm32_req_dispatch
 *
 * Description:
 *   Provide unhandled setup actions to the class driver. This is logically part
 *   of the USB interrupt handler.
 *
 *******************************************************************************/

static int stm32_req_dispatch(struct stm32_usbdev_s *priv,
                              const struct usb_ctrlreq_s *ctrl)
{
  int ret = -EIO;

  usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_DISPATCH), 0);
  if (priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, ctrl,
                        priv->ep0data, priv->ep0datlen);
    }

  if (ret < 0)
    {
      /* Stall on failure */

      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_DISPATCHSTALL), 0);
      priv->stalled = true;
    }

  return ret;
}

/*******************************************************************************
 * Name: stm32_usbreset
 *
 * Description:
 *   Reset Usb engine
 *
 *******************************************************************************/

static void stm32_usbreset(struct stm32_usbdev_s *priv)
{
  FAR struct stm32_ep_s *privep;
  uint32_t regval;
  int i;

  /* Clear the Remote Wake-up Signaling */

  regval = stm32_getreg(STM32_OTGFS_DCTL);
  regval &= ~OTGFS_DCTL_RWUSIG;
  stm32_putreg(regval, STM32_OTGFS_DCTL);

  /* Flush the EP0 Tx FIFO */

  stm32_txfifo_flush(OTGFS_GRSTCTL_TXFNUM_D(EP0));

  /* Tell the class driver that we are disconnected. The class
   * driver should then accept any new configurations.
   */

  if (priv->driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }
  priv->epavail = STM32_EP_AVAILABLE;

  /* Disable all end point interrupts */

  for (i = 0; i < STM32_NENDPOINTS ; i++)
    {
      /* Disable endpoint interrupts */

      stm32_putreg(0xff, STM32_OTGFS_DIEPINT(i));
      stm32_putreg(0xff, STM32_OTGFS_DOEPINT(i));

      /* Return write requests to the class implementation */

      privep = &priv->epin[i];
      stm32_req_cancel(privep, -ESHUTDOWN);

      /* Reset IN endpoint status */

      privep->stalled = false;

      /* Return read requests to the class implementation */

      privep = &priv->epout[i];
      stm32_req_cancel(privep, -ESHUTDOWN);

      /* Reset endpoint status */

      privep->stalled = false;
    }

  stm32_putreg(0xffffffff, STM32_OTGFS_DAINT);

  /* Mask all device endpoint interrupts except EP0 */

  regval = (OTGFS_DAINT_IEP(EP0) | OTGFS_DAINT_OEP(EP0));
  stm32_putreg(regval, STM32_OTGFS_DAINTMSK);

  /* Unmask OUT interrupts */

  regval = (OTGFS_DOEPMSK_XFRCM | OTGFS_DOEPMSK_STUPM | OTGFS_DOEPMSK_EPDM);
  stm32_putreg(regval, STM32_OTGFS_DOEPMSK);

  /* Unmask IN interrupts */

  regval = (OTGFS_DIEPMSK_XFRCM | OTGFS_DIEPMSK_EPDM | OTGFS_DIEPMSK_TOM);
  stm32_putreg(regval, STM32_OTGFS_DIEPMSK);

  /* Reset device address to 0 */

  stm32_setaddress(priv, 0);
  priv->devstate = DEVSTATE_DEFAULT;
  priv->usbdev.speed = USB_SPEED_FULL;

  /* Re-configure EP0 */

  stm32_ep0_configure(priv);

  /* Setup EP0 to receive SETUP packets */

  stm32_ep0out_ctrlsetup(priv);
}

/*******************************************************************************
 * Name: stm32_ep0out_testmode
 *
 * Description:
 *   Select test mode
 *
 *******************************************************************************/

static inline void stm32_ep0out_testmode(FAR struct stm32_usbdev_s *priv,
                                         uint16_t index)
{
  uint32_t regval;
  uint8_t testmode;

  regval = stm32_getreg(STM32_OTGFS_DCTL);

  testmode = index >> 8;
  switch (testmode)
    {
    case 1:
      priv->testmode = OTGFS_TESTMODE_J;
      break;

    case 2:
      priv->testmode = OTGFS_TESTMODE_K;
      break;

    case 3:
      priv->testmode = OTGFS_TESTMODE_SE0_NAK;
      break;

    case 4:
      priv->testmode = OTGFS_TESTMODE_PACKET;
      break;

    case 5:
      priv->testmode = OTGFS_TESTMODE_FORCE;
      break;

    default:
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADTESTMODE), testmode);
      priv->dotest   = false;
      priv->testmode = OTGFS_TESTMODE_DISABLED;
      priv->stalled  = true;
    }

  priv->dotest = true;
  stm32_ep0in_transmitzlp(priv);
}

/*******************************************************************************
 * Name: stm32_ep0out_stdrequest
 *
 * Description:
 *   Handle a stanard request on EP0.  Pick off the things of interest to the
 *   USB device controller driver; pass what is left to the class driver.
 *
 *******************************************************************************/

static inline void stm32_ep0out_stdrequest(struct stm32_usbdev_s *priv,
                                           FAR struct stm32_ctrlreq_s *ctrlreq)
{
  FAR struct stm32_ep_s *privep;

  /* Handle standard request */

  switch (ctrlreq->req)
    {
    case USB_REQ_GETSTATUS:
      {
        /* type:  device-to-host; recipient = device, interface, endpoint
         * value: 0
         * index: zero interface endpoint
         * len:   2; data = status
         */

        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_GETSTATUS), 0);
        if (!priv->addressed ||
             ctrlreq->len != 2 ||
            USB_REQ_ISOUT(ctrlreq->type) ||
            ctrlreq->value != 0)
          {
            priv->stalled = true;
          }
        else
          {
            switch (ctrlreq->type & USB_REQ_RECIPIENT_MASK)
              {
              case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPGETSTATUS), 0);
                  privep = stm32_ep_findbyaddr(priv, ctrlreq->index);
                  if (!privep)
                    {
                      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADEPGETSTATUS), 0);
                      priv->stalled = true;
                    }
                  else
                    {
                      if (privep->stalled)
                        {
                          priv->ep0data[0] = (1 << USB_FEATURE_ENDPOINTHALT);
                        }
                      else
                        {
                          priv->ep0data[0] = 0; /* Not stalled */
                        }

                      priv->ep0data[1] = 0;
                      stm32_ep0in_setupresponse(priv, priv->ep0data, 2);
                    }
                }
                break;

              case USB_REQ_RECIPIENT_DEVICE:
                {
                  if (ctrlreq->index == 0)
                    {
                      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_DEVGETSTATUS), 0);

                      /* Features:  Remote Wakeup and selfpowered */

                      priv->ep0data[0]  = (priv->selfpowered << USB_FEATURE_SELFPOWERED);
                      priv->ep0data[0] |= (priv->wakeup      << USB_FEATURE_REMOTEWAKEUP);
                      priv->ep0data[1]  = 0;

                      stm32_ep0in_setupresponse(priv, priv->ep0data, 2);
                    }
                  else
                    {
                      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADDEVGETSTATUS), 0);
                      priv->stalled = true;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_INTERFACE:
                {
                  usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_IFGETSTATUS), 0);
                  priv->ep0data[0] = 0;
                  priv->ep0data[1] = 0;

                  stm32_ep0in_setupresponse(priv, priv->ep0data, 2);
                }
                break;

              default:
                {
                  usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADGETSTATUS), 0);
                  priv->stalled = true;
                }
                break;
              }
          }
      }
      break;

    case USB_REQ_CLEARFEATURE:
      {
        /* type:  host-to-device; recipient = device, interface or endpoint
         * value: feature selector
         * index: zero interface endpoint;
         * len:   zero, data = none
         */

        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_CLEARFEATURE), 0);
        if (priv->addressed != 0 && ctrlreq->len == 0)
          {
            uint8_t recipient = ctrlreq->type & USB_REQ_RECIPIENT_MASK;
            if (recipient == USB_REQ_RECIPIENT_ENDPOINT &&
                ctrlreq->value == USB_FEATURE_ENDPOINTHALT &&
                (privep = stm32_ep_findbyaddr(priv, ctrlreq->index)) != NULL)
              {
                stm32_ep_clrstall(privep);
                stm32_ep0in_transmitzlp(priv);
              }
            else if (recipient == USB_REQ_RECIPIENT_DEVICE &&
                     ctrlreq->value == USB_FEATURE_REMOTEWAKEUP)
              {
                priv->wakeup = 0;
                stm32_ep0in_transmitzlp(priv);
              }
            else
              {
                /* Actually, I think we could just stall here. */

                (void)stm32_req_dispatch(priv, &priv->ctrlreq);
              }
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADCLEARFEATURE), 0);
            priv->stalled = true;
          }
      }
      break;

    case USB_REQ_SETFEATURE:
      {
        /* type:  host-to-device; recipient = device, interface, endpoint
         * value: feature selector
         * index: zero interface endpoint;
         * len:   0; data = none
         */

        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SETFEATURE), 0);
        if (priv->addressed != 0 && ctrlreq->len == 0)
          {
            uint8_t recipient = ctrlreq->type & USB_REQ_RECIPIENT_MASK;
            if (recipient == USB_REQ_RECIPIENT_ENDPOINT &&
                ctrlreq->value == USB_FEATURE_ENDPOINTHALT &&
                (privep = stm32_ep_findbyaddr(priv, ctrlreq->index)) != NULL)
              {
                stm32_ep_setstall(privep);
                stm32_ep0in_transmitzlp(priv);
              }
            else if (recipient == USB_REQ_RECIPIENT_DEVICE &&
                     ctrlreq->value == USB_FEATURE_REMOTEWAKEUP)
              {
                priv->wakeup = 1;
                stm32_ep0in_transmitzlp(priv);
              }
            else if (recipient == USB_REQ_RECIPIENT_DEVICE &&
                     ctrlreq->value == USB_FEATURE_TESTMODE &&
                     ((ctrlreq->index & 0xff) == 0))
              {
                 stm32_ep0out_testmode(priv, ctrlreq->index);
              }
            else if (priv->configured)
              {
                /* Actually, I think we could just stall here. */

                (void)stm32_req_dispatch(priv, &priv->ctrlreq);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADSETFEATURE), 0);
                priv->stalled = true;
              }
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADSETFEATURE), 0);
            priv->stalled = true;
          }
      }
      break;

    case USB_REQ_SETADDRESS:
      {
        /* type:  host-to-device; recipient = device
         * value: device address
         * index: 0
         * len:   0; data = none
         */

        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SETADDRESS), ctrlreq->value);
        if ((ctrlreq->type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            ctrlreq->index  == 0 &&
            ctrlreq->len == 0 &&
            ctrlreq->value < 128 &&
            priv->devstate != DEVSTATE_CONFIGURED)
          {
            /* Save the address.  We cannot actually change to the next address until
             * the completion of the status phase.
             */

            stm32_setaddress(priv, (uint16_t)priv->ctrlreq.value[0]);
            stm32_ep0in_transmitzlp(priv);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADSETADDRESS), 0);
            priv->stalled = true;
          }
      }
      break;

    case USB_REQ_GETDESCRIPTOR:
      /* type:  device-to-host; recipient = device
       * value: descriptor type and index
       * index: 0 or language ID;
       * len:   descriptor len; data = descriptor
       */

    case USB_REQ_SETDESCRIPTOR:
      /* type:  host-to-device; recipient = device
       * value: descriptor type and index
       * index: 0 or language ID;
       * len:   descriptor len; data = descriptor
       */

      {
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_GETSETDESC), 0);
        if ((ctrlreq->type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
          {
            (void)stm32_req_dispatch(priv, &priv->ctrlreq);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADGETSETDESC), 0);
            priv->stalled = true;
          }
      }
      break;

    case USB_REQ_GETCONFIGURATION:
      /* type:  device-to-host; recipient = device
       * value: 0;
       * index: 0;
       * len:   1; data = configuration value
       */

      {
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_GETCONFIG), 0);
        if (priv->addressed &&
           (ctrlreq->type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            ctrlreq->value == 0 &&
            ctrlreq->index == 0 &&
            ctrlreq->len == 1)
          {
            (void)stm32_req_dispatch(priv, &priv->ctrlreq);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADGETCONFIG), 0);
            priv->stalled = true;
          }
      }
      break;

    case USB_REQ_SETCONFIGURATION:
      /* type:  host-to-device; recipient = device
       * value: configuration value
       * index: 0;
       * len:   0; data = none
       */

      {
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SETCONFIG), 0);
        if (priv->addressed &&
            (ctrlreq->type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            ctrlreq->index == 0 &&
            ctrlreq->len == 0)
          {
            /* Give the configuration to the class driver */

            int ret = stm32_req_dispatch(priv, &priv->ctrlreq);

            /* If the class driver accepted the configuration, then mark the
             * device state as configured (or not, depending on the
             * configuration).
             */

            if (ret == OK)
              {
                uint8_t cfg = (uint8_t)ctrlreq->value;
                if (cfg != 0)
                  {
                    priv->devstate   = DEVSTATE_CONFIGURED;
                    priv->configured = true;
                  }
                else
                  {
                    priv->devstate   = DEVSTATE_ADDRESSED;
                    priv->configured = false;
                  }
              }
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADSETCONFIG), 0);
            priv->stalled = true;
          }
      }
      break;

    case USB_REQ_GETINTERFACE:
      /* type:  device-to-host; recipient = interface
       * value: 0
       * index: interface;
       * len:   1; data = alt interface
       */

    case USB_REQ_SETINTERFACE:
      /* type:  host-to-device; recipient = interface
       * value: alternate setting
       * index: interface;
       * len:   0; data = none
       */

      {
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_GETSETIF), 0);
        (void)stm32_req_dispatch(priv, &priv->ctrlreq);
      }
      break;

    case USB_REQ_SYNCHFRAME:
      /* type:  device-to-host; recipient = endpoint
       * value: 0
       * index: endpoint;
       * len:   2; data = frame number
       */

      {
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SYNCHFRAME), 0);
      }
      break;

    default:
      {
        usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDCTRLREQ), 0);
        priv->stalled = true;
      }
      break;
    }
}

/*******************************************************************************
 * Name: stm32_ep0out_setup
 *
 * Description:
 *   USB Ctrl EP Setup Event. This is logically part of the USB interrupt
 *   handler.  This event occurs when a setup packet is receive on EP0 OUT.
 *
 *******************************************************************************/

static inline void stm32_ep0out_setup(struct stm32_usbdev_s *priv)
{
  struct stm32_ctrlreq_s ctrlreq;

  /* Verify that a SETUP was received */

  if (priv->ep0state != EP0STATE_SETUP_READY)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EP0NOSETUP), priv->ep0state);
      return;
    }

  /* Terminate any pending requests */

  stm32_req_cancel(&priv->epout[EP0], -EPROTO);
  stm32_req_cancel(&priv->epin[EP0],  -EPROTO);

  /* Assume NOT stalled */

  priv->epout[EP0].stalled = false;
  priv->epin[EP0].stalled = false;
  priv->stalled = false;

  /* Starting to process a control request - update state */

  priv->ep0state = EP0STATE_SETUP_PROCESS;

  /* And extract the little-endian 16-bit values to host order */

  ctrlreq.type  = priv->ctrlreq.type;
  ctrlreq.req   = priv->ctrlreq.req;
  ctrlreq.value = GETUINT16(priv->ctrlreq.value);
  ctrlreq.index = GETUINT16(priv->ctrlreq.index);
  ctrlreq.len   = GETUINT16(priv->ctrlreq.len);

  ullvdbg("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
          ctrlreq.type, ctrlreq.req, ctrlreq.value, ctrlreq.index, ctrlreq.len);

  /* Check for a standard request */

  if ((ctrlreq.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      /* Dispatch any non-standard requests */

      (void)stm32_req_dispatch(priv, &priv->ctrlreq);
    }
  else
    {
      /* Handle standard requests. */

      stm32_ep0out_stdrequest(priv, &ctrlreq);
    }

  /* Check if the setup processing resulted in a STALL */

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EP0SETUPSTALLED), priv->ep0state);
      stm32_ep0_stall(priv);
    }

  /* Reset state/data associated with thie SETUP request */

   priv->ep0datlen = 0;
}

/*******************************************************************************
 * Name: stm32_epout
 *
 * Description:
 *   This is part of the OUT endpoint interrupt processing.  This function
 *   handles the OUT event for a single endpoint.
 *
 *******************************************************************************/

static inline void stm32_epout(FAR struct stm32_usbdev_s *priv, uint8_t epno)
{
  FAR struct stm32_ep_s *privep;

  /* Endpoint 0 is a special case. */

  if (epno == 0)
    {
      privep = &priv->epout[EP0];

      /* In the EP0STATE_DATA_OUT state, we are receiving data into the
       * request buffer.  In that case, we must continue the request
       * processing.
       */

      if (priv->ep0state == EP0STATE_DATA_OUT)
        {
          /* Continue processing data from the EP0 OUT request queue */

          stm32_epout_complete(priv, privep);
        }

      /* If we are not actively processing an OUT request, then we
       * need to setup to receive the next control request.
       */

      if (!privep->active)
        {
          stm32_ep0out_ctrlsetup(priv);
          priv->ep0state = EP0STATE_IDLE;
        }
    }

  /* For other endpoints, the only possibility is that we are continuing
   * or finishing an OUT request.
   */

  else if (priv->devstate == DEVSTATE_CONFIGURED)
    {
      stm32_epout_complete(priv, &priv->epout[epno]);
    }
}

/*******************************************************************************
 * Name: stm32_epout_interrupt
 *
 * Description:
 *   USB OUT endpoint interrupt handler.  The core generates this interrupt when
 *   there is an interrupt is pending on one of the OUT endpoints of the core.
 *   The driver must read the OTGFS DAINT register to determine the exact number
 *   of the OUT endpoint on which the interrupt occurred, and then read the
 *   corresponding OTGFS DOEPINTx register to determine the exact cause of the
 *   interrupt.
 *
 *******************************************************************************/

static inline void stm32_epout_interrupt(FAR struct stm32_usbdev_s *priv)
{
  uint32_t daint;
  uint32_t regval;
  uint32_t doepint;
  int epno;

  /* Get the pending, enabled interrupts for the OUT endpoint from the endpoint
   * interrupt status register.
   */

  regval  = stm32_getreg(STM32_OTGFS_DAINT);
  regval &= stm32_getreg(STM32_OTGFS_DAINTMSK);
  daint   = (regval & OTGFS_DAINT_OEP_MASK) >> OTGFS_DAINT_OEP_SHIFT;

  /* Process each pending IN endpoint interrupt */

  epno = 0;
  while (daint)
    {
      /* Is an OUT interrupt pending for this endpoint? */

      if ((daint & 1) != 0)
        {
          /* Yes.. get the OUT endpoint interrupt status */

          doepint  = stm32_getreg(STM32_OTGFS_DOEPINT(epno));
          doepint &= stm32_getreg(STM32_OTGFS_DOEPMSK);

          /* Transfer completed interrupt.  This interrupt is trigged when
           * stm32_rxinterrupt() removes the last packet data from the RxFIFO.
           * In this case, core internally sets the NAK bit for this endpoint to
           * prevent it from receiving any more packets.
           */

          if ((doepint & OTGFS_DOEPINT_XFRC) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPOUT_XFRC), (uint16_t)doepint);

              /* Clear the bit in DOEPINTn for this interrupt */

              stm32_putreg(OTGFS_DOEPINT_XFRC, STM32_OTGFS_DOEPINT(epno));

              /* Handle the RX transfer data ready event */

              stm32_epout(priv, epno);
            }

          /* Endpoint disabled interrupt (ignored because this interrrupt is
           * used in polled mode by the endpoint disable logic).
           */
#if 1
          /* REVISIT: */
          if ((doepint & OTGFS_DOEPINT_EPDISD) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPOUT_EPDISD), (uint16_t)doepint);

              /* Clear the bit in DOEPINTn for this interrupt */

              stm32_putreg(OTGFS_DOEPINT_EPDISD, STM32_OTGFS_DOEPINT(epno));
            }
#endif
          /* Setup Phase Done (control EPs) */

          if ((doepint & OTGFS_DOEPINT_SETUP) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPOUT_SETUP), priv->ep0state);

              /* Handle the receipt of the IN SETUP packets now (OUT setup
               * packet processing may be delayed until the accompanying
               * OUT DATA is received)
               */

              if (priv->ep0state == EP0STATE_SETUP_READY)
                {
                  stm32_ep0out_setup(priv);
                }
              stm32_putreg(OTGFS_DOEPINT_SETUP, STM32_OTGFS_DOEPINT(epno));
            }
        }

      epno++;
      daint >>= 1;
    }
}

/*******************************************************************************
 * Name: stm32_epin_runtestmode
 *
 * Description:
 *   Execute the test mode setup by the SET FEATURE request
 *
 *******************************************************************************/

static inline void stm32_epin_runtestmode(FAR struct stm32_usbdev_s *priv)
{
  uint32_t regval = stm32_getreg(STM32_OTGFS_DCTL);
  regval &= OTGFS_DCTL_TCTL_MASK;
  regval |= (uint32_t)priv->testmode << OTGFS_DCTL_TCTL_SHIFT;
  stm32_putreg(regval , STM32_OTGFS_DCTL);

  priv->dotest = 0;
  priv->testmode = OTGFS_TESTMODE_DISABLED;
}

/*******************************************************************************
 * Name: stm32_epin
 *
 * Description:
 *   This is part of the IN endpoint interrupt processing.  This function
 *   handles the IN event for a single endpoint.
 *
 *******************************************************************************/

static inline void stm32_epin(FAR struct stm32_usbdev_s *priv, uint8_t epno)
{
  FAR struct stm32_ep_s *privep = &priv->epin[epno];

  /* Endpoint 0 is a special case. */

  if (epno == 0)
    {
      /* In the EP0STATE_DATA_IN state, we are sending data from request
       * buffer.  In that case, we must continue the request processing.
       */

      if (priv->ep0state == EP0STATE_DATA_IN)
        {
          /* Continue processing data from the EP0 OUT request queue */

          stm32_epin_request(priv, privep);
        }

      /* If we are not actively processing an OUT request, then we
       * need to setup to receive the next control request.
       */

      if (!privep->active)
        {
          stm32_ep0out_ctrlsetup(priv);
          priv->ep0state = EP0STATE_IDLE;
        }

      /* Test mode is another special case */

      if (priv->dotest)
        {
          stm32_epin_runtestmode(priv);
        }
    }

  /* For other endpoints, the only possibility is that we are continuing
   * or finishing an IN request.
   */

  else if (priv->devstate == DEVSTATE_CONFIGURED)
    {
      /* Continue processing data from the endpoint write request queue */

      stm32_epin_request(priv, privep);
    }
}

/****************************************************************************
 * Name: stm32_epin_txfifoempty
 *
 * Description:
 *   TxFIFO empty interrupt handling
 *
 ****************************************************************************/

static inline void stm32_epin_txfifoempty(FAR struct stm32_usbdev_s *priv, int epno)
{
  FAR struct stm32_ep_s *privep = &priv->epin[epno];

  /* Continue processing the write request queue.  This may mean sending
   * more data from the exisiting request or terminating the current requests
   * and (perhaps) starting the IN transfer from the next write request.
   */

  stm32_epin_request(priv, privep);
}

/*******************************************************************************
 * Name: stm32_epin_interrupt
 *
 * Description:
 *   USB IN endpoint interrupt handler.  The core generates this interrupt when
 *   an interrupt is pending on one of the IN endpoints of the core. The driver
 *   must read the OTGFS DAINT register to determine the exact number of the IN
 *   endpoint on which the interrupt occurred, and then read the corresponding
 *   OTGFS DIEPINTx register to determine the exact cause of the interrupt.
 *
 *******************************************************************************/

static inline void stm32_epin_interrupt(FAR struct stm32_usbdev_s *priv)
{
  uint32_t diepint;
  uint32_t daint;
  uint32_t mask;
  uint32_t empty;
  int epno;

  /* Get the pending, enabled interrupts for the IN endpoint from the endpoint
   * interrupt status register.
   */

  daint  = stm32_getreg(STM32_OTGFS_DAINT);
  daint &= stm32_getreg(STM32_OTGFS_DAINTMSK);
  daint &= OTGFS_DAINT_IEP_MASK;

  /* Process each pending IN endpoint interrupt */

  epno = 0;
  while (daint)
    {
      /* Is an IN interrupt pending for this endpoint? */

      if ((daint & 1) != 0)
        {
          /* Get IN interrupt mask register.  Bits 0-6 correspond to enabled
           * interrupts as will be found in the DIEPINT interrupt status
           * register.
           */

          mask = stm32_getreg(STM32_OTGFS_DIEPMSK);

          /* Check if the TxFIFO not empty interrupt is enabled for this
           * endpoint in the DIEPMSK register.  Bits n corresponds to
           * endpoint n in the register. That condition corresponds to
           * bit 7 of the DIEPINT interrupt status register.  There is
           * no TXFE bit in the mask register, so we fake one here.
           */

          empty = stm32_getreg(STM32_OTGFS_DIEPEMPMSK);
          if ((empty & OTGFS_DIEPEMPMSK(epno)) != 0)
            {
              mask |= OTGFS_DIEPINT_TXFE;
            }

          /* Now, read the interrupt status and mask out all disabled
           * interrupts.
           */

          diepint = stm32_getreg(STM32_OTGFS_DIEPINT(epno)) & mask;

          /* Decode and process the enabled, pending interrupts */
          /* Transfer completed interrupt */

          if ((diepint & OTGFS_DIEPINT_XFRC) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN_XFRC),
                       (uint16_t)diepint);

              /* It is possible that logic may be waiting for a the
               * TxFIFO to become empty.  We disable the TxFIFO empty
               * interrupt here; it will be re-enabled if there is still
               * insufficient space in the TxFIFO.
               */
               
              empty &= ~OTGFS_DIEPEMPMSK(epno);
              stm32_putreg(empty, STM32_OTGFS_DIEPEMPMSK);
              stm32_putreg(OTGFS_DIEPINT_XFRC, STM32_OTGFS_DIEPINT(epno));

              /* IN transfer complete */

              stm32_epin(priv, epno);
            }

          /* Timeout condition */

          if ((diepint & OTGFS_DIEPINT_TOC) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN_TOC), (uint16_t)diepint);
              stm32_putreg(OTGFS_DIEPINT_TOC, STM32_OTGFS_DIEPINT(epno));
            }

          /* IN token received when TxFIFO is empty.  Applies to non-periodic IN
           * endpoints only.  This interrupt indicates that an IN token was received
           * when the associated TxFIFO (periodic/non-periodic) was empty. This
           * interrupt is asserted on the endpoint for which the IN token was
           * received.
           */

          if ((diepint & OTGFS_DIEPINT_ITTXFE) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN_ITTXFE), (uint16_t)diepint);
              stm32_epin_request(priv, &priv->epin[epno]);
              stm32_putreg(OTGFS_DIEPINT_ITTXFE, STM32_OTGFS_DIEPINT(epno));
            }

          /* IN endpoint NAK effective (ignored as this used only in polled
           * mode)
           */
#if 0
          if ((diepint & OTGFS_DIEPINT_INEPNE) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN_INEPNE), (uint16_t)diepint);
              stm32_putreg(OTGFS_DIEPINT_INEPNE, STM32_OTGFS_DIEPINT(epno));
            }
#endif
          /* Endpoint disabled interrupt (ignored as this used only in polled
           * mode)
           */
#if 0
          if ((diepint & OTGFS_DIEPINT_EPDISD) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN_EPDISD), (uint16_t)diepint);
              stm32_putreg(OTGFS_DIEPINT_EPDISD, STM32_OTGFS_DIEPINT(epno));
            }
#endif
          /* Transmit FIFO empty */

          if ((diepint & OTGFS_DIEPINT_TXFE) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN_TXFE), (uint16_t)diepint);

              /* If we were waiting for TxFIFO to become empty, the we might have both
               * XFRC and TXFE interrupts pending.  Since we do the same thing for both
               * cases, ignore the TXFE if we have already processed the XFRC.
               */

              if ((diepint & OTGFS_DIEPINT_XFRC) == 0)
                {
                  /* Mask further FIFO empty interrupts.  This will be re-enabled
                   * whenever we need to wait for a FIFO event.
                   */

                  empty &= ~OTGFS_DIEPEMPMSK(epno);
                  stm32_putreg(empty, STM32_OTGFS_DIEPEMPMSK);

                  /* Handle TxFIFO empty */

                  stm32_epin_txfifoempty(priv, epno);
                }

              /* Clear the pending TxFIFO empty interrupt */

              stm32_putreg(OTGFS_DIEPINT_TXFE, STM32_OTGFS_DIEPINT(epno));
            }
        }

      epno++;
      daint >>= 1;
    }
}

/*******************************************************************************
 * Name: stm32_resumeinterrupt
 *
 * Description:
 *   Resume/remote wakeup detected interrupt
 *
 *******************************************************************************/

static inline void stm32_resumeinterrupt(FAR struct stm32_usbdev_s *priv)
{
  uint32_t regval;

  /* Restart the PHY clock and un-gate USB core clock (HCLK) */

#ifdef CONFIG_USBDEV_LOWPOWER
  regval = stm32_getreg(STM32_OTGFS_PCGCCTL);
  regval &= ~(OTGFS_PCGCCTL_STPPCLK | OTGFS_PCGCCTL_GATEHCLK);
  stm32_putreg(regval, STM32_OTGFS_PCGCCTL);
#endif

  /* Clear remote wake-up signaling */

  regval  = stm32_getreg(STM32_OTGFS_DCTL);
  regval &= ~OTGFS_DCTL_RWUSIG;
  stm32_putreg(regval, STM32_OTGFS_DCTL);

  /* Restore full power -- whatever that means for this particular board */

  stm32_usbsuspend((struct usbdev_s *)priv, true);

  /* Notify the class driver of the resume event */

  if (priv->driver)
    {
      CLASS_RESUME(priv->driver, &priv->usbdev);
    }
}

/*******************************************************************************
 * Name: stm32_suspendinterrupt
 *
 * Description:
 *   USB suspend interrupt
 *
 *******************************************************************************/

static inline void stm32_suspendinterrupt(FAR struct stm32_usbdev_s *priv)
{
#ifdef CONFIG_USBDEV_LOWPOWER
  uint32_t regval;
#endif

  /* Notify the class driver of the suspend event */

  if (priv->driver)
    {
      CLASS_SUSPEND(priv->driver, &priv->usbdev);
    }

#ifdef CONFIG_USBDEV_LOWPOWER
  /* OTGFS_DSTS_SUSPSTS is set as long as the suspend condition is detected
   * on USB.  Check if we are still have the suspend condition, that we are
   * connected to the host, and that we have been configured.
   */

  regval = stm32_getreg(STM32_OTGFS_DSTS);

  if ((regval & OTGFS_DSTS_SUSPSTS) != 0 && devstate == DEVSTATE_CONFIGURED)
    {
      /* Switch off OTG FS clocking.  Setting OTGFS_PCGCCTL_STPPCLK stops the
       * PHY clock.
       */

      regval = stm32_getreg(STM32_OTGFS_PCGCCTL);
      regval |= OTGFS_PCGCCTL_STPPCLK;
      stm32_putreg(regval, STM32_OTGFS_PCGCCTL);

      /* Setting OTGFS_PCGCCTL_GATEHCLK gate HCLK to modules other than
       * the AHB Slave and Master and wakeup logic.
       */

      regval |= OTGFS_PCGCCTL_GATEHCLK;
      stm32_putreg(regval, STM32_OTGFS_PCGCCTL);
    }
#endif

  /* Let the board-specific logic know that we have entered the suspend
   * state
   */

  stm32_usbsuspend((FAR struct usbdev_s *)priv, false);
}

/*******************************************************************************
 * Name: stm32_rxinterrupt
 *
 * Description:
 *   RxFIFO non-empty interrupt.  This interrupt indicates that there is at
 *   least one packet pending to be read from the RxFIFO.
 *
 *******************************************************************************/

static inline void stm32_rxinterrupt(FAR struct stm32_usbdev_s *priv)
{
  FAR struct stm32_ep_s *privep;
  uint32_t regval;
  int bcnt;
  int epphy;

  /* Disable the Rx status queue level interrupt */

  regval = stm32_getreg(STM32_OTGFS_GINTMSK);
  regval &= ~OTGFS_GINT_RXFLVL;
  stm32_putreg(regval, STM32_OTGFS_GINTMSK);

  /* Get the status from the top of the FIFO */

  regval = stm32_getreg(STM32_OTGFS_GRXSTSP);

  /* Decode status fields */

  epphy  = (regval & OTGFS_GRXSTSD_EPNUM_MASK) >> OTGFS_GRXSTSD_EPNUM_SHIFT;
  privep = &priv->epout[epphy];

  /* Handle the RX event according to the packet status field */

  switch (regval & OTGFS_GRXSTSD_PKTSTS_MASK)
    {
    /* Global OUT NAK.  This indicate that the global OUT NAK bit has taken
     * effect.
     *
     * PKTSTS = Global OUT NAK, BCNT = 0, EPNUM = Don't Care, DPID = Don't
     * Care.
     */

    case OTGFS_GRXSTSD_PKTSTS_OUTNAK:
      {
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_OUTNAK), 0);
      }
      break;

    /* OUT data packet received.
     *
     * PKTSTS = DataOUT, BCNT = size of the received data OUT packet,
     * EPNUM = EPNUM on which the packet was received, DPID = Actual Data PID.
     */

    case OTGFS_GRXSTSD_PKTSTS_OUTRECVD:
      {
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_OUTRECVD), epphy);
        bcnt = (regval & OTGFS_GRXSTSD_BCNT_MASK) >> OTGFS_GRXSTSD_BCNT_SHIFT;
        if (bcnt > 0)
          {
            stm32_epout_receive(privep, bcnt);
          }
      }
      break;

    /* OUT transfer completed.  This indicates that an OUT data transfer for
     * the specified OUT endpoint has completed. After this entry is popped
     * from the receive FIFO, the core asserts a Transfer Completed interrupt
     * on the specified OUT endpoint.
     *
     * PKTSTS = Data OUT Transfer Done, BCNT = 0, EPNUM = OUT EP Num on
     * which the data transfer is complete, DPID = Don't Care.
     */

    case OTGFS_GRXSTSD_PKTSTS_OUTDONE:
      {
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_OUTDONE), epphy);
      }
      break;

    /* SETUP transaction completed. This indicates that the Setup stage for
     * the specified endpoint has completed and the Data stage has started.
     * After this entry is popped from the receive FIFO, the core asserts a
     * Setup interrupt on the specified control OUT endpoint (triggers an
     * interrupt).
     *
     * PKTSTS = Setup Stage Done, BCNT = 0, EPNUM = Control EP Num,
     * DPID = Don't Care.
     */

    case OTGFS_GRXSTSD_PKTSTS_SETUPDONE:
      {
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SETUPDONE), epphy);
      }
      break;

    /* SETUP data packet received.  This indicates that a SETUP packet for the
     * specified endpoint is now available for reading from the receive FIFO.
     *
     * PKTSTS = SETUP, BCNT = 8, EPNUM = Control EP Num, DPID = D0.
     */

    case OTGFS_GRXSTSD_PKTSTS_SETUPRECVD:
      {
        uint16_t datlen;

        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SETUPRECVD), epphy);

        /* Read EP0 setup data.  NOTE:  If multiple SETUP packets are received,
         * the last one overwrites the previous setup packets and only that
         * last SETUP packet will be processed.
         */

        stm32_rxfifo_read(&priv->epout[EP0], (FAR uint8_t*)&priv->ctrlreq,
                         USB_SIZEOF_CTRLREQ);

        /* Was this an IN or an OUT SETUP packet.  If it is an OUT SETUP,
         * then we need to wait for the completion of the data phase to
         * process the setup command.  If it is an IN SETUP packet, then
         * we must processing the command BEFORE we enter the DATA phase.
         *
         * If the data associated with the OUT SETUP packet is zero length,
         * then, of course, we don't need to wait.
         */

        datlen = GETUINT16(priv->ctrlreq.len);
        if (USB_REQ_ISOUT(priv->ctrlreq.type) && datlen > 0)
          {
            /* Wait for the data phase. */

            priv->ep0state = EP0STATE_SETUP_OUT;
          }
        else
          {
            /* We can process the setup data as soon as SETUP done word is
             * popped of the RxFIFO.
             */

            priv->ep0state = EP0STATE_SETUP_READY;
          }
      }
      break;

    default:
      {
        usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS),
                 (regval & OTGFS_GRXSTSD_PKTSTS_MASK) >> OTGFS_GRXSTSD_PKTSTS_SHIFT);
      }
      break;
    }

  /* Enable the Rx Status Queue Level interrupt */

  regval  = stm32_getreg(STM32_OTGFS_GINTMSK);
  regval |= OTGFS_GINT_RXFLVL;
  stm32_putreg(regval, STM32_OTGFS_GINTMSK);
}

/*******************************************************************************
 * Name: stm32_enuminterrupt
 *
 * Description:
 *   Enumeration done interrupt
 *
 *******************************************************************************/

static inline void stm32_enuminterrupt(FAR struct stm32_usbdev_s *priv)
{
  uint32_t regval;

  /* Activate EP0 */

  stm32_ep0in_activate();

  /* Set USB turn-around time for the full speed device with internal PHY interface. */

  regval  = stm32_getreg(STM32_OTGFS_GUSBCFG);
  regval &= ~OTGFS_GUSBCFG_TRDT_MASK;
  regval |=  OTGFS_GUSBCFG_TRDT(5);
  stm32_putreg(regval, STM32_OTGFS_GUSBCFG);
}

/*******************************************************************************
 * Name: stm32_isocininterrupt
 *
 * Description:
 *   Incomplete isochronous IN transfer interrupt.  Assertion of the incomplete
 *   isochronous IN transfer interrupt indicates an incomplete isochronous IN
 *   transfer on at least one of the isochronous IN endpoints.
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_ISOCHRONOUS
static inline void stm32_isocininterrupt(FAR struct stm32_usbdev_s *priv)
{
  int i;

  /* The application must read the endpoint control register for all isochronous
   * IN endpoints to detect endpoints with incomplete IN data transfers.
   */

  for (i = 0; i < STM32_NENDPOINTS; i++)
    {
      /* Is this an isochronous IN endpoint? */

      privep = &priv->epin[i];
      if (privep->eptype != USB_EP_ATTR_XFER_ISOC)
        {
          /* No... keep looking */

          continue;
        }

      /* Is there an active read request on the isochronous OUT endpoint? */

      if (!privep->active)
        {
          /* No.. the endpoint is not actively transmitting data */

          continue;
        }

      /* Check if this is the endpoint that had the incomplete transfer */

      regaddr = STM32_OTGFS_DIEPCTL(privep->epphy);
      doepctl = stm32_getreg(regaddr);
      dsts    = stm32_getreg(STM32_OTGFS_DSTS);

      /* EONUM = 0:even frame, 1:odd frame
       * SOFFN = Frame number of the received SOF
       */

      eonum = ((doepctl & OTGFS_DIEPCTL_EONUM) != 0);
      soffn = ((dsts & OTGFS_DSTS_SOFFN0) != 0);

      if (eonum != soffn)
        {
          /* Not this endpoint */

          continue;
        }

      /* For isochronous IN endpoints with incomplete transfers,
       * the application must discard the data in the memory and
       * disable the endpoint.
       */

      stm32_req_complete(privep, -EIO);
#warning "Will clear OTGFS_DIEPCTL_USBAEP too"
      stm32_epin_disable(privep);
      break;
    }
}
#endif

/*******************************************************************************
 * Name: stm32_isocoutinterrupt
 *
 * Description:
 *   Incomplete periodic transfer interrupt
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_ISOCHRONOUS
static inline void stm32_isocoutinterrupt(FAR struct stm32_usbdev_s *priv)
{
  FAR struct stm32_ep_s *privep;
  FAR struct stm32_req_s *privreq;
  uint32_t regaddr;
  uint32_t doepctl;
  uint32_t dsts;
  bool eonum;
  bool soffn;

  /* When it receives an IISOOXFR interrupt, the application must read the
   * control registers of all isochronous OUT endpoints to determine which
   * endpoints had an incomplete transfer in the current microframe. An
   * endpoint transfer is incomplete if both the following conditions are true:
   *
   *   DOEPCTLx:EONUM = DSTS:SOFFN[0], and
   *   DOEPCTLx:EPENA = 1
   */

  for (i = 0; i < STM32_NENDPOINTS; i++)
    {
      /* Is this an isochronous OUT endpoint? */

      privep = &priv->epout[i];
      if (privep->eptype != USB_EP_ATTR_XFER_ISOC)
        {
          /* No... keep looking */

          continue;
        }

      /* Is there an active read request on the isochronous OUT endpoint? */

      if (!privep->active)
        {
          /* No.. the endpoint is not actively transmitting data */

          continue;
        }

      /* Check if this is the endpoint that had the incomplete transfer */

      regaddr = STM32_OTGFS_DOEPCTL(privep->epphy);
      doepctl = stm32_getreg(regaddr);
      dsts    = stm32_getreg(STM32_OTGFS_DSTS);

      /* EONUM = 0:even frame, 1:odd frame
       * SOFFN = Frame number of the received SOF
       */

      eonum = ((doepctl & OTGFS_DOEPCTL_EONUM) != 0);
      soffn = ((dsts & OTGFS_DSTS_SOFFN0) != 0);

      if (eonum != soffn)
        {
          /* Not this endpoint */

          continue;
        }

      /* For isochronous OUT endpoints with incomplete transfers,
       * the application must discard the data in the memory and
       * disable the endpoint.
       */

      stm32_req_complete(privep, -EIO);
#warning "Will clear OTGFS_DOEPCTL_USBAEP too"
      stm32_epout_disable(privep);
      break;
    }
}
#endif

/*******************************************************************************
 * Name: stm32_sessioninterrupt
 *
 * Description:
 *   Session request/new session detected interrupt
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_VBUSSENSING
static inline void stm32_sessioninterrupt(FAR struct stm32_usbdev_s *priv)
{
#warning "Missing logic"
}
#endif

/*******************************************************************************
 * Name: stm32_otginterrupt
 *
 * Description:
 *   OTG interrupt
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_VBUSSENSING
static inline void stm32_otginterrupt(FAR struct stm32_usbdev_s *priv)
{
  uint32_t regval;

  /* Check for session end detected */

  regval = stm32_getreg(STM32_OTGFS_GOTGINT);
  if ((regval & OTGFS_GOTGINT_SEDET) != 0)
  {
#warning "Missing logic"
  }

  /* Clear OTG interrupt */

  stm32_putreg(retval, STM32_OTGFS_GOTGINT);
}
#endif

/*******************************************************************************
 * Name: stm32_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 *******************************************************************************/

static int stm32_usbinterrupt(int irq, FAR void *context)
{
  /* At present, there is only a single OTG FS device support. Hence it is
   * pre-allocated as g_otgfsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  FAR struct stm32_usbdev_s *priv = &g_otgfsdev;
  uint32_t regval;

  usbtrace(TRACE_INTENTRY(STM32_TRACEINTID_USB), 0);

  /* Assure that we are in device mode */

  DEBUGASSERT((stm32_getreg(STM32_OTGFS_GINTSTS) & OTGFS_GINTSTS_CMOD) == OTGFS_GINTSTS_DEVMODE);

  /* Get the state of all enabled interrupts.  We will do this repeatedly
   * some interrupts (like RXFLVL) will generate additional interrupting
   * events.
   */

  for (;;)
    {
      /* Get the set of pending, un-masked interrupts */

      regval  = stm32_getreg(STM32_OTGFS_GINTSTS);
      regval &= stm32_getreg(STM32_OTGFS_GINTMSK);

      /* Break out of the loop when there are no further pending (and
       * unmasked) interrupts to be processes.
       */

      if (regval == 0)
        {
          break;
        }
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_INTPENDING), (uint16_t)regval);

      /* OUT endpoint interrupt. The core sets this bit to indicate that an
       * interrupt is pending on one of the OUT endpoints of the core.
       */

      if ((regval & OTGFS_GINT_OEP) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPOUT), (uint16_t)regval);
          stm32_epout_interrupt(priv);
          stm32_putreg(OTGFS_GINT_OEP, STM32_OTGFS_GINTSTS);
        }

      /* IN endpoint interrupt.  The core sets this bit to indicate that
       * an interrupt is pending on one of the IN endpoints of the core.
       */

      if ((regval & OTGFS_GINT_IEP) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN), (uint16_t)regval);
          stm32_epin_interrupt(priv);
          stm32_putreg(OTGFS_GINT_IEP, STM32_OTGFS_GINTSTS);
        }

      /* Host/device mode mismatch error interrupt */

#ifdef CONFIG_DEBUG_USB
      if ((regval & OTGFS_GINT_MMIS) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_MISMATCH), (uint16_t)regval);
          stm32_putreg(OTGFS_GINT_MMIS, STM32_OTGFS_GINTSTS);
        }
#endif

      /* Resume/remote wakeup detected interrupt */

      if ((regval & OTGFS_GINT_WKUP) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_WAKEUP), (uint16_t)regval);
          stm32_resumeinterrupt(priv);
          stm32_putreg(OTGFS_GINT_WKUP, STM32_OTGFS_GINTSTS);
        }

     /* USB suspend interrupt */

      if ((regval & OTGFS_GINT_USBSUSP) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SUSPEND), (uint16_t)regval);
          stm32_suspendinterrupt(priv);
          stm32_putreg(OTGFS_GINT_USBSUSP, STM32_OTGFS_GINTSTS);
        }

      /* Start of frame interrupt */

#ifdef CONFIG_USBDEV_SOFINTERRUPT
      if ((regval & OTGFS_GINT_SOF) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SOF), (uint16_t)regval);
          stm32_putreg(OTGFS_GINT_SOF, STM32_OTGFS_GINTSTS);
        }
#endif

      /* RxFIFO non-empty interrupt.  Indicates that there is at least one
       * packet pending to be read from the RxFIFO.
       */

      if ((regval & OTGFS_GINT_RXFLVL) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_RXFIFO), (uint16_t)regval);
          stm32_rxinterrupt(priv);
          stm32_putreg(OTGFS_GINT_RXFLVL, STM32_OTGFS_GINTSTS);
        }

      /* USB reset interrupt */

      if ((regval & OTGFS_GINT_USBRST) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_DEVRESET), (uint16_t)regval);

          /* Perform the device reset */

          stm32_usbreset(priv);
          usbtrace(TRACE_INTEXIT(STM32_TRACEINTID_USB), 0);
          stm32_putreg(OTGFS_GINT_USBRST, STM32_OTGFS_GINTSTS);
          return OK;
        }

      /* Enumeration done interrupt */

      if ((regval & OTGFS_GINT_ENUMDNE) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_ENUMDNE), (uint16_t)regval);
          stm32_enuminterrupt(priv);
          stm32_putreg(OTGFS_GINT_ENUMDNE, STM32_OTGFS_GINTSTS);
        }

      /* Incomplete isochronous IN transfer interrupt.  When the core finds
       * non-empty any of the isochronous IN endpoint FIFOs scheduled for
       * the current frame non-empty, the core generates an IISOIXFR
       * interrupt.
       */

#ifdef CONFIG_USBDEV_ISOCHRONOUS
      if ((regval & OTGFS_GINT_IISOIXFR) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_IISOIXFR), (uint16_t)regval);
          stm32_isocininterrupt(priv);
          stm32_putreg(OTGFS_GINT_IISOIXFR, STM32_OTGFS_GINTSTS);
        }

      /* Incomplete isochronous OUT transfer.  For isochronous OUT
       * endpoints, the XFRC interrupt may not always be asserted. If the
       * core drops isochronous OUT data packets, the application could fail
       * to detect the XFRC interrupt.  The incomplete Isochronous OUT data
       * interrupt indicates that an XFRC interrupt was not asserted on at
       * least one of the isochronous OUT endpoints. At this point, the
       * endpoint with the incomplete transfer remains enabled, but no active
       * transfers remain in progress on this endpoint on the USB.
       */

      if ((regval & OTGFS_GINT_IISOOXFR) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_IISOOXFR), (uint16_t)regval);
          stm32_isocoutinterrupt(priv);
          stm32_putreg(OTGFS_GINT_IISOOXFR, STM32_OTGFS_GINTSTS);
        }
#endif

      /* Session request/new session detected interrupt */

#ifdef CONFIG_USBDEV_VBUSSENSING
      if ((regval & OTGFS_GINT_SRQ) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SRQ), (uint16_t)regval);
          stm32_sessioninterrupt(priv);
          stm32_putreg(OTGFS_GINT_SRQ, STM32_OTGFS_GINTSTS);
        }

      /* OTG interrupt */

      if ((regval & OTGFS_GINT_OTG) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_OTG), (uint16_t)regval);
          stm32_otginterrupt(priv);
          stm32_putreg(OTGFS_GINT_OTG, STM32_OTGFS_GINTSTS);
        }
#endif
    }

  usbtrace(TRACE_INTEXIT(STM32_TRACEINTID_USB), 0);
  return OK;
}

/*******************************************************************************
 * Endpoint operations
 *******************************************************************************/

/*******************************************************************************
 * Name: stm32_enablegonak
 *
 * Description:
 *   Enable global OUT NAK mode
 *
 *******************************************************************************/

static void stm32_enablegonak(FAR struct stm32_ep_s *privep)
{
  uint32_t regval;

  /* First, make sure that there is no GNOAKEFF interrupt pending. */

#if 0
  stm32_putreg(OTGFS_GINT_GONAKEFF, STM32_OTGFS_GINTSTS);
#endif

  /* Enable Global OUT NAK mode in the core. */

  regval = stm32_getreg(STM32_OTGFS_DCTL);
  regval |= OTGFS_DCTL_SGONAK;
  stm32_putreg(regval, STM32_OTGFS_DCTL);

#if 0
  /* Wait for the GONAKEFF interrupt that indicates that the OUT NAK
   * mode is in effect.  When the interrupt handler pops the OUTNAK word
   * from the RxFIFO, the core sets the GONAKEFF interrupt.
   */

  while ((stm32_getreg(STM32_OTGFS_GINTSTS) & OTGFS_GINT_GONAKEFF) == 0);
  stm32_putreg(OTGFS_GINT_GONAKEFF, STM32_OTGFS_GINTSTS);

#else
  /* Since we are in the interrupt handler, we cannot wait inline for the
   * GONAKEFF because it cannot occur until service th RXFLVL global interrupt
   * and pop the OUTNAK word from the RxFIFO.
   *
   * Perhaps it is sufficient to wait for Global OUT NAK status to be reported
   * in OTGFS DCTL register?
   */

  while ((stm32_getreg(STM32_OTGFS_DCTL) & OTGFS_DCTL_GONSTS) == 0);
#endif
}

/*******************************************************************************
 * Name: stm32_disablegonak
 *
 * Description:
 *   Disable global OUT NAK mode
 *
 *******************************************************************************/

static void stm32_disablegonak(FAR struct stm32_ep_s *privep)
{
  uint32_t regval;

  /* Set the "Clear the Global OUT NAK bit" to disable global OUT NAK mode */

  regval  = stm32_getreg(STM32_OTGFS_DCTL);
  regval |= OTGFS_DCTL_CGONAK;
  stm32_putreg(regval, STM32_OTGFS_DCTL);
}

/*******************************************************************************
 * Name: stm32_epout_configure
 *
 * Description:
 *   Configure an OUT endpoint, making it usable
 *
 * Input Parameters:
 *   privep    - a pointer to an internal endpoint structure
 *   eptype    - The type of the endpoint
 *   maxpacket - The max packet size of the endpoint
 *
 *******************************************************************************/

static int stm32_epout_configure(FAR struct stm32_ep_s *privep, uint8_t eptype,
                                 uint16_t maxpacket)
{
  uint32_t mpsiz;
  uint32_t regaddr;
  uint32_t regval;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);

  /* For EP0, the packet size is encoded */

  if (privep->epphy == EP0)
    {
      DEBUGASSERT(eptype == USB_EP_ATTR_XFER_CONTROL);

      /* Map the size in bytes to the encoded value in the register */

      switch (maxpacket)
        {
          case 8:
            mpsiz = OTGFS_DOEPCTL0_MPSIZ_8;
            break;

          case 16:
            mpsiz = OTGFS_DOEPCTL0_MPSIZ_16;
            break;

          case 32:
            mpsiz = OTGFS_DOEPCTL0_MPSIZ_32;
            break;

          case 64:
            mpsiz = OTGFS_DOEPCTL0_MPSIZ_64;
            break;

          default:
            udbg("Unsupported maxpacket: %d\n", maxpacket);
            return -EINVAL;
        }
    }

  /* For other endpoints, the packet size is in bytes */

  else
    {
      mpsiz = (maxpacket << OTGFS_DOEPCTL_MPSIZ_SHIFT);
    }

  /* If the endpoint is already active don't change the endpoint control
   * register.
   */

  regaddr = STM32_OTGFS_DOEPCTL(privep->epphy);
  regval  = stm32_getreg(regaddr);
  if ((regval & OTGFS_DOEPCTL_USBAEP) == 0)
    {
      if (regval & OTGFS_DOEPCTL_NAKSTS)
        {
          regval |= OTGFS_DOEPCTL_CNAK;
        }
      
      regval &= ~(OTGFS_DOEPCTL_MPSIZ_MASK | OTGFS_DOEPCTL_EPTYP_MASK);
      regval |= mpsiz;
      regval |= (eptype << OTGFS_DOEPCTL_EPTYP_SHIFT);
      regval |= (OTGFS_DOEPCTL_SD0PID | OTGFS_DOEPCTL_USBAEP);
      stm32_putreg(regval, regaddr);

      /* Save the endpoint configuration */

      privep->ep.maxpacket = maxpacket;
      privep->eptype       = eptype;
      privep->stalled      = false;
    }

  /* Enable the interrupt for this endpoint */

  regval = stm32_getreg(STM32_OTGFS_DAINTMSK);
  regval |= OTGFS_DAINT_OEP(privep->epphy);
  stm32_putreg(regval, STM32_OTGFS_DAINTMSK);
  return OK;
}

/*******************************************************************************
 * Name: stm32_epin_configure
 *
 * Description:
 *   Configure an IN endpoint, making it usable
 *
 * Input Parameters:
 *   privep    - a pointer to an internal endpoint structure
 *   eptype    - The type of the endpoint
 *   maxpacket - The max packet size of the endpoint
 *
 *******************************************************************************/

static int stm32_epin_configure(FAR struct stm32_ep_s *privep, uint8_t eptype,
                                uint16_t maxpacket)
{
  uint32_t mpsiz;
  uint32_t regaddr;
  uint32_t regval;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);

  /* For EP0, the packet size is encoded */

  if (privep->epphy == EP0)
    {
      DEBUGASSERT(eptype == USB_EP_ATTR_XFER_CONTROL);

      /* Map the size in bytes to the encoded value in the register */

      switch (maxpacket)
        {
          case 8:
            mpsiz = OTGFS_DIEPCTL0_MPSIZ_8;
            break;

          case 16:
            mpsiz = OTGFS_DIEPCTL0_MPSIZ_16;
            break;

          case 32:
            mpsiz = OTGFS_DIEPCTL0_MPSIZ_32;
            break;

          case 64:
            mpsiz = OTGFS_DIEPCTL0_MPSIZ_64;
            break;

          default:
            udbg("Unsupported maxpacket: %d\n", maxpacket);
            return -EINVAL;
        }
    }

  /* For other endpoints, the packet size is in bytes */

  else
    {
      mpsiz = (maxpacket << OTGFS_DIEPCTL_MPSIZ_SHIFT);
    }


  /* If the endpoint is already active don't change the endpoint control
   * register.
   */

  regaddr = STM32_OTGFS_DIEPCTL(privep->epphy);
  regval  = stm32_getreg(regaddr);
  if ((regval & OTGFS_DIEPCTL_USBAEP) == 0)
    {
      if (regval & OTGFS_DIEPCTL_NAKSTS)
        {
          regval |= OTGFS_DIEPCTL_CNAK;
        }
      
      regval &= ~(OTGFS_DIEPCTL_MPSIZ_MASK | OTGFS_DIEPCTL_EPTYP_MASK | OTGFS_DIEPCTL_TXFNUM_MASK);
      regval |= mpsiz;
      regval |= (eptype << OTGFS_DIEPCTL_EPTYP_SHIFT);
      regval |= (eptype << OTGFS_DIEPCTL_TXFNUM_SHIFT);
      regval |= (OTGFS_DIEPCTL_SD0PID | OTGFS_DIEPCTL_USBAEP);
      stm32_putreg(regval, regaddr);

      /* Save the endpoint configuration */

      privep->ep.maxpacket = maxpacket;
      privep->eptype       = eptype;
      privep->stalled      = false;
    }

  /* Enable the interrupt for this endpoint */

  regval = stm32_getreg(STM32_OTGFS_DAINTMSK);
  regval |= OTGFS_DAINT_IEP(privep->epphy);
  stm32_putreg(regval, STM32_OTGFS_DAINTMSK);

  return OK;
}

/*******************************************************************************
 * Name: stm32_ep_configure
 *
 * Description:
 *   Configure endpoint, making it usable
 *
 * Input Parameters:
 *   ep   - the struct usbdev_ep_s instance obtained from allocep()
 *   desc - A struct usb_epdesc_s instance describing the endpoint
 *   last - true if this this last endpoint to be configured.  Some hardware
 *          needs to take special action when all of the endpoints have been
 *          configured.
 *
 *******************************************************************************/

static int stm32_ep_configure(FAR struct usbdev_ep_s *ep,
                              FAR const struct usb_epdesc_s *desc,
                              bool last)
{
  FAR struct stm32_ep_s *privep = (FAR struct stm32_ep_s *)ep;
  uint16_t maxpacket;
  uint8_t  eptype;
  int ret;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);
  DEBUGASSERT(desc->addr == ep->eplog);

  /* Initialize EP capabilities */

  maxpacket = GETUINT16(desc->mxpacketsize);
  eptype    = desc->attr & USB_EP_ATTR_XFERTYPE_MASK;

  /* Setup Endpoint Control Register */

  if (privep->isin)
    {
      ret = stm32_epin_configure(privep, eptype, maxpacket);
    }
  else
    {
      ret = stm32_epout_configure(privep, eptype, maxpacket);
    }

  return ret;
}

/*******************************************************************************
 * Name: stm32_ep0_configure
 *
 * Description:
 *   Reset Usb engine
 *
 *******************************************************************************/

static void stm32_ep0_configure(FAR struct stm32_usbdev_s *priv)
{
  /* Enable EP0 IN and OUT */

  (void)stm32_epin_configure(&priv->epin[EP0], USB_EP_ATTR_XFER_CONTROL,
                             CONFIG_USBDEV_EP0_MAXSIZE);
  (void)stm32_epout_configure(&priv->epout[EP0], USB_EP_ATTR_XFER_CONTROL,
                              CONFIG_USBDEV_EP0_MAXSIZE);
}

/*******************************************************************************
 * Name: stm32_epout_disable
 *
 * Description:
 *   Diable an OUT endpoint will no longer be used
 *
 *******************************************************************************/

static void stm32_epout_disable(FAR struct stm32_ep_s *privep)
{
  uint32_t regaddr;
  uint32_t regval;
  irqstate_t flags;

  usbtrace(TRACE_EPDISABLE, privep->epphy);

  /* Is this an IN or an OUT endpoint */

  /* Before disabling any OUT endpoint, the application must enable
   * Global OUT NAK mode in the core.
   */

  flags = irqsave();
  stm32_enablegonak(privep);

  /* Disable the required OUT endpoint by setting the EPDIS and SNAK bits
   * int DOECPTL register.
   */

  regaddr = STM32_OTGFS_DOEPCTL(privep->epphy);
  regval  = stm32_getreg(regaddr);
  regval &= ~OTGFS_DOEPCTL_USBAEP;
  regval |= (OTGFS_DOEPCTL_EPDIS | OTGFS_DOEPCTL_SNAK);
  stm32_putreg(regval, regaddr);

  /* Wait for the EPDISD interrupt which indicates that the OUT
   * endpoint is completely disabled.
   */

#if 0 /* Doesn't happen */
  regaddr = STM32_OTGFS_DOEPINT(privep->epphy);
  while ((stm32_getreg(regaddr) & OTGFS_DOEPINT_EPDISD) == 0);
#else
  /* REVISIT: */
  up_mdelay(50);
#endif

  /* Then disble the Global OUT NAK mode to continue receiving data
   * from other non-disabled OUT endpoints.
   */

  stm32_disablegonak(privep);

  /* Disable endpoint interrupts */

  regval  = stm32_getreg(STM32_OTGFS_DAINTMSK);
  regval &= ~OTGFS_DAINT_OEP(privep->epphy);
  stm32_putreg(regval, STM32_OTGFS_DAINTMSK);

  /* Cancel any queued read requests */

  stm32_req_cancel(privep, -ESHUTDOWN);

  irqrestore(flags);
}

/*******************************************************************************
 * Name: stm32_epin_disable
 *
 * Description:
 *   Disable an IN endpoint when it will no longer be used
 *
 *******************************************************************************/

static void stm32_epin_disable(FAR struct stm32_ep_s *privep)
{
  uint32_t regaddr;
  uint32_t regval;
  irqstate_t flags;

  usbtrace(TRACE_EPDISABLE, privep->epphy);

  /* After USB reset, the endpoint will already be deactivated by the
   * hardware. Trying to disable again will just hang in the wait.
   */

  regaddr = STM32_OTGFS_DIEPCTL(privep->epphy);
  regval  = stm32_getreg(regaddr);
  if ((regval & OTGFS_DIEPCTL_USBAEP) == 0)
    {
      return;
    }

  /* Make sure that there is no pending IPEPNE interrupt (because we are
   * to poll this bit below).
   */

  stm32_putreg(OTGFS_DIEPINT_INEPNE, STM32_OTGFS_DIEPINT(privep->epphy));

  /* Set the endpoint in NAK mode */

  regaddr = STM32_OTGFS_DIEPCTL(privep->epphy);
  regval  = stm32_getreg(regaddr);
  regval &= ~OTGFS_DIEPCTL_USBAEP;
  regval |= (OTGFS_DIEPCTL_EPDIS | OTGFS_DIEPCTL_SNAK);
  stm32_putreg(regval, regaddr);

  /* Wait for the INEPNE interrupt that indicates that we are now in NAK mode */

  regaddr = STM32_OTGFS_DIEPINT(privep->epphy);
  while ((stm32_getreg(regaddr) & OTGFS_DIEPINT_INEPNE) == 0);
  stm32_putreg(OTGFS_DIEPINT_INEPNE, regaddr);

  /* Deactivate and disable the endpoint by setting the EPIS and SNAK bits
   * the DIEPCTLx register.
   */

  flags = irqsave();
  regaddr = STM32_OTGFS_DIEPCTL(privep->epphy);
  regval  = stm32_getreg(regaddr);
  regval &= ~OTGFS_DIEPCTL_USBAEP;
  regval |= (OTGFS_DIEPCTL_EPDIS | OTGFS_DIEPCTL_SNAK);
  stm32_putreg(regval, regaddr);

  /* Wait for the EPDISD interrupt which indicates that the IN
   * endpoint is completely disabled.
   */

  regaddr = STM32_OTGFS_DIEPINT(privep->epphy);
  while ((stm32_getreg(regaddr) & OTGFS_DIEPINT_EPDISD) == 0);

  /* Flush any data remaining in the TxFIFO */

  stm32_txfifo_flush(OTGFS_GRSTCTL_TXFNUM_D(privep->epphy));

  /* Disable endpoint interrupts */

  regval  = stm32_getreg(STM32_OTGFS_DAINTMSK);
  regval &= ~OTGFS_DAINT_IEP(privep->epphy);
  stm32_putreg(regval, STM32_OTGFS_DAINTMSK);

  /* Cancel any queued write requests */

  stm32_req_cancel(privep, -ESHUTDOWN);

  irqrestore(flags);
}

/*******************************************************************************
 * Name: stm32_ep_disable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 *******************************************************************************/

static int stm32_ep_disable(FAR struct usbdev_ep_s *ep)
{
  FAR struct stm32_ep_s *privep = (FAR struct stm32_ep_s *)ep;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPDISABLE, privep->epphy);

  /* Is this an IN or an OUT endpoint */

  if (privep->isin)
    {
      /* Disable the IN endpoint */

      stm32_epin_disable(privep);
    }
  else
    {
      /* Disable the OUT endpoint */

      stm32_epout_disable(privep);
    }

  return OK;
}

/*******************************************************************************
 * Name: stm32_ep_allocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 *******************************************************************************/

static FAR struct usbdev_req_s *stm32_ep_allocreq(FAR struct usbdev_ep_s *ep)
{
  FAR struct stm32_req_s *privreq;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif
  usbtrace(TRACE_EPALLOCREQ, ((FAR struct stm32_ep_s *)ep)->epphy);

  privreq = (FAR struct stm32_req_s *)malloc(sizeof(struct stm32_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct stm32_req_s));
  return &privreq->req;
}

/*******************************************************************************
 * Name: stm32_ep_freereq
 *
 * Description:
 *   Free an I/O request
 *
 *******************************************************************************/

static void stm32_ep_freereq(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct stm32_req_s *privreq = (FAR struct stm32_req_s *)req;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, ((FAR struct stm32_ep_s *)ep)->epphy);
  free(privreq);
}

/*******************************************************************************
 * Name: stm32_ep_allocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void *stm32_ep_allocbuffer(FAR struct usbdev_ep_s *ep, unsigned bytes)
{
  usbtrace(TRACE_EPALLOCBUFFER, privep->epphy);

#ifdef CONFIG_USBDEV_DMAMEMORY
  return usbdev_dma_alloc(bytes);
#else
  return malloc(bytes);
#endif
}
#endif

/*******************************************************************************
 * Name: stm32_ep_freebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void stm32_ep_freebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf)
{
  usbtrace(TRACE_EPFREEBUFFER, privep->epphy);

#ifdef CONFIG_USBDEV_DMAMEMORY
  usbdev_dma_free(buf);
#else
  free(buf);
#endif
}
#endif

/*******************************************************************************
 * Name: stm32_ep_submit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 *******************************************************************************/

static int stm32_ep_submit(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct stm32_req_s *privreq = (FAR struct stm32_req_s *)req;
  FAR struct stm32_ep_s *privep = (FAR struct stm32_ep_s *)ep;
  FAR struct stm32_usbdev_s *priv;
  irqstate_t flags;
  int ret = OK;

  /* Some sanity checking */

#ifdef CONFIG_DEBUG
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      ullvdbg("req=%p callback=%p buf=%p ep=%p\n", req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, privep->epphy);
  priv = privep->dev;

#ifdef CONFIG_DEBUG
  if (!priv->driver)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_NOTCONFIGURED), priv->usbdev.speed);
      return -ESHUTDOWN;
    }
#endif

  /* Handle the request from the class driver */

  req->result = -EINPROGRESS;
  req->xfrd   = 0;

  /* Disable Interrupts */

  flags = irqsave();

  /* If we are stalled, then drop all requests on the floor */

  if (privep->stalled)
    {
      ret = -EBUSY;
    }
  else
    {
      /* Add the new request to the request queue for the endpoint. */

      if (stm32_req_addlast(privep, privreq) && !privep->active)
        {
          /* If a request was added to an IN endpoint, then attempt to send
           * the request data buffer now.
           */

          if (privep->isin)
            {
              usbtrace(TRACE_INREQQUEUED(privep->epphy), privreq->req.len);

              /* If the endpoint is not busy with another write request,
               * then process the newly received write request now.
               */

              if (!privep->active)
                {
                  stm32_epin_request(priv, privep);
                }
            }

          /* If the request was added to an OUT endoutput, then attempt to
           * setup a read into the request data buffer now (this will, of
           * course, fail if there is already a read in place).
           */

          else
            {
              usbtrace(TRACE_OUTREQQUEUED(privep->epphy), privreq->req.len);
              stm32_epout_request(priv, privep);
            }
        }
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: stm32_ep_cancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 *******************************************************************************/

static int stm32_ep_cancel(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct stm32_ep_s *privep = (FAR struct stm32_ep_s *)ep;
  FAR struct stm32_usbdev_s *priv;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPCANCEL, privep->epphy);
  priv = privep->dev;

  flags = irqsave();

  /* FIXME: if the request is the first, then we need to flush the EP
   *         otherwise just remove it from the list
   *
   *  but ... all other implementations cancel all requests ...
   */

  stm32_req_cancel(privep, -ESHUTDOWN);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: stm32_epout_setstall
 *
 * Description:
 *   Stall an OUT endpoint
 *
 *******************************************************************************/

static int stm32_epout_setstall(FAR struct stm32_ep_s *privep)
{
#if 1
  /* This implementation follows the requirements from the STM32 F4 reference
   * manual.
   */

  uint32_t regaddr;
  uint32_t regval;

  /* Put the core in the Global OUT NAK mode */

  stm32_enablegonak(privep);

  /* Disable and STALL the OUT endpoint by setting the EPDIS and STALL bits
   * in the DOECPTL register.
   */

  regaddr = STM32_OTGFS_DOEPCTL(privep->epphy);
  regval  = stm32_getreg(regaddr);
  regval |= (OTGFS_DOEPCTL_EPDIS | OTGFS_DOEPCTL_STALL);
  stm32_putreg(regval, regaddr);

  /* Wait for the EPDISD interrupt which indicates that the OUT
   * endpoint is completely disabled.
   */

#if 0 /* Doesn't happen */
  regaddr = STM32_OTGFS_DOEPINT(privep->epphy);
  while ((stm32_getreg(regaddr) & OTGFS_DOEPINT_EPDISD) == 0);
#else
  /* REVISIT: */
  up_mdelay(50);
#endif

  /* Disable Global OUT NAK mode */

  stm32_disablegonak(privep);

  /* The endpoint is now stalled */

  privep->stalled = true;
  return OK;
#else
  /* This implementation follows the STMicro code example. */
  /* REVISIT: */

  uint32_t regaddr;
  uint32_t regval;

  /* Stall the OUT endpoint by setting the STALL bit in the DOECPTL register. */

  regaddr = STM32_OTGFS_DOEPCTL(privep->epphy);
  regval  = stm32_getreg(regaddr);
  regval |= OTGFS_DOEPCTL_STALL;
  stm32_putreg(regval, regaddr);

  /* The endpoint is now stalled */

  privep->stalled = true;
  return OK;
#endif
}

/*******************************************************************************
 * Name: stm32_epin_setstall
 *
 * Description:
 *   Stall an IN endpoint
 *
 *******************************************************************************/

static int stm32_epin_setstall(FAR struct stm32_ep_s *privep)
{
  uint32_t regaddr;
  uint32_t regval;

  /* Get the IN endpoint device control register */

  regaddr = STM32_OTGFS_DIEPCTL(privep->epphy);
  regval  = stm32_getreg(regaddr);

  /* Is the endpoint enabled? */

  if ((regval & OTGFS_DIEPCTL_EPENA) != 0)
    {
      /* Yes.. the endpoint is enabled, disable it */

      regval = OTGFS_DIEPCTL_EPDIS;
    }
  else
    {
      regval = 0;
    }

  /* Then stall the endpoint */

  regval |= OTGFS_DIEPCTL_STALL;
  stm32_putreg(regval, regaddr);

  /* The endpoint is now stalled */

  privep->stalled = true;
  return OK;
}

/*******************************************************************************
 * Name: stm32_ep_setstall
 *
 * Description:
 *   Stall an endpoint
 *
 *******************************************************************************/

static int stm32_ep_setstall(FAR struct stm32_ep_s *privep)
{
  usbtrace(TRACE_EPSTALL, privep->epphy);

  /* Is this an IN endpoint? */

  if (privep->isin == 1)
    {
      return stm32_epin_setstall(privep);
    }
  else
    {
      return stm32_epout_setstall(privep);
    }
}

/*******************************************************************************
 * Name: stm32_ep_clrstall
 *
 * Description:
 *   Resume a stalled endpoint
 *
 *******************************************************************************/

static int stm32_ep_clrstall(FAR struct stm32_ep_s *privep)
{
  uint32_t regaddr;
  uint32_t regval;
  uint32_t stallbit;
  uint32_t data0bit;

  usbtrace(TRACE_EPRESUME, privep->epphy);

  /* Is this an IN endpoint? */

  if (privep->isin == 1)
    {
      /* Clear the stall bit in the IN endpoint device control register */

      regaddr  = STM32_OTGFS_DIEPCTL(privep->epphy);
      stallbit = OTGFS_DIEPCTL_STALL;
      data0bit = OTGFS_DIEPCTL_SD0PID;
    }
  else
    {
      /* Clear the stall bit in the IN endpoint device control register */

      regaddr  = STM32_OTGFS_DOEPCTL(privep->epphy);
      stallbit = OTGFS_DOEPCTL_STALL;
      data0bit = OTGFS_DOEPCTL_SD0PID;
    }

  /* Clear the stall bit */

  regval  = stm32_getreg(regaddr);
  regval &= ~stallbit;

  /* Set the DATA0 pid for interrupt and bulk endpoints */

  if (privep->eptype == USB_EP_ATTR_XFER_INT ||
      privep->eptype == USB_EP_ATTR_XFER_BULK)
    {
      /* Writing this bit sets the DATA0 PID */

      regval |= data0bit;
    }

  stm32_putreg(regval, regaddr);

  /* The endpoint is no longer stalled */

  privep->stalled = false;
  return OK;
}

/*******************************************************************************
 * Name: stm32_ep_stall
 *
 * Description:
 *   Stall or resume an endpoint
 *
 *******************************************************************************/

static int stm32_ep_stall(FAR struct usbdev_ep_s *ep, bool resume)
{
  FAR struct stm32_ep_s *privep = (FAR struct stm32_ep_s *)ep;
  irqstate_t flags;
  int ret;

  /* Set or clear the stall condition as requested */

  flags = irqsave();
  if (resume)
    {
      ret = stm32_ep_clrstall(privep);
    }
  else
    {
      ret = stm32_ep_setstall(privep);
    }
  irqrestore(flags);

  return ret;
}

/*******************************************************************************
 * Name: stm32_ep0_stall
 *
 * Description:
 *   Stall endpoint 0
 *
 *******************************************************************************/

static void stm32_ep0_stall(FAR struct stm32_usbdev_s *priv)
{
  stm32_epin_setstall(&priv->epin[EP0]);
  stm32_epout_setstall(&priv->epout[EP0]);
  priv->stalled = true;
  stm32_ep0out_ctrlsetup(priv);
}

/*******************************************************************************
 * Device operations
 *******************************************************************************/

/*******************************************************************************
 * Name: stm32_ep_alloc
 *
 * Description:
 *   Allocate an endpoint matching the parameters.
 *
 * Input Parameters:
 *   eplog  - 7-bit logical endpoint number (direction bit ignored).  Zero means
 *            that any endpoint matching the other requirements will suffice.  The
 *            assigned endpoint can be found in the eplog field.
 *   in     - true: IN (device-to-host) endpoint requested
 *   eptype - Endpoint type.  One of {USB_EP_ATTR_XFER_ISOC, USB_EP_ATTR_XFER_BULK,
 *            USB_EP_ATTR_XFER_INT}
 *
 *******************************************************************************/

static FAR struct usbdev_ep_s *stm32_ep_alloc(FAR struct usbdev_s *dev,
                                              uint8_t eplog, bool in,
                                              uint8_t eptype)
{
  FAR struct stm32_usbdev_s *priv = (FAR struct stm32_usbdev_s *)dev;
  uint8_t epavail;
  irqstate_t flags;
  int epphy;
  int epno = 0;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)eplog);

  /* Ignore any direction bits in the logical address */

  epphy = USB_EPNO(eplog);

  /* Get the set of available endpoints */

  flags = irqsave();
  epavail = priv->epavail;

  /* A physical address of 0 means that any endpoint will do */

  if (epphy > 0)
    {
      /* Otherwise, we will return the endpoint structure only for the requested
       * 'logical' endpoint.  All of the other checks will still be performed.
       *
       * First, verify that the logical endpoint is in the range supported by
       * by the hardware.
       */

      if (epphy >= STM32_NENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADEPNO), (uint16_t)epphy);
          return NULL;
        }

      /* Remove all of the candidate endpoints from the bitset except for the
       * this physical endpoint number.
       */

      epavail &= (1 << epphy);
    }

  /* Is there an available endpoint? */

  if (epavail)
    {
      /* Yes.. Select the lowest numbered endpoint in the set of available
       * endpoints.
       */

      for (epno = 1; epno < STM32_NENDPOINTS; epno++)
        {
          uint8_t bit = 1 << epno;
          if ((epavail & bit) != 0)
            {
              /* Mark the endpoint no longer available */

              priv->epavail &= ~(1 << epno);

              /* And return the pointer to the standard endpoint structure */

              irqrestore(flags);
              return in ? &priv->epin[epno].ep : &priv->epout[epno].ep;
            }
       }

      /* We should not get here */
    }

  usbtrace(TRACE_DEVERROR(STM32_TRACEERR_NOEP), (uint16_t)eplog);
  irqrestore(flags);
  return NULL;
}

/*******************************************************************************
 * Name: stm32_ep_free
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 *******************************************************************************/

static void stm32_ep_free(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep)
{
  FAR struct stm32_usbdev_s *priv = (FAR struct stm32_usbdev_s *)dev;
  FAR struct stm32_ep_s *privep = (FAR struct stm32_ep_s *)ep;
  irqstate_t flags;

  usbtrace(TRACE_DEVFREEEP, (uint16_t)privep->epphy);

  if (priv && privep)
    {
      /* Mark the endpoint as available */

      flags = irqsave();
      priv->epavail |= (1 << privep->epphy);
      irqrestore(flags);
    }
}

/*******************************************************************************
 * Name: stm32_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 *******************************************************************************/

static int stm32_getframe(struct usbdev_s *dev)
{
  uint32_t regval;

  usbtrace(TRACE_DEVGETFRAME, 0);

  /* Return the last frame number of the last SOF detected by the hardware */

  regval = stm32_getreg(STM32_OTGFS_DSTS);
  return (int)((regval & OTGFS_DSTS_SOFFN_MASK) >> OTGFS_DSTS_SOFFN_SHIFT);
}

/*******************************************************************************
 * Name: stm32_wakeup
 *
 * Description:
 *   Exit suspend mode.
 *
 *******************************************************************************/

static int stm32_wakeup(struct usbdev_s *dev)
{
  FAR struct stm32_usbdev_s *priv = (FAR struct stm32_usbdev_s *)dev;
  uint32_t regval;
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, 0);

  /* Is wakeup enabled? */

  flags = irqsave();
  if (priv->wakeup)
    {
      /* Yes... is the core suspended? */

      regval = stm32_getreg(STM32_OTGFS_DSTS);
      if ((regval & OTGFS_DSTS_SUSPSTS) != 0)
        {
           /* Re-start the PHY clock and un-gate USB core clock (HCLK) */

#ifdef CONFIG_USBDEV_LOWPOWER
          regval = stm32_getreg(STM32_OTGFS_PCGCCTL);
          regval &= ~(OTGFS_PCGCCTL_STPPCLK | OTGFS_PCGCCTL_GATEHCLK);
          stm32_putreg(regval, STM32_OTGFS_PCGCCTL);
#endif
          /* Activate Remote wakeup signaling */

          regval  = stm32_getreg(STM32_OTGFS_DCTL);
          regval |= OTGFS_DCTL_RWUSIG;
          stm32_putreg(regval, STM32_OTGFS_DCTL);
          up_mdelay(5);
          regval &= ~OTGFS_DCTL_RWUSIG;
          stm32_putreg(regval, STM32_OTGFS_DCTL);
        }
    }

  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: stm32_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature
 *
 *******************************************************************************/

static int stm32_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  FAR struct stm32_usbdev_s *priv = (FAR struct stm32_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/*******************************************************************************
 * Name: stm32_pullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 *******************************************************************************/

static int stm32_pullup(struct usbdev_s *dev, bool enable)
{
  uint32_t regval;

  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  irqstate_t flags = irqsave();
  regval = stm32_getreg(STM32_OTGFS_DCTL);
  if (enable)
    {
      /* Connect the device by clearing the soft disconnect bit in the DCTL
       * register
       */

      regval &= ~OTGFS_DCTL_SDIS;
    }
  else
    {
      /* Connect the device by setting the soft disconnect bit in the DCTL
       * register
       */

      regval |= OTGFS_DCTL_SDIS;
    }

  stm32_putreg(regval, STM32_OTGFS_DCTL);
  up_mdelay(3);

  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: stm32_setaddress
 *
 * Description:
 *   Set the devices USB address
 *
 *******************************************************************************/

static void stm32_setaddress(struct stm32_usbdev_s *priv, uint16_t address)
{
  uint32_t regval;

  /* Set the device address in the DCFG register */

  regval = stm32_getreg(STM32_OTGFS_DCFG);
  regval &= ~OTGFS_DCFG_DAD_MASK;
  regval |= ((uint32_t)address << OTGFS_DCFG_DAD_SHIFT);
  stm32_putreg(regval, STM32_OTGFS_DCFG);

  /* Are we now addressed?  (i.e., do we have a non-NULL device
   * address?)
   */

  if (address != 0)
    {
      priv->devstate  = DEVSTATE_ADDRESSED;
      priv->addressed = true;
    }
  else
    {
      priv->devstate  = DEVSTATE_DEFAULT;
      priv->addressed = false;
    }
}

/*******************************************************************************
 * Name: stm32_txfifo_flush
 *
 * Description:
 *   Flush the specific TX fifo.
 *
 *******************************************************************************/

static int stm32_txfifo_flush(uint32_t txfnum)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the TX FIFO flush operation */

  regval = OTGFS_GRSTCTL_TXFFLSH | txfnum;
  stm32_putreg(regval, STM32_OTGFS_GRSTCTL);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < STM32_FLUSH_DELAY; timeout++)
    {
      regval = stm32_getreg(STM32_OTGFS_GRSTCTL);
      if ((regval & OTGFS_GRSTCTL_TXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
  return OK;
}

/*******************************************************************************
 * Name: stm32_rxfifo_flush
 *
 * Description:
 *   Flush the RX fifo.
 *
 *******************************************************************************/

static int stm32_rxfifo_flush(void)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the RX FIFO flush operation */

  stm32_putreg(OTGFS_GRSTCTL_RXFFLSH, STM32_OTGFS_GRSTCTL);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < STM32_FLUSH_DELAY; timeout++)
    {
      regval = stm32_getreg(STM32_OTGFS_GRSTCTL);
      if ((regval & OTGFS_GRSTCTL_RXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
  return OK;
}

/*******************************************************************************
 * Name: stm32_swinitialize
 *
 * Description:
 *   Initialize all driver data structures.
 *
 *******************************************************************************/

static void stm32_swinitialize(FAR struct stm32_usbdev_s *priv)
{
  FAR struct stm32_ep_s *privep;
  int i;

  /* Initialize the device state structure */

  memset(priv, 0, sizeof(struct stm32_usbdev_s));

  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->epin[EP0].ep;
  priv->epavail    = STM32_EP_AVAILABLE;

  priv->epin[EP0].ep.priv  = priv;
  priv->epout[EP0].ep.priv = priv;

  /* Initialize the endpoint lists */

  for (i = 0; i < STM32_NENDPOINTS; i++)
    {
      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the physical endpoint number (which is just the index to the
       * endpoint).
       */

      privep           = &priv->epin[i];
      privep->ep.ops   = &g_epops;
      privep->dev      = priv;
      privep->isin     = 1;

      /* The index, i, is the physical endpoint address;  Map this
       * to a logical endpoint address usable by the class driver.
       */

      privep->epphy    = i;
      privep->ep.eplog = STM32_EPPHYIN2LOG(i);

      /* Control until endpoint is activated */

      privep->eptype       = USB_EP_ATTR_XFER_CONTROL;
      privep->ep.maxpacket = CONFIG_USBDEV_EP0_MAXSIZE;
    }

  /* Initialize the endpoint lists */

  for (i = 0; i < STM32_NENDPOINTS; i++)
    {
      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the physical endpoint number (which is just the index to the
       * endpoint).
       */

      privep           = &priv->epout[i];
      privep->ep.ops   = &g_epops;
      privep->dev      = priv;

      /* The index, i, is the physical endpoint address;  Map this
       * to a logical endpoint address usable by the class driver.
       */

      privep->epphy    = i;
      privep->ep.eplog = STM32_EPPHYOUT2LOG(i);

      /* Control until endpoint is activated */

      privep->eptype       = USB_EP_ATTR_XFER_CONTROL;
      privep->ep.maxpacket = CONFIG_USBDEV_EP0_MAXSIZE;
    }
}

/*******************************************************************************
 * Name: stm32_hwinitialize
 *
 * Description:
 *   Configure the OTG FS core for operation.
 *
 *******************************************************************************/

static void stm32_hwinitialize(FAR struct stm32_usbdev_s *priv)
{
  uint32_t regval;
  uint32_t timeout;
  uint32_t address;
  int i;

  /* At startup the core is in FS mode. */

  /* Disable global interrupts by clearing the GINTMASK bit in the GAHBCFG
   * register; Set the TXFELVL bit in the GAHBCFG register so that TxFIFO
   * interrupts will occur when the TxFIFO is truly empty (not just half full).
   */

  stm32_putreg(OTGFS_GAHBCFG_TXFELVL, STM32_OTGFS_GAHBCFG);

  /* Common USB OTG core initialization */
  /* Reset after a PHY select and set Host mode.  First, wait for AHB master
   * IDLE state.
   */

  for (timeout = 0; timeout < STM32_READY_DELAY; timeout++)
    {
      up_udelay(3);
      regval = stm32_getreg(STM32_OTGFS_GRSTCTL);
      if ((regval & OTGFS_GRSTCTL_AHBIDL) != 0)
        {
          break;
        }
    }

  /* Then perform the core soft reset. */

  stm32_putreg(OTGFS_GRSTCTL_CSRST, STM32_OTGFS_GRSTCTL);
  for (timeout = 0; timeout < STM32_READY_DELAY; timeout++)
    {
      regval = stm32_getreg(STM32_OTGFS_GRSTCTL);
      if ((regval & OTGFS_GRSTCTL_CSRST) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);

  /* Deactivate the power down */

  regval  = (OTGFS_GCCFG_PWRDWN | OTGFS_GCCFG_VBUSASEN | OTGFS_GCCFG_VBUSBSEN);
#ifndef CONFIG_USBDEV_VBUSSENSING
  regval |= OTGFS_GCCFG_NOVBUSSENS;
#endif
#ifdef CONFIG_STM32_OTGFS_SOFOUTPUT
  regval |= OTGFS_GCCFG_SOFOUTEN;
#endif
  stm32_putreg(regval, STM32_OTGFS_GCCFG);
  up_mdelay(20);

  /* Force Device Mode */

  regval  = stm32_getreg(STM32_OTGFS_GUSBCFG);
  regval &= ~OTGFS_GUSBCFG_FHMOD;
  regval |= OTGFS_GUSBCFG_FDMOD;
  stm32_putreg(regval, STM32_OTGFS_GUSBCFG);
  up_mdelay(50);

  /* Initialize device mode */
  /* Restart the Phy Clock */

  stm32_putreg(0, STM32_OTGFS_PCGCCTL);

  /* Device configuration register */

  regval = stm32_getreg(STM32_OTGFS_DCFG);
  regval &= ~OTGFS_DCFG_PFIVL_MASK;
  regval |= OTGFS_DCFG_PFIVL_80PCT;
  stm32_putreg(regval, STM32_OTGFS_DCFG);

  /* Set full speed phy */

  regval = stm32_getreg(STM32_OTGFS_DCFG);
  regval &= ~OTGFS_DCFG_DSPD_MASK;
  regval |= OTGFS_DCFG_DSPD_FS;
  stm32_putreg(regval, STM32_OTGFS_DCFG);

  /* Set Rx FIFO size */

  stm32_putreg(STM32_RXFIFO_WORDS, STM32_OTGFS_GRXFSIZ);

  /* EP0 TX */

  address = STM32_RXFIFO_WORDS;
  regval  = (address << OTGFS_DIEPTXF0_TX0FD_SHIFT) |
            (STM32_EP0_TXFIFO_WORDS << OTGFS_DIEPTXF0_TX0FSA_SHIFT);
  stm32_putreg(regval, STM32_OTGFS_DIEPTXF0);

  /* EP1 TX */

  address += STM32_EP0_TXFIFO_WORDS;
  regval   = (address << OTGFS_DIEPTXF_INEPTXSA_SHIFT) |
             (STM32_EP1_TXFIFO_WORDS << OTGFS_DIEPTXF_INEPTXFD_SHIFT);
  stm32_putreg(regval, STM32_OTGFS_DIEPTXF1);

  /* EP2 TX */

  address += STM32_EP1_TXFIFO_WORDS;
  regval   = (address << OTGFS_DIEPTXF_INEPTXSA_SHIFT) |
             (STM32_EP2_TXFIFO_WORDS << OTGFS_DIEPTXF_INEPTXFD_SHIFT);
  stm32_putreg(regval, STM32_OTGFS_DIEPTXF2);

  /* EP3 TX */

  address += STM32_EP2_TXFIFO_WORDS;
  regval   = (address << OTGFS_DIEPTXF_INEPTXSA_SHIFT) |
             (STM32_EP3_TXFIFO_WORDS << OTGFS_DIEPTXF_INEPTXFD_SHIFT);
  stm32_putreg(regval, STM32_OTGFS_DIEPTXF3);

  /* Flush the FIFOs */

  stm32_txfifo_flush(OTGFS_GRSTCTL_TXFNUM_DALL);
  stm32_rxfifo_flush();

  /* Clear all pending Device Interrupts */

  stm32_putreg(0, STM32_OTGFS_DIEPMSK);
  stm32_putreg(0, STM32_OTGFS_DOEPMSK);
  stm32_putreg(0, STM32_OTGFS_DIEPEMPMSK);
  stm32_putreg(0xffffffff, STM32_OTGFS_DAINT);
  stm32_putreg(0, STM32_OTGFS_DAINTMSK);

  /* Configure all IN endpoints */

  for (i = 0; i < STM32_NENDPOINTS; i++)
    {
      regval = stm32_getreg(STM32_OTGFS_DIEPCTL(i));
      if ((regval & OTGFS_DIEPCTL_EPENA) != 0)
        {
          /* The endpoint is already enabled */

          regval = OTGFS_DIEPCTL_EPENA | OTGFS_DIEPCTL_SNAK;
        }
      else
        {
          regval = 0;
        }

      stm32_putreg(regval, STM32_OTGFS_DIEPCTL(i));
      stm32_putreg(0, STM32_OTGFS_DIEPTSIZ(i));
      stm32_putreg(0xff, STM32_OTGFS_DIEPINT(i));
    }

  /* Configure all OUT endpoints */

  for (i = 0; i < STM32_NENDPOINTS; i++)
    {
      regval = stm32_getreg(STM32_OTGFS_DOEPCTL(i));
      if ((regval & OTGFS_DOEPCTL_EPENA) != 0)
        {
          /* The endpoint is already enabled */

          regval = OTGFS_DOEPCTL_EPENA | OTGFS_DOEPCTL_SNAK;
        }
      else
        {
          regval = 0;
        }

      stm32_putreg(regval, STM32_OTGFS_DOEPCTL(i));
      stm32_putreg(0, STM32_OTGFS_DOEPTSIZ(i));
      stm32_putreg(0xff, STM32_OTGFS_DOEPINT(i));
    }

  /* Disable all interrupts. */

  stm32_putreg(0, STM32_OTGFS_GINTMSK);

  /* Clear any pending USB_OTG Interrupts */

  stm32_putreg(0xffffffff, STM32_OTGFS_GOTGINT);

  /* Clear any pending interrupts */

  stm32_putreg(0xbfffffff, STM32_OTGFS_GINTSTS);

  /* Enable the interrupts in the INTMSK */

  regval = (OTGFS_GINT_RXFLVL | OTGFS_GINT_USBSUSP | OTGFS_GINT_ENUMDNE |
            OTGFS_GINT_IEP | OTGFS_GINT_OEP | OTGFS_GINT_USBRST);

#ifdef CONFIG_USBDEV_ISOCHRONOUS
  regval |= (OTGFS_GINT_IISOIXFR | OTGFS_GINT_IISOOXFR);
#endif

#ifdef CONFIG_USBDEV_SOFINTERRUPT
  regval |= OTGFS_GINT_SOF;
#endif

#ifdef CONFIG_USBDEV_VBUSSENSING
  regval |= (OTGFS_GINT_OTG | OTGFS_GINT_SRQ);
#endif

#ifdef CONFIG_DEBUG_USB
  regval |= OTGFS_GINT_MMIS;
#endif

  stm32_putreg(regval, STM32_OTGFS_GINTMSK);

  /* Enable the USB global interrupt by setting GINTMSK in the global OTG
   * FS AHB configuration register; Set the TXFELVL bit in the GAHBCFG
   * register so that TxFIFO interrupts will occur when the TxFIFO is truly
   * empty (not just half full).
   */

  stm32_putreg(OTGFS_GAHBCFG_GINTMSK | OTGFS_GAHBCFG_TXFELVL,
               STM32_OTGFS_GAHBCFG);
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: up_usbinitialize
 *
 * Description:
 *   Initialize USB hardware.
 *
 * Assumptions:
 * - This function is called very early in the initialization sequence
 * - PLL and GIO pin initialization is not performed here but should been in
 *   the low-level  boot logic:  PLL1 must be configured for operation at 48MHz
 *   and P0.23 and PO.31 in PINSEL1 must be configured for Vbus and USB connect
 *   LED.
 *
 *******************************************************************************/

void up_usbinitialize(void)
{
  /* At present, there is only a single OTG FS device support. Hence it is
   * pre-allocated as g_otgfsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  FAR struct stm32_usbdev_s *priv = &g_otgfsdev;
  int ret;

  usbtrace(TRACE_DEVINIT, 0);

  /* Here we assume that:
   *
   * 1. GPIOA and OTG FS peripheral clocking has already been enabled as part
   *    of the boot sequence.
   * 2. Board-specific logic has already enabled other board specific GPIOs
   *    for things like soft pull-up, VBUS sensing, power controls, and over-
   *    current detection.
   */

  /* Configure OTG FS alternate function pins
   *
   * PIN* SIGNAL      DIRECTION
   * ---- ----------- ----------
   * PA8  OTG_FS_SOF  SOF clock output
   * PA9  OTG_FS_VBUS VBUS input for device, Driven by external regulator by
   *                  host (not an alternate function)
   * PA10 OTG_FS_ID   OTG ID pin (only needed in Dual mode)
   * PA11 OTG_FS_DM   D- I/O
   * PA12 OTG_FS_DP   D+ I/O
   *
   * *Pins may vary from device-to-device.
   */

  stm32_configgpio(GPIO_OTGFS_DM);
  stm32_configgpio(GPIO_OTGFS_DP);
  stm32_configgpio(GPIO_OTGFS_ID);    /* Only needed for OTG */

  /* SOF output pin configuration is configurable. */

#ifdef CONFIG_STM32_OTGFS_SOFOUTPUT
  stm32_configgpio(GPIO_OTGFS_SOF);
#endif

  /* Uninitialize the hardware so that we know that we are starting from a
   * known state. */

  up_usbuninitialize();

  /* Initialie the driver data structure */

  stm32_swinitialize(priv);

  /* Attach the OTG FS interrupt handler */

  ret = irq_attach(STM32_IRQ_OTGFS, stm32_usbinterrupt);
  if (ret < 0)
    {
      udbg("irq_attach failed\n", ret);
      goto errout;
    }

  /* Initialize the USB OTG core */

  stm32_hwinitialize(priv);

  /* Disconnect device */

  stm32_pullup(&priv->usbdev, false);

  /* Reset/Re-initialize the USB hardware */

  stm32_usbreset(priv);

  /* Enable USB controller interrupts at the NVIC */

  up_enable_irq(STM32_IRQ_OTGFS);

  /* Set the interrrupt priority */

  up_prioritize_irq(STM32_IRQ_OTGFS, CONFIG_OTGFS_PRI);
  return;

errout:
  up_usbuninitialize();
}

/*******************************************************************************
 * Name: up_usbuninitialize
 *******************************************************************************/

void up_usbuninitialize(void)
{
  /* At present, there is only a single OTG FS device support. Hence it is
   * pre-allocated as g_otgfsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  FAR struct stm32_usbdev_s *priv = &g_otgfsdev;
  irqstate_t flags;
  int i;

  usbtrace(TRACE_DEVUNINIT, 0);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  /* Disconnect device */

  flags = irqsave();
  stm32_pullup(&priv->usbdev, false);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable and detach IRQs */

  up_disable_irq(STM32_IRQ_OTGFS);
  irq_detach(STM32_IRQ_OTGFS);

  /* Disable all endpoint interrupts */

  for (i = 0; i < STM32_NENDPOINTS; i++)
    {
      stm32_putreg(0xff, STM32_OTGFS_DIEPINT(i));
      stm32_putreg(0xff, STM32_OTGFS_DOEPINT(i));
    }

  stm32_putreg(0, STM32_OTGFS_DIEPMSK);
  stm32_putreg(0, STM32_OTGFS_DOEPMSK);
  stm32_putreg(0, STM32_OTGFS_DIEPEMPMSK);
  stm32_putreg(0, STM32_OTGFS_DAINTMSK);
  stm32_putreg(0xffffffff, STM32_OTGFS_DAINT);

  /* Flush the FIFOs */

  stm32_txfifo_flush(OTGFS_GRSTCTL_TXFNUM_DALL);
  stm32_rxfifo_flush();

  /* TODO: Turn off USB power and clocking */

  priv->devstate = DEVSTATE_DEFAULT;
  irqrestore(flags);
}

/*******************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method will be
 *   called to bind it to a USB device driver.
 *
 *******************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  /* At present, there is only a single OTG FS device support. Hence it is
   * pre-allocated as g_otgfsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  FAR struct stm32_usbdev_s *priv = &g_otgfsdev;
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  priv->driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BINDFAILED), (uint16_t)-ret);
      priv->driver = NULL;
    }
  else
    {
      /* Enable USB controller interrupts */

      up_enable_irq(STM32_IRQ_OTGFS);

      /* FIXME: nothing seems to call DEV_CONNECT(), but we need to set
       *        the RS bit to enable the controller.  It kind of makes sense
       *        to do this after the class has bound to us...
       * GEN:   This bug is really in the class driver.  It should make the
       *        soft connect when it is ready to be enumerated.  I have added
       *        that logic to the class drivers but left this logic here.
       */

      stm32_pullup(&priv->usbdev, true);
      priv->usbdev.speed = USB_SPEED_FULL;
    }

  return ret;
}

/*******************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver.If the USB device is connected to a USB host,
 *   it will first disconnect().  The driver is also requested to unbind() and clean
 *   up any device state, before this procedure finally returns.
 *
 *******************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  /* At present, there is only a single OTG FS device support. Hence it is
   * pre-allocated as g_otgfsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  FAR struct stm32_usbdev_s *priv = &g_otgfsdev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (driver != priv->driver)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.
   */

  flags = irqsave();
  stm32_usbreset(priv);

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupts */

  up_disable_irq(STM32_IRQ_OTGFS);

  /* Disconnect device */

  stm32_pullup(&priv->usbdev, false);

  /* Unhook the driver */

  priv->driver = NULL;
  irqrestore(flags);

  return OK;
}

#endif /* CONFIG_USBDEV && CONFIG_STM32_OTGFSDEV */
