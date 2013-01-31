/****************************************************************************
 * drivers/usbdev/pl2303.c
 *
 *   Copyright (C) 2008-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic emulates the Prolific PL2303 serial/USB converter
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Number of requests in the write queue */

#ifndef CONFIG_PL2303_NWRREQS
#  define CONFIG_PL2303_NWRREQS 4
#endif

/* Number of requests in the read queue */

#ifndef CONFIG_PL2303_NRDREQS
#  define CONFIG_PL2303_NRDREQS 4
#endif

/* Logical endpoint numbers / max packet sizes */

#ifndef CONFIG_PL2303_EPINTIN
#  warning "EPINTIN not defined in the configuration"
#  define CONFIG_PL2303_EPINTIN 1
#endif

#ifndef CONFIG_PL2303_EPBULKOUT
#  warning "EPBULKOUT not defined in the configuration"
#  define CONFIG_PL2303_EPBULKOUT 2
#endif

#ifndef CONFIG_PL2303_EPBULKIN
#  warning "EPBULKIN not defined in the configuration"
#  define CONFIG_PL2303_EPBULKIN 3
#endif

/* Packet and request buffer sizes */

#ifndef CONFIG_PL2303_EP0MAXPACKET
#  define CONFIG_PL2303_EP0MAXPACKET 64
#endif

#undef CONFIG_PL2303_BULKREQLEN

/* Vendor and product IDs and strings */

#ifndef CONFIG_PL2303_VENDORID
#  define CONFIG_PL2303_VENDORID  0x067b
#endif

#ifndef CONFIG_PL2303_PRODUCTID
#  define CONFIG_PL2303_PRODUCTID 0x2303
#endif

#ifndef CONFIG_PL2303_VENDORSTR
#  warning "No Vendor string specified"
#  define CONFIG_PL2303_VENDORSTR  "NuttX"
#endif

#ifndef CONFIG_PL2303_PRODUCTSTR
#  warning "No Product string specified"
#  define CONFIG_PL2303_PRODUCTSTR "USBdev Serial"
#endif

#undef CONFIG_PL2303_SERIALSTR
#define CONFIG_PL2303_SERIALSTR "0"

#undef CONFIG_PL2303_CONFIGSTR
#define CONFIG_PL2303_CONFIGSTR "Bulk"

/* USB Controller */

#ifndef CONFIG_USBDEV_SELFPOWERED
#  define SELFPOWERED USB_CONFIG_ATTR_SELFPOWER
#else
#  define SELFPOWERED (0)
#endif

#ifndef CONFIG_USBDEV_REMOTEWAKEUP
#  define REMOTEWAKEUP USB_CONFIG_ATTR_WAKEUP
#else
#  define REMOTEWAKEUP (0)
#endif

#ifndef CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100
#endif

/* Descriptors ****************************************************************/

/* These settings are not modifiable via the NuttX configuration */

#define PL2303_VERSIONNO           (0x0202) /* Device version number */
#define PL2303_CONFIGIDNONE        (0)      /* Config ID means to return to address mode */
#define PL2303_CONFIGID            (1)      /* The only supported configuration ID */
#define PL2303_NCONFIGS            (1)      /* Number of configurations supported */
#define PL2303_INTERFACEID         (0)
#define PL2303_ALTINTERFACEID      (0)
#define PL2303_NINTERFACES         (1)      /* Number of interfaces in the configuration */
#define PL2303_NENDPOINTS          (3)      /* Number of endpoints in the interface  */

/* Endpoint configuration */

#define PL2303_EPINTIN_ADDR        (USB_DIR_IN|CONFIG_PL2303_EPINTIN)
#define PL2303_EPINTIN_ATTR        (USB_EP_ATTR_XFER_INT)
#define PL2303_EPINTIN_MXPACKET    (10)

#define PL2303_EPOUTBULK_ADDR      (CONFIG_PL2303_EPBULKOUT)
#define PL2303_EPOUTBULK_ATTR      (USB_EP_ATTR_XFER_BULK)

#define PL2303_EPINBULK_ADDR       (USB_DIR_IN|CONFIG_PL2303_EPBULKIN)
#define PL2303_EPINBULK_ATTR       (USB_EP_ATTR_XFER_BULK)

/* String language */

#define PL2303_STR_LANGUAGE        (0x0409) /* en-us */

/* Descriptor strings */

#define PL2303_MANUFACTURERSTRID   (1)
#define PL2303_PRODUCTSTRID        (2)
#define PL2303_SERIALSTRID         (3)
#define PL2303_CONFIGSTRID         (4)

/* Buffer big enough for any of our descriptors */

#define PL2303_MXDESCLEN           (64)

/* Vender specific control requests *******************************************/

#define PL2303_CONTROL_TYPE        (0x20)
#define PL2303_SETLINEREQUEST      (0x20) /* OUT, Recipient interface */
#define PL2303_GETLINEREQUEST      (0x21) /* IN, Recipient interface */
#define PL2303_SETCONTROLREQUEST   (0x22) /* OUT, Recipient interface */
#define PL2303_BREAKREQUEST        (0x23) /* OUT, Recipient interface */

/* Vendor read/write */

#define PL2303_RWREQUEST_TYPE      (0x40)
#define PL2303_RWREQUEST           (0x01) /* IN/OUT, Recipient device */

/* Misc Macros ****************************************************************/

/* min/max macros */

#ifndef min
#  define min(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef max
#  define max(a,b) ((a)>(b)?(a):(b))
#endif

/* Trace values *************************************************************/

#define PL2303_CLASSAPI_SETUP       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_SETUP)
#define PL2303_CLASSAPI_SHUTDOWN    TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_SHUTDOWN)
#define PL2303_CLASSAPI_ATTACH      TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_ATTACH)
#define PL2303_CLASSAPI_DETACH      TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_DETACH)
#define PL2303_CLASSAPI_IOCTL       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_IOCTL)
#define PL2303_CLASSAPI_RECEIVE     TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_RECEIVE)
#define PL2303_CLASSAPI_RXINT       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_RXINT)
#define PL2303_CLASSAPI_RXAVAILABLE TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_RXAVAILABLE)
#define PL2303_CLASSAPI_SEND        TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_SEND)
#define PL2303_CLASSAPI_TXINT       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_TXINT)
#define PL2303_CLASSAPI_TXREADY     TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_TXREADY)
#define PL2303_CLASSAPI_TXEMPTY     TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_TXEMPTY)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Container to support a list of requests */

struct pl2303_req_s
{
  FAR struct pl2303_req_s *flink;     /* Implements a singly linked list */
  FAR struct usbdev_req_s *req;       /* The contained request */
};

/* This structure describes the internal state of the driver */

struct pl2303_dev_s
{
  FAR struct uart_dev_s    serdev;    /* Serial device structure */
  FAR struct usbdev_s     *usbdev;    /* usbdev driver pointer */

  uint8_t config;                     /* Configuration number */
  uint8_t nwrq;                       /* Number of queue write requests (in reqlist)*/
  uint8_t nrdq;                       /* Number of queue read requests (in epbulkout) */
  bool    rxenabled;                  /* true: UART RX "interrupts" enabled */
  uint8_t linest[7];                  /* Fake line status */
  int16_t rxhead;                     /* Working head; used when rx int disabled */

  FAR struct usbdev_ep_s  *epintin;   /* Interrupt IN endpoint structure */
  FAR struct usbdev_ep_s  *epbulkin;  /* Bulk IN endpoint structure */
  FAR struct usbdev_ep_s  *epbulkout; /* Bulk OUT endpoint structure */
  FAR struct usbdev_req_s *ctrlreq;   /* Control request */
  struct sq_queue_s        reqlist;   /* List of write request containers */

  /* Pre-allocated write request containers.  The write requests will
   * be linked in a free list (reqlist), and used to send requests to
   * EPBULKIN; Read requests will be queued in the EBULKOUT.
   */

  struct pl2303_req_s wrreqs[CONFIG_PL2303_NWRREQS];
  struct pl2303_req_s rdreqs[CONFIG_PL2303_NWRREQS];

  /* Serial I/O buffers */

  char rxbuffer[CONFIG_PL2303_RXBUFSIZE];
  char txbuffer[CONFIG_PL2303_TXBUFSIZE];
};

/* The internal version of the class driver */

struct pl2303_driver_s
{
  struct usbdevclass_driver_s drvr;
  FAR struct pl2303_dev_s     *dev;
};

/* This is what is allocated */

struct pl2303_alloc_s
{
  struct pl2303_dev_s    dev;
  struct pl2303_driver_s drvr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Transfer helpers *********************************************************/

static uint16_t usbclass_fillrequest(FAR struct pl2303_dev_s *priv,
                 uint8_t *reqbuf, uint16_t reqlen);
static int     usbclass_sndpacket(FAR struct pl2303_dev_s *priv);
static inline int usbclass_recvpacket(FAR struct pl2303_dev_s *priv,
                 uint8_t *reqbuf, uint16_t reqlen);

/* Request helpers *********************************************************/

static struct  usbdev_req_s *usbclass_allocreq(FAR struct usbdev_ep_s *ep,
                 uint16_t len);
static void    usbclass_freereq(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);

/* Configuration ***********************************************************/

static int     usbclass_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc);
#ifdef CONFIG_USBDEV_DUALSPEED
static void    usbclass_mkepbulkdesc(const struct usb_epdesc_s *indesc,
                 uint16_t mxpacket, struct usb_epdesc_s *outdesc);
static int16_t usbclass_mkcfgdesc(uint8_t *buf, uint8_t speed, uint8_t type);
#else
static int16_t usbclass_mkcfgdesc(uint8_t *buf);
#endif
static void    usbclass_resetconfig(FAR struct pl2303_dev_s *priv);
static int     usbclass_setconfig(FAR struct pl2303_dev_s *priv,
                 uint8_t config);

/* Completion event handlers ***********************************************/

static void    usbclass_ep0incomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);
static void    usbclass_rdcomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);
static void    usbclass_wrcomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);

/* USB class device ********************************************************/

static int     usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static int     usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev,
                 FAR const struct usb_ctrlreq_s *ctrl, FAR uint8_t *dataout,
                 size_t outlen);
static void    usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);

/* Serial port *************************************************************/

static int     usbser_setup(FAR struct uart_dev_s *dev);
static void    usbser_shutdown(FAR struct uart_dev_s *dev);
static int     usbser_attach(FAR struct uart_dev_s *dev);
static void    usbser_detach(FAR struct uart_dev_s *dev);
static void    usbser_rxint(FAR struct uart_dev_s *dev, bool enable);
static void    usbser_txint(FAR struct uart_dev_s *dev, bool enable);
static bool    usbser_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/* USB class device ********************************************************/

static const struct usbdevclass_driverops_s g_driverops =
{
  usbclass_bind,        /* bind */
  usbclass_unbind,      /* unbind */
  usbclass_setup,       /* setup */
  usbclass_disconnect,  /* disconnect */
  NULL,                 /* suspend */
  NULL,                 /* resume */
};

/* Serial port *************************************************************/

static const struct uart_ops_s g_uartops =
{
  usbser_setup,         /* setup */
  usbser_shutdown,      /* shutdown */
  usbser_attach,        /* attach */
  usbser_detach,        /* detach */
  NULL,                 /* ioctl */
  NULL,                 /* receive */
  usbser_rxint,         /* rxinit */
  NULL,                 /* rxavailable */
  NULL,                 /* send */
  usbser_txint,         /* txinit */
  NULL,                 /* txready */
  usbser_txempty        /* txempty */
};

/* USB descriptor templates these will be copied and modified **************/

static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {LSBYTE(0x0200), MSBYTE(0x0200)},             /* usb */
  USB_CLASS_PER_INTERFACE,                      /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_PL2303_EP0MAXPACKET,                   /* maxpacketsize */
  { LSBYTE(CONFIG_PL2303_VENDORID),             /* vendor */
    MSBYTE(CONFIG_PL2303_VENDORID) },
  { LSBYTE(CONFIG_PL2303_PRODUCTID),            /* product */
    MSBYTE(CONFIG_PL2303_PRODUCTID) },
  { LSBYTE(PL2303_VERSIONNO),                   /* device */
    MSBYTE(PL2303_VERSIONNO) },
  PL2303_MANUFACTURERSTRID,                     /* imfgr */
  PL2303_PRODUCTSTRID,                          /* iproduct */
  PL2303_SERIALSTRID,                           /* serno */
  PL2303_NCONFIGS                               /* nconfigs */
};

static const struct usb_cfgdesc_s g_cfgdesc =
{
  USB_SIZEOF_CFGDESC,                           /* len */
  USB_DESC_TYPE_CONFIG,                         /* type */
  {0, 0},                                       /* totallen -- to be provided */
  PL2303_NINTERFACES,                           /* ninterfaces */
  PL2303_CONFIGID,                              /* cfgvalue */
  PL2303_CONFIGSTRID,                           /* icfg */
  USB_CONFIG_ATTR_ONE|SELFPOWERED|REMOTEWAKEUP, /* attr */
  (CONFIG_USBDEV_MAXPOWER + 1) / 2              /* mxpower */
};

static const struct usb_ifdesc_s g_ifdesc =
{
  USB_SIZEOF_IFDESC,                            /* len */
  USB_DESC_TYPE_INTERFACE,                      /* type */
  0,                                            /* ifno */
  0,                                            /* alt */
  PL2303_NENDPOINTS,                            /* neps */
  USB_CLASS_VENDOR_SPEC,                        /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
  PL2303_CONFIGSTRID                            /* iif */
};

static const struct usb_epdesc_s g_epintindesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  PL2303_EPINTIN_ADDR,                          /* addr */
  PL2303_EPINTIN_ATTR,                          /* attr */
  { LSBYTE(PL2303_EPINTIN_MXPACKET),            /* maxpacket */
    MSBYTE(PL2303_EPINTIN_MXPACKET) },
  1                                             /* interval */
};

static const struct usb_epdesc_s g_epbulkoutdesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  PL2303_EPOUTBULK_ADDR,                        /* addr */
  PL2303_EPOUTBULK_ATTR,                        /* attr */
  { LSBYTE(64), MSBYTE(64) },                   /* maxpacket -- might change to 512*/
  0                                             /* interval */
};

static const struct usb_epdesc_s g_epbulkindesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  PL2303_EPINBULK_ADDR,                         /* addr */
  PL2303_EPINBULK_ATTR,                         /* attr */
  { LSBYTE(64), MSBYTE(64) },                   /* maxpacket -- might change to 512*/
  0                                             /* interval */
};

#ifdef CONFIG_USBDEV_DUALSPEED
static const struct usb_qualdesc_s g_qualdesc =
{
  USB_SIZEOF_QUALDESC,                          /* len */
  USB_DESC_TYPE_DEVICEQUALIFIER,                /* type */
  {LSBYTE(0x0200), MSBYTE(0x0200) },            /* USB */
  USB_CLASS_VENDOR_SPEC,                        /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_PL2303_EP0MAXPACKET,                   /* mxpacketsize */
  PL2303_NCONFIGS,                              /* nconfigs */
  0,                                            /* reserved */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: usbclass_fillrequest
 *
 * Description:
 *   If there is data to send it is copied to the given buffer.  Called either
 *   to initiate the first write operation, or from the completion interrupt handler
 *   service consecutive write operations.
 *
 * NOTE: The USB serial driver does not use the serial drivers uart_xmitchars()
 *   API.  That logic is essentially duplicated here because unlike UART hardware,
 *   we need to be able to handle writes not byte-by-byte, but packet-by-packet.
 *   Unfortunately, that decision also exposes some internals of the serial driver
 *   in the following.
 *
 ************************************************************************************/

static uint16_t usbclass_fillrequest(FAR struct pl2303_dev_s *priv, uint8_t *reqbuf,
                                     uint16_t reqlen)
{
  FAR uart_dev_t *serdev = &priv->serdev;
  FAR struct uart_buffer_s *xmit = &serdev->xmit;
  irqstate_t flags;
  uint16_t nbytes = 0;

  /* Disable interrupts */

  flags = irqsave();

  /* Transfer bytes while we have bytes available and there is room in the request */

  while (xmit->head != xmit->tail && nbytes < reqlen)
    {
      *reqbuf++ = xmit->buffer[xmit->tail];
      nbytes++;

      /* Increment the tail pointer */

      if (++(xmit->tail) >= xmit->size)
        {
          xmit->tail = 0;
        }
    }

  /* When all of the characters have been sent from the buffer
   * disable the "TX interrupt".
   */

  if (xmit->head == xmit->tail)
    {
      uart_disabletxint(serdev);
    }

  /* If any bytes were removed from the buffer, inform any waiters
   * there there is space available.
   */

  if (nbytes)
    {
      uart_datasent(serdev);
    }

  irqrestore(flags);
  return nbytes;
}

/************************************************************************************
 * Name: usbclass_sndpacket
 *
 * Description:
 *   This function obtains write requests, transfers the TX data into the request,
 *   and submits the requests to the USB controller.  This continues untils either
 *   (1) there are no further packets available, or (2) thre is not further data
 *   to send.
 *
 ************************************************************************************/

static int usbclass_sndpacket(FAR struct pl2303_dev_s *priv)
{
  FAR struct usbdev_ep_s *ep;
  FAR struct usbdev_req_s *req;
  FAR struct pl2303_req_s *reqcontainer;
  uint16_t reqlen;
  irqstate_t flags;
  int len;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -ENODEV;
    }
#endif

  flags = irqsave();

  /* Use our IN endpoint for the transfer */

  ep = priv->epbulkin;

  /* Loop until either (1) we run out or write requests, or (2) usbclass_fillrequest()
   * is unable to fill the request with data (i.e., until there is no more data
   * to be sent).
   */

  uvdbg("head=%d tail=%d nwrq=%d empty=%d\n",
        priv->serdev.xmit.head, priv->serdev.xmit.tail,
        priv->nwrq, sq_empty(&priv->reqlist));

  /* Get the maximum number of bytes that will fit into one bulk IN request */

#ifdef CONFIG_PL2303_BULKREQLEN
  reqlen = MAX(CONFIG_PL2303_BULKREQLEN, ep->maxpacket);
#else
  reqlen = ep->maxpacket;
#endif

  while (!sq_empty(&priv->reqlist))
    {
      /* Peek at the request in the container at the head of the list */

      reqcontainer = (struct pl2303_req_s *)sq_peek(&priv->reqlist);
      req          = reqcontainer->req;

      /* Fill the request with serial TX data */

      len = usbclass_fillrequest(priv, req->buf, reqlen);
      if (len > 0)
        {
          /* Remove the empty container from the request list */

          (void)sq_remfirst(&priv->reqlist);
          priv->nwrq--;

          /* Then submit the request to the endpoint */

          req->len     = len;
          req->priv    = reqcontainer;
          req->flags   = USBDEV_REQFLAGS_NULLPKT;
          ret          = EP_SUBMIT(ep, req);
          if (ret != OK)
            {
              usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL), (uint16_t)-ret);
              break;
            }
        }
      else
        {
          break;
        }
    }

  irqrestore(flags);
  return ret;
}

/************************************************************************************
 * Name: usbclass_recvpacket
 *
 * Description:
 *   A normal completion event was received by the read completion handler at the
 *   interrupt level (with interrupts disabled).  This function handles the USB packet
 *   and provides the received data to the uart RX buffer.
 *
 * Assumptions:
 *   Called from the USB interrupt handler with interrupts disabled.
 *
 ************************************************************************************/

static inline int usbclass_recvpacket(FAR struct pl2303_dev_s *priv,
                                      uint8_t *reqbuf, uint16_t reqlen)
{
  FAR uart_dev_t *serdev = &priv->serdev;
  FAR struct uart_buffer_s *recv = &serdev->recv;
  uint16_t currhead;
  uint16_t nexthead;
  uint16_t nbytes = 0;

  /* Get the next head index. During the time that RX interrupts are disabled, the
   * the serial driver will be extracting data from the circular buffer and modifying
   * recv.tail.  During this time, we should avoid modifying recv.head; Instead we will
   * use a shadow copy of the index.  When interrupts are restored, the real recv.head
   * will be updated with this indes.
   */

  if (priv->rxenabled)
    {
      currhead = recv->head;
    }
  else
    {
      currhead = priv->rxhead;
    }

  /* Pre-calculate the head index and check for wrap around.  We need to do this
   * so that we can determine if the circular buffer will overrun BEFORE we
   * overrun the buffer!
   */

  nexthead = currhead + 1;
  if (nexthead >= recv->size)
    {
      nexthead = 0;
    }

  /* Then copy data into the RX buffer until either: (1) all of the data has been
   * copied, or (2) the RX buffer is full.  NOTE:  If the RX buffer becomes full,
   * then we have overrun the serial driver and data will be lost.
   */

  while (nexthead != recv->tail && nbytes < reqlen)
    {
      /* Copy one byte to the head of the circular RX buffer */

      recv->buffer[currhead] = *reqbuf++;

      /* Update counts and indices */

      currhead = nexthead;
      nbytes++;

      /* Increment the head index and check for wrap around */

      nexthead = currhead + 1;
      if (nexthead >= recv->size)
        {
          nexthead = 0;
        }
    }

  /* Write back the head pointer using the shadow index if RX "interrupts"
   * are disabled.
   */

  if (priv->rxenabled)
    {
      recv->head = currhead;
    }
  else
    {
      priv->rxhead = currhead;
    }

  /* If data was added to the incoming serial buffer, then wake up any
   * threads is waiting for incoming data. If we are running in an interrupt
   * handler, then the serial driver will not run until the interrupt handler
   * returns.
   */

  if (priv->rxenabled && nbytes > 0)
    {
      uart_datareceived(serdev);
    }

  /* Return an error if the entire packet could not be transferred */

  if (nbytes < reqlen)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RXOVERRUN), 0);
      return -ENOSPC;
    }
  return OK;
}

/****************************************************************************
 * Name: usbclass_allocreq
 *
 * Description:
 *   Allocate a request instance along with its buffer
 *
 ****************************************************************************/

static struct usbdev_req_s *usbclass_allocreq(FAR struct usbdev_ep_s *ep,
                                              uint16_t len)
{
  FAR struct usbdev_req_s *req;

  req = EP_ALLOCREQ(ep);
  if (req != NULL)
    {
      req->len = len;
      req->buf = EP_ALLOCBUFFER(ep, len);
      if (!req->buf)
        {
          EP_FREEREQ(ep, req);
          req = NULL;
        }
    }
  return req;
}

/****************************************************************************
 * Name: usbclass_freereq
 *
 * Description:
 *   Free a request instance along with its buffer
 *
 ****************************************************************************/

static void usbclass_freereq(FAR struct usbdev_ep_s *ep,
                             FAR struct usbdev_req_s *req)
{
  if (ep != NULL && req != NULL)
    {
      if (req->buf != NULL)
        {
          EP_FREEBUFFER(ep, req->buf);
        }
      EP_FREEREQ(ep, req);
    }
}

/****************************************************************************
 * Name: usbclass_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

static int usbclass_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc)
{
  const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
    case 0:
      {
        /* Descriptor 0 is the language id */

        strdesc->len     = 4;
        strdesc->type    = USB_DESC_TYPE_STRING;
        strdesc->data[0] = LSBYTE(PL2303_STR_LANGUAGE);
        strdesc->data[1] = MSBYTE(PL2303_STR_LANGUAGE);
        return 4;
      }

    case PL2303_MANUFACTURERSTRID:
      str = CONFIG_PL2303_VENDORSTR;
      break;

    case PL2303_PRODUCTSTRID:
      str = CONFIG_PL2303_PRODUCTSTR;
      break;

    case PL2303_SERIALSTRID:
      str = CONFIG_PL2303_SERIALSTR;
      break;

    case PL2303_CONFIGSTRID:
      str = CONFIG_PL2303_CONFIGSTR;
      break;

    default:
      return -EINVAL;
    }

   /* The string is utf16-le.  The poor man's utf-8 to utf16-le
    * conversion below will only handle 7-bit en-us ascii
    */

   len = strlen(str);
   for (i = 0, ndata = 0; i < len; i++, ndata += 2)
     {
       strdesc->data[ndata]   = str[i];
       strdesc->data[ndata+1] = 0;
     }

   strdesc->len  = ndata+2;
   strdesc->type = USB_DESC_TYPE_STRING;
   return strdesc->len;
}

/****************************************************************************
 * Name: usbclass_mkepbulkdesc
 *
 * Description:
 *   Construct the endpoint descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
static inline void usbclass_mkepbulkdesc(const FAR struct usb_epdesc_s *indesc,
                                         uint16_t mxpacket,
                                         FAR struct usb_epdesc_s *outdesc)
{
  /* Copy the canned descriptor */

  memcpy(outdesc, indesc, USB_SIZEOF_EPDESC);

  /* Then add the correct max packet size */

  outdesc->mxpacketsize[0] = LSBYTE(mxpacket);
  outdesc->mxpacketsize[1] = MSBYTE(mxpacket);
}
#endif

/****************************************************************************
 * Name: usbclass_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
static int16_t usbclass_mkcfgdesc(uint8_t *buf, uint8_t speed, uint8_t type)
#else
static int16_t usbclass_mkcfgdesc(uint8_t *buf)
#endif
{
  FAR struct usb_cfgdesc_s *cfgdesc = (struct usb_cfgdesc_s*)buf;
#ifdef CONFIG_USBDEV_DUALSPEED
  bool hispeed = (speed == USB_SPEED_HIGH);
  uint16_t bulkmxpacket;
#endif
  uint16_t totallen;

  /* This is the total length of the configuration (not necessarily the
   * size that we will be sending now.
   */

  totallen = USB_SIZEOF_CFGDESC + USB_SIZEOF_IFDESC + PL2303_NENDPOINTS * USB_SIZEOF_EPDESC;

  /* Configuration descriptor -- Copy the canned descriptor and fill in the
   * type (we'll also need to update the size below
   */

  memcpy(cfgdesc, &g_cfgdesc, USB_SIZEOF_CFGDESC);
  buf += USB_SIZEOF_CFGDESC;

  /*  Copy the canned interface descriptor */

  memcpy(buf, &g_ifdesc, USB_SIZEOF_IFDESC);
  buf += USB_SIZEOF_IFDESC;

  /* Make the three endpoint configurations.  First, check for switches
   * between high and full speed
   */

#ifdef CONFIG_USBDEV_DUALSPEED
  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG)
    {
      hispeed = !hispeed;
    }
#endif

  memcpy(buf, &g_epintindesc, USB_SIZEOF_EPDESC);
  buf += USB_SIZEOF_EPDESC;

#ifdef CONFIG_USBDEV_DUALSPEED
  if (hispeed)
    {
      bulkmxpacket = 512;
    }
  else
    {
      bulkmxpacket = 64;
    }

  usbclass_mkepbulkdesc(&g_epbulkoutdesc, bulkmxpacket, (struct usb_epdesc_s*)buf);
  buf += USB_SIZEOF_EPDESC;
  usbclass_mkepbulkdesc(&g_epbulkindesc, bulkmxpacket, (struct usb_epdesc_s*)buf);
#else
  memcpy(buf, &g_epbulkoutdesc, USB_SIZEOF_EPDESC);
  buf += USB_SIZEOF_EPDESC;
  memcpy(buf, &g_epbulkindesc, USB_SIZEOF_EPDESC);
#endif

  /* Finally, fill in the total size of the configuration descriptor */

  cfgdesc->totallen[0] = LSBYTE(totallen);
  cfgdesc->totallen[1] = MSBYTE(totallen);
  return totallen;
}

/****************************************************************************
 * Name: usbclass_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void usbclass_resetconfig(FAR struct pl2303_dev_s *priv)
{
  /* Are we configured? */

  if (priv->config != PL2303_CONFIGIDNONE)
    {
      /* Yes.. but not anymore */

      priv->config = PL2303_CONFIGIDNONE;

      /* Inform the "upper half" driver that there is no (functional) USB
       * connection.
       */

#ifdef CONFIG_SERIAL_REMOVABLE
      uart_connected(&priv->serdev, false);
#endif

      /* Disable endpoints.  This should force completion of all pending
       * transfers.
       */

      EP_DISABLE(priv->epintin);
      EP_DISABLE(priv->epbulkin);
      EP_DISABLE(priv->epbulkout);
    }
}

/****************************************************************************
 * Name: usbclass_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

static int usbclass_setconfig(FAR struct pl2303_dev_s *priv, uint8_t config)
{
  FAR struct usbdev_req_s *req;
#ifdef CONFIG_USBDEV_DUALSPEED
  struct usb_epdesc_s epdesc;
  uint16_t bulkmxpacket;
#endif
  int i;
  int ret = 0;

#if CONFIG_DEBUG
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
    }
#endif

  if (config == priv->config)
    {
      /* Already configured -- Do nothing */

      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALREADYCONFIGURED), 0);
      return 0;
    }

  /* Discard the previous configuration data */

  usbclass_resetconfig(priv);

  /* Was this a request to simply discard the current configuration? */

  if (config == PL2303_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGNONE), 0);
      return 0;
    }

  /* We only accept one configuration */

  if (config != PL2303_CONFIGID)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGIDBAD), 0);
      return -EINVAL;
    }

  /* Configure the IN interrupt endpoint */

  ret = EP_CONFIGURE(priv->epintin, &g_epintindesc, false);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINCONFIGFAIL), 0);
      goto errout;
    }
  priv->epintin->priv = priv;

  /* Configure the IN bulk endpoint */

#ifdef CONFIG_USBDEV_DUALSPEED
  if (priv->usbdev->speed == USB_SPEED_HIGH)
    {
      bulkmxpacket = 512;
    }
  else
    {
      bulkmxpacket = 64;
    }

  usbclass_mkepbulkdesc(&g_epbulkindesc, bulkmxpacket, &epdesc);
  ret = EP_CONFIGURE(priv->epbulkin, &epdesc, false);
#else
  ret = EP_CONFIGURE(priv->epbulkin, &g_epbulkindesc, false);
#endif
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Configure the OUT bulk endpoint */

#ifdef CONFIG_USBDEV_DUALSPEED
  usbclass_mkepbulkdesc(&g_epbulkoutdesc, bulkmxpacket, &epdesc);
  ret = EP_CONFIGURE(priv->epbulkout, &epdesc, true);
#else
  ret = EP_CONFIGURE(priv->epbulkout, &g_epbulkoutdesc, true);
#endif
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Queue read requests in the bulk OUT endpoint */

  DEBUGASSERT(priv->nrdq == 0);
  for (i = 0; i < CONFIG_PL2303_NRDREQS; i++)
    {
      req           = priv->rdreqs[i].req;
      req->callback = usbclass_rdcomplete;
      ret           = EP_SUBMIT(priv->epbulkout, req);
      if (ret != OK)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT), (uint16_t)-ret);
          goto errout;
        }

      priv->nrdq++;
    }

  /* We are successfully configured */

  priv->config = config;

  /* Inform the "upper half" driver that we are "open for business" */

#ifdef CONFIG_SERIAL_REMOVABLE
  uart_connected(&priv->serdev, true);
#endif

  return OK;

errout:
  usbclass_resetconfig(priv);
  return ret;
}

/****************************************************************************
 * Name: usbclass_ep0incomplete
 *
 * Description:
 *   Handle completion of EP0 control operations
 *
 ****************************************************************************/

static void usbclass_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                   FAR struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_REQRESULT), (uint16_t)-req->result);
    }
}

/****************************************************************************
 * Name: usbclass_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.  This
 *   is handled like the receipt of serial data on the "UART"
 *
 ****************************************************************************/

static void usbclass_rdcomplete(FAR struct usbdev_ep_s *ep,
                                FAR struct usbdev_req_s *req)
{
  FAR struct pl2303_dev_s *priv;
  irqstate_t flags;
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (!ep || !ep->priv || !req)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract references to private data */

  priv = (FAR struct pl2303_dev_s*)ep->priv;

  /* Process the received data unless this is some unusual condition */

  flags = irqsave();
  switch (req->result)
    {
    case 0: /* Normal completion */
      usbtrace(TRACE_CLASSRDCOMPLETE, priv->nrdq);
      usbclass_recvpacket(priv, req->buf, req->xfrd);
      break;

    case -ESHUTDOWN: /* Disconnection */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSHUTDOWN), 0);
      priv->nrdq--;
      irqrestore(flags);
      return;

    default: /* Some other error occurred */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDUNEXPECTED), (uint16_t)-req->result);
      break;
    };

  /* Requeue the read request */

#ifdef CONFIG_PL2303_BULKREQLEN
  req->len = max(CONFIG_PL2303_BULKREQLEN, ep->maxpacket);
#else
  req->len = ep->maxpacket;
#endif

  ret      = EP_SUBMIT(ep, req);
  if (ret != OK)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT), (uint16_t)-req->result);
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: usbclass_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static void usbclass_wrcomplete(FAR struct usbdev_ep_s *ep,
                                FAR struct usbdev_req_s *req)
{
  FAR struct pl2303_dev_s *priv;
  FAR struct pl2303_req_s *reqcontainer;
  irqstate_t flags;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (!ep || !ep->priv || !req || !req->priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract references to our private data */

  priv         = (FAR struct pl2303_dev_s *)ep->priv;
  reqcontainer = (FAR struct pl2303_req_s *)req->priv;

  /* Return the write request to the free list */

  flags = irqsave();
  sq_addlast((sq_entry_t*)reqcontainer, &priv->reqlist);
  priv->nwrq++;
  irqrestore(flags);

  /* Send the next packet unless this was some unusual termination
   * condition
   */

  switch (req->result)
    {
    case OK: /* Normal completion */
      usbtrace(TRACE_CLASSWRCOMPLETE, priv->nwrq);
      usbclass_sndpacket(priv);
      break;

    case -ESHUTDOWN: /* Disconnection */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRSHUTDOWN), priv->nwrq);
      break;

    default: /* Some other error occurred */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRUNEXPECTED), (uint16_t)-req->result);
      break;
    }
}

/****************************************************************************
 * USB Class Driver Methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbclass_bind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                         FAR struct usbdev_s *dev)
{
  FAR struct pl2303_dev_s *priv = ((FAR struct pl2303_driver_s*)driver)->dev;
  FAR struct pl2303_req_s *reqcontainer;
  irqstate_t flags;
  uint16_t reqlen;
  int ret;
  int i;

  usbtrace(TRACE_CLASSBIND, 0);

  /* Bind the structures */

  priv->usbdev   = dev;

  /* Save the reference to our private data structure in EP0 so that it
   * can be recovered in ep0 completion events (Unless we are part of
   * a composite device and, in that case, the composite device owns
   * EP0).
   */

  dev->ep0->priv = priv;

  /* Preallocate control request */

  priv->ctrlreq = usbclass_allocreq(dev->ep0, PL2303_MXDESCLEN);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCCTRLREQ), 0);
      ret = -ENOMEM;
      goto errout;
    }
  priv->ctrlreq->callback = usbclass_ep0incomplete;

  /* Pre-allocate all endpoints... the endpoints will not be functional
   * until the SET CONFIGURATION request is processed in usbclass_setconfig.
   * This is done here because there may be calls to kmalloc and the SET
   * CONFIGURATION processing probably occurrs within interrupt handling
   * logic where kmalloc calls will fail.
   */

  /* Pre-allocate the IN interrupt endpoint */

  priv->epintin = DEV_ALLOCEP(dev, PL2303_EPINTIN_ADDR, true, USB_EP_ATTR_XFER_INT);
  if (!priv->epintin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }
  priv->epintin->priv = priv;

  /* Pre-allocate the IN bulk endpoint */

  priv->epbulkin = DEV_ALLOCEP(dev, PL2303_EPINBULK_ADDR, true, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }
  priv->epbulkin->priv = priv;

  /* Pre-allocate the OUT bulk endpoint */

  priv->epbulkout = DEV_ALLOCEP(dev, PL2303_EPOUTBULK_ADDR, false, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkout)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }
  priv->epbulkout->priv = priv;

  /* Pre-allocate read requests */

#ifdef CONFIG_PL2303_BULKREQLEN
  reqlen = max(CONFIG_PL2303_BULKREQLEN, priv->epbulkout->maxpacket);
#else
  reqlen = priv->epbulkout->maxpacket;
#endif

  for (i = 0; i < CONFIG_PL2303_NRDREQS; i++)
    {
      reqcontainer      = &priv->rdreqs[i];
      reqcontainer->req = usbclass_allocreq(priv->epbulkout, reqlen);
      if (reqcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), -ENOMEM);
          ret = -ENOMEM;
          goto errout;
        }
      reqcontainer->req->priv     = reqcontainer;
      reqcontainer->req->callback = usbclass_rdcomplete;
    }

  /* Pre-allocate write request containers and put in a free list */

#ifdef CONFIG_PL2303_BULKREQLEN
  reqlen = max(CONFIG_PL2303_BULKREQLEN, priv->epbulkin->maxpacket);
#else
  reqlen = priv->epbulkin->maxpacket;
#endif

  for (i = 0; i < CONFIG_PL2303_NWRREQS; i++)
    {
      reqcontainer      = &priv->wrreqs[i];
      reqcontainer->req = usbclass_allocreq(priv->epbulkin, reqlen);
      if (reqcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRALLOCREQ), -ENOMEM);
          ret = -ENOMEM;
          goto errout;
        }
      reqcontainer->req->priv     = reqcontainer;
      reqcontainer->req->callback = usbclass_wrcomplete;

      flags = irqsave();
      sq_addlast((sq_entry_t*)reqcontainer, &priv->reqlist);
      priv->nwrq++;     /* Count of write requests available */
      irqrestore(flags);
    }

  /* Report if we are selfpowered */

#ifdef CONFIG_USBDEV_SELFPOWERED
  DEV_SETSELFPOWERED(dev);
#endif

  /* And pull-up the data line for the soft connect function */

  DEV_CONNECT(dev);
  return OK;

errout:
  usbclass_unbind(driver, dev);
  return ret;
}

/****************************************************************************
 * Name: usbclass_unbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev)
{
  FAR struct pl2303_dev_s *priv;
  FAR struct pl2303_req_s *reqcontainer;
  irqstate_t flags;
  int i;

  usbtrace(TRACE_CLASSUNBIND, 0);

#ifdef CONFIG_DEBUG
  if (!driver || !dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct pl2303_driver_s*)driver)->dev;

#ifdef CONFIG_DEBUG
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Make sure that we are not already unbound */

  if (priv != NULL)
    {
      /* Make sure that the endpoints have been unconfigured.  If
       * we were terminated gracefully, then the configuration should
       * already have been reset.  If not, then calling usbclass_resetconfig
       * should cause the endpoints to immediately terminate all
       * transfers and return the requests to us (with result == -ESHUTDOWN)
       */

      usbclass_resetconfig(priv);
      up_mdelay(50);

      /* Free the interrupt IN endpoint */

      if (priv->epintin)
        {
          DEV_FREEEP(dev, priv->epintin);
          priv->epintin = NULL;
        }

      /* Free the bulk IN endpoint */

      if (priv->epbulkin)
        {
          DEV_FREEEP(dev, priv->epbulkin);
          priv->epbulkin = NULL;
        }

      /* Free the pre-allocated control request */

      if (priv->ctrlreq != NULL)
        {
          usbclass_freereq(dev->ep0, priv->ctrlreq);
          priv->ctrlreq = NULL;
        }

      /* Free pre-allocated read requests (which should all have
       * been returned to the free list at this time -- we don't check)
       */

      DEBUGASSERT(priv->nrdq == 0);
      for (i = 0; i < CONFIG_PL2303_NRDREQS; i++)
        {
          reqcontainer = &priv->rdreqs[i];
          if (reqcontainer->req)
            {
              usbclass_freereq(priv->epbulkout, reqcontainer->req);
              reqcontainer->req = NULL;
            }
        }

      /* Free the bulk OUT endpoint */

      if (priv->epbulkout)
        {
          DEV_FREEEP(dev, priv->epbulkout);
          priv->epbulkout = NULL;
        }

      /* Free write requests that are not in use (which should be all
       * of them
       */

      flags = irqsave();
      DEBUGASSERT(priv->nwrq == CONFIG_PL2303_NWRREQS);
      while (!sq_empty(&priv->reqlist))
        {
          reqcontainer = (struct pl2303_req_s *)sq_remfirst(&priv->reqlist);
          if (reqcontainer->req != NULL)
            {
              usbclass_freereq(priv->epbulkin, reqcontainer->req);
              priv->nwrq--;     /* Number of write requests queued */
            }
        }
      DEBUGASSERT(priv->nwrq == 0);
      irqrestore(flags);
    }

  /* Clear out all data in the circular buffer */

  priv->serdev.xmit.head = 0;
  priv->serdev.xmit.tail = 0;
}

/****************************************************************************
 * Name: usbclass_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev,
                          FAR const struct usb_ctrlreq_s *ctrl,
                          FAR uint8_t *dataout, size_t outlen)
{
  FAR struct pl2303_dev_s *priv;
  FAR struct usbdev_req_s *ctrlreq;
  uint16_t value;
  uint16_t index;
  uint16_t len;
  int ret = -EOPNOTSUPP;

#ifdef CONFIG_DEBUG
  if (!driver || !dev || !dev->ep0 || !ctrl)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
     }
#endif

  /* Extract reference to private data */

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = ((FAR struct pl2303_driver_s*)driver)->dev;

#ifdef CONFIG_DEBUG
  if (!priv || !priv->ctrlreq)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return -ENODEV;
    }
#endif
  ctrlreq = priv->ctrlreq;

  /* Extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  index = GETUINT16(ctrl->index);
  len   = GETUINT16(ctrl->len);

  uvdbg("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, index, len);

  switch (ctrl->type & USB_REQ_TYPE_MASK)
    {
     /***********************************************************************
      * Standard Requests
      ***********************************************************************/

    case USB_REQ_TYPE_STANDARD:
      {
        switch (ctrl->req)
          {
          case USB_REQ_GETDESCRIPTOR:
            {
              /* The value field specifies the descriptor type in the MS byte and the
               * descriptor index in the LS byte (order is little endian)
               */

              switch (ctrl->value[1])
                {
                case USB_DESC_TYPE_DEVICE:
                  {
                    ret = USB_SIZEOF_DEVDESC;
                    memcpy(ctrlreq->buf, &g_devdesc, ret);
                  }
                  break;

#ifdef CONFIG_USBDEV_DUALSPEED
                case USB_DESC_TYPE_DEVICEQUALIFIER:
                  {
                    ret = USB_SIZEOF_QUALDESC;
                    memcpy(ctrlreq->buf, &g_qualdesc, ret);
                  }
                  break;

                case USB_DESC_TYPE_OTHERSPEEDCONFIG:
#endif /* CONFIG_USBDEV_DUALSPEED */

                case USB_DESC_TYPE_CONFIG:
                  {
#ifdef CONFIG_USBDEV_DUALSPEED
                    ret = usbclass_mkcfgdesc(ctrlreq->buf, dev->speed, ctrl->req);
#else
                    ret = usbclass_mkcfgdesc(ctrlreq->buf);
#endif
                  }
                  break;

                case USB_DESC_TYPE_STRING:
                  {
                    /* index == language code. */

                    ret = usbclass_mkstrdesc(ctrl->value[0], (struct usb_strdesc_s *)ctrlreq->buf);
                  }
                  break;

                default:
                  {
                    usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_GETUNKNOWNDESC), value);
                  }
                  break;
                }
            }
            break;

          case USB_REQ_SETCONFIGURATION:
            {
              if (ctrl->type == 0)
                {
                  ret = usbclass_setconfig(priv, value);
                }
            }
            break;

          case USB_REQ_GETCONFIGURATION:
            {
              if (ctrl->type == USB_DIR_IN)
                {
                  *(uint8_t*)ctrlreq->buf = priv->config;
                  ret = 1;
                }
            }
            break;

          case USB_REQ_SETINTERFACE:
            {
              if (ctrl->type == USB_REQ_RECIPIENT_INTERFACE)
                {
                  if (priv->config == PL2303_CONFIGID &&
                      index == PL2303_INTERFACEID &&
                      value == PL2303_ALTINTERFACEID)
                    {
                      usbclass_resetconfig(priv);
                      usbclass_setconfig(priv, priv->config);
                      ret = 0;
                    }
                }
            }
            break;

          case USB_REQ_GETINTERFACE:
            {
              if (ctrl->type == (USB_DIR_IN|USB_REQ_RECIPIENT_INTERFACE) &&
                  priv->config == PL2303_CONFIGIDNONE)
                {
                  if (index != PL2303_INTERFACEID)
                    {
                      ret = -EDOM;
                    }
                  else
                     {
                      *(uint8_t*) ctrlreq->buf = PL2303_ALTINTERFACEID;
                      ret = 1;
                    }
                }
             }
             break;

          default:
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDSTDREQ), ctrl->req);
            break;
          }
      }
      break;

     /***********************************************************************
      * PL2303 Vendor-Specific Requests
      ***********************************************************************/

    case PL2303_CONTROL_TYPE:
      {
        if ((ctrl->type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_INTERFACE)
          {
            switch (ctrl->req)
              {
              case PL2303_SETLINEREQUEST:
                {
                   memcpy(priv->linest, ctrlreq->buf, min(len, 7));
                   ret = 0;
                }
                break;


              case PL2303_GETLINEREQUEST:
                {
                   memcpy(ctrlreq->buf, priv->linest, 7);
                   ret = 7;
                }
                break;

              case PL2303_SETCONTROLREQUEST:
              case PL2303_BREAKREQUEST:
                {
                  ret = 0;
                }
                break;

              default:
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ), ctrl->type);
                break;
              }
          }
      }
      break;

    case PL2303_RWREQUEST_TYPE:
      {
        if ((ctrl->type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
          {
            if (ctrl->req == PL2303_RWREQUEST)
              {
                if ((ctrl->type & USB_DIR_IN) != 0)
                  {
                    *(uint32_t*)ctrlreq->buf = 0xdeadbeef;
                    ret = 4;
                  }
                else
                  {
                     ret = 0;
                  }
              }
            else
              {
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ), ctrl->type);
              }
          }
      }
      break;

    default:
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDTYPE), ctrl->type);
      break;
    }

  /* Respond to the setup command if data was returned.  On an error return
   * value (ret < 0), the USB driver will stall.
   */

  if (ret >= 0)
    {
      ctrlreq->len   = min(len, ret);
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;
      ret            = EP_SUBMIT(dev->ep0, ctrlreq);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPRESPQ), (uint16_t)-ret);
          ctrlreq->result = OK;
          usbclass_ep0incomplete(dev->ep0, ctrlreq);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: usbclass_disconnect
 *
 * Description:
 *   Invoked after all transfers have been stopped, when the host is
 *   disconnected.  This function is probably called from the context of an
 *   interrupt handler.
 *
 ****************************************************************************/

static void usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev)
{
  FAR struct pl2303_dev_s *priv;
  irqstate_t flags;

  usbtrace(TRACE_CLASSDISCONNECT, 0);

#ifdef CONFIG_DEBUG
  if (!driver || !dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct pl2303_driver_s*)driver)->dev;

#ifdef CONFIG_DEBUG
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Inform the "upper half serial driver that we have lost the USB serial
   * connection.
   */

  flags = irqsave();
#ifdef CONFIG_SERIAL_REMOVABLE
  uart_connected(&priv->serdev, false);
#endif

  /* Reset the configuration */

  usbclass_resetconfig(priv);

  /* Clear out all outgoing data in the circular buffer */

  priv->serdev.xmit.head = 0;
  priv->serdev.xmit.tail = 0;
  irqrestore(flags);

  /* Perform the soft connect function so that we will we can be
   * re-enumerated.
   */

  DEV_CONNECT(dev);
}

/****************************************************************************
 * Serial Device Methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbser_setup
 *
 * Description:
 *   This method is called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int usbser_setup(FAR struct uart_dev_s *dev)
{
  FAR struct pl2303_dev_s *priv;

  usbtrace(PL2303_CLASSAPI_SETUP, 0);

  /* Sanity check */

#if CONFIG_DEBUG
  if (!dev || !dev->priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
    }
#endif

  /* Extract reference to private data */

  priv = (FAR struct pl2303_dev_s*)dev->priv;

  /* Check if we have been configured */

  if (priv->config == PL2303_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SETUPNOTCONNECTED), 0);
      return -ENOTCONN;
    }

  return OK;
}

/****************************************************************************
 * Name: usbser_shutdown
 *
 * Description:
 *   This method is called when the serial port is closed.  This operation
 *   is very simple for the USB serial backend because the serial driver
 *   has already assured that the TX data has full drained -- it calls
 *   usbser_txempty() until that function returns true before calling this
 *   function.
 *
 ****************************************************************************/

static void usbser_shutdown(FAR struct uart_dev_s *dev)
{
  usbtrace(PL2303_CLASSAPI_SHUTDOWN, 0);

  /* Sanity check */

#if CONFIG_DEBUG
  if (!dev || !dev->priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
    }
#endif
}

/****************************************************************************
 * Name: usbser_attach
 *
 * Description:
 *   Does not apply to the USB serial class device
 *
 ****************************************************************************/

static int usbser_attach(FAR struct uart_dev_s *dev)
{
  usbtrace(PL2303_CLASSAPI_ATTACH, 0);
  return OK;
}

/****************************************************************************
 * Name: usbser_detach
 *
 * Description:
*   Does not apply to the USB serial class device
  *
 ****************************************************************************/

static void usbser_detach(FAR struct uart_dev_s *dev)
{
  usbtrace(PL2303_CLASSAPI_DETACH, 0);
}

/****************************************************************************
 * Name: usbser_rxint
 *
 * Description:
 *   Called by the serial driver to enable or disable RX interrupts.  We, of
 *   course, have no RX interrupts but must behave consistently.  This method
 *   is called under the conditions:
 *
 *   1. With enable==true when the port is opened (just after usbser_setup
 *      and usbser_attach are called called)
 *   2. With enable==false while transferring data from the RX buffer
 *   2. With enable==true while waiting for more incoming data
 *   3. With enable==false when the port is closed (just before usbser_detach
 *      and usbser_shutdown are called).
 *
 ****************************************************************************/

static void usbser_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct pl2303_dev_s *priv;
  FAR uart_dev_t *serdev;
  irqstate_t flags;

  usbtrace(PL2303_CLASSAPI_RXINT, (uint16_t)enable);

  /* Sanity check */

#if CONFIG_DEBUG
  if (!dev || !dev->priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
       return;
    }
#endif

  /* Extract reference to private data */

  priv   = (FAR struct pl2303_dev_s*)dev->priv;
  serdev = &priv->serdev;

  /* We need exclusive access to the RX buffer and private structure
   * in the following.
   */

  flags = irqsave();
  if (enable)
    {
       /* RX "interrupts" are enabled.  Is this a transition from disabled
        * to enabled state?
        */

       if (!priv->rxenabled)
         {
           /* Yes.  During the time that RX interrupts are disabled, the
            * the serial driver will be extracting data from the circular
            * buffer and modifying recv.tail.  During this time, we
            * should avoid modifying recv.head; When interrupts are restored,
            * we can update the head pointer for all of the data that we
            * put into cicular buffer while "interrupts" were disabled.
            */

          if (priv->rxhead != serdev->recv.head)
            {
              serdev->recv.head = priv->rxhead;

              /* Yes... signal the availability of new data */

              uart_datareceived(serdev);
            }

          /* RX "interrupts are no longer disabled */

          priv->rxenabled = true;
        }
    }

  /* RX "interrupts" are disabled.  Is this a transition from enabled
   * to disabled state?
   */

  else if (priv->rxenabled)
    {
      /* Yes.  During the time that RX interrupts are disabled, the
       * the serial driver will be extracting data from the circular
       * buffer and modifying recv.tail.  During this time, we
       * should avoid modifying recv.head; When interrupts are disabled,
       * we use a shadow index and continue adding data to the circular
       * buffer.
       */

      priv->rxhead    = serdev->recv.head;
      priv->rxenabled = false;
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: usbser_txint
 *
 * Description:
 *   Called by the serial driver to enable or disable TX interrupts.  We, of
 *   course, have no TX interrupts but must behave consistently.  Initially,
 *   TX interrupts are disabled.  This method is called under the conditions:
 *
 *   1. With enable==false while transferring data into the TX buffer
 *   2. With enable==true when data may be taken from the buffer.
 *   3. With enable==false when the TX buffer is empty
 *
 ****************************************************************************/

static void usbser_txint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct pl2303_dev_s *priv;

  usbtrace(PL2303_CLASSAPI_TXINT, (uint16_t)enable);

  /* Sanity checks */

#if CONFIG_DEBUG
  if (!dev || !dev->priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
       return;
    }
#endif

 /* Extract references to private data */

  priv = (FAR struct pl2303_dev_s*)dev->priv;

  /* If the new state is enabled and if there is data in the XMIT buffer,
   * send the next packet now.
   */

  uvdbg("enable=%d head=%d tail=%d\n",
        enable, priv->serdev.xmit.head, priv->serdev.xmit.tail);

  if (enable && priv->serdev.xmit.head != priv->serdev.xmit.tail)
    {
      usbclass_sndpacket(priv);
    }
}

/****************************************************************************
 * Name: usbser_txempty
 *
 * Description:
 *   Return true when all data has been sent.  This is called from the
 *   serial driver when the driver is closed.  It will call this API
 *   periodically until it reports true.  NOTE that the serial driver takes all
 *   responsibility for flushing TX data through the hardware so we can be
 *   a bit sloppy about that.
 *
 ****************************************************************************/

static bool usbser_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct pl2303_dev_s *priv = (FAR struct pl2303_dev_s*)dev->priv;

  usbtrace(PL2303_CLASSAPI_TXEMPTY, 0);

#if CONFIG_DEBUG
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return true;
    }
#endif

  /* When all of the allocated write requests have been returned to the
   * reqlist, then there is no longer any TX data in flight.
   */

  return priv->nwrq >= CONFIG_PL2303_NWRREQS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_serialinitialize
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured).
 *
 ****************************************************************************/

int usbdev_serialinitialize(int minor)
{
  FAR struct pl2303_alloc_s *alloc;
  FAR struct pl2303_dev_s *priv;
  FAR struct pl2303_driver_s *drvr;
  char devname[16];
  int ret;

  /* Allocate the structures needed */

  alloc = (FAR struct pl2303_alloc_s*)kmalloc(sizeof(struct pl2303_alloc_s));
  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCDEVSTRUCT), 0);
      return -ENOMEM;
    }

  /* Convenience pointers into the allocated blob */

  priv = &alloc->dev;
  drvr = &alloc->drvr;

  /* Initialize the USB serial driver structure */

  memset(priv, 0, sizeof(struct pl2303_dev_s));
  sq_init(&priv->reqlist);

  /* Fake line status */

  priv->linest[0] = (115200) & 0xff;       /* Baud=115200 */
  priv->linest[1] = (115200 >> 8) & 0xff;
  priv->linest[2] = (115200 >> 16) & 0xff;
  priv->linest[3] = (115200 >> 24) & 0xff;
  priv->linest[4] = 0;                     /* One stop bit */
  priv->linest[5] = 0;                     /* No parity */
  priv->linest[6] = 8;                     /*8 data bits */

  /* Initialize the serial driver sub-structure */

#ifdef CONFIG_SERIAL_REMOVABLE
  priv->serdev.disconnected = true;
#endif
  priv->serdev.recv.size    = CONFIG_PL2303_RXBUFSIZE;
  priv->serdev.recv.buffer  = priv->rxbuffer;
  priv->serdev.xmit.size    = CONFIG_PL2303_TXBUFSIZE;
  priv->serdev.xmit.buffer  = priv->txbuffer;
  priv->serdev.ops          = &g_uartops;
  priv->serdev.priv         = priv;

  /* Initialize the USB class driver structure */

#ifdef CONFIG_USBDEV_DUALSPEED
  drvr->drvr.speed         = USB_SPEED_HIGH;
#else
  drvr->drvr.speed         = USB_SPEED_FULL;
#endif
  drvr->drvr.ops           = &g_driverops;
  drvr->dev                = priv;

  /* Register the USB serial class driver */

  ret = usbdev_register(&drvr->drvr);
  if (ret)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_DEVREGISTER), (uint16_t)-ret);
      goto errout_with_alloc;
    }

  /* Register the USB serial console */

#ifdef CONFIG_PL2303_CONSOLE
  priv->serdev.isconsole = true;
  ret = uart_register("/dev/console", &priv->serdev);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONSOLEREGISTER), (uint16_t)-ret);
      goto errout_with_class;
    }
#endif

  /* Register the single port supported by this implementation */

  sprintf(devname, "/dev/ttyUSB%d", minor);
  ret = uart_register(devname, &priv->serdev);
  if (ret)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UARTREGISTER), (uint16_t)-ret);
      goto errout_with_class;
    }
  return OK;

errout_with_class:
  usbdev_unregister(&drvr->drvr);
errout_with_alloc:
  kfree(alloc);
  return ret;
}
