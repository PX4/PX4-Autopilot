/****************************************************************************
 * drivers/usbdev/usbmsc.h
 *
 *   Copyright (C) 2008-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Mass storage class device.  Bulk-only with SCSI subclass.
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

#ifndef __DRIVERS_USBDEV_USBMSC_H
#define __DRIVERS_USBDEV_USBMSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <queue.h>

#include <nuttx/fs/fs.h>
#include <nuttx/usb/storage.h>
#include <nuttx/usb/usbdev.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* If the USB mass storage device is configured as part of a composite device
 * then both CONFIG_USBDEV_COMPOSITE and CONFIG_USBMSC_COMPOSITE must be
 * defined.
 */

#ifndef CONFIG_USBDEV_COMPOSITE
#  undef CONFIG_USBMSC_COMPOSITE
#endif

#if defined(CONFIG_USBMSC_COMPOSITE) && !defined(CONFIG_USBMSC_STRBASE)
#  define CONFIG_USBMSC_STRBASE (4)
#endif

/* Interface IDs.  If the mass storage driver is built as a component of a 
 * composite device, then the interface IDs may need to be offset.
 */

#ifndef CONFIG_USBMSC_COMPOSITE
#  undef CONFIG_USBMSC_IFNOBASE
#  define CONFIG_USBMSC_IFNOBASE 0
#endif

#ifndef CONFIG_USBMSC_IFNOBASE
#  define CONFIG_USBMSC_IFNOBASE 0
#endif

/* Number of requests in the write queue */

#ifndef CONFIG_USBMSC_NWRREQS
#  define CONFIG_USBMSC_NWRREQS 4
#endif

/* Number of requests in the read queue */

#ifndef CONFIG_USBMSC_NRDREQS
#  define CONFIG_USBMSC_NRDREQS 4
#endif

/* Logical endpoint numbers / max packet sizes */

#ifndef CONFIG_USBMSC_EPBULKOUT
#  warning "EPBULKOUT not defined in the configuration"
#  define CONFIG_USBMSC_EPBULKOUT 2
#endif

#ifndef CONFIG_USBMSC_EPBULKIN
#  warning "EPBULKIN not defined in the configuration"
#  define CONFIG_USBMSC_EPBULKIN 3
#endif

/* Packet and request buffer sizes */

#ifndef CONFIG_USBMSC_COMPOSITE
#  ifndef CONFIG_USBMSC_EP0MAXPACKET
#    define CONFIG_USBMSC_EP0MAXPACKET 64
#  endif
#endif

#ifndef CONFIG_USBMSC_BULKINREQLEN
#  ifdef CONFIG_USBDEV_DUALSPEED
#    define CONFIG_USBMSC_BULKINREQLEN 512
#  else
#    define CONFIG_USBMSC_BULKINREQLEN 64
#  endif
#else
#  ifdef CONFIG_USBDEV_DUALSPEED
#    if CONFIG_USBMSC_BULKINREQLEN < 512
#      warning "Bulk in buffer size smaller than max packet"
#      undef CONFIG_USBMSC_BULKINREQLEN
#      define CONFIG_USBMSC_BULKINREQLEN 512
#    endif
#  else
#    if CONFIG_USBMSC_BULKINREQLEN < 64
#      warning "Bulk in buffer size smaller than max packet"
#      undef CONFIG_USBMSC_BULKINREQLEN
#      define CONFIG_USBMSC_BULKINREQLEN 64
#    endif
#  endif
#endif

#ifndef CONFIG_USBMSC_BULKOUTREQLEN
#  ifdef CONFIG_USBDEV_DUALSPEED
#    define CONFIG_USBMSC_BULKOUTREQLEN 512
#  else
#    define CONFIG_USBMSC_BULKOUTREQLEN 64
#  endif
#else
#  ifdef CONFIG_USBDEV_DUALSPEED
#    if CONFIG_USBMSC_BULKOUTREQLEN < 512
#      warning "Bulk in buffer size smaller than max packet"
#      undef CONFIG_USBMSC_BULKOUTREQLEN
#      define CONFIG_USBMSC_BULKOUTREQLEN 512
#    endif
#  else
#    if CONFIG_USBMSC_BULKOUTREQLEN < 64
#      warning "Bulk in buffer size smaller than max packet"
#      undef CONFIG_USBMSC_BULKOUTREQLEN
#      define CONFIG_USBMSC_BULKOUTREQLEN 64
#    endif
#  endif
#endif

/* Vendor and product IDs and strings */

#ifndef CONFIG_USBMSC_COMPOSITE
#  ifndef CONFIG_USBMSC_VENDORID
#    warning "CONFIG_USBMSC_VENDORID not defined"
#    define CONFIG_USBMSC_VENDORID 0x584e
#  endif

#  ifndef CONFIG_USBMSC_PRODUCTID
#    warning "CONFIG_USBMSC_PRODUCTID not defined"
#    define CONFIG_USBMSC_PRODUCTID 0x5342
#  endif

#  ifndef CONFIG_USBMSC_VERSIONNO
#    define CONFIG_USBMSC_VERSIONNO (0x0399)
#  endif

#  ifndef CONFIG_USBMSC_VENDORSTR
#    warning "No Vendor string specified"
#    define CONFIG_USBMSC_VENDORSTR  "NuttX"
# endif

#  ifndef CONFIG_USBMSC_PRODUCTSTR
#    warning "No Product string specified"
#    define CONFIG_USBMSC_PRODUCTSTR "USBdev Storage"
#  endif

#  undef CONFIG_USBMSC_SERIALSTR
#  define CONFIG_USBMSC_SERIALSTR "0101"
#endif

#undef CONFIG_USBMSC_CONFIGSTR
#define CONFIG_USBMSC_CONFIGSTR "Bulk"

/* Debug -- must be consistent with include/debug.h */

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    ifdef CONFIG_ARCH_LOWPUTC
#      define dbgprintf(format, arg...) lowsyslog(format, ##arg)
#    else
#      define dbgprintf(format, arg...) syslog(format, ##arg)
#     endif
#  else
#      define dbgprintf(x...)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    ifdef CONFIG_ARCH_LOWPUTC
#      define dbgprintf lowsyslog
#    else
#      define dbgprintf syslog
#     endif
#  else
#      define dbgprintf (void)
#  endif
#endif

/* Packet and request buffer sizes */

#ifndef CONFIG_USBMSC_EP0MAXPACKET
#  define CONFIG_USBMSC_EP0MAXPACKET 64
#endif

/* USB Controller */

#ifndef CONFIG_USBDEV_SELFPOWERED
#  define SELFPOWERED USB_CONFIG_ATTR_SELFPOWER
#else
#  define SELFPOWERED (0)
#endif

#ifndef  CONFIG_USBDEV_REMOTEWAKEUP
#  define REMOTEWAKEUP USB_CONFIG_ATTR_WAKEUP
#else
#  define REMOTEWAKEUP (0)
#endif

#ifndef CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100
#endif

/* Current state of the worker thread */

#define USBMSC_STATE_NOTSTARTED       (0)  /* Thread has not yet been started */
#define USBMSC_STATE_STARTED          (1)  /* Started, but is not yet initialized */
#define USBMSC_STATE_IDLE             (2)  /* Started and waiting for commands */
#define USBMSC_STATE_CMDPARSE         (3)  /* Processing a received command */
#define USBMSC_STATE_CMDREAD          (4)  /* Processing a SCSI read command */
#define USBMSC_STATE_CMDWRITE         (5)  /* Processing a SCSI write command */
#define USBMSC_STATE_CMDFINISH        (6)  /* Finish command processing */
#define USBMSC_STATE_CMDSTATUS        (7)  /* Processing the final status of the command */
#define USBMSC_STATE_TERMINATED       (8)  /* Thread has exitted */

/* Event communicated to worker thread */

#define USBMSC_EVENT_NOEVENTS         (0)      /* There are no outstanding events */
#define USBMSC_EVENT_READY            (1 << 0) /* Initialization is complete */
#define USBMSC_EVENT_RDCOMPLETE       (1 << 1) /* A read has completed there is data to be processed */
#define USBMSC_EVENT_WRCOMPLETE       (1 << 2) /* A write has completed and a request is available */
#define USBMSC_EVENT_TERMINATEREQUEST (1 << 3) /* Shutdown requested */
#define USBMSC_EVENT_DISCONNECT       (1 << 4) /* USB disconnect received */
#define USBMSC_EVENT_RESET            (1 << 5) /* USB storage setup reset received */
#define USBMSC_EVENT_CFGCHANGE        (1 << 6) /* USB setup configuration change received */
#define USBMSC_EVENT_IFCHANGE         (1 << 7) /* USB setup interface change received */
#define USBMSC_EVENT_ABORTBULKOUT     (1 << 8) /* SCSI receive failure */

/* SCSI command flags (passed to usbmsc_setupcmd()) */

#define USBMSC_FLAGS_DIRMASK          (0x03) /* Bits 0-1: Data direction */
#define USBMSC_FLAGS_DIRNONE          (0x00) /*   No data to send */
#define USBMSC_FLAGS_DIRHOST2DEVICE   (0x01) /*   Host-to-device */
#define USBMSC_FLAGS_DIRDEVICE2HOST   (0x02) /*   Device-to-host */
#define USBMSC_FLAGS_BLOCKXFR         (0x04) /* Bit 2: Command is a block transfer request */
#define USBMSC_FLAGS_LUNNOTNEEDED     (0x08) /* Bit 3: Command does not require a valid LUN */
#define USBMSC_FLAGS_UACOKAY          (0x10) /* Bit 4: Command OK if unit attention condition */
#define USBMSC_FLAGS_RETAINSENSEDATA  (0x20) /* Bit 5: Do not clear sense data */

/* Descriptors **************************************************************/

/* Big enough to hold our biggest descriptor */

#define USBMSC_MXDESCLEN              (64)

/* String language */

#define USBMSC_STR_LANGUAGE           (0x0409) /* en-us */

/* Descriptor strings */

#ifndef CONFIG_USBMSC_COMPOSITE
#  define USBMSC_MANUFACTURERSTRID    (1)
#  define USBMSC_PRODUCTSTRID         (2)
#  define USBMSC_SERIALSTRID          (3)
#  define USBMSC_CONFIGSTRID          (4)
#  define USBMSC_INTERFACESTRID       USBMSC_CONFIGSTRID

#  undef CONFIG_USBMSC_STRBASE
#  define CONFIG_USBMSC_STRBASE       (0)
#else
#  define USBMSC_INTERFACESTRID       (CONFIG_USBMSC_STRBASE+1)
#endif

#define USBMSC_LASTSTRID              USBMSC_INTERFACESTRID
#define USBMSC_NSTRIDS               (USBMSC_LASTSTRID - CONFIG_USBMSC_STRBASE)

#define USBMSC_NCONFIGS               (1) /* Number of configurations supported */

/* Configuration Descriptor */

#define USBMSC_NINTERFACES            (1) /* Number of interfaces in the configuration */
#define USBMSC_INTERFACEID            (CONFIG_USBMSC_IFNOBASE+0)
#define USBMSC_ALTINTERFACEID         (0)

#define USBMSC_CONFIGIDNONE           (0) /* Config ID means to return to address mode */
#define USBMSC_CONFIGID               (1) /* The only supported configuration ID */

/* Interface description */

#define USBMSC_NENDPOINTS             (2) /* Number of endpoints in the interface  */

/* Endpoint configuration */

#define USBMSC_EPOUTBULK_ADDR         (CONFIG_USBMSC_EPBULKOUT)
#define USBMSC_EPOUTBULK_ATTR         (USB_EP_ATTR_XFER_BULK)

#define USBMSC_EPINBULK_ADDR          (USB_DIR_IN|CONFIG_USBMSC_EPBULKIN)
#define USBMSC_EPINBULK_ATTR          (USB_EP_ATTR_XFER_BULK)

#define USBMSC_HSBULKMAXPACKET        (512)
#define USBMSC_HSBULKMXPKTSHIFT       (9)
#define USBMSC_HSBULKMXPKTMASK        (0x000001ff)
#define USBMSC_FSBULKMAXPACKET        (64)
#define USBMSC_FSBULKMXPKTSHIFT       (6)
#define USBMSC_FSBULKMXPKTMASK        (0x0000003f)

/* Macros for dual speed vs. full speed only operation */

#ifdef CONFIG_USBDEV_DUALSPEED
#  define USBMSC_EPBULKINDESC(hs)  \
   usbmsc_getepdesc((hs) ? USBMSC_EPHSBULKIN : USBMSC_EPFSBULKIN)
#  define USBMSC_EPBULKOUTDESC(hs) \
   usbmsc_getepdesc((hs) ? USBMSC_EPHSBULKOUT : USBMSC_EPFSBULKOUT)
#  define USBMSC_BULKMAXPACKET(hs) \
   ((hs) ? USBMSC_HSBULKMAXPACKET : USBMSC_FSBULKMAXPACKET)
#  define USBMSC_BULKMXPKTSHIFT(d) \
   (((d)->speed==USB_SPEED_HIGH) ? USBMSC_HSBULKMXPKTSHIFT : USBMSC_FSBULKMXPKTSHIFT)
#  define USBMSC_BULKMXPKTMASK(d) \
   (((d)->speed==USB_SPEED_HIGH) ? USBMSC_HSBULKMXPKTMASK : USBMSC_FSBULKMXPKTMASK)
#else
#  define USBMSC_EPBULKINDESC(d)    usbmsc_getepdesc(USBMSC_EPFSBULKIN)
#  define USBMSC_EPBULKOUTDESC(d)   usbmsc_getepdesc(USBMSC_EPFSBULKOUT)
#  define USBMSC_BULKMAXPACKET(hs)  USBMSC_FSBULKMAXPACKET
#  define USBMSC_BULKMXPKTSHIFT(d)  USBMSC_FSBULKMXPKTSHIFT
#  define USBMSC_BULKMXPKTMASK(d)   USBMSC_FSBULKMXPKTMASK
#endif

/* Configuration descriptor size */

#ifndef CONFIG_USBMSC_COMPOSITE

/* Number of individual descriptors in the configuration descriptor:
 * (1) Configuration descriptor + (1) interface descriptor + (2) interface
 * descriptors.
 */

#  define USBMSC_CFGGROUP_SIZE      (4)

/* The size of the config descriptor: (9 + 9 + 2*7) = 32 */

#  define SIZEOF_USBMSC_CFGDESC \
     (USB_SIZEOF_CFGDESC + USB_SIZEOF_IFDESC + USBMSC_NENDPOINTS * USB_SIZEOF_EPDESC)

#else

/* Number of individual descriptors in the configuration descriptor:
 * (1) interface descriptor + (2) interface descriptors.
 */

#  define USBMSC_CFGGROUP_SIZE     (3)

/* The size of the config descriptor: (9 + 2*7) = 23 */

#  define SIZEOF_USBMSC_CFGDESC \
     (USB_SIZEOF_IFDESC + USBMSC_NENDPOINTS * USB_SIZEOF_EPDESC)

#endif

/* Block driver helpers *****************************************************/

#define USBMSC_DRVR_READ(l,b,s,n) ((l)->inode->u.i_bops->read((l)->inode,b,s,n))
#define USBMSC_DRVR_WRITE(l,b,s,n) ((l)->inode->u.i_bops->write((l)->inode,b,s,n))
#define USBMSC_DRVR_GEOMETRY(l,g) ((l)->inode->u.i_bops->geometry((l)->inode,g))

/* Everpresent MIN/MAX macros ***********************************************/

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* Endpoint descriptors */

enum usbmsc_epdesc_e
{
  USBMSC_EPFSBULKOUT = 0, /* Full speed bulk OUT endpoint descriptor */
  USBMSC_EPFSBULKIN       /* Full speed bulk IN endpoint descriptor */
#ifdef CONFIG_USBDEV_DUALSPEED
  , 
  USBMSC_EPHSBULKOUT,     /* High speed bulk OUT endpoint descriptor */
  USBMSC_EPHSBULKIN       /* High speed bulk IN endpoint descriptor */
#endif
};

/* Container to support a list of requests */

struct usbmsc_req_s
{
  FAR struct usbmsc_req_s *flink;     /* Implements a singly linked list */
  FAR struct usbdev_req_s *req;       /* The contained request */
};

/* This structure describes one LUN: */

struct usbmsc_lun_s
{
  struct inode    *inode;             /* Inode structure of open'ed block driver */
  uint8_t          readonly:1;        /* Media is read-only */
  uint8_t          locked:1;          /* Media removal is prevented */
  uint16_t         sectorsize;        /* The size of one sector */
  uint32_t         sd;                /* Sense data */
  uint32_t         sdinfo;            /* Sense data information */
  uint32_t         uad;               /* Unit needs attention data */
  off_t            startsector;       /* Sector offset to start of partition */
  size_t           nsectors;          /* Number of sectors in the partition */
};

/* Describes the overall state of the driver */

struct usbmsc_dev_s
{
  FAR struct usbdev_s *usbdev;        /* usbdev driver pointer (Non-null if registered) */

  /* Worker thread interface */

  pthread_t         thread;           /* The worker thread */
  pthread_mutex_t   mutex;            /* Mutually exclusive access to resources*/
  pthread_cond_t    cond;             /* Used to signal worker thread */
  volatile uint8_t  thstate;          /* State of the worker thread */
  volatile uint16_t theventset;       /* Set of pending events signaled to worker thread */
  volatile uint8_t  thvalue;          /* Value passed with the event (must persist) */

  /* Storage class configuration and state */

  uint8_t           nluns:4;          /* Number of LUNs */
  uint8_t           config;           /* Configuration number */

  /* Endpoints */

  FAR struct usbdev_ep_s  *epbulkin;  /* Bulk IN endpoint structure */
  FAR struct usbdev_ep_s  *epbulkout; /* Bulk OUT endpoint structure */
  FAR struct usbdev_req_s *ctrlreq;   /* Control request (for ep0 setup responses) */

  /* SCSI command processing */

  struct usbmsc_lun_s *lun;           /* Currently selected LUN */
  struct usbmsc_lun_s *luntab;        /* Allocated table of all LUNs */
  uint8_t cdb[USBMSC_MAXCDBLEN];     /* Command data (cdb[]) from CBW */
  uint8_t           phaseerror:1;     /* Need to send phase sensing status */
  uint8_t           shortpacket:1;    /* Host transmission stopped unexpectedly */
  uint8_t           cbwdir:2;         /* Direction from CBW. See USBMSC_FLAGS_DIR* definitions */
  uint8_t           cdblen;           /* Length of cdb[] from CBW */
  uint8_t           cbwlun;           /* LUN from the CBW */
  uint16_t          nsectbytes;       /* Bytes buffered in iobuffer[] */
  uint16_t          nreqbytes;        /* Bytes buffered in head write requests */
  uint16_t          iosize;           /* Size of iobuffer[] */
  uint32_t          cbwlen;           /* Length of data from CBW */
  uint32_t          cbwtag;           /* Tag from the CBW */
  union
    {
      uint32_t      xfrlen;           /* Read/Write: Sectors remaining to be transferred */
      uint32_t      alloclen;         /* Other device-to-host: Host allocation length */
    } u;
  uint32_t          sector;           /* Current sector (relative to lun->startsector) */
  uint32_t          residue;          /* Untransferred amount reported in the CSW */
  uint8_t          *iobuffer;         /* Buffer for data transfers */

  /* Write request list */

  struct sq_queue_s wrreqlist;        /* List of empty write request containers */
  struct sq_queue_s rdreqlist;        /* List of filled read request containers */

  /* Pre-allocated write request containers.  The write requests will
   * be linked in a free list (wrreqlist), and used to send requests to
   * EPBULKIN; Read requests will be queued in the EBULKOUT.
   */

  struct usbmsc_req_s    wrreqs[CONFIG_USBMSC_NWRREQS];
  struct usbmsc_req_s    rdreqs[CONFIG_USBMSC_NRDREQS];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/* String *******************************************************************/

/* Mass storage class vendor/product/serial number strings */

#ifndef CONFIG_USBMSC_COMPOSITE
EXTERN const char g_mscvendorstr[];
EXTERN const char g_mscproductstr[];
EXTERN const char g_mscserialstr[];

/* If we are using a composite device, then vendor/product/serial number strings
 * are provided by the composite device logic.
 */

#else
EXTERN const char g_compvendorstr[];
EXTERN const char g_compproductstr[];
EXTERN const char g_compserialstr[];

#define g_mscvendorstr  g_compvendorstr
#define g_mscproductstr g_compproductstr
#define g_mscserialstr  g_compserialstr
#endif
/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: usbmsc_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ************************************************************************************/

struct usb_strdesc_s;
int usbmsc_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc);

/************************************************************************************
 * Name: usbmsc_getepdesc
 *
 * Description:
 *   Return a pointer to the raw device descriptor
 *
 ************************************************************************************/

#ifndef CONFIG_USBMSC_COMPOSITE
FAR const struct usb_devdesc_s *usbmsc_getdevdesc(void);
#endif

/************************************************************************************
 * Name: usbmsc_getepdesc
 *
 * Description:
 *   Return a pointer to the raw endpoint descriptor (used for configuring endpoints)
 *
 ************************************************************************************/

struct usb_epdesc_s;
FAR const struct usb_epdesc_s *usbmsc_getepdesc(enum usbmsc_epdesc_e epid);

/************************************************************************************
 * Name: usbmsc_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ************************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
int16_t usbmsc_mkcfgdesc(FAR uint8_t *buf, uint8_t speed, uint8_t type);
#else
int16_t usbmsc_mkcfgdesc(FAR uint8_t *buf);
#endif

/************************************************************************************
 * Name: usbmsc_getqualdesc
 *
 * Description:
 *   Return a pointer to the raw qual descriptor
 *
 ************************************************************************************/

#if !defined(CONFIG_USBMSC_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
FAR const struct usb_qualdesc_s *usbmsc_getqualdesc(void);
#endif

/****************************************************************************
 * Name: usbmsc_workerthread
 *
 * Description:
 *   This is the main function of the USB storage worker thread.  It loops
 *   until USB-related events occur, then processes those events accordingly
 *
 ****************************************************************************/

EXTERN void *usbmsc_workerthread(void *arg);

/****************************************************************************
 * Name: usbmsc_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

EXTERN int usbmsc_setconfig(FAR struct usbmsc_dev_s *priv, uint8_t config);

/****************************************************************************
 * Name: usbmsc_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

EXTERN void usbmsc_resetconfig(FAR struct usbmsc_dev_s *priv);

/****************************************************************************
 * Name: usbmsc_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

EXTERN void usbmsc_wrcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req);

/****************************************************************************
 * Name: usbmsc_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.  This
 *   is handled like the receipt of serial data on the "UART"
 *
 ****************************************************************************/

EXTERN void usbmsc_rdcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req);

/****************************************************************************
 * Name: usbmsc_deferredresponse
 *
 * Description:
 *   Some EP0 setup request cannot be responded to immediately becuase they
 *   require some asynchronous action from the SCSI worker thread.  This
 *   function is provided for the SCSI thread to make that deferred response.
 *   The specific requests that require this deferred response are:
 *
 *   1. USB_REQ_SETCONFIGURATION,
 *   2. USB_REQ_SETINTERFACE, or
 *   3. USBMSC_REQ_MSRESET
 *
 *   In all cases, the success reponse is a zero-length packet; the failure
 *   response is an EP0 stall.
 *
 * Input parameters:
 *   priv  - Private state structure for this USB storage instance
 *   stall - true is the action failed and a stall is required
 *
 ****************************************************************************/

EXTERN void usbmsc_deferredresponse(FAR struct usbmsc_dev_s *priv, bool failed);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* #define __DRIVERS_USBDEV_USBMSC_H */
