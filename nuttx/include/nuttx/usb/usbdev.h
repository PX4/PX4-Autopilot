/************************************************************************************
 * include/nuttx/usb/usbdev.h
 *
 *   Copyright (C) 2008-2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * NOTE:  This interface was inspired by the Linux gadget interface by
 * David Brownell. That work was very helpful in determining a usable
 * partitioning of functionality between standard class drivers and various
 * implementations of USB controller drivers.  This work, however, does
 * not derive directly from that work and is licensed differently.
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
 ************************************************************************************/

#ifndef _INCLUDE_NUTTX_USB_USBDEV_H
#define _INCLUDE_NUTTX_USB_USBDEV_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Endpoint helpers *****************************************************************/

/* Configure endpoint, making it usable.  The class driver may deallocate or re-use
 * the 'desc' structure after returning:
 *
 * ep   - the struct usbdev_ep_s instance obtained from allocep()
 * desc - A struct usb_epdesc_s instance describing the endpoint
 * last - true if this this last endpoint to be configured.  Some hardware needs
 *        to take special action when all of the endpoints have been configured.
 */

#define EP_CONFIGURE(ep,desc,last) (ep)->ops->configure(ep,desc,last)

/* The endpoint will no longer be used */

#define EP_DISABLE(ep)             (ep)->ops->disable(ep)

/* Allocate/free I/O requests.  Should not be called from interrupt processing! */

#define EP_ALLOCREQ(ep)            (ep)->ops->allocreq(ep)
#define EP_FREEREQ(ep,req)         (ep)->ops->freereq(ep,req)

/* Allocate/free an I/O buffer.  Should not be called from interrupt processing! */

#ifdef CONFIG_ARCH_USBDEV_DMA
#  define EP_ALLOCBUFFER(ep,nb)    (ep)->ops->alloc(ep,nb)
#  define EP_FREEBUFFER(ep,buff)   (ep)->ops->free(ep,buf)
#else
#  define EP_ALLOCBUFFER(ep,nb)    malloc(nb)
#  define EP_FREEBUFFER(ep,buf)    free(buf)
#endif

/* Submit an I/O request to the endpoint */

#define EP_SUBMIT(ep,req)          (ep)->ops->submit(ep,req)

/* Cancel an I/O request previously sent to an endpoint */

#define EP_CANCEL(ep,req)          (ep)->ops->cancel(ep,req)

/* Stall or resume an endpoint */

#define EP_STALL(ep)               (ep)->ops->stall(ep,false)
#define EP_RESUME(ep)              (ep)->ops->stall(ep,true)

/* USB Device Driver Helpers ********************************************************/

/* Allocate an endpoint:
 *
 *   ep     - 7-bit logical endpoint number (direction bit ignored).  Zero means
 *            that any endpoint matching the other requirements will suffice.  The
 *            assigned endpoint can be found in the eplog field.
 *   in     - true: IN (device-to-host) endpoint requested
 *   eptype - Endpoint type.  One of {USB_EP_ATTR_XFER_ISOC, USB_EP_ATTR_XFER_BULK,
 *            USB_EP_ATTR_XFER_INT}
 */

#define DEV_ALLOCEP(dev,ep,in,type) (dev)->ops->allocep(dev,ep,in,type)

/* Release an endpoint */

#define DEV_FREEEP(dev,ep)         (dev)->ops->freeep(dev,ep)

/* Returns the current frame number */

#define DEV_GETFRAME(dev)          (dev)->ops->getframe(dev)

/* Tries to wake up the host connected to this device */

#define DEV_WAKEUP(dev)            (dev)->ops->wakeup(dev)

/* Sets the device selfpowered feature */

#define DEV_SETSELFPOWERED(dev)    (dev)->ops->selfpowered(dev,true)

/* Clears the device selfpowered feature */

#define DEV_CLRSELFPOWERED(dev)    (dev)->ops->selfpowered(dev, false)

/* Software-controlled connect to USB host. All USB class drivers need to call
 * DEV_CONNECT() when they are ready to be enumerated.  That is, (1) initially when
 * bound to the USB driver, and (2) after a USB reset.
 */

#define DEV_CONNECT(dev)           (dev)->ops->pullup ? (dev)->ops->pullup(dev,true) : -EOPNOTSUPP

/* Software-controlled disconnect from USB host */

#define DEV_DISCONNECT(dev)        (dev)->ops->pullup ? (dev)->ops->pullup(dev,false) : -EOPNOTSUPP

/* USB Class Driver Helpers *********************************************************/
/* All may be called from interupt handling logic except bind() and unbind() */

/* Invoked when the driver is bound to a USB device driver. */

#define CLASS_BIND(drvr,dev)      (drvr)->ops->bind(dev, drvr)

/* Invoked when the driver is unbound from a USB device driver */

#define CLASS_UNBIND(drvr,dev)    (drvr)->ops->unbind(dev)

/* Invoked after all transfers have been stopped, when the host is disconnected. */

#define CLASS_DISCONNECT(drvr,dev) (drvr)->ops->disconnect(dev)

/* Invoked for ep0 control requests */

#define CLASS_SETUP(drvr,dev,ctrl) (drvr)->ops->setup(dev, ctrl)

/* Invoked on USB suspend. */

#define CLASS_SUSPEND(drvr,dev)   \
  do { if ((drvr)->ops->suspend) (drvr)->ops->suspend(dev); } while (0)

/* Invoked on USB resume */

#define CLASS_RESUME(drvr,dev)  \
  do { if ((drvr)->ops->resume) (drvr)->ops->resume(dev); } while (0)

/* Device speeds */

#define USB_SPEED_UNKNOWN         0 /* Transfer rate not yet set */
#define USB_SPEED_LOW             1 /* USB 1.1 */
#define USB_SPEED_FULL            2 /* USB 1.1 */
#define USB_SPEED_HIGH            3 /* USB 2.0 */
#define USB_SPEED_VARIABLE        4 /* Wireless USB 2.5 */

/* Request flags */

#define USBDEV_REQFLAGS_NULLPKT   1 /* Bit 0: Terminate w/short packet; null packet if necessary */
                                    /* Bits 1-7: Available */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* USB Controller Structures ********************************************************/

/* struct usbdev_req_s - describes one i/o request */

struct usbdev_ep_s;
struct usbdev_req_s
{
  uint8_t *buf;    /* Call: Buffer used for data; Return: Unchanged */
  uint8_t  flags;  /* See USBDEV_REQFLAGS_* definitions */
  uint16_t len;    /* Call: Total length of data in buf; Return: Unchanged */
  uint16_t xfrd;   /* Call: zero; Return: Bytes transferred so far */
  int16_t  result; /* Call: zero; Return: Result of transfer (O or -errno) */

  /* Callback when the transfer completes */

  void   (*callback)(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req);
  void    *priv;    /* Used only by callee */
};

/* Endpoint-specific interface to USB controller hardware. */

struct usbdev_epops_s
{
  /* Configure/enable and disable endpoint */

  int (*configure)(FAR struct usbdev_ep_s *ep, FAR const struct usb_epdesc_s *desc,
                   bool last);
  int (*disable)(FAR struct usbdev_ep_s *ep);

  /* Allocate and free I/O requests */

  FAR struct usbdev_req_s *(*allocreq)(FAR struct usbdev_ep_s *ep);
  void (*freereq)(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req);

  /* Allocate and free I/O buffers */

#ifdef CONFIG_ARCH_USBDEV_DMA
  FAR void *(*allocbuffer)(FAR struct usbdev_ep_s *ep, uint16_t nbytes);
  void (*freebuffer)(FAR struct usbdev_ep_s *ep, FAR void *buf);
#endif

  /* Submit and cancel I/O requests */

  int (*submit)(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req);
  int (*cancel)(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req);

  /* Stall or resume an endpoint */

  int (*stall)(FAR struct usbdev_ep_s *ep, bool resume);
};

/* Representation of one USB endpoint */

struct usbdev_ep_s
{
  const struct usbdev_epops_s *ops; /* Endpoint operations */
  uint8_t  eplog;                   /* Logical endpoint address */
  uint16_t maxpacket;               /* Maximum packet size for this endpoint */
  void    *priv;                    /* For use by class driver */
};

/* struct usbdev_s represents a usb device */

struct usbdev_s;
struct usbdev_ops_s
{
  /* Allocate and free endpoints */

  FAR struct usbdev_ep_s *(*allocep)(FAR struct usbdev_s *dev, uint8_t epphy, bool in, uint8_t eptype);
  void (*freeep)(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep);

  /* Get the frame number from the last SOF */

  int (*getframe)(FAR struct usbdev_s *dev);

  /* Hardware specific features */

  int (*wakeup)(FAR struct usbdev_s *dev);
  int (*selfpowered)(FAR struct usbdev_s *dev, bool selfpowered);
  int (*pullup)(FAR struct usbdev_s *dev,  bool enable);

  /* Device-specific I/O command support */

  int (*ioctl)(FAR struct usbdev_s *dev, unsigned code, unsigned long param);
};

struct usbdev_s
{
  const struct usbdev_ops_s *ops; /* Access to hardware specific features */
  struct usbdev_ep_s *ep0;        /* Endpoint zero */
  uint8_t  speed;                 /* Current speed of host connection */
  uint8_t  dualspeed:1;           /* 1:supports high and full speed operation */
};

/* USB Device Class Implementations *************************************************/

struct usbdevclass_driver_s;
struct usbdevclass_driverops_s
{
  int  (*bind)(FAR struct usbdev_s *dev, FAR struct usbdevclass_driver_s *driver);
  void (*unbind)(FAR struct usbdev_s *dev);
  int  (*setup)(FAR struct usbdev_s *dev, const struct usb_ctrlreq_s *ctrl);
  void (*disconnect)(FAR struct usbdev_s *dev);
  void (*suspend)(FAR struct usbdev_s *dev);
  void (*resume)(FAR struct usbdev_s *dev);
};

struct usbdevclass_driver_s
{
  const struct usbdevclass_driverops_s *ops;
  uint8_t speed;                  /* Highest speed that the driver handles */
};

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: usbdevclass_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method will be
 *   called to bind it to a USB device driver.
 *
 ************************************************************************************/

EXTERN int usbdev_register(FAR struct usbdevclass_driver_s *driver);

/************************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver.If the USB device is connected to a USB host,
 *   it will first disconnect().  The driver is also requested to unbind() and clean
 *   up any device state, before this procedure finally returns.
 *
 ************************************************************************************/

EXTERN int usbdev_unregister(FAR struct usbdevclass_driver_s *driver);

/****************************************************************************
 * Name: usbdev_serialinit
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured).
 *
 ****************************************************************************/

EXTERN int usbdev_serialinitialize(int minor);

/****************************************************************************
 * Name: board_mscclassobject
 *
 * Description:
 *   If the mass storage class driver is part of composite device, then
 *   its instantiation and configuration is a multi-step, board-specific,
 *   process (See comments for usbstrg_configure below).  In this case,
 *   board-specific logic must provide board_mscclassobject().
 *
 *   board_mscclassobject() is called from the composite driver.  It must
 *   encapsulate the instantiation and configuration of the mass storage
 *   class and the return the mass storage device's class driver instance
 *   to the composite dirver.
 *
 * Input Parameters:
 *   classdev - The location to return the mass storage class' device
 *     instance.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_USBSTRG_COMPOSITE)
EXTERN int board_mscclassobject(FAR struct usbdevclass_driver_s **classdev);
#endif

/****************************************************************************
 * Name: usbstrg_configure
 *
 * Description:
 *   One-time initialization of the USB storage driver.  The initialization
 *   sequence is as follows:
 *
 *   1. Call usbstrg_configure to perform one-time initialization specifying
 *      the number of luns.
 *   2. Call usbstrg_bindlun to configure each supported LUN
 *   3. Call usbstrg_exportluns when all LUNs are configured
 *
 * Input Parameters:
 *   nluns  - the number of LUNs that will be registered
 *   handle - Location to return a handle that is used in other API calls.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

EXTERN int usbstrg_configure(unsigned int nluns, void **handle);

/****************************************************************************
 * Name: usbstrg_bindlun
 *
 * Description:
 *   Bind the block driver specified by drvrpath to a USB storage LUN.
 *
 * Input Parameters:
 *   handle      - The handle returned by a previous call to usbstrg_configure().
 *   drvrpath    - the full path to the block driver
 *   startsector - A sector offset into the block driver to the start of the
 *                 partition on drvrpath (0 if no partitions)
 *   nsectors    - The number of sectors in the partition (if 0, all sectors
 *                 to the end of the media will be exported).
 *   lunno       - the LUN to bind to
 *
 * Returned Value:
 *  0 on success; a negated errno on failure.
 *
 ****************************************************************************/

EXTERN int usbstrg_bindlun(FAR void *handle, FAR const char *drvrpath,
                           unsigned int lunno, off_t startsector, size_t nsectors,
                            bool readonly);

/****************************************************************************
 * Name: usbstrg_unbindlun
 *
 * Description:
 *   Un-bind the block driver for the specified LUN
 *
 * Input Parameters:
 *   handle - The handle returned by a previous call to usbstrg_configure().
 *   lun    - the LUN to unbind from
 *
 * Returned Value:
 *  0 on success; a negated errno on failure.
 *
 ****************************************************************************/

EXTERN int usbstrg_unbindlun(FAR void *handle, unsigned int lunno);

/****************************************************************************
 * Name: usbstrg_exportluns
 *
 * Description:
 *   After all of the LUNs have been bound, this function may be called
 *   in order to export those LUNs in the USB storage device.
 *
 * Input Parameters:
 *   handle - The handle returned by a previous call to usbstrg_configure().
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

EXTERN int usbstrg_exportluns(FAR void *handle);

/****************************************************************************
 * Name: cdcser_classobject
 *
 * Description:
 *   .
 *
 * Input Parameters:
 *   handle - The handle returned by a previous call to usbstrg_configure().
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_USBSTRG_COMPOSITE)
EXTERN int usbstrg_classobject(FAR void *handle, FAR struct usbdevclass_driver_s **classdev);
#endif

/****************************************************************************
 * Name: usbstrg_uninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver
 *
 * Input Parameters:
 *   handle - The handle returned by a previous call to usbstrg_configure().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

EXTERN void usbstrg_uninitialize(FAR void *handle);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* _INCLUDE_NUTTX_USB_USBDEV_H */
