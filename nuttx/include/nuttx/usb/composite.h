/************************************************************************************
 * include/nuttx/usb/composite.h
 *
 *   Copyright (C) 2008-2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_USB_COMPOSITE_H
#define __INCLUDE_NUTTX_USB_COMPOSITE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_USBDEV_COMPOSITE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_USBDEV_COMPOSITE
 *   Enables USB composite device support
 * CONFIG_CDCACM_COMPOSITE
 *   Configure the CDC serial driver as part of a composite driver
 *   (only if CONFIG_USBDEV_COMPOSITE is also defined)
 * CONFIG_COMPOSITE_COMPOSITE
 *   Configure the mass storage driver as part of a composite driver
 *   (only if CONFIG_USBDEV_COMPOSITE is also defined)
 * CONFIG_COMPOSITE_EP0MAXPACKET
 *   Max packet size for endpoint 0
 * CONFIG_COMPOSITE_VENDORID and CONFIG_COMPOSITE_VENDORSTR
 *   The vendor ID code/string
 * CONFIG_COMPOSITE_PRODUCTID and CONFIG_COMPOSITE_PRODUCTSTR
 *   The product ID code/string
 * CONFIG_COMPOSITE_SERIALSTR
 *   Device serial number string
 * CONFIG_COMPOSITE_CONFIGSTR
 *   Configuration string
 * CONFIG_COMPOSITE_VERSIONNO
 *   Interface version number.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: composite_initialize
 *
 * Description:
 *   Register USB composite device as configured.  This function will call
 *   board-specific implementations in order to obtain the class objects for
 *   each of the members of the composite (see board_mscclassobject(),
 *   board_cdcclassobjec(), ...) 
 *
 * Input Parameter:
 *   None
 *
 * Returned Value:
 *   A non-NULL "handle" is returned on success.  This handle may be used
 *   later with composite_uninitialize() in order to removed the composite
 *   device.  This handle is the (untyped) internal representation of the
 *   the class driver instance.
 *
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

EXTERN FAR void *composite_initialize(void);

/****************************************************************************
 * Name: composite_uninitialize
 *
 * Description:
 *   Un-initialize the USB composite driver.  The handle is the USB composite
 *   class' device object as was returned by composite_initialize().  This
 *   function will call  board-specific implementations in order to free the
 *   class objects for each of the members of the composite (see
 *   board_mscuninitialize(), board_cdcuninitialize(), ...) 
 *
 * Input Parameters:
 *   handle - The handle returned by a previous call to composite_initialize().
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

EXTERN void composite_uninitialize(FAR void *handle);

/****************************************************************************
 * Name: composite_initialize
 *
 * Description:
 *   Register USB composite device as configured.  This function will call
 *   board-specific implementations in order to obtain the class objects for
 *   each of the members of the composite (see board_mscclassobject(),
 *   board_cdcclassobjec(), ...) 
 *
 * Input Parameter:
 *   None
 *
 * Returned Value:
 *   A non-NULL "handle" is returned on success.  This handle may be used
 *   later with composite_uninitialize() in order to removed the composite
 *   device.  This handle is the (untyped) internal representation of the
 *   the class driver instance.
 *
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

EXTERN FAR void *composite_initialize(void);

/****************************************************************************
 * Name: composite_ep0submit
 *
 * Description:
 *   Members of the composite cannot send on EP0 directly because EP0 is
 *   is "owned" by the composite device.  Instead, when configured as members
 *   of a composite device, those classes should call this method so that
 *   the composite device can send on EP0 onbehalf of the class.
 *
 ****************************************************************************/

struct usbdevclass_driver_s;
struct usbdev_s;
struct usbdev_req_s;

EXTERN int composite_ep0submit(FAR struct usbdevclass_driver_s *driver,
                               FAR struct usbdev_s *dev,
                               FAR struct usbdev_req_s *ctrlreq);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_USBDEV_COMPOSITE */
#endif /* __INCLUDE_NUTTX_USB_COMPOSITE_H */
