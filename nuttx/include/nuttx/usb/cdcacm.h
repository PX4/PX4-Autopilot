/****************************************************************************
 * include/nuttx/usb/cdcacm.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_USB_CDCACM_H
#define __INCLUDE_NUTTX_USB_CDCACM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/usb/usb.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_CDCACM
 *   Enable compilation of the USB serial driver
 * CONFIG_CDCACM_EP0MAXPACKET
 *   Endpoint 0 max packet size. Default 64.
 * CONFIG_CDCACM_EPINTIN
 *   The logical 7-bit address of a hardware endpoint that supports
 *   interrupt IN operation.  Default 1.
 * CONFIG_CDCACM_EPINTIN_FSSIZE
 *   Max package size for the interrupt IN endpoint if full speed mode.
 *   Default 64.
 * CONFIG_CDCACM_EPINTIN_HSSIZE
 *   Max package size for the interrupt IN endpoint if high speed mode.
 *   Default 64.
 * CONFIG_CDCACM_EPBULKOUT
 *   The logical 7-bit address of a hardware endpoint that supports
 *   bulk OUT operation.  Default: 3
 * CONFIG_CDCACM_EPBULKOUT_FSSIZE
 *   Max package size for the bulk OUT endpoint if full speed mode.
 *   Default 64.
 * CONFIG_CDCACM_EPBULKOUT_HSSIZE
 *   Max package size for the bulk OUT endpoint if high speed mode.
 *   Default 512.
 * CONFIG_CDCACM_EPBULKIN
 *   The logical 7-bit address of a hardware endpoint that supports
 *   bulk IN operation.  Default: 2
 * CONFIG_CDCACM_EPBULKIN_FSSIZE
 *   Max package size for the bulk IN endpoint if full speed mode.
 *   Default 64.
 * CONFIG_CDCACM_EPBULKIN_HSSIZE
 *   Max package size for the bulk IN endpoint if high speed mode.
 *   Default 512.
 * CONFIG_CDCACM_NWRREQS and CONFIG_CDCACM_NRDREQS
 *   The number of write/read requests that can be in flight.
 *   CONFIG_CDCACM_NWRREQS includes write requests used for both the
 *   interrupt and bulk IN endpoints.  Default 4.
 * CONFIG_CDCACM_VENDORID and CONFIG_CDCACM_VENDORSTR
 *   The vendor ID code/string.  Default 0x0525 and "NuttX"
 *   0x0525 is the Netchip vendor and should not be used in any
 *   products.  This default VID was selected for compatibility with
 *   the Linux CDC ACM default VID.
 * CONFIG_CDCACM_PRODUCTID and CONFIG_CDCACM_PRODUCTSTR
 *   The product ID code/string. Default 0xa4a7 and "CDC/ACM Serial"
 *   0xa4a7 was selected for compatibility with the Linux CDC ACM
 *   default PID.
 * CONFIG_CDCACM_RXBUFSIZE and CONFIG_CDCACM_TXBUFSIZE
 *   Size of the serial receive/transmit buffers. Default 256.
 */

/* EP0 max packet size */

#ifndef CONFIG_CDCACM_EP0MAXPACKET
#  define CONFIG_CDCACM_EP0MAXPACKET 64
#endif

/* Endpoint number and size (in bytes) of the CDC serial device-to-host (IN)
 * notification interrupt endpoint.
 */

#ifndef CONFIG_CDCACM_EPINTIN
#  define CONFIG_CDCACM_EPINTIN 1
#endif

#ifndef CONFIG_CDCACM_EPINTIN_FSSIZE
#  define CONFIG_CDCACM_EPINTIN_FSSIZE 64
#endif

#ifndef CONFIG_CDCACM_EPINTIN_HSSIZE
#  define CONFIG_CDCACM_EPINTIN_HSSIZE 64
#endif

/* Endpoint number and size (in bytes) of the CDC device-to-host (IN) data
 * bulk endpoint.  NOTE that difference sizes may be selected for full (FS)
 * or high speed (HS) modes.
 */

#ifndef CONFIG_CDCACM_EPBULKIN
#  define CONFIG_CDCACM_EPBULKIN 2
#endif

#ifndef CONFIG_CDCACM_EPBULKIN_FSSIZE
#  define CONFIG_CDCACM_EPBULKIN_FSSIZE 64
#endif

#ifndef CONFIG_CDCACM_EPBULKIN_HSSIZE
#  define CONFIG_CDCACM_EPBULKIN_HSSIZE 512
#endif

/* Endpoint number and size (in bytes) of the CDC host-to-device (OUT) data
 * bulk endpoint.  NOTE that difference sizes may be selected for full (FS)
 * or high speed (HS) modes.
 */

#ifndef CONFIG_CDCACM_EPBULKOUT
#  define CONFIG_CDCACM_EPBULKOUT 3
#endif

#ifndef CONFIG_CDCACM_EPBULKOUT_FSSIZE
#  define CONFIG_CDCACM_EPBULKOUT_FSSIZE 64
#endif

#ifndef CONFIG_CDCACM_EPBULKOUT_HSSIZE
#  define CONFIG_CDCACM_EPBULKOUT_HSSIZE 512
#endif

/* Number of requests in the write queue.  This includes write requests used
 * for both the interrupt and bulk IN endpoints.
 */

#ifndef CONFIG_CDCACM_NWRREQS
#  define CONFIG_CDCACM_NWRREQS 4
#endif

/* Number of requests in the read queue */

#ifndef CONFIG_CDCACM_NRDREQS
#  define CONFIG_CDCACM_NRDREQS 4
#endif

/* TX/RX buffer sizes */

#ifndef CONFIG_CDCACM_RXBUFSIZE
#  define CONFIG_CDCACM_RXBUFSIZE 256
#endif

#ifndef CONFIG_CDCACM_TXBUFSIZE
#  define CONFIG_CDCACM_TXBUFSIZE 256
#endif

/* Vendor and product IDs and strings.  The default is the Linux Netchip
 * CDC ACM VID and PID.
 */

#ifndef CONFIG_CDCACM_VENDORID
#  define CONFIG_CDCACM_VENDORID  0x0525
#endif

#ifndef CONFIG_CDCACM_PRODUCTID
#  define CONFIG_CDCACM_PRODUCTID 0xa4a7
#endif

#ifndef CONFIG_CDCACM_VENDORSTR
#  define CONFIG_CDCACM_VENDORSTR  "NuttX"
#endif

#ifndef CONFIG_CDCACM_PRODUCTSTR
#  define CONFIG_CDCACM_PRODUCTSTR "CDC ACM Serial"
#endif

#undef CONFIG_CDCACM_SERIALSTR
#define CONFIG_CDCACM_SERIALSTR "0"

#undef CONFIG_CDCACM_CONFIGSTR
#define CONFIG_CDCACM_CONFIGSTR "Bulk"

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

/* IOCTL Commands ***********************************************************/
/* The USB serial driver will support a subset of the TIOC IOCTL commands
 * defined in include/nuttx/serial/tioctl.h.  This subset includes:
 *
 * CAICO_REGISTERCB
 *   Register a callback for serial event notification. Argument:
 *   cdcacm_callback_t.  See cdcacm_callback_t type definition below.
 *   NOTE:  The callback will most likely invoked at the interrupt level.
 *   The called back function should, therefore, limit its operations to
 *   invoking some kind of IPC to handle the serial event in some normal
 *   task environment.
 * CAIOC_GETLINECODING
 *   Get current line coding.  Argument: struct cdc_linecoding_s*.
 *   See include/nuttx/usb/cdc.h for structure definition.  This IOCTL
 *   should be called to get the data associated with the
 *   CDCACM_EVENT_LINECODING event (see devent definition below).
 * CAIOC_GETCTRLLINE
 *   Get control line status bits. Argument FAR int*.  See
 *   include/nuttx/usb/cdc.h for bit definitions.  This IOCTL should be
 *   called to get the data associated CDCACM_EVENT_CTRLLINE event (see event
 *   definition below).
 * CAIOC_NOTIFY
 *   Send a serial state to the host via the Interrupt IN endpoint.
 *   Argument: int.  This includes the current state of the carrier detect,
 *   DSR, break, and ring signal.  See "Table 69: UART State Bitmap Values"
 *   and CDC_UART_definitions in include/nuttx/usb/cdc.h.
 */

#define CAIOC_REGISTERCB    _CAIOC(0x0001)
#define CAIOC_GETLINECODING _CAIOC(0x0002)
#define CAIOC_GETCTRLLINE   _CAIOC(0x0003)
#define CAIOC_NOTIFY        _CAIOC(0x0004)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C" {
#else
# define EXTERN extern
#endif

/* Reported serial events.  Data is associated with CDCACM_EVENT_LINECODING
 * and CDCACM_EVENT_CTRLLINE.  The data may be obtained using CDCACM IOCTL
 * commands described above.
 *
 * CDCACM_EVENT_LINECODING - See "Table 50: Line Coding Structure" and struct
 *   cdc_linecoding_s in include/nuttx/usb/cdc.h.
 * CDCACM_EVENT_CTRLLINE - See "Table 51: Control Signal Bitmap Values for
 *   SetControlLineState" and definitions in include/nutt/usb/cdc.h
 * CDCACM_EVENT_SENDBREAK - See Paragraph "6.2.15 SendBreak." This request
 *   sends special carrier modulation that generates an RS-232 style break.
 */

enum cdcacm_event_e
{
  CDCACM_EVENT_LINECODING = 0, /* New line coding received from host */
  CDCACM_EVENT_CTRLLINE,       /* New control line status received from host */
  CDCACM_EVENT_SENDBREAK       /* Send break request received */
};

typedef FAR void (*cdcacm_callback_t)(enum cdcacm_event_e event);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_cdcclassobject
 *
 * Description:
 *   If the CDC serial class driver is part of composite device, then
 *   board-specific logic must provide board_cdcclassobject().  In the simplest
 *   case, board_cdcclassobject() is simply a wrapper around cdcacm_classobject()
 *   that provides the correct device minor number.
 *
 * Input Parameters:
 *   classdev - The location to return the CDC serial class' device
 *     instance.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_CDCACM_COMPOSITE)
struct usbdevclass_driver_s;
EXTERN int board_cdcclassobject(FAR struct usbdevclass_driver_s **classdev);
#endif

/****************************************************************************
 * Name: board_cdcuninitialize
 *
 * Description:
 *   Un-initialize the USB serial class driver.  This is just an application-
 *   specific wrapper aboutn cdcadm_unitialize() that is called form the composite
 *   device logic.
 *
 * Input Parameters:
 *   classdev - The class driver instrance previously give to the composite
 *     driver by board_cdcclassobject().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_CDCACM_COMPOSITE)
struct usbdevclass_driver_s;
EXTERN void board_cdcuninitialize(FAR struct usbdevclass_driver_s *classdev);
#endif

/****************************************************************************
 * Name: cdcacm_classobject
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured) and
 *   return the class object.
 *
 * Input Parameter:
 *   minor - Device minor number.  E.g., minor 0 would correspond to
 *     /dev/ttyACM0.
 *   classdev - The location to return the CDC serial class' device
 *     instance.
 *
 * Returned Value:
 *   A pointer to the allocated class object (NULL on failure).
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_CDCACM_COMPOSITE)
int cdcacm_classobject(int minor, FAR struct usbdevclass_driver_s **classdev);
#endif

/****************************************************************************
 * Name: cdcacm_initialize
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured).
 *
 * Input Parameter:
 *   minor - Device minor number.  E.g., minor 0 would correspond to
 *     /dev/ttyACM0.
 *   handle - An optional opaque reference to the CDC/ACM class object that
 *     may subsequently be used with cdcacm_uninitialize().
 *
 * Returned Value:
 *   Zero (OK) means that the driver was successfully registered.  On any
 *   failure, a negated errno value is retured.
 *
 ****************************************************************************/

#if !defined(CONFIG_USBDEV_COMPOSITE) || !defined(CONFIG_CDCACM_COMPOSITE)
EXTERN int cdcacm_initialize(int minor, FAR void **handle);
#endif

/****************************************************************************
 * Name: cdcacm_uninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver.  This function is used
 *   internally by the USB composite driver to unitialized the CDC/ACM
 *   driver.  This same interface is available (with an untyped input
 *   parameter) when the CDC/ACM driver is used standalone.
 *
 * Input Parameters:
 *   There is one parameter, it differs in typing depending upon whether the
 *   CDC/ACM driver is an internal part of a composite device, or a standalone
 *   USB driver:
 *
 *     classdev - The class object returned by board_cdcclassobject() or
 *       cdcacm_classobject()
 *     handle - The opaque handle represetning the class object returned by
 *       a previous call to cdcacm_initialize().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_CDCACM_COMPOSITE)
EXTERN void cdcacm_uninitialize(FAR struct usbdevclass_driver_s *classdev);
#else
EXTERN void cdcacm_uninitialize(FAR void *handle);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_CDCACM_H */
