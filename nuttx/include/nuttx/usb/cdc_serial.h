/****************************************************************************
 * include/nuttx/usb/cdc_serial.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __INCLUDE_NUTTX_USB_CDC_SERIAL_H
#define __INCLUDE_NUTTX_USB_CDC_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/ioctl.h>
#include <nuttx/usb/usb.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_CDCSER
 *   Enable compilation of the USB serial driver
 * CONFIG_CDCSER_EP0MAXPACKET
 *   Endpoint 0 max packet size. Default 64.
 * CONFIG_CDCSER_EPINTIN
 *   The logical 7-bit address of a hardware endpoint that supports
 *   interrupt IN operation.  Default 2.
 * CONFIG_CDCSER_EPINTIN_FSSIZE
 *   Max package size for the interrupt IN endpoint if full speed mode.
 *   Default 64.
 * CONFIG_CDCSER_EPINTIN_HSSIZE
 *   Max package size for the interrupt IN endpoint if high speed mode.
 *   Default 64.
 * CONFIG_CDCSER_EPBULKOUT
 *   The logical 7-bit address of a hardware endpoint that supports
 *   bulk OUT operation
 * CONFIG_CDCSER_EPBULKOUT_FSSIZE
 *   Max package size for the bulk OUT endpoint if full speed mode.
 *   Default 64.
 * CONFIG_CDCSER_EPBULKOUT_HSSIZE
 *   Max package size for the bulk OUT endpoint if high speed mode.
 *   Default 512.
 * CONFIG_CDCSER_EPBULKIN
 *   The logical 7-bit address of a hardware endpoint that supports
 *   bulk IN operation
 * CONFIG_CDCSER_EPBULKIN_FSSIZE
 *   Max package size for the bulk IN endpoint if full speed mode.
 *   Default 64.
 * CONFIG_CDCSER_EPBULKIN_HSSIZE
 *   Max package size for the bulk IN endpoint if high speed mode.
 *   Default 512.
 * CONFIG_CDCSER_NWRREQS and CONFIG_CDCSER_NRDREQS
 *   The number of write/read requests that can be in flight.
 *   CONFIG_CDCSER_NWRREQS includes write requests used for both the
 *   interrupt and bulk IN endpoints.  Default 4.
 * CONFIG_CDCSER_VENDORID and CONFIG_CDCSER_VENDORSTR
 *   The vendor ID code/string.  Default 0x0525 and "NuttX"
 *   0x0525 is the Netchip vendor and should not be used in any
 *   products.  This default VID was selected for compatibility with
 *   the Linux CDC ACM default VID.
 * CONFIG_CDCSER_PRODUCTID and CONFIG_CDCSER_PRODUCTSTR
 *   The product ID code/string. Default 0xa4a7 and "CDC/ACM Serial"
 *   0xa4a7 was selected for compatibility with the Linux CDC ACM
 *   default PID.
 * CONFIG_CDCSER_RXBUFSIZE and CONFIG_CDCSER_TXBUFSIZE
 *   Size of the serial receive/transmit buffers. Default 256.
 */

/* EP0 max packet size */

#ifndef CONFIG_CDCSER_EP0MAXPACKET
#  define CONFIG_CDCSER_EP0MAXPACKET 64
#endif

/* Endpoint number and size (in bytes) of the CDC serial device-to-host (IN)
 * notification interrupt endpoint.
 */

#ifndef CONFIG_CDCSER_EPINTIN
#  define CONFIG_CDCSER_EPINTIN 2
#endif

#ifndef CONFIG_CDCSER_EPINTIN_FSSIZE
#  define CONFIG_CDCSER_EPINTIN_FSSIZE 64
#endif

#ifndef CONFIG_CDCSER_EPINTIN_HSSIZE
#  define CONFIG_CDCSER_EPINTIN_HSSIZE 64
#endif

/* Endpoint number and size (in bytes) of the CDC device-to-host (IN) data
 * bulk endpoint.  NOTE that difference sizes may be selected for full (FS)
 * or high speed (HS) modes.
 */

#ifndef CONFIG_CDCSER_EPBULKIN
#  define CONFIG_CDCSER_EPBULKIN 3
#endif

#ifndef CONFIG_CDCSER_EPBULKIN_FSSIZE
#  define CONFIG_CDCSER_EPBULKIN_FSSIZE 64
#endif

#ifndef CONFIG_CDCSER_EPBULKIN_HSSIZE
#  define CONFIG_CDCSER_EPBULKIN_HSSIZE 512
#endif

/* Endpoint number and size (in bytes) of the CDC host-to-device (OUT) data
 * bulk endpoint.  NOTE that difference sizes may be selected for full (FS)
 * or high speed (HS) modes.
 */

#ifndef CONFIG_CDCSER_EPBULKOUT
#  define CONFIG_CDCSER_EPBULKOUT 4
#endif

#ifndef CONFIG_CDCSER_EPBULKOUT_FSSIZE
#  define CONFIG_CDCSER_EPBULKOUT_FSSIZE 64
#endif

#ifndef CONFIG_CDCSER_EPBULKOUT_HSSIZE
#  define CONFIG_CDCSER_EPBULKOUT_HSSIZE 512
#endif

/* Number of requests in the write queue.  This includes write requests used
 * for both the interrupt and bulk IN endpoints.
 */

#ifndef CONFIG_CDCSER_NWRREQS
#  define CONFIG_CDCSER_NWRREQS 4
#endif

/* Number of requests in the read queue */

#ifndef CONFIG_CDCSER_NRDREQS
#  define CONFIG_CDCSER_NRDREQS 4
#endif

/* TX/RX buffer sizes */

#ifndef CONFIG_CDCSER_RXBUFSIZE
#  define CONFIG_CDCSER_RXBUFSIZE 256
#endif

#ifndef CONFIG_CDCSER_TXBUFSIZE
#  define CONFIG_CDCSER_TXBUFSIZE 256
#endif

/* Vendor and product IDs and strings.  The default is the Linux Netchip
 * CDC ACM VID and PID.
 */

#ifndef CONFIG_CDCSER_VENDORID
#  define CONFIG_CDCSER_VENDORID  0x0525
#endif

#ifndef CONFIG_CDCSER_PRODUCTID
#  define CONFIG_CDCSER_PRODUCTID 0xa4a7
#endif

#ifndef CONFIG_CDCSER_VENDORSTR
#  define CONFIG_CDCSER_VENDORSTR  "NuttX"
#endif

#ifndef CONFIG_CDCSER_PRODUCTSTR
#  define CONFIG_CDCSER_PRODUCTSTR "CDC ACM Serial"
#endif

#undef CONFIG_CDCSER_SERIALSTR
#define CONFIG_CDCSER_SERIALSTR "0"

#undef CONFIG_CDCSER_CONFIGSTR
#define CONFIG_CDCSER_CONFIGSTR "Bulk"

/* USB Controller */

#ifndef CONFIG_USBDEV_SELFPOWERED
#  define SELFPOWERED USB_CONFIG_ATT_SELFPOWER
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
 * defined in include/nuttx/tioctl.h.  This subset includes:
 *
 * CAICO_REGISTERCB
 *   Register a callback for serial event notification. Argument:
 *   cdcser_callback_t.  See cdcser_callback_t type definition below.
 *   NOTE:  The callback will most likely invoked at the interrupt level.
 *   The called back function should, therefore, limit its operations to
 *   invoking some kind of IPC to handle the serial event in some normal
 *   task environment.
 * CAIOC_GETLINECODING
 *   Get current line coding.  Argument: struct cdc_linecoding_s*.
 *   See include/nuttx/usb/cdc.h for structure definition.  This IOCTL
 *   should be called to get the data associated with the
 *   CDCSER_EVENT_LINECODING event (see devent definition below).
 * CAIOC_GETCTRLLINE
 *   Get control line status bits. Argument FAR int*.  See
 *   include/nuttx/usb/cdc.h for bit definitions.  This IOCTL should be
 *   called to get the data associated CDCSER_EVENT_CTRLLINE event (see event
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

/* Reported serial events.  Data is associated with CDCSER_EVENT_LINECODING
 * and CDCSER_EVENT_CTRLLINE.  The data may be obtained using CDCSER IOCTL
 * commands described above.
 *
 * CDCSER_EVENT_LINECODING - See "Table 50: Line Coding Structure" and struct
 *   cdc_linecoding_s in include/nuttx/usb/cdc.h.
 * CDCSER_EVENT_CTRLLINE - See "Table 51: Control Signal Bitmap Values for
 *   SetControlLineState" and definitions in include/nutt/usb/cdc.h
 * CDCSER_EVENT_SENDBREAK - See Paragraph "6.2.15 SendBreak." This request
 *   sends special carrier modulation that generates an RS-232 style break.
 */

enum cdcser_event_e
{
  CDCSER_EVENT_LINECODING = 0, /* New line coding received from host */
  CDCSER_EVENT_CTRLLINE,       /* New control line status received from host */
  CDCSER_EVENT_SENDBREAK       /* Send break request received */
};

typedef FAR void (*cdcser_callback_t)(enum cdcser_event_e event);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_cdcclassobject
 *
 * Description:
 *   If the CDC serial class driver is part of composite device, then
 *   board-specific logic must provide board_cdcclassobject().  In the simplest
 *   case, board_cdcclassobject() is simply a wrapper around cdcser_classobject()
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

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_CDCSER_COMPOSITE)
EXTERN int board_cdcclassobject(FAR struct usbdevclass_driver_s **classdev);
#endif

/****************************************************************************
 * Name: cdcser_classobject
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured) and
 *   return the class object.
 *
 * Input Parameter:
 *   minor - Device minor number.  E.g., minor 0 would correspond to
 *     /dev/ttyUSB0.
 *   classdev - The location to return the CDC serial class' device
 *     instance.
 *
 * Returned Value:
 *   A pointer to the allocated class object (NULL on failure).
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_CDCSER_COMPOSITE)
int cdcser_classobject(int minor, FAR struct usbdevclass_driver_s **classdev);
#endif

/****************************************************************************
 * Name: cdcser_initialize
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured).
 *
 * Input Parameter:
 *   Device minor number.  E.g., minor 0 would correspond to /dev/ttyUSB0.
 *
 * Returned Value:
 *   Zero (OK) means that the driver was successfully registered.  On any
 *   failure, a negated errno value is retured.
 *
 ****************************************************************************/

EXTERN int cdcser_initialize(int minor);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_CDC_SERIAL_H */
