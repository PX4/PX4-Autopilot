/****************************************************************************
 * drivers/usbdev/composite.h
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

#ifndef __DRIVERS_USBDEV_COMPOSITE_H
#define __DRIVERS_USBDEV_COMPOSITE_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev_trace.h>

#ifdef CONFIG_USBDEV_COMPOSITE

#ifdef CONFIG_CDCACM_COMPOSITE
#  include <nuttx/usb/cdcacm.h>
#  include "cdcacm.h"
#endif

#ifdef CONFIG_USBMSC_COMPOSITE
#  include "usbmsc.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Packet sizes */

#ifndef CONFIG_COMPOSITE_EP0MAXPACKET
#  define CONFIG_COMPOSITE_EP0MAXPACKET 64
#endif

/* Vendor and product IDs and strings */

#ifndef CONFIG_COMPOSITE_COMPOSITE
#  ifndef CONFIG_COMPOSITE_VENDORID
#    warning "CONFIG_COMPOSITE_VENDORID not defined"
#    define CONFIG_COMPOSITE_VENDORID 0x03eb
#  endif

#  ifndef CONFIG_COMPOSITE_PRODUCTID
#    warning "CONFIG_COMPOSITE_PRODUCTID not defined"
#    define CONFIG_COMPOSITE_PRODUCTID 0x2022
#  endif

#  ifndef CONFIG_COMPOSITE_VERSIONNO
#    define CONFIG_COMPOSITE_VERSIONNO (0x0101)
#  endif

#  ifndef CONFIG_COMPOSITE_VENDORSTR
#    warning "No Vendor string specified"
#    define CONFIG_COMPOSITE_VENDORSTR  "NuttX"
# endif

#  ifndef CONFIG_COMPOSITE_PRODUCTSTR
#    warning "No Product string specified"
#    define CONFIG_COMPOSITE_PRODUCTSTR "Composite Device"
#  endif

#  undef CONFIG_COMPOSITE_SERIALSTR
#  define CONFIG_COMPOSITE_SERIALSTR "0101"
#endif

#undef CONFIG_COMPOSITE_CONFIGSTR
#define CONFIG_COMPOSITE_CONFIGSTR "Composite"

/* Constituent devices ******************************************************/

#undef DEV1_IS_CDCACM
#undef DEV1_IS_USBMSC

#undef DEV2_IS_CDCACM
#undef DEV2_IS_USBMSC

/* Pick the first device in the composite.  At present, this may only be
 * the CDC serial device or the mass storage device.
 */

#if defined(CONFIG_CDCACM_COMPOSITE)
#  define DEV1_IS_CDCACM   1
#  define DEV1_MKCFGDESC      cdcacm_mkcfgdesc
#  define DEV1_MKSTRDESC      cdcacm_mkstrdesc
#  define DEV1_CLASSOBJECT    board_cdcclassobject
#  define DEV1_UNINITIALIZE   board_cdcuninitialize
#  define DEV1_NCONFIGS       CDCACM_NCONFIGS
#  define DEV1_CONFIGID       CDCACM_CONFIGID
#  define DEV1_FIRSTINTERFACE CONFIG_CDCACM_IFNOBASE
#  define DEV1_NINTERFACES    CDCACM_NINTERFACES
#  define DEV1_STRIDBASE      CONFIG_CDCACM_STRBASE
#  define DEV1_NSTRIDS        CDCACM_NSTRIDS
#  define DEV1_CFGDESCSIZE    SIZEOF_CDCACM_CFGDESC
#elif defined(CONFIG_CDCACM_COMPOSITE)
#  define DEV1_IS_USBMSC     1
#  define DEV1_MKCFGDESC      usbmsc_mkcfgdesc
#  define DEV1_MKSTRDESC      usbmsc_mkstrdesc
#  define DEV1_CLASSOBJECT    board_mscclassobject
#  define DEV1_UNINITIALIZE   board_mscuninitialize
#  define DEV1_NCONFIGS       USBMSC_NCONFIGS
#  define DEV1_CONFIGID       USBMSC_CONFIGID
#  define DEV1_FIRSTINTERFACE CONFIG_USBMSC_IFNOBASE
#  define DEV1_NINTERFACES    USBMSC_NINTERFACES
#  define DEV1_STRIDBASE      CONFIG_USBMSC_IFNOBASE
#  define DEV1_NSTRIDS        USBMSC_NSTRIDS
#  define DEV1_CFGDESCSIZE    SIZEOF_USBMSC_CFGDESC
#else
#  error "No members of the composite defined"
#endif

/* Pick the second device in the composite.  At present, this may only be
 * the CDC serial device or the mass storage device.
 */

#if defined(CONFIG_CDCACM_COMPOSITE) && !defined(DEV1_IS_CDCACM)
#  define DEV2_IS_CDCACM 1
#  define DEV2_MKCFGDESC      cdcacm_mkcfgdesc
#  define DEV2_MKSTRDESC      cdcacm_mkstrdesc
#  define DEV2_CLASSOBJECT    board_cdcclassobject
#  define DEV2_UNINITIALIZE   board_cdcuninitialize
#  define DEV2_NCONFIGS       CDCACM_NCONFIGS
#  define DEV2_CONFIGID       CDCACM_CONFIGID
#  define DEV2_FIRSTINTERFACE CONFIG_CDCACM_IFNOBASE
#  define DEV2_NINTERFACES    CDCACM_NINTERFACES
#  define DEV2_STRIDBASE      CONFIG_CDCACM_STRBASE
#  define DEV2_NSTRIDS        CDCACM_NSTRIDS
#  define DEV2_CFGDESCSIZE    SIZEOF_CDCACM_CFGDESC
#elif defined(CONFIG_CDCACM_COMPOSITE) && !defined(DEV1_IS_USBMSC)
#  define DEV2_IS_USBMSC     1
#  define DEV2_MKCFGDESC      usbmsc_mkcfgdesc
#  define DEV2_MKSTRDESC      usbmsc_mkstrdesc
#  define DEV2_UNINITIALIZE   board_mscuninitialize
#  define DEV2_CLASSOBJECT    board_mscclassobject
#  define DEV2_NCONFIGS       USBMSC_NCONFIGS
#  define DEV2_CONFIGID       USBMSC_CONFIGID
#  define DEV2_FIRSTINTERFACE CONFIG_USBMSC_IFNOBASE
#  define DEV2_NINTERFACES    USBMSC_NINTERFACES
#  define DEV2_STRIDBASE      CONFIG_USBMSC_STRBASE
#  define DEV2_NSTRIDS        USBMSC_NSTRIDS
#  define DEV2_CFGDESCSIZE    SIZEOF_USBMSC_CFGDESC
#else
#  error "Insufficient members of the composite defined"
#endif

/* Verify interface configuration */

#if DEV1_FIRSTINTERFACE != 0
#  warning "The first interface number should be zero"
#endif

#if (DEV1_FIRSTINTERFACE + DEV1_NINTERFACES) != DEV2_FIRSTINTERFACE
#  warning "Interface numbers are not contiguous"
#endif

/* Check if an IAD is needed */

#ifdef CONFIG_COMPOSITE_IAD
#  if DEV1_NINTERFACES == 1 && DEV2_NINTERFACES == 1
#    warning "CONFIG_COMPOSITE_IAD not needed"
#  endif
#endif

#if !defined(CONFIG_COMPOSITE_IAD) && DEV1_NINTERFACES > 1 && DEV2_NINTERFACES > 1
#  warning "CONFIG_COMPOSITE_IAD may be needed"
#endif

/* Total size of the configuration descriptor: */

#define COMPOSITE_CFGDESCSIZE (USB_SIZEOF_CFGDESC + DEV1_CFGDESCSIZE + DEV2_CFGDESCSIZE)

/* The total number of interfaces */

#define COMPOSITE_NINTERFACES (DEV1_NINTERFACES + DEV2_NINTERFACES)

/* Composite configuration ID value */

#if DEV1_NCONFIGS != 1 || DEV1_CONFIGID != 1
#  error "DEV1:  Only a single configuration is supported"
#endif

#if DEV2_NCONFIGS != 1 || DEV2_CONFIGID != 1
#  error "DEV2:  Only a single configuration is supported"
#endif

/* Descriptors **************************************************************/
/* These settings are not modifiable via the NuttX configuration */

#define COMPOSITE_CONFIGIDNONE        (0)  /* Config ID = 0 means to return to address mode */
#define COMPOSITE_NCONFIGS            (1)  /* The number of configurations supported */
#define COMPOSITE_CONFIGID            (1)  /* The only supported configuration ID */

/* String language */

#define COMPOSITE_STR_LANGUAGE        (0x0409) /* en-us */

/* Descriptor strings */

#define COMPOSITE_MANUFACTURERSTRID   (1)
#define COMPOSITE_PRODUCTSTRID        (2)
#define COMPOSITE_SERIALSTRID         (3)
#define COMPOSITE_CONFIGSTRID         (4)
#define COMPOSITE_NSTRIDS             (4)

/* Verify string configuration */

#if COMPOSITE_NSTRIDS != DEV1_STRIDBASE
#  warning "The DEV1 string base should be COMPOSITE_NSTRIDS"
#endif

#if (DEV1_STRIDBASE + DEV1_NSTRIDS) != DEV2_STRIDBASE
#  warning "String IDs are not contiguous"
#endif

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

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const char g_compvendorstr[];
extern const char g_compproductstr[];
extern const char g_compserialstr[];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: composite_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

int composite_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc);

/****************************************************************************
 * Name: composite_getepdesc
 *
 * Description:
 *   Return a pointer to the composite device descriptor
 *
 ****************************************************************************/

#ifndef CONFIG_COMPOSITE_COMPOSITE
FAR const struct usb_devdesc_s *composite_getdevdesc(void);
#endif

/****************************************************************************
 * Name: composite_mkcfgdesc
 *
 * Description:
 *   Construct the composite configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
int16_t composite_mkcfgdesc(uint8_t *buf, uint8_t speed, uint8_t type);
#else
int16_t composite_mkcfgdesc(uint8_t *buf);
#endif

/****************************************************************************
 * Name: composite_getqualdesc
 *
 * Description:
 *   Return a pointer to the composite qual descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
FAR const struct usb_qualdesc_s *composite_getqualdesc(void);
#endif

#endif /* CONFIG_USBDEV_COMPOSITE */
#endif /* __DRIVERS_USBDEV_COMPOSITE_H */
