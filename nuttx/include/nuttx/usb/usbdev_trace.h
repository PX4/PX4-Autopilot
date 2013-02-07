/****************************************************************************
 * include/nuttx/usb/usbdev_trace.h
 *
 *   Copyright (C) 2008, 2009-2010, 2012 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_USB_USBDEV_TRACE_H
#define __INCLUDE_NUTTX_USB_USBDEV_TRACE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

/* Event encoding/decoding macros *******************************************/

#define TRACE_EVENT(id,data)     ((uint16_t)(id)|(data))
#define TRACE_ID(event)          ((event)&0xff00)
#define TRACE_DATA(event)        ((event)&0x00ff)

/* Events ******************************************************************/

/* Event class IDs */

#define TRACE_INIT_ID            (0x0000) /* Initialization events */
#define TRACE_EP_ID              (0x0100) /* Endpoint API calls */
#define TRACE_DEV_ID             (0x0200) /* USB device API calls */
#define TRACE_CLASS_ID           (0x0300) /* USB class driver API calls */
#define TRACE_CLASSAPI_ID        (0x0400) /* Other class driver system API calls */
#define TRACE_CLASSSTATE_ID      (0x0500) /* Track class driver state changes */
#define TRACE_INTENTRY_ID        (0x0600) /* Interrupt handler entry */
#define TRACE_INTDECODE_ID       (0x0700) /* Decoded interrupt event */
#define TRACE_INTEXIT_ID         (0x0800) /* Interrupt handler exit */
#define TRACE_OUTREQQUEUED_ID    (0x0900) /* Request queued for OUT endpoint */
#define TRACE_INREQQUEUED_ID     (0x0a00) /* Request queued for IN endpoint */
#define TRACE_READ_ID            (0x0b00) /* Read (OUT) action */
#define TRACE_WRITE_ID           (0x0c00) /* Write (IN) action */
#define TRACE_COMPLETE_ID        (0x0d00) /* Request completed */
#define TRACE_DEVERROR_ID        (0x0e00) /* USB controller driver error event */
#define TRACE_CLSERROR_ID        (0x0f00) /* USB class driver error event */

#define TRACE_NIDS               16       /* Cannot exceed bits in usbtrace_idset_t */

/* Bit settings for usbtrace_enable */

#define TRACE_ID2BIT(id)         ((1) << ((id) >> 8))
#define TRACE_INIT_BIT           TRACE_ID2BIT(TRACE_INIT_ID)
#define TRACE_EP_BIT             TRACE_ID2BIT(TRACE_EP_ID)
#define TRACE_DEV_BIT            TRACE_ID2BIT(TRACE_DEV_ID)
#define TRACE_CLASS_BIT          TRACE_ID2BIT(TRACE_CLASS_ID)
#define TRACE_CLASSAPI_BIT       TRACE_ID2BIT(TRACE_CLASSAPI_ID)
#define TRACE_CLASSSTATE_BIT     TRACE_ID2BIT(TRACE_CLASSSTATE_ID)
#define TRACE_INTENTRY_BIT       TRACE_ID2BIT(TRACE_INTENTRY_ID)
#define TRACE_INTDECODE_BIT      TRACE_ID2BIT(TRACE_INTDECODE_ID)
#define TRACE_INTEXIT_BIT        TRACE_ID2BIT(TRACE_INTEXIT_ID)
#define TRACE_OUTREQQUEUED_BIT   TRACE_ID2BIT(TRACE_OUTREQQUEUED_ID)
#define TRACE_INREQQUEUED_BIT    TRACE_ID2BIT(TRACE_INREQQUEUED_ID)
#define TRACE_READ_BIT           TRACE_ID2BIT(TRACE_READ_ID)
#define TRACE_WRITE_BIT          TRACE_ID2BIT(TRACE_WRITE_ID)
#define TRACE_COMPLETE_BIT       TRACE_ID2BIT(TRACE_COMPLETE_ID)
#define TRACE_DEVERROR_BIT       TRACE_ID2BIT(TRACE_DEVERROR_ID)
#define TRACE_CLSERROR_BIT       TRACE_ID2BIT(TRACE_CLSERROR_ID)
#define TRACE_ALLBITS            ((usbtrace_idset_t)-1)

/* Initialization events */

#define TRACE_DEVINIT            TRACE_EVENT(TRACE_INIT_ID, 0x0001)
#define TRACE_DEVUNINIT          TRACE_EVENT(TRACE_INIT_ID, 0x0002)
#define TRACE_DEVREGISTER        TRACE_EVENT(TRACE_INIT_ID, 0x0003)
#define TRACE_DEVUNREGISTER      TRACE_EVENT(TRACE_INIT_ID, 0x0004)
#define TRACE_DEVINIT_USER       TRACE_EVENT(TRACE_INIT_ID, 0x0005) /* First user-defined */

/* API calls (see usbdev.h) */

#define TRACE_EPCONFIGURE        TRACE_EVENT(TRACE_EP_ID, 0x0001)
#define TRACE_EPDISABLE          TRACE_EVENT(TRACE_EP_ID, 0x0002)
#define TRACE_EPALLOCREQ         TRACE_EVENT(TRACE_EP_ID, 0x0003)
#define TRACE_EPFREEREQ          TRACE_EVENT(TRACE_EP_ID, 0x0004)
#define TRACE_EPALLOCBUFFER      TRACE_EVENT(TRACE_EP_ID, 0x0005)
#define TRACE_EPFREEBUFFER       TRACE_EVENT(TRACE_EP_ID, 0x0006)
#define TRACE_EPSUBMIT           TRACE_EVENT(TRACE_EP_ID, 0x0007)
#define TRACE_EPCANCEL           TRACE_EVENT(TRACE_EP_ID, 0x0008)
#define TRACE_EPSTALL            TRACE_EVENT(TRACE_EP_ID, 0x0009)
#define TRACE_EPRESUME           TRACE_EVENT(TRACE_EP_ID, 0x000a)
#define TRACE_EPAPI_USER         TRACE_EVENT(TRACE_EP_ID, 0x000b) /* First user-defined */

#define TRACE_DEVALLOCEP         TRACE_EVENT(TRACE_DEV_ID, 0x0001)
#define TRACE_DEVFREEEP          TRACE_EVENT(TRACE_DEV_ID, 0x0002)
#define TRACE_DEVGETFRAME        TRACE_EVENT(TRACE_DEV_ID, 0x0003)
#define TRACE_DEVWAKEUP          TRACE_EVENT(TRACE_DEV_ID, 0x0004)
#define TRACE_DEVSELFPOWERED     TRACE_EVENT(TRACE_DEV_ID, 0x0005)
#define TRACE_DEVPULLUP          TRACE_EVENT(TRACE_DEV_ID, 0x0006)
#define TRACE_DEVAPI_USER        TRACE_EVENT(TRACE_DEV_ID, 0x0007) /* First user-defined */

#define TRACE_CLASSBIND          TRACE_EVENT(TRACE_CLASS_ID, 0x0001)
#define TRACE_CLASSUNBIND        TRACE_EVENT(TRACE_CLASS_ID, 0x0002)
#define TRACE_CLASSDISCONNECT    TRACE_EVENT(TRACE_CLASS_ID, 0x0003)
#define TRACE_CLASSSETUP         TRACE_EVENT(TRACE_CLASS_ID, 0x0004)
#define TRACE_CLASSSUSPEND       TRACE_EVENT(TRACE_CLASS_ID, 0x0005)
#define TRACE_CLASSRESUME        TRACE_EVENT(TRACE_CLASS_ID, 0x0006)

#define TRACE_CLASSRDCOMPLETE    TRACE_EVENT(TRACE_CLASS_ID, 0x0007)
#define TRACE_CLASSWRCOMPLETE    TRACE_EVENT(TRACE_CLASS_ID, 0x0008)

#define TRACE_CLASSAPI_USER      TRACE_EVENT(TRACE_CLASS_ID, 0x0009) /* First user-defined */

#define TRACE_CLASSAPI(id)       TRACE_EVENT(TRACE_CLASSAPI_ID, id)

#define TRACE_CLASSSTATE(id)     TRACE_EVENT(TRACE_CLASSSTATE_ID, id)

/* USB device controller interrupt events.  The 'id' is specific to the driver.
 * Particular values for 'id' are unique for a given implementation of a
 * controller driver
 */

#define TRACE_INTENTRY(id)       TRACE_EVENT(TRACE_INTENTRY_ID, id)
#define TRACE_INTDECODE(id)      TRACE_EVENT(TRACE_INTDECODE_ID, id)
#define TRACE_INTEXIT(id)        TRACE_EVENT(TRACE_INTEXIT_ID, id)

/* Controller data transfer */

#define TRACE_OUTREQQUEUED(ep)   TRACE_EVENT(TRACE_OUTREQQUEUED_ID, ep)
#define TRACE_INREQQUEUED(ep)    TRACE_EVENT(TRACE_INREQQUEUED_ID, ep)
#define TRACE_READ(ep)           TRACE_EVENT(TRACE_READ_ID, ep)
#define TRACE_WRITE(ep)          TRACE_EVENT(TRACE_WRITE_ID, ep)
#define TRACE_COMPLETE(ep)       TRACE_EVENT(TRACE_COMPLETE_ID, ep)

/* USB device controller error events.  The 'id' is specific to the driver.
 * Particular values for 'id' are unique for a given implementation of a
 * controller driver
 */

#define TRACE_DEVERROR(id)       TRACE_EVENT(TRACE_DEVERROR_ID, id)

/* USB class driver error events.  The 'id' is specific to the class driver,
 * but common to all driver controller instances.
 */

#define TRACE_CLSERROR(id)       TRACE_EVENT(TRACE_CLSERROR_ID, id)

/* USB Serial driver class events *******************************************/
/* Used by both the CDC/ACM and the PL2303 serial class drivers */
/* UART interface API calls */

#define USBSER_TRACECLASSAPI_SETUP                   0x0001
#define USBSER_TRACECLASSAPI_SHUTDOWN                0x0002
#define USBSER_TRACECLASSAPI_ATTACH                  0x0003
#define USBSER_TRACECLASSAPI_DETACH                  0x0004
#define USBSER_TRACECLASSAPI_IOCTL                   0x0005
#define USBSER_TRACECLASSAPI_RECEIVE                 0x0006
#define USBSER_TRACECLASSAPI_RXINT                   0x0007
#define USBSER_TRACECLASSAPI_RXAVAILABLE             0x0008
#define USBSER_TRACECLASSAPI_SEND                    0x0009
#define USBSER_TRACECLASSAPI_TXINT                   0x000a
#define USBSER_TRACECLASSAPI_TXREADY                 0x000b
#define USBSER_TRACECLASSAPI_TXEMPTY                 0x000c

/* Values of the class error ID used by the USB serial driver */

#define USBSER_TRACEERR_ALLOCCTRLREQ                 0x0001
#define USBSER_TRACEERR_ALLOCDEVSTRUCT               0x0002
#define USBSER_TRACEERR_ALREADYCLOSED                0x0003
#define USBSER_TRACEERR_ALREADYCONFIGURED            0x0004
#define USBSER_TRACEERR_CONFIGIDBAD                  0x0005
#define USBSER_TRACEERR_CONFIGNONE                   0x0006
#define USBSER_TRACEERR_CONSOLEREGISTER              0x0007
#define USBSER_TRACEERR_DEVREGISTER                  0x0008
#define USBSER_TRACEERR_EPRESPQ                      0x0009
#define USBSER_TRACEERR_GETUNKNOWNDESC               0x000a
#define USBSER_TRACEERR_INVALIDARG                   0x000b
#define USBSER_TRACEERR_EP0NOTBOUND                  0x000c
#define USBSER_TRACEERR_EPBULKINALLOCFAIL            0x000d
#define USBSER_TRACEERR_EPBULKINCONFIGFAIL           0x000e
#define USBSER_TRACEERR_EPBULKOUTALLOCFAIL           0x000f
#define USBSER_TRACEERR_EPINTINALLOCFAIL             0x0010
#define USBSER_TRACEERR_EPINTINCONFIGFAIL            0x0011
#define USBSER_TRACEERR_EPBULKOUTCONFIGFAIL          0x0012
#define USBSER_TRACEERR_RDALLOCREQ                   0x0013
#define USBSER_TRACEERR_RDSHUTDOWN                   0x0014
#define USBSER_TRACEERR_RDSUBMIT                     0x0015
#define USBSER_TRACEERR_RDUNEXPECTED                 0x0016
#define USBSER_TRACEERR_REQRESULT                    0x0017
#define USBSER_TRACEERR_RXOVERRUN                    0x0018
#define USBSER_TRACEERR_SETUPNOTCONNECTED            0x0019
#define USBSER_TRACEERR_SUBMITFAIL                   0x001a
#define USBSER_TRACEERR_UARTREGISTER                 0x001b
#define USBSER_TRACEERR_UARTUNREGISTER               0x001c
#define USBSER_TRACEERR_UNSUPPORTEDCTRLREQ           0x001d
#define USBSER_TRACEERR_UNSUPPORTEDCLASSREQ          0x001e
#define USBSER_TRACEERR_UNSUPPORTEDSTDREQ            0x001f
#define USBSER_TRACEERR_UNSUPPORTEDTYPE              0x0020
#define USBSER_TRACEERR_WRALLOCREQ                   0x0021
#define USBSER_TRACEERR_WRSHUTDOWN                   0x0022
#define USBSER_TRACEERR_WRUNEXPECTED                 0x0023

/* USB Storage driver class events ******************************************/

#define USBCOMPOSITE_TRACEERR_REQRESULT              0x0001
#define USBCOMPOSITE_TRACEERR_ALLOCCTRLREQ           0x0002
#define USBCOMPOSITE_TRACEERR_INVALIDARG             0x0003
#define USBCOMPOSITE_TRACEERR_EP0NOTBOUND            0x0004
#define COMPOSITE_TRACEERR_SETUPINVALIDARGS          0x0005
#define COMPOSITE_TRACEERR_EP0NOTBOUND2              0x0006
#define COMPOSITE_TRACEERR_GETUNKNOWNDESC            0x0007
#define COMPOSITE_TRACEERR_UNSUPPORTEDSTDREQ         0x0008
#define COMPOSITE_TRACEERR_EPRESPQ                   0x0009
#define USBCOMPOSITE_TRACEERR_ALLOCDEVSTRUCT         0x000a
#define USBCOMPOSITE_TRACEERR_CLASSOBJECT            0x000b
#define USBCOMPOSITE_TRACEERR_DEVREGISTER            0x000c

/* USB Storage driver class events ******************************************/

/* State transitions */

#define USBMSC_CLASSSTATE_IDLECMDPARSE               0x0001
#define USBMSC_CLASSSTATE_CMDPARSECMDFINISH          0x0002
#define USBMSC_CLASSSTATE_CMDPARSECMDREAD6           0x0003
#define USBMSC_CLASSSTATE_CMDPARSECMDREAD10          0x0004
#define USBMSC_CLASSSTATE_CMDPARSECMDREAD12          0x0005
#define USBMSC_CLASSSTATE_CMDPARSECMDWRITE6          0x0006
#define USBMSC_CLASSSTATE_CMDPARSECMDWRITE10         0x0007
#define USBMSC_CLASSSTATE_CMDPARSECMDWRITE12         0x0008
#define USBMSC_CLASSSTATE_CMDREAD                    0x0009
#define USBMSC_CLASSSTATE_CMDREADCMDFINISH           0x000a
#define USBMSC_CLASSSTATE_CMDWRITE                   0x000b
#define USBMSC_CLASSSTATE_CMDWRITECMDFINISH          0x000c
#define USBMSC_CLASSSTATE_CMDFINISHCMDSTATUS         0x000d
#define USBMSC_CLASSSTATE_CMDSTATUSIDLE              0x000e

/* Values of the class error ID used by the USB storage driver */

#define USBMSC_TRACEERR_ALLOCCTRLREQ                 0x0001
#define USBMSC_TRACEERR_ALLOCDEVSTRUCT               0x0002
#define USBMSC_TRACEERR_ALLOCIOBUFFER                0x0003
#define USBMSC_TRACEERR_ALREADYCONFIGURED            0x0004
#define USBMSC_TRACEERR_ALREADYUNINIT                0x0005
#define USBMSC_TRACEERR_BADREQUEST                   0x0006
#define USBMSC_TRACEERR_BINDLUNINVALIDARGS2          0x0007
#define USBMSC_TRACEERR_BINDLUNINVALIDARGS3          0x0008
#define USBMSC_TRACEERR_BINDLUNINVALIDARGS4          0x0009
#define USBMSC_TRACEERR_BINLUNINVALIDARGS1           0x000a
#define USBMSC_TRACEERR_BLKDRVEOPEN                  0x000b
#define USBMSC_TRACEERR_CMDBADLUN                    0x000c
#define USBMSC_TRACEERR_CMDFINISHRESIDUE             0x000d
#define USBMSC_TRACEERR_CMDFINISHRQEMPTY             0x000e
#define USBMSC_TRACEERR_CMDFINISHSHORTPKT            0x000f
#define USBMSC_TRACEERR_CMDFINISHSUBMIT              0x0010
#define USBMSC_TRACEERR_CMDFINSHDIR                  0x0011
#define USBMSC_TRACEERR_CMDFINSHSUBMIT               0x0012
#define USBMSC_TRACEERR_CMDPARSEWRREQLISTEMPTY       0x0013
#define USBMSC_TRACEERR_CMDREADREADFAIL              0x0014
#define USBMSC_TRACEERR_CMDREADSUBMIT                0x0015
#define USBMSC_TRACEERR_CMDREADWRRQEMPTY             0x0016
#define USBMSC_TRACEERR_CMDSTATUSRDREQLISTEMPTY      0x0017
#define USBMSC_TRACEERR_CMDUNEVIOLATION              0x0018
#define USBMSC_TRACEERR_CMDWRITERDSUBMIT             0x0019
#define USBMSC_TRACEERR_CMDWRITERDRQEMPTY            0x001a
#define USBMSC_TRACEERR_CMDWRITEWRITEFAIL            0x001b
#define USBMSC_TRACEERR_CONFIGIDBAD                  0x001c
#define USBMSC_TRACEERR_CONFIGNONE                   0x001d
#define USBMSC_TRACEERR_DEFERREDRESPINVALIDARGS      0x001e
#define USBMSC_TRACEERR_DEFERREDRESPSTALLED          0x001f
#define USBMSC_TRACEERR_DEFERREDRESPSUBMIT           0x0020
#define USBMSC_TRACEERR_DEVREGISTER                  0x0021
#define USBMSC_TRACEERR_DISCONNECTINVALIDARGS        0x0022
#define USBMSC_TRACEERR_EP0NOTBOUND1                 0x0023
#define USBMSC_TRACEERR_EP0NOTBOUND2                 0x0024
#define USBMSC_TRACEERR_EP0NOTBOUND3                 0x0025
#define USBMSC_TRACEERR_EPBULKINALLOCFAIL            0x0026
#define USBMSC_TRACEERR_EPBULKINCONFIGFAIL           0x0027
#define USBMSC_TRACEERR_EPBULKOUTALLOCFAIL           0x0028
#define USBMSC_TRACEERR_EPBULKOUTCONFIGFAIL          0x0029
#define USBMSC_TRACEERR_EPRESPQ                      0x002a
#define USBMSC_TRACEERR_EXPORTLUNSINVALIDARGS        0x002b
#define USBMSC_TRACEERR_GETMAXLUNNDX                 0x002c
#define USBMSC_TRACEERR_GETUNKNOWNDESC               0x002d
#define USBMSC_TRACEERR_IDLERDREQLISTEMPTY           0x002e
#define USBMSC_TRACEERR_IDLERDSUBMIT                 0x002f
#define USBMSC_TRACEERR_INQUIRYFLAGS                 0x0030
#define USBMSC_TRACEERR_INTERNALCONFUSION1           0x0031
#define USBMSC_TRACEERR_INTERNALCONFUSION2           0x0032
#define USBMSC_TRACEERR_INVALIDCBWCONTENT            0x0033
#define USBMSC_TRACEERR_INVALIDCBWSIGNATURE          0x0034
#define USBMSC_TRACEERR_INVALIDSTATE                 0x0035
#define USBMSC_TRACEERR_LUNALREADYBOUND              0x0036
#define USBMSC_TRACEERR_LUNNOTBOUND                  0x0037
#define USBMSC_TRACEERR_MODEPAGEFLAGS                0x0038
#define USBMSC_TRACEERR_MODESENSE10FLAGS             0x0039
#define USBMSC_TRACEERR_MODESENSE6FLAGS              0x003a
#define USBMSC_TRACEERR_MSRESETNDX                   0x003b
#define USBMSC_TRACEERR_NOGEOMETRY                   0x003c
#define USBMSC_TRACEERR_NOTCONFIGURED                0x003d
#define USBMSC_TRACEERR_NOTREMOVABLE                 0x003e
#define USBMSC_TRACEERR_PCSAVED                      0x003f
#define USBMSC_TRACEERR_PHASEERROR1                  0x0040
#define USBMSC_TRACEERR_PHASEERROR2                  0x0041
#define USBMSC_TRACEERR_PHASEERROR3                  0x0042
#define USBMSC_TRACEERR_PREVENTMEDIUMREMOVALPREVENT  0x0043
#define USBMSC_TRACEERR_RDALLOCREQ                   0x0044
#define USBMSC_TRACEERR_RDCOMPLETEINVALIDARGS        0x0045
#define USBMSC_TRACEERR_RDCOMPLETERDSUBMIT           0x0046
#define USBMSC_TRACEERR_RDSHUTDOWN                   0x0047
#define USBMSC_TRACEERR_RDSUBMIT                     0x0048
#define USBMSC_TRACEERR_RDUNEXPECTED                 0x0049
#define USBMSC_TRACEERR_READ10FLAGS                  0x004a
#define USBMSC_TRACEERR_READ10LBARANGE               0x004b
#define USBMSC_TRACEERR_READ10MEDIANOTPRESENT        0x004c
#define USBMSC_TRACEERR_READ12FLAGS                  0x004d
#define USBMSC_TRACEERR_READ12LBARANGE               0x004e
#define USBMSC_TRACEERR_READ12MEDIANOTPRESENT        0x004f
#define USBMSC_TRACEERR_READ6LBARANGE                0x0050
#define USBMSC_TRACEERR_READ6MEDIANOTPRESENT         0x0051
#define USBMSC_TRACEERR_READCAPACITYFLAGS            0x0052
#define USBMSC_TRACEERR_REALLOCIOBUFFER              0x0053
#define USBMSC_TRACEERR_REQRESULT                    0x0054
#define USBMSC_TRACEERR_SCSICMDCONTROL               0x0055
#define USBMSC_TRACEERR_SETCONFIGINVALIDARGS         0x0056
#define USBMSC_TRACEERR_SETUPINVALIDARGS             0x0057
#define USBMSC_TRACEERR_SNDCSWFAIL                   0x0058
#define USBMSC_TRACEERR_SNDPHERROR                   0x0059
#define USBMSC_TRACEERR_SNDSTATUSSUBMIT              0x005a
#define USBMSC_TRACEERR_SYNCCACHEMEDIANOTPRESENT     0x005b
#define USBMSC_TRACEERR_THREADCREATE                 0x005c
#define USBMSC_TRACEERR_TOOMANYLUNS                  0x005d
#define USBMSC_TRACEERR_UNBINDINVALIDARGS            0x005e
#define USBMSC_TRACEERR_UNBINDLUNINVALIDARGS1        0x005f
#define USBMSC_TRACEERR_UNBINDLUNINVALIDARGS2        0x0060
#define USBMSC_TRACEERR_UNINITIALIZEINVALIDARGS      0x0061
#define USBMSC_TRACEERR_UNSUPPORTEDSTDREQ            0x0062
#define USBMSC_TRACEERR_VERIFY10FLAGS                0x0063
#define USBMSC_TRACEERR_VERIFY10LBARANGE             0x0064
#define USBMSC_TRACEERR_VERIFY10MEDIANOTPRESENT      0x0065
#define USBMSC_TRACEERR_VERIFY10NOBLOCKS             0x0066
#define USBMSC_TRACEERR_VERIFY10READFAIL             0x0067
#define USBMSC_TRACEERR_WRALLOCREQ                   0x0068
#define USBMSC_TRACEERR_SNDPHERROR                   0x0069
#define USBMSC_TRACEERR_WRCOMPLETEINVALIDARGS        0x006a
#define USBMSC_TRACEERR_WRITE10FLAGS                 0x006b
#define USBMSC_TRACEERR_WRITE10LBARANGE              0x006c
#define USBMSC_TRACEERR_WRITE10MEDIANOTPRESENT       0x006d
#define USBMSC_TRACEERR_WRITE10READONLY              0x006e
#define USBMSC_TRACEERR_WRITE12FLAGS                 0x006f
#define USBMSC_TRACEERR_WRITE12LBARANGE              0x0070
#define USBMSC_TRACEERR_WRITE12MEDIANOTPRESENT       0x0071
#define USBMSC_TRACEERR_WRITE12READONLY              0x0072
#define USBMSC_TRACEERR_WRITE6LBARANGE               0x0073
#define USBMSC_TRACEERR_WRITE6MEDIANOTPRESENT        0x0074
#define USBMSC_TRACEERR_WRITE6READONLY               0x0075
#define USBMSC_TRACEERR_WRSHUTDOWN                   0x0076
#define USBMSC_TRACEERR_WRUNEXPECTED                 0x0077
#define USBMSC_TRACEERR_UNSUPPORTEDTYPE              0x0078

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The reported trace information */

struct usbtrace_s
{
  uint16_t event;
  uint16_t value;
};

/* Enumeration callback function signature */

typedef int (*trace_callback_t)(struct usbtrace_s *trace, void *arg);

/* Bit mask input type for usbtrace_enable().  If  TRACE_NIDS grows beyond
 * 16, then this will have to be changed to uint32_t
 */

typedef uint16_t usbtrace_idset_t;

/* Print routine to use for usbdev_trprint() output */

typedef int (*trprintf_t)(const char *fmt, ...);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C" {
#else
# define EXTERN extern
#endif

/*******************************************************************************
 * Name: usbtrace_enable
 *
 * Description:
 *  Enable/disable tracing per trace ID.  The initial state is all IDs enabled.
 *
 * Input Parameters:
 *  idset - The bitset of IDs to be masked.  TRACE_ALLIDS enables all IDS; zero
 *  masks all IDs.
 *
 * Returned Value:
 *  The previous idset value.
 *
 * Assumptions:
 * - May be called from an interrupt handler
 *
 *******************************************************************************/

#if defined(CONFIG_USBDEV_TRACE) || (defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_USB))
EXTERN usbtrace_idset_t usbtrace_enable(usbtrace_idset_t idset);
#else
#  define usbtrace_enable(idset)
#endif

/*******************************************************************************
 * Name: usbtrace
 *
 * Description:
 *  Record a USB event (tracing must be enabled)
 *
 * Assumptions:
 *   May be called from an interrupt handler
 *
 *******************************************************************************/

#if defined(CONFIG_USBDEV_TRACE) || (defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_USB))
EXTERN void usbtrace(uint16_t event, uint16_t value);
#else
#  define usbtrace(event, value)
#endif

/*******************************************************************************
 * Name: usbtrace_enumerate
 *
 * Description:
 *   Enumerate all buffer trace data (will temporarily disable tracing)
 *
 * Assumptions:
 *   NEVER called from an interrupt handler
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_TRACE
EXTERN int usbtrace_enumerate(trace_callback_t callback, void *arg);
#else
#  define usbtrace_enumerate(event)
#endif

/*******************************************************************************
 * Name: usbtrace_trprint
 *
 * Description:
 *   Print the trace record using the supplied printing function
 *
 *******************************************************************************/

EXTERN void usbtrace_trprintf(trprintf_t trprintf, uint16_t event, uint16_t value);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_USBDEV_TRACE_H */
