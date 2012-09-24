/********************************************************************************************
 * include/nuttx/usb/cdc.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References: "Universal Serial Bus Class Definitions for Communication
 *   Devices," Version 1.1, January 19, 1999
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_USB_CDC_H
#define __INCLUDE_NUTTX_USB_CDC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* Device Class Codes ***********************************************************************/
/* Table 14: Communication Device Class Code (see definition USB_CLASS_CDC in usb.h) */
/* Table 18: Data Interface Class Code  (see definition USB_CLASS_CDC_DATA in usb.h) */

/* Communication Inteface Class Codes *******************************************************/
/* Table 15: Communication Interface Class Code */

#define CDC_CLASS_COMM          0x02 /* Communication Interface Class */

/* Communication Interface Sub-Class Codes **************************************************/

#define CDC_SUBCLASS_NONE       0x00 /* Reserved */
#define CDC_SUBCLASS_DLC        0x01 /* Direct Line Control Model */
#define CDC_SUBCLASS_ACM        0x02 /* Abstract Control Model */
#define CDC_SUBCLASS_TCM        0x03 /* Telephone Control Model */
#define CDC_SUBCLASS_MCM        0x04 /* Multi-Channel Control Model */
#define CDC_SUBCLASS_CAPI       0x05 /* CAPI Control Model */
#define CDC_SUBCLASS_ECM        0x06 /* Ethernet Networking Control Model */
#define CDC_SUBCLASS_ATM        0x07 /* ATM Networking Control Model */
                                     /* 0x08-0x7f Reserved (future use) */
                                     /* 0x80-0xfe Reserved (vendor specific) */
/* Communication Interface Class Protocol Codes ********************************************/
/* Table 17: Communication Interface Class Control Protocol Codes */

#define CDC_PROTO_NONE          0x00 /* No class specific protocol required */
#define CDC_PROTO_ATM           0x01 /* Common AT commands (also known as Hayes compatible”) */
                                     /* 0x02-0xfe Reserved (future use) */
#define CDC_PROTO_VENDOR        0xff /* Vendor-specific */

/* Data Interface Sub-Class Codes ***********************************************************/
/* None defined, should be zero */

#define CDC_DATA_SUBCLASS_NONE  0x00

/* Date Interface Class Protocol Codes ******************************************************/
/* Table 19: Data Interface Class Protocol Codes */

#define CDC_DATA_PROTO_NONE     0x00 /* No class specific protocol required */
                                     /* 0x01-0x2f Reserved (future use) */
#define CDC_DATA_PROTO_ISDN     0x30 /* Physical interface protocol for ISDN BRI */
#define CDC_DATA_PROTO_HDLC     0x31 /* HDLC */
#define CDC_DATA_PROTO_TRANSP   0x32 /* Transparent */
                                     /* 0x33-0x4f Reserved (future use) */
#define CDC_DATA_PROTO_Q921M    0x50 /* Management protocol for Q.921 data link protocol */
#define CDC_DATA_PROTO_Q921     0x51 /* Data link protocol for Q.931 */
#define CDC_DATA_PROTO_Q921TM   0x52 /* TEI-multiplexor for Q.921 data link protocol */
                                     /* 0x53-0x8f Reserved (future use) */
#define CDC_DATA_PROTO_V42BIS   0x90 /* Data compression procedures */
#define CDC_DATA_PROTO_EUROISDN 0x91 /* Euro-ISDN protocol control */
#define CDC_DATA_PROTO_V120     0x92 /* V.24 rate adaptation to ISDN */
#define CDC_DATA_PROTO_CAPI     0x93 /* CAPI Commands */
                                     /* 0x94-0xfc Reserved (future use) */
#define CDC_DATA_PROTO_HBD      0xfd /* Host based driver. */
#define CDC_DATA_PROTO_PUFD     0xfe /* The protocol(s) are described using a Protocol Unit
                                      * Functional Descriptors on Communication Class
                                      * Interface.
                                      */
#define CDC_DATA_PROTO_VENDOR   0xff /* Vendor-specific */

/* Requests and Notifications ***************************************************************/
/* Table 2: Requests, Direct Line Control Model */

#define DLC_SET_AUX_LINE_STATE  0x10 /* Request to connect or disconnect secondary jack from
                                      * POTS circuit or CODEC, depending on hook state.
                                      * (Optional).
                                      */
#define DLC_SET_HOOK_STATE      0x11 /* Select relay setting for on-hook, off-hook, and caller
                                      * ID. (Required)
                                      */
#define DLC_PULSE_SETUP         0x12 /* Initiate pulse dialing preparation. (Optional).
                                      */
#define DLC_SEND_PULSE          0x13 /* Request number of make/break cycles to generate.
                                      * (Optional)
                                      */
#define DLC_SET_PULSE_TIME      0x14 /* Setup value for time of make and break periods when
                                      * pulse dialing. (Optional)
                                      */
#define DLC_RING_AUX_JACK       0x15 /* Request for a ring signal to be generated on secondary 
                                      * phone jack. (Optional)
                                      */
/* Table 3: Notifications, Direct Line Control Model */

#define DLC_AUX_JACK_HOOK_STATE 0x08 /* Indicates hook state of secondary device plugged
                                      * into the auxiliary phone jack. (Optional)
                                      */
#define DLC_RING_DETECT         0x09 /* Message to notify host that ring voltage was
                                      * detected on POTS interface. (Required)
                                      */
/* Table 4: Requests, Abstract Control Model */

#define ACM_SEND_COMMAND        0x00 /* Issues a command in the format of the supported
                                      * control protocol. (Required)
                                      */
#define ACM_GET_RESPONSE        0x01 /* Requests a response in the format of the
                                      * supported control protocol. (Required)
                                      */
#define ACM_SET_COMM_FEATURE    0x02 /* Controls the settings for a particular
                                      * communication feature. (Optional)
                                      */
#define ACM_GET_COMM_FEATURE    0x03 /* Returns the current settings for the
                                      * communication feature. (Optional)
                                      */
#define ACM_CLEAR_COMM_FEATURE  0x04 /* Clears the settings for a particular
                                      * communication feature. (Optional)
                                      */
#define ACM_SET_LINE_CODING     0x20 /* Configures DTE rate, stop-bits, parity, and
                                      * number-of-character bits. (Optional)
                                      */
#define ACM_GET_LINE_CODING     0x21 /* Requests current DTE rate, stop-bits, parity, and
                                      * number-of-character bits. (Optional)
                                      */
#define ACM_SET_CTRL_LINE_STATE 0x22 /* RS-232 signal used to tell the DCE device the
                                      * DTE device is now present. (Optional)
                                      */
#define ACM_SEND_BREAK          0x23 /* Sends special carrier
                                      */
/* Table 5: Notifications, Abstract Control Model */

#define ACM_NETWORK_CONNECTION  0x00 /* Notification to host of network connection status.
                                      * (Optional)
                                      */
#define ACM_RESPONSE_AVAILABLE  0x01 /* Notification to host to issue a GET_ENCAPSULATED_RESPONSE 
                                      * request. (Required)
                                      */
#define ACM_SERIAL_STATE        0x20 /* Returns the current state of the carrier detect, DSR,
                                      * break, and ring signal. (Optional)
                                      */
/* Table 6: Requests, Telephone Control Model */

#define TCM_SET_COMM_FEATURE    0x02 /* Used to set a unique communication feature, which is
                                      * normally specific to a particular device.
                                      * (Optional)
                                      */
#define TCM_GET_COMM_FEATURE    0x03 /* Returns the current settings for the communication
                                      * feature. (Optional)
                                      */
#define TCM_CLEAR_COMM_FEATURE  0x04 /* Clears the settings for a particular communication
                                      * feature. (Optional)
                                      */
#define TCM_SET_RINGER_PARMS    0x30 /* Configures the ringer for a telephone device.
                                      * (Optional)
                                      */
#define TCM_GET_RINGER_PARMS    0x31 /* Gets the current ringer configuration for a telephone
                                      * device. (Required)
                                      */
#define TCM_SET_OPERATION_PARMS 0x32 /* Configures the operational mode of the telephone.
                                      * (Optional)
                                      */
#define TCM_GET_OPERATION_PARMS 0x33 /* Gets the current operational mode of the telephone.
                                      * (Optional)
                                      */
#define TCM_SET_LINE_PARMS      0x34 /* Allows changing the current state of the line
                                      * associated with the interface, providing basic call
                                      * capabilities, such as dialing and answering calls.
                                      * (Required)
                                      */
#define TCM_GET_LINE_PARMS      0x35 /* Gets current status of the line. (Required)
                                      */
#define TCM_DIAL_DIGITS         0x36 /* Dials digits on the network connection. (Required)
                                      */
/* Table 7: Notifications, Telephone Control Model */

#define TCM_CALL_STATE_CHANGE   0x28 /* DReports a state change on a call. (Required)
                                      */
#define TCM_LINE_STATE_CHANGE   0x29 /* DReports a state change on a line. (Optional)
                                      */
/* Table 8: Requests, Multi-Channel Model */

#define MCM_SET_UNIT_PARAM      0x37 /* Used to set a Unit specific parameter (Optional)
                                      */
#define MCM_GET_UNIT_PARAM      0x38 /* Used to retrieve a Unit specific parameter (Required)
                                      */
#define MCM_CLEAR_UNIT_PARAM    0x39 /* Used to set a Unit specific parameter to its default
                                      * state. (Optional)
                                      */
/* Table 9: Request, CAPI Control Model */

#define CAPI_GET_PROFILE        0x3a /* Returns the implemented capabilities of the device
                                      * (Required)
                                      */
/* Table 10: Requests, Ethernet Networking Control Model */

#define ECM_SEND_COMMAND        0x00 /* Issues a command in the format of the supported
                                      * control protocol. The intent of this mechanism is
                                      * to support networking devices (e.g., host-based
                                      * cable modems) that require an additional vendor-
                                      * defined interface for media specific hardware
                                      * configuration and management. (Optional)
                                      */
#define ECM_GET_RESPONSE        0x01 /* equests a response in the format of the supported
                                      * control protocol.
                                      * (Optional)
                                      */
#define ECM_SET_MCAST_FILTERS   0x40 /* As applications are loaded and unloaded on the host,
                                      * the networking transport will instruct the device’s MAC
                                      * driver to change settings of the Networking device’s
                                      * multicast filters. (Optional)
                                      */
#define ECM_SET_PM_PAT_FILTER   0x41 /* Some hosts are able to conserve energy and stay quiet
                                      * in a “sleeping” state while not being used. USB
                                      * Networking devices may provide special pattern filtering
                                      * hardware that enables it to wake up the attached host
                                      * on demand when something is attempting to contact the
                                      * host (e.g., an incoming web browser connection).
                                      * Primitives are needed in management plane to negotiate
                                      * the setting of these special filters
                                      * (Optional)
                                      */
#define ECM_GET_PM_PAT_FILTER   0x42 /* Retrieves the status of the above power management
                                      * pattern filter setting
                                      * (Optional)
                                      */
#define ECM_SET_PACKET_FILTER   0x43 /* Sets device filter for running a network analyzer
                                      * application on the host machine (Required)
                                      */
#define ECM_GET_STATISTIC       0x44 /* Retrieves Ethernet device statistics such as frames
                                      * transmitted, frames received, and bad frames received.
                                      * (Optional)
                                      */
/* Table 11: Notifications, Ethernet Networking Control Model */

#define ECM_NETWORK_CONNECTION  0x00 /* Reports whether or not the physical layer (modem,
                                      * Ethernet PHY, etc.) link is up. (Required)
                                      */
#define ECM_RESPONSE_AVAILABLE  0x01 /* Notification to host to issue a
                                      * GET_ENCAPSULATED_RESPONSE request. (Optional)
                                      */
#define ECM_SPEED_CHANGE        0x2a /* Reports a change in upstream or downstream (Required)
                                      */
/* Table 12: Requests, ATM Networking Control Model */

#define ATM_SEND_COMMAND        0x00 /* Issues a command in the format of the supported control
                                      * protocol. The intent of this mechanism is to support
                                      * networking devices (e.g., host-based cable modems)
                                      * that require an additional vendor-defined interface for
                                      * media specific hardware configuration and
                                      * management. (Optional)
                                      */
#define ATM_GET_RESPONSE        0x01 /* Requests a response in the format of the supported
                                      * control protocol. (Optional)
                                      */
#define ATM_SET_DATA_FORMAT     0x50 /* Chooses which ATM data format will be exchanged
                                      * between the host and the ATM Networking device.
                                      * (Required)
                                      */
#define ATM_GET_DEV_STATISTICS  0x51 /* Retrieves global statistics from the ATM Networking
                                      * device. (Required)
                                      */
#define ATM_SET_DEFAULT_VC      0x52 /* Pre-selects the VPI/VCI value for subsequent
                                      * GetATMVCStatistics requests (Optional)
                                      */
#define ATM_GET_VC_STATISTICS   0x53 /* Retrieves statistics from the ATM Networking device for
                                      * a particular VPI/VCI. (Optional)
                                      */
/* Table 13: Requests, Ethernet and ATM Networking Control Model */

#define ATM_NETWORK_CONNECTION  0x00 /* Reports whether or not the physical layer (modem,
                                      * Ethernet PHY, etc.) link is up. (Required)
                                      */
#define ECM_NETWORK_CONNECTION  ATM_NETWORK_CONNECTION
#define ATM_RESPONSE_AVAILABLE  0x01 /* Notification to host to issue a
                                      * GET_ENCAPSULATED_RESPONSE request. (Optional)
                                      */
#define ECM_RESPONSE_AVAILABLE  ATM_RESPONSE_AVAILABLE
#define ATM_SPEED_CHANGE        0x2a /* Reports a change in upstream or downstream speed of the
                                      * networking device connection. (Required)
                                      */
#define ECM_SPEED_CHANGE        ATM_SPEED_CHANGE

/* Descriptors ******************************************************************************/
/* Table 25: bDescriptor SubType in Functional Descriptors */

#define CDC_DSUBTYPE_HDR        0x00 /* Header Functional Descriptor, which marks the
                                      * beginning of the concatenated set of functional
                                      * descriptors for the interface. */
#define CDC_DSUBTYPE_CALLMGMT   0x01 /* Call Management Functional Descriptor */
#define CDC_DSUBTYPE_ACM        0x02 /* Abstract Control Management Functional Descriptor */
#define CDC_DSUBTYPE_DLC        0x03 /* Direct Line Management Functional Descriptor */
#define CDC_DSUBTYPE_TCMRINGER  0x04 /* Telephone Ringer Functional Descriptor */
#define CDC_DSUBTYPE_TCMCALL    0x05 /* Telephone Call and Line State Reporting Capabilities
                                      *  Functional Descriptor. */
#define CDC_DSUBTYPE_UNION      0x06 /* Union Functional descriptor */
#define CDC_DSUBTYPE_COUNTRY    0x07 /* Country Selection Functional Descriptor */
#define CDC_DSUBTYPE_TCMOPS     0x08 /* Telephone Operational Modes Functional Descriptor */
#define CDC_DSUBTYPE_USBTERM    0x09 /* USB Terminal Functional Descriptor */
#define CDC_DSUBTYPE_NETCHAN    0x0a /* Network Channel Terminal Descriptor */
#define CDC_DSUBTYPE_PROTOUNIT  0x0b /* Protocol Unit Functional Descriptor */
#define CDC_DSUBTYPE_EXTUNIT    0x0c /* Extension Unit Functional Descriptor */
#define CDC_DSUBTYPE_MCM        0x0d /* Multi-Channel Management Functional Descriptor */
#define CDC_DSUBTYPE_CAPI       0x0e /* CAPI Control Management Functional Descriptor */
#define CDC_DSUBTYPE_ECM        0x0f /* Ethernet Networking Functional Descriptor */
#define CDC_DSUBTYPE_ATM        0x10 /* ATM Networking Functional Descriptor */
                                     /* 0x11-0xff Reserved (future use) */

/* Table 42: Ethernet Statistics Capabilities */

#define ECMCAP_XMIT_OK             (1 << 0)  /* Frames transmitted without errors */
#define ECMCAP_RVC_OK              (1 << 1)  /* Frames received without errors */
#define ECMCAP_XMIT_ERROR          (1 << 2)  /* Frames not transmitted, or transmitted with errors */
#define ECMCAP_RCV_ERROR           (1 << 3)  /* Frames received with errors that are not delivered
                                           * to the USB host
                                           */
#define ECMCAP_RCV_NO_BUFFER       (1 << 4)  /* Frame missed, no buffers */
#define ECMCAP_DIR_BYTES_XMIT      (1 << 5)  /* Directed bytes transmitted without errors */
#define ECMCAP_DIR_FRAMES_XMIT     (1 << 6)  /* Directed frames transmitted without errors */
#define ECMCAP_MCAST_BYTES_XMIT    (1 << 7)  /* Multicast bytes transmitted without errors */
#define ECMCAP_MCAST_FRAMES_XMIT   (1 << 8)  /* Multicast frames transmitted without errors */
#define ECMCAP_BCAST_BYTES_XMIT    (1 << 9)  /* Broadcast bytes transmitted without errors */
#define ECMCAP_BCAST_FRAMES_XMIT   (1 << 10) /* Broadcast frames transmitted without errors */
#define ECMCAP_DIR_BYTES_RCV       (1 << 11) /* Directed bytes received without errors */
#define ECMCAP_DIR_FRAMES_RCV      (1 << 12) /* Directed frames received without errors */
#define ECMCAP_MCAST_BYTES_RCV     (1 << 13) /* Multicast bytes received without errors */
#define ECMCAP_MCAST_FRAMES_RCV    (1 << 14) /* Multicast frames received without errors */
#define ECMCAP_BCAST_BYTES_RCV     (1 << 15) /* Broadcast bytes received without errors */
#define ECMCAP_BCAST_FRAMES_RCV    (1 << 16) /* Broadcast frames received without errors */
#define ECMCAP_RCV_CRC_ERROR       (1 << 17) /* Frames received with circular redundancy check
                                              * (CRC) or frame check sequence (FCS) error
                                              */
#define ECMCAP_TRANSMIT_QUEUE_LENG (1 << 18) /* Length of transmit queue */
#define ECMCAP_RCV_ERROR_ALIGNMENT (1 << 19) /* Frames received with alignment error */
#define ECMCAP_XMIT_ONE_COLL       (1 << 20) /* Frames transmitted with one collision */
#define ECMCAP_XMIT_MORE_COLLS     (1 << 21) /* Frames transmitted with more than one collision */
#define ECMCAP_XMIT_DEFERRED       (1 << 22) /* Frames transmitted after deferral */
#define ECMCAP_XMIT_MAX_COLLS      (1 << 23) /* Frames not transmitted due to collisions */
#define ECMCAP_RCV_OVERRUN         (1 << 24) /* Frames not received due to overrun */
#define ECMCAP_XMIT_UNDERRUN       (1 << 25) /* Frames not transmitted due to underrun */
#define ECMCAP_XMIT_HB_FAILURE     (1 << 26) /* Frames transmitted with heartbeat failure */
#define ECMCAP_XMIT_TIMES_CRS_LOST (1 << 27) /* Times carrier sense signal lost during
                                              * transmission
                                              */
#define ECMCAP_XMIT_LATE_COLLS     (1 << 28) /* Late collisions detected */
                                             /* Bits 29-31 Resrved, Must be set to zero */

/* Table 47: Communication Feature Selector Codes */

#define FEATURE_ABSTRACT_STATE  0x01 /* Two bytes of data describing multiplexed state
                                      * and idle state for this Abstract Model
                                      * communications device
                                      */
#define FEATURE_COUNTRY_SETTING 0x02 /* Country code in hexadecimal format as defined in
                                      * ISO 3166
                                      */
/* Table 49: POTS Relay Configuration Values */

#define POTS_ON_HOOK            0x0000
#define POTS_OFF_HOOK           0x0001
#define POTS_SNOOPING           0x0002

/* Table 50: Line Coding Structure */

#define CDC_CHFMT_STOP1         0 /* One stop bit */
#define CDC_CHFMT_STOP1p5       1 /* 1.5 stop bits */
#define CDC_CHFMT_STOP2         2 /* 2 stop bits */

#define CDC_PARITY_NONE         0 /* No parity  */
#define CDC_PARITY_ODD          1 /* Odd parity */
#define CDC_PARITY_EVEN         2 /* Even parity */
#define CDC_PARITY_MARK         3 /* Mark parity */
#define CDC_PARITY_SPACE        4 /* Space parity */

/* Table 51: Control Signal Bitmap Values for SetControlLineState */

#define CDC_DTE_PRESENT         (1 << 0) /* Indicates to DCE if DTE is present or not.
                                          * This signal corresponds to V.24 signal
                                          * 108/2 and RS-232 signal DTR.
                                          */
#define CDC_ACTIVATE_CARRIER    (1 << 1) /* Carrier control for half duplex modems.
                                          * This signal corresponds to V.24 signal
                                          * 105 and RS-232 signal RTS.
                                          */

/* Table 58: Call State Value Definitions */

#define CDC_CALLST_IDLE         0x00 /* Call is idle */
#define CDC_CALLST_DIAL         0x01 /* Typical dial tone */
#define CDC_CALLST_INTDIAL      0x02 /* Interrupted dial tone */
#define CDC_CALLST_DIALING      0x03 /* Dialing is in progress */
#define CDC_CALLST_RINGBACK     0x04 /* Ringback */
#define CDC_CALLST_CONNECTED    0x05 /* Connected */
#define CDC_CALLSTINCOMING      0x06 /* Incoming call */

/* Table 62: Ethernet Packet Filter Bitmap */

#define PACKET_TYPE_PROMISCUOUS   (1 << 0)
#define PACKET_TYPE_ALL_MULTICAST (1 << 1)
#define PACKET_TYPE_DIRECTED      (1 << 2)
#define PACKET_TYPE_BROADCAST     (1 << 3)
#define PACKET_TYPE_MULTICAST     (1 << 4)

/* Table 63: Ethernet Statistics Feature Selector Codes */

#define ECM_XMIT_OK             0x01 /* Frames transmitted without errors */
#define ECM_RVC_OK              0x02 /* Frames received without errors */
#define ECM_XMIT_ERROR          0x03 /* Frames not transmitted, or transmitted with errors */
#define ECM_RCV_ERROR           0x04 /* Frames received with errors that are not delivered
                                      * to the USB host
                                      */
#define ECM_RCV_NO_BUFFER       0x05 /* Frame missed, no buffers */
#define ECM_DIR_BYTES_XMIT      0x06 /* Directed bytes transmitted without errors */
#define ECM_DIR_FRAMES_XMIT     0x07 /* Directed frames transmitted without errors */
#define ECM_MCAST_BYTES_XMIT    0x08 /* Multicast bytes transmitted without errors */
#define ECM_MCAST_FRAMES_XMIT   0x09 /* Multicast frames transmitted without errors */
#define ECM_BCAST_BYTES_XMIT    0x0a /* Broadcast bytes transmitted without errors */
#define ECM_BCAST_FRAMES_XMIT   0x0b /* Broadcast frames transmitted without errors */
#define ECM_DIR_BYTES_RCV       0x0c /* Directed bytes received without errors */
#define ECM_DIR_FRAMES_RCV      0x0d /* Directed frames received without errors */
#define ECM_MCAST_BYTES_RCV     0x0e /* Multicast bytes received without errors */
#define ECM_MCAST_FRAMES_RCV    0x0f /* Multicast frames received without errors */
#define ECM_BCAST_BYTES_RCV     0x10 /* Broadcast bytes received without errors */
#define ECM_BCAST_FRAMES_RCV    0x11 /* Broadcast frames received without errors */
#define ECM_RCV_CRC_ERROR       0x12 /* Frames received with circular redundancy check
                                      * (CRC) or frame check sequence (FCS) error
                                      */
#define ECM_TRANSMIT_QUEUE_LENG 0x13 /* Length of transmit queue */
#define ECM_RCV_ERROR_ALIGNMENT 0x14 /* Frames received with alignment error */
#define ECM_XMIT_ONE_COLL       0x15 /* Frames transmitted with one collision */
#define ECM_XMIT_MORE_COLLS     0x16 /* Frames transmitted with more than one collision */
#define ECM_XMIT_DEFERRED       0x17 /* Frames transmitted after deferral */
#define ECM_XMIT_MAX_COLLS      0x18 /* Frames not transmitted due to collisions */
#define ECM_RCV_OVERRUN         0x19 /* Frames not received due to overrun */
#define ECM_XMIT_UNDERRUN       0x1a /* Frames not transmitted due to underrun */
#define ECM_XMIT_HB_FAILURE     0x1b /* Frames transmitted with heartbeat failure */
#define ECM_XMIT_TIMES_CRS_LOST 0x1c /* Times carrier sense signal lost during
                                      * transmission
                                      */
#define ECM_XMIT_LATE_COLLS     0x1d /* Late collisions detected */

/* Table 64: ATM Data Format */

#define ATM_FMT_TYPE1           1  /* Type 1 format: concatenated ATM cells */
#define ATM_FMT_TYPE1           2  /* Type 2 format: ATM header template + concatenated ATM
                                    * cell payloads
                                    */
#define ATM_FMT_TYPE1           3  /* Type 3 format: AAL 5 SDU */

/* Table 65: ATM Device Statistics Feature Selector Codes */

#define US_CELLS_SENT                0x01h /* The number of cells that have been sent
                                            * upstream to the WAN link by the ATM layer.
                                            */
#define DS_CELLS_RECEIVED            0x02h /* The number of cells that have been received
                                            * downstream from the WAN link by the ATM
                                            * layer.
                                            */
#define DS_CELLS_USB_CONGESTION      0x03h /* The number of cells that have been received
                                            * downstream from the WAN link by the ATM
                                            * layer and discarded due to congestion on the
                                            * USB link.
                                            */
#define DS_CELLS_AAL5_CRC_ERROR      0x04h /* The number of cells that have been received
                                            * downstream from the WAN link by the ATM
                                             * layer and discarded due to AAL5 CRC errors.
                                            */
#define DS_CELLS_HEC_ERROR           0x05h /* The number of cells that have been received
                                            * downstream from the WAN link and discarded
                                            * due to HEC errors in the cell header.
                                            */
#define DS_CELLS_HEC_ERROR_CORRECTED 0x06h /* The number of cells that have been received
                                            * downstream from the WAN link and have
                                            * been detected with HEC errors in the cell
                                            * header and successfully corrected.
                                            */
/* Table 66: ATM VC Selector Codes */

#define VC_US_CELLS_SENT        0x01 /* The number of cells that have been sent upstream to
                                      * the WAN link for the specified VPI/VCI since the
                                      * device has been powered on or reset
                                      */
#define VC_DS_CELLS_RECEIVED    0x02 /* The number of cells that have been received
                                      * downstream from the WAN link for the specified
                                      * VPI/VCI since the device has been
                                      * powered on or reset
                                      */
/* Notifications ****************************************************************************/
/* Table 69: UART State Bitmap Values */

#define CDC_UART_RXCARRIER      (1 << 0) /* bRxCarrier State of receiver carrier detection
                                          * mechanism of device. This signal corresponds to
                                          * V.24 signal 109 and RS-232 signal DCD.
                                          */
#define CDC_UART_TXCARRIER      (1 << 1) /* bTxCarrier State of transmission carrier. This
                                          * signal corresponds to V.24 signal 106 and RS-232
                                          * signal DSR.
                                          */
#define CDC_UART_BREAK          (1 << 2) /* bBreak State of break detection mechanism of the
                                          * device.
                                          */
#define CDC_UART_RING           (1 << 3) /* bRingSignal State of ring signal detection of the
                                          * device.
                                          */
#define CDC_UART_FRAMING        (1 << 4) /* bFraming A framing error has occurred */
#define CDC_UART_PARITY         (1 << 5) /* bParity A parity error has occurred */
#define CDC_UART_OVERRUN        (1 << 6) /* bOverRun Received data has been discarded due to
                                          * overrun in the device.
                                          */
/* Table 70: Call State Change Value Definitions */

#define CDC_CALLST_IDLE         0x01 /* Call has become idle */
#define CDC_CALLST_DIALING      0x02 /* Dialing */
#define CDC_CALLST_RINGBACK     0x03 /* Ringback, with an extra byte of data provided to
                                      * describe the type of ringback signaling
                                      */
#define CDC_CALLST_CONNECTED    0x04 /* Connected, with an extra byte of data provided to
                                      * describe the type of connection
                                      */
#define CDC_CALLST_INCOMING     0x05 /* Incoming Call, with the extra bytes of data */

/* Table 71: Line State Change Values */

#define CDC_LINEST_IDLE         0x0000 /* Line has become idle */
#define CDC_LINEST_HOLD         0x0001 /* Line connected to hold position */
#define CDC_LINEST_OFFHOOK      0x0002 /* Hook-switch has gone off hook */
#define CDC_LINEST_ONHOOK       0x0003 /* Hook-switch has gone on hook */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/* Table 1: Data Class Protocol Wrapper */

struct cdc_protowrapper_s
{
  uint8_t size[2];   /* Size of wrapper in bytes */
  uint8_t dstproto;  /* bDstProtocol, Destination protocol ID */
  uint8_t srcproto;  /* bSrcProtocol, Source protocol ID */
  uint8_t data[1];   /* Data payload, actual size depends of size of the wrapper */
};

/* Functional Descriptors *******************************************************************/
/* Table 23: Functional Descriptor General Format */

struct cdc_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t data[1];   /* Function-specific data follows */
};

/* Table 26: Class-Specific Descriptor Header Format */

struct cdc_hdr_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_HDR as defined in Table 25 */
  uint8_t cdc[2];    /* bcdCDC, USB Class Definitions for Communication Devices Specification release
                      * number in binary-coded decimal.
                      */
};
#define SIZEOF_HDR_FUNCDESC 5

/* Table 27: Call Management Functional Descriptor */

struct cdc_callmgmt_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_CALLMGMT as defined in Table 25 */
  uint8_t caps;      /* bmCapabilities: Bit encoded */
  uint8_t ifno;      /* bDataInterface, Interface number of Data Class interface
                      * optionally used for call management
                      */
};
#define SIZEOF_CALLMGMT_FUNCDESC 5

/* Table 28: Abstract Control Management Functional Descriptor */

struct cdc_acm_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_ACM as defined in Table 25 */
  uint8_t caps;      /* bmCapabilities: Bit encoded */
};
#define SIZEOF_ACM_FUNCDESC 4

/* Table 29: Direct Line Management Functional Descriptor */

struct cdc_dlc_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_DLC as defined in Table 25 */
  uint8_t caps;      /* bmCapabilities: Bit encoded */
};
#define SIZEOF_DLC_FUNCDESC 4

/* Table 30: Telephone Ringer Functional Descriptor */

struct cdc_tcmr_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_TCMRINGER as defined in Table 25 */
  uint8_t volsteps;  /* bRingerVolSteps, Number of discrete steps in volume supported
                      * by the ringer.
                      */
  uint8_t npats;     /* bNumRingerPatterns: Number of ringer patterns supported. */
};
#define SIZEOF_TCMR_FUNCDESC 5

/* Table 31: Telephone Operational Modes Functional Descriptor */

struct cdc_tcmops_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_TCMOPS as defined in Table 25 */
  uint8_t caps;      /* bmCapabilities: Bit encoded */
};
#define SIZEOF_TCMOPS_FUNCDESC 4

/* Table 32: Telephone Call State Reporting Capabilities Descriptor */

struct cdc_tcmc_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_TCMCALL as defined in Table 25 */
  uint8_t caps;      /* bmCapabilities: Bit encoded */
};
#define SIZEOF_TCMC_FUNCDESC 4

/* Table 33: Union Interface Functional Descriptor */

struct cdc_union_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_UNION as defined in Table 25 */
  uint8_t master;    /* bMasterInterface: The interface number of the Communication or Data
                      * Class interface, designated as the master or controlling interface
                      * for the union
                      */
  uint8_t slave[1];  /* bSlaveInterfaceN: Interface number of N slave or associated
                      * interface in the union
                      */
};
#define SIZEOF_UNION_FUNCDESC(n) ((n)+4)

/* Table 34: Country Selection Functional Descriptor */

struct cdc_country_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_COUNTRY as defined in Table 25 */
  uint8_t reldate;   /* iCountryCodeRelDate: Index of a string giving the release date for the
                      * implemented ISO 3166 Country Codes
                      */
  uint16_t code[1];  /* wCountryCodeN: Country code in hexadecimal format as defined in ISO 3166,
                      * release date as specified in offset 3 for Nth country supported
                      */
};
#define SIZEOF_COUNTRY_FUNCDESC(n) (sizeof(uint16_t)*(n) + 4)

/* Table 35: USB Terminal Functional Descriptor */

struct cdc_usbterm_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_USBTERM as defined in Table 25 */
  uint8_t id;        /* bEntityId, Constant uniquely identifying the Terminal */
  uint8_t ifno;      /* bInInterfaceNo, The input interface number of the associated
                      * USB interface
                      */
  uint8_t outif;     /* bOutInterfaceNo, The output interface number of the associated
                      * USB interface
                      */
  uint8_t options;   /* bmOptions, bit-encoded options */
  uint8_t child[1];  /* Nth ID of lower Terminal or Unit to which this Terminal is connected. */
};
#define SIZEOF_USBTERM_FUNCDESC(n) ((n)+7)

/* Table 36: Network Channel Terminal Functional Descriptor */

struct cdc_netchan_funcdesc_s
{
  uint8_t size;     /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_NETCHAN as defined in Table 25 */
  uint8_t id;        /* bEntityId, Constant uniquely identifying the Terminal */
  uint8_t name;      /* iName, Index of string descriptor, describing the name of the Network
                      * Channel Terminal
                      */
  uint8_t index;     /* bChannelIndex, The channel index of the associated network channel */
  uint8_t phyif;     /* bPhysicalInterface, Type of physical interface */
};
#define SIZEOF_NETCHAN_FUNCDESC 7

/* Table 37: Protocol Unit Functional Descriptor */

struct cdc_protounit_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_PROTOUNIT as defined in Table 25 */
  uint8_t id;        /* bEntityId, Constant uniquely identifying the Unit */
  uint8_t proto;     /* bProtocol, Protocol code as defined in Table 19 */
  uint8_t child[1];  /* Nth ID of lower Terminal or Unit to which this Terminal is connected */
};
#define SIZEOF_PROTOUNIT_FUNCDESC(n) ((n)+5)

/* Table 38: Extension Unit Functional Descriptor */

struct cdc_extunit_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_EXTUNIT as defined in Table 25 */
  uint8_t id;        /* bEntityId, Constant uniquely identifying the Extension Unit */
  uint8_t code;      /* bExtensionCode, Vendor specific code identifying the Extension Unit */
  uint8_t name;      /* iName, Index of string descriptor, describing the name of the Extension Unit */
  uint8_t child[1];  /* Nth ID of lower Terminal or Unit to which this Terminal is connected */
};
#define SIZEOF_EXTUNIT_FUNCDESC(n) ((n)+6)

/* Table 39: Multi-Channel Management Functional Descriptor */

struct cdc_mcm_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_MCM as defined in Table 25 */
  uint8_t caps;      /* bmCapabilities: Bit encoded */
};
#define SIZEOF_MCM_FUNCDESC 4

/* Table 40: CAPI Control Management Functional Descriptor */

struct cdc_capi_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_CAPI as defined in Table 25 */
  uint8_t caps;      /* bmCapabilities: Bit encoded */
};
#define SIZEOF_CAPI_FUNCDESC 4

/* Table 41: Ethernet Networking Functional Descriptor*/

struct cdc_ecm_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_ECM as defined in Table 25 */
  uint8_t mac;       /* iMACAddress, Index of teh 48bit Ethernet MAC address string descriptor */
  uint8_t stats[4];  /* bmEthernetStatistics, Indicates which Ethernet statistics functions
                      * the device collects.  See Table 42.
                      */
  uint8_t maxseg[2];   /* wMaxSegmentSize, The maximum segment size that the Ethernet device is
                      * capable of supporting.
                      */
  uint8_t nmcflts[2];  /* wNumberMCFilters, Contains the number of multicast filters that can be
                      * configured by the host.
                      */
  uint8_t  npwrflts; /* bNumberPowerFilters, Contains the number of pattern filters that are
                      * available for causing wake-up of the host.
                      */
};
#define SIZEOF_ECM_FUNCDESC 13

/* Table 43: ATM Networking Functional Descriptor */

struct cdc_atm_funcdesc_s
{
  uint8_t size;      /* bFunctionLength, Size of this descriptor */
  uint8_t type;      /* bDescriptorType, USB_DESC_TYPE_CSINTERFACE */
  uint8_t subtype;   /* bDescriptorSubType, CDC_DSUBTYPE_ATM as defined in Table 25 */
  uint8_t endid;     /* iEndSystemIdentifier, Index of End System Identifier string descriptor */
  uint8_t datacaps;  /* bmDataCapabilities, The ATM data types the device supports */
  uint8_t devstats;  /* bmATMDeviceStatistics, Indicates which optional statistics functions the
                      * device collects.
                      */
  uint8_t mxseg2[2]; /* wType2MaxSegmentSize, The maximum segment size that the Type 2 device is
                      * capable of supporting.
                      */
  uint8_t mxseg3[2];  /* wType3MaxSegmentSize, The maximum segment size that the Type 3 device is
                      * capable of supporting
                      */
  uint8_t mxvc[2];    /* wMaxVC, The maximum number of simultaneous virtual circuits the device is
                      * capable of supporting
                      */
};
#define SIZEOF_CALLMGMT_FUNCDESC 12

/* Descriptor Data Structures ***************************************************************/
/* Table 50: Line Coding Structure */

struct cdc_linecoding_s
{
  uint8_t baud[4];   /* dwDTERate, Data terminal rate, in bits per second */
  uint8_t stop;      /* bCharFormat 0=1, 1=1.5, 2=2 stop bits */
  uint8_t parity;    /* bParityType, 0=None, 1=Odd, 2=Even, 3=Mark, 4=Space */
  uint8_t nbits;     /* bDataBits, Data bits (5,6,7,8, or 16) */
};
#define SIZEOF_CDC_LINECODING 7

/* Table 55: Line Status Information Structure */

struct cdc_linestatus_s
{
  uint8_t size[2];   /* wLength, Size of this structure, in bytes */
  uint8_t ringer[4]; /* dwRingerBitmap, Ringer Configuration bitmap for this line */
  uint8_t line[4];   /* dwLineState, Defines current state of the line */
  uint32_t call[1];  /* dwCallStateN, Defines current state of call N on the line */
};

/* Table 60: Unit Parameter Structure */

struct cdc_unitparm_s
{
  uint8_t id;        /* bEntityId, Unit ID */
  uint8_t index;     /* bParameterIndex, A zero based value indicating Unit parameter index */
};

/* Table 61: Power Management Pattern Filter Structure */

/* Notification Data Structures *************************************************************/
/* Table 72: ConnectionSpeedChange Data Structure */

struct cdc_speedchange_s
{
  uint8_t us[4];     /* Contains the upstream bit rate, in bits per second */
  uint8_t ds[4];     /* Contains the downstream bit rate, in bits per second */
};

#endif /* __INCLUDE_NUTTX_USB_CDC_H */
