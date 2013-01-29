/****************************************************************************
 * include/nuttx/usb/hid.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   HID Universal Serial Bus (USB), Device Class Definition for Human
 *       Interface Devices (HID), Firmware Specification—6/27/01, Version
 *       1.11.
 *
 *   HuT Universal Serial Bus (USB), HID Usage Tables, 10/28/2004, Version
 *       1.12
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

#ifndef __INCLUDE_NUTTX_USB_HID_H
#define __INCLUDE_NUTTX_USB_HID_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usb.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

/* Subclass and Protocol ****************************************************/
/* Subclass codes (HID 4.2) */

#define USBHID_SUBCLASS_NONE              0 /* No subclass */
#define USBHID_SUBCLASS_BOOTIF            1 /* Boot Interface Subclass */

/* A variety of protocols are supported HID devices. The protocol member of
 * an Interface descriptor only has meaning if the subclass member declares
 * that the device supports a boot interface, otherwise it is 0. (HID 4.3)
 */

#define USBHID_PROTOCOL_NONE              0
#define USBHID_PROTOCOL_KEYBOARD          1
#define USBHID_PROTOCOL_MOUSE             2

/* Descriptor Requests ******************************************************/
/* "When a Get_Descriptor(Configuration) request is issued, it returns the
 *  Configuration descriptor, all Interface descriptors, all Endpoint
 *  descriptors, and the HID descriptor for each interface."
 */

/* Standard Requests (HID 7.1)
 * GET_DESCRIPTOR (HID 7.1.1):
 *
 *   bmRequestType (USB_REQ_DIR_IN | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_INTERFACE)
 *   bRequest      (USB_REQ_GETDESCRIPTOR)
 *   wValue        Descriptor Type (MS) and Descriptor Index (LS)
 *   wIndex        Interface Number
 *   wLength       Descriptor Length
 *   Data          Descriptor
 *
 * SET_DESCRIPTOR (HID 7.1.2):
 *
 *   bmRequestType (USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_INTERFACE)
 *   bRequest      (USB_REQ_SETDESCRIPTOR)
 *   wValue        Descriptor Type (MS) and Descriptor Index (LS)
 *   wIndex        Interface Number
 *   wLength       Descriptor Length
 *   Data          Descriptor
 */

/* Class Descriptor Types (HID 7.1) */

#define USBHID_DESCTYPE_HID               0x21 /* HID */
#define USBHID_DESCTYPE_REPORT            0x22 /* Report */
#define USBHID_DESCTYPE_PHYSICAL          0x23 /* Physical descriptor */

/* Class-specific requests (HID 7.2)
 *
 *   bmRequestType (                 USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE) -or- 
 *                 (USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE)
 *   bRequest      Class-specific request
 *   wValue        Varies according to request
 *   wIndex        Varies according to request
 *   wLength       Number of bytes to transfer in the data phase
 *   Data          Data
 */
 
#define USBHID_REQUEST_GETREPORT          0x01
#define USBHID_REQUEST_GETIDLE            0x02
#define USBHID_REQUEST_GETPROTOCOL        0x03
#define USBHID_REQUEST_SETREPORT          0x09
#define USBHID_REQUEST_SETIDLE            0x0a
#define USBHID_REQUEST_SETPROTOCOL        0x0b

/* Report Type (MS byte of wValue for GET_REPORT) (HID 7.2.1) */

#define USBHID_REPORTTYPE_INPUT           0x01 
#define USBHID_REPORTTYPE_OUTPUT          0x02 
#define USBHID_REPORTTYPE_FEATURE         0x03 

/* HID Descriptor ***********************************************************/

#define USBHID_COUNTRY_NONE               0x00 /* Not Supported */
#define USBHID_COUNTRY_ARABIC             0x01 /* Arabic */
#define USBHID_COUNTRY_BELGIAN            0x02 /* Belgian */
#define USBHID_COUNTRY_CANADA             0x03 /* Canadian-Bilingual */
#define USBHID_COUNTRY_CANADRFR           0x04 /* Canadian-French */
#define USBHID_COUNTRY_CZECH              0x05 /* Czech Republic */
#define USBHID_COUNTRY_DANISH             0x06 /* Danish */
#define USBHID_COUNTRY_FINNISH            0x07 /* Finnish */
#define USBHID_COUNTRY_FRENCH             0x08 /* French */
#define USBHID_COUNTRY_GERMAN             0x09 /* German */
#define USBHID_COUNTRY_GREEK              0x10 /* Greek */
#define USBHID_COUNTRY_HEBREW             0x11 /* Hebrew */
#define USBHID_COUNTRY_HUNGARY            0x12 /* Hungary */
#define USBHID_COUNTRY_ISO                0x13 /* International (ISO) */
#define USBHID_COUNTRY_ITALIAN            0x14 /* Italian */
#define USBHID_COUNTRY_JAPAN              0x15 /* Japan (Katakana) */
#define USBHID_COUNTRY_KOREAN             0x16 /* Korean  */
#define USBHID_COUNTRY_LATINAM            0x17 /* Latin American */
#define USBHID_COUNTRY_DUTCH              0x18 /* Netherlands/Dutch */
#define USBHID_COUNTRY_NORWEGIAN          0x19 /* Norwegian */
#define USBHID_COUNTRY_PERSIAN            0x20 /* Persian (Farsi) */
#define USBHID_COUNTRY_POLAND             0x21 /* Poland */
#define USBHID_COUNTRY_PORTUGUESE         0x22 /* Portuguese */
#define USBHID_COUNTRY_RUSSIA             0x23 /* Russia */
#define USBHID_COUNTRY_SLOVAKIA           0x24 /* Slovakia */
#define USBHID_COUNTRY_SPANISH            0x25 /* Spanish */
#define USBHID_COUNTRY_SWEDISH            0x26 /* Swedish */
#define USBHID_COUNTRY_SWISSFR            0x27 /* Swiss/French */
#define USBHID_COUNTRY_SWISSGR            0x28 /* Swiss/German */
#define USBHID_COUNTRY_SWITZERLAND        0x29 /* Switzerland */
#define USBHID_COUNTRY_TAIWAN             0x30 /* Taiwan */
#define USBHID_COUNTRY_TURKISHQ           0x31 /* Turkish-Q */
#define USBHID_COUNTRY_UK                 0x32 /* UK */
#define USBHID_COUNTRY_US                 0x33 /* US */
#define USBHID_COUNTRY_YUGOSLAVIA         0x34 /* Yugoslavia */
#define USBHID_COUNTRY_TURKISHF           0x35 /* Turkish-F */

/* HID report items */

#define USBHID_RPTITEM_SIZE_MASK          0x03
#  define USBHID_RPTITEM_SIZE_0           0x00 /* No data follows */
#  define USBHID_RPTITEM_SIZE_1           0x01 /* 1 byte of data follows */
#  define USBHID_RPTITEM_SIZE_2           0x02 /* 2 bytes of data follow */
#  define USBHID_RPTITEM_SIZE_4           0x03 /* 4 bytes of data follow */
#define USBHID_RPTITEM_TYPE_MASK          0x0c
#  define USBHID_RPTITEM_TYPE_MAIN        0x00
#  define USBHID_RPTITEM_TYPE_GLOBAL      0x04
#  define USBHID_RPTITEM_TYPE_LOCAL       0x08
#define USBHID_RPTITEM_TAG_MASK           0xf0

/* Main Items (HID 6.2.2.4) */

#define USBHID_MAIN_CONSTANT              (1 << 0) /* Constant(1) vs Data(0) */
#define USBHID_MAIN_VARIABLE              (1 << 1) /* Variable(1) vs Array(0) */
#define USBHID_MAIN_RELATIVE              (1 << 2) /* Relative(1) vs Absolute(0) */
#define USBHID_MAIN_WRAP                  (1 << 3) /* Wrap(1) vs No Wrap(0) */
#define USBHID_MAIN_NONLINEAR             (1 << 4) /* Non Linear(1) vs Linear(0) */
#define USBHID_MAIN_NOPREFERRED           (1 << 5) /* No Preferred (1) vs Preferred State(0) */
#define USBHID_MAIN_NULLSTATE             (1 << 6) /* Null state(1) vs No Null position(0) */
#define USBHID_MAIN_VOLATILE              (1 << 7) /* Volatile(1) vs Non volatile(0) */
#define USBHID_MAIN_BUFFEREDBYTES         (1 << 8) /* Buffered Bytes(1) vs Bit Field(0) */

#define USBHID_MAIN_SIZE(pfx)             ((pfx) & USBHID_RPTITEM_SIZE_MASK)
#define USBHID_MAIN_INPUT_PREFIX          0x80
#define USBHID_MAIN_INPUT_CONSTANT        USBHID_MAIN_CONSTANT
#define USBHID_MAIN_INPUT_VARIABLE        USBHID_MAIN_VARIABLE
#define USBHID_MAIN_INPUT_RELATIVE        USBHID_MAIN_RELATIVE
#define USBHID_MAIN_INPUT_WRAP            USBHID_MAIN_WRAP
#define USBHID_MAIN_INPUT_NONLINEAR       USBHID_MAIN_NONLINEAR
#define USBHID_MAIN_INPUT_NOPREFERRED     USBHID_MAIN_NOPREFERRED
#define USBHID_MAIN_INPUT_NULLSTATE       USBHID_MAIN_NULLSTATE
#define USBHID_MAIN_INPUT_BUFFEREDBYTES   USBHID_MAIN_BUFFEREDBYTES

#define USBHID_MAIN_OUTPUT_PREFIX         0x90
#define USBHID_MAIN_OUTPUT_CONSTANT       USBHID_MAIN_CONSTANT
#define USBHID_MAIN_OUTPUT_VARIABLE       USBHID_MAIN_VARIABLE
#define USBHID_MAIN_OUTPUT_RELATIVE       USBHID_MAIN_RELATIVE
#define USBHID_MAIN_OUTPUT_WRAP           USBHID_MAIN_WRAP
#define USBHID_MAIN_OUTPUT_NONLINEAR      USBHID_MAIN_NONLINEAR
#define USBHID_MAIN_OUTPUT_NOPREFERRED    USBHID_MAIN_NOPREFERRED
#define USBHID_MAIN_OUTPUT_NULLSTATE      USBHID_MAIN_NULLSTATE
#define USBHID_MAIN_OUTPUT_VOLATILE       USBHID_MAIN_VOLATILE
#define USBHID_MAIN_OUTPUT_BUFFEREDBYTES  USBHID_MAIN_BUFFEREDBYTES

#define USBHID_MAIN_FEATURE_PREFIX        0xb0
#define USBHID_MAIN_FEATURE_CONSTANT      USBHID_MAIN_CONSTANT
#define USBHID_MAIN_FEATURE_VARIABLE      USBHID_MAIN_VARIABLE
#define USBHID_MAIN_FEATURE_RELATIVE      USBHID_MAIN_RELATIVE
#define USBHID_MAIN_FEATURE_WRAP          USBHID_MAIN_WRAP
#define USBHID_MAIN_FEATURE_NONLINEAR     USBHID_MAIN_NONLINEAR
#define USBHID_MAIN_FEATURE_NOPREFERRED   USBHID_MAIN_NOPREFERRED
#define USBHID_MAIN_FEATURE_NULLSTATE     USBHID_MAIN_NULLSTATE
#define USBHID_MAIN_FEATURE_VOLATILE      USBHID_MAIN_VOLATILE
#define USBHID_MAIN_FEATURE_BUFFEREDBYTES USBHID_MAIN_BUFFEREDBYTES

#define USBHID_MAIN_COLLECTION_PREFIX     0xa0
#define USBHID_MAIN_COLLECTION_PHYSICAL   0x00 /* Physical (group of axes) */
#define USBHID_MAIN_COLLECTION_APPL       0x01 /* Application (mouse, keyboard) */
#define USBHID_MAIN_COLLECTION_LOGICAL    0x02 /* Logical (interrelated data) */
#define USBHID_MAIN_COLLECTION_REPORT     0x03 /* Report */
#define USBHID_MAIN_COLLECTION_ARRAY      0x04 /* Named Array */
#define USBHID_MAIN_COLLECTION_SWITCH     0x05 /* Usage Switch */
#define USBHID_MAIN_COLLECTION_MODIFIER   0x06 /* Usage Modifier */

#define USBHID_MAIN_ENDCOLLECTION_PREFIX  0xc0

/* Global Items (HID 6.2.2.7) */

#define USBHID_GLOBAL_SIZE(pfx)           ((pfx) & USBHID_RPTITEM_SIZE_MASK)
#define USBHID_GLOBAL_USAGEPAGE_PREFIX    0x04 /* Usage Page */
#define USBHID_GLOBAL_LOGICALMIN_PREFIX   0x14 /* Logical Minimum */
#define USBHID_GLOBAL_LOGICALMAX_PREFIX   0x24 /* Logical Maximum */
#define USBHID_GLOBAL_PHYSICALMIN_PREFIX  0x34 /* Physical Minimum */
#define USBHID_GLOBAL_PHYSMICALAX_PREFIX  0x44 /* Physical Maximum */
#define USBHID_GLOBAL_UNITEXP_PREFIX      0x54 /* Unit Exponent */
#define USBHID_GLOBAL_UNIT_PREFIX         0x64 /* Unit */
#define USBHID_GLOBAL_REPORTSIZE_PREFIX   0x74 /* Report Size */
#define USBHID_GLOBAL_REPORTID_PREFIX     0x84 /* Report ID */
#define USBHID_GLOBAL_REPORTCOUNT_PREFIX  0x94 /* Report Count */
#define USBHID_GLOBAL_PUSH_PREFIX         0xa4 /* Push */
#define USBHID_GLOBAL_POP_PREFIX          0xb4 /* Pop */

/* Local Items (HID 6.2.2.8) */

#define USBHID_LOCAL_SIZE(pfx)            ((pfx) & USBHID_RPTITEM_SIZE_MASK)
#define USBHID_LOCAL_USAGE_PREFIX         0x08 /* Usage */
#define USBHID_LOCAL_USAGEMIN_PREFIX      0x18 /* Usage Minimum */
#define USBHID_LOCAL_USAGEMAX_PREFIX      0x28 /* Usage Maximum */
#define USBHID_LOCAL_DESIGNATORIDX_PREFIX 0x38 /* Designator Index  */
#define USBHID_LOCAL_DESIGNATORMIN_PREFIX 0x48 /* Designator Minimum */
#define USBHID_LOCAL_DESIGNATORMAX_PREFIX 0x58 /* Designator Maximum */
#define USBHID_LOCAL_STRINGIDX_PREFIX     0x78 /* String Index */
#define USBHID_LOCAL_STRINGMIN_PREFIX     0x88 /* String Minimum */
#define USBHID_LOCAL_STRINGMAX_PREFIX     0x98 /* xx */
#define USBHID_LOCAL_DELIMITER_PREFIX     0xa8 /*Delimiter */

/* Modifier Keys (HID 8.3) */

#define USBHID_MODIFER_LCTRL              (1 << 0) /* Left Ctrl */
#define USBHID_MODIFER_LSHIFT             (1 << 1) /* Left Shift */
#define USBHID_MODIFER_LALT               (1 << 2) /* Left Alt */
#define USBHID_MODIFER_LGUI               (1 << 3) /* Left GUI */
#define USBHID_MODIFER_RCTRL              (1 << 4) /* Right Ctrl */
#define USBHID_MODIFER_RSHIFT             (1 << 5) /* Right Shift */
#define USBHID_MODIFER_RALT               (1 << 6) /* Right Alt */
#define USBHID_MODIFER_RGUI               (1 << 7) /* Right GUI */

/* Keyboard output report (1 byte) (HID B.1) */

#define USBHID_KBDOUT_NUMLOCK             (1 << 0)
#define USBHID_KBDOUT_CAPSLOCK            (1 << 1)
#define USBHID_KBDOUT_SCROLLLOCK          (1 << 2)
#define USBHID_KBDOUT_COMPOSE             (1 << 3)
#define USBHID_KBDOUT_KANA                (1 << 4)

/* Mouse input report (HID B.2) */

#define USBHID_MOUSEIN_BUTTON1            (1 << 0)
#define USBHID_MOUSEIN_BUTTON2            (1 << 1)
#define USBHID_MOUSEIN_BUTTON3            (1 << 2)

/* Joystick input report (4 bytes) (HID D.1) */

#define USBHID_JSIN_HATSWITCH_SHIFT       (0)
#define USBHID_JSIN_HATSWITCH_MASK        (15 << USBHID_JSIN_HATSWITCH_SHIFT)
#define USBHID_JSIN_BUTTON1               (1 << 4)
#define USBHID_JSIN_BUTTON2               (1 << 5)
#define USBHID_JSIN_BUTTON3               (1 << 6)
#define USBHID_JSIN_BUTTON4               (1 << 7)

/* Usage pages (HuT 3) */

#define USBHID_USAGE_PAGE_UNDEFINED       0x00 /* Undefined */
#define USBHID_USAGE_PAGE_GENERIC_DCTRL   0x01 /* Generic Desktop Controls */
#define USBHID_USAGE_PAGE_SIMCTRL         0x02 /* Simulation Controls */
#define USBHID_USAGE_PAGE_VRCTRL          0x03 /* VR Controls */
#define USBHID_USAGE_PAGE_SPORTCTRL       0x04 /* Sport Controls */
#define USBHID_USAGE_PAGE_GAMECTRL        0x05 /* Game Controls */
#define USBHID_USAGE_PAGE_GENERIC_DEVCTRL 0x06 /* Generic Device Controls */
#define USBHID_USAGE_PAGE_KBD             0x07 /* Keyboard/Keypad */
#define USBHID_USAGE_PAGE_LEDS            0x08 /* LEDs */
#define USBHID_USAGE_PAGE_BUTTON          0x09 /* Button */
#define USBHID_USAGE_PAGE_ORDINAL         0x0a /* Ordinal */
#define USBHID_USAGE_PAGE_TELEPHONY       0x0b /* Telephony */
#define USBHID_USAGE_PAGE_CONSUMER        0x0c /* Consumer */
#define USBHID_USAGE_PAGE_DIGITIZER       0x0d /* Digitizer */
                                               /* 0x0e Reserved */
#define USBHID_USAGE_PAGE_PIDPAGE         0x0f /* PID Page USB Physical Interface Device */
#define USBHID_USAGE_PAGE_UNICODE         0x10 /* Unicode */
                                               /* 0x11-13 Reserved */
#define USBHID_USAGE_PAGE_ALPHA_DISPLAY   0x14 /* Alphanumeric Display */
                                               /* 0x15-3f Reserved */
#define USBHID_USAGE_PAGE_MEDICAL         0x40 /* Medical Instruments */
                                               /* 0x41-7f Reserved */
                                               /* 0x80-83 Monitor Devices */
                                               /* 0x84-87 Power Devices */
                                               /* 0x88-8b Reserved */
#define USBHID_USAGE_PAGE_BARCODE_SCANNER 0x8c /* Bar Code Scanner page */
#define USBHID_USAGE_PAGE_SCALE           0x8d /* Scale page */
#define USBHID_USAGE_PAGE_MSR             0x8e /* Magnetic Stripe Reading (MSR) Devices */
#define USBHID_USAGE_PAGE_POS             0x8f /* Point of Sale devices */
#define USBHID_USAGE_PAGE_CAMERA_CTRL     0x90 /* Camera Control Page */

/* Generic Desktop Page Usage IDs (HuT 4) */

#define USBHID_DCTRLUSE_UNDEFINED         0x00 /* Undefined */
#define USBHID_DCTRLUSE_POINTER           0x01 /* Pointer */
#define USBHID_DCTRLUSE_MOUSE             0x02 /* Mouse */
                                               /* 0x03 Reserved */
#define USBHID_DCTRLUSE_JOYSTICK          0x04 /* Joystick */
#define USBHID_DCTRLUSE_GAMEPAD           0x05 /* Game Pad */
#define USBHID_DCTRLUSE_KEYBOARD          0x06 /* Keyboard */
#define USBHID_DCTRLUSE_KEYPAD            0x07 /* Keypad */
#define USBHID_DCTRLUSE_MULTIAXIS         0x08 /* Multi-axis Controller */
#define USBHID_DCTRLUSE_TABLET            0x09 /* Tablet PC System Controls */
                                               /* 0x0a-2f Reserved */
#define USBHID_DCTRLUSE_X                 0x30 /* X */
#define USBHID_DCTRLUSE_Y                 0x31 /* Y */
#define USBHID_DCTRLUSE_Z                 0x32 /* Z */
#define USBHID_DCTRLUSE_RX                0x33 /* Rx */
#define USBHID_DCTRLUSE_RY                0x34 /* Ry */
#define USBHID_DCTRLUSE_RZ                0x35 /* Rz */
#define USBHID_DCTRLUSE_SLIDER            0x36 /* Slider */
#define USBHID_DCTRLUSE_DIAL              0x37 /* Dial */
#define USBHID_DCTRLUSE_WHEEL             0x38 /* Wheel */
#define USBHID_DCTRLUSE_HATSWITCH         0x39 /* Hat switch */
#define USBHID_DCTRLUSE_COUNTED           0x3a /* Counted Buffer */
#define USBHID_DCTRLUSE_BYTECOUNT         0x3b /* Byte Count */
#define USBHID_DCTRLUSE_MOTION            0x3c /* Motion Wakeup */
#define USBHID_DCTRLUSE_START             0x3d /* Start */
#define USBHID_DCTRLUSE_SELECT            0x3e /* Select */
                                               /* 0x3f Reserved */
#define USBHID_DCTRLUSE_VX                0x40 /* Vx */
#define USBHID_DCTRLUSE_VY                0x41 /* Vy */
#define USBHID_DCTRLUSE_VZ                0x42 /* Vz */
#define USBHID_DCTRLUSE_VBRX              0x43 /* Vbrx */
#define USBHID_DCTRLUSE_VBRY              0x44 /* Vbry */
#define USBHID_DCTRLUSE_VBRZ              0x45 /* Vbrz */
#define USBHID_DCTRLUSE_VNO               0x46 /* Vno */
#define USBHID_DCTRLUSE_FEATURE           0x47 /* Feature Notification */
#define USBHID_DCTRLUSE_RESOLUTION        0x48 /* Resolution Multiplier */
                                               /* 0x49-7f Reserved */
#define USBHID_DCTRLUSE_CONTROL           0x80 /* System Control */
#define USBHID_DCTRLUSE_POWERDOWN         0x81 /* System Power Down */
#define USBHID_DCTRLUSE_SLEEP             0x82 /* System Sleep */
#define USBHID_DCTRLUSE_WAKEUP            0x83 /* System Wake Up */
#define USBHID_DCTRLUSE_CONTEXT_MENU      0x84 /* System Context Menu */
#define USBHID_DCTRLUSE_MAIN_MENU         0x85 /* System Main Menu */
#define USBHID_DCTRLUSE_APP_MENU          0x86 /* System App Menu */
#define USBHID_DCTRLUSE_MENU_HELP         0x87 /* System Menu Help */
#define USBHID_DCTRLUSE_MENU_EXIT         0x88 /* System Menu Exit */
#define USBHID_DCTRLUSE_MENU_SELECT       0x89 /* System Menu Select */
#define USBHID_DCTRLUSE_MENU_RIGHT        0x8a /* System Menu Right */
#define USBHID_DCTRLUSE_MENU_LEFT         0x8b /* System Menu Left */
#define USBHID_DCTRLUSE_MENU_UP           0x8c /* System Menu Up */
#define USBHID_DCTRLUSE_MENU_DOWN         0x8d /* System Menu Down */
#define USBHID_DCTRLUSE_COLD_RESTART      0x8e /* System Cold Restart */
#define USBHID_DCTRLUSE_WARM_RESTART      0x8f /* System Warm Restart */
#define USBHID_DCTRLUSE_DPAD_UP           0x90 /* D-pad Up */
#define USBHID_DCTRLUSE_DPAD_DOWN         0x91 /* D-pad Down */
#define USBHID_DCTRLUSE_DPAD_RIGHT        0x92 /* D-pad Right */
#define USBHID_DCTRLUSE_DPAD_LEFT         0x93 /* D-pad Left */
                                               /* 0x94-9f Reserved */
#define USBHID_DCTRLUSE_DOCK              0xa0 /* System Dock */
#define USBHID_DCTRLUSE_UNDOCK            0xa1 /* System Undock */
#define USBHID_DCTRLUSE_SETUP             0xa2 /* System Setup */
#define USBHID_DCTRLUSE_BREAK             0xa3 /* System Break */
#define USBHID_DCTRLUSE_DEBUG_BREAK       0xa4 /* System Debugger Break */
#define USBHID_DCTRLUSE_APP_BREAK         0xa5 /* Application Break */
#define USBHID_DCTRLUSE_APP_DEBUG_BREAK   0xa6 /* Application Debugger Break */
#define USBHID_DCTRLUSE_MUTE              0xa7 /* System Speaker Mute */
#define USBHID_DCTRLUSE_HIBERNATE         0xa8 /* System Hibernate */
                                               /* 0xa9-af Reserved */
#define USBHID_DCTRLUSE_DISPLAY_INVERT    0xb0 /* System Display Invert */
#define USBHID_DCTRLUSE_DISPALY_INTERNAL  0xb1 /* System Display Internal */
#define USBHID_DCTRLUSE_DISPLAY_EXTERNAL  0xb2 /* System Display External */
#define USBHID_DCTRLUSE_DISPLAY_BOTH      0xb3 /* System Display Both */
#define USBHID_DCTRLUSE_DISPLAY_DUAL      0xb4 /* System Display Dual */
#define USBHID_DCTRLUSE_DISPLAY_TOGGLE    0xb5 /* System Display Toggle Int/Ext */
#define USBHID_DCTRLUSE_DISPLAY_SWAP      0xb6 /* System Display Swap */
#define USBHID_DCTRLUSE_                  0xb7 /* System Display LCD Autoscale */
                                               /* 0xb8-ffff Reserved */

/* Keyboard usage IDs (HuT 10) */

#define USBHID_KBDUSE_NONE                0x00 /* Reserved (no event indicated) */
#define USBHID_KBDUSE_ERRORROLLOVER       0x01 /* Keyboard ErrorRollOver */
#define USBHID_KBDUSE_POSTFAIL            0x02 /* Keyboard POSTFail */
#define USBHID_KBDUSE_ERRUNDEF            0x03 /* Keyboard ErrorUndefined */
#define USBHID_KBDUSE_A                   0x04 /* Keyboard a or A (B-Z follow) */
#define USBHID_KBDUSE_1                   0x1e /* Keyboard 1 (2-9 follow) */
#define USBHID_KBDUSE_EXCLAM              0x1e /* Keyboard 1 and ! */
#define USBHID_KBDUSE_AT                  0x1f /* Keyboard 2 and @ */
#define USBHID_KBDUSE_POUND               0x20 /* Keyboard 3 and # */
#define USBHID_KBDUSE_DOLLAR              0x21 /* Keyboard 4 and $ */
#define USBHID_KBDUSE_PERCENT             0x22 /* Keyboard 5 and % */
#define USBHID_KBDUSE_CARAT               0x23 /* Keyboard 6 and ^ */
#define USBHID_KBDUSE_AMPERSAND           0x24 /* Keyboard 7 and & */
#define USBHID_KBDUSE_ASTERISK            0x25 /* Keyboard 8 and * */
#define USBHID_KBDUSE_LPAREN              0x26 /* Keyboard 9 and ( */
#define USBHID_KBDUSE_0                   0x27 /* Keyboard 0 and ) */
#define USBHID_KBDUSE_RPAREN              0x27 /* Keyboard 0 and ) */
#define USBHID_KBDUSE_ENTER               0x28 /* Keyboard Return (ENTER) */
#define USBHID_KBDUSE_ESCAPE              0x29 /* Keyboard ESCAPE */
#define USBHID_KBDUSE_DELETE              0x2a /* Keyboard DELETE (Backspace) */
#define USBHID_KBDUSE_TAB                 0x2b /* Keyboard Tab */
#define USBHID_KBDUSE_SPACE               0x2c /* Keyboard Spacebar */
#define USBHID_KBDUSE_HYPHEN              0x2d /* Keyboard - and (underscore) */
#define USBHID_KBDUSE_UNDERSCORE          0x2d /* Keyboard - and (underscore) */
#define USBHID_KBDUSE_EQUAL               0x2e /* Keyboard = and + */
#define USBHID_KBDUSE_PLUS                0x2e /* Keyboard = and + */
#define USBHID_KBDUSE_LBRACKET            0x2f /* Keyboard [ and { */
#define USBHID_KBDUSE_LBRACE              0x2f /* Keyboard [ and { */
#define USBHID_KBDUSE_RBRACKET            0x30 /* Keyboard ] and } */
#define USBHID_KBDUSE_RBRACE              0x30 /* Keyboard ] and } */
#define USBHID_KBDUSE_BSLASH              0x31 /* Keyboard \ and | */
#define USBHID_KBDUSE_VERTBAR             0x31 /* Keyboard \ and | */
#define USBHID_KBDUSE_NONUSPOUND          0x32 /* Keyboard Non-US # and ~ */
#define USBHID_KBDUSE_TILDE               0x32 /* Keyboard Non-US # and ~ */
#define USBHID_KBDUSE_SEMICOLON           0x33 /* Keyboard ; and : */
#define USBHID_KBDUSE_COLON               0x33 /* Keyboard ; and : */
#define USBHID_KBDUSE_SQUOTE              0x34 /* Keyboard ' and " */
#define USBHID_KBDUSE_DQUOUTE             0x34 /* Keyboard ' and " */
#define USBHID_KBDUSE_GACCENT             0x35 /* Keyboard Grave Accent and Tilde */
#define USBHID_KBDUSE_GTILDE              0x35 /* Keyboard Grave Accent and Tilde */
#define USBHID_KBDUSE_COMMON              0x36 /* Keyboard , and < */
#define USBHID_KBDUSE_LT                  0x36 /* Keyboard , and < */
#define USBHID_KBDUSE_PERIOD              0x37 /* Keyboard . and > */
#define USBHID_KBDUSE_GT                  0x37 /* Keyboard . and > */
#define USBHID_KBDUSE_DIV                 0x38 /* Keyboard / and ? */
#define USBHID_KBDUSE_QUESTION            0x38 /* Keyboard / and ? */
#define USBHID_KBDUSE_CAPSLOCK            0x39 /* Keyboard Caps Lock */
#define USBHID_KBDUSE_F1                  0x3a /* Keyboard F1 */
#define USBHID_KBDUSE_F2                  0x3b /* Keyboard F2 */
#define USBHID_KBDUSE_F3                  0x3c /* Keyboard F3 */
#define USBHID_KBDUSE_F4                  0x3d /* Keyboard F4 */
#define USBHID_KBDUSE_F5                  0x3e /* Keyboard F5 */
#define USBHID_KBDUSE_F6                  0x3f /* Keyboard F6 */
#define USBHID_KBDUSE_F7                  0x40 /* Keyboard F7 */
#define USBHID_KBDUSE_F8                  0x41 /* Keyboard F8 */
#define USBHID_KBDUSE_F9                  0x42 /* Keyboard F9 */
#define USBHID_KBDUSE_F10                 0x43 /* Keyboard F10 */
#define USBHID_KBDUSE_F11                 0x44 /* Keyboard F11 */
#define USBHID_KBDUSE_F12                 0x45 /* Keyboard F12 */
#define USBHID_KBDUSE_PRINTSCN            0x46 /* Keyboard PrintScreen */
#define USBHID_KBDUSE_SCROLLLOCK          0x47 /* Keyboard Scroll Lock */
#define USBHID_KBDUSE_PAUSE               0x48 /* Keyboard Pause */
#define USBHID_KBDUSE_INSERT              0x49 /* Keyboard Insert */
#define USBHID_KBDUSE_HOME                0x4a /* Keyboard Home */
#define USBHID_KBDUSE_PAGEUP              0x4b /* Keyboard PageUp */
#define USBHID_KBDUSE_DELFWD              0x4c /* Keyboard Delete Forward */
#define USBHID_KBDUSE_END                 0x4d /* Keyboard End */
#define USBHID_KBDUSE_PAGEDOWN            0x4e /* Keyboard PageDown */
#define USBHID_KBDUSE_RIGHT               0x4f /* eyboard RightArrow */
#define USBHID_KBDUSE_LEFT                0x50 /* Keyboard LeftArrow */
#define USBHID_KBDUSE_DOWN                0x5a /* Keyboard DownArrow */
#define USBHID_KBDUSE_UP                  0x52 /* Keyboard UpArrow */
#define USBHID_KBDUSE_KPDNUMLOCK          0x53 /* Keypad Num Lock and Clear */
#define USBHID_KBDUSE_KPDNUMLOCKCLEAR     0x53 /* Keypad Num Lock and Clear */
#define USBHID_KBDUSE_KPDDIV              0x54 /* Keypad / */
#define USBHID_KBDUSE_KPDMUL              0x55 /* Keypad * */
#define USBHID_KBDUSE_KPDHMINUS           0x56 /* Keypad - */
#define USBHID_KBDUSE_KPDPLUS             0x57 /* Keypad + */
#define USBHID_KBDUSE_KPDEMTER            0x58 /* Keypad ENTER */
#define USBHID_KBDUSE_KPD1                0x59 /* Keypad 1 (2-9 follow) */
#define USBHID_KBDUSE_KPDEND              0x59 /* Keypad 1 and End */
#define USBHID_KBDUSE_KPDDOWN             0x5a /* Keypad 2 and Down Arrow */
#define USBHID_KBDUSE_KPDPAGEDN           0x5b /* Keypad 3 and PageDn */
#define USBHID_KBDUSE_KPDLEFT             0x5c /* Keypad 4 and Left Arrow */
#define USBHID_KBDUSE_KPDRIGHT            0x5e /* Keypad 6 and Right Arrow */
#define USBHID_KBDUSE_KPDHOME             0x5f /* Keypad 7 and Home */
#define USBHID_KBDUSE_KPDUP               0x60 /* Keypad 8 and Up Arrow */
#define USBHID_KBDUSE_KPDPAGEUP           0x61 /* Keypad 9 and PageUp */
#define USBHID_KBDUSE_KPD0                0x62 /* Keypad 0 and Insert */
#define USBHID_KBDUSE_KPDINSERT           0x62 /* Keypad 0 and Insert */
#define USBHID_KBDUSE_KPDDECIMALPT        0x63 /* Keypad . and Delete */
#define USBHID_KBDUSE_KPDDELETE           0x63 /* Keypad . and Delete */
#define USBHID_KBDUSE_NONUSBSLASH         0x64 /* Keyboard Non-US \ and | */
#define USBHID_KBDUSE_NONUSVERT           0x64 /* Keyboard Non-US \ and | */
#define USBHID_KBDUSE_APPLICATION         0x65 /* Keyboard Application */
#define USBHID_KBDUSE_POWER               0x66 /* Keyboard Power */
#define USBHID_KBDUSE_KPDEQUAL            0x67 /* Keypad = */
#define USBHID_KBDUSE_F13                 0x68 /* Keyboard F13 */
#define USBHID_KBDUSE_F14                 0x69 /* Keyboard F14 */
#define USBHID_KBDUSE_F15                 0x6a /* Keyboard F15 */
#define USBHID_KBDUSE_F16                 0x6b /* Keyboard F16 */
#define USBHID_KBDUSE_F17                 0x6c /* Keyboard F17 */
#define USBHID_KBDUSE_F18                 0x6d /* Keyboard F18 */
#define USBHID_KBDUSE_F19                 0x6e /* Keyboard F19 */
#define USBHID_KBDUSE_F20                 0x6f /* Keyboard F20 */
#define USBHID_KBDUSE_F21                 0x70 /* Keyboard F21 */
#define USBHID_KBDUSE_F22                 0x71 /* Keyboard F22 */
#define USBHID_KBDUSE_F23                 0x72 /* Keyboard F23 */
#define USBHID_KBDUSE_F24                 0x73 /* Keyboard F24 */
#define USBHID_KBDUSE_EXECUTE             0x74 /* Keyboard Execute */
#define USBHID_KBDUSE_HELP                0x75 /* Keyboard Help */
#define USBHID_KBDUSE_MENU                0x76 /* Keyboard Menu */
#define USBHID_KBDUSE_SELECT              0x77 /* Keyboard Select */
#define USBHID_KBDUSE_STOP                0x78 /* Keyboard Stop */
#define USBHID_KBDUSE_AGAIN               0x79 /* Keyboard Again */
#define USBHID_KBDUSE_UNDO                0x7a /* Keyboard Undo */
#define USBHID_KBDUSE_CUT                 0x7b /* Keyboard Cut */
#define USBHID_KBDUSE_COPY                0x7c /* Keyboard Copy */
#define USBHID_KBDUSE_PASTE               0x7d /* Keyboard Paste */
#define USBHID_KBDUSE_FIND                0x7e /* Keyboard Find */
#define USBHID_KBDUSE_MUTE                0x7f /* Keyboard Mute */
#define USBHID_KBDUSE_VOLUP               0x80 /* Keyboard Volume Up */
#define USBHID_KBDUSE_VOLDOWN             0x81 /* Keyboard Volume Down */
#define USBHID_KBDUSE_LCAPSLOCK           0x82 /* Keyboard Locking Caps Lock */
#define USBHID_KBDUSE_LNUMLOCK            0x83 /* Keyboard Locking Num Lock */
#define USBHID_KBDUSE_LSCROLLLOCK         0x84 /* Keyboard Locking Scroll Lock */
#define USBHID_KBDUSE_KPDCOMMA            0x85 /* Keypad Comma */
#define USBHID_KBDUSE_KPDEQUALSIGN        0x86 /* Keypad Equal Sign */
#define USBHID_KBDUSE_INTERNATIONAL1      0x87 /* Keyboard International 1 */
#define USBHID_KBDUSE_INTERNATIONAL2      0x88 /* Keyboard International 2 */
#define USBHID_KBDUSE_INTERNATIONAL3      0x89 /* Keyboard International 3 */
#define USBHID_KBDUSE_INTERNATIONAL4      0x8a /* Keyboard International 4 */
#define USBHID_KBDUSE_INTERNATIONAL5      0x8b /* Keyboard International 5 */
#define USBHID_KBDUSE_INTERNATIONAL6      0x8c /* Keyboard International 6 */
#define USBHID_KBDUSE_INTERNATIONAL7      0x8d /* Keyboard International 7 */
#define USBHID_KBDUSE_INTERNATIONAL8      0x8e /* Keyboard International 8 */
#define USBHID_KBDUSE_INTERNATIONAL9      0x8f /* Keyboard International 9 */
#define USBHID_KBDUSE_LANG1               0x90 /* Keyboard LANG1 */
#define USBHID_KBDUSE_LANG2               0x91 /* Keyboard LANG2 */
#define USBHID_KBDUSE_LANG3               0x92 /* Keyboard LANG3 */
#define USBHID_KBDUSE_LANG4               0x93 /* Keyboard LANG4 */
#define USBHID_KBDUSE_LANG5               0x94 /* Keyboard LANG5 */
#define USBHID_KBDUSE_LANG6               0x95 /* Keyboard LANG6 */
#define USBHID_KBDUSE_LANG7               0x96 /* Keyboard LANG7 */
#define USBHID_KBDUSE_LANG8               0x97 /* Keyboard LANG8 */
#define USBHID_KBDUSE_LANG9               0x98 /* Keyboard LANG9 */
#define USBHID_KBDUSE_ALTERASE            0x99 /* Keyboard Alternate Erase */
#define USBHID_KBDUSE_SYSREQ              0x9a /* Keyboard SysReq/Attention */
#define USBHID_KBDUSE_CANCEL              0x9b /* Keyboard Cancel */
#define USBHID_KBDUSE_CLEAR               0x9c /* Keyboard Clear */
#define USBHID_KBDUSE_PRIOR               0x9d /* Keyboard Prior */
#define USBHID_KBDUSE_RETURN              0x9e /* Keyboard Return */
#define USBHID_KBDUSE_SEPARATOR           0x9f /* Keyboard Separator */
#define USBHID_KBDUSE_OUT                 0xa0 /* Keyboard Out */
#define USBHID_KBDUSE_OPER                0xa1 /* Keyboard Oper */
#define USBHID_KBDUSE_CLEARAGAIN          0xa2 /* Keyboard Clear/Again */
#define USBHID_KBDUSE_CLRSEL              0xa3 /* Keyboard CrSel/Props */
#define USBHID_KBDUSE_EXSEL               0xa4 /* Keyboard ExSel */
#define USBHID_KBDUSE_KPD00               0xb0 /* Keypad 00 */
#define USBHID_KBDUSE_KPD000              0xb1 /* Keypad 000 */
#define USBHID_KBDUSE_THOUSEPARATOR       0xb2 /* Thousands Separator */
#define USBHID_KBDUSE_DECSEPARATOR        0xb3 /* Decimal Separator */
#define USBHID_KBDUSE_CURRUNIT            0xb4 /* Currency Unit */
#define USBHID_KBDUSE_CURRSUBUNIT         0xb5 /* Currency Sub-unit */
#define USBHID_KBDUSE_KPDLPAREN           0xb6 /* Keypad ( */
#define USBHID_KBDUSE_KPDRPAREN           0xb7 /* Keypad ) */
#define USBHID_KBDUSE_KPDLBRACE           0xb8 /* Keypad { */
#define USBHID_KBDUSE_KPDRBRACE           0xb9 /* Keypad } */
#define USBHID_KBDUSE_KPDTAB              0xba /* Keypad Tab */
#define USBHID_KBDUSE_KPDBACKSPACE        0xbb /* Keypad Backspace */
#define USBHID_KBDUSE_KPDA                0xbc /* Keypad A (B-F follow) */
#define USBHID_KBDUSE_KPDXOR              0xc2 /* Keypad XOR */
#define USBHID_KBDUSE_KPDEXP              0xc3 /* Keypad ^ */
#define USBHID_KBDUSE_KPDPERCENT          0xc4 /* Keypad % */
#define USBHID_KBDUSE_KPDLT               0xc5 /* Keypad < */
#define USBHID_KBDUSE_KPDGT               0xc6 /* Keypad > */
#define USBHID_KBDUSE_KPDAMPERSAND        0xc7 /* Keypad & */
#define USBHID_KBDUSE_KPDAND              0xc8 /* Keypad && */
#define USBHID_KBDUSE_KPDVERT             0xc9 /* Keypad | */
#define USBHID_KBDUSE_KPDOR               0xca /* Keypad || */
#define USBHID_KBDUSE_KPDCOLON            0xcb /* Keypad : */
#define USBHID_KBDUSE_KPDPOUND            0xcc /* Keypad # */
#define USBHID_KBDUSE_KPDSPACE            0xcd /* Keypad Space */
#define USBHID_KBDUSE_KPDAT               0xce /* Keypad @ */
#define USBHID_KBDUSE_KPDEXCLAM           0xcf /* Keypad ! */
#define USBHID_KBDUSE_KPDMEMSTORE         0xd0 /* Keypad Memory Store */
#define USBHID_KBDUSE_KPDMEMRECALL        0xd1 /* Keypad Memory Recall */
#define USBHID_KBDUSE_KPDMEMCLEAR         0xd2 /* Keypad Memory Clear */
#define USBHID_KBDUSE_KPDMEMADD           0xd3 /* Keypad Memory Add */
#define USBHID_KBDUSE_KPDMEMSUB           0xd4 /* Keypad Memory Subtract */
#define USBHID_KBDUSE_KPDMEMMULT          0xd5 /* Keypad Memory Multiply */
#define USBHID_KBDUSE_KPDMEMDIV           0xd6 /* Keypad Memory Divide */
#define USBHID_KBDUSE_KPDPLUSMINUS        0xd7 /* Keypad +/- */
#define USBHID_KBDUSE_KPDCLEAR            0xd8 /* Keypad Clear */
#define USBHID_KBDUSE_KPDCLEARENTRY       0xd9 /* Keypad Clear Entry */
#define USBHID_KBDUSE_KPDBINARY           0xda /* Keypad Binary */
#define USBHID_KBDUSE_KPDOCTAL            0xdb /* Keypad Octal */
#define USBHID_KBDUSE_KPDDECIMAL          0xdc /* Keypad Decimal */
#define USBHID_KBDUSE_KPDHEXADECIMAL      0xdd /* Keypad Hexadecimal */
#define USBHID_KBDUSE_LCTRL               0xe0 /* Keyboard LeftControl */
#define USBHID_KBDUSE_LSHIFT              0xe1 /* Keyboard LeftShift */
#define USBHID_KBDUSE_LALT                0xe2 /* Keyboard LeftAlt */
#define USBHID_KBDUSE_LGUI                0xe3 /* Keyboard Left GUI */
#define USBHID_KBDUSE_RCTRL               0xe4 /* Keyboard RightControl */
#define USBHID_KBDUSE_RSHIFT              0xe5 /* Keyboard RightShift */
#define USBHID_KBDUSE_RALT                0xe6 /* Keyboard RightAlt */
#define USBHID_KBDUSE_RGUI                0xe7 /* Keyboard Right GUI */

#define USBHID_KBDUSE_MAX                 0xe7

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* HID Descriptor (HID 6.2.1) ***********************************************/

struct usbhid_descriptor_s
{
  uint8_t len;        /* Size of the HID descriptor */
  uint8_t type;       /* HID descriptor type */
  uint8_t hid[2];     /* HID class specification release */
  uint8_t country;    /* Country code */
  uint8_t ndesc;      /* Number of descriptors (>=1) */
  uint8_t classdesc;  /* Class descriptor type (See 7.1) */
  uint8_t desclen[2]; /* Size of the report descriptor */
  uint8_t optdesc;    /* Type of optional descriptor */
  uint8_t optlen[2];  /* Size of the optional descriptor */
};

/* Standard Reports *********************************************************/
/* Keyboard input report (8 bytes) (HID B.1) */

struct usbhid_kbdreport_s
{
  uint8_t modifier;  /* Modifier keys. See USBHID_MODIFIER_* definitions */
  uint8_t reserved;
  uint8_t key[6];    /* Keycode 1-6 */
};

/* Keyboard output report (1 byte) (HID B.1), see USBHID_KBDOUT_* definitions */

/* Mouse input report (HID B.2) */

struct usbhid_mousereport_s
{
  uint8_t buttons;   /* See USBHID_MOUSEIN_* definitions */
  uint8_t xdisp;     /* X displacement */
  uint8_t ydisp;     /* y displacement */
                     /* Device specific additional bytes may follow */
};

/* Joystick input report (1 bytes) (HID D.1) */

struct usbhid_jsreport_s
{
  uint8_t xpos;      /* X position */
  uint8_t ypos;      /* X position */
  uint8_t buttons;   /* See USBHID_JSIN_* definitions */
  uint8_t throttle;  /* Throttle */
};

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_HID_H */
