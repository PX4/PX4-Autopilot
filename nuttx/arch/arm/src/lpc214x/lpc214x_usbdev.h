/*******************************************************************************
 * arch/arm/src/lpc214x/lpc214x_usbdev.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
 *******************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC214X_LPC214X_USBDEV_H
#define __ARCH_ARM_SRC_LPC214X_LPC214X_USBDEV_H

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/* PINSEL1 bit definitions for UART0/1:
 *
 * P0.23 = 01 to enable VBus sense (bits 14-15)
 * P0.31 = 10 to enable CONNECT (bits 30-31)
 */

#define LPC214X_USBDEV_PINSEL        (0x80004000)  /* PINSEL1 value for USB */
#define LPC214X_USBDEV_PINMASK       (0xc000c000)  /* PINSEL1 mask for USB */

/* USB RAM  ********************************************************************/

#define LPC214X_USBDEV_RAMBASE       (0x7fd00000)
#define LPC214X_USBDEV_RAMSIZE       (8*1024)

/* USB register address definitions ********************************************/

#define LPC214X_USBDEV_PLLCON        (0xe01fc0a0)
#define LPC214X_USBDEV_PLLCFG        (0xe01fc0a4)
#define LPC214X_USBDEV_PLLSTAT       (0xe01fc0a8)
#define LPC214X_USBDEV_PLLFEED       (0xe01fc0ac)

#define LPC214X_USBDEV_INTST         (0xe01fc1c0)

#define LPC214X_USBDEV_DEVINTST      (0xe0090000)
#define LPC214X_USBDEV_DEVINTEN      (0xe0090004)
#define LPC214X_USBDEV_DEVINTCLR     (0xe0090008)
#define LPC214X_USBDEV_DEVINTSET     (0xe009000c)
#define LPC214X_USBDEV_CMDCODE       (0xe0090010)
#define LPC214X_USBDEV_CMDDATA       (0xe0090014)
#define LPC214X_USBDEV_RXDATA        (0xe0090018)
#define LPC214X_USBDEV_TXDATA        (0xe009001c)
#define LPC214X_USBDEV_RXPLEN        (0xe0090020)
#define LPC214X_USBDEV_TXPLEN        (0xe0090024)
#define LPC214X_USBDEV_CTRL          (0xe0090028)
#define LPC214X_USBDEV_DEVINTPRI     (0xe009002c)
#define LPC214X_USBDEV_EPINTST       (0xe0090030)
#define LPC214X_USBDEV_EPINTEN       (0xe0090034)
#define LPC214X_USBDEV_EPINTCLR      (0xe0090038)
#define LPC214X_USBDEV_EPINTSET      (0xe009003c)
#define LPC214X_USBDEV_EPINTPRI      (0xe0090040)
#define LPC214X_USBDEV_REEP          (0xe0090044)
#define LPC214X_USBDEV_EPIND         (0xe0090048)
#define LPC214X_USBDEV_MAXPSIZE      (0xe009004c)
#define LPC214X_USBDEV_DMARST        (0xe0090050)
#define LPC214X_USBDEV_DMARCLR       (0xe0090054)
#define LPC214X_USBDEV_DMARSET       (0xe0090058)

#define LPC214X_USBDEV_UDCAH         (0xe0090080)
#define LPC214X_USBDEV_EPDMAST       (0xe0090084)
#define LPC214X_USBDEV_EPDMAEN       (0xe0090088)
#define LPC214X_USBDEV_EPDMADIS      (0xe009008c)
#define LPC214X_USBDEV_DMAINTST      (0xe0090090)
#define LPC214X_USBDEV_DMAINTEN      (0xe0090094)

#define LPC214X_USBDEV_EOTINTST      (0xe00900a0)
#define LPC214X_USBDEV_EOTINTCLR     (0xe00900a4)
#define LPC214X_USBDEV_EOTINTSET     (0xe00900a8)
#define LPC214X_USBDEV_NDDRINTST     (0xe00900ac)
#define LPC214X_USBDEV_NDDRINTCLR    (0xe00900b0)
#define LPC214X_USBDEV_NDDRINTSET    (0xe00900b4)
#define LPC214X_USBDEV_SYSERRINTST   (0xe00900b8)
#define LPC214X_USBDEV_SYSERRINTCLR  (0xe00900bc)
#define LPC214X_USBDEV_SYSERRINTSET  (0xe00900c0)

/* USB register bit definitions ************************************************/

/* INTST bit definitions */

#define USBDEV_INTST_REQLP           (0x00000001)
#define USBDEV_INTST_REQHP           (0x00000002)
#define USBDEV_INTST_REQDMA          (0x00000004)
#define USBDEV_INTST_NEEDCLOCK       (0x00000100)
#define USBDEV_INTST_ENUSBINTS       (0x80000000)
#define USBDEV_INTST_MASK            (0x80000107)

/* DEVINTST/DEVINTEN/DEVINTCLR/DEVINTSET bit definitions */

#define USBDEV_DEVINT_FRAME          (0x00000001)
#define USBDEV_DEVINT_EPFAST         (0x00000002)
#define USBDEV_DEVINT_EPSLOW         (0x00000004)
#define USBDEV_DEVINT_DEVSTAT        (0x00000008)
#define USBDEV_DEVINT_CCEMTY         (0x00000010)
#define USBDEV_DEVINT_CDFULL         (0x00000020)
#define USBDEV_DEVINT_RXENDPKT       (0x00000040)
#define USBDEV_DEVINT_TXENDPKT       (0x00000080)
#define USBDEV_DEVINT_EPRLZED        (0x00000100)
#define USBDEV_DEVINT_EPRINT         (0x00000200)
#define USBDEV_DEVINT_MASK           (0x000003ff)

/* DEVINTPRI bit definitions */

#define USBDEV_DEVINTPRI_FRAME       (0x00000001)
#define USBDEV_DEVINTPRI_EPFAST      (0x00000002)
#define USBDEV_DEVINTPRI_MASK        (0x00000003)

/* RXPLEN bit definitions */

#define USBDEV_RXPLEN_PKTLENGTH      (0x000003ff)
#define USBDEV_RXPLEN_PKTLENGTH_MASK (0x000003ff)
#define USBDEV_RXPLEN_DV             (0x00000400)
#define USBDEV_RXPLEN_PKTRDY         (0x00000800)
#define USBDEV_RXPLEN_MASK           (0x00000fff)

/* TXPLEN bit definitions */

#define USBDEV_TXPLEN_PKTLENGTH      (0x000003ff)
#define USBDEV_TXPLEN_MASK           (0x000003ff)

/* USBCTRL bit definitions */

#define USBDEV_CTRL_RDEN             (0x00000001)
#define USBDEV_CTRL_WREN             (0x00000002)
#define USBDEV_CTRL_LOGENDPOINT      (0x0000003c)
#define USBDEV_CTRL_MASK             (0x0000003f)

/* CMDCODE bit definitions */

#define USBDEV_CMDCODE_CMDPHASE      (0x0000ff00)
#define USBDEV_CMDCODE_CMDCODE       (0x00ff0000)
#define USBDEV_CMDCODE_MASK          (0x00ffff00)

/* DMAINSTST/DMAINSTEN bit defintions */

#define USBDEV_DMAINST_EOT           (0x00000001)
#define USBDEV_DMAINST_NDDR          (0x00000002)
#define USBDEV_DMAINST_SE            (0x00000004)
#define USBDEV_DMAINST_MASK          (0x00000007)

/* Device Status Bits */

#define USBDEV_EPSTALL               (0x00000001)
#define USBDEV_EPSTALLSTATUS         (0x00000002)
#define USBDEV_EPSETUPPACKET         (0x00000004)
#define USBDEV_EPPOSSTATUS           (0x00000010)
#define USBDEV_EPCONDSTALL           (0x00000080)

/* USB Control register bit definitions */

#define LPC214X_USBCTRL_RDEN         (0x00000001) /* Bit 0=1: Read is enabled */
#define LPC214X_USBCTRL_WREN         (0x00000002) /* Bit 0=1: Write is enabled */
#define LPC214X_USBCTRL_EPMASK       (0x0000003c) /* Bits 2:5: Logical endpoint 0-15 */

/* Endpoints *******************************************************************/

#define LPC214X_EP0_OUT                0
#define LPC214X_EP0_IN                 1
#define LPC214X_CTRLEP_OUT            LPC214X_EP0_OUT
#define LPC214X_CTRLEP_IN             LPC214X_EP0_IN
#define LPC214X_EP1_OUT                2
#define LPC214X_EP1_IN                 3
#define LPC214X_EP2_OUT                4
#define LPC214X_EP2_IN                 5
#define LPC214X_EP3_OUT                6
#define LPC214X_EP3_IN                 7
#define LPC214X_EP4_OUT                8
#define LPC214X_EP4_IN                 9
#define LPC214X_EP5_OUT               10
#define LPC214X_EP5_IN                11
#define LPC214X_EP6_OUT               12
#define LPC214X_EP6_IN                13
#define LPC214X_EP7_OUT               14
#define LPC214X_EP7_IN                15
#define LPC214X_EP8_OUT               16
#define LPC214X_EP8_IN                17
#define LPC214X_EP9_OUT               18
#define LPC214X_EP9_IN                19
#define LPC214X_EP10_OUT              20
#define LPC214X_EP10_IN               21
#define LPC214X_EP11_OUT              22
#define LPC214X_EP11_IN               23
#define LPC214X_EP12_OUT              24
#define LPC214X_EP12_IN               25
#define LPC214X_EP13_OUT              26
#define LPC214X_EP13_IN               27
#define LPC214X_EP14_OUT              28
#define LPC214X_EP14_IN               29
#define LPC214X_EP15_OUT              30
#define LPC214X_EP15_IN               31
#define LPC214X_NUMEPS                32

/* Commands ********************************************************************/

/* USB Command Code Register -- Command phase values */

#define CMD_USB_CMDWR                (0x00000500)
#define CMD_USB_DATAWR               (0x00000100)
#define CMD_USB_DATARD               (0x00000200)

/* Device Commands */

#define CMD_USB_DEV_SETADDRESS       (0x00d0)
#define CMD_USB_DEV_CONFIG           (0x00d8)
#define CMD_USB_DEV_SETMODE          (0x00f3)
#define CMD_USB_DEV_READFRAMENO      (0x00f5)
#define CMD_USB_DEV_READTESTREG      (0x00fd)
#define CMD_USB_DEV_SETSTATUS        (0x01fe)
#define CMD_USB_DEV_GETSTATUS        (0x00fe)
#define CMD_USB_DEV_GETERRORCODE     (0x00ff)
#define CMD_USB_DEV_READERRORSTATUS  (0x00fb)

/* Endpoint Commands */

#define CMD_USB_EP_SELECT            (0x0000)
#define CMD_USB_EP_SELECTCLEAR       (0x0040)
#define CMD_USB_EP_SETSTATUS         (0x0140)
#define CMD_USB_EP_CLRBUFFER         (0x00f2)
#define CMD_USB_EP_VALIDATEBUFFER    (0x00fa)

/* Command Data ****************************************************************/

/* DEV SETADDRESS command bits */

#define CMD_USB_SETADDRESS_DEVEN     (0x80)

/* Command Responses ***********************************************************/

/* Device Status Bits (8-bits) */

#define USBDEV_DEVSTATUS_CONNECT     (0x01)       /* Bit 0: Connected */
#define USBDEV_DEVSTATUS_CONNCHG     (0x02)       /* Bit 1: Connect change */
#define USBDEV_DEVSTATUS_SUSPEND     (0x04)       /* Bit 2: Suspend */
#define USBDEV_DEVSTATUS_SUSPCHG     (0x08)       /* Bit 3: Suspend change */
#define USBDEV_DEVSTATUS_RESET       (0x10)       /* Bit 4: Bus reset bit */

/* EP Select response */

#define CMD_USB_EPSELECT_FE          (0x01)       /* Bit 0=1: IN empty or OUT full */
#define CMD_USB_EPSELECT_ST          (0x02)       /* Bit 1=1: Endpoint is stalled */
#define CMD_USB_EPSELECT_STP         (0x04)       /* Bit 2=1: Last packet was setup */
#define CMD_USB_EPSELECT_PO          (0x05)       /* Bit 3=1: Previous packet was overwritten */
#define CMD_USB_EPSELECT_EPN         (0x10)       /* Bit 4=1: NAK sent */
#define CMD_USB_EPSELECT_B1FULL      (0x20)       /* Bit 5=1: Buffer 1 full */
#define CMD_USB_EPSELECT_B2FULL      (0x40)       /* Bit 6=1: Buffer 2 full */

/* EP CLRBUFFER response */

#define CMD_USB_CLRBUFFER_PO         (0x00000001)

/* DMA *************************************************************************/

/* The DMA descriptor */

#define USB_DMADESC_NEXTDDPTR        0 /* Offset 0: Next USB descriptor in RAM */
#define USB_DMADESC_CONFIG           1 /* Offset 1: DMA configuration info. */
#define USB_DMADESC_STARTADDR        2 /* Offset 2: DMA start address */
#define USB_DMADESC_STATUS           3 /* Offset 3: DMA status info (read only) */
#define USB_DMADESC_ISOCSIZEADDR     4 /* Offset 4: Isoc. packet size address */

/* Bit settings for offset 1 */

#define USB_DMADESC_MODENORMAL       (0x00000000) /* Bits 0-1=00: Mode normal */
#define USB_DMADESC_MODEATLE         (0x00000001) /* Bits 0-1=01: ATLE normal */
#define USB_DMADESC_NEXTDDVALID      (0x00000004) /* Bit 2=1: next descriptor valid */
#define USB_DMADESC_ISCOEP           (0x00000010) /* Bit 4=1: isoc endpoint */
#define USB_DMADESC_PKTSIZEMASK      (0x0000ffe0) /* Bits 5-15: packet size */
#define USB_DMADESC_PKTSIZESHIFT     (5)          /* Bits 5-15: packet size */
#define USB_DMADESC_BUFLENMASK       (0xffff0000) /* Bits 16-31: buffer length */
#define USB_DMADESC_BULENSHIFT       (16)         /* Bits 16-31: buffer length */

/* Bit settings for offset 3 (all must be initialized to zero) */

#define USB_DMADESC_STATUSMASK       (0x0000001e) /* Bits 1-4: DMA status */
#define USB_DMADESC_NOTSERVICED      (0x00000000)
#define USB_DMADESC_BEINGSERVICED    (0x00000002)
#define USB_DMADESC_NORMALCOMPLETION (0x00000004)
#define USB_DMADESC_DATAUNDERRUN     (0x00000006)
#define USB_DMADESC_DATAOVERRUN      (0x00000010)
#define USB_DMADESC_SYSTEMERROR      (0x00000012)
#define USB_DMADESC_PKTVALID         (0x00000020) /* Bit 5=1: Packet valid */
#define USB_DMADESC_LSBEXTRACTED     (0x00000040) /* Bit 6=1: LS byte extracted */
#define USB_DMADESC_MSBEXTRACTED     (0x00000080) /* Bit 7=1: MS byte extracted */
#define USB_DMADESC_MSGLENPOSMASK    (0x00003f00) /* Bits 8-13: Message length position */
#define USB_DMADESC_MSGLENPOSSHIFT   (8)          /* Bits 8-13: Message length position */
#define USB_DMADESC_DMACOUNTMASK     (0xffff0000) /* Bits 16-31: DMA count */
#define USB_DMADESC_DMACOUNTSHIFT    (16)         /* Bits 16-31:  DMA count */

/* DMA packet size format */

#define USB_DMAPKTSIZE_PKTLENMASK    (0x0000ffff) /* Bits 0-15: Packet length */
#define USB_DMAPKTSIZE_PKTLENSHIFT   (0)          /* Bits 0-15: Packet length */
#define USB_DMAPKTSIZE_PKTVALID      (0x00010000) /* Bit 16=1: Packet valid */
#define USB_DMAPKTSIZE_FRAMENOMASK   (0xfffe0000) /* Bit 17-31: Frame number */
#define USB_DMAPKTSIZE_FRAMENOSHIFT  (17)         /* Bit 17-31: Frame number */


/*******************************************************************************
 * Private Types
 *******************************************************************************/

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC214X_LPC214X_USBDEV_H */
