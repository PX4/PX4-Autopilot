/************************************************************************************
 * dm320/dm320_uart.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_DM320_DM320_USB_H
#define __ARCH_ARM_SRC_DM320_DM320_USB_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* USB Controller Registers *********************************************************/

#define DM320_USB_FADDR       (DM320_USBOTG_VADDR+0x0000) /* Peripheral Address */
#define DM320_USB_POWER       (DM320_USBOTG_VADDR+0x0001) /* Power Control */
#define DM320_USB_INTRTX1     (DM320_USBOTG_VADDR+0x0002) /* Transmit EP Interrupt Status #1 */
#define DM320_USB_RSV1        (DM320_USBOTG_VADDR+0x0003) /* Reserved */
#define DM320_USB_INTRRX1     (DM320_USBOTG_VADDR+0x0004) /* Receive EP Interrupt Status #1 */
#define DM320_USB_RSV2        (DM320_USBOTG_VADDR+0x0005) /* Reserved */
#define DM320_USB_INTRUSB     (DM320_USBOTG_VADDR+0x0006) /* USB Interrupt Status */
#define DM320_USB_INTRTX1E    (DM320_USBOTG_VADDR+0x0007) /* Transmit EP Interrupt Enable #1 */
#define DM320_USB_RSV3        (DM320_USBOTG_VADDR+0x0008) /* Reserved */
#define DM320_USB_INTRRX1E    (DM320_USBOTG_VADDR+0x0009) /* Receive EP Interrupt Enable #1 */
#define DM320_USB_RSV4        (DM320_USBOTG_VADDR+0x000a) /* Reserved */
#define DM320_USB_INTRUSBE    (DM320_USBOTG_VADDR+0x000b) /* USB Interrupt Enable */
#define DM320_USB_FRAME1      (DM320_USBOTG_VADDR+0x000c) /* Lower Frame Number */
#define DM320_USB_FRAME2      (DM320_USBOTG_VADDR+0x000d) /* Upper Frame Number */
#define DM320_USB_INDEX       (DM320_USBOTG_VADDR+0x000e) /* Endpoint Index */
#define DM320_USB_DEVCTL      (DM320_USBOTG_VADDR+0x000f) /* Device Control */
#define DM320_USB_TXMAXP      (DM320_USBOTG_VADDR+0x0010) /* Transmit Maximum Packet Size */
#define DM320_USB_PERCSR0     (DM320_USBOTG_VADDR+0x0011) /* Peripheral EP0 Control */
#define DM320_USB_PERTXCSR1   (DM320_USBOTG_VADDR+0x0011) /* Peripheral Tx EP Control #1 */
#define DM320_USB_CSR2        (DM320_USBOTG_VADDR+0x0012) /* EP0 Control #2 */
#define DM320_USB_TXCSR2      (DM320_USBOTG_VADDR+0x0012) /* Tx EP Control #2 */
#define DM320_USB_RXMAXP      (DM320_USBOTG_VADDR+0x0013) /* Receive Maximum Packet Size */
#define DM320_USB_PERRXCSR1   (DM320_USBOTG_VADDR+0x0014) /* Peripheral Rx EP Control #1 */
#define DM320_USB_PERRXCSR2   (DM320_USBOTG_VADDR+0x0015) /* Peripheral Rx EP Control #2 */
#define DM320_USB_COUNT0      (DM320_USBOTG_VADDR+0x0016) /* Count EP0 Data Bytes */
#define DM320_USB_RXCOUNT1    (DM320_USBOTG_VADDR+0x0016) /* Count EP Data bytes #1 */
#define DM320_USB_RXCOUNT2    (DM320_USBOTG_VADDR+0x0017) /* Count EP Data bytes #2 */
#define DM320_USB_TXTYPE      (DM320_USBOTG_VADDR+0x0018) /* EP Transmit Type */
#define DM320_USB_NAKLMT0     (DM320_USBOTG_VADDR+0x0019) /* EP0 NAK Limit */
#define DM320_USB_TXINTVL     (DM320_USBOTG_VADDR+0x0019) /* EP Transmit Interval */
#define DM320_USB_RXTYPE      (DM320_USBOTG_VADDR+0x001a) /* EP Receive Type */
#define DM320_USB_RXINTVL     (DM320_USBOTG_VADDR+0x001b) /* EP Receive Interval */
#define DM320_USB_TXFIFO1     (DM320_USBOTG_VADDR+0x001c) /* EP Transmit FIFO Address #1 */
#define DM320_USB_TXFIFO2     (DM320_USBOTG_VADDR+0x001d) /* EP Transmit FIFO Address #2 */
#define DM320_USB_RXFIFO1     (DM320_USBOTG_VADDR+0x001e) /* EP Receive FIFO Address #1 */
#define DM320_USB_RXFIFO2     (DM320_USBOTG_VADDR+0x001f) /* EP Receive FIFO Address #2 */
#define DM320_USB_HST_CSR0    (DM320_USBOTG_VADDR+0x0011) /* Host EP0 Control */
#define DM320_USB_HSTTXCSR    (DM320_USBOTG_VADDR+0x0011) /* Host Tx EP Control #1 */
#define DM320_USB_HSTRXCSR1   (DM320_USBOTG_VADDR+0x0014) /* Host Rx EP Control #1 */
#define DM320_USB_HSTRXCSR2   (DM320_USBOTG_VADDR+0x0015) /* Host Rx EP Control #2 */
#define DM320_USB_FIFO0       (DM320_USBOTG_VADDR+0x0020) /* EP0 FIFO Access */
#define DM320_USB_FIFO1       (DM320_USBOTG_VADDR+0x0024) /* EP1 FIFO Access */
#define DM320_USB_FIFO2       (DM320_USBOTG_VADDR+0x0028) /* EP2 FIFO Access */
#define DM320_USB_FIFO3       (DM320_USBOTG_VADDR+0x002c) /* EP3 FIFO Access */
#define DM320_USB_FIFO4       (DM320_USBOTG_VADDR+0x0030) /* EP4 FIFO Access */
#define DM320_USB_DMAINTR     (DM320_USBOTG_VADDR+0x0200) /* Interrupt Status */
#define DM320_USB_DMACNTL1    (DM320_USBOTG_VADDR+0x0204) /* DMA Channel1 Control */
#define DM320_USB_DMAADDR1    (DM320_USBOTG_VADDR+0x0208) /* DMA Channel1 Address */
#define DM320_USB_DMACOUNT1   (DM320_USBOTG_VADDR+0x020c) /* DMA Channel1 Byte Count */
#define DM320_USB_DMACNTL2    (DM320_USBOTG_VADDR+0x0214) /* DMA Channel2 Control */
#define DM320_USB_DMAADDR2    (DM320_USBOTG_VADDR+0x0218) /* DMA Channel2 Address */
#define DM320_USB_DMACOUNT2   (DM320_USBOTG_VADDR+0x021c) /* DMA Channel2 Byte Count */
#define DM320_USB_DMACNTL3    (DM320_USBOTG_VADDR+0x0224) /* DMA Channel3 Control */
#define DM320_USB_DMAADDR3    (DM320_USBOTG_VADDR+0x0228) /* DMA Channel3 Address */
#define DM320_USB_DMACOUNT3   (DM320_USBOTG_VADDR+0x022c) /* DMA Channel3 Byte Count */
#define DM320_USB_DMACNTL4    (DM320_USBOTG_VADDR+0x0234) /* DMA Channel4 Control */
#define DM320_USB_DMAADDR4    (DM320_USBOTG_VADDR+0x0238) /* DMA Channel4 Address */
#define DM320_USB_DMACOUNT4   (DM320_USBOTG_VADDR+0x023c) /* DMA Channel4 Byte Count */

/* POWER register bit settings ******************************************************/

#define USB_POWER_ENSUS       (0x00000001)
#define USB_POWER_SUSPEND     (0x00000002)
#define USB_POWER_RESUME      (0x00000004)
#define USB_POWER_RESET       (0x00000008)
#define USB_POWER_VBUSLO      (0x00000010)
#define USB_POWER_VBUSSESS    (0x00000020)
#define USB_POWER_VBUSVAL     (0x00000040)
#define USB_POWER_ISO         (0x00000080)

/* USB interrupt bits **************************************************************/

#define USB_INT_NOINTERRUPT   (0x00000000)
#define USB_INT_SUSPEND       (0x00000001)
#define USB_INT_RESUME        (0x00000002)
#define USB_INT_RESET         (0x00000004)
#define USB_INT_SOF           (0x00000008)
#define USB_INT_CONNECTED     (0x00000010)
#define USB_INT_DISCONNECTED  (0x00000020)
#define USB_INT_SESSRQ        (0x00000040)
#define USB_INT_VBUSERR       (0x00000080)
#define USB_INT_RXFIFO        (0x00000f00)
#define USB_INT_RXFIFO1       (0x00000100)
#define USB_INT_RXFIFO2       (0x00000200)
#define USB_INT_RXFIFO3       (0x00000400)
#define USB_INT_RXFIFO4       (0x00000800)
#define USB_INT_CONTROL       (0x00001000)
#define USB_INT_TXFIFO        (0x0001e000)
#define USB_INT_TXFIFO1       (0x00002000)
#define USB_INT_TXFIFO2       (0x00004000)
#define USB_INT_TXFIFO3       (0x00008000)
#define USB_INT_TXFIFO4       (0x00010000)

#define USB_EP4_TX            (0x10)
#define USB_EP3_TX            (0x08)
#define USB_EP2_TX            (0x04)
#define USB_EP1_TX            (0x02)
#define USB_EP0               (0x01)

#define USB_EP4_RX            (0x10)
#define USB_EP3_RX            (0x08)
#define USB_EP2_RX            (0x04)
#define USB_EP1_RX            (0x02)

/* Endpoint control register index *************************************************/

#define USB_EP0_SELECT        (0x00)

/* DEVCTL register bit settings ****************************************************/

#define USB_DEVCTL_CID        (0x80)
#define USB_DEVCTL_FSDEV      (0x40)
#define USB_DEVCTL_LSDEV      (0x20)
#define USB_DEVCTL_PUCON      (0x10)
#define USB_DEVCTL_PDCON      (0x08)
#define USB_DEVCTL_MODE       (0x04)
#define USB_DEVCTL_HOSTREQ    (0x02)
#define USB_DEVCTL_SESSREQ    (0x01)

/* PERCSR0 register bit settings ***************************************************/

#define USB_PERCSR0_CLRSETEND  (0x80)
#define USB_PERCSR0_CLRRXRDY   (0x40)
#define USB_PERCSR0_SENDST     (0x20)
#define USB_PERCSR0_SETEND     (0x10)
#define USB_PERCSR0_DATAEND    (0x08)
#define USB_PERCSR0_SENTST     (0x04)
#define USB_PERCSR0_TXPKTRDY   (0x02)
#define USB_PERCSR0_RXPKTRDY   (0x01)

/* TXCSR1 register bit settings ****************************************************/

#define USB_TXCSR1_CLRDATTOG   (0x40)
#define USB_TXCSR1_SENTST      (0x20)
#define USB_TXCSR1_SENDST      (0x10)
#define USB_TXCSR1_FLFIFO      (0x08)
#define USB_TXCSR1_UNDERRUN    (0x04)
#define USB_TXCSR1_FIFOEMP     (0x02)
#define USB_TXCSR1_TXPKTRDY    (0x01)

/* CSR2 register bit settings ******************************************************/

#define USB_CSR2_FLFIFO        (0x01)

/* TXCSR2 register bit settings ****************************************************/

#define USB_TXCSR2_AUTOSET     (0x80)
#define USB_TXCSR2_ISO         (0x40)
#define USB_TXCSR2_MODE_TX     (0x20)
#define USB_TXCSR2_DMAEN       (0x10)
#define USB_TXCSR2_FRDATTOG    (0x08)
#define USB_TXCSR2_DMAMODE1    (0x04)

/* PERRXCSR1 register bit settings *************************************************/

#define USB_PERRXCSR1_CLRDATTOG (0x80)
#define USB_PERRXCSR1_SENTST   (0x40)
#define USB_PERRXCSR1_SENDST   (0x20)
#define USB_PERRXCSR1_FLFIFO   (0x10)
#define USB_PERRXCSR1_DATERR   (0x08)
#define USB_PERRXCSR1_OVERRUN  (0x04)
#define USB_PERRXCSR1_FIFOFUL  (0x02)
#define USB_PERRXCSR1_RXPKTRDY (0x01)

/* PERRXCSR2 register bit settings *************************************************/

#define USB_PERPXCSR2_AUTOCLR  (0x80)
#define USB_PERPXCSR2_ISO      (0x40)
#define USB_PERPXCSR2_DMAEN    (0x20)
#define USB_PERPXCSR2_DMAMODE1 (0x10)

/* TXFIFO2 register bit settings **************************************************/

#define USB_TXFIF02_SZMASK     (0xe0)
#define USB_TXFIFO2_SZ_8       (0x00)
#define USB_TXFIFO2_SZ_16      (0x20)
#define USB_TXFIFO2_SZ_32      (0x40)
#define USB_TXFIFO2_SZ_64      (0x60)
#define USB_TXFIFO2_SZ_128     (0x80)
#define USB_TXFIFO2_SZ_256     (0xa0)
#define USB_TXFIFO2_SZ_512     (0xc0)
#define USB_TXFIFO2_SZ_1024    (0xe0)
#define USB_TXFIFO2_SINGLE_BUF (0x00)
#define USB_TXFIFO2_DOUBLE_BUF (0x10)

/* RXFIFO2 register bit settings **************************************************/

#define USB_RXFIF02_DPB        (0x10)

/* USBDMA control register bit settings ********************************************/

#define USBDMA_CNTL_DMAEN      (0x01)
#define USBDMA_CNTL_DIR_IN     (0x02)
#define USBDMA_CNTL_DMAMODE1   (0x04)
#define USBDMA_CNTL_INTREN     (0x08)

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_SRC_DM320_DM320_USB_H */
