/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-usbotg.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_USBOTG_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_USBOTG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "pic32mx-memorymap.h"

/************************************************************************************
 * Pre-Processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define PIC32MX_USBOTG_IR_OFFSET   0x0040 /* USB OTG Interrupt Flags Register */
#define PIC32MX_USBOTG_IE_OFFSET   0x0050 /* USB OTG Interrupt Enable Register */
#define PIC32MX_USBOTG_STAT_OFFSET 0x0060 /* USB Comparator and Pin Status Register */
#define PIC32MX_USBOTG_CON_OFFSET  0x0070 /* USB Resistor and Pin Control Register */
#define PIC32MX_USB_PWRC_OFFSET    0x0080 /* USB Power Control Register */
#define PIC32MX_USB_IR_OFFSET      0x0200 /* USB Pending Interrupt Register */
#define PIC32MX_USB_IE_OFFSET      0x0210 /* USB Interrupt Enable Register */
#define PIC32MX_USB_EIR_OFFSET     0x0220 /* USB Pending Error Interrupt Register */
#define PIC32MX_USB_EIE_OFFSET     0x0230 /* USB Interrupt Error Enable Register */
#define PIC32MX_USB_STAT_OFFSET    0x0240 /* USB Status FIFO Register */
#define PIC32MX_USB_CON_OFFSET     0x0250 /* USB Module Control Register */
#define PIC32MX_USB_ADDR_OFFSET    0x0260 /* USB Address Register */
#define PIC32MX_USB_FRMH_OFFSET    0x0290 /* USB Frame Counter Register (high) */
#define PIC32MX_USB_FRML_OFFSET    0x0280 /* USB Frame Counter Register (low) */
#define PIC32MX_USB_TOK_OFFSET     0x02a0 /* USB Host Control Register */
#define PIC32MX_USB_SOF_OFFSET     0x02b0 /* USB SOF Counter Register */
#define PIC32MX_USB_BDTP1_OFFSET   0x0270 /* USB Buffer Descriptor Table Pointer Register 1 */
#define PIC32MX_USB_BDTP2_OFFSET   0x02c0 /* USB Buffer Descriptor Table Pointer Register 2 */
#define PIC32MX_USB_BDTP3_OFFSET   0x02d0 /* USB Buffer Descriptor Table Pointer Register 3 */
#define PIC32MX_USB_CNFG1_OFFSET   0x02e0 /* USB Debug and Idle Register */

#define PIC32MX_USB_EP_OFFSET(n)   (0x0300+((n)<<4))
#define PIC32MX_USB_EP0_OFFSET     0x0300 /* USB Endpoint 0 Control Register */
#define PIC32MX_USB_EP1_OFFSET     0x0310 /* USB Endpoint 1 Control Register */
#define PIC32MX_USB_EP2_OFFSET     0x0320 /* USB Endpoint 2 Control Register */
#define PIC32MX_USB_EP3_OFFSET     0x0330 /* USB Endpoint 3 Control Register */
#define PIC32MX_USB_EP4_OFFSET     0x0340 /* USB Endpoint 4 Control Register */
#define PIC32MX_USB_EP5_OFFSET     0x0350 /* USB Endpoint 5 Control Register */
#define PIC32MX_USB_EP6_OFFSET     0x0360 /* USB Endpoint 6 Control Register */
#define PIC32MX_USB_EP7_OFFSET     0x0370 /* USB Endpoint 7 Control Register */
#define PIC32MX_USB_EP8_OFFSET     0x0380 /* USB Endpoint 8 Control Register */
#define PIC32MX_USB_EP9_OFFSET     0x0390 /* USB Endpoint 9 Control Register */
#define PIC32MX_USB_EP10_OFFSET    0x03a0 /* USB Endpoint 10 Control Register */
#define PIC32MX_USB_EP11_OFFSET    0x03b0 /* USB Endpoint 11 Control Register */
#define PIC32MX_USB_EP12_OFFSET    0x03c0 /* USB Endpoint 12 Control Register */
#define PIC32MX_USB_EP13_OFFSET    0x03d0 /* USB Endpoint 13 Control Register */
#define PIC32MX_USB_EP14_OFFSET    0x03e0 /* USB Endpoint 14 Control Register */
#define PIC32MX_USB_EP15_OFFSET    0x03f0 /* USB Endpoint 15 Control Register */

/* Register Addresses ***************************************************************/

#define PIC32MX_USBOTG_IR          (PIC32MX_USB_K1BASE+PIC32MX_USBOTG_IR_OFFSET)
#define PIC32MX_USBOTG_IE          (PIC32MX_USB_K1BASE+PIC32MX_USBOTG_IE_OFFSET)
#define PIC32MX_USBOTG_STAT        (PIC32MX_USB_K1BASE+PIC32MX_USBOTG_STAT_OFFSET)
#define PIC32MX_USBOTG_CON         (PIC32MX_USB_K1BASE+PIC32MX_USBOTG_CON_OFFSET)
#define PIC32MX_USB_PWRC           (PIC32MX_USB_K1BASE+PIC32MX_USB_PWRC_OFFSET)
#define PIC32MX_USB_IR             (PIC32MX_USB_K1BASE+PIC32MX_USB_IR_OFFSET)
#define PIC32MX_USB_IE             (PIC32MX_USB_K1BASE+PIC32MX_USB_IE_OFFSET)
#define PIC32MX_USB_EIR            (PIC32MX_USB_K1BASE+PIC32MX_USB_EIR_OFFSET)
#define PIC32MX_USB_EIE            (PIC32MX_USB_K1BASE+PIC32MX_USB_EIE_OFFSET)
#define PIC32MX_USB_STAT           (PIC32MX_USB_K1BASE+PIC32MX_USB_STAT_OFFSET)
#define PIC32MX_USB_CON            (PIC32MX_USB_K1BASE+PIC32MX_USB_CON_OFFSET)
#define PIC32MX_USB_ADDR           (PIC32MX_USB_K1BASE+PIC32MX_USB_ADDR_OFFSET)
#define PIC32MX_USB_FRMH           (PIC32MX_USB_K1BASE+PIC32MX_USB_FRMH_OFFSET)
#define PIC32MX_USB_FRML           (PIC32MX_USB_K1BASE+PIC32MX_USB_FRML_OFFSET)
#define PIC32MX_USB_TOK            (PIC32MX_USB_K1BASE+PIC32MX_USB_TOK_OFFSET)
#define PIC32MX_USB_SOF            (PIC32MX_USB_K1BASE+PIC32MX_USB_SOF_OFFSET)
#define PIC32MX_USB_BDTP1          (PIC32MX_USB_K1BASE+PIC32MX_USB_BDTP1_OFFSET)
#define PIC32MX_USB_BDTP2          (PIC32MX_USB_K1BASE+PIC32MX_USB_BDTP2_OFFSET)
#define PIC32MX_USB_BDTP3          (PIC32MX_USB_K1BASE+PIC32MX_USB_BDTP3_OFFSET)
#define PIC32MX_USB_CNFG1          (PIC32MX_USB_K1BASE+PIC32MX_USB_CNFG1_OFFSET)

#define PIC32MX_USB_EP(n)          (PIC32MX_USB_K1BASE+PIC32MX_USB_EP_OFFSET(n))
#define PIC32MX_USB_EP0            (PIC32MX_USB_K1BASE+PIC32MX_USB_EP0_OFFSET)
#define PIC32MX_USB_EP1            (PIC32MX_USB_K1BASE+PIC32MX_USB_EP1_OFFSET)
#define PIC32MX_USB_EP2            (PIC32MX_USB_K1BASE+PIC32MX_USB_EP2_OFFSET)
#define PIC32MX_USB_EP3            (PIC32MX_USB_K1BASE+PIC32MX_USB_EP3_OFFSET)
#define PIC32MX_USB_EP4            (PIC32MX_USB_K1BASE+PIC32MX_USB_EP4_OFFSET)
#define PIC32MX_USB_EP5            (PIC32MX_USB_K1BASE+PIC32MX_USB_EP5_OFFSET)
#define PIC32MX_USB_EP6            (PIC32MX_USB_K1BASE+PIC32MX_USB_EP6_OFFSET)
#define PIC32MX_USB_EP7            (PIC32MX_USB_K1BASE+PIC32MX_USB_EP7_OFFSET)
#define PIC32MX_USB_EP8            (PIC32MX_USB_K1BASE+PIC32MX_USB_EP8_OFFSET)
#define PIC32MX_USB_EP9            (PIC32MX_USB_K1BASE+PIC32MX_USB_EP9_OFFSET)
#define PIC32MX_USB_EP10           (PIC32MX_USB_K1BASE+PIC32MX_USB_EP10_OFFSET)
#define PIC32MX_USB_EP11           (PIC32MX_USB_K1BASE+PIC32MX_USB_EP11_OFFSET)
#define PIC32MX_USB_EP12           (PIC32MX_USB_K1BASE+PIC32MX_USB_EP12_OFFSET)
#define PIC32MX_USB_EP13           (PIC32MX_USB_K1BASE+PIC32MX_USB_EP13_OFFSET)
#define PIC32MX_USB_EP14           (PIC32MX_USB_K1BASE+PIC32MX_USB_EP14_OFFSET)
#define PIC32MX_USB_EP15           (PIC32MX_USB_K1BASE+PIC32MX_USB_EP15_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* USB OTG Interrupt Flags Register */
/* USB OTG Interrupt Enable Register */

#define USBOTG_INT_VBUSVD          (1 << 0) /* Bit 0: A-Device VBUS Change Indicator */
#define USBOTG_INT_SESEND          (1 << 2) /* Bit 2: B-Device VBUS Change Indicator */
#define USBOTG_INT_ACTV            (1 << 4) /* Bit 4: Bus Activity Indicator */
#define USBOTG_INT_SESVD           (1 << 3) /* Bit 3: Session Valid Change Indicator */
#define USBOTG_INT_LSTATE          (1 << 5) /* Bit 5: Line State Stable Indicator */
#define USBOTG_INT_T1MSEC          (1 << 6) /* Bit 6: 1 Millisecond Timer */
#define USBOTG_INT_ID              (1 << 7) /* Bit 7: ID State Change Indicator */

/* USB Comparator and Pin Status Register */

#define USBOTG_STAT_VBUSVD         (1 << 0) /* Bit 0: A-VBUS Valid Indicator */
#define USBOTG_STAT_SESEND         (1 << 2) /* Bit 2: B-Session End Indicator */
#define USBOTG_STAT_SESVD          (1 << 3) /* Bit 3: Session Valid Indicator */
#define USBOTG_STAT_LSTATE         (1 << 5) /* Bit 5: Line State Stable Indicator */
#define USBOTG_STAT_ID             (1 << 7) /* Bit 7: ID Pin State Indicator */

/* USB Resistor and Pin Control Register */

#define USBOTG_CON_VBUSDIS         (1 << 0) /* Bit 0: VBUS Discharge Enable */
#define USBOTG_CON_VBUSCHG         (1 << 1) /* Bit 1: VBUS Charge Enable */
#define USBOTG_CON_OTGEN           (1 << 2) /* Bit 2: OTG Functionality Enable */
#define USBOTG_CON_VBUSON          (1 << 3) /* Bit 3: VBUS Power-on */
#define USBOTG_CON_DMPULDWN        (1 << 4) /* Bit 4: D- Pull-Down Enable */
#define USBOTG_CON_DPPULDWN        (1 << 5) /* Bit 5: D+ Pull-Down Enable */
#define USBOTG_CON_DMPULUP         (1 << 6) /* Bit 6: D- Pull-Up Enable */
#define USBOTG_CON_DPPULUP         (1 << 7) /* Bit 7: D+ Pull-Up Enable */

/* USB Power Control Register */

#define USB_PWRC_USBPWR            (1 << 0) /* Bit 0: USB Operation Enable */
#define USB_PWRC_USUSPEND          (1 << 1) /* Bit 1: USB Suspend Mode */
#if defined(CHIP_PIC32MX5) || defined(CHIP_PIC32MX6) || defined(CHIP_PIC32MX7)
#  define USB_PWRC_USBBUSY         (1 << 3) /* Bit 3: USB Module Busy */
#endif
#define USB_PWRC_USLPGRD           (1 << 4) /* Bit 4: USB Sleep Entry Guard */
#define USB_PWRC_UACTPND           (1 << 7) /* Bit 7: USB Activity Pending */

/* USB Pending Interrupt Register */
/* USB Interrupt Enable Register */

#define USB_INT_URST               (1 << 0) /* Bit 0: USB Reset Interrupt (Device mode) */
#define USB_INT_DETACH             (1 << 0) /* Bit 0: USB Detach Interrupt (Host mode) */
#define USB_INT_UERR               (1 << 1) /* Bit 1: USB Error Condition Interrupt */
#define USB_INT_SOF                (1 << 2) /* Bit 2: SOF Token Interrupt */
#define USB_INT_TRN                (1 << 3) /* Bit 3: Token Processing Complete Interrupt */
#define USB_INT_IDLE               (1 << 4) /* Bit 4: Idle Detect Interrupt */
#define USB_INT_RESUME             (1 << 5) /* Bit 5: Resume Interrupt */
#define USB_INT_ATTACH             (1 << 6) /* Bit 6: Peripheral Attach Interrupt */
#define USB_INT_STALL              (1 << 7) /* Bit 7: STALL Handshake Interrupt */

#define USB_INT_ALL                0xff

/* USB Pending Error Interrupt Register */
/* USB Interrupt Error Enable Register */

#define USB_EINT_PID               (1 << 0) /* Bit 0: PID Check Failure Flag */
#define USB_EINT_CRC5              (1 << 1) /* Bit 1: CRC5 Host Error Flag */
#define USB_EINT_EOF               (1 << 1) /* Bit 1: EOF Error Flag */
#define USB_EINT_CRC16             (1 << 2) /* Bit 2: CRC16 Failure Flag */
#define USB_EINT_DFN8              (1 << 3) /* Bit 3: Data Field Size Error Flag */
#define USB_EINT_BTO               (1 << 4) /* Bit 4: Bus Turnaround Time-Out Error Flag */
#define USB_EINT_DMA               (1 << 5) /* Bit 5: DMA Error Flag */
#define USB_EINT_BMX               (1 << 6) /* Bit 6: Bus Matrix Error Flag */
#define USB_EINT_BTS               (1 << 7) /* Bit 7: Bit Stuff Error Flag */

#define USB_EINT_ALL               0xff

/* USB Status FIFO Register */

#define USB_STAT_PPBI              (1 << 2) /* Bit 2: Ping-Pong BD Pointer Indicator */
#define USB_STAT_DIR               (1 << 3) /* Bit 3: Last BD Direction Indicator */
#define USB_STAT_ENDPT_SHIFT       (4)      /* Bits 4-7: Encoded Endpoint Activity */
#define USB_STAT_ENDPT_MASK        (15 << USB_STAT_ENDPT_SHIFT)
#  define USB_STAT_ENDPT(n)        ((n) << USB_STAT_ENDPT_SHIFT) /* Endpoint n, n=0..15 */

#define USB_STAT_PPBI_ODD          USB_STAT_PPBI /* The last transaction was to the ODD BD bank */
#define USB_STAT_PPBI_EVEN         0             /* The last transaction was to the EVEN BD bank */

#define USB_STAT_DIR_IN            USB_STAT_DIR  /* Last transaction was a transmit transfer (TX) */
#define USB_STAT_DIR_OUT           0             /* Last transaction was a receive transfer (RX) */

/* USB Module Control Register */

#define USB_CON_USBEN              (1 << 0) /* Bit 0: USB Module Enable */
#define USB_CON_SOFEN              (1 << 0) /* Bit 0: SOF Enable */
#define USB_CON_PPBRST             (1 << 1) /* Bit 1: Ping-Pong Buffers Reset */
#define USB_CON_RESUME             (1 << 2) /* Bit 2: RESUME Signaling Enable */
#define USB_CON_HOSTEN             (1 << 3) /* Bit 3: Host Mode Enable */
#define USB_CON_USBRST             (1 << 4) /* Bit 4: Module Reset */
#define USB_CON_PKTDIS             (1 << 5) /* Bit 5: Packet Transfer Disable */
#define USB_CON_TOKBUSY            (1 << 5) /* Bit 5: Token Busy Indicator */
#define USB_CON_SE0                (1 << 6) /* Bit 6: Live Single-Ended Zero flag */
#define USB_CON_JSTATE             (1 << 7) /* Bit 7: Live Differential Receiver JSTATE flag */

/* USB Address Register */

#define USB_ADDR_DEVADDR_SHIFT     (0)      /* Bit 0-6: 7-bit USB Device Address */
#define USB_ADDR_DEVADDR_MASK      (0x7f << USB_ADDR_DEVADDR_SHIFT)
#define USB_ADDR_LSPDEN            (1 << 7) /* Bit 7: Low Speed Enable Indicator */

/* USB Frame Counter Register (high) */

#define USB_FRMH_MASK              0x07     /* Bits 0-2: 11-bit frame number upper 3 bits */

/* USB Frame Counter Register (low) */

#define USB_FRML_MASK              0xff     /* Bits 0-7: 11-bit frame number lower 8 bits */

/* USB Host Control Register */

#define USB_TOK_PID_SHIFT          (4)      /* Bits 4-7:  Token Type Indicator */
#define USB_TOK_PID_MASK           (15 << USB_TOK_PID_SHIFT)
#  define USB_TOK_PID_OUT          (1 << USB_TOK_PID_SHIFT)  /* OUT (TX) token type transaction */
#  define USB_TOK_PID_IN           (9 << USB_TOK_PID_SHIFT)  /* IN (RX) token type transaction */
#  define USB_TOK_PID_SETUP        (13 << USB_TOK_PID_SHIFT) /* SETUP (TX) token type transaction */
#define USB_TOK_EP_SHIFT           (0)      /* Bits 0-3: Token Command Endpoint Address */
#define USB_TOK_EP_MASK            (15 << USB_TOK_EP_SHIFT)

/* USB SOF Counter Register */

#define USB_SOF_SHIFT              (0)      /* Bits 0-7: SOF Threshold Value */
#define USB_SOF_MASK               (0xff << USB_SOF_SHIFT)
#  define USB_SOF_8BTYE            (0x12 << USB_SOF_SHIFT) /* 8-byte packet */
#  define USB_SOF_16BTYE           (0x1a << USB_SOF_SHIFT) /* 16-byte packet */
#  define USB_SOF_32BTYE           (0x2a << USB_SOF_SHIFT) /* 32-byte packet */
#  define USB_SOF_64BTYE           (0x4a << USB_SOF_SHIFT) /* 64-byte packet */

/* USB Buffer Descriptor Table Pointer Register 1 */

#define USB_BDTP1_MASK             0x7e /* Bits 1-7:  BDT Base Address bits 9-15 */

/* USB Buffer Descriptor Table Pointer Register 2 */

#define USB_BDTP2_MASK             0xff /* Bits 0-7:  BDT Base Address bits 16-23 */

/* USB Buffer Descriptor Table Pointer Register 3 */

#define USB_BDTP3_MASK             0xff /* Bits 0-7:  BDT Base Address bits 24-31 */

/* USB Debug and Idle Register */

#if defined(CHIP_PIC32MX5) || defined(CHIP_PIC32MX6) || defined(CHIP_PIC32MX7)
#  define USB_CNFG1_UASUSPND       (1 << 0) /* Bit 0: Automatic Suspend Enable */
#endif
#define USB_CNFG1_USBSIDL          (1 << 4) /* Bit 4: Stop in Idle Mode */
#define USB_CNFG1_USBFRZ           (1 << 5) /* Bit 5: Freeze in Debug Mode */
#define USB_CNFG1_UOEMON           (1 << 6) /* Bit 6: USB OE Monitor Enable */
#define USB_CNFG1_UTEYE            (1 << 7) /* Bit 7: USB Eye-Pattern Test Enable */

/* USB Endpoint Control Register */

#define USB_EP_EPHSHK              (1 << 0) /* Bit 0: Endpoint Handshake Enable */
#define USB_EP_EPSTALL             (1 << 1) /* Bit 1: Endpoint Stall Status */
#define USB_EP_EPTXEN              (1 << 2) /* Bit 2: Endpoint Transmit Enable */
#define USB_EP_EPRXEN              (1 << 3) /* Bit 3: Endpoint Receive Enable */
#define USB_EP_EPCONDIS            (1 << 4) /* Bit 4: Bidirectional Endpoint Control */
#define USB_EP_RETRYDIS            (1 << 6) /* Bit 6: Retry Disable (Host mode and U1EP0 only) */
#define USB_EP_LSPD                (1 << 7) /* Bit 7: Low-Speed Direct Connection Enable */

/* Buffer Descriptor Table (BDT) ****************************************************/
/* Offset 0: On write (software->hardware) */

#define USB_BDT_STATUS_MASK        0xfc     /* Bits 2-7: Status bits */
#define USB_BDT_BSTALL             (1 << 2) /*   Bit 2: Buffer Stall Enable bit */
#define USB_BDT_DTS                (1 << 3) /*   Bit 3: Data Toggle Synchronization Enable bit */
#define USB_BDT_NINC               (1 << 4) /*   Bit 4: DMA Address Increment Disable bit */
#define USB_BDT_KEEP               (1 << 5) /*   Bit 5: BD Keep Enable bit */
#define USB_BDT_DATA01             (1 << 6) /*   Bit 6: Data Toggle Packet bit */
#define USB_BDT_UOWN               (1 << 7) /*   Bit 7: USB Own bit */
#define USB_BDT_BYTECOUNT_SHIFT    (16)     /* Bits 16-25: Byte Count bits */
#define USB_BDT_BYTECOUNT_MASK     (0x3ff << USB_BDT_BYTECOUNT_SHIFT)

#define USB_BDT_DATA0              0              /* DATA0 packet expected next */
#define USB_BDT_DATA1              USB_BDT_DATA01 /* DATA1 packet expected next */
#define USB_BDT_COWN               0              /* CPU owns the descriptor */

/* Offset 0: On read (hardware->software) */

#define USB_BDT_PID_SHIFT          (2)      /* Bits 2-5: Packet Identifier bits */
#define USB_BDT_PID_MASK           (15 << USB_BDT_PID_SHIFT)
                                            /* Bit 7: USB Own bit (same) */
                                            /* Bits 16-25: Byte Count bits (same) */

/* Offset 4: BUFFER_ADDRESS, 32-bit Buffer Address bits */

#define USB_BDT_BYTES_SIZE         8        /* Eight bytes per BDT */
#define USB_BDT_WORD_SIZE          2        /* Two 32-bit words per BDT */
#define USB_NBDTS_PER_EP           4        /* Number of BDTS per endpoint: IN/OUT and EVEN/ODD */

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/* Buffer Descriptor Status Register layout. */

struct usbotg_bdtentry_s
{
  uint32_t status;  /* Status, byte count, and PID */
  uint8_t *addr;    /* Buffer address */
};

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_USBOTG_H */
