/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_usbb.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_USBB_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_USBB_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

/* USB Device Registers */

#define AVR32_USBB_UDCON_OFFSET       0x0000 /* Device General Control Register */
#define AVR32_USBB_UDINT_OFFSET       0x0004 /* Device Global Interrupt Register */
#define AVR32_USBB_UDINTCLR_OFFSET    0x0008 /* Device Global Interrupt Clear Register */
#define AVR32_USBB_UDINTSET_OFFSET    0x000c /* Device Global Interrupt Set Register */
#define AVR32_USBB_UDINTE_OFFSET      0x0010 /* Device Global Interrupt Enable Register */
#define AVR32_USBB_UDINTECLR_OFFSET   0x0014 /* Device Global Interrupt Enable Clear Register */
#define AVR32_USBB_UDINTESET_OFFSET   0x0018 /* Device Global Interrupt Enable Set Register */
#define AVR32_USBB_UERST_OFFSET       0x001c /* Endpoint Enable/Reset Register */
#define AVR32_USBB_UDFNUM_OFFSET      0x0020 /* Device Frame Number Register */

#define AVR32_USBB_UECFG_OFFSET(n)    (0x0100+((n)<<2))
#define AVR32_USBB_UECFG0_OFFSET      0x0100 /* Endpoint 0 Configuration Register */
#define AVR32_USBB_UECFG1_OFFSET      0x0104 /* Endpoint 1 Configuration Register */
#define AVR32_USBB_UECFG2_OFFSET      0x0108 /* Endpoint 2 Configuration Register */
#define AVR32_USBB_UECFG3_OFFSET      0x010c /* Endpoint 3 Configuration Register */
#define AVR32_USBB_UECFG4_OFFSET      0x0110 /* Endpoint 4 Configuration Register */
#define AVR32_USBB_UECFG5_OFFSET      0x0114 /* Endpoint 5 Configuration Register */
#define AVR32_USBB_UECFG6_OFFSET      0x0118 /* Endpoint 6 Configuration Register */

#define AVR32_USBB_UESTA_OFFSET(n)    (0x0130+((n)<<2))
#define AVR32_USBB_UESTA0_OFFSET      0x0130 /* Endpoint 0 Status Register */
#define AVR32_USBB_UESTA1_OFFSET      0x0134 /* Endpoint 1 Status Register */
#define AVR32_USBB_UESTA2_OFFSET      0x0138 /* Endpoint 2 Status Register */
#define AVR32_USBB_UESTA3_OFFSET      0x013c /* Endpoint 3 Status Register */
#define AVR32_USBB_UESTA4_OFFSET      0x0140 /* Endpoint 4 Status Register */
#define AVR32_USBB_UESTA5_OFFSET      0x0144 /* Endpoint 5 Status Register */
#define AVR32_USBB_UESTA6_OFFSET      0x0148 /* Endpoint 6 Status Register */

#define AVR32_USBB_UESTACLR_OFFSET(n) (0x0160+((n)<<2))
#define AVR32_USBB_UESTA0CLR_OFFSET   0x0160 /* Endpoint 0 Status Clear Register */
#define AVR32_USBB_UESTA1CLR_OFFSET   0x0164 /* Endpoint 1 Status Clear Register */
#define AVR32_USBB_UESTA2CLR_OFFSET   0x0168 /* Endpoint 2 Status Clear Register */
#define AVR32_USBB_UESTA3CLR_OFFSET   0x016c /* Endpoint 3 Status Clear Register */
#define AVR32_USBB_UESTA4CLR_OFFSET   0x0170 /* Endpoint 4 Status Clear Register */
#define AVR32_USBB_UESTA5CLR_OFFSET   0x0174 /* Endpoint 5 Status Clear Register */
#define AVR32_USBB_UESTA6CLR_OFFSET   0x0178 /* Endpoint 6 Status Clear Register */

#define AVR32_USBB_UESTASET_OFFSET(n) (0x0190+((n)<<2))
#define AVR32_USBB_UESTA0SET_OFFSET   0x0190 /* Endpoint 0 Status Set Register */
#define AVR32_USBB_UESTA1SET_OFFSET   0x0194 /* Endpoint 1 Status Set Register */
#define AVR32_USBB_UESTA2SET_OFFSET   0x0198 /* Endpoint 2 Status Set Register */
#define AVR32_USBB_UESTA3SET_OFFSET   0x019c /* Endpoint 3 Status Set Register */
#define AVR32_USBB_UESTA4SET_OFFSET   0x01a0 /* Endpoint 4 Status Set Register */
#define AVR32_USBB_UESTA5SET_OFFSET   0x01a4 /* Endpoint 5 Status Set Register */
#define AVR32_USBB_UESTA6SET_OFFSET   0x01a8 /* Endpoint 6 Status Set Register */

#define AVR32_USBB_UECON_OFFSET(n)    (0x01c0+((n)<<2))
#define AVR32_USBB_UECON0_OFFSET      0x01c0 /* Endpoint 0 Control Register */
#define AVR32_USBB_UECON1_OFFSET      0x01c4 /* Endpoint 1 Control Register */
#define AVR32_USBB_UECON2_OFFSET      0x01c8 /* Endpoint 2 Control Register */
#define AVR32_USBB_UECON3_OFFSET      0x01cc /* Endpoint 3 Control Register */
#define AVR32_USBB_UECON4_OFFSET      0x01d0 /* Endpoint 4 Control Register */
#define AVR32_USBB_UECON5_OFFSET      0x01d4 /* Endpoint 5 Control Register */
#define AVR32_USBB_UECON6_OFFSET      0x01d8 /* Endpoint 7 Control Register */

#define AVR32_USBB_UECONSET_OFFSET(n) (0x01f0+((n)<<2))
#define AVR32_USBB_UECON0SET_OFFSET   0x01f0 /* Endpoint 0 Control Set Register */
#define AVR32_USBB_UECON1SET_OFFSET   0x01f4 /* Endpoint 1 Control Set Register */
#define AVR32_USBB_UECON2SET_OFFSET   0x01f8 /* Endpoint 2 Control Set Register */
#define AVR32_USBB_UECON3SET_OFFSET   0x01fc /* Endpoint 3 Control Set Register */
#define AVR32_USBB_UECON4SET_OFFSET   0x0200 /* Endpoint 4 Control Set Register */
#define AVR32_USBB_UECON5SET_OFFSET   0x0204 /* Endpoint 5 Control Set Register */
#define AVR32_USBB_UECON6SET_OFFSET   0x0208 /* Endpoint 6 Control Set Register */

#define AVR32_USBB_UECONCLR_OFFSET(n) (0x0220+((n)<<2))
#define AVR32_USBB_UECON0CLR_OFFSET   0x0220 /* Endpoint 0 Control Clear Register */
#define AVR32_USBB_UECON1CLR_OFFSET   0x0224 /* Endpoint 1 Control Clear Register */
#define AVR32_USBB_UECON2CLR_OFFSET   0x0228 /* Endpoint 2 Control Clear Register */
#define AVR32_USBB_UECON3CLR_OFFSET   0x022c /* Endpoint 3 Control Clear Register */
#define AVR32_USBB_UECON4CLR_OFFSET   0x0230 /* Endpoint 4 Control Clear Register */
#define AVR32_USBB_UECON5CLR_OFFSET   0x0234 /* Endpoint 5 Control Clear Register */
#define AVR32_USBB_UECON6CLR_OFFSET   0x0238 /* Endpoint 6 Control Clear Register */

#define AVR32_UDDMA_OFFSET(n)         (0x0300+((n)<<4))
#define AVR32_UDDMA_NEXTDESC_OFFSET   0x0000 /* Device DMA Channel Next Descriptor Address Register */
#define AVR32_UDDMA_ADDR_OFFSET       0x0004 /* Device DMA Channel HSB Address Register */
#define AVR32_UDDMA_CTRL_OFFSET       0x0008 /* Device DMA Channel Control Register */
#define AVR32_UDDMA_STATUS_OFFSET     0x000c /* Device DMA Channel Status Register */

#define AVR32_UDDMA1_NEXTDESC_OFFSET  0x0310 /* Device DMA Channel 1 Next Descriptor Address Register */
#define AVR32_UDDMA1_ADDR_OFFSET      0x0314 /* Device DMA Channel 1 HSB Address Register */
#define AVR32_UDDMA1_CTRL_OFFSET      0x0318 /* Device DMA Channel 1 Control Register */
#define AVR32_UDDMA1_STATUS_OFFSET    0x031c /* Device DMA Channel 1 Status Register */

#define AVR32_UDDMA1_NEXTDESC_OFFSET  0x0310 /* Device DMA Channel 1 Next Descriptor Address Register */
#define AVR32_UDDMA1_ADDR_OFFSET      0x0314 /* Device DMA Channel 1 HSB Address Register */
#define AVR32_UDDMA1_CTRL_OFFSET      0x0318 /* Device DMA Channel 1 Control Register */
#define AVR32_UDDMA1_STATUS_OFFSET    0x031c /* Device DMA Channel 1 Status Register */

#define AVR32_UDDMA2_NEXTDESC_OFFSET  0x0320 /* Device DMA Channel 2 Next Descriptor Address Register */
#define AVR32_UDDMA2_ADDR_OFFSET      0x0324 /* Device DMA Channel 2 HSB Address Register */
#define AVR32_UDDMA2_CTRL_OFFSET      0x0328 /* Device DMA Channel 2 Control Register */
#define AVR32_UDDMA2_STATUS_OFFSET    0x032c /* Device DMA Channel 2 Status Register */

#define AVR32_UDDMA3_NEXTDESC_OFFSET  0x0330 /* Device DMA Channel 3 Next Descriptor Address Register */
#define AVR32_UDDMA3_ADDR_OFFSET      0x0334 /* Device DMA Channel 3 HSB Address Register */
#define AVR32_UDDMA3_CTRL_OFFSET      0x0338 /* Device DMA Channel 3 Control Register */
#define AVR32_UDDMA3_STATUS_OFFSET    0x033c /* Device DMA Channel 3 Status Register */

#define AVR32_UDDMA4_NEXTDESC_OFFSET  0x0340 /* Device DMA Channel 4 Next Descriptor Address Register */
#define AVR32_UDDMA4_ADDR_OFFSET      0x0344 /* Device DMA Channel 4 HSB Address Register */
#define AVR32_UDDMA4_CTRL_OFFSET      0x0348 /* Device DMA Channel 4 Control Register */
#define AVR32_UDDMA4_STATUS_OFFSET    0x034c /* Device DMA Channel 4 Status Register */

#define AVR32_UDDMA5_NEXTDESC_OFFSET  0x0350 /* Device DMA Channel 5 Next Descriptor Address Register */
#define AVR32_UDDMA5_ADDR_OFFSET      0x0354 /* Device DMA Channel 5 HSB Address Register */
#define AVR32_UDDMA5_CTRL_OFFSET      0x0358 /* Device DMA Channel 5 Control Register */
#define AVR32_UDDMA5_STATUS_OFFSET    0x035c /* Device DMA Channel 5 Status Register */

#define AVR32_UDDMA6_NEXTDESC_OFFSET  0x0360 /* Device DMA Channel 6 Next Descriptor Address Register */
#define AVR32_UDDMA6_ADDR_OFFSET      0x0364 /* Device DMA Channel 6 HSB Address Register */
#define AVR32_UDDMA6_CTRL_OFFSET      0x0368 /* Device DMA Channel 6 Control Register */
#define AVR32_UDDMA6_STATUS_OFFSET    0x036c /* Device DMA Channel 6 Status Register */

/* USB Host Registers */

#define AVR32_USBB_UHCON_OFFSET       0x0400 /* Host General Control Register */
#define AVR32_USBB_UHINT_OFFSET       0x0404 /* Host Global Interrupt Register */
#define AVR32_USBB_UHINTCLR_OFFSET    0x0408 /* Host Global Interrupt Clear Register */
#define AVR32_USBB_UHINTSET_OFFSET    0x040c /* Host Global Interrupt Set Register */
#define AVR32_USBB_UHINTE_OFFSET      0x0410 /* Host Global Interrupt Enable Register */
#define AVR32_USBB_UHINTECLR_OFFSET   0x0414 /* Host Global Interrupt Enable Clear Register */
#define AVR32_USBB_UHINTESET_OFFSET   0x0418 /* Host Global Interrupt Enable Set Register */
#define AVR32_USBB_UPRST_OFFSET       0x041c /* Pipe Enable/Reset Register */
#define AVR32_USBB_UHFNUM_OFFSET      0x0420 /* Host Frame Number Register */
#define AVR32_USBB_UHADDR1_OFFSET     0x0424 /* Host Address 1 Register */
#define AVR32_USBB_UHADDR2_OFFSET     0x0428 /* Host Address 2 Register */

#define AVR32_USBB_UPCFG_OFFSET(n)    (0x0500+((n)<<2))
#define AVR32_USBB_UPCFG0_OFFSET      0x0500 /* Pipe 0 Configuration Register */
#define AVR32_USBB_UPCFG1_OFFSET      0x0504 /* Pipe 1 Configuration Register */
#define AVR32_USBB_UPCFG2_OFFSET      0x0508 /* Pipe 2 Configuration Register */
#define AVR32_USBB_UPCFG3_OFFSET      0x050c /* Pipe 3 Configuration Register */
#define AVR32_USBB_UPCFG4_OFFSET      0x0510 /* Pipe 4 Configuration Register */
#define AVR32_USBB_UPCFG5_OFFSET      0x0514 /* Pipe 5 Configuration Register */
#define AVR32_USBB_UPCFG6_OFFSET      0x0518 /* Pipe 6 Configuration Register */

#define AVR32_USBB_UPSTA_OFFSET(n)    (0x0530+((n)<<2))
#define AVR32_USBB_UPSTA0_OFFSET      0x0530 /* Pipe 0 Status Register */
#define AVR32_USBB_UPSTA1_OFFSET      0x0534 /* Pipe 1 Status Register */
#define AVR32_USBB_UPSTA2_OFFSET      0x0538 /* Pipe 2 Status Register */
#define AVR32_USBB_UPSTA3_OFFSET      0x053c /* Pipe 3 Status Register */
#define AVR32_USBB_UPSTA4_OFFSET      0x0540 /* Pipe 4 Status Register */
#define AVR32_USBB_UPSTA5_OFFSET      0x0544 /* Pipe 5 Status Register */
#define AVR32_USBB_UPSTA6_OFFSET      0x0548 /* Pipe 6 Status Register */

#define AVR32_USBB_UPSTACLR_OFFSET(n) (0x0560+((n)<<2))
#define AVR32_USBB_UPSTA0CLR_OFFSET   0x0560 /* Pipe 0 Status Clear Register */
#define AVR32_USBB_UPSTA1CLR_OFFSET   0x0564 /* Pipe 1 Status Clear Register */
#define AVR32_USBB_UPSTA2CLR_OFFSET   0x0568 /* Pipe 2 Status Clear Register */
#define AVR32_USBB_UPSTA3CLR_OFFSET   0x056c /* Pipe 3 Status Clear Register */
#define AVR32_USBB_UPSTA4CLR_OFFSET   0x0570 /* Pipe 4 Status Clear Register */
#define AVR32_USBB_UPSTA5CLR_OFFSET   0x0574 /* Pipe 5 Status Clear Register */
#define AVR32_USBB_UPSTA6CLR_OFFSET   0x0578 /* Pipe 6 Status Clear Register */

#define AVR32_USBB_UPSTASET_OFFSET(n) (0x0590+((n)<<2))
#define AVR32_USBB_UPSTA0SET_OFFSET   0x0590 /* Pipe 0 Status Set Register */
#define AVR32_USBB_UPSTA1SET_OFFSET   0x0594 /* Pipe 1 Status Set Register */
#define AVR32_USBB_UPSTA2SET_OFFSET   0x0598 /* Pipe 2 Status Set Register */
#define AVR32_USBB_UPSTA3SET_OFFSET   0x059c /* Pipe 3 Status Set Register */
#define AVR32_USBB_UPSTA4SET_OFFSET   0x05a0 /* Pipe 4 Status Set Register */
#define AVR32_USBB_UPSTA5SET_OFFSET   0x05a4 /* Pipe 5 Status Set Register */
#define AVR32_USBB_UPSTA6SET_OFFSET   0x05a8 /* Pipe 6 Status Set Register */

#define AVR32_USBB_UPCON_OFFSET(n)    (0x05c0+((n)<<2))
#define AVR32_USBB_UPCON0_OFFSET      0x05c0 /* Pipe 0 Control Register */
#define AVR32_USBB_UPCON1_OFFSET      0x05c4 /* Pipe 1 Control Register */
#define AVR32_USBB_UPCON2_OFFSET      0x05c8 /* Pipe 2 Control Register */
#define AVR32_USBB_UPCON3_OFFSET      0x05cc /* Pipe 3 Control Register */
#define AVR32_USBB_UPCON4_OFFSET      0x05d0 /* Pipe 4 Control Register */
#define AVR32_USBB_UPCON5_OFFSET      0x05d4 /* Pipe 5 Control Register */
#define AVR32_USBB_UPCON6_OFFSET      0x05d8 /* Pipe 6 Control Register */

#define AVR32_USBB_UPCONSET_OFFSET(n) (0x05f0+((n)<<2))
#define AVR32_USBB_UPCON0SET_OFFSET   0x05f0 /* Pipe 0 Control Set Register */
#define AVR32_USBB_UPCON1SET_OFFSET   0x05f4 /* Pipe 1 Control Set Register */
#define AVR32_USBB_UPCON2SET_OFFSET   0x05f8 /* Pipe 2 Control Set Register */
#define AVR32_USBB_UPCON3SET_OFFSET   0x05fc /* Pipe 3 Control Set Register */
#define AVR32_USBB_UPCON4SET_OFFSET   0x0600 /* Pipe 4 Control Set Register */
#define AVR32_USBB_UPCON5SET_OFFSET   0x0604 /* Pipe 5 Control Set Register */
#define AVR32_USBB_UPCON6SET_OFFSET   0x0608 /* Pipe 6 Control Set Register */

#define AVR32_USBB_UPCONCLR_OFFSET(n) (0x0620+((n)<<2))
#define AVR32_USBB_UPCON0CLR_OFFSET   0x0620 /* Pipe 0 Control Clear Register */
#define AVR32_USBB_UPCON1CLR_OFFSET   0x0624 /* Pipe 1 Control Clear Register */
#define AVR32_USBB_UPCON2CLR_OFFSET   0x0628 /* Pipe 2 Control Clear Register */
#define AVR32_USBB_UPCON3CLR_OFFSET   0x062c /* Pipe 3 Control Clear Register */
#define AVR32_USBB_UPCON4CLR_OFFSET   0x0630 /* Pipe 4 Control Clear Register */
#define AVR32_USBB_UPCON5CLR_OFFSET   0x0634 /* Pipe 5 Control Clear Register */
#define AVR32_USBB_UPCON6CLR_OFFSET   0x0638 /* Pipe 6 Control Clear Register */

#define AVR32_USBB_UPINRQ_OFFSET(n)   (0x0650+((n)<<2))
#define AVR32_USBB_UPINRQ0_OFFSET     0x0650 /* Pipe 0 IN Request Register */
#define AVR32_USBB_UPINRQ1_OFFSET     0x0654 /* Pipe 1 IN Request Register */
#define AVR32_USBB_UPINRQ2_OFFSET     0x0658 /* Pipe 2 IN Request Register */
#define AVR32_USBB_UPINRQ3_OFFSET     0x065c /* Pipe 3 IN Request Register */
#define AVR32_USBB_UPINRQ4_OFFSET     0x0660 /* Pipe 4 IN Request Register */
#define AVR32_USBB_UPINRQ5_OFFSET     0x0664 /* Pipe 5 IN Request Register */
#define AVR32_USBB_UPINRQ6_OFFSET     0x0668 /* Pipe 6 IN Request Register */

#define AVR32_USBB_UPERR_OFFSET(n)    (0x0680+((n)<<2))
#define AVR32_USBB_UPERR0_OFFSET      0x0680 /* Pipe 0 Error Register */
#define AVR32_USBB_UPERR1_OFFSET      0x0684 /* Pipe 1 Error Register */
#define AVR32_USBB_UPERR2_OFFSET      0x0688 /* Pipe 2 Error Register */
#define AVR32_USBB_UPERR3_OFFSET      0x068c /* Pipe 3 Error Register */
#define AVR32_USBB_UPERR4_OFFSET      0x0690 /* Pipe 4 Error Register */
#define AVR32_USBB_UPERR5_OFFSET      0x0694 /* Pipe 5 Error Register */
#define AVR32_USBB_UPERR6_OFFSET      0x0698 /* Pipe 6 Error Register */

#define AVR32_UHDMA_OFFSET(n)         (0x0700+((n)<<4))
#define AVR32_UHDMA_NEXTDESC_OFFSET   0x0000 /* Host DMA Channel Next Descriptor Address Register */
#define AVR32_UHDMA_ADDR_OFFSET       0x0004 /* Host DMA Channel HSB Address Register */
#define AVR32_UHDMA_CTRL_OFFSET       0x0008 /* Host DMA Channel Control Register */
#define AVR32_UHDMA_STATUS_OFFSET     0x000c /* Host DMA Channel Status Register */

#define AVR32_UHDMA1_NEXTDESC_OFFSET  0x0710 /* Host DMA Channel 1 Next Descriptor Address Register */
#define AVR32_UHDMA1_ADDR_OFFSET      0x0714 /* Host DMA Channel 1 HSB Address Register */
#define AVR32_UHDMA1_CTRL_OFFSET      0x0718 /* Host DMA Channel 1 Control Register */
#define AVR32_UHDMA1_STATUS_OFFSET    0x071c /* Host DMA Channel 1 Status Register */

#define AVR32_UHDMA2_NEXTDESC_OFFSET  0x0720 /* Host DMA Channel 2 Next Descriptor Address Register */
#define AVR32_UHDMA2_ADDR_OFFSET      0x0724 /* Host DMA Channel 2 HSB Address Register */
#define AVR32_UHDMA2_CTRL_OFFSET      0x0728 /* Host DMA Channel 2 Control Register */
#define AVR32_UHDMA2_STATUS_OFFSET    0x072c /* Host DMA Channel 2 Status Register */

#define AVR32_UHDMA3_NEXTDESC_OFFSET  0x0730 /* Host DMA Channel 3 Next Descriptor Address Register */
#define AVR32_UHDMA3_ADDR_OFFSET      0x0734 /* Host DMA Channel 3 HSB Address Register */
#define AVR32_UHDMA3_CTRL_OFFSET      0x0738 /* Host DMA Channel 3 Control Register */
#define AVR32_UHDMA3_STATUS_OFFSET    0x073c /* Host DMA Channel 3 Status Register */

#define AVR32_UHDMA4_NEXTDESC_OFFSET  0x0740 /* Host DMA Channel 4 Next Descriptor Address Register */
#define AVR32_UHDMA4_ADDR_OFFSET      0x0744 /* Host DMA Channel 4 HSB Address Register */
#define AVR32_UHDMA4_CTRL_OFFSET      0x0748 /* Host DMA Channel 4 Control Register */
#define AVR32_UHDMA4_STATUS_OFFSET    0x074c /* Host DMA Channel 4 Status Register */

#define AVR32_UHDMA5_NEXTDESC_OFFSET  0x0750 /* Host DMA Channel 5 Next Descriptor Address Register */
#define AVR32_UHDMA5_ADDR_OFFSET      0x0754 /* Host DMA Channel 5 HSB Address Register */
#define AVR32_UHDMA5_CTRL_OFFSET      0x0758 /* Host DMA Channel 5 Control Register */
#define AVR32_UHDMA5_STATUS_OFFSET    0x075c /* Host DMA Channel 5 Status Register */

#define AVR32_UHDMA6_NEXTDESC_OFFSET  0x0760 /* Host DMA Channel 6 Next Descriptor Address Register */
#define AVR32_UHDMA6_ADDR_OFFSET      0x0764 /* Host DMA Channel 6 HSB Address Register */
#define AVR32_UHDMA6_CTRL_OFFSET      0x0768 /* Host DMA Channel 6 Control Register */
#define AVR32_UHDMA6_STATUS_OFFSET    0x076c /* Host DMA Channel 6 Status Register */

/* USB General Registers */

#define AVR32_USBB_USBCON_OFFSET      0x0800 /* General Control Register */
#define AVR32_USBB_USBSTA_OFFSET      0x0804 /* General Status Register */
#define AVR32_USBB_USBSTACLR_OFFSET   0x0808 /* General Status Clear Register */
#define AVR32_USBB_USBSTASET_OFFSET   0x080c /* General Status Set Register */
#define AVR32_USBB_UVERS_OFFSET       0x0818 /* IP Version Register */
#define AVR32_USBB_UFEATURES_OFFSET   0x081c /* IP Features Register */
#define AVR32_USBB_UADDRSIZE_OFFSET   0x0820 /* IP PB Address Size Register */
#define AVR32_USBB_UNAME1_OFFSET      0x0824 /* IP Name Register 1 */
#define AVR32_USBB_UNAME2_OFFSET      0x0828 /* IP Name Register 2 */
#define AVR32_USBB_USBFSM_OFFSET      0x082c /* USB Finite State Machine Status Register */

/* Register Addresses ***************************************************************/

/* USB Device Registers */

#define AVR32_USBB_UDCON              (AVR32_USB_BASE+AVR32_USBB_UDCON_OFFSET)
#define AVR32_USBB_UDINT              (AVR32_USB_BASE+AVR32_USBB_UDINT_OFFSET)
#define AVR32_USBB_UDINTCLR           (AVR32_USB_BASE+AVR32_USBB_UDINTCLR_OFFSET)
#define AVR32_USBB_UDINTSET           (AVR32_USB_BASE+AVR32_USBB_UDINTSET_OFFSET)
#define AVR32_USBB_UDINTE             (AVR32_USB_BASE+AVR32_USBB_UDINTE_OFFSET)
#define AVR32_USBB_UDINTECLR          (AVR32_USB_BASE+AVR32_USBB_UDINTECLR_OFFSET)
#define AVR32_USBB_UDINTESET          (AVR32_USB_BASE+AVR32_USBB_UDINTESET_OFFSET)
#define AVR32_USBB_UERST              (AVR32_USB_BASE+AVR32_USBB_UERST_OFFSET)
#define AVR32_USBB_UDFNUM             (AVR32_USB_BASE+AVR32_USBB_UDFNUM_OFFSET)

#define AVR32_USBB_UECFG(n)           (AVR32_USB_BASE+AVR32_USBB_UECFG_OFFSET(n))
#define AVR32_USBB_UECFG0             (AVR32_USB_BASE+AVR32_USBB_UECFG0_OFFSET)
#define AVR32_USBB_UECFG1             (AVR32_USB_BASE+AVR32_USBB_UECFG1_OFFSET)
#define AVR32_USBB_UECFG2             (AVR32_USB_BASE+AVR32_USBB_UECFG2_OFFSET)
#define AVR32_USBB_UECFG3             (AVR32_USB_BASE+AVR32_USBB_UECFG3_OFFSET)
#define AVR32_USBB_UECFG4             (AVR32_USB_BASE+AVR32_USBB_UECFG4_OFFSET)
#define AVR32_USBB_UECFG5             (AVR32_USB_BASE+AVR32_USBB_UECFG5_OFFSET)
#define AVR32_USBB_UECFG6             (AVR32_USB_BASE+AVR32_USBB_UECFG6_OFFSET)

#define AVR32_USBB_UESTA(n)           (AVR32_USB_BASE+AVR32_USBB_UESTA_OFFSET(n))
#define AVR32_USBB_UESTA0             (AVR32_USB_BASE+AVR32_USBB_UESTA0_OFFSET)
#define AVR32_USBB_UESTA1             (AVR32_USB_BASE+AVR32_USBB_UESTA1_OFFSET)
#define AVR32_USBB_UESTA2             (AVR32_USB_BASE+AVR32_USBB_UESTA2_OFFSET)
#define AVR32_USBB_UESTA3             (AVR32_USB_BASE+AVR32_USBB_UESTA3_OFFSET)
#define AVR32_USBB_UESTA4             (AVR32_USB_BASE+AVR32_USBB_UESTA4_OFFSET)
#define AVR32_USBB_UESTA5             (AVR32_USB_BASE+AVR32_USBB_UESTA5_OFFSET)
#define AVR32_USBB_UESTA6             (AVR32_USB_BASE+AVR32_USBB_UESTA6_OFFSET)

#define AVR32_USBB_UESTACLR(n)        (AVR32_USB_BASE+AVR32_USBB_UESTACLR_OFFSET(n))
#define AVR32_USBB_UESTA0CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA0CLR_OFFSET)
#define AVR32_USBB_UESTA1CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA1CLR_OFFSET)
#define AVR32_USBB_UESTA2CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA2CLR_OFFSET)
#define AVR32_USBB_UESTA3CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA3CLR_OFFSET)
#define AVR32_USBB_UESTA4CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA4CLR_OFFSET)
#define AVR32_USBB_UESTA5CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA5CLR_OFFSET)
#define AVR32_USBB_UESTA6CLR          (AVR32_USB_BASE+AVR32_USBB_UESTA6CLR_OFFSET)

#define AVR32_USBB_UESTASET(n)        (AVR32_USB_BASE+AVR32_USBB_UESTASET_OFFSET(n))
#define AVR32_USBB_UESTA0SET          (AVR32_USB_BASE+AVR32_USBB_UESTA0SET_OFFSET)
#define AVR32_USBB_UESTA1SET          (AVR32_USB_BASE+AVR32_USBB_UESTA1SET_OFFSET)
#define AVR32_USBB_UESTA2SET          (AVR32_USB_BASE+AVR32_USBB_UESTA2SET_OFFSET)
#define AVR32_USBB_UESTA3SET          (AVR32_USB_BASE+AVR32_USBB_UESTA3SET_OFFSET)
#define AVR32_USBB_UESTA4SET          (AVR32_USB_BASE+AVR32_USBB_UESTA4SET_OFFSET)
#define AVR32_USBB_UESTA5SET          (AVR32_USB_BASE+AVR32_USBB_UESTA5SET_OFFSET)
#define AVR32_USBB_UESTA6SET          (AVR32_USB_BASE+AVR32_USBB_UESTA6SET_OFFSET)

#define AVR32_USBB_UECON(n)           (AVR32_USB_BASE+AVR32_USBB_UECON_OFFSET(n))
#define AVR32_USBB_UECON0             (AVR32_USB_BASE+AVR32_USBB_UECON0_OFFSET)
#define AVR32_USBB_UECON1             (AVR32_USB_BASE+AVR32_USBB_UECON1_OFFSET)
#define AVR32_USBB_UECON2             (AVR32_USB_BASE+AVR32_USBB_UECON2_OFFSET)
#define AVR32_USBB_UECON3             (AVR32_USB_BASE+AVR32_USBB_UECON3_OFFSET)
#define AVR32_USBB_UECON4             (AVR32_USB_BASE+AVR32_USBB_UECON4_OFFSET)
#define AVR32_USBB_UECON5             (AVR32_USB_BASE+AVR32_USBB_UECON5_OFFSET)
#define AVR32_USBB_UECON6             (AVR32_USB_BASE+AVR32_USBB_UECON6_OFFSET)

#define AVR32_USBB_UECONSET(n)        (AVR32_USB_BASE+AVR32_USBB_UECONSET_OFFSET(n))
#define AVR32_USBB_UECON0SET          (AVR32_USB_BASE+AVR32_USBB_UECON0SET_OFFSET)
#define AVR32_USBB_UECON1SET          (AVR32_USB_BASE+AVR32_USBB_UECON1SET_OFFSET)
#define AVR32_USBB_UECON2SET          (AVR32_USB_BASE+AVR32_USBB_UECON2SET_OFFSET)
#define AVR32_USBB_UECON3SET          (AVR32_USB_BASE+AVR32_USBB_UECON3SET_OFFSET)
#define AVR32_USBB_UECON4SET          (AVR32_USB_BASE+AVR32_USBB_UECON4SET_OFFSET)
#define AVR32_USBB_UECON5SET          (AVR32_USB_BASE+AVR32_USBB_UECON5SET_OFFSET)
#define AVR32_USBB_UECON6SET          (AVR32_USB_BASE+AVR32_USBB_UECON6SET_OFFSET)

#define AVR32_USBB_UECONCLR(n)        (AVR32_USB_BASE+AVR32_USBB_UECONCLR_OFFSET(n))
#define AVR32_USBB_UECON0CLR          (AVR32_USB_BASE+AVR32_USBB_UECON0CLR_OFFSET)
#define AVR32_USBB_UECON1CLR          (AVR32_USB_BASE+AVR32_USBB_UECON1CLR_OFFSET)
#define AVR32_USBB_UECON2CLR          (AVR32_USB_BASE+AVR32_USBB_UECON2CLR_OFFSET)
#define AVR32_USBB_UECON3CLR          (AVR32_USB_BASE+AVR32_USBB_UECON3CLR_OFFSET)
#define AVR32_USBB_UECON4CLR          (AVR32_USB_BASE+AVR32_USBB_UECON4CLR_OFFSET)
#define AVR32_USBB_UECON5CLR          (AVR32_USB_BASE+AVR32_USBB_UECON5CLR_OFFSET)
#define AVR32_USBB_UECON6CLR          (AVR32_USB_BASE+AVR32_USBB_UECON6CLR_OFFSET)

#define AVR32_UDDMA_BASE(n)           (AVR32_USB_BASE+AVR32_UDDMA_OFFSET(n))
#define AVR32_UDDMA_NEXTDESC(n)       (AVR32_UDDMA_BASE(n)+AVR32_UDDMA_NEXTDESC_OFFSET)
#define AVR32_UDDMA_ADDR(n)           (AVR32_UDDMA_BASE(n)+AVR32_UDDMA_ADDR_OFFSET)
#define AVR32_UDDMA_CTRL(n)           (AVR32_UDDMA_BASE(n)+AVR32_UDDMA_CTRL_OFFSET)
#define AVR32_UDDMA_STATUS(n)         (AVR32_UDDMA_BASE(n)+AVR32_UDDMA_STATUS_OFFSET)

#define AVR32_UDDMA1_NEXTDESC         (AVR32_USB_BASE+AVR32_UDDMA1_NEXTDESC_OFFSET)
#define AVR32_UDDMA1_ADDR             (AVR32_USB_BASE+AVR32_UDDMA1_ADDR_OFFSET)
#define AVR32_UDDMA1_CTRL             (AVR32_USB_BASE+AVR32_UDDMA1_CTRL_OFFSET)
#define AVR32_UDDMA1_STATUS           (AVR32_USB_BASE+AVR32_UDDMA1_STATUS_OFFSET)

#define AVR32_UDDMA2_NEXTDESC         (AVR32_USB_BASE+AVR32_UDDMA2_NEXTDESC_OFFSET)
#define AVR32_UDDMA2_ADDR             (AVR32_USB_BASE+AVR32_UDDMA2_ADDR_OFFSET)
#define AVR32_UDDMA2_CTRL             (AVR32_USB_BASE+AVR32_UDDMA2_CTRL_OFFSET)
#define AVR32_UDDMA2_STATUS           (AVR32_USB_BASE+AVR32_UDDMA2_STATUS_OFFSET)

#define AVR32_UDDMA3_NEXTDESC         (AVR32_USB_BASE+AVR32_UDDMA3_NEXTDESC_OFFSET)
#define AVR32_UDDMA3_ADDR             (AVR32_USB_BASE+AVR32_UDDMA3_ADDR_OFFSET)
#define AVR32_UDDMA3_CTRL             (AVR32_USB_BASE+AVR32_UDDMA3_CTRL_OFFSET)
#define AVR32_UDDMA3_STATUS           (AVR32_USB_BASE+AVR32_UDDMA3_STATUS_OFFSET)

#define AVR32_UDDMA4_NEXTDESC         (AVR32_USB_BASE+AVR32_UDDMA4_NEXTDESC_OFFSET)
#define AVR32_UDDMA4_ADDR             (AVR32_USB_BASE+AVR32_UDDMA4_ADDR_OFFSET )
#define AVR32_UDDMA4_CTRL             (AVR32_USB_BASE+AVR32_UDDMA4_CTRL_OFFSET)
#define AVR32_UDDMA4_STATUS           (AVR32_USB_BASE+AVR32_UDDMA4_STATUS_OFFSET)

#define AVR32_UDDMA5_NEXTDESC         (AVR32_USB_BASE+AVR32_UDDMA5_NEXTDESC_OFFSET)
#define AVR32_UDDMA5_ADDR             (AVR32_USB_BASE+AVR32_UDDMA5_ADDR_OFFSET)
#define AVR32_UDDMA5_CTRL             (AVR32_USB_BASE+AVR32_UDDMA5_CTRL_OFFSET)
#define AVR32_UDDMA5_STATUS           (AVR32_USB_BASE+AVR32_UDDMA5_STATUS_OFFSET)

#define AVR32_UDDMA6_NEXTDESC         (AVR32_USB_BASE+AVR32_UDDMA6_NEXTDESC_OFFSET)
#define AVR32_UDDMA6_ADDR             (AVR32_USB_BASE+AVR32_UDDMA6_ADDR_OFFSET)
#define AVR32_UDDMA6_CTRL             (AVR32_USB_BASE+AVR32_UDDMA6_CTRL_OFFSET)
#define AVR32_UDDMA6_STATUS           (AVR32_USB_BASE+AVR32_UDDMA6_STATUS_OFFSET)

/* USB Host Registers */

#define AVR32_USBB_UHCON              (AVR32_USB_BASE+AVR32_USBB_UHCON_OFFSET)
#define AVR32_USBB_UHINT              (AVR32_USB_BASE+AVR32_USBB_UHINT_OFFSET)
#define AVR32_USBB_UHINTCLR           (AVR32_USB_BASE+AVR32_USBB_UHINTCLR_OFFSET)
#define AVR32_USBB_UHINTSET           (AVR32_USB_BASE+AVR32_USBB_UHINTSET_OFFSET)
#define AVR32_USBB_UHINTE             (AVR32_USB_BASE+AVR32_USBB_UHINTE_OFFSET)
#define AVR32_USBB_UHINTECLR          (AVR32_USB_BASE+AVR32_USBB_UHINTECLR_OFFSET)
#define AVR32_USBB_UHINTESET          (AVR32_USB_BASE+AVR32_USBB_UHINTESET_OFFSET)
#define AVR32_USBB_UPRST              (AVR32_USB_BASE+AVR32_USBB_UPRST_OFFSET)
#define AVR32_USBB_UHFNUM             (AVR32_USB_BASE+AVR32_USBB_UHFNUM_OFFSET)
#define AVR32_USBB_UHADDR1            (AVR32_USB_BASE+AVR32_USBB_UHADDR1_OFFSET)
#define AVR32_USBB_UHADDR2            (AVR32_USB_BASE+AVR32_USBB_UHADDR2_OFFSET)

#define AVR32_USBB_UPCFG(n)           (AVR32_USB_BASE+AVR32_USBB_UPCFG_OFFSET(n))
#define AVR32_USBB_UPCFG0             (AVR32_USB_BASE+AVR32_USBB_UPCFG0_OFFSET)
#define AVR32_USBB_UPCFG1             (AVR32_USB_BASE+AVR32_USBB_UPCFG1_OFFSET)
#define AVR32_USBB_UPCFG2             (AVR32_USB_BASE+AVR32_USBB_UPCFG2_OFFSET)
#define AVR32_USBB_UPCFG3             (AVR32_USB_BASE+AVR32_USBB_UPCFG3_OFFSET)
#define AVR32_USBB_UPCFG4             (AVR32_USB_BASE+AVR32_USBB_UPCFG4_OFFSET)
#define AVR32_USBB_UPCFG5             (AVR32_USB_BASE+AVR32_USBB_UPCFG5_OFFSET)
#define AVR32_USBB_UPCFG6             (AVR32_USB_BASE+AVR32_USBB_UPCFG6_OFFSET)

#define AVR32_USBB_UPSTA(n)           (AVR32_USB_BASE+AVR32_USBB_UPSTA_OFFSET(n))
#define AVR32_USBB_UPSTA0             (AVR32_USB_BASE+AVR32_USBB_UPSTA0_OFFSET)
#define AVR32_USBB_UPSTA1             (AVR32_USB_BASE+AVR32_USBB_UPSTA1_OFFSET)
#define AVR32_USBB_UPSTA2             (AVR32_USB_BASE+AVR32_USBB_UPSTA2_OFFSET)
#define AVR32_USBB_UPSTA3             (AVR32_USB_BASE+AVR32_USBB_UPSTA3_OFFSET)
#define AVR32_USBB_UPSTA4             (AVR32_USB_BASE+AVR32_USBB_UPSTA4_OFFSET)
#define AVR32_USBB_UPSTA5             (AVR32_USB_BASE+AVR32_USBB_UPSTA5_OFFSET)
#define AVR32_USBB_UPSTA6             (AVR32_USB_BASE+AVR32_USBB_UPSTA6_OFFSET)

#define AVR32_USBB_UPSTACLR(n)        (AVR32_USB_BASE+AVR32_USBB_UPSTACLR_OFFSET(n))
#define AVR32_USBB_UPSTA0CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA0CLR_OFFSET)
#define AVR32_USBB_UPSTA1CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA1CLR_OFFSET)
#define AVR32_USBB_UPSTA2CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA2CLR_OFFSET)
#define AVR32_USBB_UPSTA3CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA3CLR_OFFSET)
#define AVR32_USBB_UPSTA4CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA4CLR_OFFSET)
#define AVR32_USBB_UPSTA5CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA5CLR_OFFSET)
#define AVR32_USBB_UPSTA6CLR          (AVR32_USB_BASE+AVR32_USBB_UPSTA6CLR_OFFSET)

#define AVR32_USBB_UPSTASET(n)        (AVR32_USB_BASE+AVR32_USBB_UPSTASET_OFFSET(n))
#define AVR32_USBB_UPSTA0SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA0SET_OFFSET)
#define AVR32_USBB_UPSTA1SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA1SET_OFFSET)
#define AVR32_USBB_UPSTA2SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA2SET_OFFSET)
#define AVR32_USBB_UPSTA3SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA3SET_OFFSET)
#define AVR32_USBB_UPSTA4SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA4SET_OFFSET)
#define AVR32_USBB_UPSTA5SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA5SET_OFFSET)
#define AVR32_USBB_UPSTA6SET          (AVR32_USB_BASE+AVR32_USBB_UPSTA6SET_OFFSET)

#define AVR32_USBB_UPCON(n)           (AVR32_USB_BASE+AVR32_USBB_UPCON_OFFSET(n))
#define AVR32_USBB_UPCON0             (AVR32_USB_BASE+AVR32_USBB_UPCON0_OFFSET)
#define AVR32_USBB_UPCON1             (AVR32_USB_BASE+AVR32_USBB_UPCON1_OFFSET)
#define AVR32_USBB_UPCON2             (AVR32_USB_BASE+AVR32_USBB_UPCON2_OFFSET)
#define AVR32_USBB_UPCON3             (AVR32_USB_BASE+AVR32_USBB_UPCON3_OFFSET)
#define AVR32_USBB_UPCON4             (AVR32_USB_BASE+AVR32_USBB_UPCON4_OFFSET)
#define AVR32_USBB_UPCON5             (AVR32_USB_BASE+AVR32_USBB_UPCON5_OFFSET)
#define AVR32_USBB_UPCON6             (AVR32_USB_BASE+AVR32_USBB_UPCON6_OFFSET)

#define AVR32_USBB_UPCONSET(n)        (AVR32_USB_BASE+AVR32_USBB_UPCONSET_OFFSET(n))
#define AVR32_USBB_UPCON0SET          (AVR32_USB_BASE+AVR32_USBB_UPCON0SET_OFFSET)
#define AVR32_USBB_UPCON1SET          (AVR32_USB_BASE+AVR32_USBB_UPCON1SET_OFFSET)
#define AVR32_USBB_UPCON2SET          (AVR32_USB_BASE+AVR32_USBB_UPCON2SET_OFFSET)
#define AVR32_USBB_UPCON3SET          (AVR32_USB_BASE+AVR32_USBB_UPCON3SET_OFFSET)
#define AVR32_USBB_UPCON4SET          (AVR32_USB_BASE+AVR32_USBB_UPCON4SET_OFFSET)
#define AVR32_USBB_UPCON5SET          (AVR32_USB_BASE+AVR32_USBB_UPCON5SET_OFFSET)
#define AVR32_USBB_UPCON6SET          (AVR32_USB_BASE+AVR32_USBB_UPCON6SET_OFFSET)

#define AVR32_USBB_UPCONCLR(n)        (AVR32_USB_BASE+AVR32_USBB_UPCONCLR_OFFSET(n))
#define AVR32_USBB_UPCON0CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON0CLR_OFFSET)
#define AVR32_USBB_UPCON1CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON1CLR_OFFSET)
#define AVR32_USBB_UPCON2CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON2CLR_OFFSET)
#define AVR32_USBB_UPCON3CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON3CLR_OFFSET)
#define AVR32_USBB_UPCON4CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON4CLR_OFFSET)
#define AVR32_USBB_UPCON5CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON5CLR_OFFSET)
#define AVR32_USBB_UPCON6CLR          (AVR32_USB_BASE+AVR32_USBB_UPCON6CLR_OFFSET)

#define AVR32_USBB_UPINRQ(n)          (AVR32_USB_BASE+AVR32_USBB_UPINRQ_OFFSET(n))
#define AVR32_USBB_UPINRQ0            (AVR32_USB_BASE+AVR32_USBB_UPINRQ0_OFFSET)
#define AVR32_USBB_UPINRQ1            (AVR32_USB_BASE+AVR32_USBB_UPINRQ1_OFFSET)
#define AVR32_USBB_UPINRQ2            (AVR32_USB_BASE+AVR32_USBB_UPINRQ2_OFFSET)
#define AVR32_USBB_UPINRQ3            (AVR32_USB_BASE+AVR32_USBB_UPINRQ3_OFFSET)
#define AVR32_USBB_UPINRQ4            (AVR32_USB_BASE+AVR32_USBB_UPINRQ4_OFFSET)
#define AVR32_USBB_UPINRQ5            (AVR32_USB_BASE+AVR32_USBB_UPINRQ5_OFFSET)
#define AVR32_USBB_UPINRQ6            (AVR32_USB_BASE+AVR32_USBB_UPINRQ6_OFFSET)

#define AVR32_USBB_UPERR(n)           (AVR32_USB_BASE+AVR32_USBB_UPERR_OFFSET(n))
#define AVR32_USBB_UPERR0             (AVR32_USB_BASE+AVR32_USBB_UPERR0_OFFSET)
#define AVR32_USBB_UPERR1             (AVR32_USB_BASE+AVR32_USBB_UPERR1_OFFSET)
#define AVR32_USBB_UPERR2             (AVR32_USB_BASE+AVR32_USBB_UPERR2_OFFSET)
#define AVR32_USBB_UPERR3             (AVR32_USB_BASE+AVR32_USBB_UPERR3_OFFSET)
#define AVR32_USBB_UPERR4             (AVR32_USB_BASE+AVR32_USBB_UPERR4_OFFSET)
#define AVR32_USBB_UPERR5             (AVR32_USB_BASE+AVR32_USBB_UPERR5_OFFSET)
#define AVR32_USBB_UPERR6             (AVR32_USB_BASE+AVR32_USBB_UPERR6_OFFSET)

#define AVR32_UHDMA_BASE(n)           (AVR32_USB_BASE+AVR32_UHDMA_OFFSET(n))
#define AVR32_UHDMA_NEXTDESC(n)       (AVR32_UHDMA_BASE(n)+AVR32_UHDMA_NEXTDESC_OFFSET)
#define AVR32_UHDMA_ADDR(n)           (AVR32_UHDMA_BASE(n)+AVR32_UHDMA_ADDR_OFFSET)
#define AVR32_UHDMA_CTRL(n)           (AVR32_UHDMA_BASE(n)+AVR32_UHDMA_CTRL_OFFSET)
#define AVR32_UHDMA_STATUS(n)         (AVR32_UHDMA_BASE(n)+AVR32_UHDMA_STATUS_OFFSET)

#define AVR32_UHDMA1_NEXTDESC         (AVR32_USB_BASE+AVR32_UHDMA1_NEXTDESC_OFFSET)
#define AVR32_UHDMA1_ADDR             (AVR32_USB_BASE+AVR32_UHDMA1_ADDR_OFFSET)
#define AVR32_UHDMA1_CTRL             (AVR32_USB_BASE+AVR32_UHDMA1_CTRL_OFFSET)
#define AVR32_UHDMA1_STATUS           (AVR32_USB_BASE+AVR32_UHDMA1_STATUS_OFFSET)

#define AVR32_UHDMA2_NEXTDESC         (AVR32_USB_BASE+AVR32_UHDMA2_NEXTDESC_OFFSET)
#define AVR32_UHDMA2_ADDR             (AVR32_USB_BASE+AVR32_UHDMA2_ADDR_OFFSET)
#define AVR32_UHDMA2_CTRL             (AVR32_USB_BASE+AVR32_UHDMA2_CTRL_OFFSET)
#define AVR32_UHDMA2_STATUS           (AVR32_USB_BASE+AVR32_UHDMA2_STATUS_OFFSET)

#define AVR32_UHDMA3_NEXTDESC         (AVR32_USB_BASE+AVR32_UHDMA3_NEXTDESC_OFFSET)
#define AVR32_UHDMA3_ADDR             (AVR32_USB_BASE+AVR32_UHDMA3_ADDR_OFFSET)
#define AVR32_UHDMA3_CTRL             (AVR32_USB_BASE+AVR32_UHDMA3_CTRL_OFFSET)
#define AVR32_UHDMA3_STATUS           (AVR32_USB_BASE+AVR32_UHDMA3_STATUS_OFFSET)

#define AVR32_UHDMA4_NEXTDESC         (AVR32_USB_BASE+AVR32_UHDMA4_NEXTDESC_OFFSET)
#define AVR32_UHDMA4_ADDR             (AVR32_USB_BASE+AVR32_UHDMA4_ADDR_OFFSET)
#define AVR32_UHDMA4_CTRL             (AVR32_USB_BASE+AVR32_UHDMA4_CTRL_OFFSET)
#define AVR32_UHDMA4_STATUS           (AVR32_USB_BASE+AVR32_UHDMA4_STATUS_OFFSET)

#define AVR32_UHDMA5_NEXTDESC         (AVR32_USB_BASE+AVR32_UHDMA5_NEXTDESC_OFFSET)
#define AVR32_UHDMA5_ADDR             (AVR32_USB_BASE+AVR32_UHDMA5_ADDR_OFFSET)
#define AVR32_UHDMA5_CTRL             (AVR32_USB_BASE+AVR32_UHDMA5_CTRL_OFFSET)
#define AVR32_UHDMA5_STATUS           (AVR32_USB_BASE+AVR32_UHDMA5_STATUS_OFFSET)

#define AVR32_UHDMA6_NEXTDESC         (AVR32_USB_BASE+AVR32_UHDMA6_NEXTDESC_OFFSET)
#define AVR32_UHDMA6_ADDR             (AVR32_USB_BASE+AVR32_UHDMA6_ADDR_OFFSET)
#define AVR32_UHDMA6_CTRL             (AVR32_USB_BASE+AVR32_UHDMA6_CTRL_OFFSET)
#define AVR32_UHDMA6_STATUS           (AVR32_USB_BASE+AVR32_UHDMA6_STATUS_OFFSET)

/* USB General Registers */

#define AVR32_USBB_USBCON             (AVR32_USB_BASE+AVR32_USBB_USBCON_OFFSET)
#define AVR32_USBB_USBSTA             (AVR32_USB_BASE+AVR32_USBB_USBSTA_OFFSET)
#define AVR32_USBB_USBSTACLR          (AVR32_USB_BASE+AVR32_USBB_USBSTACLR_OFFSET)
#define AVR32_USBB_USBSTASET          (AVR32_USB_BASE+AVR32_USBB_USBSTASET_OFFSET)
#define AVR32_USBB_UVERS              (AVR32_USB_BASE+AVR32_USBB_UVERS_OFFSET)
#define AVR32_USBB_UFEATURES          (AVR32_USB_BASE+AVR32_USBB_UFEATURES_OFFSET)
#define AVR32_USBB_UADDRSIZE          (AVR32_USB_BASE+AVR32_USBB_UADDRSIZE_OFFSET)
#define AVR32_USBB_UNAME1             (AVR32_USB_BASE+AVR32_USBB_UNAME1_OFFSET)
#define AVR32_USBB_UNAME2             (AVR32_USB_BASE+AVR32_USBB_UNAME2_OFFSET)
#define AVR32_USBB_USBFSM             (AVR32_USB_BASE+AVR32_USBB_USBFSM_OFFSET)

/* Register Bit-field Definitions ***************************************************/
/* USB Device Registers Bit-field Definitions ***************************************/
/* Device General Control Register Bit-field Definitions */

#define USBB_UDCON_UADD_SHIFT         (0)       /* Bits 0-6: USB Address */
#define USBB_UDCON_UADD_MASK          (0x7f << USBB_UDCON_UADD_SHIFT)
#define USBB_UDCON_ADDEN:             (1 << 7)  /* Bit 7:  Address Enable */
#define USBB_UDCON_DETACH             (1 << 8)  /* Bit 8:  Detach */
#define USBB_UDCON_RMWKUP             (1 << 9)  /* Bit 9:  Remote Wake-Up */
#define USBB_UDCON_LS                 (1 << 12) /* Bit 12: Low-Speed Mode Force */

/* Device Global Interrupt Register Bit-field Definitions */
/* Device Global Interrupt Clear Register Bit-field Definitions */
/* Device Global Interrupt Set Register Bit-field Definitions */
/* Device Global Interrupt Enable Register Bit-field Definitions */
/* Device Global Interrupt Enable Clear Register Bit-field Definitions */
/* Device Global Interrupt Enable Set Register Bit-field Definitions */

#define USBB_UDINT_SUSP               (1 << 0)  /* Bit 0:  Suspend Interrupt */
#define USBB_UDINT_SOF                (1 << 2)  /* Bit 2:  Start of Frame Interrupt */
#define USBB_UDINT_EORST              (1 << 3)  /* Bit 3:  End of Reset Interrupt */
#define USBB_UDINT_WAKEUP             (1 << 4)  /* Bit 4:  Wake-Up Interrupt */
#define USBB_UDINT_EORSM              (1 << 5)  /* Bit 5:  End of Resume Interrupt */
#define USBB_UDINT_UPRSM              (1 << 6)  /* Bit 6:  Upstream Resume Interrupt */
#define USBB_UDINT_EPINT(n)           (1 << ((n)+12))  /* Endpoint n Interrupt */
#define USBB_UDINT_EP0INT             (1 << 12) /* Bit 12: Endpoint n Interrupt */
#define USBB_UDINT_EP1INT             (1 << 13) /* Bit 13: Endpoint n Interrupt */
#define USBB_UDINT_EP2INT             (1 << 14) /* Bit 14: Endpoint n Interrupt */
#define USBB_UDINT_EP3INT             (1 << 15) /* Bit 15: Endpoint n Interrupt */
#define USBB_UDINT_EP4INT             (1 << 16) /* Bit 16: Endpoint n Interrupt */
#define USBB_UDINT_EP5INT             (1 << 17) /* Bit 17: Endpoint n Interrupt */
#define USBB_UDINT_EP6INT             (1 << 18) /* Bit 18: Endpoint n Interrupt */
#define USBB_UDINT_DMAINT(n)          (1 << ((n)+24)) /* DMA Channel n Interrupt */
#define USBB_UDINT_DMA1INT            (1 << 25) /* Bit 25: DMA Channel n Interrupt */
#define USBB_UDINT_DMA2INT            (1 << 26) /* Bit 26: DMA Channel 1 Interrupt */
#define USBB_UDINT_DMA3INT            (1 << 27) /* Bit 27: DMA Channel 2 Interrupt */
#define USBB_UDINT_DMA4INT            (1 << 28) /* Bit 28: DMA Channel 3 Interrupt */
#define USBB_UDINT_DMA5INT            (1 << 29) /* Bit 29: DMA Channel 4 Interrupt */
#define USBB_UDINT_DMA6INT            (1 << 30) /* Bit 30: DMA Channel 5 Interrupt */

/* Endpoint Enable/Reset Register Bit-field Definitions */

#define USBB_UERST_EPRST(n)           (1 << ((n)+16))  /* Endpoint n Reset */
#define USBB_UERST_EPRST0             (1 << 16) /* Bit 16: Endpoint 0 Reset */
#define USBB_UERST_EPRST1             (1 << 17) /* Bit 17: Endpoint 1 Reset */
#define USBB_UERST_EPRST2             (1 << 18) /* Bit 18: Endpoint 2 Reset */
#define USBB_UERST_EPRST3             (1 << 19) /* Bit 19: Endpoint 3 Reset */
#define USBB_UERST_EPRST4             (1 << 20) /* Bit 20: Endpoint 4 Reset */
#define USBB_UERST_EPRST5             (1 << 21) /* Bit 21: Endpoint 5 Reset */
#define USBB_UERST_EPRST6             (1 << 22) /* Bit 22: Endpoint 6 Reset */
#define USBB_UERST_EPEN(n)            (1 << n)  /* Endpoint n Enable */
#define USBB_UERST_EPEN0              (1 << 0)  /* Bit 0:  Endpoint 0 Enable */
#define USBB_UERST_EPEN1              (1 << 1)  /* Bit 1:  Endpoint 1 Enable */
#define USBB_UERST_EPEN2              (1 << 2)  /* Bit 2:  Endpoint 2 Enable */
#define USBB_UERST_EPEN3              (1 << 3)  /* Bit 3:  Endpoint 3 Enable */
#define USBB_UERST_EPEN4              (1 << 4)  /* Bit 4:  Endpoint 4 Enable */
#define USBB_UERST_EPEN5              (1 << 5)  /* Bit 5:  Endpoint 5 Enable */
#define USBB_UERST_EPEN6              (1 << 6)  /* Bit 5:  Endpoint 6 Enable */

/* Device Frame Number Register Bit-field Definitions */

#define USBB_UDFNUM_FNUM_SHIFT        (3)       /* Bits 3-13: Frame Number */
#define USBB_UDFNUM_FNUM_MASK         (0x7ff << USBB_UDFNUM_FNUM_SHIFT)
#define USBB_UDFNUM_FNCERR            (1 << 15) /* Bit 15: Frame Number CRC Error */

/* Endpoint Configuration Register Bit-field Definitions */

#define USBB_UECFG_ALLOC              (1 << 1)  /* Bit 1:  Endpoint Memory Allocate */
#define USBB_UECFG_EPBK_SHIFT         (2)       /* Bits 2-3: Endpoint Banks */
#define USBB_UECFG_EPBK_MASK          (3 << USBB_UECFG_EPBK_SHIFT)
#  define USBB_UECFG_EPBK_1           (0 << USBB_UECFG_EPBK_SHIFT) /* 1 (single-bank endpoint) */
#  define USBB_UECFG_EPBK_2           (1 << USBB_UECFG_EPBK_SHIFT) /* 2 (double-bank endpoint) */
#  define USBB_UECFG_EPBK_3           (2 << USBB_UECFG_EPBK_SHIFT) /* 3 (triple-bank endpoint) */
#define USBB_UECFG_EPSIZE_SHIFT       (4)       /* Bits 4-6: Endpoint Size */
#define USBB_UECFG_EPSIZE_MASK        (7 << USBB_UECFG_EPSIZE_SHIFT)
#  define USBB_UECFG_EPSIZE_8         (0 << USBB_UECFG_EPSIZE_SHIFT) /* 8 bytes */
#  define USBB_UECFG_EPSIZE_16        (1 << USBB_UECFG_EPSIZE_SHIFT) /* 16 bytes */
#  define USBB_UECFG_EPSIZE_32        (2 << USBB_UECFG_EPSIZE_SHIFT) /* 32 bytes */
#  define USBB_UECFG_EPSIZE_64        (3 << USBB_UECFG_EPSIZE_SHIFT) /* 64 bytes */
#  define USBB_UECFG_EPSIZE_128       (4 << USBB_UECFG_EPSIZE_SHIFT) /* 128 bytes */
#  define USBB_UECFG_EPSIZE_256       (5 << USBB_UECFG_EPSIZE_SHIFT) /* 256 bytes */
#  define USBB_UECFG_EPSIZE_512       (6 << USBB_UECFG_EPSIZE_SHIFT) /* 512 bytes */
#  define USBB_UECFG_EPSIZE_1024      (7 << USBB_UECFG_EPSIZE_SHIFT) /* 1024 bytes */
#define USBB_UECFG_EPDIR              (1 << 8)  /* Bit 8:  Endpoint Direction */
#define USBB_UECFG_AUTOSW             (1 << 9)  /* Bit 9:  Automatic Switch */
#define USBB_UECFG_EPTYPE_SHIFT       (11)      /* Bits 11-12: Endpoint Type */
#define USBB_UECFG_EPTYPE_MASK        (3 << USBB_UECFG_EPTYPE_SHIFT)
#  define USBB_UECFG_EPTYPE_CTRL      (0 << USBB_UECFG_EPTYPE_SHIFT) /* Control */
#  define USBB_UECFG_EPTYPE_ISOC      (1 << USBB_UECFG_EPTYPE_SHIFT) /* Isochronous */
#  define USBB_UECFG_EPTYPE_BULK      (2 << USBB_UECFG_EPTYPE_SHIFT) /* Bulk */
#  define USBB_UECFG_EPTYPE_INTR      (3 << USBB_UECFG_EPTYPE_SHIFT) /* Interrupt */

/* Endpoint Status Register Bit-field Definitions (common fields) */
/* Endpoint Status Clear Register Bit-field Definitions */
/* Endpoint Status Set Register Bit-field Definitions (common fields) */

#define USBB_UESTA_TXINI              (1 << 0)  /* Bit 0:  Transmitted IN Data Interrupt */
#define USBB_UESTA_RXOUTI             (1 << 1)  /* Bit 1:  Received OUT Data Interrupt */
#define USBB_UESTA_UNDERFI            (1 << 2)  /* Bit 2:  Underflow Interrupt */
#define USBB_UESTA_RXSTPI             (1 << 2)  /* Bit 2:  Received SETUP Interrupt */
#define USBB_UESTA_NAKOUTI            (1 << 3)  /* Bit 3:  NAKed OUT Interrupt */
#define USBB_UESTA_NAKINI             (1 << 4)  /* Bit 4:  NAKed IN Interrupt */
#define USBB_UESTA_OVERFI             (1 << 5)  /* Bit 5:  Overflow Interrupt */
#define USBB_UESTA_STALLEDI           (1 << 6)  /* Bit 6:  STALLed Interrupt */
#define USBB_UESTA_CRCERRI            (1 << 6)  /* Bit 6:  CRC Error Interrupt */
#define USBB_UESTA_SHORTPACKET        (1 << 7)  /* Bit 7:  Short Packet Interrupt */

/* Endpoint Status Register Bit-field Definitions (only in UESTA) */

#define USBB_UESTA_DTSEQ_SHIFT        (8)       /* Bits 8-9: Data Toggle Sequence */
#define USBB_UESTA_DTSEQ_MASK         (3 << USBB_UESTA_DTSEQ_SHIFT)
#  define USBB_UESTA_DTSEQ_DATA0      (0 << USBB_UESTA_DTSEQ_SHIFT) /* Data0 */
#  define USBB_UESTA_DTSEQ_DATA1      (1 << USBB_UESTA_DTSEQ_SHIFT) /* Data1 */
#define USBB_UESTA_NBUSYBK_SHIFT      (12)      /* Bits 12-13: Number of Busy Banks */
#define USBB_UESTA_NBUSYBK_MASK       (3 << USBB_UESTA_NBUSYBK_SHIFT)
#  define USBB_UESTA_NBUSYBK_NONE     (0 << USBB_UESTA_NBUSYBK_SHIFT) /* 0 (all banks free) */
#  define USBB_UESTA_NBUSYBK_1BANK    (1 << USBB_UESTA_NBUSYBK_SHIFT) /* 1 */
#  define USBB_UESTA_NBUSYBK_2BANKS   (2 << USBB_UESTA_NBUSYBK_SHIFT) /* 2 */
#  define USBB_UESTA_NBUSYBK_3BANKS   (3 << USBB_UESTA_NBUSYBK_SHIFT) /* 3 */
#define USBB_UESTA_CURRBK_SHIFT       (14)      /* Bits 14-15: Current Bank */
#define USBB_UESTA_CURRBK_MASK        (3 << USBB_UESTA_CURRBK_SHIFT)
#  define USBB_UESTA_CURRBK_BANK0     (0 << USBB_UESTA_CURRBK_SHIFT) /* Bank0 */
#  define USBB_UESTA_CURRBK_BANK1     (1 << USBB_UESTA_CURRBK_SHIFT) /* Bank1 */
#  define USBB_UESTA_CURRBK_BANK2     (2 << USBB_UESTA_CURRBK_SHIFT) /* Bank2 */
#define USBB_UESTA_RWALL              (1 << 16) /* Bit 16: Read/Write Allowed */
#define USBB_UESTA_CTRLDIR            (1 << 17) /* Bit 17: Control Direction */
#define USBB_UESTA_CFGOK              (1 << 18) /* Bit 18: Configuration OK Status */
#define USBB_UESTA_BYCT_SHIFT         (20)       /* Bits 20-30: Byte Count */
#define USBB_UESTA_BYCT_MASK          (0x7ff << USBB_UESTA_BYCT_SHIFT)

/* Endpoint Status Set Register Bit-field Definitions (only in UESTASET) */

#define USBB_UESTASET_NBUSYBKS        (1 << 12)  /* Bit 12 */

/* Endpoint Control Register Bit-field Definitions */
/* Endpoint Control Set Register Bit-field Definitions */
/* Endpoint Control Clear Register Bit-field Definitions */

#define USBB_UECON_TXINE              (1 << 0)  /* Bit 0:  Transmitted IN Data Interrupt Enable */
#define USBB_UECON_RXOUTE             (1 << 1)  /* Bit 1:  Received OUT Data Interrupt Enable */
#define USBB_UECON_RXSTPE             (1 << 2)  /* Bit 2:  Received SETUP Interrupt Enable */
#define USBB_UECON_UNDERFE            (1 << 2)  /* Bit 2:  Underflow Interrupt Enable */
#define USBB_UECON_NAKOUTE            (1 << 3)  /* Bit 3:  NAKed OUT Interrupt Enable */
#define USBB_UECON_NAKINE             (1 << 4)  /* Bit 4:  NAKed IN Interrupt Enable */
#define USBB_UECON_OVERFE             (1 << 5)  /* Bit 5:  Overflow Interrupt Enable */
#define USBB_UECON_STALLEDE           (1 << 6)  /* Bit 6:  STALLed Interrupt Enable */
#define USBB_UECON_CRCERRE            (1 << 6)  /* Bit 6:  CRC Error Interrupt Enable */
#define USBB_UECON_SHORTPACKETE       (1 << 7)  /* Bit 7:  Short Packet Interrupt Enable */
#define USBB_UECON_NBUSYBKE           (1 << 12) /* Bit 12: Number of Busy Banks Interrupt Enable  */
#define USBB_UECON_KILLBK             (1 << 13) /* Bit 13: Kill IN Bank (SET only) */
#define USBB_UECON_FIFOCON            (1 << 14) /* Bit 14: FIFO Control (CLR only) */
#define USBB_UECON_EPDISHDMA          (1 << 16) /* Bit 16: Endpoint Interrupts Disable HDMA Request Enable  */
#define USBB_UECON_RSTDT              (1 << 18) /* Bit 18: Reset Data Toggle (SET only) */
#define USBB_UECON_STALLRQ            (1 << 19) /* Bit 19: STALL Request */

/* Device DMA Channel Next Descriptor Address Register Bit-field Definitions */

#define UDDMA_NEXTDESC_MASK           (0xfffffff0)

/* Device DMA Channel HSB Address Register Bit-field Definitions */
/* This register holds a 32-bit address with internal bit fields */

/* Device DMA Channel Control Register Bit-field Definitions */

#define UDDMA_CTRL_CHEN               (1 << 0)  /* Bit 0:  Channel Enable */
#define UDDMA_CTRL_LDNXTCHDESCEN      (1 << 1)  /* Bit 1:  Load Next Channel Descriptor Enable */
#define UDDMA_CTRL_BUFFCLOSEINEN      (1 << 2)  /* Bit 2:  Buffer Close Input Enable */
#define UDDMA_CTRL_DMAENDEN           (1 << 3)  /* Bit 3:  End of DMA Buffer Output Enable */
#define UDDMA_CTRL_EOTIRQEN           (1 << 4)  /* Bit 4:  End of USB Transfer Interrupt Enable */
#define UDDMA_CTRL_EOBUFFIRQEN        (1 << 5)  /* Bit 5:  End of Buffer Interrupt Enable */
#define UDDMA_CTRL_DESCLDIRQEN        (1 << 6)  /* Bit 6:  Descriptor Loaded Interrupt Enable */
#define UDDMA_CTRL_BURSTLOCKEN        (1 << 7)  /* Bit 7:  Burst Lock Enable */
#define UDDMA_CTRL_CHBYTELENGTH_SHIFT (16)      /* Bits 16-31: Channel Byte Length */
#define UDDMA_CTRL_CHBYTELENGTH_MASK  (0xffff << UDDMA_CTRL_CHBYTELENGTH_SHIFT)

/* Device DMA Channel Status Register Bit-field Definitions */

#define UDDMA_STATUS_CHEN             (1 << 0)  /* Bit 0:  Channel Enabled */
#define UDDMA_STATUS_CHACTIVE         (1 << 1)  /* Bit 1:  Channel Active */
#define UDDMA_STATUS_EOTSTA           (1 << 4)  /* Bit 4:  End of USB Transfer Status */
#define UDDMA_STATUS_EOCHBUFFSTA      (1 << 5)  /* Bit 5:  End of Channel Buffer Status */
#define UDDMA_STATUS_DESCLDSTA        (1 << 6)  /* Bit 6:  Descriptor Loaded Status */
#define UDDMA_STATUS_CHBYTECNT_SHIFT  (16)       /* Bits 16-31: Channel Byte Count */
#define UDDMA_STATUS_CHBYTECNT_MASK   (0xffff << UDDMA_STATUS_CHBYTECNT_SHIFT)

/* USB Host Registers Bit-field Definitions *********************************/

/* Host General Control Register Bit-field Definitions */

#define USBB_UHCON_SOFE               (1 << 8)  /* Bit 8:  Start of Frame Generation Enable */
#define USBB_UHCON_RESET              (1 << 9)  /* Bit 9:  Send USB Reset */
#define USBB_UHCON_RESUME             (1 << 10) /* Bit 10: Send USB Resume */

/* Host Global Interrupt Register Bit-field Definitions */
/* Host Global Interrupt Clear Register Bit-field Definitions (Except as noted 1) */
/* Host Global Interrupt Set Register Bit-field Definitions (Except as noted 2) */
/* Host Global Interrupt Enable Register Bit-field Definitions */
/* Host Global Interrupt Enable Clear Register Bit-field Definitions */
/* Host Global Interrupt Enable Set Register Bit-field Definitions */

#define USBB_UHINT_DCONNI             (1 << 0)  /* Bit 0:  Device Connection Interrupt (1) */
#define USBB_UHINT_DDISCI             (1 << 1)  /* Bit 1:  Device Disconnection Interrupt (1) */
#define USBB_UHINT_RSTI               (1 << 2)  /* Bit 2:  USB Reset Sent Interrupt (1) */
#define USBB_UHINT_RSMEDI             (1 << 3)  /* Bit 3:  Downstream Resume Sent Interrupt (1) */
#define USBB_UHINT_RXRSMI             (1 << 4)  /* Bit 4:  Upstream Resume Received Interrupt (1) */
#define USBB_UHINT_HSOFI              (1 << 5)  /* Bit 5:  Host Start of Frame Interrupt (1) */
#define USBB_UHINT_HWUPI              (1 << 6)  /* Bit 6:  Host Wake-Up Interrupt (1) */
#define USBB_UHINT_PINT(n)            (1 << ((n)+8))
#define USBB_UHINT_P0INT              (1 << 8)  /* Bit 8:  Pipe 0 Interrupt (1,2) */
#define USBB_UHINT_P1INT              (1 << 9)  /* Bit 9:  Pipe 1 Interrupt (1,2) */
#define USBB_UHINT_P2INT              (1 << 10) /* Bit 10: Pipe 2 Interrupt (1,2) */
#define USBB_UHINT_P3INT              (1 << 11) /* Bit 11: Pipe 3 Interrupt (1,2) */
#define USBB_UHINT_P4INT              (1 << 12) /* Bit 12: Pipe 4 Interrupt (1,2) */
#define USBB_UHINT_P5INT              (1 << 13) /* Bit 13: Pipe 5 Interrupt (1,2) */
#define USBB_UHINT_P6INT              (1 << 14) /* Bit 14: Pipe 6 Interrupt (1,2) */
#define USBB_UHINT_DMAINT(n)          (1 << ((n)+24))
#define USBB_UHINT_DMAINT1            (1 << 25) /* Bit 25: DMA Channel 1 Interrupt */
#define USBB_UHINT_DMAINT2            (1 << 26) /* Bit 26: DMA Channel 2 Interrupt */
#define USBB_UHINT_DMAINT3            (1 << 27) /* Bit 27: DMA Channel 3 Interrupt */
#define USBB_UHINT_DMAINT4            (1 << 28) /* Bit 28: DMA Channel 4 Interrupt */
#define USBB_UHINT_DMAINT5            (1 << 29) /* Bit 29: DMA Channel 5 Interrupt */
#define USBB_UHINT_DMAINT6            (1 << 30) /* Bit 30: DMA Channel 6 Interrupt */

/* Pipe Enable/Reset Register Bit-field Definitions */

#define USBB_UPRST_PEN(n)             (1 << (n))
#define USBB_UPRST_PEN0               (1 << 0)  /* Bit 0:  Pipe 0 Enable */
#define USBB_UPRST_PEN1               (1 << 1)  /* Bit 1:  Pipe 1 Enable */
#define USBB_UPRST_PEN2               (1 << 2)  /* Bit 2:  Pipe 2 Enable */
#define USBB_UPRST_PEN3               (1 << 3)  /* Bit 3:  Pipe 3 Enable */
#define USBB_UPRST_PEN4               (1 << 4)  /* Bit 4:  Pipe 4 Enable */
#define USBB_UPRST_PEN5               (1 << 5)  /* Bit 5:  Pipe 5 Enable */
#define USBB_UPRST_PEN6               (1 << 6)  /* Bit 6:  Pipe 6 Enable */
#define USBB_UPRST_PRST(n)            (1 << ((n)+16))
#define USBB_UPRST_PRST0              (1 << 16) /* Bit 16: Pipe 0 Reset */
#define USBB_UPRST_PRST1              (1 << 17) /* Bit 17: Pipe 1 Reset */
#define USBB_UPRST_PRST2              (1 << 18) /* Bit 18: Pipe 2 Reset */
#define USBB_UPRST_PRST3              (1 << 19) /* Bit 19: Pipe 3 Reset */
#define USBB_UPRST_PRST4              (1 << 20) /* Bit 20: Pipe 4 Reset */
#define USBB_UPRST_PRST5              (1 << 21) /* Bit 21: Pipe 5 Reset */
#define USBB_UPRST_PRST6              (1 << 22) /* Bit 22: Pipe 6 Reset */

/* Host Frame Number Register Bit-field Definitions */

#define USBB_UHFNUM_FNUM_SHIFT        (3)       /* Bits 3-13: Frame Number */
#define USBB_UHFNUM_FNUM_MASK         (0x7ff << USBB_UHFNUM_FNUM_SHIFT)
#define USBB_UHFNUM_FLENHIGH_SHIFT    (16)      /* Bits 16-23: Frame Length */
#define USBB_UHFNUM_FLENHIGH_MASK     (0xff << USBB_UHFNUM_FLENHIGH_SHIFT)

/* Host Address 1 Register Bit-field Definitions */

#define USBB_UHADDR1_UHADDRP0_SHIFT   (0)       /* Bits 0-6: USB Host Address (Pipe 0) */
#define USBB_UHADDR1_UHADDRP0_MASK    (0x7f << USBB_UHADDR1_UHADDRP0_SHIFT)
#define USBB_UHADDR1_UHADDRP1_SHIFT   (8)       /* Bits 8-14: USB Host Address (Pipe 1) */
#define USBB_UHADDR1_UHADDRP1_MASK    (0x7f << USBB_UHADDR1_UHADDRP1_SHIFT)
#define USBB_UHADDR1_UHADDRP2_SHIFT   (16)      /* Bits 16-22: USB Host Address (Pipe 2) */
#define USBB_UHADDR1_UHADDRP2_MASK    (0x7f << USBB_UHADDR1_UHADDRP2_SHIFT)
#define USBB_UHADDR1_UHADDRP3_SHIFT   (24)      /* Bits 24-30: USB Host Address (Pipe 3) */
#define USBB_UHADDR1_UHADDRP3_MASK    (0x7f << USBB_UHADDR1_UHADDRP3_SHIFT)

/* Host Address 2 Register Bit-field Definitions */

#define USBB_UHADDR2_UHADDRP4_SHIFT   (0)       /* Bits 0-6: USB Host Address (Pipe 4) */
#define USBB_UHADDR2_UHADDRP4_MASK    (0x7f << USBB_UHADDR1_UHADDRP4_SHIFT)
#define USBB_UHADDR2_UHADDRP5_SHIFT   (8)       /* Bits 8-14: USB Host Address (Pipe 5) */
#define USBB_UHADDR2_UHADDRP5_MASK    (0x7f << USBB_UHADDR1_UHADDRP5_SHIFT)
#define USBB_UHADDR2_UHADDRP6_SHIFT   (16)      /* Bits 16-22: USB Host Address (Pipe 6) */
#define USBB_UHADDR2_UHADDRP6_MASK    (0x7f << USBB_UHADDR1_UHADDRP6_SHIFT)
#
/* Pipe Configuration Register Bit-field Definitions */

#define USBB_UPCFG_ALLOC              (1 << 1)  /* Bit 1:  Pipe Memory Allocate */
#define USBB_UPCFG_PBK_SHIFT          (2)       /* Bits 2-3: Pipe Banks */
#define USBB_UPCFG_PBK_MASK           (3 << USBB_UPCFG_PBK_SHIFT)
#  define USBB_UPCFG_PBK_1            (0 << USBB_UPCFG_PBK_SHIFT) /* 1 (single-bank pipe) */
#  define USBB_UPCFG_PBK_2            (1 << USBB_UPCFG_PBK_SHIFT) /* 2 (double-bank pipe) */
#  define USBB_UPCFG_PBK_3            (2 << USBB_UPCFG_PBK_SHIFT) /* 3 (triple-bank pipe) */
#define USBB_UPCFG_PSIZE_SHIFT        (4)       /* Bits 4-6: Pipe Size */
#define USBB_UPCFG_PSIZE_MASK         (7 << USBB_UPCFG_PSIZE_SHIFT)
#  define USBB_UPCFG_PSIZE_8          (0 << USBB_UPCFG_PSIZE_SHIFT) /* 8 bytes */
#  define USBB_UPCFG_PSIZE_16         (1 << USBB_UPCFG_PSIZE_SHIFT) /* 16 bytes */
#  define USBB_UPCFG_PSIZE_32         (2 << USBB_UPCFG_PSIZE_SHIFT) /* 32 bytes */
#  define USBB_UPCFG_PSIZE_64         (3 << USBB_UPCFG_PSIZE_SHIFT) /* 64 bytes */
#  define USBB_UPCFG_PSIZE_128        (4 << USBB_UPCFG_PSIZE_SHIFT) /* 128 bytes */
#  define USBB_UPCFG_PSIZE_256        (5 << USBB_UPCFG_PSIZE_SHIFT) /* 256 bytes */
#  define USBB_UPCFG_PSIZE_512        (6 << USBB_UPCFG_PSIZE_SHIFT) /* 512 bytes */
#  define USBB_UPCFG_PSIZE_1024       (7 << USBB_UPCFG_PSIZE_SHIFT) /* 1024 bytes */
#define USBB_UPCFG_PTOKEN_SHIFT       (8)       /* Bits 8-9: Pipe Token */
#define USBB_UPCFG_PTOKEN_MASK        (3 << USBB_UPCFG_PTOKEN_SHIFT)
#  define USBB_UPCFG_PTOKEN_SETUP     (0 << USBB_UPCFG_PTOKEN_SHIFT) /* SETUP */
#  define USBB_UPCFG_PTOKEN_IN        (1 << USBB_UPCFG_PTOKEN_SHIFT) /* IN */
#  define USBB_UPCFG_PTOKEN_OUT       (2 << USBB_UPCFG_PTOKEN_SHIFT) /* OUT */
#define USBB_UPCFG_AUTOSW             (1 << 10) /* Bit 10:  Automatic Switch */
#define USBB_UPCFG_PTYPE_SHIFT        (11)      /* Bits 11-12: Pipe Type */
#define USBB_UPCFG_PTYPE_MASK         (3 << USBB_UPCFG_PTYPE_SHIFT)
#  define USBB_UPCFG_PTYPE_CTRL       (0 << USBB_UPCFG_PTYPE_SHIFT) /* Control */
#  define USBB_UPCFG_PTYPE_ISOC       (1 << USBB_UPCFG_PTYPE_SHIFT) /* Isochronous */
#  define USBB_UPCFG_PTYPE_BULK       (2 << USBB_UPCFG_PTYPE_SHIFT) /* Bulk */
#  define USBB_UPCFG_PTYPE_INTR       (3 << USBB_UPCFG_PTYPE_SHIFT) /* Interrupt */
#define USBB_UPCFG_PEPNUM_SHIFT       (16)      /* Bits 16-19: Pipe Endpoint Number */
#define USBB_UPCFG_PEPNUM_MASK        (15 << USBB_UPCFG_PEPNUM_SHIFT)
#define USBB_UPCFG_INTFRQ_SHIFT       (24)      /* Bits 24-31: Pipe Interrupt Request Frequency */
#define USBB_UPCFG_INTFRQ_MASK        (0xff << USBB_UPCFG_INTFRQ_SHIFT)

/* Pipe Status Register Bit-field Definitions (common) */
/* Pipe Status Clear Register Bit-field Definitions (common) */
/* Pipe Status Set Register Bit-field Definitions (common) */

#define USBB_UPSTA_RXINI              (1 << 0)  /* Bit 0:  Received IN Data Interrupt */
#define USBB_UPSTA_TXOUTI             (1 << 1)  /* Bit 1:  Transmitted OUT Data Interrupt */
#define USBB_UPSTA_TXSTPI             (1 << 2)  /* Bit 2:  Transmitted SETUP Interrupt */
#define USBB_UPSTA_UNDERFI            (1 << 2)  /* Bit 2:  Underflow Interrupt */
#define USBB_UPSTA_PERRI              (1 << 3)  /* Bit 3:  Pipe Error Interrupt */
#define USBB_UPSTA_NAKEDI             (1 << 4)  /* Bit 4:  NAKed Interrupt */
#define USBB_UPSTA_OVERFI             (1 << 5)  /* Bit 5:  Overflow Interrupt */
#define USBB_UPSTA_RXSTALLDI          (1 << 6)  /* Bit 6:  Received STALLed Interrupt */
#define USBB_UPSTA_CRCERRI            (1 << 6)  /* Bit 6:  CRC Error Interrupt */
#define USBB_UPSTA_SHORTPACKET        (1 << 7)  /* Bit 7:  Short Packet Interrupt */

/* Pipe Status Register Bit-field Definitions (only in UPSTA) */

#define USBB_UPSTA_DTSEQ_SHIFT        (8)       /* Bits 8-9: Data Toggle Sequence */
#define USBB_UPSTA_DTSEQ_MASK         (3 << USBB_UPSTA_DTSEQ_SHIFT)
#  define USBB_UPSTA_DTSEQ_DATA0      (0 << USBB_UPSTA_DTSEQ_SHIFT) /* Data0 */
#  define USBB_UPSTA_DTSEQ_DATA1      (1 << USBB_UPSTA_DTSEQ_SHIFT) /* Data1 */
#define USBB_UPSTA_NBUSYBK_SHIFT      (12)      /* Bits 12-13: Number of Busy Banks */
#define USBB_UPSTA_NBUSYBK_MASK       (3 << USBB_UPSTA_NBUSYBK_SHIFT)
#  define USBB_UPSTA_NBUSYBK_NONE     (0 << USBB_UPSTA_NBUSYBK_SHIFT) /* 0 (all banks free) */
#  define USBB_UPSTA_NBUSYBK_1BANK    (1 << USBB_UPSTA_NBUSYBK_SHIFT) /* 1 */
#  define USBB_UPSTA_NBUSYBK_2BANKS   (2 << USBB_UPSTA_NBUSYBK_SHIFT) /* 2 */
#define USBB_UPSTA_CURRBK_SHIFT       (14)      /* Bits 14-15: Current Bank */
#define USBB_UPSTA_CURRBK_MASK        (3 << USBB_UPSTA_CURRBK_SHIFT)
#  define USBB_UPSTA_CURRBK_BANK0     (0 << USBB_UPSTA_CURRBK_SHIFT) /* Bank0 */
#  define USBB_UPSTA_CURRBK_BANK1     (1 << USBB_UPSTA_CURRBK_SHIFT) /* Bank1 */
#  define USBB_UPSTA_CURRBK_BANK2     (2 << USBB_UPSTA_CURRBK_SHIFT) /* Bank2 */
#define USBB_UPSTA_RWALL              (1 << 16) /* Bit 16: Read/Write Allowed */
#define USBB_UPSTA_CFGOK              (1 << 18) /* Bit 18: Configuration OK Status */
#define USBB_UPSTA_PBYCT_SHIFT        (20)       /* Bits 20-30: Pipe Byte Count */
#define USBB_UPSTA_PBYCT_MASK         (0x7ff << USBB_UPSTA_BYCT_SHIFT)

/* Pipe Status Set Register Bit-field Definitions (only in UPSTASET) */

#define USBB_UPSTASET_NBUSYBKS        (1 << 12)  /* Bit 12 */

/* Pipe Control Register Bit-field Definitions */
/* Pipe Control Clear Register Bit-field Definitions (except as noted 1) */
/* Pipe Control Set Register Bit-field Definitions (except as noted 2) */

#define USBB_UPCON_RXINE              (1 << 0)  /* Bit 0:  Received IN Data Interrupt Enable */
#define USBB_UPCON_TXOUTE             (1 << 1)  /* Bit 1:  Transmitted OUT Data Interrupt Enable */
#define USBB_UPCON_TXSTPE             (1 << 2)  /* Bit 2:  Transmitted SETUP Interrupt Enable */
#define USBB_UPCON_UNDERFIE           (1 << 2)  /* Bit 2:  Underflow Interrupt Enable */
#define USBB_UPCON_PERRE              (1 << 3)  /* Bit 3:  Pipe Error Interrupt Enable */
#define USBB_UPCON_NAKEDE             (1 << 4)  /* Bit 4:  NAKed Interrupt Enable */
#define USBB_UPCON_OVERFIE            (1 << 5)  /* Bit 5:  Overflow Interrupt Enable */
#define USBB_UPCON_RXSTALLDE          (1 << 6)  /* Bit 6:  Received STALLed Interrupt Enable */
#define USBB_UPCON_CRCERRE            (1 << 6)  /* Bit 6:  CRC Error Interrupt Enable */
#define USBB_UPCON_SHORTPACKETIE      (1 << 7)  /* Bit 7:  Short Packet Interrupt Enable */
#define USBB_UPCON_NBUSYBKE           (1 << 12) /* Bit 12:  Number of Busy Banks Interrupt Enable */
#define USBB_UPCON_FIFOCON            (1 << 14) /* Bit 14: FIFO Control (2) */
#define USBB_UPCON_PDISHDMA           (1 << 16) /* Bit 16: Pipe Interrupts Disable HDMA Request Enable */
#define USBB_UPCON_PFREEZE            (1 << 17) /* Bit 17: Pipe Freeze */
#define USBB_UPCON_RSTDT              (1 << 18) /* Bit 18: Reset Data Toggle (1) */

/* Pipe IN Request Register Bit-field Definitions */

#define USBB_UPINRQ_INRQ_SHIFT        (0)       /* Bits 0-7: IN Request Number before Freeze */
#define USBB_UPINRQ_INRQ_MASK         (0xff << USBB_UPINRQ_INRQ_SHIFT)
#define USBB_UPINRQ_INMODE            (1 << 8)  /* Bit 8:  IN Request Mode */

/* Pipe Error Register Bit-field Definitions */

#define USBB_UPERR_DATATGL            (1 << 0)  /* Bit 0:  Data Toggle Error */
#define USBB_UPERR_DATAPID            (1 << 1)  /* Bit 1:  Data PID Error */
#define USBB_UPERR_PID                (1 << 2)  /* Bit 2:  PID Error */
#define USBB_UPERR_TIMEOUT            (1 << 3)  /* Bit 3:  Time-Out Error */
#define USBB_UPERR_CRC16              (1 << 4)  /* Bit 4:  CRC16 Error */
#define USBB_UPERR_COUNTER_SHIFT      (5)       /* Bits 5-6:  Error Counter */
#define USBB_UPERR_COUNTER_MASK       (3 << USBB_UPERR_COUNTER_SHIFT)

/* Host DMA Channel Next Descriptor Address Register Bit-field Definitions */

#define UHDMA_NEXTDESC_MASK           (0xfffffff0)

/* Host DMA Channel HSB Address Register Bit-field Definitions */
/* This register holds a 32-bit address with internal bit fields */

/* Host DMA Channel Control Register Bit-field Definitions */

#define UHDMA_CTRL_CHEN               (1 << 0)  /* Bit 0:  Channel Enable */
#define UHDMA_CTRL_LDNXTCHDESCEN      (1 << 1)  /* Bit 1:  Load Next Channel Descriptor Enable */
#define UHDMA_CTRL_BUFFCLOSEINEN      (1 << 2)  /* Bit 2:  Buffer Close Input Enable */
#define UHDMA_CTRL_DMAENDEN           (1 << 3)  /* Bit 3:  End of DMA Buffer Output Enable */
#define UHDMA_CTRL_EOTIRQEN           (1 << 4)  /* Bit 4:  End of USB Transfer Interrupt Enable */
#define UHDMA_CTRL_EOBUFFIRQEN        (1 << 5)  /* Bit 5:  End of Buffer Interrupt Enable */
#define UHDMA_CTRL_DESCLDIRQEN        (1 << 6)  /* Bit 6:  Descriptor Loaded Interrupt Enable */
#define UHDMA_CTRL_BURSTLOCKEN        (1 << 7)  /* Bit 7:  Burst Lock Enable */
#define UHDMA_CTRL_CHBYTELENGTH_SHIFT (16)      /* Bits 16-31: Channel Byte Length */
#define UHDMA_CTRL_CHBYTELENGTH_MASK  (0xffff << UHDMA_CTRL_CHBYTELENGTH_SHIFT)

/* Host DMA Channel Status Register Bit-field Definitions */

#define UHDMA_STATUS_CHEN             (1 << 0)  /* Bit 0:  Channel Enabled */
#define UHDMA_STATUS_CHACTIVE         (1 << 1)  /* Bit 1:  Channel Active */
#define UHDMA_STATUS_EOTSTA           (1 << 4)  /* Bit 4:  End of USB Transfer Status */
#define UHDMA_STATUS_EOCHBUFFSTA      (1 << 5)  /* Bit 5:  End of Channel Buffer Status */
#define UHDMA_STATUS_DESCLDSTA        (1 << 6)  /* Bit 6:  Descriptor Loaded Status */
#define UHDMA_STATUS_CHBYTECNT_SHIFT  (16)       /* Bits 16-31: Channel Byte Count */
#define UHDMA_STATUS_CHBYTECNT_MASK   (0xffff << UHDMA_STATUS_CHBYTECNT_SHIFT)

/* USB General Registers Bit-field Definitions ******************************/

/* General Control Register Bit-field Definitions */

#define USBB_USBCON_IDTE              (1 << 0)  /* Bit 0:  ID Transition Interrupt Enable */
#define USBB_USBCON_VBUSTE            (1 << 1)  /* Bit 1:  VBus Transition Interrupt Enable */
#define USBB_USBCON_VBERRE            (1 << 3)  /* Bit 3:  VBus Error Interrupt Enable */
#define USBB_USBCON_BCERRE            (1 << 4)  /* Bit 4:  B-Connection Error Interrupt Enable */
#define USBB_USBCON_ROLEEXE           (1 << 5)  /* Bit 5:  Role Exchange Interrupt Enable */
#define USBB_USBCON_STOE              (1 << 7)  /* Bit 7:  Suspend Time-Out Interrupt Enable */
#define USBB_USBCON_VBUSHWC           (1 << 8)  /* Bit 8:  VBus Hardware Control */
#define USBB_USBCON_OTGPADE           (1 << 12) /* Bit 12: OTG Pad Enable */
#define USBB_USBCON_VBUSPO            (1 << 13) /* Bit 13: VBus Polarity */
#define USBB_USBCON_FRZCLK            (1 << 14) /* Bit 14: Freeze USB Clock */
#define USBB_USBCON_USBE              (1 << 15) /* Bit 15: USBB Enable */
#define USBB_USBCON_TIMVALUE_SHIFT    (16)      /* Bits 16-17: Timmer Value */
#define USBB_USBCON_TIMVALUE_MASK     (3 << USBB_USBCON_TIMVALUE_SHIFT)
#define USBB_USBCON_TIMPAGE_SHIFT     (20)      /* Bits 20-21: Timer Page */
#define USBB_USBCON_TIMPAGE_MASK      (3 << USBB_USBCON_TIMPAGE_SHIFT)
#define USBB_USBCON_UNLOCK            (1 << 22) /* Bit 22: Timer Access Unlock */
#define USBB_USBCON_UIDE              (1 << 24) /* Bit 24: USB_ID Pin Enable */
#define USBB_USBCON_UIMOD             (1 << 25) /* Bit 25: USBB Mode */

/* General Status Register Bit-field Definitions */
/* General Status Clear Register Bit-field Definitions */
/* General Status Set Register Bit-field Definitions */

#define USBB_USBSTA_IDTI              (1 << 0)  /* Bit 0:  ID Transition Interrupt */
#define USBB_USBSTA_VBUSTI            (1 << 1)  /* Bit 1:  VBus Transition Interrupt */
#define USBB_USBSTA_VBERRI            (1 << 3)  /* Bit 3:  VBus Error Interrupt */
#define USBB_USBSTA_BCERRI            (1 << 4)  /* Bit 4:  B-Connection Error Interrupt */
#define USBB_USBSTA_ROLEEXI           (1 << 5)  /* Bit 5:  Role Exchange Interrupt */
#define USBB_USBSTA_STOI              (1 << 7)  /* Bit 7:  Suspend Time-Out Interrupt */
#define USBB_USBSTA_VBUSRQ            (1 << 9)  /* Bit 8:  VBus Request */
#define USBB_USBSTA_ID                (1 << 10) /* Bit 10: USB_ID Pin State (read-only) */
#define USBB_USBSTA_VBUS              (1 << 11) /* Bit 11: VBus Level (read-only) */
#define USBB_USBSTA_SPEED_SHIFT       (12)      /* Bits 12-13:  Speed Status (read-only) */
#define USBB_USBSTA_SPEED_MASK        (3 << USBB_USBSTA_SPEED_SHIFT)
#  define USBB_USBSTA_SPEED_FULL      (0 << USBB_USBSTA_SPEED_SHIFT) /* Full-Speed mode */
#  define USBB_USBSTA_SPEED_LOW       (2 << USBB_USBSTA_SPEED_SHIFT) /* Low-Speed mode */

/* IP Version Register Bit-field Definitions */

#define USBB_UVERS_SHIFT              (0)       /* Bits 0-11: Version Number */
#define USBB_UVERS_MASK               (0xfff << USBB_UVERS_SHIFT)
#define USBB_UVERS_VARIANT_SHIFT      (16)      /* Bits 16-19: Variant Number */
#define USBB_UVERS_VARIANT_MASK       (15 << USBB_UVERS_VARIANT_SHIFT)

/* IP Features Register Bit-field Definitions */

#define USBB_UFEAT_EPTNBRMAX_SHIFT    (0)       /* Bits 0-3: Maximal Number of Pipes/Endpoints */
#define USBB_UFEAT_EPTNBRMAX_MASK     (15 << USBB_UFEAT_EPTNBRMAX_SHIFT)
#  define USBB_UFEAT_EPTNBRMAX_16     (0 << USBB_UFEAT_EPTNBRMAX_SHIFT) /* 16 is a special case */
#define USBB_UFEAT_DMACHANNBR_SHIFT   (4)       /* Bits 4-6: Number of DMA Channels */
#define USBB_UFEAT_DMACHANNBR_MASK    (7 << USBB_UFEAT_DMACHANNBR_SHIFT)
#define USBB_UFEAT_DMABUFFERSZ        (1 << 7)  /* Bit 7:  DMA Buffer Size */
#define USBB_UFEAT_DMAWDDEPTH_SHIFT   (8)       /* Bits 8-11: DMA FIFO Depth in Words */
#define USBB_UFEAT_DMAWDDEPTH_MASK    (15 << USBB_UFEAT_DMAWDDEPTH_SHIFT)
#  define USBB_UFEAT_DMAWDDEPTH_16    (0 << USBB_UFEAT_DMAWDDEPTH_SHIFT) /* 16 is a special case */
#define USBB_UFEAT_FIFOMAXSZ_SHIFT    (12)      /* Bits 12-14: Maximal FIFO Size */
#define USBB_UFEAT_FIFOMAXSZ_MASK     (7 << USBB_UFEAT_FIFOMAXSZ_SHIFT)
#  define USBB_UFEAT_FIFOMAXSZ_LT256  (0 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 256 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_LT512  (1 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 512 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_LT1K   (2 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 1024 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_LT2K   (3 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 2048 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_LT4K   (4 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 4096 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_LT8K   (5 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 8192 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_LT16K  (6 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* < 16384 bytes */
#  define USBB_UFEAT_FIFOMAXSZ_GE16K  (7 << USBB_UFEAT_FIFOMAXSZ_SHIFT) /* >= 16384 bytes */
#define USBB_UFEAT_BWRDPRAM           (1 << 15) /* Bit 15: DPRAM Byte-Write Capability */

/* IP PB Address Size Register Bit-field Definitions */
/* IP Name Register 1 Bit-field Definitions */
/* IP Name Register 2 Bit-field Definitions */

/* These registers contain a 32-value and, hence, have no bit fields */

/* USB Finite State Machine Status Register Bit-field Definitions */

#define USBB_USBFSM_MASK              (15)
#  define USBB_USBFSM_A_IDLESTATE     (0)
#  define USBB_USBFSM_A_WAITVRISE     (1)
#  define USBB_USBFSM_A_WAITBCON      (2)
#  define USBB_USBFSM_A_HOST          (3)
#  define USBB_USBFSM_A_SUSPEND       (4)
#  define USBB_USBFSM_A_PERIPHERAL    (5)
#  define USBB_USBFSM_A_WAITVFALL     (6)
#  define USBB_USBFSM_A_VBUSERR       (7)
#  define USBB_USBFSM_A_WAITDISCHARGE (8)
#  define USBB_USBFSM_B_IDLE          (9)
#  define USBB_USBFSM_B_PERIPHERAL    (10)
#  define USBB_USBFSM_B_WAITBEGINHNP  (11)
#  define USBB_USBFSM_B_WAITDISCHARGE (12)
#  define USBB_USBFSM_B_WAITACON      (13)
#  define USBB_USBFSM_B_HOST          (14)
#  define USBB_USBFSM_B_SRPINIT       (15)

/* USB HSB Memory Map ***************************************************************/

#define USB_FIFO0_DATA_OFFSET         0x00000 /* Pipe/Endpoint 0 FIFO Data Register */
#define USB_FIFO1_DATA_OFFSET         0x10000 /* Pipe/Endpoint 1 FIFO Data Register */
#define USB_FIFO2_DATA_OFFSET         0x20000 /* Pipe/Endpoint 2 FIFO Data Register */
#define USB_FIFO3_DATA_OFFSET         0x30000 /* Pipe/Endpoint 3 FIFO Data Register */
#define USB_FIFO4_DATA_OFFSET         0x40000 /* Pipe/Endpoint 4 FIFO Data Register */
#define USB_FIFO5_DATA_OFFSET         0x50000 /* Pipe/Endpoint 5 FIFO Data Register */
#define USB_FIFO6_DATA_OFFSET         0x60000 /* Pipe/Endpoint 6 FIFO Data Register */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_USBB_H */

