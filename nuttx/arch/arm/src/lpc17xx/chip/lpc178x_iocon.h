/************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc178x_iocon.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Rommel Marcelo
 *           Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_LPC178X_IOCON_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_LPC178X_IOCON_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC17_IOCON_P0_OFFSET       (LPC17_IOCON_BASE+0x0000)
#define LPC17_IOCON_P1_OFFSET       (LPC17_IOCON_BASE+0x0080)
#define LPC17_IOCON_P2_OFFSET       (LPC17_IOCON_BASE+0x0100)
#define LPC17_IOCON_P3_OFFSET       (LPC17_IOCON_BASE+0x0180)
#define LPC17_IOCON_P4_OFFSET       (LPC17_IOCON_BASE+0x0200)
#define LPC17_IOCON_P5_OFFSET       (LPC17_IOCON_BASE+0x0280)

#define LPC17_IOCON_PP0_OFFSET      (0x0000) /* IOCON Port(n) register 0  */
#define LPC17_IOCON_PP1_OFFSET      (0x0004) /* IOCON Port(n) register 1  */
#define LPC17_IOCON_PP2_OFFSET      (0x0008) /* IOCON Port(n) register 2  */
#define LPC17_IOCON_PP3_OFFSET      (0x000c) /* IOCON Port(n) register 3  */
#define LPC17_IOCON_PP4_OFFSET      (0x0010) /* IOCON Port(n) register 4  */
#define LPC17_IOCON_PP5_OFFSET      (0x0014) /* IOCON Port(n) register 5  */
#define LPC17_IOCON_PP6_OFFSET      (0x0018) /* IOCON Port(n) register 6  */
#define LPC17_IOCON_PP7_OFFSET      (0x001c) /* IOCON Port(n) register 7 */
#define LPC17_IOCON_PP8_OFFSET      (0x0020) /* IOCON Port(n) register 8  */
#define LPC17_IOCON_PP9_OFFSET      (0x0024) /* IOCON Port(n) register 9  */
#define LPC17_IOCON_PP10_OFFSET     (0x0028) /* IOCON Port(n) register 10  */
#define LPC17_IOCON_PP11_OFFSET     (0x002c) /* IOCON Port(n) register 11  */
#define LPC17_IOCON_PP12_OFFSET     (0x0030) /* IOCON Port(n) register 12  */
#define LPC17_IOCON_PP13_OFFSET     (0x0034) /* IOCON Port(n) register 13  */
#define LPC17_IOCON_PP14_OFFSET     (0x0038) /* IOCON Port(n) register 14  */
#define LPC17_IOCON_PP15_OFFSET     (0x003c) /* IOCON Port(n) register 15  */
#define LPC17_IOCON_PP16_OFFSET     (0x0040) /* IOCON Port(n) register 16  */
#define LPC17_IOCON_PP17_OFFSET     (0x0044) /* IOCON Port(n) register 17  */
#define LPC17_IOCON_PP18_OFFSET     (0x0048) /* IOCON Port(n) register 18  */
#define LPC17_IOCON_PP19_OFFSET     (0x004c) /* IOCON Port(n) register 19  */
#define LPC17_IOCON_PP20_OFFSET     (0x0050) /* IOCON Port(n) register 20  */
#define LPC17_IOCON_PP21_OFFSET     (0x0054) /* IOCON Port(n) register 21  */
#define LPC17_IOCON_PP22_OFFSET     (0x0058) /* IOCON Port(n) register 22  */
#define LPC17_IOCON_PP23_OFFSET     (0x005c) /* IOCON Port(n) register 23  */
#define LPC17_IOCON_PP24_OFFSET     (0x0060) /* IOCON Port(n) register 24  */
#define LPC17_IOCON_PP25_OFFSET     (0x0064) /* IOCON Port(n) register 25  */
#define LPC17_IOCON_PP26_OFFSET     (0x0068) /* IOCON Port(n) register 26  */
#define LPC17_IOCON_PP27_OFFSET     (0x006c) /* IOCON Port(n) register 27  */
#define LPC17_IOCON_PP28_OFFSET     (0x0070) /* IOCON Port(n) register 28  */
#define LPC17_IOCON_PP29_OFFSET     (0x0074) /* IOCON Port(n) register 29  */
#define LPC17_IOCON_PP30_OFFSET     (0x0078) /* IOCON Port(n) register 30  */
#define LPC17_IOCON_PP31_OFFSET     (0x007c) /* IOCON Port(n) register 31  */

/* Register addresses ***************************************************************/

//~ #define LPC17_IOCON_PP1(portoffset)        (portoffset+LPC17_IOCON_P0_OFFSET)

/* Register bit definitions *********************************************************/
/* Pin Function Select register 0 (PINSEL0: 0x4002c000) */
/* IOCON pin function select */

#define IOCON_FUNC_GPIO             (0)
#define IOCON_FUNC_ALT1             (1)
#define IOCON_FUNC_ALT2             (2)
#define IOCON_FUNC_ALT3             (3)
#define IOCON_FUNC_ALT4             (4)
#define IOCON_FUNC_ALT5             (5)
#define IOCON_FUNC_ALT6             (6)
#define IOCON_FUNC_ALT7             (7)

#define IOCON_FUNC_SHIFT            (0)   /* Bits 0-2: All types */ 
#define IOCON_FUNC_MASK             (7 << IOCON_FUNC_SHIFT)
#define IOCON_MODE_SHIFT            (3)   /* Bits 3-4: Type D,A,W */ 
#define IOCON_MODE_MASK             (3 << IOCON_MODE_SHIFT )
#define IOCON_HYS_SHIFT             (5)   /* Bit 5: Type D,W  */
#define IOCON_HYS_MASK              (1 << IOCON_HYS_SHIFT)
#define IOCON_INV_SHIFT             (6)   /* Bit 6: Typ D,A,I,W  */
#define IOCON_INV_MASK              (1 << IOCON_INV_SHIFT)
#define IOCON_ADMODE_SHIFT          (7)   /* Bit 7: Type A */
#define IOCON_ADMODE_MASK           (1 << IOCON_ADMODE_SHIFT)
#define IOCON_FILTER_SHIFT          (8)   /* Bit 8: Type A */
#define IOCON_FILTER_MASK           (1 << IOCON_FILTER_SHIFT)
#define IOCON_SLEW_SHIFT            (9)   /* Bit 9: Type W*/
#define IOCON_SLEW_MASK             (1 << IOCON_SLEW_SHIFT)
#define IOCON_HIDRIVE_SHIFT         (9)   /* Bit 9: Type I */
#define IOCON_HIDRIVE_MASK          (1 << IOCON_HIDRIVE_SHIFT)
#define IOCON_OD_SHIFT              (10)  /* Bit 10: Type D,A,W */
#define IOCON_OD_MASK               (1 << IOCON_OD_SHIFT)
#define IOCON_DACEN_SHIFT           (16)  /* Bit 16: Type A */
#define IOCON_DACEN_MASK            (1 << IOCON_DACEN_SHIFT)

/* Pin modes */

#define IOCON_MODE_FLOAT            (0)      /* 00: pin has neither pull-up nor pull-down */
#define IOCON_MODE_PD               (1)      /* 00: pin has a pull-down resistor enabled */
#define IOCON_MODE_PU               (2)      /* 00: pin has a pull-up resistor enabled */
#define IOCON_MODE_RM               (3)      /* 00: pin has repeater mode enabled */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_LPC178X_IOCON_H */
