/************************************************************************************
 * arch/hc/src/m9s12/m9s12_mebi.h (v3)
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_HC_SRC_M9S12_M9S12_MEBI_H
#define __ARCH_ARM_HC_SRC_M9S12_M9S12_MEBI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/
/* Offsets relative to CORE1 */

#define HCS12_MEBI_PORTA_OFFSET    0x0000 /* Port A Data Register */
#define HCS12_MEBI_PORTB_OFFSET    0x0001 /* Port B Data Register */
#define HCS12_MEBI_DDRA_OFFSET     0x0002 /* Data Direction Register A */
#define HCS12_MEBI_DDRB_OFFSET     0x0003 /* Data Direction Register B */
#define HCS12_MEBI_PORTE_OFFSET    0x0008 /* Port E Data Register */
#define HCS12_MEBI_DDRE_OFFSET     0x0009 /* Data Direction Register E */
#define HCS12_MEBI_PEAR_OFFSET     0x000a /* Port E Assignment Register */
#define HCS12_MEBI_MODE_OFFSET     0x000b /* Mode Register */
#define HCS12_MEBI_PUCR_OFFSET     0x000c /* Pull Control Register */
#define HCS12_MEBI_RDRIV_OFFSET    0x000d /* Reduced Drive Register */
#define HCS12_MEBI_EBICTL_OFFSET   0x000e /* External Bus Interface Control Register */

/* Offsets relative to CORE2 */

#define HCS12_MEBI_IRQCR_OFFSET    0x0002 /* IRQ Control Register */

/* Offsets relative to CORE4 */

#define HCS12_MEBI_PORTK_OFFSET    0x0002 /* Port K Data Register */
#define HCS12_MEBI_DDRK_OFFSET     0x0003 /* Data Direction Register K */

/* Register Addresses ***************************************************************/

#define HCS12_MEBI_PORTA           (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MEBI_PORTA_OFFSET)
#define HCS12_MEBI_PORTB           (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MEBI_PORTB_OFFSET)
#define HCS12_MEBI_DDRA            (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MEBI_DDRA_OFFSET)
#define HCS12_MEBI_DDRB            (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MEBI_DDRB_OFFSET)
#define HCS12_MEBI_PORTE           (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MEBI_PORTE_OFFSET)
#define HCS12_MEBI_DDRE            (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MEBI_DDRE_OFFSET)
#define HCS12_MEBI_PEAR            (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MEBI_PEAR_OFFSET)
#define HCS12_MEBI_MODE            (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MEBI_MODE_OFFSET)
#define HCS12_MEBI_PUCR            (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MEBI_PUCR_OFFSET)
#define HCS12_MEBI_RDRIV           (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MEBI_RDRIV_OFFSET)
#define HCS12_MEBI_EBICTL          (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_MEBI_EBICTL_OFFSET)
#define HCS12_MEBI_IRQCR           (HCS12_REG_BASE+HCS12_CORE2_BASE+HCS12_MEBI_IRQCR_OFFSET)
#define HCS12_MEBI_PORTK           (HCS12_REG_BASE+HCS12_CORE4_BASE+HCS12_MEBI_PORTK_OFFSET)
#define HCS12_MEBI_DDRK            (HCS12_REG_BASE+HCS12_CORE4_BASE+HCS12_MEBI_DDRK_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* Port A Data Register Bit-Field Definitions */
/* Port B Data Register Bit-Field Definitions */
/* Data Direction Register A Bit-Field Definitions */
/* Data Direction Register B Bit-Field Definitions */
/* Port E Data Register Bit-Field Definitions */
/* Data Direction Register E Bit-Field Definitions */
/* Port K Data Register Bit-Field Definitions */
/* Data Direction Register K Bit-Field Definitions */

#define MEBI_PIN(n)                (1 << (n))         
#define MEBI_PIN0                  (1 << 0)
#define MEBI_PIN1                  (1 << 1)
#define MEBI_PIN2                  (1 << 2)
#define MEBI_PIN3                  (1 << 3)
#define MEBI_PIN4                  (1 << 4)
#define MEBI_PIN5                  (1 << 5)
#define MEBI_PIN6                  (1 << 6)
#define MEBI_PIN7                  (1 << 7)

/* Port E Assignment Register Bit-Field Definitions */

#define MEBI_PEAR_RDWE             (1 << 2)  /* Bit 2:  Read/Write Enable */
#define MEBI_PEAR_LSTRE            (1 << 3)  /* Bit 3:  Low Strobe (~LSTRB) Enable */
#define MEBI_PEAR_NECLK            (1 << 4)  /* Bit 4:  No External E Clock */
#define MEBI_PEAR_PIPOE            (1 << 5)  /* Bit 5:  Pipe Status Signal Output Enable */
#define MEBI_PEAR_NOACCE           (1 << 7)  /* Bit 7:  CPU No Access Output Enable */

/* Mode Register Bit-Field Definitions */

#define MEBI_MODE_EME              (1 << 0)  /* Bit 0:  Emulate Port E */
#define MEBI_MODE_EMK              (1 << 1)  /* Bit 1:  Emulate Port K */
#define MEBI_MODE_IVIS             (1 << 3)  /* Bit 3:  Internal Visibility */
#define MEBI_MODE_MOD_SHIFT        (5)       /* Bits 5-7: Mode Select */
#define MEBI_MODE_MOD_MASK         (7 << MEBI_MODE_MOD_SHIFT)
#  define MEBI_MODE_MODA           (1 << MEBI_MODE_MOD_SHIFT)
#  define MEBI_MODE_MODB           (2 << MEBI_MODE_MOD_SHIFT)
#  define MEBI_MODE_MODC           (4 << MEBI_MODE_MOD_SHIFT)

/* Pull Control Register Bit-Field Definitions */

#define MEBI_PUCR_PUPAE             (1 << 0)  /* Bit 0:  Pull resistors Port A Enable */
#define MEBI_PUCR_PUPBE             (1 << 1)  /* Bit 1:  Pull resistors Port B Enable */
#define MEBI_PUCR_PUPEE             (1 << 4)  /* Bit 4:  Pull resistors Port E Enable */
#define MEBI_PUCR_PUPKE             (1 << 7)  /* Bit 7:  Pull resistors Port K Enable */

/* Reduced Drive Register Bit-Field Definitions */

#define MEBI_RDRIV_RDPA             (1 << 0)  /* Bit 0:  Reduced Drive of Port A */
#define MEBI_RDRIV_RDPB             (1 << 1)  /* Bit 1:  Reduced Drive of Port B */
#define MEBI_RDRIV_RDPE             (1 << 4)  /* Bit 4:  Reduced Drive of Port E */
#define MEBI_RDRIV_RDRK             (1 << 7)  /* Bit 7:  Reduced Drive of Port K */

/* External Bus Interface Control Register Bit-Field Definitions */

#define MEBI_EBICTL_ESTR            (1 << 0)  /* Bit 0:  E Clock Stretches */

/* IRQ Control Register Bit-Field Definitions */

#define MEBI_IRQCR_IRQEN            (1 << 6)  /* Bit 6:  External IRQ Enable */
#define MEBI_IRQCR_IRQE             (1 << 7)  /* Bit 7:  IRQ Select Edge Sensitive Only */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_M9S12_M9S12_MEBI_H */
