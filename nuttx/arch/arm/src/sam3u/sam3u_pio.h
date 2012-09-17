/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_pio.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_PIO_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_PIO_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* PIO register offsets *****************************************************************/

#define SAM3U_PIO_PER_OFFSET         0x0000 /* PIO Enable Register */
#define SAM3U_PIO_PDR_OFFSET         0x0004 /* PIO Disable Register */
#define SAM3U_PIO_PSR_OFFSET         0x0008 /* PIO Status Register */
                                            /* 0x000c: Reserved */
#define SAM3U_PIO_OER_OFFSET         0x0010 /* Output Enable Register */
#define SAM3U_PIO_ODR_OFFSET         0x0014 /* Output Disable Register */
#define SAM3U_PIO_OSR_OFFSET         0x0018 /* utput Status Register */
                                            /* 0x001c: Reserved */
#define SAM3U_PIO_IFER_OFFSET        0x0020 /* Glitch Input Filter Enable Register */
#define SAM3U_PIO_IFDR_OFFSET        0x0024 /* Glitch Input Filter Disable Register */
#define SAM3U_PIO_IFSR_OFFSET        0x0028 /* Glitch Input Filter Status Register */
                                            /* 0x002c: Reserved */
#define SAM3U_PIO_SODR_OFFSET        0x0030 /* Set Output Data Register */
#define SAM3U_PIO_CODR_OFFSET        0x0034 /* Clear Output Data Register */
#define SAM3U_PIO_ODSR_OFFSET        0x0038 /* Output Data Status Register */
#define SAM3U_PIO_PDSR_OFFSET        0x003c /* Pin Data Status Register */
#define SAM3U_PIO_IER_OFFSET         0x0040 /* Interrupt Enable Register */
#define SAM3U_PIO_IDR_OFFSET         0x0044 /* Interrupt Disable Register */
#define SAM3U_PIO_IMR_OFFSET         0x0048 /* Interrupt Mask Register */
#define SAM3U_PIO_ISR_OFFSET         0x004c /* Interrupt Status Register */
#define SAM3U_PIO_MDER_OFFSET        0x0050 /* Multi-driver Enable Register */
#define SAM3U_PIO_MDDR_OFFSET        0x0054 /* Multi-driver Disable Register */
#define SAM3U_PIO_MDSR_OFFSET        0x0058 /* Multi-driver Status Register */
                                            /* 0x005c: Reserved */
#define SAM3U_PIO_PUDR_OFFSET        0x0060 /* Pull-up Disable Register */
#define SAM3U_PIO_PUER_OFFSET        0x0064 /* Pull-up Enable Register */
#define SAM3U_PIO_PUSR_OFFSET        0x0068 /* Pad Pull-up Status Register */
                                            /* 0x006c: Reserved */
#define SAM3U_PIO_ABSR_OFFSET        0x0070 /* Peripheral AB Select Register */
                                            /* 0x0074-0x007c: Reserved */
#define SAM3U_PIO_SCIFSR_OFFSET      0x0080 /* System Clock Glitch Input Filter Select Register */
#define SAM3U_PIO_DIFSR_OFFSET       0x0084 /* Debouncing Input Filter Select Register */
#define SAM3U_PIO_IFDGSR_OFFSET      0x0088 /* Glitch or Debouncing Input Filter Clock Selection Status Register */
#define SAM3U_PIO_SCDR_OFFSET        0x008c /* Slow Clock Divider Debouncing Register */
                                            /* 0x0090-0x009c: Reserved */
#define SAM3U_PIO_OWER_OFFSET        0x00a0 /* Output Write Enable */
#define SAM3U_PIO_OWDR_OFFSET        0x00a4 /* Output Write Disable */
#define SAM3U_PIO_OWSR_OFFSET        0x00a8 /* Output Write Status Register */
                                            /* 0x00ac: Reserved */
#define SAM3U_PIO_AIMER_OFFSET       0x00b0 /* Additional Interrupt Modes Enable Register */
#define SAM3U_PIO_AIMDR_OFFSET       0x00b4 /* Additional Interrupt Modes Disables Register */
#define SAM3U_PIO_AIMMR_OFFSET       0x00b8 /* Additional Interrupt Modes Mask Register */
                                            /* 0x00bc: Reserved */
#define SAM3U_PIO_ESR_OFFSET         0x00c0 /* Edge Select Register */
#define SAM3U_PIO_LSR_OFFSET         0x00c4 /* Level Select Register */
#define SAM3U_PIO_ELSR_OFFSET        0x00c8 /* Edge/Level Status Register */
                                            /* 0x00cc: Reserved */
#define SAM3U_PIO_FELLSR_OFFSET      0x00d0 /* Falling Edge/Low Level Select Register */
#define SAM3U_PIO_REHLSR_OFFSET      0x00d4 /* Rising Edge/ High Level Select Register */
#define SAM3U_PIO_FRLHSR_OFFSET      0x00d8 /* Fall/Rise - Low/High Status Register */
                                            /* 0x00dc: Reserved */
#define SAM3U_PIO_LOCKSR_OFFSET      0x00e0 /* Lock Status */
#define SAM3U_PIO_WPMR_OFFSET        0x00e4 /* Write Protect Mode Register */
#define SAM3U_PIO_WPSR_OFFSET        0x00e8 /* Write Protect Status Register */
                                            /* 0x00ec-0x00f8: Reserved */
                                            /* 0x0100-0x0144: Reserved */

/* PIO register adresses ****************************************************************/

#define PIOA                         (0)
#define PIOB                         (1)
#define PIOC                         (2)
#define NPIO                         (3)

#define SAM3U_PIO_PER(n)             (SAM3U_PIO_BASE(n)+SAM3U_PIO_PER_OFFSET)
#define SAM3U_PIO_PDR(n)             (SAM3U_PIO_BASE(n)+SAM3U_PIO_PDR_OFFSET)
#define SAM3U_PIO_PSR(n)             (SAM3U_PIO_BASE(n)+SAM3U_PIO_PSR_OFFSET)
#define SAM3U_PIO_OER(n)             (SAM3U_PIO_BASE(n)+SAM3U_PIO_OER_OFFSET)
#define SAM3U_PIO_ODR(n)             (SAM3U_PIO_BASE(n)+SAM3U_PIO_ODR_OFFSET)
#define SAM3U_PIO_OSR(n)             (SAM3U_PIO_BASE(n)+SAM3U_PIO_OSR_OFFSET)
#define SAM3U_PIO_IFER(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_IFER_OFFSET)
#define SAM3U_PIO_IFDR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_IFDR_OFFSET)
#define SAM3U_PIO_IFSR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_IFSR_OFFSET)
#define SAM3U_PIO_SODR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_SODR_OFFSET)
#define SAM3U_PIO_CODR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_CODR_OFFSET)
#define SAM3U_PIO_ODSR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_ODSR_OFFSET)
#define SAM3U_PIO_PDSR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_PDSR_OFFSET)
#define SAM3U_PIO_IER(n)             (SAM3U_PIO_BASE(n)+SAM3U_PIO_IER_OFFSET)
#define SAM3U_PIO_IDR(n)             (SAM3U_PIO_BASE(n)+SAM3U_PIO_IDR_OFFSET)
#define SAM3U_PIO_IMR(n)             (SAM3U_PIO_BASE(n)+SAM3U_PIO_IMR_OFFSET)
#define SAM3U_PIO_ISR(n)             (SAM3U_PIO_BASE(n)+SAM3U_PIO_ISR_OFFSET)
#define SAM3U_PIO_MDER(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_MDER_OFFSET)
#define SAM3U_PIO_MDDR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_MDDR_OFFSET)
#define SAM3U_PIO_MDSR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_MDSR_OFFSET)
#define SAM3U_PIO_PUDR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_PUDR_OFFSET)
#define SAM3U_PIO_PUER(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_PUER_OFFSET)
#define SAM3U_PIO_PUSR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_PUSR_OFFSET)
#define SAM3U_PIO_ABSR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_ABSR_OFFSET)
#define SAM3U_PIO_SCIFSR(n)          (SAM3U_PIO_BASE(n)+SAM3U_PIO_SCIFSR_OFFSET)
#define SAM3U_PIO_DIFSR(n)           (SAM3U_PIO_BASE(n)+SAM3U_PIO_DIFSR_OFFSET)
#define SAM3U_PIO_IFDGSR(n)          (SAM3U_PIO_BASE(n)+SAM3U_PIO_IFDGSR_OFFSET)
#define SAM3U_PIO_SCDR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_SCDR_OFFSET)
#define SAM3U_PIO_OWER(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_OWER_OFFSET)
#define SAM3U_PIO_OWDR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_OWDR_OFFSET)
#define SAM3U_PIO_OWSR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_OWSR_OFFSET)
#define SAM3U_PIO_AIMER(n)           (SAM3U_PIO_BASE(n)+SAM3U_PIO_AIMER_OFFSET)
#define SAM3U_PIO_AIMDR(n)           (SAM3U_PIO_BASE(n)+SAM3U_PIO_AIMDR_OFFSET)
#define SAM3U_PIO_AIMMR(n)           (SAM3U_PIO_BASE(n)+SAM3U_PIO_AIMMR_OFFSET)
#define SAM3U_PIO_ESR(n)             (SAM3U_PIO_BASE(n)+SAM3U_PIO_ESR_OFFSET)
#define SAM3U_PIO_LSR(n)             (SAM3U_PIO_BASE(n)+SAM3U_PIO_LSR_OFFSET)
#define SAM3U_PIO_ELSR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_ELSR_OFFSET)
#define SAM3U_PIO_FELLSR(n)          (SAM3U_PIO_BASE(n)+SAM3U_PIO_FELLSR_OFFSET)
#define SAM3U_PIO_REHLSR(n)          (SAM3U_PIO_BASE(n)+SAM3U_PIO_REHLSR_OFFSET)
#define SAM3U_PIO_FRLHSR(n)          (SAM3U_PIO_BASE(n)+SAM3U_PIO_FRLHSR_OFFSET)
#define SAM3U_PIO_LOCKSR(n)          (SAM3U_PIO_BASE(n)+SAM3U_PIO_LOCKSR_OFFSET)
#define SAM3U_PIO_WPMR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_WPMR_OFFSET)
#define SAM3U_PIO_WPSR(n)            (SAM3U_PIO_BASE(n)+SAM3U_PIO_WPSR_OFFSET)

#define SAM3U_PIOA_PER               (SAM3U_PIOA_BASE+SAM3U_PIO_PER_OFFSET)
#define SAM3U_PIOA_PDR_              (SAM3U_PIOA_BASE+SAM3U_PIO_PDR_OFFSET)
#define SAM3U_PIOA_PSR               (SAM3U_PIOA_BASE+SAM3U_PIO_PSR_OFFSET)
#define SAM3U_PIOA_OER               (SAM3U_PIOA_BASE+SAM3U_PIO_OER_OFFSET)
#define SAM3U_PIOA_ODR               (SAM3U_PIOA_BASE+SAM3U_PIO_ODR_OFFSET)
#define SAM3U_PIOA_OSR               (SAM3U_PIOA_BASE+SAM3U_PIO_OSR_OFFSET)
#define SAM3U_PIOA_IFER              (SAM3U_PIOA_BASE+SAM3U_PIO_IFER_OFFSET)
#define SAM3U_PIOA_IFDR              (SAM3U_PIOA_BASE+SAM3U_PIO_IFDR_OFFSET)
#define SAM3U_PIOA_IFSR              (SAM3U_PIOA_BASE+SAM3U_PIO_IFSR_OFFSET)
#define SAM3U_PIOA_SODR              (SAM3U_PIOA_BASE+SAM3U_PIO_SODR_OFFSET)
#define SAM3U_PIOA_CODR              (SAM3U_PIOA_BASE+SAM3U_PIO_CODR_OFFSET)
#define SAM3U_PIOA_ODSR              (SAM3U_PIOA_BASE+SAM3U_PIO_ODSR_OFFSET)
#define SAM3U_PIOA_PDSR              (SAM3U_PIOA_BASE+SAM3U_PIO_PDSR_OFFSET)
#define SAM3U_PIOA_IER               (SAM3U_PIOA_BASE+SAM3U_PIO_IER_OFFSET)
#define SAM3U_PIOA_IDR               (SAM3U_PIOA_BASE+SAM3U_PIO_IDR_OFFSET)
#define SAM3U_PIOA_IMR               (SAM3U_PIOA_BASE+SAM3U_PIO_IMR_OFFSET)
#define SAM3U_PIOA_ISR               (SAM3U_PIOA_BASE+SAM3U_PIO_ISR_OFFSET)
#define SAM3U_PIOA_MDER              (SAM3U_PIOA_BASE+SAM3U_PIO_MDER_OFFSET)
#define SAM3U_PIOA_MDDR              (SAM3U_PIOA_BASE+SAM3U_PIO_MDDR_OFFSET)
#define SAM3U_PIOA_MDSR              (SAM3U_PIOA_BASE+SAM3U_PIO_MDSR_OFFSET)
#define SAM3U_PIOA_PUDR              (SAM3U_PIOA_BASE+SAM3U_PIO_PUDR_OFFSET)
#define SAM3U_PIOA_PUER              (SAM3U_PIOA_BASE+SAM3U_PIO_PUER_OFFSET)
#define SAM3U_PIOA_PUSR              (SAM3U_PIOA_BASE+SAM3U_PIO_PUSR_OFFSET)
#define SAM3U_PIOA_ABSR              (SAM3U_PIOA_BASE+SAM3U_PIO_ABSR_OFFSET)
#define SAM3U_PIOA_SCIFSR            (SAM3U_PIOA_BASE+SAM3U_PIO_SCIFSR_OFFSET)
#define SAM3U_PIOA_DIFSR             (SAM3U_PIOA_BASE+SAM3U_PIO_DIFSR_OFFSET)
#define SAM3U_PIOA_IFDGSR            (SAM3U_PIOA_BASE+SAM3U_PIO_IFDGSR_OFFSET)
#define SAM3U_PIOA_SCDR              (SAM3U_PIOA_BASE+SAM3U_PIO_SCDR_OFFSET)
#define SAM3U_PIOA_OWER              (SAM3U_PIOA_BASE+SAM3U_PIO_OWER_OFFSET)
#define SAM3U_PIOA_OWDR              (SAM3U_PIOA_BASE+SAM3U_PIO_OWDR_OFFSET)
#define SAM3U_PIOA_OWSR              (SAM3U_PIOA_BASE+SAM3U_PIO_OWSR_OFFSET)
#define SAM3U_PIOA_AIMER             (SAM3U_PIOA_BASE+SAM3U_PIO_AIMER_OFFSET)
#define SAM3U_PIOA_AIMDR             (SAM3U_PIOA_BASE+SAM3U_PIO_AIMDR_OFFSET)
#define SAM3U_PIOA_AIMMR             (SAM3U_PIOA_BASE+SAM3U_PIO_AIMMR_OFFSET)
#define SAM3U_PIOA_ESR               (SAM3U_PIOA_BASE+SAM3U_PIO_ESR_OFFSET)
#define SAM3U_PIOA_LSR               (SAM3U_PIOA_BASE+SAM3U_PIO_LSR_OFFSET)
#define SAM3U_PIOA_ELSR              (SAM3U_PIOA_BASE+SAM3U_PIO_ELSR_OFFSET)
#define SAM3U_PIOA_FELLSR            (SAM3U_PIOA_BASE+SAM3U_PIO_FELLSR_OFFSET)
#define SAM3U_PIOA_REHLSR            (SAM3U_PIOA_BASE+SAM3U_PIO_REHLSR_OFFSET)
#define SAM3U_PIOA_FRLHSR            (SAM3U_PIOA_BASE+SAM3U_PIO_FRLHSR_OFFSET)
#define SAM3U_PIOA_LOCKSR            (SAM3U_PIOA_BASE+SAM3U_PIO_LOCKSR_OFFSET)
#define SAM3U_PIOA_WPMR              (SAM3U_PIOA_BASE+SAM3U_PIO_WPMR_OFFSET)
#define SAM3U_PIOA_WPSR              (SAM3U_PIOA_BASE+SAM3U_PIO_WPSR_OFFSET)

#define SAM3U_PIOB_PER               (SAM3U_PIOB_BASE+SAM3U_PIO_PER_OFFSET)
#define SAM3U_PIOB_PDR_              (SAM3U_PIOB_BASE+SAM3U_PIO_PDR_OFFSET)
#define SAM3U_PIOB_PSR               (SAM3U_PIOB_BASE+SAM3U_PIO_PSR_OFFSET)
#define SAM3U_PIOB_OER               (SAM3U_PIOB_BASE+SAM3U_PIO_OER_OFFSET)
#define SAM3U_PIOB_ODR               (SAM3U_PIOB_BASE+SAM3U_PIO_ODR_OFFSET)
#define SAM3U_PIOB_OSR               (SAM3U_PIOB_BASE+SAM3U_PIO_OSR_OFFSET)
#define SAM3U_PIOB_IFER              (SAM3U_PIOB_BASE+SAM3U_PIO_IFER_OFFSET)
#define SAM3U_PIOB_IFDR              (SAM3U_PIOB_BASE+SAM3U_PIO_IFDR_OFFSET)
#define SAM3U_PIOB_IFSR              (SAM3U_PIOB_BASE+SAM3U_PIO_IFSR_OFFSET)
#define SAM3U_PIOB_SODR              (SAM3U_PIOB_BASE+SAM3U_PIO_SODR_OFFSET)
#define SAM3U_PIOB_CODR              (SAM3U_PIOB_BASE+SAM3U_PIO_CODR_OFFSET)
#define SAM3U_PIOB_ODSR              (SAM3U_PIOB_BASE+SAM3U_PIO_ODSR_OFFSET)
#define SAM3U_PIOB_PDSR              (SAM3U_PIOB_BASE+SAM3U_PIO_PDSR_OFFSET)
#define SAM3U_PIOB_IER               (SAM3U_PIOB_BASE+SAM3U_PIO_IER_OFFSET)
#define SAM3U_PIOB_IDR               (SAM3U_PIOB_BASE+SAM3U_PIO_IDR_OFFSET)
#define SAM3U_PIOB_IMR               (SAM3U_PIOB_BASE+SAM3U_PIO_IMR_OFFSET)
#define SAM3U_PIOB_ISR               (SAM3U_PIOB_BASE+SAM3U_PIO_ISR_OFFSET)
#define SAM3U_PIOB_MDER              (SAM3U_PIOB_BASE+SAM3U_PIO_MDER_OFFSET)
#define SAM3U_PIOB_MDDR              (SAM3U_PIOB_BASE+SAM3U_PIO_MDDR_OFFSET)
#define SAM3U_PIOB_MDSR              (SAM3U_PIOB_BASE+SAM3U_PIO_MDSR_OFFSET)
#define SAM3U_PIOB_PUDR              (SAM3U_PIOB_BASE+SAM3U_PIO_PUDR_OFFSET)
#define SAM3U_PIOB_PUER              (SAM3U_PIOB_BASE+SAM3U_PIO_PUER_OFFSET)
#define SAM3U_PIOB_PUSR              (SAM3U_PIOB_BASE+SAM3U_PIO_PUSR_OFFSET)
#define SAM3U_PIOB_ABSR              (SAM3U_PIOB_BASE+SAM3U_PIO_ABSR_OFFSET)
#define SAM3U_PIOB_SCIFSR            (SAM3U_PIOB_BASE+SAM3U_PIO_SCIFSR_OFFSET)
#define SAM3U_PIOB_DIFSR             (SAM3U_PIOB_BASE+SAM3U_PIO_DIFSR_OFFSET)
#define SAM3U_PIOB_IFDGSR            (SAM3U_PIOB_BASE+SAM3U_PIO_IFDGSR_OFFSET)
#define SAM3U_PIOB_SCDR              (SAM3U_PIOB_BASE+SAM3U_PIO_SCDR_OFFSET)
#define SAM3U_PIOB_OWER              (SAM3U_PIOB_BASE+SAM3U_PIO_OWER_OFFSET)
#define SAM3U_PIOB_OWDR              (SAM3U_PIOB_BASE+SAM3U_PIO_OWDR_OFFSET)
#define SAM3U_PIOB_OWSR              (SAM3U_PIOB_BASE+SAM3U_PIO_OWSR_OFFSET)
#define SAM3U_PIOB_AIMER             (SAM3U_PIOB_BASE+SAM3U_PIO_AIMER_OFFSET)
#define SAM3U_PIOB_AIMDR             (SAM3U_PIOB_BASE+SAM3U_PIO_AIMDR_OFFSET)
#define SAM3U_PIOB_AIMMR             (SAM3U_PIOB_BASE+SAM3U_PIO_AIMMR_OFFSET)
#define SAM3U_PIOB_ESR               (SAM3U_PIOB_BASE+SAM3U_PIO_ESR_OFFSET)
#define SAM3U_PIOB_LSR               (SAM3U_PIOB_BASE+SAM3U_PIO_LSR_OFFSET)
#define SAM3U_PIOB_ELSR              (SAM3U_PIOB_BASE+SAM3U_PIO_ELSR_OFFSET)
#define SAM3U_PIOB_FELLSR            (SAM3U_PIOB_BASE+SAM3U_PIO_FELLSR_OFFSET)
#define SAM3U_PIOB_REHLSR            (SAM3U_PIOB_BASE+SAM3U_PIO_REHLSR_OFFSET)
#define SAM3U_PIOB_FRLHSR            (SAM3U_PIOB_BASE+SAM3U_PIO_FRLHSR_OFFSET)
#define SAM3U_PIOB_LOCKSR            (SAM3U_PIOB_BASE+SAM3U_PIO_LOCKSR_OFFSET)
#define SAM3U_PIOB_WPMR              (SAM3U_PIOB_BASE+SAM3U_PIO_WPMR_OFFSET)
#define SAM3U_PIOB_WPSR              (SAM3U_PIOB_BASE+SAM3U_PIO_WPSR_OFFSET)

#define SAM3U_PIOC_PER               (SAM3U_PIOC_BASE+SAM3U_PIO_PER_OFFSET)
#define SAM3U_PIOC_PDR_              (SAM3U_PIOC_BASE+SAM3U_PIO_PDR_OFFSET)
#define SAM3U_PIOC_PSR               (SAM3U_PIOC_BASE+SAM3U_PIO_PSR_OFFSET)
#define SAM3U_PIOC_OER               (SAM3U_PIOC_BASE+SAM3U_PIO_OER_OFFSET)
#define SAM3U_PIOC_ODR               (SAM3U_PIOC_BASE+SAM3U_PIO_ODR_OFFSET)
#define SAM3U_PIOC_OSR               (SAM3U_PIOC_BASE+SAM3U_PIO_OSR_OFFSET)
#define SAM3U_PIOC_IFER              (SAM3U_PIOC_BASE+SAM3U_PIO_IFER_OFFSET)
#define SAM3U_PIOC_IFDR              (SAM3U_PIOC_BASE+SAM3U_PIO_IFDR_OFFSET)
#define SAM3U_PIOC_IFSR              (SAM3U_PIOC_BASE+SAM3U_PIO_IFSR_OFFSET)
#define SAM3U_PIOC_SODR              (SAM3U_PIOC_BASE+SAM3U_PIO_SODR_OFFSET)
#define SAM3U_PIOC_CODR              (SAM3U_PIOC_BASE+SAM3U_PIO_CODR_OFFSET)
#define SAM3U_PIOC_ODSR              (SAM3U_PIOC_BASE+SAM3U_PIO_ODSR_OFFSET)
#define SAM3U_PIOC_PDSR              (SAM3U_PIOC_BASE+SAM3U_PIO_PDSR_OFFSET)
#define SAM3U_PIOC_IER               (SAM3U_PIOC_BASE+SAM3U_PIO_IER_OFFSET)
#define SAM3U_PIOC_IDR               (SAM3U_PIOC_BASE+SAM3U_PIO_IDR_OFFSET)
#define SAM3U_PIOC_IMR               (SAM3U_PIOC_BASE+SAM3U_PIO_IMR_OFFSET)
#define SAM3U_PIOC_ISR               (SAM3U_PIOC_BASE+SAM3U_PIO_ISR_OFFSET)
#define SAM3U_PIOC_MDER              (SAM3U_PIOC_BASE+SAM3U_PIO_MDER_OFFSET)
#define SAM3U_PIOC_MDDR              (SAM3U_PIOC_BASE+SAM3U_PIO_MDDR_OFFSET)
#define SAM3U_PIOC_MDSR              (SAM3U_PIOC_BASE+SAM3U_PIO_MDSR_OFFSET)
#define SAM3U_PIOC_PUDR              (SAM3U_PIOC_BASE+SAM3U_PIO_PUDR_OFFSET)
#define SAM3U_PIOC_PUER              (SAM3U_PIOC_BASE+SAM3U_PIO_PUER_OFFSET)
#define SAM3U_PIOC_PUSR              (SAM3U_PIOC_BASE+SAM3U_PIO_PUSR_OFFSET)
#define SAM3U_PIOC_ABSR              (SAM3U_PIOC_BASE+SAM3U_PIO_ABSR_OFFSET)
#define SAM3U_PIOC_SCIFSR            (SAM3U_PIOC_BASE+SAM3U_PIO_SCIFSR_OFFSET)
#define SAM3U_PIOC_DIFSR             (SAM3U_PIOC_BASE+SAM3U_PIO_DIFSR_OFFSET)
#define SAM3U_PIOC_IFDGSR            (SAM3U_PIOC_BASE+SAM3U_PIO_IFDGSR_OFFSET)
#define SAM3U_PIOC_SCDR              (SAM3U_PIOC_BASE+SAM3U_PIO_SCDR_OFFSET)
#define SAM3U_PIOC_OWER              (SAM3U_PIOC_BASE+SAM3U_PIO_OWER_OFFSET)
#define SAM3U_PIOC_OWDR              (SAM3U_PIOC_BASE+SAM3U_PIO_OWDR_OFFSET)
#define SAM3U_PIOC_OWSR              (SAM3U_PIOC_BASE+SAM3U_PIO_OWSR_OFFSET)
#define SAM3U_PIOC_AIMER             (SAM3U_PIOC_BASE+SAM3U_PIO_AIMER_OFFSET)
#define SAM3U_PIOC_AIMDR             (SAM3U_PIOC_BASE+SAM3U_PIO_AIMDR_OFFSET)
#define SAM3U_PIOC_AIMMR             (SAM3U_PIOC_BASE+SAM3U_PIO_AIMMR_OFFSET)
#define SAM3U_PIOC_ESR               (SAM3U_PIOC_BASE+SAM3U_PIO_ESR_OFFSET)
#define SAM3U_PIOC_LSR               (SAM3U_PIOC_BASE+SAM3U_PIO_LSR_OFFSET)
#define SAM3U_PIOC_ELSR              (SAM3U_PIOC_BASE+SAM3U_PIO_ELSR_OFFSET)
#define SAM3U_PIOC_FELLSR            (SAM3U_PIOC_BASE+SAM3U_PIO_FELLSR_OFFSET)
#define SAM3U_PIOC_REHLSR            (SAM3U_PIOC_BASE+SAM3U_PIO_REHLSR_OFFSET)
#define SAM3U_PIOC_FRLHSR            (SAM3U_PIOC_BASE+SAM3U_PIO_FRLHSR_OFFSET)
#define SAM3U_PIOC_LOCKSR            (SAM3U_PIOC_BASE+SAM3U_PIO_LOCKSR_OFFSET)
#define SAM3U_PIOC_WPMR              (SAM3U_PIOC_BASE+SAM3U_PIO_WPMR_OFFSET)
#define SAM3U_PIOC_WPSR              (SAM3U_PIOC_BASE+SAM3U_PIO_WPSR_OFFSET)

/* PIO register bit definitions *********************************************************/

/* Common bit definitions for ALMOST all IO registers (exceptions follow) */

#define PIO(n)                       (1<<(n)) /* Bit n: PIO n */

/* PIO Write Protect Mode Register */

#define PIO_WPMR_WPEN               (1 << 0)  /* Bit 0:  Write Protect Enable */
#define PIO_WPMR_WPKEY_SHIFT        (8)       /* Bits 8-31: Write Protect KEY */
#define PIO_WPMR_WPKEY_MASK         (0xffffff << PIO_WPMR_WPKEY_SHIFT)

/* PIO Write Protect Status Register */

#define PIO_WPSR_WPVS               (1 << 0)  /* Bit 0:  Write Protect Violation Status */
#define PIO_WPSR_WPVSRC_SHIFT       (8)       /* Bits 8-23: Write Protect Violation Source */
#define PIO_WPSR_WPVSRC_MASK        (0xffff << PIO_WPSR_WPVSRC_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_PIO_H */
