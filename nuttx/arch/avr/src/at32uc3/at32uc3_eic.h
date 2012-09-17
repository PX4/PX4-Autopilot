/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_eic.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_EIC_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_EIC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_EIC_IER_OFFSET     0x000 /* Interrupt Enable Register */
#define AVR32_EIC_IDR_OFFSET     0x004 /* Interrupt Disable Register */
#define AVR32_EIC_IMR_OFFSET     0x008 /* Interrupt Mask Register */
#define AVR32_EIC_ISR_OFFSET     0x00c /* Interrupt Status Register */
#define AVR32_EIC_ICR_OFFSET     0x010 /* Interrupt Clear Register */
#define AVR32_EIC_MODE_OFFSET    0x014 /* Mode Register */
#define AVR32_EIC_EDGE_OFFSET    0x018 /* Edge Register */
#define AVR32_EIC_LEVEL_OFFSET   0x01c /* Level Register */
#define AVR32_EIC_FILTER_OFFSET  0x020 /* Filter Register */
#define AVR32_EIC_TEST_OFFSET    0x024 /* Test Register */
#define AVR32_EIC_ASYNC_OFFSET   0x028 /* Asynchronous Register */
#define AVR32_EIC_SCAN_OFFSET    0x02c /* Scan Register */
#define AVR32_EIC_EN_OFFSET      0x030 /* Enable Register */
#define AVR32_EIC_DIS_OFFSET     0x034 /* Disable Register */
#define AVR32_EIC_CTRL_OFFSET    0x038 /* Control Register */

/* Register Addresses ***************************************************************/

#define AVR32_EIC_IER           (AVR32_EIC_BASE+AVR32_EIC_IER_OFFSET)
#define AVR32_EIC_IDR           (AVR32_EIC_BASE+AVR32_EIC_IDR_OFFSET)
#define AVR32_EIC_IMR           (AVR32_EIC_BASE+AVR32_EIC_IMR_OFFSET)
#define AVR32_EIC_ISR           (AVR32_EIC_BASE+AVR32_EIC_ISR_OFFSET)
#define AVR32_EIC_ICR           (AVR32_EIC_BASE+AVR32_EIC_ICR_OFFSET)
#define AVR32_EIC_MODE          (AVR32_EIC_BASE+AVR32_EIC_MODE_OFFSET)
#define AVR32_EIC_EDGE          (AVR32_EIC_BASE+AVR32_EIC_EDGE_OFFSET)
#define AVR32_EIC_LEVEL         (AVR32_EIC_BASE+AVR32_EIC_LEVEL_OFFSET)
#define AVR32_EIC_FILTER        (AVR32_EIC_BASE+AVR32_EIC_FILTER_OFFSET)
#define AVR32_EIC_TEST          (AVR32_EIC_BASE+AVR32_EIC_TEST_OFFSET)
#define AVR32_EIC_ASYNC         (AVR32_EIC_BASE+AVR32_EIC_ASYNC_OFFSET)
#define AVR32_EIC_SCAN          (AVR32_EIC_BASE+AVR32_EIC_SCAN_OFFSET)
#define AVR32_EIC_EN            (AVR32_EIC_BASE+AVR32_EIC_EN_OFFSET)
#define AVR32_EIC_DIS           (AVR32_EIC_BASE+AVR32_EIC_DIS_OFFSET)
#define AVR32_EIC_CTRL          (AVR32_EIC_BASE+AVR32_EIC_CTRL_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Interrupt Enable Register Bit-field Definitions */
/* Interrupt Disable Register Bit-field Definitions */
/* Interrupt Mask Register Bit-field Definitions */
/* Interrupt Status Register Bit-field Definitions */
/* Interrupt Clear Register Bit-field Definitions */
/* Mode Register Bit-field Definitions */
/* Edge Register Bit-field Definitions */
/* Level Register Bit-field Definitions */
/* Filter Register Bit-field Definitions */
/* Test Register Bit-field Definitions */
/* Asynchronous Register Bit-field Definitions */
/* Enable Register Bit-field Definitions */
/* Disable Register Bit-field Definitions */
/* Control Register Bit-field Definitions */

#define EIC_INT0                (1 << 0)  /* Bit 0: External interrupt 0 */
#define EIC_INT1                (1 << 1)  /* Bit 1: External interrupt 1 */
#define EIC_INT2                (1 << 2)  /* Bit 2: External interrupt 2 */
#define EIC_INT3                (1 << 3)  /* Bit 3: External interrupt 3 */
#define EIC_INT4                (1 << 4)  /* Bit 4: External interrupt 4 */
#define EIC_INT5                (1 << 5)  /* Bit 5: External interrupt 5 */
#define EIC_INT6                (1 << 6)  /* Bit 6: External interrupt 6 */
#define EIC_INT7                (1 << 7)  /* Bit 7: External interrupt 7 */
#define EIC_NMI                 (1 << 8)  /* Bit 8: NMI */

/* Scan Register Bit-field Definitions */

#define EIC_SCAN_PIN_SHIFT      (24)      /* Bit 24-26: Currently active scan pin */
#define EIC_SCAN_PIN_MASK       (7 << EIC_SCAN_PIN_SHIFT)
#define EIC_SCAN_PRESC_SHIFT    (8)       /* Bit 8-12: Prescale select for keypad scan rate */
#define EIC_SCAN_PRESC_MASK     (0x1f << EIC_SCAN_PRESC_SHIFT)
#define EIC_SCAN_EN             (1 << 0) /* Bit 0: Enable keypad scanning */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_EIC_H */

