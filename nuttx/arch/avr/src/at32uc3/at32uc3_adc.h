/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_adc.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_ADC_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_ADC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_ADC_CR_OFFSET       0x000 /* Control Register */
#define AVR32_ADC_MR_OFFSET       0x004 /* Mode Register */
#define AVR32_ADC_CHER_OFFSET     0x010 /* Channel Enable Register */
#define AVR32_ADC_CHDR_OFFSET     0x014 /* Channel Disable Register */
#define AVR32_ADC_CHSR_OFFSET     0x018 /* Channel Status Register */
#define AVR32_ADC_SR_OFFSET       0x01c /* Status Register */
#define AVR32_ADC_LCDR_OFFSET     0x020 /* Last Converted Data Register */
#define AVR32_ADC_IER_OFFSET      0x024 /* Interrupt Enable Register */
#define AVR32_ADC_IDR_OFFSET      0x028 /* Interrupt Disable Register */
#define AVR32_ADC_IMR_OFFSET      0x02c /* Interrupt Mask Register */
#define AVR32_ADC_CDR_OFFSET(n)   (0x030+((n)<<2))
#define AVR32_ADC_CDR0_OFFSET     0x030 /* Channel Data Register 0 */
#define AVR32_ADC_CDR1_OFFSET     0x034 /* Channel Data Register 1 */
#define AVR32_ADC_CDR2_OFFSET     0x038 /* Channel Data Register 2 */
#define AVR32_ADC_CDR3_OFFSET     0x03c /* Channel Data Register 3 */
#define AVR32_ADC_CDR4_OFFSET     0x040 /* Channel Data Register 4 */
#define AVR32_ADC_CDR5_OFFSET     0x044 /* Channel Data Register 5 */
#define AVR32_ADC_CDR6_OFFSET     0x048 /* Channel Data Register 6 */
#define AVR32_ADC_CDR7_OFFSET     0x04c /* Channel Data Register 7 */
#define AVR32_ADC_VERSION_OFFSET  0x0fc /* Version Register */

/* Register Addresses ***************************************************************/

#define AVR32_ADC_CR              (AVR32_ADC_BASE+AVR32_ADC_CR_OFFSET)
#define AVR32_ADC_MR              (AVR32_ADC_BASE+AVR32_ADC_MR_OFFSET)
#define AVR32_ADC_CHER            (AVR32_ADC_BASE+AVR32_ADC_CHER_OFFSET)
#define AVR32_ADC_CHDR            (AVR32_ADC_BASE+AVR32_ADC_CHDR_OFFSET)
#define AVR32_ADC_CHSR            (AVR32_ADC_BASE+AVR32_ADC_CHSR_OFFSET)
#define AVR32_ADC_SR              (AVR32_ADC_BASE+AVR32_ADC_SR_OFFSET)
#define AVR32_ADC_LCDR            (AVR32_ADC_BASE+AVR32_ADC_LCDR_OFFSET)
#define AVR32_ADC_IER             (AVR32_ADC_BASE+AVR32_ADC_IER_OFFSET)
#define AVR32_ADC_IDR             (AVR32_ADC_BASE+AVR32_ADC_IDR_OFFSET)
#define AVR32_ADC_IMR             (AVR32_ADC_BASE+AVR32_ADC_IMR_OFFSET)
#define AVR32_ADC_CDR(n)          (AVR32_ADC_BASE+AVR32_ADC_CDR_OFFSET(n))
#define AVR32_ADC_CDR0            (AVR32_ADC_BASE+AVR32_ADC_CDR0_OFFSET)
#define AVR32_ADC_CDR1            (AVR32_ADC_BASE+AVR32_ADC_CDR1_OFFSET)
#define AVR32_ADC_CDR2            (AVR32_ADC_BASE+AVR32_ADC_CDR2_OFFSET)
#define AVR32_ADC_CDR3            (AVR32_ADC_BASE+AVR32_ADC_CDR3_OFFSET)
#define AVR32_ADC_CDR4            (AVR32_ADC_BASE+AVR32_ADC_CDR4_OFFSET)
#define AVR32_ADC_CDR5            (AVR32_ADC_BASE+AVR32_ADC_CDR5_OFFSET)
#define AVR32_ADC_CDR6            (AVR32_ADC_BASE+AVR32_ADC_CDR6_OFFSET)
#define AVR32_ADC_CDR7            (AVR32_ADC_BASE+AVR32_ADC_CDR7_OFFSET)
#define AVR32_ADC_VERSION         (AVR32_ADC_BASE+AVR32_ADC_VERSION_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Control Register Bit-field Definitions */

#define ADC_CR_SWRST              (1 << 0)  /* Bit 0: Software Reset */
#define ADC_CR_START              (1 << 1)  /* Bit 1: Start Conversion */

/* Mode Register Bit-field Definitions */

#define ADC_MR_TRGEN              (1 << 0)  /* Bit 0: Trigger Enable */
#define ADC_MR_TRGSEL_SHIFT       (1)       /* Bits 1-3: Trigger Selection */
#define ADC_MR_TRGSEL_MASK        (7 << ADC_MR_TRGSEL_SHIFT)
# define ADC_MR_TRGSEL_TRIG(n)    ((n) << ADC_MR_TRGSEL_SHIFT) /* Internal trigger n */
# define ADC_MR_TRGSEL_TRIG0      (0 << ADC_MR_TRGSEL_SHIFT) /* Internal trigger 0 */
# define ADC_MR_TRGSEL_TRIG1      (1 << ADC_MR_TRGSEL_SHIFT) /* Internal trigger 1 */
# define ADC_MR_TRGSEL_TRIG2      (2 << ADC_MR_TRGSEL_SHIFT) /* Internal trigger 2 */
# define ADC_MR_TRGSEL_TRIG3      (3 << ADC_MR_TRGSEL_SHIFT) /* Internal trigger 3 */
# define ADC_MR_TRGSEL_TRIG4      (4 << ADC_MR_TRGSEL_SHIFT) /* Internal trigger 4 */
# define ADC_MR_TRGSEL_TRIG5      (5 << ADC_MR_TRGSEL_SHIFT) /* Internal trigger 5 */
# define ADC_MR_TRGSEL_TRIG6      (6 << ADC_MR_TRGSEL_SHIFT) /* Internal trigger 6 */
# define ADC_MR_TRGSEL_EXT        (7 << ADC_MR_TRGSEL_SHIFT) /* External trigger */
#define ADC_MR_LOWRES             (1 << 4)  /* Bit 4: Resolution */
#define ADC_MR_SLEEP              (1 << 5)  /* Bit 5: Sleep Mode */
#define ADC_MR_PRESCAL_SHIFT      (8)       /* Bits 8-15: Prescaler Rate Selection */
#define ADC_MR_PRESCAL_MASK       (0xff << ADC_MR_PRESCAL_SHIFT)
#define ADC_MR_STARTUP_SHIFT      (16)      /* Bits 16-22: Start Up Time */
#define ADC_MR_STARTUP_MASK       (0x7f << ADC_MR_STARTUP_SHIFT)
#define ADC_MR_SHTIM_SHIFT        (24)       /* Bits 24-27 Sample & Hold Time */
#define ADC_MR_SHTIM_MASK         (15 << ADC_MR_SHTIM_SHIFT)

/* Channel Enable Register Bit-field Definitions */
/* Channel Disable Register Bit-field Definitions */
/* Channel Status Register Bit-field Definitions */

#define ADC_CHAN(n)               (1 << (n))
#define ADC_CHAN0                 (1 << 0)
#define ADC_CHAN1                 (1 << 1)
#define ADC_CHAN2                 (1 << 2)
#define ADC_CHAN3                 (1 << 3)
#define ADC_CHAN4                 (1 << 4)
#define ADC_CHAN5                 (1 << 5)
#define ADC_CHAN6                 (1 << 6)
#define ADC_CHAN7                 (1 << 7)

/* Status Register Bit-field Definitions */
/* Interrupt Enable Register Bit-field Definitions */
/* Interrupt Disable Register Bit-field Definitions */
/* Interrupt Mask Register Bit-field Definitions */

#define ADC_INT_EOC(n)            (1 << (n))
#define ADC_INT_EOC0              (1 << 0)  /* Bit 0:  End of Conversion 0 */
#define ADC_INT_EOC1              (1 << 1)  /* Bit 1:  End of Conversion 1 */
#define ADC_INT_EOC2              (1 << 2)  /* Bit 2:  End of Conversion 2 */
#define ADC_INT_EOC3              (1 << 3)  /* Bit 3:  End of Conversion 3 */
#define ADC_INT_EOC4              (1 << 4)  /* Bit 4:  End of Conversion 4 */
#define ADC_INT_EOC5              (1 << 5)  /* Bit 5:  End of Conversion 5 */
#define ADC_INT_EOC6              (1 << 6)  /* Bit 6:  End of Conversion 6 */
#define ADC_INT_EOC7              (1 << 7)  /* Bit 7:  End of Conversion 7 */
#define ADC_INT_OVRE(n)           (1 << ((n)+8))
#define ADC_INT_OVRE0             (1 << 8)  /* Bit 8:  Overrun Error 0 */
#define ADC_INT_OVRE1             (1 << 9)  /* Bit 9:  Overrun Error 1 */
#define ADC_INT_OVRE2             (1 << 10) /* Bit 10: Overrun Error 2 */
#define ADC_INT_OVRE3             (1 << 11) /* Bit 11: Overrun Error 3 */
#define ADC_INT_OVRE4             (1 << 12) /* Bit 12: Overrun Error 4 */
#define ADC_INT_OVRE5             (1 << 13) /* Bit 13: Overrun Error 5 */
#define ADC_INT_OVRE6             (1 << 14) /* Bit 14: Overrun Error 6 */
#define ADC_INT_OVRE7             (1 << 15) /* Bit 15: Overrun Error 7 */
#define ADC_INT_DRDY              (1 << 16) /* Bit 16: Data Ready */
#define ADC_INT_GOVRE             (1 << 17) /* Bit 17: General Overrun Error */
#define ADC_INT_ENDRX             (1 << 18) /* Bit 18: End of RX Buffer */
#define ADC_INT_RXBUFF            (1 << 19) /* Bit 19: RX Buffer Full */

/* Last Converted Data Register Bit-field Definitions */

#define ADC_LCDR_MASK             (0x3ff)

/* Channel Data Registers 0-7 Bit-field Definitions */

#define ADC_CDR_MASK              (0x3ff)

/* Version Register Bit-field Definitions */

#define ADC_VERSION_SHIFT         (0)       /* Bits 0-11: Version Number */
#define ADC_VERSION_MASK          (0xfff << ADC_VERSION_SHIFT)
#define ADC_VERSION_VARIANT_SHIFT (16)      /* Bits 16-19: Variant Number */
#define ADC_VERSION_VARIANT_MASK  (15 << ADC_VERSION_VARIANT_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_ADC_H */

