/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_adc.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_ADC_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_ADC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* ADC register offsets ****************************************************************/

#define SAM3U_ADC_CR_OFFSET          0x00 /* Control Register (Both) */
#define SAM3U_ADC_MR_OFFSET          0x04 /* Mode Register (Both) */
                                          /* 0x08: Reserved */
                                          /* 0x0c: Reserved */
#define SAM3U_ADC_CHER_OFFSET        0x10 /* Channel Enable Register (Both) */
#define SAM3U_ADC_CHDR_OFFSET        0x14 /* Channel Disable Register (Both) */
#define SAM3U_ADC_CHSR_OFFSET        0x18 /* Channel Status Register (Both) */
#define SAM3U_ADC_SR_OFFSET          0x1c /* Status Register (Both) */
#define SAM3U_ADC_LCDR_OFFSET        0x20 /* Last Converted Data Register (Both) */
#define SAM3U_ADC_IER_OFFSET         0x24 /* Interrupt Enable Register (Both) */
#define SAM3U_ADC_IDR_OFFSET         0x28 /* Interrupt Disable Register (Both) */
#define SAM3U_ADC_IMR_OFFSET         0x2c /* Interrupt Mask Register (Both) */
#define SAM3U_ADC_CDR_OFFSET(n)      (0x30+((n)<<2))
#define SAM3U_ADC_CDR0_OFFSET        0x30 /* Channel Data Register 0 (Both) */
#define SAM3U_ADC_CDR1_OFFSET        0x34 /* Channel Data Register 1 (Both) */
#define SAM3U_ADC_CDR2_OFFSET        0x38 /* Channel Data Register 2 (Both) */
#define SAM3U_ADC_CDR3_OFFSET        0x3c /* Channel Data Register 3 (Both) */
#define SAM3U_ADC_CDR4_OFFSET        0x40 /* Channel Data Register 4 (Both) */
#define SAM3U_ADC_CDR5_OFFSET        0x44 /* Channel Data Register 5 (Both) */
#define SAM3U_ADC_CDR6_OFFSET        0x48 /* Channel Data Register 6 (Both) */
#define SAM3U_ADC_CDR7_OFFSET        0x4c /* Channel Data Register 7 (Both) */
#define SAM3U_ADC12B_ACR_OFFSET      0x64 /* Analog Control Register (ADC12B only) */
#define SAM3U_ADC12B_EMR_OFFSET      0x68 /* Extended Mode Register (ADC12B only) */

/* ADC register adresses ***************************************************************/

#define SAM3U_ADC12B_CR              (SAM3U_ADC12B_BASE+SAM3U_ADC_CR_OFFSET)
#define SAM3U_ADC12B_MR              (SAM3U_ADC12B_BASE+SAM3U_ADC_MR_OFFSET)
#define SAM3U_ADC12B_CHER            (SAM3U_ADC12B_BASE+SAM3U_ADC_CHER_OFFSET)
#define SAM3U_ADC12B_CHDR            (SAM3U_ADC12B_BASE+SAM3U_ADC_CHDR_OFFSET)
#define SAM3U_ADC12B_CHSR            (SAM3U_ADC12B_BASE+SAM3U_ADC_CHSR_OFFSET)
#define SAM3U_ADC12B_SR              (SAM3U_ADC12B_BASE+SAM3U_ADC_SR_OFFSET)
#define SAM3U_ADC12B_LCDR_           (SAM3U_ADC12B_BASE+SAM3U_ADC_LCDR_OFFSET)
#define SAM3U_ADC12B_IER             (SAM3U_ADC12B_BASE+SAM3U_ADC_IER_OFFSET)
#define SAM3U_ADC12B_IDR             (SAM3U_ADC12B_BASE+SAM3U_ADC_IDR_OFFSET)
#define SAM3U_ADC12B_IMR             (SAM3U_ADC12B_BASE+SAM3U_ADC_IMR_OFFSET)
#define SAM3U_ADC12B_CDR(n))         (SAM3U_ADC12B_BASE+SAM3U_ADC_CDR_OFFSET(n))
#define SAM3U_ADC12B_CDR0            (SAM3U_ADC12B_BASE+SAM3U_ADC_CDR0_OFFSET)
#define SAM3U_ADC12B_CDR1            (SAM3U_ADC12B_BASE+SAM3U_ADC_CDR1_OFFSET)
#define SAM3U_ADC12B_CDR2            (SAM3U_ADC12B_BASE+SAM3U_ADC_CDR2_OFFSET)
#define SAM3U_ADC12B_CDR3            (SAM3U_ADC12B_BASE+SAM3U_ADC_CDR3_OFFSET)
#define SAM3U_ADC12B_CDR4            (SAM3U_ADC12B_BASE+SAM3U_ADC_CDR4_OFFSET)
#define SAM3U_ADC12B_CDR5            (SAM3U_ADC12B_BASE+SAM3U_ADC_CDR5_OFFSET)
#define SAM3U_ADC12B_CDR6            (SAM3U_ADC12B_BASE+SAM3U_ADC_CDR6_OFFSET)
#define SAM3U_ADC12B_CDR7            (SAM3U_ADC12B_BASE+SAM3U_ADC_CDR7_OFFSET)
#define SAM3U_ADC12B_ACR             (SAM3U_ADC12B_BASE+SAM3U_ADC12B_ACR_OFFSET)
#define SAM3U_ADC12B_EMR             (SAM3U_ADC12B_BASE+SAM3U_ADC12B_EMR_OFFSET)

#define SAM3U_ADC_CR                 (SAM3U_ADC_BASE+SAM3U_ADC_CR_OFFSET)
#define SAM3U_ADC_MR                 (SAM3U_ADC_BASE+SAM3U_ADC_MR_OFFSET)
#define SAM3U_ADC_CHER               (SAM3U_ADC_BASE+SAM3U_ADC_CHER_OFFSET)
#define SAM3U_ADC_CHDR               (SAM3U_ADC_BASE+SAM3U_ADC_CHDR_OFFSET)
#define SAM3U_ADC_CHSR               (SAM3U_ADC_BASE+SAM3U_ADC_CHSR_OFFSET)
#define SAM3U_ADC_SR                 (SAM3U_ADC_BASE+SAM3U_ADC_SR_OFFSET)
#define SAM3U_ADC_LCDR               (SAM3U_ADC_BASE+SAM3U_ADC_LCDR_OFFSET)
#define SAM3U_ADC_IER                (SAM3U_ADC_BASE+SAM3U_ADC_IER_OFFSET)
#define SAM3U_ADC_IDR                (SAM3U_ADC_BASE+SAM3U_ADC_IDR_OFFSET)
#define SAM3U_ADC_IMR                (SAM3U_ADC_BASE+SAM3U_ADC_IMR_OFFSET)
#define SAM3U_ADC_CDR(n))            (SAM3U_ADC_BASE+SAM3U_ADC_CDR_OFFSET(n))
#define SAM3U_ADC_CDR0               (SAM3U_ADC_BASE+SAM3U_ADC_CDR0_OFFSET)
#define SAM3U_ADC_CDR1               (SAM3U_ADC_BASE+SAM3U_ADC_CDR1_OFFSET)
#define SAM3U_ADC_CDR2               (SAM3U_ADC_BASE+SAM3U_ADC_CDR2_OFFSET)
#define SAM3U_ADC_CDR3               (SAM3U_ADC_BASE+SAM3U_ADC_CDR3_OFFSET)
#define SAM3U_ADC_CDR4               (SAM3U_ADC_BASE+SAM3U_ADC_CDR4_OFFSET)
#define SAM3U_ADC_CDR5               (SAM3U_ADC_BASE+SAM3U_ADC_CDR5_OFFSET)
#define SAM3U_ADC_CDR6               (SAM3U_ADC_BASE+SAM3U_ADC_CDR6_OFFSET)
#define SAM3U_ADC_CDR7               (SAM3U_ADC_BASE+SAM3U_ADC_CDR7_OFFSET)

/* ADC register bit definitions ********************************************************/

/* ADC12B Control Register and ADC(10B) Control Register common bit-field definitions */

#define ADC_CR_SWRST                 (1 << 0)  /* Bit 0:  Software Reset */
#define ADC_CR_START                 (1 << 1)  /* Bit 1:  Start Conversion */

/* ADC12B Mode Register and ADC(10B) Mode Register common bit-field definitions */

#define ADC_MR_TRGEN                 (1 << 0)  /* Bit 0:  Trigger Enable */
#define ADC_MR_TRGSEL_SHIFT          (1)       /* Bits 1-3: Trigger Selection */
#define ADC_MR_TRGSEL_MASK           (7 << ADC_MR_TRGSEL_SHIFT)
#define ADC_MR_LOWRES                (1 << 4)  /* Bit 4:  Resolution */
#define ADC_MR_SLEEP                 (1 << 5)  /* Bit 5:  Sleep Mode */
#define ADC_MR_PRESCAL_SHIFT         (8)       /* Bits 8-15: Prescaler Rate Selection */
#define ADC_MR_PRESCAL_MASK          (0xff << ADC_MR_PRESCAL_SHIFT)
#define ADB12B_MRSTARTUP_SHIFT       (16)      /* Bits 16-23: Start Up Time (ADC12B) */
#define ADB12B_MRSTARTUP_MASK        (0xff << ADB12B_MRSTARTUP_SHIFT
#define ADB10B_MRSTARTUP_SHIFT       (16)      /* Bits 16-22: Start Up Time (ADC10B) */
#define ADB10B_MRSTARTUP_MASK        (0x7f << ADB10B_MRSTARTUP_SHIFT)
#define ADC_MR_SHTIM_SHIFT           (24)      /* Bits 24-27: Sample & Hold Time */
#define ADC_MR_SHTIM_MASK            (15 << ADC_MR_SHTIM_SHIFT)

/* ADC12B Channel Enable Register, ADC12B Channel Disable Register, ADC12B Channel
 * Status Register, ADC(10B) Channel Enable Register, ADC(10B) Channel Disable Register,
 * and ADC(10B) Channel Status Register common bit-field definitions
 */

#define ADC_CH(n)                    (1 << (n))
#define ADC_CH0                      (1 << 0)  /* Bit 0:  Channel x Enable */
#define ADC_CH1                      (1 << 1)  /* Bit 1:  Channel x Enable */
#define ADC_CH2                      (1 << 2)  /* Bit 2:  Channel x Enable */
#define ADC_CH3                      (1 << 3)  /* Bit 3:  Channel x Enable */
#define ADC_CH4                      (1 << 4)  /* Bit 4:  Channel x Enable */
#define ADC_CH5                      (1 << 5)  /* Bit 5:  Channel x Enable */
#define ADC_CH6                      (1 << 6)  /* Bit 6:  Channel x Enable */
#define ADC_CH7                      (1 << 7)  /* Bit 7:  Channel x Enable */

/* ADC12B Analog Control Register (ADC12B only) */

#define ADC12B_ACR_GAIN_SHIFT        (0)       /* Bits 0-1: Input Gain */
#define ADC12B_ACR_GAIN_MASK         (3 << ADC12B_ACR_GAIN_SHIFT)
#define ADC12B_ACR_IBCTL_SHIFT       (8)       /* Bits 8-9: Bias Current Control */
#define ADC12B_ACR_IBCTL_MASK        (3 << ADC12B_ACR_IBCTL_SHIFT)
#define ADC12B_ACR_DIFF              (1 << 16) /* Bit 16: Differential Mode */
#define ADC12B_ACR_OFFSET            (1 << 17) /* Bit 17: Input OFFSET */

/* ADC12B Extended Mode Register (ADC12B only) */

#define ADC12B_EMR_OFFMODES          (1 << 0)  /* Bit 0:  Off Mode if Sleep Bit (ADC12B_MR) = 1 */
#define ADC12B_EMR_OFFMSTIME_SHIFT   (16)      /* Bits 16-23: Startup Time */
#define ADC12B_EMR_OFFMSTIME_MASK    (0xff << ADC12B_EMR_OFFMSTIME_SHIFT)

/* ADC12B Status Register , ADC12B Interrupt Enable Register, ADC12B Interrupt
 * Disable Register, ADC12B Interrupt Mask Register, ADC(10B) Status Register,
 * ADC(10B) Interrupt Enable Register, ADC(10B) Interrupt Disable Register, and
 * ADC(10B) Interrupt Mask Register common bit-field definitions
 */

#define ADC_INT_EOC(n)               (1<<(n))
#define ADC_INT_EOC0                 (1 << 0)  /* Bit 0:  End of Conversion 0 */
#define ADC_INT_EOC1                 (1 << 1)  /* Bit 1:  End of Conversion 1 */
#define ADC_INT_EOC2                 (1 << 2)  /* Bit 2:  End of Conversion 2 */
#define ADC_INT_EOC3                 (1 << 3)  /* Bit 3:  End of Conversion 3 */
#define ADC_INT_EOC4                 (1 << 4)  /* Bit 4:  End of Conversion 4 */
#define ADC_INT_EOC5                 (1 << 5)  /* Bit 5:  End of Conversion 5 */
#define ADC_INT_EOC6                 (1 << 6)  /* Bit 6:  End of Conversion 6 */
#define ADC_INT_EOC7                 (1 << 7)  /* Bit 0:  End of Conversion 7 */
#define ADC_INT_OVRE(n)              (1<<((n)+8))
#define ADC_INT_OVRE0                (1 << 8)  /* Bit 8:  Overrun Error 0 */
#define ADC_INT_OVRE1                (1 << 9)  /* Bit 9:  Overrun Error 1 */
#define ADC_INT_OVRE2                (1 << 10) /* Bit 10: Overrun Error 2 */
#define ADC_INT_OVRE3                (1 << 11) /* Bit 11: Overrun Error 3 */
#define ADC_INT_OVRE4                (1 << 12) /* Bit 12: Overrun Error 4 */
#define ADC_INT_OVRE5                (1 << 13) /* Bit 13: Overrun Error 5 */
#define ADC_INT_OVRE6                (1 << 14) /* Bit 14: Overrun Error 6 */
#define ADC_INT_OVRE7                (1 << 15) /* Bit 15: Overrun Error 7 */
#define ADC_INT_DRDY                 (1 << 16) /* Bit 16: Data Ready */
#define ADC_INT_GOVRE                (1 << 17) /* Bit 17: General Overrun Error */
#define ADC_INT_ENDRX                (1 << 18) /* Bit 18: End of RX Buffer */
#define ADC_INT_RXBUFF               (1 << 19) /* Bit 19: RX Buffer Full */

/* ADC12B Last Converted Data Register */

#define ADC12B_LCDR_DATA_SHIFT       (0)       /* Bits 0-11: Last Data Converted */
#define ADC12B_LCDR_DATA_MASK        (0xfff << ADC12B_LCDR_DATA_SHIFT)

/* ADC(10B) Last Converted Data Register */

#define ADC10B_LCDR_DATA_SHIFT       (0)       /* Bits 0-9: Last Data Converted */
#define ADC10B_LCDR_DATA_MASK        (0x1ff << ADC10B_LCDR_DATA_SHIFT)

/* ADC12B Channel Data Register */

#define ADC12B_CDR_DATA_SHIFT        (0)       /* Bits 0-11: Converted Data */
#define ADC12B_CDR_DATA_MASK         (0xfff << ADC12B_CDR_DATA_SHIFT)

/* ADC(10B) Channel Data Register */

#define ADC10B_CDR_DATA_SHIFT        (0)       /* Bits 0-9: Converted Data */
#define ADC10B_CDR_DATA_MASK         (0x1ff << ADC10B_CDR_DATA_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_ADC_H */
