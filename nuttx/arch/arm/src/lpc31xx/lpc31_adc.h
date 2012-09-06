/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_adc.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_ADC_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_ADC_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* ADC register base address offset into the APB0 domain ****************************************/

#define LPC31_ADC_VBASE                (LPC31_APB0_VADDR+LPC31_APB0_ADC_OFFSET)
#define LPC31_ADC_PBASE                (LPC31_APB0_PADDR+LPC31_APB0_ADC_OFFSET)

/* ADC register offsets (with respect to the ADC base) ******************************************/

#define LPC31_ADC_R0_OFFSET            0x000 /* Data for analog input channel 0 */
#define LPC31_ADC_R1_OFFSET            0x004 /* Data for analog input channel 1 */
#define LPC31_ADC_R2_OFFSET            0x008 /* Data for analog input channel 2 */
#define LPC31_ADC_R3_OFFSET            0x00c /* Data for analog input channel 3 */
                                               /* 0x010-0x01c: Reserved */
#define LPC31_ADC_CON_OFFSET           0x020 /* ADC control register */
#define LPC31_ADC_CSEL_OFFSET          0x024 /* Configure and select analog input channels */
#define LPC31_ADC_INTEN_OFFSET         0x028 /* Enable ADC interrupts */
#define LPC31_ADC_INTST_OFFSET         0x02C /* ADC interrupt status */
#define LPC31_ADC_INTCLR_OFFSET        0x030 /* Clear ADC interrupt status */
                                               /* 0x034-: Reserved */

/* ADC register (virtual) addresses *************************************************************/

#define LPC31_ADC_R0                   (LPC31_ADC_VBASE+LPC31_ADC_R0_OFFSET)
#define LPC31_ADC_R1                   (LPC31_ADC_VBASE+LPC31_ADC_R1_OFFSET)
#define LPC31_ADC_R2                   (LPC31_ADC_VBASE+LPC31_ADC_R2_OFFSET)
#define LPC31_ADC_R3                   (LPC31_ADC_VBASE+LPC31_ADC_R3_OFFSET)
#define LPC31_ADC_CON                  (LPC31_ADC_VBASE+LPC31_ADC_CON_OFFSET)
#define LPC31_ADC_CSEL                 (LPC31_ADC_VBASE+LPC31_ADC_CSEL_OFFSET)
#define LPC31_ADC_INTEN                (LPC31_ADC_VBASE+LPC31_ADC_INTEN_OFFSET)
#define LPC31_ADC_INTST                (LPC31_ADC_VBASE+LPC31_ADC_INTST_OFFSET)
#define LPC31_ADC_INTCLR               (LPC31_ADC_VBASE+LPC31_ADC_INTCLR_OFFSET)

/* ADC register bit definitions *****************************************************************/

/* ADC_Rx (ADC_R0, address 0x13002000; ADC_R1, address 0x13002004, ADC_R2, address 0x13002008;
 *         ADC_R3, address 0x1300200c)
 */

#define ADC_RX_SHIFT                     (0)  /* Bits 0-9: Digital conversion data */
#define ADC_RX_MASK                      (0x3ff << ADC_RX_SHIFT)

/* ADC_CON, address 0x13002020 */

#define ADC_CON_STATUS                   (1 << 4)  /* Bit 4:  ADC Status */
#define ADC_CON_START                    (1 << 3)  /* Bit 3:  Start command */
#define ADC_CON_CSCAN                    (1 << 2)  /* Bit 2:  Continuous scan */
#define ADC_CON_ENABLE                   (1 << 1)  /* Bit 1:  ADC enable */

/* ADC_CSEL, address 0x13002024 */

#define ADC_CSEL_CHAN3_SHIFT             (12)      /* Bits 12-15: Select and configure channel 3*/
#define ADC_CSEL_CHAN3_MASK              (15 << ADC_CSEL_CHAN3_SHIFT)
#define ADC_CSEL_CHAN2_SHIFT             (8)       /* Bits 8-10: Select and configure channel 2*/
#define ADC_CSEL_CHAN2_MASK              (15 << ADC_CSEL_CHAN2_SHIFT)
#define ADC_CSEL_CHAN1_SHIFT             (4)       /* Bits 4-7: Select and configure channel 1*/
#define ADC_CSEL_CHAN1_MASK              (15 << ADC_CSEL_CHAN1_SHIFT)
#define ADC_CSEL_CHAN0_SHIFT             (0)       /* Bits 0-3: Select and configure channel 0*/
#define ADC_CSEL_CHAN0_MASK              (15 << ADC_CSEL_CHAN0_SHIFT)

/* ADC_INTEN, address 0x13002028 */

#define ADC_INTEN_ENABLE                 (1 << 0)

/* ADC_INTST, address 0x1300202c */

#define ADC_INTST_PENDING                (1 << 0)

/* ADC_INTCLR, address 0x13002030 */

#define ADC_INTCLR_CLEAR                 (1 << 0)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_ADC_H */
