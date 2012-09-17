/************************************************************************************
 * arch/arm/src/str71x/str71x_adc12.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_ADC12_H
#define __ARCH_ARM_SRC_STR71X_STR71X_ADC12_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/************************************************************************************
 * Pre-procesor Definitions
 ************************************************************************************/

/* ADC12 registers ******************************************************************/

#define STR71X_ADC12_DATA0    (STR71X_ADC12_BASE + 0x0000)    /* 16-bits wide */
#define STR71X_ADC12_DATA1    (STR71X_ADC12_BASE + 0x0008)    /* 16-bits wide */
#define STR71X_ADC12_DATA2    (STR71X_ADC12_BASE + 0x0010)    /* 16-bits wide */
#define STR71X_ADC12_DATA3    (STR71X_ADC12_BASE + 0x0018)    /* 16-bits wide */
#define STR71X_ADC12_CSR      (STR71X_ADC12_BASE + 0x0020)    /* 16-bits wide */
#define STR71X_ADC12_CPR      (STR71X_ADC12_BASE + 0x0030)    /* 16-bits wide */

/* Register bit settings ************************************************************/
/* ADC12 Conversion modes */

#define STR71X_ADC12_SINGLE   (0)
#define STR71X_ADC12_ROUND    (1)

/* ADC12 Channels */

#define STR71X_ADC12_CHANNEL0 (0x00)
#define STR71X_ADC12_CHANNEL1 (0x10)
#define STR71X_ADC12_CHANNEL2 (0x20)
#define STR71X_ADC12_CHANNEL3 (0x30)

/* ADC12 control status register */

#define STR71X_ADC12_DA0      (0x0001)
#define STR71X_ADC12_DA1      (0x0002)
#define STR71X_ADC12_DA2      (0x0004)
#define STR71X_ADC12_DA3      (0x0008)
#define STR71X_ADC12_OR       (0x2000)

/* Interrupt bits for channel n */

#define STR71X_ADC12_IT0      (0x0100)
#define STR71X_ADC12_IT1      (0x0200)
#define STR71X_ADC12_IT2      (0x0400)
#define STR71X_ADC12_IT3      (0x0800)
#define STR71X_ADC12_ITALL    (0x0f00)

/* Mode selection */

#define STR71X_ADC12_MODE     (0x0040)

/* Converter configuration */

#define STR71X_ADC12_START    (0x0020)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_ADC12_H */
