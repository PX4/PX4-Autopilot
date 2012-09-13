/************************************************************************************
 * arch/arm/src/str71x/str71x_gpio.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_GPIO_H
#define __ARCH_ARM_SRC_STR71X_STR71X_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/************************************************************************************
 * Pre-procesor Definitions
 ************************************************************************************/

/* GPIO register offsets ************************************************************/

#define STR71X_GPIO_PC0_OFFSET (0x0000)  /* 16-bits wide */
#define STR71X_GPIO_PC1_OFFSET (0x0004)  /* 16-bits wide */
#define STR71X_GPIO_PC2_OFFSET (0x0008)  /* 16-bits wide */
#define STR71X_GPIO_PD_OFFSET  (0x000c)  /* 16-bits wide */

/* GPIO register addresses **********************************************************/

#define STR71X_GPIO_PC0(b)     ((b) + STR71X_GPIO_PC0_OFFSET)
#define STR71X_GPIO_PC1(b)     ((b) + STR71X_GPIO_PC1_OFFSET)
#define STR71X_GPIO_PC2(b)     ((b) + STR71X_GPIO_PC2_OFFSET)
#define STR71X_GPIO_PD(b)      ((b) + STR71X_GPIO_PD_OFFSET)

#define STR71X_GPIO0_PC0       (STR71X_GPIO0_BASE + STR71X_GPIO_PC0_OFFSET)
#define STR71X_GPIO0_PC1       (STR71X_GPIO0_BASE + STR71X_GPIO_PC1_OFFSET)
#define STR71X_GPIO0_PC2       (STR71X_GPIO0_BASE + STR71X_GPIO_PC2_OFFSET)
#define STR71X_GPIO0_PD        (STR71X_GPIO0_BASE + STR71X_GPIO_PD_OFFSET)

#define STR71X_GPIO1_PC0       (STR71X_GPIO1_BASE + STR71X_GPIO_PC0_OFFSET)
#define STR71X_GPIO1_PC1       (STR71X_GPIO1_BASE + STR71X_GPIO_PC1_OFFSET)
#define STR71X_GPIO1_PC2       (STR71X_GPIO1_BASE + STR71X_GPIO_PC2_OFFSET)
#define STR71X_GPIO1_PD        (STR71X_GPIO1_BASE + STR71X_GPIO_PD_OFFSET)

#define STR71X_GPIO2_PC0       (STR71X_GPIO2_BASE + STR71X_GPIO_PC0_OFFSET)
#define STR71X_GPIO2_PC1       (STR71X_GPIO2_BASE + STR71X_GPIO_PC1_OFFSET)
#define STR71X_GPIO2_PC2       (STR71X_GPIO2_BASE + STR71X_GPIO_PC2_OFFSET)
#define STR71X_GPIO2_PD        (STR71X_GPIO2_BASE + STR71X_GPIO_PD_OFFSET)

/* Register bit settings ************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_GPIO_H */
