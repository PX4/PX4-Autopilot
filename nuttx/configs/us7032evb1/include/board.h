/****************************************************************************
 * configs/us7032evb1/include/board.h
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
 ****************************************************************************/

#ifndef _CONFIGS_US7032EVB1_BOARD_H
#define _CONFIGS_US7032EVB1_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif
#include "chip.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define SH1_CLOCK            20000000  /* 20 MHz */

/* LED definitions **********************************************************/

/* The SH1_LPEVB only a single LED controlled by either port A, pin 15, or
 * port B, pin 15 (selectable via JP8).
 */

#define LED_STARTED          0
#define LED_HEAPALLOCATE     1
#define LED_IRQSENABLED      1
#define LED_STACKCREATED     1
#define LED_INIRQ            0
#define LED_SIGNAL           0
#define LED_ASSERTION        0
#define LED_PANIC            1

/* Button definitions *******************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
EXTERN void up_buttoninit(void);
EXTERN uint8_t up_buttons(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif  /* _CONFIGS_US7032EVB1_BOARD_H */
