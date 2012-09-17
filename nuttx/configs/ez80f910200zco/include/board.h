/****************************************************************************
 * arch/ez80f910200zco/include/board.h
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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Clocking */

#define EZ80_SYS_CLK_FREQ           50000000

/* LED pattern definitions                 ON                OFF            */

#define LED_STARTED                 0  /*  '0'               N/A            */
#define LED_HEAPALLOCATE            1  /*  'H'               N/A            */
#define LED_IRQSENABLED             2  /*  'E'               N/A            */
#define LED_STACKCREATED            3  /*  'C'               N/A            */
#define LED_IDLE                    4  /*  'R'               N/A            */
#define LED_INIRQ                   5  /*  (ignored)        (ignored)       */
#define LED_ASSERTION               6  /*  'A'              (previous)      */
#define LED_SIGNAL                  7  /*  'S'              (previous)      */
#define LED_PANIC                   8  /*  '*'              (previous)      */

/* Button definitions */

#define BUTTON_PB0                 0x01 /* PB0: SW1 Bit 0 of GPIO Port B    */
#define BUTTON_PB1                 0x02 /* PB1: SW2 Bit 1 of GPIO Port B    */
#define BUTTON_PB2                 0x04 /* PB2: SW3 Bit 2 of GPIO Port B    */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#ifdef CONFIG_ARCH_BUTTONS
EXTERN void up_buttoninit(void);
EXTERN uint8_t up_buttons(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif  /* __ARCH_BOARD_BOARD_H */
