/****************************************************************************
 * config/vsn/src/power.c
 * arch/arm/src/board/power.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *
 *   Authors: Uros Platise <uros.platise@isotel.eu>
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

/** \file
 *  \author Uros Platise
 *  \brief VSN Power
 */

#include <nuttx/config.h>

#include <arch/board/board.h>
#include <arch/stm32/irq.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include "vsn.h"


/****************************************************************************
 * Declarations and Structures
 ****************************************************************************/ 

/****************************************************************************
 * Private Functions
 ****************************************************************************/ 

void board_power_register(void);
void board_power_adjust(void);
void board_power_status(void);



/****************************************************************************
 * Public Functions
 ****************************************************************************/ 

void board_power_init(void)
{
    stm32_configgpio(GPIO_PVS);
    stm32_configgpio(GPIO_PST);
    stm32_configgpio(GPIO_XPWR);
    stm32_configgpio(GPIO_SCTC);
    stm32_configgpio(GPIO_PCLR);
}


void board_power_reboot(void)
{
    // low-level board reset (not just MCU reset)
    // if external power is present, stimulate power-off as board
    // will wake-up immediatelly, if power is not present, set an alarm
    // before power off the board.
}


void board_power_off(void)
{
    // Check if external supply is not present, otherwise return
    // notifying that it is not possible to power-off the board

    // \todo
    
    // stop background processes
    irqsave();

    // switch to internal HSI and get the PD0 and PD1 as GPIO
    sysclock_select_hsi();

    // trigger shutdown with pull-up resistor (not push-pull!) and wait.
    stm32_gpiowrite(GPIO_PCLR, true);
    for(;;);
}
