/****************************************************************************
 * config/vsn/src/muxbus.c
 * arch/arm/src/board/muxbus.c
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
 *  \brief VSN Multiplexed Bus, SDIO Interface and the Analog Front-End
 */

#include <nuttx/config.h>
#include <semaphore.h>
#include <errno.h>

#include "vsn.h"
#include "stm32_gpio.h"


/****************************************************************************
 * Private data
 ****************************************************************************/

/** Set true when the bus has been locked by the SDIO interface */
bool vsn_muxbus_ownedbysdio;

/** Semaphore to resolve conflicts between muxbus locks */
sem_t vsn_muxbus_sem;


/****************************************************************************
 * Private functions
 ****************************************************************************/

void vsn_muxbus_takeownership(void)
{
    while (sem_wait(&vsn_muxbus_sem) != 0) {
    
        /* The only case that an error should occr here is if the wait was
         * awakened by a signal.
         */

        ASSERT(errno == EINTR);
    }
}


void vsn_muxbus_sdio_release(void)
{
    stm32_unconfiggpio(GPIO_SDIO_D0);
    stm32_unconfiggpio(GPIO_SDIO_CK);
    stm32_unconfiggpio(GPIO_SDIO_CMD);
    
    vsn_muxbus_ownedbysdio = FALSE;
}


void vsn_muxbus_sdio_access(void)
{
    stm32_configgpio(GPIO_SDIO_D0);
    stm32_configgpio(GPIO_SDIO_CK);
    stm32_configgpio(GPIO_SDIO_CMD);

    vsn_muxbus_ownedbysdio = TRUE;
}


/****************************************************************************
 * Public functions
 ****************************************************************************/


void vsn_muxbus_init(void)
{
    /* Put the PGA in default shut-down state */

    stm32_configgpio(GPIO_PGIA_AEN);

    /* setup semaphore in non-locked condition */
    
    sem_init(&vsn_muxbus_sem, 0, 1);
    
    /* by default give the bus to the SDIO */
    
    vsn_muxbus_sdio_access();
}


/** 
 * We could do extra checks: who is the owner to prevent
 * unlocking from SDIO side eventhough it was not locked 
 * by him, but temporarily by the PGA
 */
void stm32_muxbus_sdio_lock(bool lock)
{
    if ( lock )
        vsn_muxbus_takeownership();
    else
        sem_post(&vsn_muxbus_sem);
}


/**
 * The following rules apply for the SDcard:
 * 
 *  - CMD serial line always starts with 0 (start-bit) and ends with 1 (stop-bit)
 *    The total length is always 48 bits protected by CRCs. When changing the 
 *    gain, CMD must be seen as 1 on CK changes.
 * 
 *  - An alternative mechanism would be to use suspend/resume commands
 * 
 *  - If SDcard internal shift-register is 8-bit oriented there might be a need
 *    to shift 7 dummy bits to properly detect invalid start of packet 
 *    (with start bit set as 1) to invalidate bus transitions (in case CK 
 *    is changing).
 * 
 * SDIO returns the bus in HiZ states, where CLK = 0, D = CMD = external pull-up
 */
int vsn_muxbus_setpgagain(int gain)
{
    /* Shutdown the Analog Devices AD8231 and exit if gain is invalid */

    stm32_gpiowrite(GPIO_PGIA_AEN, FALSE);
    
    if (gain < 0 || gain > 7)
        return -1;

    vsn_muxbus_takeownership();
    vsn_muxbus_sdio_release();
    
    /* If we have to set CLK = 1, made that first as D, CMD are 1 by pull-ups */
    
    if (gain & 2)
        stm32_configgpio(GPIO_PGIA_A1_H);
    else stm32_configgpio(GPIO_PGIA_A1_L);
    
    /* Set the D and CMD bits */
    
    if (gain & 1)
        stm32_configgpio(GPIO_PGIA_A0_H);
    else stm32_configgpio(GPIO_PGIA_A0_L);
        
    if (gain & 4)
        stm32_configgpio(GPIO_PGIA_A2_H);
    else stm32_configgpio(GPIO_PGIA_A2_L);
    
    /* Sample GAIN on rising edge */
    
    stm32_gpiowrite(GPIO_PGIA_AEN, TRUE);
    
    /* Release D and CMD pins to 1; however shorten rising edge actively */
    
    stm32_gpiowrite(GPIO_PGIA_A0_H, TRUE);
    stm32_gpiowrite(GPIO_PGIA_A2_H, TRUE);
    
    stm32_unconfiggpio(GPIO_PGIA_A0_H);
    stm32_unconfiggpio(GPIO_PGIA_A2_H);
    
    /* Release CLK by going down and return the bus */
    
    stm32_unconfiggpio(GPIO_PGIA_A1_L);    
    
    vsn_muxbus_sdio_access();
    sem_post(&vsn_muxbus_sem);
    
    return gain;
}
