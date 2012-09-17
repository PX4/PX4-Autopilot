/****************************************************************************
 * config/vsn/src/rtac.c
 * arch/arm/src/board/rtac.c
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
 *  \brief Real Time Alarm Clock
 * 
 * Implementation of the Real-Time Alarm Clock as per SNP Specifications.
 * It provides real-time and phase controlled timer module while it 
 * cooperates with hardware RTC for low-power operation.
 * 
 * It provides a replacement for a system 32-bit UTC time/date counter.
 * 
 * It runs at maximum STM32 allowed precision of 16384 Hz, providing 
 * resolution of 61 us, required by the Sensor Network Protocol.
 */

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/** Execute from a group of events
 **/
int rtac_execg(int group)
{
    // called by each thread to spawn its apps given by its ID or group ID of
    // multiple threads, when group parameter is set.
}


/** Wait and execute from a group of events
 **/
int rtac_waitg(int group, int time)
{
    // blocking variant of rtac_exec with timeout if specified
}


/** Power optimization of base systick timer
 * 
 * 1. Simple method to skip wake-ups:
 *  - ask timers about the min. period, which is Ns * systick
 *  - set the preload register with floor(Ns) * DEFAULT_PRELOAD
 *  - on wake-up call routines Ns times.
 * 
 * 2. If intermediate ISR occuried then:
 *  - check how many periods have passed by reading the counter: Np
 *  - set the new counter value as (counter % DEFAULT_PRELOAD)
 *  - call timer routines Np times; the next call is as usual, starting
 *    at 1. point above
 * 
 * This is okay if ISR's do not read timers, if they read timers then:
 *  - on ISR wake-up the code described under 2. must be called first
 *    (on wake-up from IDLE)
 * 
 * BUT: the problem is that SYSTICK does not run in Stop mode but RTC
 *   only, so it might be better to replace SYSTICK with RTAC (this
 *   module) and do the job above, permitting ultra low power modes of
 *   25 uA or further down to 5 uA.
 */
