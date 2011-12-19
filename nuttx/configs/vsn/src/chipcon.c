/****************************************************************************
 * configs/vsn/src/chipcon.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 * 
 *   Author: Uros Platise <uros.platise@isotel.eu>
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
 *  \brief Chipcon CC1101 Interface
 */


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/** Set external clock frequency, or disable it
 */
int chipcon_setXclock(int prescaler)
{
	// check present state, if it is enabled (in the chip!)
	
	// change state and with OK if everything is OK.
	
	return ERROR;
}


int chipcon_setchannel(uint16_t channel)
{
}


void chipcon_init(int spino)
{
	// create stream driver, where STDIN is packet oriented
	// means that two messages received are kept separated
	// in internal buffers.
	
	// default mode is AUTO, RX enabled and auto TX on writes and
	// when chipcon is IDLE.
}


void chipcon_open(void)
{
}


void chipcon_ioctl(void)
{
	// access to setXclock
}
