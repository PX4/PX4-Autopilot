/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *   Author: David Sidrane<david_s5@nscdg.com>
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>
#include "chip/stm32_tim.h"


#include "led.hpp"

void rgb_led(int r, int g, int b, int freqs)
{

	long fosc = 72000000;
	long prescale = 2048;
	long p1s = fosc / prescale;
	long p0p5s  = p1s / 2;
	stm32_tim_channel_t mode = (stm32_tim_channel_t)(STM32_TIM_CH_OUTPWM | STM32_TIM_CH_POLARITY_NEG);
	static struct stm32_tim_dev_s *tim = 0;

	if (tim == 0) {
		tim = stm32_tim_init(3);
		STM32_TIM_SETMODE(tim, STM32_TIM_MODE_UP);
		STM32_TIM_SETCLOCK(tim, p1s - 8);
		STM32_TIM_SETPERIOD(tim, p1s);
		STM32_TIM_SETCOMPARE(tim, 1, 0);
		STM32_TIM_SETCOMPARE(tim, 2, 0);
		STM32_TIM_SETCOMPARE(tim, 3, 0);
		STM32_TIM_SETCHANNEL(tim, 1, mode);
		STM32_TIM_SETCHANNEL(tim, 2, mode);
		STM32_TIM_SETCHANNEL(tim, 3, mode);
	}

	long p  = freqs == 0 ? p1s : p1s / freqs;
	STM32_TIM_SETPERIOD(tim, p);

	p  = freqs == 0 ? p1s + 1 : p0p5s / freqs;

	STM32_TIM_SETCOMPARE(tim, 1, (r * p) / 255);
	STM32_TIM_SETCOMPARE(tim, 2, (g * p) / 255);
	STM32_TIM_SETCOMPARE(tim, 3, (b * p) / 255);
}
