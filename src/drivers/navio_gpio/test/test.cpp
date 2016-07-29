/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include "navio_gpio.h"

#include <stdio.h>
#include <unistd.h>

#define LED_CNF     (GPIO_CNF_OUTPUT)
#define LED_pinR    GPIO_PIN4
#define LED_pinG    GPIO_PIN27
#define LED_pinB    GPIO_PIN6

#define LED_OFF 1
#define LED_ON  0

using namespace navio_gpio;

int do_test();

int do_test()
{
	Gpio gpio;

	if (gpio.start() < 0) {
		return -1;
	}

	gpio.configgpio(LED_CNF | LED_pinR);
	gpio.configgpio(LED_CNF | LED_pinG);
	gpio.configgpio(LED_CNF | LED_pinB);


	gpio.gpiowrite(LED_pinR, LED_OFF);
	gpio.gpiowrite(LED_pinG, LED_OFF);
	gpio.gpiowrite(LED_pinB, LED_OFF);
	printf("off\n");
	sleep(2);

	gpio.gpiowrite(LED_pinR, LED_ON);
	gpio.gpiowrite(LED_pinG, LED_OFF);
	gpio.gpiowrite(LED_pinB, LED_OFF);
	printf("red\n");
	sleep(2);

	gpio.gpiowrite(LED_pinR, LED_OFF);
	gpio.gpiowrite(LED_pinG, LED_ON);
	gpio.gpiowrite(LED_pinB, LED_OFF);
	printf("green\n");
	sleep(2);

	gpio.gpiowrite(LED_pinR, LED_OFF);
	gpio.gpiowrite(LED_pinG, LED_OFF);
	gpio.gpiowrite(LED_pinB, LED_ON);
	printf("blue\n");
	sleep(2);

	gpio.gpiowrite(LED_pinR, LED_OFF);
	gpio.gpiowrite(LED_pinG, LED_OFF);
	gpio.gpiowrite(LED_pinB, LED_OFF);
	printf("off\n");
	gpio.stop();

	return 0;
}
