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
#pragma once

#include "DevObj.hpp"

#include "navio_gpio.h"

#define RGBLED_BASE_DEVICE_PATH "/dev/rgbled"
#define RGBLED_DEVICE_PATH  "/dev/rgbled0"

// inverted
#define LED_ON  0
#define LED_OFF 1


using namespace navio_gpio;

class RGBLED : public DevObj
{
public:
	RGBLED(const char *name) :
		DevObj(name,
		       RGBLED_DEVICE_PATH,
		       RGBLED_BASE_DEVICE_PATH,
		       DeviceBusType_UNKNOWN,
		       0),
		_rgbsets{
		{LED_OFF, LED_OFF, LED_OFF}, /* OFF */
		{LED_ON,  LED_OFF, LED_OFF}, /* red */
		{LED_ON,  LED_ON,  LED_OFF}, /* yellow */
		{LED_ON,  LED_OFF, LED_ON},  /* purple */
		{LED_OFF, LED_ON,  LED_OFF}, /* green */
		{LED_OFF, LED_OFF, LED_ON},  /* blue */
		{LED_ON,  LED_ON,  LED_ON},  /* white */
	},
	_max_color(7),
		   _rgb{LED_OFF, LED_OFF, LED_OFF},
		   _turn(true)
	{ };
	virtual ~RGBLED()
	{ };

	int start();
	int stop();
	int devIOCTL(unsigned long request, unsigned long arg);

protected:
	void _measure();

private:
	Gpio _gpio;
	const rgbled_rgbset_t _rgbsets[7];
	const int _max_color;
	rgbled_rgbset_t _rgb;
	bool _turn;
};
