/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

/**
 * @file led.cpp
 *
 * LED driver.
 */

#include <nuttx/config.h>
#include <drivers/device/device.h>
#include <drivers/drv_led.h>

/* Ideally we'd be able to get these from up_internal.h */
//#include <up_internal.h>
__BEGIN_DECLS
extern void up_ledinit();
extern void up_ledon(int led);
extern void up_ledoff(int led);
__END_DECLS

class LED : device::CDev
{
public:
	LED();
	virtual ~LED();

	virtual int		init();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);
};

LED::LED() :
	CDev("led", LED_DEVICE_PATH)
{
	// force immediate init/device registration
	init();
}

LED::~LED()
{
}

int
LED::init()
{
	CDev::init();
	up_ledinit();

	return 0;
}

int
LED::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int result = OK;

	switch (cmd) {
	case LED_ON:
		up_ledon(arg);
		break;

	case LED_OFF:
		up_ledoff(arg);
		break;

	default:
		result = CDev::ioctl(filp, cmd, arg);
	}
	return result;
}

namespace
{
LED	*gLED;
}

void
drv_led_start()
{
	if (gLED == nullptr) {
		gLED = new LED;
		if (gLED != nullptr)
			gLED->init();
	}
}