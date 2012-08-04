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
 * @file GPIO driver for PX4IO
 */

#include <sys/ioctl.h>

#define _GPIO_IOCTL_BASE	0x7700

#define GPIO_SET(_x)		_IOC(_GPIO_IOCTL_BASE, _x)
#define GPIO_GET(_x)		_IOC(_GPIO_IOCTL_BASE + 1, _x)

/*
 * List of GPIOs; must be sorted with settable GPIOs first.
 */
#define GPIO_ACC1_POWER		0	/* settable */
#define GPIO_ACC2_POWER		1
#define GPIO_SERVO_POWER	2
#define GPIO_RELAY1		3
#define GPIO_RELAY2		4
#define GPIO_LED_BLUE		5
#define GPIO_LED_AMBER		6
#define GPIO_LED_SAFETY		7

#define GPIO_ACC_OVERCURRENT	8	/* readonly */
#define GPIO_SERVO_OVERCURRENT	9
#define GPIO_SAFETY_BUTTON	10

#define GPIO_MAX_SETTABLE	7
#define GPIO_MAX		10

/*
 * GPIO driver init function.
 */
extern int gpio_drv_init(void);
