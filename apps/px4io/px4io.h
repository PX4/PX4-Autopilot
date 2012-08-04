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
  * @file General defines and structures for the PX4IO module firmware.
  */

#include <arch/board/drv_gpio.h>
#include "protocol.h"

/*
 * Constants and limits.
 */
#define MAX_CONTROL_CHANNELS	12
#define IO_SERVO_COUNT		8

/*
 * System state structure.
 */
struct sys_state_s 
{

	bool		armed;		/* actually armed */
	bool		arm_ok;		/* FMU says OK to arm */

	/*
	 * Data from the remote control input(s)
	 */
	int		rc_channels;
	uint16_t	rc_channel_data[PX4IO_INPUT_CHANNELS];

	/*
	 * Control signals from FMU.
	 */
	uint16_t	fmu_channel_data[PX4IO_OUTPUT_CHANNELS];

	/*
	 * Relay controls
	 */
	bool		relays[PX4IO_RELAY_CHANNELS];

	/*
	 * If true, we are using the FMU controls.
	 */
	bool		mixer_use_fmu;

	/*
	 * If true, state that should be reported to FMU has been updated.
	 */
	bool		fmu_report_due;

	/*
	 * If true, new control data from the FMU has been received.
	 */
	bool		fmu_data_received;
};

extern struct sys_state_s system_state;

/*
 * Software countdown timers.
 *
 * Each timer counts down to zero at one tick per ms.
 */
#define TIMER_BLINK_AMBER	0
#define TIMER_BLINK_BLUE	1
#define TIMER_STATUS_PRINT	2
#define TIMER_SANITY		7
#define TIMER_NUM_TIMERS	8
extern volatile int	timers[TIMER_NUM_TIMERS];

/*
 * GPIO handling.
 */
extern int gpio_fd;

#define POWER_SERVO(_s)		ioctl(gpio_fd, GPIO_SET(GPIO_SERVO_POWER), (_s))
#define POWER_ACC1(_s)		ioctl(gpio_fd, GPIO_SET(GPIO_SERVO_ACC1), (_s))
#define POWER_ACC2(_s)		ioctl(gpio_fd, GPIO_SET(GPIO_SERVO_ACC2), (_s))
#define POWER_RELAY1(_s)	ioctl(gpio_fd, GPIO_SET(GPIO_RELAY1, (_s))
#define POWER_RELAY2(_s)	ioctl(gpio_fd, GPIO_SET(GPIO_RELAY2, (_s))

#define LED_AMBER(_s)		ioctl(gpio_fd, GPIO_SET(GPIO_LED_AMBER), !(_s))
#define LED_BLUE(_s)		ioctl(gpio_fd, GPIO_SET(GPIO_LED_BLUE), !(_s))
#define LED_SAFETY(_s)		ioctl(gpio_fd, GPIO_SET(GPIO_LED_SAFETY), !(_s))

#define OVERCURRENT_ACC		ioctl(gpio_fd, GPIO_GET(GPIO_ACC_OVERCURRENT), 0)
#define OVERCURRENT_SERVO	ioctl(gpio_fd, GPIO_GET(GPIO_SERVO_OVERCURRENT), 0)
#define BUTTON_SAFETY		ioctl(gpio_fd, GPIO_GET(GPIO_SAFETY_BUTTON), 0)

/*
 * Mixer
 */
extern int	mixer_init(const char *mq_name);

/*
 * Safety switch/LED.
 */
extern void	safety_init(void);

/*
 * FMU communications
 */
extern void	comms_init(void);
extern void	comms_check(void);

/*
 * Assertion codes
 */
#define A_GPIO_OPEN_FAIL		100
#define A_SERVO_OPEN_FAIL		101
#define A_INPUTQ_OPEN_FAIL		102
