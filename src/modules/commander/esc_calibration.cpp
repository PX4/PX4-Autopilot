/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file esc_calibration.cpp
 *
 * Definition of esc calibration
 *
 * @author Roman Bapst <bapstr@ethz.ch>
 */

#include "esc_calibration.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <poll.h>
#include "drivers/drv_pwm_output.h"
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/uORB.h>
#include <drivers/drv_hrt.h>
#include <mavlink/mavlink_log.h>


int do_esc_calibration(int mavlink_fd) {
	struct battery_status_s battery;
	memset(&battery,0,sizeof(battery));
	int batt_sub = orb_subscribe(ORB_ID(battery_status));
	memset(&battery,0,sizeof(battery));
	orb_copy(ORB_ID(battery_status), batt_sub, &battery);

	if (battery.voltage_filtered_v > 3.0f && !(hrt_absolute_time() - battery.timestamp > 500000)) {
		warnx("Please disconnect battery before calibration!");
		mavlink_log_info(mavlink_fd, "Please disconnect battery and try again!");
		return -1;
	}

	unsigned max_channels = 0;

	mavlink_log_info(mavlink_fd, "Please select pins on which ESC's are attached.");

	// wait for the user to provide the number of engines connected
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	struct vehicle_command_s cmd;
	bool updated = false;
	while(true) {
		orb_check(cmd_sub,&updated);
		if(updated) {
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);
			if((int)(cmd.param7) > 100 && cmd.command == VEHICLE_CMD_PREFLIGHT_CALIBRATION) {
				break;
			}
		}
		usleep(500000);
	}

	// parse user input
	uint32_t set_mask = 0;
	int config = (int)(cmd.param7);
	int multiplier = 10000000;
	for(unsigned i = 0;i<8;i++) {
		if(2*multiplier > config) {
			// this pin does not have an esc
			config -= multiplier;
		} else {
			// this pin has an esc
			set_mask |= (1<<i);
			config -= 2*multiplier;
		}
		multiplier /= 10;
	}

	uint16_t pwm_high = PWM_DEFAULT_MAX;
	uint16_t pwm_low = PWM_DEFAULT_MIN;

	int fd = open(PWM_OUTPUT0_DEVICE_PATH, 0);

	if(fd < 0) {
		err(1,"Can't open %s", PWM_OUTPUT0_DEVICE_PATH);
	}

	/* get number of channels available on the device */
	int ret;
	ret = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&max_channels);
	if (ret != OK)
		err(1, "PWM_SERVO_GET_COUNT");

	/* tell IO/FMU that its ok to disable its safety with the switch */
	ret = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);
	if (ret != OK)
		err(1, "PWM_SERVO_SET_ARM_OK");
	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	ret = ioctl(fd, PWM_SERVO_ARM, 0);
	if (ret != OK)
		err(1, "PWM_SERVO_ARM");
	/* tell IO to switch off safety without using the safety switch */
	ret = ioctl(fd, PWM_SERVO_SET_FORCE_SAFETY_OFF, 0);
	if(ret!=0) {
		err(1,"PWM_SERVO_SET_FORCE_SAFETY_OFF");
	}

	warnx("Please connect battery now");
	mavlink_log_info(mavlink_fd,"Please connect battery now");

	/* wait for one of the following events:
		1) user has pressed the button in QGroundControl
		2) timeout of 5 seconds is reached
	*/
	hrt_abstime start_time = hrt_absolute_time();

	while(true) {
		for (unsigned i = 0; i < max_channels; i++) {
			if (set_mask & 1<<i) {
				ret = ioctl(fd, PWM_SERVO_SET(i), pwm_high);

				if (ret != OK)
					err(1, "PWM_SERVO_SET(%d), value: %d", i, pwm_high);
			}
		}

		orb_check(batt_sub,&updated);
		if(updated) {
			orb_copy(ORB_ID(battery_status), batt_sub, &battery);
		}
		// user has connected battery
		if(battery.voltage_filtered_v > 3.0f) {
			orb_check(cmd_sub,&updated);
			if(updated) {
				orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);
			}
			if((int)(cmd.param7) == 2 && cmd.command == VEHICLE_CMD_PREFLIGHT_CALIBRATION) {
				break;
			} else if (hrt_absolute_time() - start_time > 5000000) {
				// waited for 5 seconds, switch to low pwm
				break;
			}
		}
		else {
			start_time = hrt_absolute_time();
		}
	}

	/* set low PWM */
	for (unsigned i = 0; i < max_channels; i++) {
		if (set_mask & 1<<i) {
			ret = ioctl(fd, PWM_SERVO_SET(i), pwm_low);

			if (ret != OK)
				err(1, "PWM_SERVO_SET(%d), value: %d", i, pwm_low);
		}
	}

	usleep(2000000);	// give esc some time to save low pwm value

	/* disarm */
	ret = ioctl(fd, PWM_SERVO_DISARM, 0);
	if (ret != OK)
		err(1, "PWM_SERVO_DISARM");

	warnx("ESC calibration finished");
	mavlink_log_info(mavlink_fd,"ESC calibration finished");

	exit(0);
 }