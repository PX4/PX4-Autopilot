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

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

int check_if_batt_disconnected(int mavlink_fd) {
	struct battery_status_s battery;
	memset(&battery,0,sizeof(battery));
	int batt_sub = orb_subscribe(ORB_ID(battery_status));
	orb_copy(ORB_ID(battery_status), batt_sub, &battery);

	if (battery.voltage_filtered_v > 3.0f && !(hrt_absolute_time() - battery.timestamp > 500000)) {
		mavlink_log_info(mavlink_fd, "Please disconnect battery and try again!");
		return ERROR;
	}
	return OK;
}


int do_esc_calibration(int mavlink_fd) {

	int fd = open(PWM_OUTPUT0_DEVICE_PATH, 0);
	int ret;

	if(fd < 0) {
		err(1,"Can't open %s", PWM_OUTPUT0_DEVICE_PATH);
	}

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

	mavlink_log_info(mavlink_fd,"Please connect battery now");

	struct battery_status_s battery;
	memset(&battery,0,sizeof(battery));
	int batt_sub = orb_subscribe(ORB_ID(battery_status));
	orb_copy(ORB_ID(vehicle_command),batt_sub, &battery);
	bool updated = false;

	struct vehicle_command_s cmd;
	memset(&cmd, 0, sizeof(cmd));
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	orb_copy(ORB_ID(vehicle_command),cmd_sub, &cmd);

	/* wait for one of the following events:
		1) user has pressed the button in QGroundControl
		2) timeout of 5 seconds is reached
	*/
	hrt_abstime start_time = hrt_absolute_time();

	while(true) {
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
		usleep(50000);
	}

	/* disarm */
	ret = ioctl(fd, PWM_SERVO_DISARM, 0);
	if (ret != OK)
		err(1, "PWM_SERVO_DISARM");

	mavlink_log_info(mavlink_fd,"ESC calibration finished");
	return OK;
 }