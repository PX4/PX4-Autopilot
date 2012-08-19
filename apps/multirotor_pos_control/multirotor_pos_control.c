/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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

/*
 * @file Implementation of AR.Drone 1.0 / 2.0 control interface
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <time.h>
#include <sys/prctl.h>
#include <arch/board/up_hrt.h>
#include "ardrone_control.h"
#include "attitude_control.h"
#include "rate_control.h"
#include "ardrone_motor_control.h"
#include "position_control.h"
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>

__EXPORT int multirotor_pos_control_main(int argc, char *argv[]);

static bool thread_should_exit;
static bool thread_running = false;
static int mpc_task;

static int
mpc_thread_main(int argc, char *argv[])
{
	/* welcome user */
	printf("[multirotor pos control] Control started, taking over position control\n");

	/* structures */
	struct vehicle_status_s state;
	struct vehicle_attitude_s att;
	//struct vehicle_global_position_setpoint_s global_pos_sp;
	struct vehicle_local_position_setpoint_s local_pos_sp;
	struct vehicle_local_position_s local_pos;
	struct manual_control_setpoint_s manual;
	struct vehicle_attitude_setpoint_s att_sp;

	/* subscribe to attitude, motor setpoints and system state */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	//int global_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
	int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));

	/* publish attitude setpoint */
	int att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	while (1) {
		/* get a local copy of the vehicle state */
		orb_copy(ORB_ID(vehicle_status), state_sub, &state);
		/* get a local copy of manual setpoint */
		orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
		/* get a local copy of attitude */
		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
		/* get a local copy of local position */
		orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);
		/* get a local copy of local position setpoint */
		orb_copy(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_sub, &local_pos_sp);

		if (state.state_machine == SYSTEM_STATE_AUTO) {
			position_control(&state, &manual, &att, &local_pos, &local_pos_sp, &att_sp);
			/* publish new attitude setpoint */
			orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
		} else if (state.state_machine == SYSTEM_STATE_STABILIZE) {
			/* set setpoint to current position */
			// XXX select pos reset channel on remote
			/* reset setpoint to current position  (position hold) */
			// if (1 == 2) {
			// 	local_pos_sp.x = local_pos.x;
			// 	local_pos_sp.y = local_pos.y;
			// 	local_pos_sp.z = local_pos.z;
			// 	local_pos_sp.yaw = att.yaw;
			// }
		}

		/* run at approximately 50 Hz */
		usleep(20000);

		counter++;
	}

	/* close uarts */
	close(ardrone_write);
	ar_multiplexing_deinit(gpios);

	printf("[multirotor pos control] ending now...\r\n");
	fflush(stdout);
	return 0;
}

