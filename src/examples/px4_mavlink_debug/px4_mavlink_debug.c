/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file px4_mavlink_debug.c
 * Debug application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>

__EXPORT int px4_mavlink_debug_main(int argc, char *argv[]);

int px4_mavlink_debug_main(int argc, char *argv[])
{
	printf("Hello Debug!\n");

	/* advertise named debug value */
	struct debug_key_value_s dbg_key = { .key = "velx", .value = 0.0f };
	orb_advert_t pub_dbg_key = orb_advertise(ORB_ID(debug_key_value), &dbg_key);

	/* advertise indexed debug value */
	struct debug_value_s dbg_ind = { .ind = 42, .value = 0.5f };
	orb_advert_t pub_dbg_ind = orb_advertise(ORB_ID(debug_value), &dbg_ind);

	/* advertise debug vect */
	struct debug_vect_s dbg_vect = { .name = "vel3D", .x = 1.0f, .y = 2.0f, .z = 3.0f };
	orb_advert_t pub_dbg_vect = orb_advertise(ORB_ID(debug_vect), &dbg_vect);

	int value_counter = 0;

	while (value_counter < 100) {
		uint64_t timestamp_us = hrt_absolute_time();
		uint32_t timestamp_ms = timestamp_us / 1000;

		/* send one named value */
		dbg_key.value = value_counter;
		dbg_key.timestamp_ms = timestamp_ms;
		orb_publish(ORB_ID(debug_key_value), pub_dbg_key, &dbg_key);

		/* send one indexed value */
		dbg_ind.value = 0.5f * value_counter;
		dbg_ind.timestamp_ms = timestamp_ms;
		orb_publish(ORB_ID(debug_value), pub_dbg_ind, &dbg_ind);

		/* send one vector */
		dbg_vect.x = 1.0f * value_counter;
		dbg_vect.y = 2.0f * value_counter;
		dbg_vect.z = 3.0f * value_counter;
		dbg_vect.timestamp_us = timestamp_us;
		orb_publish(ORB_ID(debug_vect), pub_dbg_vect, &dbg_vect);

		warnx("sent one more value..");

		value_counter++;
		usleep(500000);
	}

	return 0;
}
