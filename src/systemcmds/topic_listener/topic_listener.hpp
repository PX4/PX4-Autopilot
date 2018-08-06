/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file listener.hpp
 *
 */

#pragma once

#include <px4_defines.h>
#include <drivers/drv_hrt.h>
#include <px4_middleware.h>
#include <px4_app.h>
#include <px4_config.h>
#include <px4_log.h>
#include <uORB/uORB.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

template <typename T>
void listener(const orb_id_t &id, unsigned num_msgs, unsigned topic_instance, unsigned topic_interval)
{
	if (orb_exists(id, topic_instance) != 0) {
		PX4_INFO_RAW("never published\n");
		return;
	}

	int sub = orb_subscribe_multi(id, topic_instance);
	orb_set_interval(sub, topic_interval);

	bool updated = false;
	unsigned i = 0;
	hrt_abstime start_time = hrt_absolute_time();

	while (i < num_msgs) {
		orb_check(sub, &updated);

		if (i == 0) {
			updated = true;

		} else {
			usleep(500);
		}

		if (updated) {
			start_time = hrt_absolute_time();
			i++;

			PX4_INFO_RAW("\nTOPIC: %s instance %d #%d\n", id->o_name, topic_instance, i);

			T container;

			if (orb_copy(id, sub, &container) == PX4_OK) {
				print_message(container);

			} else {
				PX4_ERR("orb_copy failed");
			}

		} else {
			if (hrt_elapsed_time(&start_time) > 2 * 1000 * 1000) {
				PX4_INFO_RAW("Waited for 2 seconds without a message. Giving up.\n");
				break;
			}
		}
	}

	orb_unsubscribe(sub);
}
