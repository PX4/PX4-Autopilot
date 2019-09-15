/****************************************************************************
 *
 *  Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 * @file test_tone.cpp
 * Test for audio tones.
 */

#include <stdlib.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <lib/tunes/tunes.h>
#include <px4_posix.h>
#include <uORB/topics/tune_control.h>

#include "tests_main.h"

int test_tone(int argc, char *argv[])
{
	int result = PX4_ERROR;
	tune_control_s tune_control = {};
	tune_control.tune_id = static_cast<int>(TuneID::NOTIFY_NEGATIVE);

	orb_advert_t tune_control_pub = orb_advertise_queue(ORB_ID(tune_control), &tune_control,
					tune_control_s::ORB_QUEUE_LENGTH);

	if (argc == 1) {
		PX4_INFO("Volume silenced for testing predefined tunes 0-20.");
		tune_control.volume = tune_control_s::VOLUME_LEVEL_MIN;

		for (size_t i = 0; i <= 20; i++) {
			tune_control.tune_id = i;
			result = orb_publish(ORB_ID(tune_control), tune_control_pub, &tune_control);

			if (result != PX4_OK) {
				PX4_INFO("Error publishing TuneID: %d", tune_control.tune_id);
				return result;
			}
		}

		tune_control.tune_id = static_cast<int>(TuneID::NOTIFY_POSITIVE);
	}

	if (argc == 2) {
		tune_control.tune_id = atoi(argv[1]);

		if (tune_control.tune_id <= 20) {
			PX4_INFO("TuneID: %d", tune_control.tune_id);
		}
	}

	if (argc == 3) {
		int volume = tune_control_s::VOLUME_LEVEL_DEFAULT;
		Tunes tunes{};
		tunes.set_string(argv[2], volume);
		PX4_INFO("Custom tune.");
	}

	tune_control.volume = tune_control_s::VOLUME_LEVEL_DEFAULT;
	result = orb_publish(ORB_ID(tune_control), tune_control_pub, &tune_control);

	orb_unadvertise(tune_control_pub);
	return result;
}
