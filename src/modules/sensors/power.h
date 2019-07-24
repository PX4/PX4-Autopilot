/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#ifndef PX4_POWER_H
#define PX4_POWER_H

// TODO: Clean out

#include <board_config.h>

#include <px4_config.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <mathlib/mathlib.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_airspeed.h>

#include <airspeed/airspeed.h>
#include <parameters/param.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>
#include <battery/battery.h>

#include <conversion/rotation.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/sensor_preflight.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_magnetometer.h>

#include <DevMgr.hpp>

#include "parameters.h"
#include "rc_update.h"
#include "voted_sensors_update.h"

#ifdef BOARD_NUMBER_DIGITAL_BRICKS
#define TOTAL_BRICKS (BOARD_NUMBER_BRICKS + BOARD_NUMBER_DIGITAL_BRICKS)
#else
#define TOTAL_BRICKS BOARD_NUMBER_BRICKS
#endif

class Power
{
public:
	Power(sensors::Parameters *parameters);

	void update(px4_adc_msg_t buf_adc[PX4_MAX_ADC_CHANNELS], int nchannels);

private:

#if BOARD_NUMBER_BRICKS > 0
	Battery0 _battery0;
#endif
#if BOARD_NUMBER_BRICKS > 1
	// TODO: Change to Battery1
	Battery1 _battery1;
#endif

	BatteryBase *_analogBatteries[BOARD_NUMBER_BRICKS] {
#if BOARD_NUMBER_BRICKS > 0
		&_battery0,
#endif
#if BOARD_NUMBER_BRICKS > 1
		&_battery1,
#endif
	};

#if BOARD_NUMBER_BRICKS > 0
	orb_advert_t	_battery_pub[BOARD_NUMBER_BRICKS] {};			/**< battery status */

	Battery0		_battery[BOARD_NUMBER_BRICKS];			/**< Helper lib to publish battery_status topic. */
#endif /* BOARD_NUMBER_BRICKS > 0 */

#if BOARD_NUMBER_BRICKS > 1
	int 			_battery_pub_intance0ndx {0}; /**< track the index of instance 0 */
#endif /* BOARD_NUMBER_BRICKS > 1 */

	sensors::Parameters *_parameters;

};


#endif //PX4_POWER_H
