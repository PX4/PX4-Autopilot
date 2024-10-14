/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/getopt.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_global_position.h>



#include "mip_sdk/src/mip/mip_all.h"
#include "mip_sdk/src/mip/mip_interface.h"
#include "mip_sdk/src/mip/utils/serial_port.h"

#include "mip_sdk/src/mip/definitions/commands_aiding.h"

#include <containers/Array.hpp>

using namespace mip::C;

using namespace time_literals;

#include "modal_io_serial.hpp"

class MicrosStrain : public ModuleBase<MicrosStrain>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MicrosStrain(const char *device, int32_t rotation);
	~MicrosStrain() override;

	/* Callbacks */

	// Sensor Callbacks
	static void sensor_callback(void *user, const mip_packet *packet, mip::Timestamp timestamp);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	bool init();

	uint32_t _debug_rx_bytes[4] {0};
	uint32_t _debug_tx_bytes{0};

	uint64_t _delay_offset{0};

private:
	/** @see ModuleBase */
	void Run() override;

	/// @brief Attempt to connect to the Sensor and set in known configuration
	void initialize_ins();

	void set_sensor_rate(mip_descriptor_rate *sensor_descriptors, uint16_t len);

	int connect_at_baud(int32_t baud);

	void apply_mag_cal();

	enum Rotation rotation = ROTATION_NONE;
	uint32_t dev_id{0};

	// Sensor types needed for message creation / updating / publishing
	PX4Accelerometer _px4_accel{0};
	PX4Gyroscope _px4_gyro{0};
	PX4Magnetometer _px4_mag{0};
	sensor_baro_s _sensor_baro{0};

	uORB::Publication<vehicle_local_position_s> _vehicle_local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_angular_velocity_s> _vehicle_angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};
	uORB::Publication<vehicle_attitude_s> _vehicle_attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_odometry_s> _vehicle_odometry_pub{ORB_ID(vehicle_odometry)};
	uORB::Publication<debug_array_s> _debug_array_pub{ORB_ID(debug_array)};

	// Needed for health checks
	uORB::Publication<estimator_status_s> _estimator_status_pub{ORB_ID(estimator_status)};

	// Must publish to prevent sensor stale failure (sensors module)
	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};
	uORB::Publication<sensor_selection_s> _sensor_selection_pub{ORB_ID(sensor_selection)};

	// Subscriptions
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription                 _sensor_gps_sub{ORB_ID(sensor_gps)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	uint8_t parse_buffer[2048];
	bool _is_initialized{false};
	bool _is_init_failed{false};

	/******************************/
	mip::C::mip_interface device;

	// Handlers
	mip_dispatch_handler sensor_data_handler;
	mip_dispatch_handler filter_data_handler;

	char _port[20];

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MS_SCHEDULE>) _param_ms_schedule,
		(ParamInt<px4::params::SENSOR_DELAY>) _param_sensor_delay,
		(ParamInt<px4::params::PR_UPDATE_RATE>) _param_pr_update_rate,
		(ParamInt<px4::params::SEC_UPDATE_RATE>) _param_sec_update_rate

	)

};
