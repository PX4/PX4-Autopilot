/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 *
 * Driver for the VectorNav VN-100, VN-200, VN-300 series
 */

#pragma once

#include <stdio.h>
#include <inttypes.h>
#include <termios.h>

extern "C" {
#include "vn/sensors/searcher.h"

#include "vn/sensors/compositedata.h"
}

#include "vn/sensors.h"

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>

using namespace time_literals;

class VectorNav : public ModuleBase<VectorNav>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VectorNav(const char *port);
	~VectorNav() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	bool init();
	bool configure();

	void Run() override;

	static void binaryAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex);

	// return the square of two floating point numbers
	static constexpr float sq(float var) { return var * var; }

	void sensorCallback(VnUartPacket *packet);

	char _port[20] {};

	bool _initialized{false};

	bool _connected{false};
	bool _configured{false};

	px4::atomic<hrt_abstime> _time_configured_us{false};
	px4::atomic<hrt_abstime> _time_last_valid_imu_us{false};

	VnSensor _vs{};

	BinaryOutputRegister _binary_output_group_1{};
	BinaryOutputRegister _binary_output_group_2{};
	BinaryOutputRegister _binary_output_group_3{};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	PX4Accelerometer _px4_accel{0};
	PX4Gyroscope     _px4_gyro{0};
	PX4Magnetometer  _px4_mag{0};

	MapProjection _pos_ref{};
	float _gps_alt_ref{NAN};		///< WGS-84 height (m)

	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};
	uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub{ORB_ID(sensor_gps)};

	uORB::Publication<sensor_selection_s> _sensor_selection_pub{ORB_ID(sensor_selection)};

	uORB::PublicationMulti<vehicle_attitude_s> _attitude_pub;
	uORB::PublicationMulti<vehicle_local_position_s> _local_position_pub;
	uORB::PublicationMulti<vehicle_global_position_s> _global_position_pub;

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

	perf_counter_t _accel_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": accel publish interval")};
	perf_counter_t _gyro_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": gyro publish interval")};
	perf_counter_t _mag_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": mag publish interval")};
	perf_counter_t _gnss_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": GNSS publish interval")};
	perf_counter_t _baro_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": baro publish interval")};

	perf_counter_t _attitude_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": attitude publish interval")};
	perf_counter_t _local_position_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": local position publish interval")};
	perf_counter_t _global_position_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": global position publish interval")};


	// TODO: params for GNSS antenna offsets
	// A
	// B

	// As mentioned previously, the VN-300 has a factory default baseline of {1, 0, 0} [m]. This vector represents
	// the position of a point on GNSS antenna B relative to the same point on GNSS antenna A in the output
	// coordinate system on the VN-300. The default output coordinate system is engraved on the top of the
	// aluminum enclosure. For the factory default case, GNSS antenna B should be positioned in front of GNSS
	// antenna A relative to the X-axis marked on the VN-300 enclosure as shown in the figure below. If a different
	// baseline length or direction required, then you will need to write the new baseline vector and the measurement
	// uncertainty to the sensor using the GNSS Compass Baseline Register.

	// GNSS Antenna A Offset
	//  Relative position of GNSS antenna. (X-axis)
	// VN_GNSS_ANTA_POS_X


	// Relative position of GNSS antenna. (X-axis)
	// VN_GNSS_ANTB_POS_X

	// Uncertainty in the X-axis position measurement.

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VN_MODE>) _param_vn_mode
	)
};
