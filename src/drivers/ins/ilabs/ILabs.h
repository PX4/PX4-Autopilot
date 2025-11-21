/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * Driver for the InertialLabs INS, AHRS
 */

#pragma once

#include <stdio.h>

#include <drivers/accelerometer/PX4Accelerometer.hpp>
#include <drivers/device/Device.hpp>
#include <drivers/gyroscope/PX4Gyroscope.hpp>
#include <drivers/magnetometer/PX4Magnetometer.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

#include "sensor.h"

class ILabs : public ModuleBase<ILabs>, public ModuleParams, public px4::ScheduledWorkItem {
public:
	ILabs(const char *port);
	~ILabs() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	void        Run() override;
	void        processData(InertialLabs::SensorsData *sensordata);
	static void processDataProxy(void *context, InertialLabs::SensorsData *data) {
		if (!context || !data) {
			return;
		}

		ILabs *self = static_cast<ILabs *>(context);
		self->processData(data);
	}

	InertialLabs::Sensor _sensor{};

	char _serialDeviceName[20]{};
	InertialLabs::AverageSensorsData _average_sensors_data{};
	device::Device::DeviceId _device_id{};

	px4::atomic<hrt_abstime> _time_initialized{0};
	px4::atomic<hrt_abstime> _time_last_valid_imu_data{0};

	PX4Accelerometer _px4_accel{0};
	PX4Gyroscope     _px4_gyro{0};
	PX4Magnetometer  _px4_mag{0};

	MapProjection _pos_ref{};

	uORB::PublicationMulti<vehicle_attitude_s>        _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::PublicationMulti<vehicle_local_position_s>  _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::PublicationMulti<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::PublicationMulti<sensor_baro_s>             _sensor_baro_pub{ORB_ID(sensor_baro)};
	uORB::PublicationMulti<sensor_gps_s>              _sensor_gps_pub{ORB_ID(sensor_gps)};
	uORB::Publication<sensor_selection_s>             _sensor_selection_pub{ORB_ID(sensor_selection)};
	uORB::Publication<estimator_status_s>             _estimator_status_pub{ORB_ID(estimator_status)};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME ": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": read")};

	perf_counter_t _accel_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": accel publish interval")};
	perf_counter_t _gyro_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": gyro publish interval")};
	perf_counter_t _mag_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": mag publish interval")};
	perf_counter_t _gnss_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": GNSS publish interval")};
	perf_counter_t _baro_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": baro publish interval")};

	perf_counter_t _attitude_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": attitude publish interval")};
	perf_counter_t _local_position_pub_interval_perf{
	    perf_alloc(PC_INTERVAL, MODULE_NAME ": local position publish interval")};
	perf_counter_t _global_position_pub_interval_perf{
	    perf_alloc(PC_INTERVAL, MODULE_NAME ": global position publish interval")};

	DEFINE_PARAMETERS((ParamInt<px4::params::ILABS_MODE>)_param_ilabs_mode)
};
