/****************************************************************************
 *
 *   Copyright (c) 2012-2024 PX4 Development Team. All rights reserved.
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
 * @file failure_detector_HITL.hpp
 *
 * @author Ilia Loginov <ilia.loginov@tii.ae>
 */

#pragma once

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_air_data.h>

using namespace time_literals;

/**
 * @brief FailureDetectorHITL class
 *
 * This class tracks cmd vehicle command \ref VEHICLE_CMD_INJECT_FAILURE and provide information about current state.
 * It is expected to work only in HITL mode.
 *
 * @note
 * Supported message:
 * \ref FAILURE_UNIT_SENSOR_GPS
 * \ref FAILURE_UNIT_SENSOR_MAG
 * \ref FAILURE_UNIT_SENSOR_BARO
 *
 * Supported commands:
 * \ref FAILURE_TYPE_OK
 * \ref FAILURE_TYPE_FAIL
 * \ref FAILURE_TYPE_STUCK(only \ref FAILURE_UNIT_SENSOR_GPS and \ref FAILURE_UNIT_SENSOR_BARO)
 */
class FailureDetectorHITL final
{
public:
	FailureDetectorHITL(bool hil_enabled);
	bool update();

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
	bool isGpsOk() const;
	bool isGpsOff() const;
	bool isGpsStuck() const;
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
	bool isBaroOk() const;
	bool isBaroOff() const;
	bool isBaroStuck() const;
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
	bool isMagOk() const;
	bool isMagOff() const;
#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER

private:
	enum class FailureStatus {
		ok,
		off,
		stuck
	};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
	FailureStatus _gps {FailureStatus::ok};
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
	FailureStatus _baro {FailureStatus::ok};
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
	FailureStatus _mag {FailureStatus::ok};
#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER
};


namespace sensors
{

/**
 * @brief FakeStuckSensor class
 *
 * This class implements the simulation of a sensor experiencing a "stuck" state,
 * where the sensor's last recorded data is continuously published at regular intervals,
 * mimicking a sensor that has stopped updating its readings.
 * It is expected to work only in HITL mode.
 *
 * @tparam sensorsData Type of the sensor data to be published.
 */
template <typename sensorsData>
class FakeStuckSensor final: public ModuleParams, public px4::ScheduledWorkItem
{
public:
	/**
	 * @brief Constructor for FakeStuckSensor class.
	 * @param meta ORB object metadata for the sensor data topic.
	 */
	FakeStuckSensor(const orb_metadata *meta):
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
		_sensor_sub(meta),
		meta_(meta)
	{
		pthread_mutex_init(&_mutex, nullptr);
		PX4_INFO("Fake stuck sensor %s:id(%d) was created with ", meta->o_name, meta->o_id);
	}

	~FakeStuckSensor() override
	{
		setEnabled(false);
		stop();
		perf_free(_cycle_perf);
	}

	bool start()
	{
		ScheduleNow();
		return true;
	}

	void stop()
	{
		Deinit();
	}

	void setEnabled(bool enabled)
	{
		pthread_mutex_lock(&_mutex);

		if (_enabled && !enabled) {
			delete _sensor_pub;

		} else if (!_enabled && enabled) {
			_sensor_pub = new uORB::Publication<sensorsData>(meta_);
		}

		PX4_INFO("Fake stuck sensor %s was %s", meta_->o_name, enabled ? "enabled" : "disabled");
		_enabled = enabled;

		pthread_mutex_unlock(&_mutex);
	}
private:
	void Run() override
	{
		perf_begin(_cycle_perf);

		while (_sensor_sub.update(&_last_output)) {
			_first_init = true;
		}

		pthread_mutex_lock(&_mutex);

		if (_enabled && _first_init) {
			_last_output.timestamp = hrt_absolute_time();
			_sensor_pub->publish(_last_output);
		}

		pthread_mutex_unlock(&_mutex);

		ScheduleDelayed(300_ms); // backup schedule

		perf_end(_cycle_perf);
	}

	uORB::Publication<sensorsData> *_sensor_pub {};
	uORB::Subscription _sensor_sub {};

	const orb_metadata *meta_;
	bool _enabled{};
	bool _first_init{}; /**< Flag indicating whether the sensor has been initialized for the first time. */
	sensorsData _last_output{};
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	pthread_mutex_t	_mutex {};
};

}

/**
 * @brief FakeSensors class
 *
 * This class represents a collection of fake sensors used for simulation purposes.
 * It is expected to work only in HITL mode.
 */
class FakeSensors final
{
public:
	FakeSensors(bool hil_enabled);

	/**
	 * @brief Updates states of the fake sensors with data from the failure detector.
	 */
	void update(const FailureDetectorHITL &detector);
private:

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
	sensors::FakeStuckSensor<vehicle_air_data_s> _fake_baro_publisher {ORB_ID(vehicle_air_data)};
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
	sensors::FakeStuckSensor<sensor_gps_s> _fake_gps_publisher {ORB_ID(vehicle_gps_position)};
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION
};
