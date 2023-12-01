/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file sensors.hpp
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <lib/sensor_calibration/Utilities.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include "voted_sensors_update.h"
#include "vehicle_imu/VehicleIMU.hpp"

#if defined(CONFIG_SENSORS_VEHICLE_ACCELERATION)
# include "vehicle_acceleration/VehicleAcceleration.hpp"
#endif // CONFIG_SENSORS_VEHICLE_ACCELERATION

#if defined(CONFIG_SENSORS_VEHICLE_AIRSPEED)
# include <drivers/drv_sensor.h>
# include <drivers/drv_adc.h>
# include <lib/airspeed/airspeed.h>
# include <uORB/topics/airspeed.h>
# include <uORB/topics/differential_pressure.h>
# include <uORB/topics/vehicle_air_data.h>
#endif // CONFIG_SENSORS_VEHICLE_AIRSPEED

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
# include <uORB/topics/sensor_baro.h>
# include "vehicle_air_data/VehicleAirData.hpp"
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY)
# include "vehicle_angular_velocity/VehicleAngularVelocity.hpp"
#endif // CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
# include "vehicle_gps_position/VehicleGPSPosition.hpp"
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION

#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
# include "vehicle_magnetometer/VehicleMagnetometer.hpp"
# include <lib/sensor_calibration/Magnetometer.hpp>
# include <uORB/topics/sensor_mag.h>
#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER

#if defined(CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW)
# include "vehicle_optical_flow/VehicleOpticalFlow.hpp"
#endif // CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW

using namespace sensors;
using namespace time_literals;
/**
 * HACK - true temperature is much less than indicated temperature in baro,
 * subtract 5 degrees in an attempt to account for the electrical upheating of the PCB
 */
#define PCB_TEMP_ESTIMATE_DEG		5.0f
class Sensors : public ModuleBase<Sensors>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	explicit Sensors(bool hil_enabled);
	~Sensors() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:

	int		parameters_update();

	void		InitializeVehicleAirData();

	void		InitializeVehicleGPSPosition();

	void		InitializeVehicleIMU();

	void		InitializeVehicleMagnetometer();

	void		InitializeVehicleOpticalFlow();

	const bool _hil_enabled;	/**< if true, HIL is active */

	perf_counter_t	_loop_perf;	/**< loop performance counter */

	VehicleIMU *_vehicle_imu_list[MAX_SENSOR_COUNT] {};

	VotedSensorsUpdate _voted_sensors_update;

	sensor_combined_s _sensor_combined{};

	hrt_abstime     _last_config_update{0};
	hrt_abstime     _sensor_combined_prev_timestamp{0};

	uint8_t _n_accel{0};
	uint8_t _n_gyro{0};

	bool _armed{false};		/**< arming status of the vehicle */

	uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub[MAX_SENSOR_COUNT] {
		{this, ORB_ID(vehicle_imu), 0},
		{this, ORB_ID(vehicle_imu), 1},
		{this, ORB_ID(vehicle_imu), 2},
		{this, ORB_ID(vehicle_imu), 3}
	};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};

	uORB::Publication<sensor_combined_s> _sensor_pub{ORB_ID(sensor_combined)};

#if defined(CONFIG_SENSORS_VEHICLE_AIRSPEED)
	/**
	 * Poll the differential pressure sensor for updated data.
	 *
	 * @param raw	Combined sensor data structure into which
	 *		data should be returned.
	 */
	void diff_pres_poll();

	/**
	 * Poll the ADC and update readings to suit.
	 *
	 * @param raw	Combined sensor data structure into which
	 *		data should be returned.
	 */
	void adc_poll();

	uORB::Subscription _diff_pres_sub {ORB_ID(differential_pressure)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};

	uORB::Publication<airspeed_s>             _airspeed_pub{ORB_ID(airspeed)};

	DataValidator	_airspeed_validator;		/**< data validator to monitor airspeed */

	float _diff_pres_pressure_sum{0.f};
	float _diff_pres_temperature_sum{0.f};
	float _baro_pressure_sum{0.f};

	int _diff_pres_count{0};

	uint64_t _airspeed_last_publish{0};
	uint64_t _diff_pres_timestamp_sum{0};

# ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
	uORB::Subscription _adc_report_sub {ORB_ID(adc_report)};
	uORB::PublicationMulti<differential_pressure_s> _diff_pres_pub{ORB_ID(differential_pressure)};
# endif // ADC_AIRSPEED_VOLTAGE_CHANNEL

	struct Parameters {
		float diff_pres_offset_pa;
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
		float diff_pres_analog_scale;
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

		int32_t air_cmodel;
		float air_tube_length;
		float air_tube_diameter_mm;
	} _parameters{}; /**< local copies of interesting parameters */

	struct ParameterHandles {
		param_t diff_pres_offset_pa;
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
		param_t diff_pres_analog_scale;
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

		param_t air_cmodel;
		param_t air_tube_length;
		param_t air_tube_diameter_mm;
	} _parameter_handles{};		/**< handles for interesting parameters */
#endif // CONFIG_SENSORS_VEHICLE_AIRSPEED

#if defined(CONFIG_SENSORS_VEHICLE_ACCELERATION)
	VehicleAcceleration _vehicle_acceleration;
#endif // CONFIG_SENSORS_VEHICLE_ACCELERATION

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
	VehicleAirData *_vehicle_air_data {nullptr};
	uint8_t _n_baro{0};
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY)
	VehicleAngularVelocity	_vehicle_angular_velocity;
#endif // CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY

#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
	VehicleMagnetometer *_vehicle_magnetometer {nullptr};
	uint8_t _n_mag{0};
#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
	VehicleGPSPosition *_vehicle_gps_position {nullptr};
	uint8_t _n_gps{0};
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION

#if defined(CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW)
	VehicleOpticalFlow *_vehicle_optical_flow {nullptr};
	uint8_t _n_optical_flow{0};
#endif // CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW

	DEFINE_PARAMETERS(
#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
		(ParamBool<px4::params::SYS_HAS_BARO>) _param_sys_has_baro,
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA
#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
		(ParamBool<px4::params::SYS_HAS_GPS>) _param_sys_has_gps,
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION
#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
		(ParamInt<px4::params::SYS_HAS_MAG>) _param_sys_has_mag,
#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER
		(ParamBool<px4::params::SENS_IMU_MODE>) _param_sens_imu_mode
	)
};
