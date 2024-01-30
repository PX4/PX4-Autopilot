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
 * @file sensors.cpp
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include "sensors.hpp"

Sensors::Sensors(bool hil_enabled) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_hil_enabled(hil_enabled),
	_loop_perf(perf_alloc(PC_ELAPSED, "sensors")),
	_voted_sensors_update(hil_enabled, _vehicle_imu_sub)
{
	_sensor_pub.advertise();

#if defined(CONFIG_SENSORS_VEHICLE_ACCELERATION)
	_vehicle_acceleration.Start();
#endif // CONFIG_SENSORS_VEHICLE_ACCELERATION

#if defined(CONFIG_SENSORS_VEHICLE_AIRSPEED)
	/* Differential pressure offset */
	_parameter_handles.diff_pres_offset_pa = param_find("SENS_DPRES_OFF");
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
	_parameter_handles.diff_pres_analog_scale = param_find("SENS_DPRES_ANSC");
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

	_parameter_handles.air_cmodel = param_find("CAL_AIR_CMODEL");
	_parameter_handles.air_tube_length = param_find("CAL_AIR_TUBELEN");
	_parameter_handles.air_tube_diameter_mm = param_find("CAL_AIR_TUBED_MM");

	_airspeed_validator.set_timeout(300000);
	_airspeed_validator.set_equal_value_threshold(100);
#endif // CONFIG_SENSORS_VEHICLE_AIRSPEED

#if defined(CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY)
	_vehicle_angular_velocity.Start();
#endif // CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY

	param_find("SYS_FAC_CAL_MODE");

	// Parameters controlling the on-board sensor thermal calibrator
	param_find("SYS_CAL_TDEL");
	param_find("SYS_CAL_TMAX");
	param_find("SYS_CAL_TMIN");

	_sensor_combined.accelerometer_timestamp_relative = sensor_combined_s::RELATIVE_TIMESTAMP_INVALID;

	parameters_update();

	InitializeVehicleIMU();
}

Sensors::~Sensors()
{
	// clear all registered callbacks
	for (auto &sub : _vehicle_imu_sub) {
		sub.unregisterCallback();
	}

#if defined(CONFIG_SENSORS_VEHICLE_ACCELERATION)
	_vehicle_acceleration.Stop();
#endif // CONFIG_SENSORS_VEHICLE_ACCELERATION

#if defined(CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY)
	_vehicle_angular_velocity.Stop();
#endif // CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)

	if (_vehicle_air_data) {
		_vehicle_air_data->Stop();
		delete _vehicle_air_data;
	}

#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)

	if (_vehicle_gps_position) {
		_vehicle_gps_position->Stop();
		delete _vehicle_gps_position;
	}

#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION

#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)

	if (_vehicle_magnetometer) {
		_vehicle_magnetometer->Stop();
		delete _vehicle_magnetometer;
	}

#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER

#if defined(CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW)

	if (_vehicle_optical_flow) {
		_vehicle_optical_flow->Stop();
		delete _vehicle_optical_flow;
	}

#endif // CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW

	for (auto &vehicle_imu : _vehicle_imu_list) {
		if (vehicle_imu) {
			vehicle_imu->Stop();
			delete vehicle_imu;
		}
	}

	perf_free(_loop_perf);
}

bool Sensors::init()
{
	_vehicle_imu_sub[0].registerCallback();
	ScheduleNow();
	return true;
}

int Sensors::parameters_update()
{
	if (_armed) {
		return 0;
	}

#if defined(CONFIG_SENSORS_VEHICLE_AIRSPEED)
	/* Airspeed offset */
	param_get(_parameter_handles.diff_pres_offset_pa, &(_parameters.diff_pres_offset_pa));
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
	param_get(_parameter_handles.diff_pres_analog_scale, &(_parameters.diff_pres_analog_scale));
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

	param_get(_parameter_handles.air_cmodel, &_parameters.air_cmodel);
	param_get(_parameter_handles.air_tube_length, &_parameters.air_tube_length);
	param_get(_parameter_handles.air_tube_diameter_mm, &_parameters.air_tube_diameter_mm);
#endif // CONFIG_SENSORS_VEHICLE_AIRSPEED

	_voted_sensors_update.parametersUpdate();

	// 1. mark all existing sensor calibrations active even if sensor is missing
	//    this preserves the calibration in the event of a parameter export while the sensor is missing
	// 2. ensure calibration slots are active for the number of sensors currently available
	//    this to done to eliminate differences in the active set of parameters before and after sensor calibration
	for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		// sensor_accel
		{
			const uint32_t device_id_accel = calibration::GetCalibrationParamInt32("ACC",  "ID", i);

			if (device_id_accel != 0) {
				calibration::Accelerometer accel_cal(device_id_accel);
			}

			uORB::SubscriptionData<sensor_accel_s> sensor_accel_sub{ORB_ID(sensor_accel), i};

			if (sensor_accel_sub.advertised() && (sensor_accel_sub.get().device_id != 0)) {
				calibration::Accelerometer cal;
				cal.set_calibration_index(i);
				cal.ParametersLoad();
			}
		}


		// sensor_gyro
		{
			const uint32_t device_id_gyro = calibration::GetCalibrationParamInt32("GYRO", "ID", i);

			if (device_id_gyro != 0) {
				calibration::Gyroscope gyro_cal(device_id_gyro);
			}

			uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

			if (sensor_gyro_sub.advertised() && (sensor_gyro_sub.get().device_id != 0)) {
				calibration::Gyroscope cal;
				cal.set_calibration_index(i);
				cal.ParametersLoad();
			}
		}


#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
		// sensor_mag
		{
			uint32_t device_id_mag = calibration::GetCalibrationParamInt32("MAG",  "ID", i);

			if (device_id_mag != 0) {
				calibration::Magnetometer mag_cal(device_id_mag);
			}

			uORB::SubscriptionData<sensor_mag_s> sensor_mag_sub{ORB_ID(sensor_mag), i};

			if (sensor_mag_sub.advertised() && (sensor_mag_sub.get().device_id != 0)) {
				calibration::Magnetometer cal;
				cal.set_calibration_index(i);
				cal.ParametersLoad();
			}
		}
#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER
	}

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
	InitializeVehicleAirData();
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
	InitializeVehicleGPSPosition();
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION

#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
	InitializeVehicleMagnetometer();
#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER

#if defined(CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW)
	InitializeVehicleOpticalFlow();
#endif // CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW

	return PX4_OK;
}

#if defined(CONFIG_SENSORS_VEHICLE_AIRSPEED)
void Sensors::diff_pres_poll()
{
	differential_pressure_s diff_pres{};

	if (_diff_pres_sub.update(&diff_pres)) {

		if (!PX4_ISFINITE(diff_pres.differential_pressure_pa)) {
			// ignore invalid data and reset accumulated

			// reset
			_diff_pres_timestamp_sum = 0;
			_diff_pres_pressure_sum = 0;
			_diff_pres_temperature_sum = 0;
			_baro_pressure_sum = 0;
			_diff_pres_count = 0;
			return;
		}

		vehicle_air_data_s air_data{};
		_vehicle_air_data_sub.copy(&air_data);

		float air_temperature_celsius = NAN;

		// assume anything outside of a (generous) operating range of -40C to 125C is invalid
		if (PX4_ISFINITE(diff_pres.temperature) && (diff_pres.temperature >= -40.f) && (diff_pres.temperature <= 125.f)) {

			air_temperature_celsius = diff_pres.temperature;

		} else {
			// differential pressure temperature invalid, check barometer
			if ((air_data.timestamp != 0) && PX4_ISFINITE(air_data.baro_temp_celcius)
			    && (air_data.baro_temp_celcius >= -40.f) && (air_data.baro_temp_celcius <= 125.f)) {

				// TODO: review PCB_TEMP_ESTIMATE_DEG, ignore for external baro
				air_temperature_celsius = air_data.baro_temp_celcius - PCB_TEMP_ESTIMATE_DEG;
			}
		}

		// push raw data into validator
		float airspeed_input[3] { diff_pres.differential_pressure_pa, air_temperature_celsius, 0.0f };
		_airspeed_validator.put(diff_pres.timestamp_sample, airspeed_input, diff_pres.error_count, 100); // TODO: real priority?

		// accumulate average for publication
		_diff_pres_timestamp_sum += diff_pres.timestamp_sample;
		_diff_pres_pressure_sum += diff_pres.differential_pressure_pa;
		_diff_pres_temperature_sum += air_temperature_celsius;
		_baro_pressure_sum += air_data.baro_pressure_pa;
		_diff_pres_count++;

		if ((_diff_pres_count > 0) && hrt_elapsed_time(&_airspeed_last_publish) >= 50_ms) {

			// average data and apply calibration offset (SENS_DPRES_OFF)
			const uint64_t timestamp_sample = _diff_pres_timestamp_sum / _diff_pres_count;
			const float differential_pressure_pa = _diff_pres_pressure_sum / _diff_pres_count - _parameters.diff_pres_offset_pa;
			const float baro_pressure_pa = _baro_pressure_sum / _diff_pres_count;
			const float temperature = _diff_pres_temperature_sum / _diff_pres_count;

			// reset
			_diff_pres_timestamp_sum = 0;
			_diff_pres_pressure_sum = 0;
			_diff_pres_temperature_sum = 0;
			_baro_pressure_sum = 0;
			_diff_pres_count = 0;


			enum AIRSPEED_SENSOR_MODEL smodel;

			switch ((diff_pres.device_id >> 16) & 0xFF) {
			case DRV_DIFF_PRESS_DEVTYPE_SDP31:

			// fallthrough
			case DRV_DIFF_PRESS_DEVTYPE_SDP32:

			// fallthrough
			case DRV_DIFF_PRESS_DEVTYPE_SDP33:
				smodel = AIRSPEED_SENSOR_MODEL_SDP3X;
				break;

			default:
				smodel = AIRSPEED_SENSOR_MODEL_MEMBRANE;
				break;
			}

			float indicated_airspeed_m_s = calc_IAS_corrected((enum AIRSPEED_COMPENSATION_MODEL)_parameters.air_cmodel,
						       smodel, _parameters.air_tube_length, _parameters.air_tube_diameter_mm,
						       differential_pressure_pa, baro_pressure_pa, temperature);

			// assume that CAS = IAS as we don't have an CAS-scale here
			float true_airspeed_m_s = calc_TAS_from_CAS(indicated_airspeed_m_s, baro_pressure_pa, temperature);

			if (PX4_ISFINITE(indicated_airspeed_m_s) && PX4_ISFINITE(true_airspeed_m_s)) {

				airspeed_s airspeed;
				airspeed.timestamp_sample = timestamp_sample;
				airspeed.indicated_airspeed_m_s = indicated_airspeed_m_s;
				airspeed.true_airspeed_m_s = true_airspeed_m_s;
				airspeed.air_temperature_celsius = temperature;
				airspeed.confidence = _airspeed_validator.confidence(hrt_absolute_time());
				airspeed.timestamp = hrt_absolute_time();
				_airspeed_pub.publish(airspeed);

				_airspeed_last_publish = airspeed.timestamp;
			}
		}
	}
}

void Sensors::adc_poll()
{
	/* only read if not in HIL mode */
	if (_hil_enabled) {
		return;
	}

#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL

	if (_parameters.diff_pres_analog_scale > 0.0f) {
		adc_report_s adc;

		if (_adc_report_sub.update(&adc)) {
			/* Read add channels we got */
			for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; i++) {
				if (adc.channel_id[i] == -1) {
					continue;	// skip non-exist channels
				}

				if (ADC_AIRSPEED_VOLTAGE_CHANNEL == adc.channel_id[i]) {

					/* calculate airspeed, raw is the difference from */
					const float voltage = (float)(adc.raw_data[i]) * adc.v_ref / adc.resolution * ADC_DP_V_DIV;

					/**
					 * The voltage divider pulls the signal down, only act on
					 * a valid voltage from a connected sensor. Also assume a non-
					 * zero offset from the sensor if its connected.
					 *
					 * Notice: This won't work on devices which have PGA controlled
					 * vref. Those devices require no divider at all.
					 */
					if (voltage > 0.4f) {
						const float diff_pres_pa_raw = voltage * _parameters.diff_pres_analog_scale;

						differential_pressure_s diff_pres{};
						diff_pres.timestamp_sample = adc.timestamp;
						diff_pres.differential_pressure_pa = diff_pres_pa_raw;
						diff_pres.temperature = NAN;
						diff_pres.timestamp = hrt_absolute_time();

						_diff_pres_pub.publish(diff_pres);
					}
				}
			}
		}
	}

#endif // ADC_AIRSPEED_VOLTAGE_CHANNEL
}
#endif // CONFIG_SENSORS_VEHICLE_AIRSPEED)

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
void Sensors::InitializeVehicleAirData()
{
	if (_param_sys_has_baro.get()) {
		if (_vehicle_air_data == nullptr) {
			_vehicle_air_data = new VehicleAirData();

			if (_vehicle_air_data) {
				_vehicle_air_data->Start();
			}
		}
	}
}
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
void Sensors::InitializeVehicleGPSPosition()
{
	if (_param_sys_has_gps.get()) {
		if (_vehicle_gps_position == nullptr) {
			_vehicle_gps_position = new VehicleGPSPosition();

			if (_vehicle_gps_position) {
				_vehicle_gps_position->Start();
			}
		}
	}
}
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION

void Sensors::InitializeVehicleIMU()
{
	// create a VehicleIMU instance for each accel/gyro pair
	for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (_vehicle_imu_list[i] == nullptr) {

			uORB::Subscription accel_sub{ORB_ID(sensor_accel), i};
			uORB::Subscription gyro_sub{ORB_ID(sensor_gyro), i};

			if (accel_sub.advertised() && gyro_sub.advertised()) {
				// if the sensors module is responsible for voting (SENS_IMU_MODE 1) then run every VehicleIMU in the same WQ
				//   otherwise each VehicleIMU runs in a corresponding INSx WQ
				const bool multi_mode = (_param_sens_imu_mode.get() == 0);
				const px4::wq_config_t &wq_config = multi_mode ? px4::ins_instance_to_wq(i) : px4::wq_configurations::INS0;

				VehicleIMU *imu = new VehicleIMU(i, i, i, wq_config);

				if (imu != nullptr) {
					// Start VehicleIMU instance and store
					if (imu->Start()) {
						_vehicle_imu_list[i] = imu;

					} else {
						delete imu;
					}
				}

			} else {
				// abort on first failure, try again later
				return;
			}
		}
	}
}

#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
void Sensors::InitializeVehicleMagnetometer()
{
	if (_param_sys_has_mag.get()) {
		if (_vehicle_magnetometer == nullptr) {
			_vehicle_magnetometer = new VehicleMagnetometer();

			if (_vehicle_magnetometer) {
				_vehicle_magnetometer->Start();
			}
		}
	}
}
#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER

#if defined(CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW)
void Sensors::InitializeVehicleOpticalFlow()
{
	if (_vehicle_optical_flow == nullptr) {
		uORB::Subscription sensor_optical_flow_sub{ORB_ID(sensor_optical_flow)};

		if (sensor_optical_flow_sub.advertised()) {
			_vehicle_optical_flow = new VehicleOpticalFlow();

			if (_vehicle_optical_flow) {
				_vehicle_optical_flow->Start();
			}
		}
	}
}
#endif // CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW

void Sensors::Run()
{
	if (should_exit()) {
		// clear all registered callbacks
		for (auto &sub : _vehicle_imu_sub) {
			sub.unregisterCallback();
		}

		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// check vehicle status for changes to publication state
	if (_vcontrol_mode_sub.updated()) {
		vehicle_control_mode_s vcontrol_mode{};

		if (_vcontrol_mode_sub.copy(&vcontrol_mode)) {
			_armed = vcontrol_mode.flag_armed;
		}
	}

	// keep adding sensors as long as we are not armed,
	// when not adding sensors poll for param updates
	if ((!_armed && hrt_elapsed_time(&_last_config_update) > 500_ms) || (_last_config_update == 0)) {

		bool updated = false;

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
		const int n_baro = orb_group_count(ORB_ID(sensor_baro));

		if (n_baro != _n_baro) {
			_n_baro = n_baro;
			updated = true;
		}

#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
		const int n_gps = orb_group_count(ORB_ID(sensor_gps));

		if (n_gps != _n_gps) {
			_n_gps = n_gps;
			updated = true;
		}

#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION

#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
		const int n_mag = orb_group_count(ORB_ID(sensor_mag));

		if (n_mag != _n_mag) {
			_n_mag = n_mag;
			updated = true;
		}

#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER

#if defined(CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW)
		const int n_optical_flow = orb_group_count(ORB_ID(sensor_optical_flow));

		if (n_optical_flow != _n_optical_flow) {
			_n_optical_flow = n_optical_flow;
			updated = true;
		}

#endif // CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW


		const int n_accel = orb_group_count(ORB_ID(sensor_accel));
		const int n_gyro  = orb_group_count(ORB_ID(sensor_gyro));

		if ((n_accel != _n_accel) || (n_gyro != _n_gyro) || updated) {
			_n_accel = n_accel;
			_n_gyro = n_gyro;

			parameters_update();
		}

		// sensor device id (not just orb_group_count) must be populated before IMU init can succeed
		_voted_sensors_update.initializeSensors();
		InitializeVehicleIMU();

		_last_config_update = hrt_absolute_time();

	} else {
		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			parameters_update();
			updateParams();
		}
	}

	_voted_sensors_update.sensorsPoll(_sensor_combined);

	if (_sensor_combined.timestamp != _sensor_combined_prev_timestamp) {

		_voted_sensors_update.setRelativeTimestamps(_sensor_combined);
		_sensor_pub.publish(_sensor_combined);
		_sensor_combined_prev_timestamp = _sensor_combined.timestamp;
	}

#if defined(CONFIG_SENSORS_VEHICLE_AIRSPEED)
	// check analog airspeed
	adc_poll();
	diff_pres_poll();
#endif // CONFIG_SENSORS_VEHICLE_AIRSPEED

	// backup schedule as a watchdog timeout
	ScheduleDelayed(10_ms);

	perf_end(_loop_perf);
}

int Sensors::task_spawn(int argc, char *argv[])
{
	bool hil_enabled = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "h", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'h':
			hil_enabled = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	Sensors *instance = new Sensors(hil_enabled);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Sensors::print_status()
{
	_voted_sensors_update.printStatus();

#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)

	if (_vehicle_magnetometer) {
		PX4_INFO_RAW("\n");
		_vehicle_magnetometer->PrintStatus();
	}

#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)

	if (_vehicle_air_data) {
		PX4_INFO_RAW("\n");
		_vehicle_air_data->PrintStatus();
	}

#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_AIRSPEED)
	PX4_INFO_RAW("\n");
	PX4_INFO_RAW("Airspeed status:\n");
	_airspeed_validator.print();
#endif // CONFIG_SENSORS_VEHICLE_AIRSPEED

#if defined(CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW)

	if (_vehicle_optical_flow) {
		PX4_INFO_RAW("\n");
		_vehicle_optical_flow->PrintStatus();
	}

#endif // CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW

#if defined(CONFIG_SENSORS_VEHICLE_ACCELERATION)
	PX4_INFO_RAW("\n");
	_vehicle_acceleration.PrintStatus();
#endif // CONFIG_SENSORS_VEHICLE_ACCELERATION

#if defined(CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY)
	PX4_INFO_RAW("\n");
	_vehicle_angular_velocity.PrintStatus();
#endif // CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)

	if (_vehicle_gps_position) {
		PX4_INFO_RAW("\n");
		_vehicle_gps_position->PrintStatus();
	}

#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION

	PX4_INFO_RAW("\n");

	for (auto &i : _vehicle_imu_list) {
		if (i != nullptr) {
			PX4_INFO_RAW("\n");
			i->PrintStatus();
		}
	}

	return 0;
}

int Sensors::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Sensors::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The sensors module is central to the whole system. It takes low-level output from drivers, turns
it into a more usable form, and publishes it for the rest of the system.

The provided functionality includes:
- Read the output from the sensor drivers (`SensorGyro`, etc.).
  If there are multiple of the same type, do voting and failover handling.
  Then apply the board rotation and temperature calibration (if enabled). And finally publish the data; one of the
  topics is `SensorCombined`, used by many parts of the system.
- Make sure the sensor drivers get the updated calibration parameters (scale & offset) when the parameters change or
  on startup. The sensor drivers use the ioctl interface for parameter updates. For this to work properly, the
  sensor drivers must already be running when `sensors` is started.
- Do sensor consistency checks and publish the `SensorsStatusImu` topic.

### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensors", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Start in HIL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sensors_main(int argc, char *argv[])
{
	return Sensors::main(argc, argv);
}
