/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "VehicleMagnetometer.hpp"

#include <px4_platform_common/log.h>
#include <lib/ecl/geo/geo.h>
#include <drivers/drv_mag.h>

#if !defined(CONSTRAINED_FLASH)
// status debugging
#include <lib/ecl/geo_lookup/geo_mag_declination.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#endif // !CONSTRAINED_FLASH

using namespace matrix;
using namespace time_literals;
using math::radians;

static constexpr int32_t MAG_ROT_VAL_INTERNAL{-1};

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

VehicleMagnetometer::VehicleMagnetometer() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::navigation_and_controllers)
{
	_voter.set_timeout(SENSOR_TIMEOUT);
	_voter.set_equal_value_threshold(1000);

	ParametersUpdate(true);
}

VehicleMagnetometer::~VehicleMagnetometer()
{
	Stop();

	perf_free(_cycle_perf);
}

bool VehicleMagnetometer::Start()
{
	ScheduleNow();

	return true;
}

void VehicleMagnetometer::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

void VehicleMagnetometer::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		// fine tune the rotation
		const Dcmf board_rotation_offset{Eulerf{
				radians(_param_sens_board_x_off.get()),
				radians(_param_sens_board_y_off.get()),
				radians(_param_sens_board_z_off.get())}};

		// get transformation matrix from sensor/board to body frame
		const Dcmf board_rotation = board_rotation_offset * get_rot_matrix((enum Rotation)_param_sens_board_rot.get());

		for (int cal_index = 0; cal_index < MAX_SENSOR_COUNT; cal_index++) {
			char str[30] {};
			// ID
			(void)sprintf(str, "CAL_MAG%u_ID", cal_index);
			int32_t device_id = 0;
			param_get(param_find(str), &device_id);

			// apply calibration
			for (int dev_index = 0; dev_index < MAX_SENSOR_COUNT; dev_index++) {

				sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, dev_index);

				int fd = px4_open(str, O_RDWR);

				const uint32_t driver_device_id = (uint32_t)px4_ioctl(fd, DEVIOCGDEVICEID, 0);

				if ((device_id > 0) && ((uint32_t)device_id == driver_device_id)) {
					mag_calibration_s mscale{};

					sprintf(str, "CAL_MAG%u_XOFF", dev_index);
					param_get(param_find(str), &mscale.x_offset);

					sprintf(str, "CAL_MAG%u_YOFF", dev_index);
					param_get(param_find(str), &mscale.y_offset);

					sprintf(str, "CAL_MAG%u_ZOFF", dev_index);
					param_get(param_find(str), &mscale.z_offset);

					sprintf(str, "CAL_MAG%u_XSCALE", dev_index);
					param_get(param_find(str), &mscale.x_scale);

					sprintf(str, "CAL_MAG%u_YSCALE", dev_index);
					param_get(param_find(str), &mscale.y_scale);

					sprintf(str, "CAL_MAG%u_ZSCALE", dev_index);
					param_get(param_find(str), &mscale.z_scale);

					// apply scaling and offsets
					if (px4_ioctl(fd, MAGIOCSSCALE, (long unsigned int)&mscale) != PX4_OK) {
						PX4_ERR("FAILED APPLYING mag CAL %u", dev_index);
					}
				}

				px4_close(fd);
			}

			// find corresponding enabled and rotation parameters
			for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {

				uORB::SubscriptionData<sensor_mag_s> mag_report{ORB_ID(sensor_mag)};

				if ((device_id > 0) && ((uint32_t)device_id == mag_report.get().device_id)) {

					// Enabled? CAL_MAGx_EN
					sprintf(str, "CAL_MAG%u_EN", cal_index);
					int32_t device_enabled = 1;
					param_get(param_find(str), &device_enabled);
					_enabled[uorb_index] = (device_enabled == 1);

					// Rotation
					sprintf(str, "CAL_MAG%u_ROT", cal_index);
					int32_t mag_rot = 0;
					param_get(param_find(str), &mag_rot);

					if (mag_report.get().is_external) {
						// check if this mag is still set as internal, otherwise leave untouched
						if (mag_rot < 0) {
							// it was marked as internal, change to external with no rotation
							mag_rot = 0;
							param_set_no_notification(param_find(str), &mag_rot);
						}

					} else {
						// mag is internal - reset param to -1 to indicate internal mag
						if (mag_rot != MAG_ROT_VAL_INTERNAL) {
							mag_rot = MAG_ROT_VAL_INTERNAL;
							param_set_no_notification(param_find(str), &mag_rot);
						}
					}

					// now get the mag rotation
					if (mag_rot >= 0) {
						// Set external magnetometers to use the parameter value
						_mag_rotation[uorb_index] = get_rot_matrix((enum Rotation)mag_rot);

					} else {
						// Set internal magnetometers to use the board rotation
						_mag_rotation[uorb_index] = board_rotation;
					}

					// CAL_MAGx_{X,Y,Z}COMP
					sprintf(str, "CAL_MAG%u_XCOMP", cal_index);
					param_get(param_find(str), &_power_compensation[uorb_index](0));

					sprintf(str, "CAL_MAG%u_YCOMP", cal_index);
					param_get(param_find(str), &_power_compensation[uorb_index](1));

					sprintf(str, "CAL_MAG%u_ZCOMP", cal_index);
					param_get(param_find(str), &_power_compensation[uorb_index](2));
				}
			}
		}
	}
}

void VehicleMagnetometer::Run()
{
	perf_begin(_cycle_perf);

	bool updated[MAX_SENSOR_COUNT] {};

	for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {

		if (!_enabled[uorb_index]) {
			continue;
		}

		if (!_advertised[uorb_index]) {
			// use data's timestamp to throttle advertisement checks
			if (hrt_elapsed_time(&_last_data[uorb_index].timestamp) > 1_s) {
				if (_sensor_sub[uorb_index].advertised()) {
					if (uorb_index > 0) {
						/* the first always exists, but for each further sensor, add a new validator */
						if (!_voter.add_new_validator()) {
							PX4_ERR("failed to add validator for sensor_mag:%i", uorb_index);
						}
					}

					_advertised[uorb_index] = true;

				} else {
					_last_data[uorb_index].timestamp = hrt_absolute_time();
				}
			}

		} else {
			sensor_mag_s report;
			updated[uorb_index] = _sensor_sub[uorb_index].update(&report);

			if (updated[uorb_index]) {
				if (_priority[uorb_index] == 0) {
					// set initial priority
					_priority[uorb_index] = _sensor_sub[uorb_index].get_priority();
				}

				Vector3f vect(report.x, report.y, report.z);

				// throttle-/current-based mag compensation when armed
				if (_mag_comp_type != MagCompensationType::Disabled && _armed) {
					vect = vect + _power * _power_compensation[uorb_index];
				}

				vect = _mag_rotation[uorb_index] * vect;

				_last_data[uorb_index].timestamp = report.timestamp;
				_last_data[uorb_index].x = vect(0);
				_last_data[uorb_index].y = vect(1);
				_last_data[uorb_index].z = vect(2);

				float mag_array[3];
				vect.copyTo(mag_array);
				_voter.put(uorb_index, report.timestamp, mag_array, report.error_count, _priority[uorb_index]);
			}
		}
	}

	// check for the current best sensor
	int best_index = 0;
	_voter.get_best(hrt_absolute_time(), &best_index);

	if (best_index >= 0) {
		if (_selected_sensor_sub_index != best_index) {
			// clear all registered callbacks
			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			_selected_sensor_sub_index = best_index;
			_sensor_sub[_selected_sensor_sub_index].registerCallback();
		}
	}

	if ((_selected_sensor_sub_index >= 0) && updated[_selected_sensor_sub_index]) {
		ParametersUpdate();

		const sensor_mag_s &mag = _last_data[_selected_sensor_sub_index];

		// populate vehicle_magnetometer with primary mag and publish
		vehicle_magnetometer_s out{};
		out.timestamp_sample = mag.timestamp; // TODO: mag.timestamp_sample;
		out.device_id = mag.device_id;
		out.magnetometer_ga[0] = _last_data[_selected_sensor_sub_index].x;
		out.magnetometer_ga[1] = _last_data[_selected_sensor_sub_index].y;
		out.magnetometer_ga[2] = _last_data[_selected_sensor_sub_index].z;

		out.timestamp = hrt_absolute_time();
		_vehicle_magnetometer_pub.publish(out);
	}

	// check failover and report
	if (_last_failover_count != _voter.failover_count()) {
		uint32_t flags = _voter.failover_state();
		int failover_index = _voter.failover_index();

		if (flags == DataValidator::ERROR_FLAG_NO_ERROR) {
			if (failover_index != -1) {
				// we switched due to a non-critical reason. No need to panic.
				PX4_INFO("sensor_mag switch from #%i", failover_index);
			}

		} else {
			if (failover_index != -1) {
				const hrt_abstime now = hrt_absolute_time();

				if (now - _last_error_message > 3_s) {
					mavlink_log_emergency(&_mavlink_log_pub, "sensor_mag:#%i failed: %s%s%s%s%s!, reconfiguring priorities",
							      failover_index,
							      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
							      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
							      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TIMEOUT" : ""),
							      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ERR CNT" : ""),
							      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " ERR DNST" : ""));
					_last_error_message = now;
				}

				// reduce priority of failed sensor to the minimum
				_priority[failover_index] = ORB_PRIO_MIN;
			}
		}
	}

	// check vehicle status for changes to publication state
	if (_vcontrol_mode_sub.updated()) {
		vehicle_control_mode_s vcontrol_mode;

		if (_vcontrol_mode_sub.copy(&vcontrol_mode)) {
			_armed = vcontrol_mode.flag_armed;
		}

		//check mag power compensation type (change battery current subscription instance if necessary)
		if ((MagCompensationType)_param_mag_comp_typ.get() == MagCompensationType::Current_inst0
		    && _mag_comp_type != MagCompensationType::Current_inst0) {

			_battery_status_sub = uORB::Subscription{ORB_ID(battery_status), 0};
		}

		if ((MagCompensationType)_param_mag_comp_typ.get() == MagCompensationType::Current_inst1
		    && _mag_comp_type != MagCompensationType::Current_inst1) {

			_battery_status_sub = uORB::Subscription{ORB_ID(battery_status), 1};
		}

		_mag_comp_type = (MagCompensationType)_param_mag_comp_typ.get();

		//update power signal for mag compensation
		if (_mag_comp_type == MagCompensationType::Throttle) {
			actuator_controls_s controls;

			if (_actuator_ctrl_0_sub.update(&controls)) {
				_power = controls.control[actuator_controls_s::INDEX_THROTTLE];
			}

		} else if (_mag_comp_type == MagCompensationType::Current_inst0
			   || _mag_comp_type == MagCompensationType::Current_inst1) {

			battery_status_s bat_stat;

			if (_battery_status_sub.update(&bat_stat)) {
				_power = bat_stat.current_a * 0.001f; //current in [kA]
			}
		}
	}



	if (!_armed) {
		calcMagInconsistency();
	}

	// reschedule timeout
	ScheduleDelayed(100_ms);

	perf_end(_cycle_perf);
}

void VehicleMagnetometer::calcMagInconsistency()
{
	sensor_preflight_mag_s preflt{};

	const sensor_mag_s &primary_mag_report = _last_data[_selected_sensor_sub_index];
	const Vector3f primary_mag(primary_mag_report.x, primary_mag_report.y,
				   primary_mag_report.z); // primary mag field vector

	float mag_angle_diff_max = 0.0f; // the maximum angle difference
	unsigned check_index = 0; // the number of sensors the primary has been checked against

	// Check each sensor against the primary
	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		// check that the sensor we are checking against is not the same as the primary
		if (_enabled[i] && (_priority[i] > 0) && (i != _selected_sensor_sub_index)) {
			// calculate angle to 3D magnetic field vector of the primary sensor
			const sensor_mag_s &current_mag_report = _last_data[i];
			Vector3f current_mag{current_mag_report.x, current_mag_report.y, current_mag_report.z};

			float angle_error = AxisAnglef(Quatf(current_mag, primary_mag)).angle();

			// complementary filter to not fail/pass on single outliers
			_mag_angle_diff[check_index] *= 0.95f;
			_mag_angle_diff[check_index] += 0.05f * angle_error;

			mag_angle_diff_max = math::max(mag_angle_diff_max, _mag_angle_diff[check_index]);

			// increment the check index
			check_index++;
		}

		// check to see if the maximum number of checks has been reached and break
		if (check_index >= 2) {
			break;
		}
	}

	// get the vector length of the largest difference and write to the combined sensor struct
	// will be zero if there is only one magnetometer and hence nothing to compare
	preflt.mag_inconsistency_angle = mag_angle_diff_max;

	preflt.timestamp = hrt_absolute_time();
	_sensor_preflight_mag_pub.publish(preflt);
}

void VehicleMagnetometer::PrintStatus()
{
	if (_selected_sensor_sub_index >= 0) {
		PX4_INFO("selected magnetometer: %d (%d)", _last_data[_selected_sensor_sub_index].device_id,
			 _selected_sensor_sub_index);
	}

	_voter.print();



#if !defined(CONSTRAINED_FLASH)
	uORB::Subscription vehicle_gps_position_sub {ORB_ID(vehicle_gps_position)};

	if (vehicle_gps_position_sub.advertised()) {
		vehicle_gps_position_s gps;

		if (vehicle_gps_position_sub.copy(&gps)) {
			if ((hrt_elapsed_time(&gps.timestamp) < 1_s) && (gps.eph < 1000)) {
				const double lat = gps.lat / 1.e7;
				const double lon = gps.lon / 1.e7;

				// set the magnetic field data returned by the geo library using the current GPS position
				const float mag_declination_gps = math::radians(get_mag_declination(lat, lon));
				const float mag_inclination_gps = math::radians(get_mag_inclination(lat, lon));

				const float mag_strength_gps = 0.01f * get_mag_strength(lat, lon);

				const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -mag_inclination_gps, mag_declination_gps)) * Vector3f(mag_strength_gps,
								0, 0);


				uORB::Subscription sensor_combined_sub{ORB_ID(sensor_combined)};
				sensor_combined_s imu{};
				sensor_combined_sub.copy(&imu);
				const Vector3f accel{imu.accelerometer_m_s2};

				uORB::Subscription vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
				vehicle_attitude_s attitude{};
				vehicle_attitude_sub.copy(&attitude);

				if (attitude.timestamp > 0) {
					Vector3f expected_field = Dcmf{Quatf{attitude.q}} .transpose() * mag_earth_pred;

					PX4_INFO(" X    Y    Z");
					PX4_INFO("[%.3f, %.3f, %.3f] Expected WMM ", (double)expected_field(0), (double)expected_field(1),
						 (double)expected_field(2));

					for (int mag_instance = 0; mag_instance < MAX_SENSOR_COUNT; mag_instance++) {
						sensor_mag_s mag{};

						if (_sensor_sub[mag_instance].advertised() && _sensor_sub[mag_instance].copy(&mag)) {
							//PX4_INFO("[%.3f, %.3f, %.3f] Magnetometer %d", (double)mag.x, (double)mag.y, (double)mag.z, mag_instance);

							const Vector3f m{mag.x, mag.y, mag.z};

							// Rotation matrix can be easily constructed from acceleration and mag field vectors
							// 'k' is Earth Z axis (Down) unit vector in body frame
							Vector3f k = -accel;
							k.normalize();

							// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
							Vector3f i = (m - k * (m * k));
							i.normalize();

							// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
							Vector3f j = k % i;

							// Fill rotation matrix
							Dcmf R;
							R.row(0) = i;
							R.row(1) = j;
							R.row(2) = k;

							// Convert to quaternion
							Quatf q = R;

							// Compensate for magnetic declination
							Quatf decl_rotation = Eulerf(0.0f, 0.0f, mag_declination_gps);
							q = q * decl_rotation;

							q.normalize();

							const Eulerf euler{q};
							PX4_INFO("Roll: %3.1fº, Pitch: %3.1fº, Yaw: %3.1fº] Attitude %d", (double)math::degrees(euler(0)),
								 (double)math::degrees(euler(1)), (double)math::degrees(euler(2)), mag_instance);
						}
					}
				}
			}
		}
	}

#endif // !CONSTRAINED_FLASH
}
