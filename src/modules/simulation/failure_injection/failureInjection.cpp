
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

#include <simulation/failure_injection/failureInjection.hpp>

bool FailureInjection::handle_gps_failure(sensor_gps_s &gps)
{
	if (_gps_blocked) {
		return false;
	}

	if (!_gps_stuck) {

		if (_gps_wrong) {
			gps.latitude_deg += 1.0;
			gps.longitude_deg += 1.0;
			gps.altitude_msl_m += 100.0;
			gps.altitude_ellipsoid_m += 100.0;

			gps.vel_m_s -= 1.f;
			gps.vel_n_m_s += 5.f;
			gps.vel_e_m_s -= 8.f;
			gps.vel_d_m_s += 2.f;
		}

		if (_gps_drift) {
			if (!_has_drift_ref) {

				_mp.initReference(gps.latitude_deg, gps.longitude_deg);
				_mp.project(gps.latitude_deg, gps.longitude_deg, _gps_drift_pos(0), _gps_drift_pos(1));
				_has_drift_ref = true;
			}

			matrix::Vector2f vel_gps(gps.vel_n_m_s, gps.vel_e_m_s);
			const float vel_norm = vel_gps.norm();

			// directional drift by kRelativeGpsDrift
			if (vel_norm > 1.f) {
				vel_gps *= 1.f + kRelativeGpsDrift;

			} else if (vel_norm > 0.01f) {
				vel_gps *= 0.5f / vel_norm;

			} else {
				vel_gps(0) = 0.5f;
			}

			// perpendicular drift
			matrix::Vector2f vel_normal = vel_gps.normalized();
			vel_gps(0) += vel_norm * kRelativeGpsDrift * vel_normal(1);
			vel_gps(1) -= vel_norm * kRelativeGpsDrift * vel_normal(0);

			gps.vel_n_m_s = vel_gps(0);
			gps.vel_e_m_s = vel_gps(1);

			if (_last_gps_timestamp > 0) {
				float dt = 1e-6f * (gps.timestamp - _last_gps_timestamp);
				_gps_drift_pos += vel_gps * dt;
				_mp.reproject(_gps_drift_pos(0), _gps_drift_pos(1), gps.latitude_deg, gps.longitude_deg);
			}

		}

		_gps_prev = gps;

	} else {
		gps = _gps_prev;
	}

	_last_gps_timestamp = gps.timestamp;

	return true;
}

bool FailureInjection::handle_gps_alt_failure(sensor_gps_s &gps)
{
	if (_gps_alt_blocked) {
		return false;
	}

	if (!_gps_alt_stuck) {

		if (_gps_alt_wrong) {
			gps.altitude_msl_m += 100.0;
			gps.altitude_ellipsoid_m += 100.0;
		}

		if (_gps_alt_drift) {
			if (_alt_drift_t0 == 0) {
				_alt_drift_t0 = gps.timestamp;
			}

			if (_gps_alt_offset < DBL_EPSILON) { _gps_alt_offset = gps.altitude_msl_m; }

			static constexpr double kDriftPeriodT = 20.;
			static constexpr double kDriftMagnitude = 5.;
			gps.vel_d_m_s = kDriftMagnitude * sin(2 * M_PI / kDriftPeriodT * 1e-6 * (gps.timestamp - _alt_drift_t0));

			if (_last_gps_timestamp > 0) {
				_gps_alt_offset += -(double)(gps.vel_d_m_s * (gps.timestamp - _last_gps_timestamp) / 1.e6f);
			}

			gps.altitude_msl_m = _gps_alt_offset;
		}

		_gps_prev = gps;

	} else {
		gps = _gps_prev;
	}

	_last_gps_timestamp = gps.timestamp;

	return true;

}


void FailureInjection::check_failure_injections()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE) {
			continue;
		}

		bool handled = false;
		bool supported = false;

		const int failure_unit = static_cast<int>(vehicle_command.param1 + 0.5f);
		const int failure_type = static_cast<int>(vehicle_command.param2 + 0.5f);
		const int instance = static_cast<int>(vehicle_command.param3 + 0.5f);

		if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_GPS) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, GPS off");
				supported = true;
				_gps_blocked = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, GPS ok");
				supported = true;
				_gps_blocked = false;
				_gps_stuck = false;
				_gps_wrong = false;
				_gps_drift = false;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				supported = true;
				_gps_stuck = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_WRONG) {
				supported = true;
				_gps_wrong = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_DRIFT) {
				PX4_INFO("CMD_INJECT_FAILURE, GPS drift");
				supported = true;
				_gps_drift = true;

			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_GPS_ALT) {

			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, GPS altitude off");
				supported = true;
				_gps_alt_blocked = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, GPS altitude ok");
				supported = true;
				_gps_alt_blocked = false;
				_gps_alt_stuck = false;
				_gps_alt_wrong = false;
				_gps_alt_drift = false;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				supported = true;
				_gps_alt_stuck = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_WRONG) {
				supported = true;
				_gps_alt_wrong = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_DRIFT) {
				PX4_INFO("CMD_INJECT_FAILURE, GPS altitude drift");
				supported = true;
				_gps_alt_drift = true;
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_ACCEL) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < kAccelCountMax; i++) {
						PX4_WARN("CMD_INJECT_FAILURE, accel %d off", i);
						_accel_blocked[i] = true;
						_accel_stuck[i] = false;
					}

				} else if (instance >= 1 && instance <= kAccelCountMax) {
					PX4_WARN("CMD_INJECT_FAILURE, accel %d off", instance - 1);
					_accel_blocked[instance - 1] = true;
					_accel_stuck[instance - 1] = false;
				}

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < kAccelCountMax; i++) {
						PX4_WARN("CMD_INJECT_FAILURE, accel %d stuck", i);
						_accel_blocked[i] = false;
						_accel_stuck[i] = true;
					}

				} else if (instance >= 1 && instance <= kAccelCountMax) {
					PX4_WARN("CMD_INJECT_FAILURE, accel %d stuck", instance - 1);
					_accel_blocked[instance - 1] = false;
					_accel_stuck[instance - 1] = true;
				}

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < kAccelCountMax; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, accel %d ok", i);
						_accel_blocked[i] = false;
						_accel_stuck[i] = false;
					}

				} else if (instance >= 1 && instance <= kAccelCountMax) {
					PX4_INFO("CMD_INJECT_FAILURE, accel %d ok", instance - 1);
					_accel_blocked[instance - 1] = false;
					_accel_stuck[instance - 1] = false;
				}
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_GYRO) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < kGyroCountMax; i++) {
						PX4_WARN("CMD_INJECT_FAILURE, gyro %d off", i);
						_gyro_blocked[i] = true;
						_gyro_stuck[i] = false;
					}

				} else if (instance >= 1 && instance <= kGyroCountMax) {
					PX4_WARN("CMD_INJECT_FAILURE, gyro %d off", instance - 1);
					_gyro_blocked[instance - 1] = true;
					_gyro_stuck[instance - 1] = false;
				}

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < kGyroCountMax; i++) {
						PX4_WARN("CMD_INJECT_FAILURE, gyro %d stuck", i);
						_gyro_blocked[i] = false;
						_gyro_stuck[i] = true;
					}

				} else if (instance >= 1 && instance <= kGyroCountMax) {
					PX4_INFO("CMD_INJECT_FAILURE, gyro %d stuck", instance - 1);
					_gyro_blocked[instance - 1] = false;
					_gyro_stuck[instance - 1] = true;
				}

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < kGyroCountMax; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, gyro %d ok", i);
						_gyro_blocked[i] = false;
						_gyro_stuck[i] = false;
					}

				} else if (instance >= 1 && instance <= kGyroCountMax) {
					PX4_INFO("CMD_INJECT_FAILURE, gyro %d ok", instance - 1);
					_gyro_blocked[instance - 1] = false;
					_gyro_stuck[instance - 1] = false;
				}
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_MAG) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < kMagCountMax; i++) {
						PX4_WARN("CMD_INJECT_FAILURE, mag %d off", i);
						_mag_blocked[i] = true;
						_mag_stuck[i] = false;
					}

				} else if (instance >= 1 && instance <= kMagCountMax) {
					PX4_WARN("CMD_INJECT_FAILURE, mag %d off", instance - 1);
					_mag_blocked[instance - 1] = true;
					_mag_stuck[instance - 1] = false;
				}

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < kMagCountMax; i++) {
						PX4_WARN("CMD_INJECT_FAILURE, mag %d stuck", i);
						_mag_blocked[i] = false;
						_mag_stuck[i] = true;
					}

				} else if (instance >= 1 && instance <= kMagCountMax) {
					PX4_WARN("CMD_INJECT_FAILURE, mag %d stuck", instance - 1);
					_mag_blocked[instance - 1] = false;
					_mag_stuck[instance - 1] = true;
				}

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < kMagCountMax; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, mag %d ok", i);
						_mag_blocked[i] = false;
						_mag_stuck[i] = false;
					}

				} else if (instance >= 1 && instance <= kMagCountMax) {
					PX4_INFO("CMD_INJECT_FAILURE, mag %d ok", instance - 1);
					_mag_blocked[instance - 1] = false;
					_mag_stuck[instance - 1] = false;
				}
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_BARO) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, baro off");
				supported = true;
				_baro_blocked = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				PX4_WARN("CMD_INJECT_FAILURE, baro stuck");
				supported = true;
				_baro_stuck = true;
				_baro_blocked = false;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, baro ok");
				supported = true;
				_baro_blocked = false;
				_baro_stuck = false;
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_AIRSPEED) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, airspeed off");
				supported = true;
				_airspeed_disconnected = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_WRONG) {
				PX4_WARN("CMD_INJECT_FAILURE, airspeed wrong (simulate pitot blockage)");
				supported = true;
				_airspeed_blocked_timestamp = hrt_absolute_time();

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, airspeed ok");
				supported = true;
				_airspeed_disconnected = false;
				_airspeed_blocked_timestamp = 0;
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_VIO) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, vio off");
				supported = true;
				_vio_blocked = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, vio ok");
				supported = true;
				_vio_blocked = false;
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_AGP) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, agp off");
				supported = true;
				_agp_blocked = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, agp ok");
				supported = true;
				_agp_blocked = false;
				_agp_stuck = false;
				_agp_drift = false;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				PX4_WARN("CMD_INJECT_FAILURE, agp stuck");
				supported = true;
				_agp_stuck = true;
				_agp_blocked = false;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_DRIFT) {
				PX4_WARN("CMD_INJECT_FAILURE, agp drift");
				supported = true;
				_agp_drift = true;
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_DISTANCE_SENSOR) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, distance sensor off");
				supported = true;
				_distance_sensor_blocked = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, distance sensor ok");
				supported = true;
				_distance_sensor_blocked = false;
				_distance_sensor_stuck = false;
				_distance_sensor_wrong = false;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				PX4_WARN("CMD_INJECT_FAILURE, distance sensor stuck");
				supported = true;
				_distance_sensor_stuck = true;
				_distance_sensor_blocked = false;
				_distance_sensor_wrong = false;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_WRONG) {
				PX4_WARN("CMD_INJECT_FAILURE, distance sensor wrong (blocked at 0.5m)");
				supported = true;
				_distance_sensor_wrong = true;
				_distance_sensor_blocked = false;
				_distance_sensor_stuck = false;
			}
		}

		if (handled) {
			vehicle_command_ack_s ack{};
			ack.command = vehicle_command.command;
			ack.from_external = false;
			ack.result = supported ?
				     vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED :
				     vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
			ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(ack);
		}
	}
}
