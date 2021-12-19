############################################################################
#
#   Copyright (c) 2015-2019 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_subdirectory(data_validator)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(vehicle_acceleration)
add_subdirectory(vehicle_angular_velocity)
add_subdirectory(vehicle_air_data)
add_subdirectory(vehicle_gps_position)
add_subdirectory(vehicle_imu)
add_subdirectory(vehicle_magnetometer)

px4_add_module(
	MODULE modules__sensors
	MAIN sensors
	SRCS
		sensors.cpp
		voted_sensors_update.cpp
		voted_sensors_update.h
	MODULE_CONFIG
		module.yaml
	DEPENDS
		airspeed
		conversion
		data_validator
		mathlib
		sensor_calibration
		vehicle_acceleration
		vehicle_angular_velocity
		vehicle_air_data
		vehicle_gps_position
		vehicle_imu
		vehicle_magnetometer
	PUBLICATIONS
		"px4::msg::Airspeed /airspeed"
		"px4::msg::SensorCombined /sensor_combined"
		"px4::msg::SensorSelection /sensor_selection"
		"px4::msg::SensorsStatusImu /sensors_status_imu"
		"px4::msg::SensorPreflightMag /sensor_preflight_mag"

		"px4::msg::VehicleAcceleration /vehicle_acceleration"
		"px4::msg::VehicleMagnetometer /vehicle_magnetometer"
		"px4::msg::VehicleAirData /vehicle_air_data"
		"px4::msg::VehicleImu /vehicle_imu"
		"px4::msg::VehicleImuStatus /vehicle_imu_status"
		"px4::msg::VehicleGpsPosition /vehicle_gps_position"
		"px4::msg::VehicleAngularVelocity /vehicle_angular_velocity"
		"px4::msg::VehicleAngularAcceleration /vehicle_angular_acceleration"

		# TODO: find a proper home
		"px4::msg::IrlockReport /irlock_report"
		"px4::msg::Rpm /rpm"

	SUBSCRIPTIONS
		"px4::msg::SensorCorrection /sensor_correction"

		"px4::msg::SensorAccel /sensor_accel" # TODO
		"px4::msg::SensorAccel /sensor_accel0"
		"px4::msg::SensorAccel /sensor_accel1"
		"px4::msg::SensorAccel /sensor_accel2"
		"px4::msg::SensorAccel /sensor_accel3"

		"px4::msg::SensorAccelFifo /sensor_accel_fifo"
		"px4::msg::SensorAccelFifo /sensor_accel_fifo0"
		"px4::msg::SensorAccelFifo /sensor_accel_fifo1"
		"px4::msg::SensorAccelFifo /sensor_accel_fifo2"
		"px4::msg::SensorAccelFifo /sensor_accel_fifo3"

		"px4::msg::SensorGyro /sensor_gyro"
		"px4::msg::SensorGyro /sensor_gyro0"
		"px4::msg::SensorGyro /sensor_gyro1"
		"px4::msg::SensorGyro /sensor_gyro2"
		"px4::msg::SensorGyro /sensor_gyro3"

		"px4::msg::SensorGyroFifo /sensor_gyro_fifo"
		"px4::msg::SensorGyroFifo /sensor_gyro_fifo0"
		"px4::msg::SensorGyroFifo /sensor_gyro_fifo1"
		"px4::msg::SensorGyroFifo /sensor_gyro_fifo2"
		"px4::msg::SensorGyroFifo /sensor_gyro_fifo3"

		"px4::msg::SensorGyroFft /sensor_gyro_fft"

		"px4::msg::DistanceSensor /distance_sensor" # TODO

		# vehicle_angular_velocity
		"px4::msg::EstimatorSelectorStatus /estimator_selector_status"
		"px4::msg::EstimatorSensorBias /estimator_sensor_bias"
		"px4::msg::EscStatus /esc_status"
		"px4::msg::VehicleControlMode /vehicle_control_mode"

		# vehicle_air_data
		"px4::msg::SensorBaro /sensor_baro" # TODO
		"px4::msg::SensorBaro /sensor_baro0"
		"px4::msg::SensorBaro /sensor_baro1"
		"px4::msg::SensorBaro /sensor_baro2"
		"px4::msg::SensorBaro /sensor_baro3"

		# vehicle_magnetometer
		"px4::msg::SensorMag /sensor_mag" # TODO
		"px4::msg::SensorMag /sensor_mag0"
		"px4::msg::SensorMag /sensor_mag1"
		"px4::msg::SensorMag /sensor_mag2"
		"px4::msg::SensorMag /sensor_mag3"
		"px4::msg::MagnetometerBiasEstimate /magnetometer_bias_estimate"
		"px4::msg::ActuatorControls /actuator_controls_0"
	)
