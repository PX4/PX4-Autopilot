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

#include <lib/drivers/device/Device.hpp>

#include "MicroStrain.hpp"

ModalIoSerial device_uart;

MicroStrain::MicroStrain(const char *uart_port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::INS3),
	_vehicle_global_position_pub((_param_ms_mode.get() == 0) ? ORB_ID(external_ins_global_position) : ORB_ID(
					     vehicle_global_position)),
	_vehicle_attitude_pub((_param_ms_mode.get() == 0) ? ORB_ID(external_ins_attitude) : ORB_ID(vehicle_attitude)),
	_vehicle_local_position_pub((_param_ms_mode.get() == 0) ? ORB_ID(external_ins_local_position) : ORB_ID(
					    vehicle_local_position))
{
	// Store port name
	memset(_port, '\0', sizeof(_port));
	const size_t max_len = math::min(sizeof(_port), strnlen(uart_port, sizeof(_port)));
	strncpy(_port, uart_port, max_len);

	// Enforce null termination
	_port[max_len] = '\0';

	// The scheduling rate (in microseconds) is set higher than the highest data rate param (in Hz)
	const int max_param_rate = math::max(math::max(_param_ms_imu_rate_hz.get(), _param_ms_mag_rate_hz.get(),
					     _param_ms_baro_rate_hz.get()), _param_ms_filter_rate_hz.get());

	// We always want the schedule rate faster than the highest sensor data rate
	_ms_schedule_rate_us = (1e6) / (2 * max_param_rate);
	PX4_DEBUG("Schedule rate: %i", _ms_schedule_rate_us);

	device::Device::DeviceId device_id{};
	device_id.devid_s.devtype = DRV_INS_DEVTYPE_MICROSTRAIN;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	device_id.devid_s.bus = 2;
	_dev_id = device_id.devid;

	_px4_accel.set_device_id(_dev_id);
	_px4_gyro.set_device_id(_dev_id);
	_px4_mag.set_device_id(_dev_id);

	_sensor_baro.device_id = _dev_id;
	_sensor_baro.pressure = 0;
	_sensor_baro.temperature = 0;
	_sensor_baro.error_count = 0;

	gnss_antenna_offset1[0] = _param_gnss_offset1_x.get();
	gnss_antenna_offset1[1] = _param_gnss_offset1_y.get();
	gnss_antenna_offset1[2] = _param_gnss_offset1_z.get();
	PX4_DEBUG("GNSS antenna offset 1: %f/%f/%f", (double)_param_gnss_offset1_x.get(),
		  (double)_param_gnss_offset1_y.get(),
		  (double)_param_gnss_offset1_z.get());

	gnss_antenna_offset2[0] = _param_gnss_offset2_x.get();
	gnss_antenna_offset2[1] = _param_gnss_offset2_y.get();
	gnss_antenna_offset2[2] = _param_gnss_offset2_z.get();
	PX4_DEBUG("GNSS antenna offset 2: %f/%f/%f", (double)_param_gnss_offset2_x.get(),
		  (double)_param_gnss_offset2_y.get(),
		  (double)_param_gnss_offset2_z.get());

	rotation.euler[0] = _param_ms_sensor_roll.get();
	rotation.euler[1] = _param_ms_sensor_pitch.get();
	rotation.euler[2] = _param_ms_sensor_yaw.get();
	PX4_DEBUG("Device Roll/Pitch/Yaw: %f/%f/%f", (double)_param_ms_sensor_roll.get(),
		  (double)_param_ms_sensor_pitch.get(),
		  (double)_param_ms_sensor_yaw.get());

	_sensor_baro_pub.advertise();
	_sensor_selection_pub.advertise();
	_vehicle_local_position_pub.advertise();
	_vehicle_angular_velocity_pub.advertise();
	_vehicle_attitude_pub.advertise();
	_vehicle_global_position_pub.advertise();
	_vehicle_odometry_pub.advertise();
	_estimator_status_pub.advertise();

}

MicroStrain::~MicroStrain()
{
	if (device_uart.isOpen()) {
		device_uart.uartClose();
	}

	PX4_DEBUG("Destructor");
	_sensor_baro_pub.unadvertise();
	_sensor_selection_pub.unadvertise();
	_vehicle_local_position_pub.unadvertise();
	_vehicle_angular_velocity_pub.unadvertise();
	_vehicle_attitude_pub.unadvertise();
	_vehicle_global_position_pub.unadvertise();
	_vehicle_odometry_pub.unadvertise();
	_estimator_status_pub.unadvertise();

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool mipInterfaceUserRecvFromDevice(mip_interface *device, uint8_t *buffer, size_t max_length,
				    timeout_type wait_time,
				    size_t *out_length, timestamp_type *timestamp_out)
{
	(void)device;

	*timestamp_out = hrt_absolute_time();

	int res = device_uart.uartRead(buffer, max_length);

	if (res == -1 && errno != EAGAIN) {
		PX4_ERR("MicroStrain driver failed to read(%d): %s", errno, strerror(errno));
		*out_length = 0;
		return false;
	}

	if (res >= 0) {
		*out_length = res;
	}

	return true;
}

bool mipInterfaceUserSendToDevice(mip_interface *device, const uint8_t *data, size_t length)
{

	int res = device_uart.uartWrite(const_cast<uint8_t *>(data), length);

	if (res >= 0) {
		return true;
	}

	PX4_ERR("MicroStrain driver failed to write(%d): %s", errno, strerror(errno));
	return false;

}

mip_cmd_result MicroStrain::forceIdle()
{
	// Setting to idle may fail the first couple times, so call it a few times in case the device is streaming too much data
	PX4_DEBUG("Setting device to idle");
	mip_cmd_result res = MIP_PX4_ERROR;
	uint8_t set_to_idle_tries = 0;

	while (set_to_idle_tries++ < 3) {
		if (mip_cmd_result_is_ack((res = mip_base_set_idle(&_device)))) {
			break;

		} else {
			usleep(1_s);
		}
	}

	return res;
}

int MicroStrain::connectAtBaud(int32_t baud)
{
	if (device_uart.isOpen()) {
		if (device_uart.uartSetBaud(baud) == PX4_ERROR) {
			PX4_ERR("Failed to set UART %lu baud", baud);
		}

	} else if (device_uart.uartOpen(_port, baud) == PX4_ERROR) {
		PX4_ERR("Could not open device port!");
		return PX4_ERROR;
	}

	PX4_INFO("Serial Port %s with baud of %lu baud", (device_uart.isOpen() ? "CONNECTED" : "NOT CONNECTED"), baud);

	// Re-init the interface with the correct timeouts
	mip_interface_init(&_device, _parse_buffer, sizeof(_parse_buffer), mip_timeout_from_baudrate(baud) * 1_ms, 250_ms,
			   &mipInterfaceUserSendToDevice, &mipInterfaceUserRecvFromDevice, &mip_interface_default_update, NULL);

	if (!mip_cmd_result_is_ack(forceIdle())) {
		PX4_ERR("Could not set device to idle");
		return PX4_ERROR;
	}

	PX4_INFO("Successfully opened and pinged");
	return PX4_OK;
}

mip_cmd_result MicroStrain::getSupportedDescriptors()
{
	const size_t descriptors_max_size = sizeof(_supported_descriptors) / sizeof(_supported_descriptors[0]);
	uint8_t descriptors_count, extended_descriptors_count;

	// Pull the descriptors and the extended descriptors from the device
	mip_cmd_result res = mip_base_get_device_descriptors(&_device, _supported_descriptors, descriptors_max_size,
			     &descriptors_count);
	mip_cmd_result res_extended = mip_base_get_extended_descriptors(&_device, &(_supported_descriptors[descriptors_count]),
				      descriptors_max_size - descriptors_count, &extended_descriptors_count);

	if (!mip_cmd_result_is_ack(res)) {
		return res;
	}

	if (mip_cmd_result_is_ack(res_extended)) {
		PX4_DEBUG("Device does not support the extended descriptors command.");
	}

	_supported_desc_len = descriptors_count + extended_descriptors_count;

	// Get the supported descriptor sets from the obtained descriptors
	for (uint16_t i = 0; i < _supported_desc_len; i++) {
		auto descriptor_set = static_cast<uint8_t>((_supported_descriptors[i] & 0xFF00) >> 8);
		bool unique = true;

		uint16_t j;

		for (j = 0; j < _supported_desc_set_len; j++) {
			if (_supported_descriptor_sets[j] == descriptor_set) {
				unique = false;
				break;

			} else if (_supported_descriptor_sets[j] == 0) {
				break;
			}
		}

		if (unique) {_supported_descriptor_sets[j] = descriptor_set; _supported_desc_set_len++;}
	}

	return res;
}

bool MicroStrain::supportsDescriptorSet(uint8_t descriptor_set)
{
	for (uint16_t i = 0; i < _supported_desc_set_len; i++) {
		if (_supported_descriptor_sets[i] == descriptor_set) {
			return true;
		}
	}

	return false;
}

bool MicroStrain::supportsDescriptor(uint8_t descriptor_set, uint8_t field_descriptor)
{
	// Check if the descriptor set is supported
	if (!supportsDescriptorSet(descriptor_set)) {return false;}

	// Check if the field descriptor is supported
	const uint16_t full_descriptor = (descriptor_set << 8) | field_descriptor;

	for (uint16_t i = 0; i < _supported_desc_len; i++) {
		if (_supported_descriptors[i] == full_descriptor) {
			return true;
		}
	}

	return false;
}

mip_cmd_result MicroStrain::writeBaudRate(uint32_t baudrate, uint8_t port)
{
	mip_cmd_result res;

	// Writes the baudrate using whichever command the device supports
	if (supportsDescriptor(MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED)) {
		res = mip_base_write_comm_speed(&_device, port, baudrate);

	}

	else if (supportsDescriptor(MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_UART_BAUDRATE)) {
		res = mip_3dm_write_uart_baudrate(&_device, baudrate);
	}

	else {
		PX4_ERR("Does not support write baudrate commands");
		res = MIP_PX4_ERROR;
	}

	return res;
}

mip_cmd_result MicroStrain::getBaseRate(uint8_t descriptor_set, uint16_t *base_rate)
{
	mip_cmd_result res;

	// Gets the base rate using whichever command the device supports
	if (supportsDescriptor(MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GET_BASE_RATE)) {
		res = mip_3dm_get_base_rate(&_device, descriptor_set, base_rate);

	} else {
		switch (descriptor_set) {
		case MIP_SENSOR_DATA_DESC_SET: {
				if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_CMD_DESC_3DM_GET_IMU_BASE_RATE)) {
					res = mip_3dm_imu_get_base_rate(&_device, base_rate);

				} else {
					PX4_ERR("IMU base rate command is not supported");
					res = MIP_PX4_ERROR;
				}

				break;
			}

		case MIP_GNSS_DATA_DESC_SET: {
				if (supportsDescriptor(MIP_GNSS_DATA_DESC_SET, MIP_CMD_DESC_3DM_GET_GNSS_BASE_RATE)) {
					res = mip_3dm_gps_get_base_rate(&_device, base_rate);

				} else {
					PX4_ERR("GNSS base rate command is not supported");
					res = MIP_PX4_ERROR;
				}

				break;
			}

		case MIP_FILTER_DATA_DESC_SET: {
				if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_CMD_DESC_3DM_GET_FILTER_BASE_RATE)) {
					res = mip_3dm_filter_get_base_rate(&_device, base_rate);

				} else {
					PX4_ERR("Filter base rate command is not supported");
					res = MIP_PX4_ERROR;
				}

				break;
			}

		default:
			PX4_ERR("Descriptor set for base rate is not supported");
			res = MIP_PX4_ERROR;
			break;
		}
	}

	return res;

}

mip_cmd_result MicroStrain::writeMessageFormat(uint8_t descriptor_set, uint8_t num_descriptors,
		const mip::DescriptorRate *descriptors)
{
	mip_cmd_result res;

	// Writes the message format using whichever command the device supports
	if (supportsDescriptor(MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_MESSAGE_FORMAT)) {
		return mip_3dm_write_message_format(&_device, descriptor_set, num_descriptors,
						    descriptors);

	} else {
		switch (descriptor_set) {
		case MIP_SENSOR_DATA_DESC_SET: {
				if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_CMD_DESC_3DM_IMU_MESSAGE_FORMAT)) {
					res = mip_3dm_write_imu_message_format(&_device, num_descriptors, descriptors);

				} else {
					PX4_ERR("IMU message format command is not supported");
					res = MIP_PX4_ERROR;
				}

				break;
			}

		case MIP_GNSS_DATA_DESC_SET: {
				if (supportsDescriptor(MIP_GNSS_DATA_DESC_SET, MIP_CMD_DESC_3DM_GNSS_MESSAGE_FORMAT)) {
					res = mip_3dm_write_gps_message_format(&_device, num_descriptors, descriptors);

				} else {
					PX4_ERR("GNSS messaage format command is not supported");
					res = MIP_PX4_ERROR;
				}

				break;
			}

		case MIP_FILTER_DATA_DESC_SET: {
				if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_CMD_DESC_3DM_FILTER_MESSAGE_FORMAT)) {
					res = mip_3dm_write_filter_message_format(&_device, num_descriptors, descriptors);


				} else {
					PX4_ERR("Filter message format command is not supported");
					res = MIP_PX4_ERROR;
				}

				break;
			}

		default:
			PX4_ERR("Descriptor set for writing message format is not supported");
			res = MIP_PX4_ERROR;
			break;
		}
	}

	return res;
}

mip_cmd_result MicroStrain::configureImuMessageFormat()
{
	PX4_DEBUG("Configuring IMU Message Format");

	uint8_t num_imu_descriptors = 0;
	mip_descriptor_rate imu_descriptors[4];

	// Get the base rate
	uint16_t base_rate;
	mip_cmd_result res = getBaseRate(MIP_SENSOR_DATA_DESC_SET, &base_rate);

	PX4_DEBUG("The IMU base rate is %d", base_rate);

	if (!mip_cmd_result_is_ack(res)) {
		PX4_ERR("Could not get the IMU base rate");
		return res;
	}

	// Configure the Message Format depending on if the device supports the descriptor
	uint16_t imu_decimation = base_rate / (uint16_t)_param_ms_imu_rate_hz.get();
	uint16_t mag_decimation = base_rate / (uint16_t)_param_ms_mag_rate_hz.get();
	uint16_t baro_decimation = base_rate / (uint16_t)_param_ms_baro_rate_hz.get();

	PX4_DEBUG("IMU decimation: %i", imu_decimation);
	PX4_DEBUG("Mag decimation: %i", mag_decimation);
	PX4_DEBUG("Baro decimation: %i", baro_decimation);


	if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_ACCEL_SCALED)
	    && _param_ms_imu_rate_hz.get() > 0) {
		imu_descriptors[num_imu_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_SENSOR_ACCEL_SCALED, imu_decimation};
		PX4_DEBUG("IMU: Scaled accel enabled");
	}

	if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_GYRO_SCALED)
	    && _param_ms_imu_rate_hz.get() > 0) {
		imu_descriptors[num_imu_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_SENSOR_GYRO_SCALED, imu_decimation};
		PX4_DEBUG("IMU: Scaled gyro enabled");
	}

	if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_MAG_SCALED)
	    && _param_ms_mag_rate_hz.get() > 0) {
		imu_descriptors[num_imu_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_SENSOR_MAG_SCALED, mag_decimation};
		PX4_DEBUG("IMU: Scaled mag enabled");
	}

	if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_PRESSURE_SCALED)
	    && _param_ms_baro_rate_hz.get() > 0) {
		imu_descriptors[num_imu_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_SENSOR_PRESSURE_SCALED, baro_decimation};
		PX4_DEBUG("IMU: Scaled pressure enabled");
	}

	// Write the settings
	res = writeMessageFormat(MIP_SENSOR_DATA_DESC_SET, num_imu_descriptors,
				 imu_descriptors);

	return res;

}

mip_cmd_result MicroStrain::configureFilterMessageFormat()
{
	PX4_DEBUG("Configuring Filter Message Format");

	uint8_t num_filter_descriptors = 0;
	mip_descriptor_rate filter_descriptors[11];

	// Get the base rate
	uint16_t base_rate;
	mip_cmd_result res = getBaseRate(MIP_FILTER_DATA_DESC_SET, &base_rate);

	PX4_DEBUG("The filter base rate is %d", base_rate);

	if (!mip_cmd_result_is_ack(res)) {
		PX4_ERR("Could not get the filter base rate");
		return res;
	}

	// Configure the Message Format depending on if the device supports the descriptor
	uint16_t filter_decimation = base_rate / (uint16_t)_param_ms_filter_rate_hz.get();

	PX4_DEBUG("Filter decimation: %i", filter_decimation);

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_POS_LLH)
	    && _param_ms_filter_rate_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_POS_LLH, filter_decimation};
		PX4_DEBUG("Filter: LLH pos enabled");
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_ATT_QUATERNION)
	    && _param_ms_filter_rate_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_ATT_QUATERNION, filter_decimation};
		PX4_DEBUG("Filter: Attitude quaternion enabled");
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_VEL_NED)
	    && _param_ms_filter_rate_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_VEL_NED, filter_decimation};
		PX4_DEBUG("Filter: Velocity NED enabled");
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION)
	    && _param_ms_filter_rate_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION, filter_decimation};
		PX4_DEBUG("Filter: Linear accel FRD enabled");
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_POS_UNCERTAINTY)
	    && _param_ms_filter_rate_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_POS_UNCERTAINTY, filter_decimation};
		PX4_DEBUG("Filter: Position uncertainty enabled");
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY)
	    && _param_ms_filter_rate_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY, filter_decimation};
		PX4_DEBUG("Filter: Velocity uncertainty enabled");
	}

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER)
	    && _param_ms_filter_rate_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER, filter_decimation};
		PX4_DEBUG("Filter: Attitude euler uncertainty enabled");
	}

	// if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE)
	//     && _param_ms_filter_rate_hz.get() > 0) {
	// 	filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE, filter_decimation};
	// 	PX4_DEBUG("Filter: Compensated angular rate enabled");
	// }

	if (supportsDescriptor(MIP_FILTER_DATA_DESC_SET, MIP_DATA_DESC_FILTER_FILTER_STATUS)
	    && _param_ms_filter_rate_hz.get() > 0) {
		filter_descriptors[num_filter_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_FILTER_FILTER_STATUS, filter_decimation};
		PX4_DEBUG("Filter: Status enabled");
	}

	// Write the settings
	res = writeMessageFormat(MIP_FILTER_DATA_DESC_SET, num_filter_descriptors,
				 filter_descriptors);

	return res;

}

mip_cmd_result MicroStrain::configureAidingMeasurement(uint16_t aiding_source, bool enable)
{
	mip_cmd_result res;

	// Configures the aiding measurement if the device support it
	if (supportsDescriptor(MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE)) {
		res = mip_filter_write_aiding_measurement_enable(&_device, aiding_source, enable);

		// If the device doesnt support the aiding source but the command is to disable it, we consider it a success
		if (res == MIP_NACK_INVALID_PARAM && !enable) {
			res = MIP_ACK_OK;
		}

	} else {
		PX4_WARN("Aiding measurements are not supported");

		// If the command was to disable, we consider it a success
		if (!enable) {
			res = MIP_ACK_OK;

		} else {
			res = MIP_PX4_ERROR;
		}
	}

	return res;
}

mip_cmd_result MicroStrain::configureAidingSources()
{
	PX4_DEBUG("Configuring aiding sources");
	mip_cmd_result res;

	// Selectively turn on the magnetometer aiding source
	if (!mip_cmd_result_is_ack(res = configureAidingMeasurement(
			MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_MAGNETOMETER,
			_param_ms_int_mag_en.get()))) {
		PX4_ERR("Could not configure magnetometer aiding");
		return res;
	}

	// Enables GNSS Position & Velocity as an aiding measurement
	res = configureAidingMeasurement(MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_POS_VEL,
					 true);

	if (!mip_cmd_result_is_ack(res)) {
		if (res != MIP_NACK_INVALID_PARAM && res != MIP_PX4_ERROR) {
			PX4_ERR("Error enabling GNSS Position & Velocity aiding");
			return res;

		} else {
			PX4_WARN("Could not enable GNSS Position & Velocity aiding");
		}

	} else {
		// Check to see if sending GNSS position and velocity as an aiding measurement is supported
		bool pos_aiding = supportsDescriptor(MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_POS_LLH);
		bool vel_aiding = supportsDescriptor(MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_VEL_NED);
		_ext_pos_vel_aiding = pos_aiding && vel_aiding;

		if (!_ext_pos_vel_aiding) {
			PX4_ERR("Sending GNSS pos/vel aiding messages is not supported");
			return MIP_PX4_ERROR;
		}

	}

	// Selectively turn on external heading as an aiding measurement
	if (!mip_cmd_result_is_ack(res = configureAidingMeasurement(
			MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_HEADING,
			_param_ms_ext_heading_en.get()))) {
		PX4_ERR("Could not configure external heading aiding");
		return res;

	} else {
		// Check to see if sending external heading as an aiding measurement is supported
		_ext_heading_aiding = supportsDescriptor(MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_HEADING_TRUE)
				      && _param_ms_ext_heading_en.get();

		if (!_ext_heading_aiding && _param_ms_ext_heading_en.get()) {
			PX4_ERR("Sending external heading aiding messages is not supported");
			return MIP_PX4_ERROR;
		}

	}

	// Prioritizing setting up multi antenna offsets if it is supported
	if (supportsDescriptor(MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET)) {
		PX4_INFO("Inside");
		mip_cmd_result res1 = mip_filter_write_multi_antenna_offset(&_device, 1, gnss_antenna_offset1);
		mip_cmd_result res2 = mip_filter_write_multi_antenna_offset(&_device, 2, gnss_antenna_offset2);

		if (!mip_cmd_result_is_ack(res1)) {
			PX4_ERR("Could not write multi antenna offsets");
			return res1;

		}

		else if (!mip_cmd_result_is_ack(res2)) {
			PX4_ERR("Could not write multi antenna offsets");
			return res2;

		}

		// Selectively enables dual antenna heading as an aiding measurement
		if (!mip_cmd_result_is_ack(res = configureAidingMeasurement(
				MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_HEADING,
				_param_ms_int_heading_en.get()))) {
			PX4_ERR("Could not configure dual antenna heading aiding");
			return res;
		}

	}

	// Otherwise sets up the aiding frame
	else if (supportsDescriptor(MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_FRAME_CONFIG)) {
		if (!mip_cmd_result_is_ack(res = mip_aiding_write_frame_config(&_device, 1,
						 MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER, false,
						 gnss_antenna_offset1, &rotation))) {
			PX4_ERR("Could not write aiding frame config");
			return res;
		}
	}

	else {
		PX4_WARN("Aiding frames are not supported");
	}

	_ext_aiding = _ext_pos_vel_aiding || _ext_heading_aiding;

	return res;
}

mip_cmd_result MicroStrain::writeFilterInitConfig()
{
	PX4_DEBUG("Initializing filter");
	mip_cmd_result res;

	float filter_init_pos[3] = {0};
	float filter_init_vel[3] = {0};

	uint8_t initial_alignment;

	switch (_param_ms_alignment.get()) {
	case 1: {
			initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_KINEMATIC;
			break;
		}

	case 2: {
			initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_EXTERNAL;
			break;
		}

	case 3: {
			initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_DUAL_ANTENNA;
			break;
		}

	default: {
			initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_MAGNETOMETER;
			break;
		}
	}

	// Filter initialization configuration
	if (supportsDescriptor(MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION)) {
		res = mip_filter_write_initialization_configuration(&_device, 0,
				MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_AUTO_POS_VEL_ATT,
				initial_alignment,
				0.0, 0.0, 0.0, filter_init_pos, filter_init_vel, MIP_FILTER_REFERENCE_FRAME_LLH);
	}

	else {
		PX4_WARN("Filter initialization is not supported");
		res = MIP_ACK_OK;
	}

	return res;
}

bool MicroStrain::initializeIns()
{

	mip_cmd_result res;

	const uint32_t DESIRED_BAUDRATE = 921600;

	static constexpr uint32_t BAUDRATES[] {115200, 921600, 460800, 230400, 128000, 38400, 19200, 57600, 9600};
	bool is_connected = false;

	for (auto &baudrate : BAUDRATES) {
		if (connectAtBaud(baudrate) == PX4_OK) {
			PX4_INFO("found baudrate %lu", baudrate);
			is_connected = true;
			break;
		}
	}

	if (!is_connected) {
		PX4_ERR("Could not connect to the device, exiting");
		return false;
	}

	// Get the supported descriptors for the device in use
	if (!mip_cmd_result_is_ack(getSupportedDescriptors())) {
		PX4_ERR("Could not get descriptors");
		return false;
	}

	// Setting the device baudrate to the desired value
	PX4_INFO("Setting the baud to desired baud rate");

	if (!mip_cmd_result_is_ack(res = writeBaudRate(DESIRED_BAUDRATE, 1))) {
		PX4_ERR("Could not set the baudrate!");
		return false;
	}

	tcflush(device_uart.uartGetFd(), TCIOFLUSH);

	// Connecting using the desired baudrate
	if (connectAtBaud(DESIRED_BAUDRATE) != PX4_OK) {
		PX4_ERR("Could not Connect at %lu", DESIRED_BAUDRATE);
		return false;
	}

	// Configure the IMU message formt based on what descriptors are supported
	if (!mip_cmd_result_is_ack(res = configureImuMessageFormat())) {
		MS_PX4_ERROR(res, "Could not write message format");
		return false;
	}

	// Register data callbacks
	mip_interface_register_packet_callback(&_device, &_sensor_data_handler, MIP_SENSOR_DATA_DESC_SET, false,
					       &sensorCallback,
					       this);


	// Configure the Filter message format based on what descriptors are supported
	if (!mip_cmd_result_is_ack(res = configureFilterMessageFormat())) {
		MS_PX4_ERROR(res, "Could not write message format");
		return false;
	}

	// Register data callbacks
	mip_interface_register_packet_callback(&_device, &_filter_data_handler, MIP_FILTER_DATA_DESC_SET, false,
					       &filterCallback,
					       this);

	// Configure the aiding sources based on what the sensor supports
	if (!mip_cmd_result_is_ack(res = configureAidingSources())) {
		MS_PX4_ERROR(res, "Could not configure aiding frames!");
		return false;
	}

	// Initialize the filter
	if (!mip_cmd_result_is_ack(res = writeFilterInitConfig())) {
		MS_PX4_ERROR(res, "Could not configure filter initialization!");
		return false;
	}

	// Setup the rotation based on PX4 standard rotation sets
	if (_param_ms_svt_en.get() && supportsDescriptor(MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_SENSOR2VEHICLE_TRANSFORM_EUL)) {
		PX4_DEBUG("Writing SVT");

		if (!mip_cmd_result_is_ack(res = mip_3dm_write_sensor_2_vehicle_transform_euler(&_device,
						 math::radians<float>(rotation.euler[0]),
						 math::radians<float>(rotation.euler[1]), math::radians<float>(rotation.euler[2])))) {
			MS_PX4_ERROR(res, "Could not set sensor-to-vehicle transformation!");
			return false;
		}
	}

	// Reset the filter
	if (supportsDescriptor(MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_RESET_FILTER)) {
		PX4_DEBUG("Reseting filter");

		if (!mip_cmd_result_is_ack(res = mip_filter_reset(&_device))) {
			MS_PX4_ERROR(res, "Could not reset the filter!");
			return false;
		}
	}

	if (supportsDescriptor(MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_CONTROL_DATA_STREAM)) {
		PX4_DEBUG("Writing datastream control");

		if (!mip_cmd_result_is_ack(res = mip_3dm_write_datastream_control(&_device,
						 MIP_3DM_DATASTREAM_CONTROL_COMMAND_ALL_STREAMS, true))) {
			MS_PX4_ERROR(res, "Could not enable the data stream");
			return false;
		}
	}

	// Resume the device
	if (supportsDescriptor(MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_RESUME)) {
		PX4_DEBUG("Resuming device");

		if (!mip_cmd_result_is_ack(res = mip_base_resume(&_device))) {
			MS_PX4_ERROR(res, "Could not resume the device!");
			return false;
		}
	}

	return true;
}

void MicroStrain::sensorCallback(void *user, const mip_packet *packet, mip::Timestamp timestamp)
{
	MicroStrain *ref = static_cast<MicroStrain *>(user);

	assert(mip_packet_descriptor_set(packet) == MIP_SENSOR_DATA_DESC_SET);

	SensorSample<mip_sensor_scaled_accel_data> accel;
	SensorSample<mip_sensor_scaled_gyro_data> gyro;
	SensorSample<mip_sensor_scaled_mag_data> mag;
	SensorSample<mip_sensor_scaled_pressure_data> baro;

	// Iterate through the packet and extract based on the descriptor present
	auto t = hrt_absolute_time();

	for (mip_field field = mip_field_first_from_packet(packet); mip_field_is_valid(&field); mip_field_next(&field)) {
		switch (mip_field_field_descriptor(&field)) {

		case MIP_DATA_DESC_SENSOR_ACCEL_SCALED:
			extract_mip_sensor_scaled_accel_data_from_field(&field, &accel.sample);
			accel.updated = true;
			break;


		case MIP_DATA_DESC_SENSOR_GYRO_SCALED:
			extract_mip_sensor_scaled_gyro_data_from_field(&field, &gyro.sample);
			gyro.updated = true;
			break;


		case MIP_DATA_DESC_SENSOR_MAG_SCALED:
			extract_mip_sensor_scaled_mag_data_from_field(&field, &mag.sample);
			mag.updated = true;
			break;


		case MIP_DATA_DESC_SENSOR_PRESSURE_SCALED:
			extract_mip_sensor_scaled_pressure_data_from_field(&field, &baro.sample);
			baro.updated = true;
			break;


		default:
			break;


		}
	}

	// Publish only if the corresponding data was extracted from the packet
	if (accel.updated) {
		ref->_px4_accel.update(t, accel.sample.scaled_accel[0]*CONSTANTS_ONE_G,
				       accel.sample.scaled_accel[1]*CONSTANTS_ONE_G,
				       accel.sample.scaled_accel[2]*CONSTANTS_ONE_G);

	}

	if (gyro.updated) {
		ref->_px4_gyro.update(t, gyro.sample.scaled_gyro[0],
				      gyro.sample.scaled_gyro[1],
				      gyro.sample.scaled_gyro[2]);
	}

	if (mag.updated) {
		ref->_px4_mag.update(t, mag.sample.scaled_mag[0],
				     mag.sample.scaled_mag[1],
				     mag.sample.scaled_mag[2]);
	}

	if (baro.updated) {
		ref->_sensor_baro.timestamp = timestamp;
		ref->_sensor_baro.timestamp_sample = t;
		ref->_sensor_baro.pressure = baro.sample.scaled_pressure * 100.f; // convert [Pa] to [mBar]
		ref->_sensor_baro_pub.publish(ref->_sensor_baro);

	}

}

void MicroStrain::filterCallback(void *user, const mip_packet *packet, mip::Timestamp timestamp)
{
	MicroStrain *ref = static_cast<MicroStrain *>(user);

	assert(mip_packet_descriptor_set(packet) == MIP_FILTER_DATA_DESC_SET);

	SensorSample<mip_filter_position_llh_data> pos_llh;
	SensorSample<mip_filter_attitude_quaternion_data> att_quat;
	//SensorSample<mip_filter_comp_angular_rate_data> ang_rate;
	SensorSample<mip_filter_velocity_ned_data> vel_ned;
	SensorSample<mip_filter_status_data> stat;
	SensorSample<mip_filter_position_llh_uncertainty_data> llh_uncert;
	SensorSample<mip_filter_velocity_ned_uncertainty_data> vel_uncert;
	SensorSample<mip_filter_euler_angles_uncertainty_data> att_euler_uncert;
	SensorSample<mip_filter_linear_accel_data> lin_accel;

	// Iterate through the packet and extract based on the descriptor present
	auto t = hrt_absolute_time();

	for (mip_field field = mip_field_first_from_packet(packet); mip_field_is_valid(&field); mip_field_next(&field)) {
		switch (mip_field_field_descriptor(&field)) {
		case MIP_DATA_DESC_FILTER_POS_LLH:
			extract_mip_filter_position_llh_data_from_field(&field, &pos_llh.sample);
			pos_llh.updated = true;
			break;

		case MIP_DATA_DESC_FILTER_ATT_QUATERNION:
			extract_mip_filter_attitude_quaternion_data_from_field(&field, &att_quat.sample);
			att_quat.updated = true;
			break;

		case MIP_DATA_DESC_FILTER_VEL_NED:
			extract_mip_filter_velocity_ned_data_from_field(&field, &vel_ned.sample);
			vel_ned.updated = true ;
			break;

		case MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION:
			extract_mip_filter_linear_accel_data_from_field(&field, &lin_accel.sample);
			lin_accel.updated = true;
			break;

		case MIP_DATA_DESC_FILTER_POS_UNCERTAINTY:
			extract_mip_filter_position_llh_uncertainty_data_from_field(&field, &llh_uncert.sample);
			llh_uncert.updated = true;
			break;

		case MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY:
			extract_mip_filter_velocity_ned_uncertainty_data_from_field(&field, &vel_uncert.sample);
			vel_uncert.updated = true;
			break;

		case MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER:
			extract_mip_filter_euler_angles_uncertainty_data_from_field(&field, &att_euler_uncert.sample);
			att_euler_uncert.updated = true;
			break;

		// case MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE:
		// 	extract_mip_filter_comp_angular_rate_data_from_field(&field, &ang_rate.sample);
		// 	ang_rate.updated = true;
		// 	break;

		case MIP_DATA_DESC_FILTER_FILTER_STATUS:
			extract_mip_filter_status_data_from_field(&field, &stat.sample);
			stat.updated = true;
			break;

		default:
			break;
		}
	}

	// Publish only if the required data was extracted from the packet
	bool vehicle_global_position_valid = pos_llh.updated && llh_uncert.updated && stat.updated;
	bool vehicle_attitude_valid = att_quat.updated;
	bool vehicle_local_position_valid = pos_llh.updated && att_quat.updated && vel_ned.updated && lin_accel.updated
					    && llh_uncert.updated && vel_uncert.updated && att_euler_uncert.updated && stat.updated;
	bool vehicle_odometry_valid = pos_llh.updated && att_quat.updated && vel_ned.updated && llh_uncert.updated
				      && vel_uncert.updated && att_euler_uncert.updated;
	bool estimator_status_valid = stat.updated && llh_uncert.updated;
	//bool vehicle_angular_velocity_valid = ang_rate.updated;

	if (vehicle_global_position_valid) {
		vehicle_global_position_s gp{0};
		gp.timestamp_sample = t;

		gp.lat = pos_llh.sample.latitude;
		gp.lon = pos_llh.sample.longitude;

		gp.lat_lon_valid = (stat.sample.filter_state == MIP_FILTER_MODE_FULL_NAV);

		// gp.alt is supposed to be in MSL, since the filter only estimates ellipsoid, we publish that instead.
		gp.alt_ellipsoid = pos_llh.sample.ellipsoid_height;
		gp.alt = pos_llh.sample.ellipsoid_height;

		gp.alt_valid = (stat.sample.filter_state == MIP_FILTER_MODE_FULL_NAV);

		gp.eph = sqrtf(sq(llh_uncert.sample.north) + sq(llh_uncert.sample.east) + sq(ref->_gps_origin_ep[0]));
		gp.epv = sqrtf(sq(llh_uncert.sample.down) + sq(ref->_gps_origin_ep[1]));

		// ------- Fields we cannot obtain -------
		gp.delta_alt = 0;
		gp.delta_terrain = 0;
		gp.lat_lon_reset_counter = 0;
		gp.alt_reset_counter = 0;
		gp.terrain_reset_counter = 0;

		gp.dead_reckoning = false;
		gp.terrain_alt_valid = false;
		gp.terrain_alt = 0;
		// ---------------------------------------

		gp.timestamp = hrt_absolute_time();
		ref->_vehicle_global_position_pub.publish(gp);
	}

	if (vehicle_attitude_valid) {
		vehicle_attitude_s att_data{0};
		att_data.timestamp_sample = t;

		att_data.q[0] = att_quat.sample.q[0];
		att_data.q[1] = att_quat.sample.q[1];
		att_data.q[2] = att_quat.sample.q[2];
		att_data.q[3] = att_quat.sample.q[3];

		// ------- Fields we cannot obtain -------
		att_data.delta_q_reset[0] = 0;
		att_data.delta_q_reset[0] = 0;
		att_data.delta_q_reset[0] = 0;
		att_data.delta_q_reset[0] = 0;
		att_data.quat_reset_counter = 0;
		// ---------------------------------------

		att_data.timestamp = hrt_absolute_time();
		ref->_vehicle_attitude_pub.publish(att_data);
	}

	if (vehicle_local_position_valid) {
		vehicle_local_position_s vp{0};
		vp.timestamp_sample = t;

		const Vector2f pos_ned = ref->_pos_ref.project(pos_llh.sample.latitude, pos_llh.sample.longitude);

		vp.x = pos_ned(0);
		vp.y = pos_ned(1);
		vp.z = -(pos_llh.sample.ellipsoid_height - ref->_ref_alt);

		vp.xy_valid = (stat.sample.filter_state == MIP_FILTER_MODE_FULL_NAV);

		vp.z_valid = (stat.sample.filter_state == MIP_FILTER_MODE_FULL_NAV);

		vp.vx = vel_ned.sample.north;
		vp.vy = vel_ned.sample.east;
		vp.vz = vel_ned.sample.down;
		vp.z_deriv = vp.vz;

		vp.v_xy_valid = (stat.sample.filter_state == MIP_FILTER_MODE_FULL_NAV);

		vp.v_z_valid = (stat.sample.filter_state == MIP_FILTER_MODE_FULL_NAV);

		// Conversion of liner acceleration from body frame to NED frame
		matrix::Quatf quat{att_quat.sample.q[0], att_quat.sample.q[1], att_quat.sample.q[2], att_quat.sample.q[3]};
		matrix::Vector3f acc_body{lin_accel.sample.accel[0], lin_accel.sample.accel[1], lin_accel.sample.accel[2]};
		matrix::Vector3f acc_ned = quat.rotateVectorInverse(acc_body);

		vp.ax = acc_ned(0);
		vp.ay = acc_ned(1);
		vp.az = acc_ned(2);

		// Get yaw from attitude for heading
		matrix::Eulerf euler_attitude(matrix::Quatf(att_quat.sample.q[0], att_quat.sample.q[1], att_quat.sample.q[2],
					      att_quat.sample.q[3]));
		float yaw = euler_attitude.psi();

		vp.heading_good_for_control = (yaw != NAN);
		vp.heading = yaw;
		vp.heading_var = att_euler_uncert.sample.yaw;

		// vp.ref_alt is supposed to be in MSL, since the filter only estimates ellipsoid, we publish that instead.
		vp.xy_global = (ref->_pos_ref.isInitialized() == true);
		vp.z_global = (ref->_pos_ref.isInitialized() == true);
		vp.ref_timestamp = ref->_pos_ref.getProjectionReferenceTimestamp();
		vp.ref_lat = ref->_pos_ref.getProjectionReferenceLat();
		vp.ref_lon = ref->_pos_ref.getProjectionReferenceLon();
		vp.ref_alt = ref->_ref_alt;

		vp.eph = sqrtf(sq(llh_uncert.sample.north) + sq(llh_uncert.sample.east));
		vp.epv = llh_uncert.sample.down;
		vp.evh = sqrtf(sq(vel_uncert.sample.north) + sq(vel_uncert.sample.east));
		vp.evv = vel_uncert.sample.down;

		vp.vxy_max = INFINITY;
		vp.vz_max = INFINITY;
		vp.hagl_min = INFINITY;
		vp.hagl_max_xy = INFINITY;
		vp.hagl_max_z = INFINITY;

		// ------- Fields we cannot obtain -------
		vp.delta_xy[0] = 0;
		vp.delta_xy[1] = 0;
		vp.xy_reset_counter = 0;
		vp.delta_z = 0;
		vp.z_reset_counter = 0;

		vp.delta_vxy[0] = 0;
		vp.delta_vxy[1] = 0;
		vp.vxy_reset_counter = 0;
		vp.delta_vz = 0;
		vp.vz_reset_counter = 0;

		vp.unaided_heading = NAN;
		vp.delta_heading = 0;
		vp.heading_reset_counter = 0;

		vp.tilt_var = 0;

		vp.dist_bottom_valid = 0;
		vp.dist_bottom = 0;
		vp.dist_bottom_var = 0;
		vp.delta_dist_bottom = 0;
		vp.dist_bottom_reset_counter = 0;
		vp.dist_bottom_sensor_bitfield = 0;

		vp.dead_reckoning = false;
		// ---------------------------------------

		vp.timestamp = hrt_absolute_time();
		ref->_vehicle_local_position_pub.publish(vp);

	}

	if (vehicle_odometry_valid && ref->_param_ms_mode.get()) {
		vehicle_odometry_s vo{0};
		vo.timestamp_sample = t;

		const Vector2f pos_ned = ref->_pos_ref.project(pos_llh.sample.latitude, pos_llh.sample.longitude);

		vo.pose_frame = 1;
		vo.position[0] = pos_ned(0);
		vo.position[1] = pos_ned(1);
		vo.position[2] = -(pos_llh.sample.ellipsoid_height - ref->_ref_alt);

		vo.q[0] = att_quat.sample.q[0];
		vo.q[1] = att_quat.sample.q[1];
		vo.q[2] = att_quat.sample.q[2];
		vo.q[3] = att_quat.sample.q[3];

		vo.velocity_frame = 1;
		vo.velocity[0] = vel_ned.sample.north;
		vo.velocity[1] = vel_ned.sample.east;
		vo.velocity[2] = vel_ned.sample.down;

		// vo.angular_velocity[0] = ang_rate.sample.gyro[0];
		// vo.angular_velocity[1] = ang_rate.sample.gyro[1];
		// vo.angular_velocity[2] = ang_rate.sample.gyro[2];

		vo.position_variance[0] = sq(llh_uncert.sample.north);
		vo.position_variance[1] = sq(llh_uncert.sample.east);
		vo.position_variance[2] = sq(llh_uncert.sample.down);

		vo.velocity_variance[0] = sq(vel_uncert.sample.north);
		vo.velocity_variance[1] = sq(vel_uncert.sample.east);
		vo.velocity_variance[2] = sq(vel_uncert.sample.down);

		vo.orientation_variance[0] = sq(att_euler_uncert.sample.roll);
		vo.orientation_variance[1] = sq(att_euler_uncert.sample.pitch);
		vo.orientation_variance[2] = sq(att_euler_uncert.sample.yaw);

		// ------- Fields we cannot obtain -------
		vo.reset_counter = 0;
		vo.quality = 0;
		// ---------------------------------------

		vo.timestamp = hrt_absolute_time();
		ref->_vehicle_odometry_pub.publish(vo);
	}

	// if (vehicle_angular_velocity_valid && ref->_param_ms_mode.get()) {
	// 	vehicle_angular_velocity_s av{0};
	// 	av.timestamp_sample = t;

	// 	av.xyz[0] = ang_rate.sample.gyro[0];
	// 	av.xyz[1] = ang_rate.sample.gyro[1];
	// 	av.xyz[2] = ang_rate.sample.gyro[2];

	// 	// ------- Fields we cannot obtain -------
	// 	av.xyz_derivative[0] = 0;
	// 	av.xyz_derivative[1] = 0;
	// 	av.xyz_derivative[2] = 0;
	// 	// ---------------------------------------

	// 	av.timestamp = hrt_absolute_time();
	// 	ref->_vehicle_angular_velocity_pub.publish(av);
	// }

	if (estimator_status_valid && ref->_param_ms_mode.get()) {
		estimator_status_s status{0};
		status.timestamp_sample = t;

		status.output_tracking_error[0] = 0;
		status.output_tracking_error[1] = 0;
		status.output_tracking_error[2] = 0;

		status.gps_check_fail_flags = 0;

		// Minimal mapping of error flags from device to the PX4 health flags
		status.control_mode_flags = stat.sample.filter_state == 4 ? (0x1 << estimator_status_s::CS_GPS) |
					    (0x1 << estimator_status_s::CS_GPS_HGT) : 0x00;
		status.filter_fault_flags = stat.sample.status_flags;

		status.pos_horiz_accuracy = sqrtf(sq(llh_uncert.sample.north) + sq(llh_uncert.sample.east));
		status.pos_vert_accuracy = llh_uncert.sample.down;

		status.hdg_test_ratio = 0.1f;
		status.vel_test_ratio = 0.1f;
		status.pos_test_ratio = 0.1f;
		status.hgt_test_ratio = 0.1f;
		status.tas_test_ratio = 0.1f;
		status.hagl_test_ratio = 0.1f;
		status.beta_test_ratio = 0.1f;

		status.solution_status_flags = 0;

		status.reset_count_vel_ne = 0;
		status.reset_count_vel_d = 0;
		status.reset_count_pos_ne = 0;
		status.reset_count_pod_d = 0;
		status.reset_count_quat = 0;

		status.time_slip = 0;

		status.pre_flt_fail_innov_heading = false;
		status.pre_flt_fail_innov_height = false;
		status.pre_flt_fail_innov_pos_horiz = false;
		status.pre_flt_fail_innov_vel_horiz = false;
		status.pre_flt_fail_innov_vel_vert = false;
		status.pre_flt_fail_mag_field_disturbed = false;

		status.accel_device_id = ref->_dev_id;
		status.gyro_device_id = ref->_dev_id;
		status.mag_device_id = ref->_dev_id;
		status.baro_device_id = ref->_dev_id;

		status.health_flags = 0;
		status.timeout_flags = 0;

		status.mag_inclination_deg = 0;
		status.mag_inclination_ref_deg = 0;
		status.mag_strength_gs = 0;
		status.mag_strength_ref_gs = 0;


		status.timestamp = hrt_absolute_time();
		ref->_estimator_status_pub.publish(status);

		sensor_selection_s sensor_selection{};
		sensor_selection.accel_device_id = ref->_dev_id;
		sensor_selection.gyro_device_id = ref->_dev_id;

		sensor_selection.timestamp = hrt_absolute_time();
		ref->_sensor_selection_pub.publish(sensor_selection);
	}
}

void MicroStrain::sendAidingMeasurements()
{
	sensor_gps_s gps{0};

	// No new data
	if (!_sensor_gps_sub.update(&gps)) {
		return;
	}

	// Fix isn't 3D or RTK or RTCM
	if ((gps.fix_type < 3) || (gps.fix_type > 6)) {
		return;
	}

	// If the timestamp has not been set, then don't send any data into the filter
	if (gps.time_utc_usec == 0) {
		return;
	}

	const hrt_abstime time_now_us = hrt_absolute_time();
	mip_time t;
	t.timebase = MIP_TIME_TIMEBASE_TIME_OF_ARRIVAL;
	t.reserved = 0x01;
	t.nanoseconds = 0;

	// Sends GNSS position and velocity aiding data if they are both supported
	if (_ext_pos_vel_aiding) {
		if (!_pos_ref.isInitialized()) {
			_pos_ref.initReference(gps.latitude_deg, gps.longitude_deg, time_now_us);
			_ref_alt = gps.altitude_ellipsoid_m;

			_gps_origin_ep[0] = gps.eph;
			_gps_origin_ep[1] = gps.epv;
		}

		float llh_uncertainty[3] = {gps.eph, gps.eph, gps.epv};
		mip_aiding_llh_pos(&_device, &t, MIP_FILTER_REFERENCE_FRAME_LLH, gps.latitude_deg,
				   gps.longitude_deg,
				   gps.altitude_ellipsoid_m, llh_uncertainty, MIP_AIDING_LLH_POS_COMMAND_VALID_FLAGS_ALL);

		if (gps.vel_ned_valid) {
			float ned_v[3] = {gps.vel_n_m_s, gps.vel_e_m_s, gps.vel_d_m_s};
			float ned_velocity_uncertainty[3] = {sqrtf(gps.s_variance_m_s), sqrtf(gps.s_variance_m_s), sqrtf(gps.s_variance_m_s)};
			mip_aiding_ned_vel(&_device, &t, MIP_FILTER_REFERENCE_FRAME_LLH, ned_v, ned_velocity_uncertainty,
					   MIP_AIDING_NED_VEL_COMMAND_VALID_FLAGS_ALL);
		}
	}

	// Sends external heading aiding data if they are both supported
	if (_ext_heading_aiding && PX4_ISFINITE(gps.heading)) {
		float heading = gps.heading + gps.heading_offset;
		mip_aiding_true_heading(&_device, &t, MIP_FILTER_REFERENCE_FRAME_LLH, heading, gps.heading_accuracy, 0xff);
	}

}

bool MicroStrain::init()
{
	// Run on fixed interval
	ScheduleOnInterval(_ms_schedule_rate_us);

	return true;
}

void MicroStrain::Run()
{

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (!_is_initialized) {
		_is_initialized = initializeIns();
		PX4_INFO("Initialization complete");
	}

	// Initialization failed, stop the module
	if (!_is_initialized) {
		request_stop();
		perf_end(_loop_perf);
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)

	}

	mip_interface_update(&_device, false);

	// Sends aiding data only if external aiding was set up
	if (_ext_aiding) {sendAidingMeasurements();}

	perf_end(_loop_perf);

}

int MicroStrain::task_spawn(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	const char *dev = "/dev/ttyS4";

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			dev = myoptarg;
			break;

		default:
			PX4_WARN("Unrecognized option, Using defaults");
			break;
		}
	}

	if (dev == nullptr || strlen(dev) == 0) {
		print_usage("no device specified");
		_object.store(nullptr);
		_task_id = -1;

		return PX4_ERROR;
	}

	PX4_INFO("Opening device port %s", dev);
	MicroStrain *instance = new MicroStrain(dev);

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

int MicroStrain::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int MicroStrain::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MicroStrain::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
MicroStrain by HBK Inertial Sensor Driver.
Currently supports the CV7-AR, CV7-AHRS, CV7-INS and the CV7-GNSS/INS

Communicates over serial port using submodule MIP_SDK.

### Examples
Attempt to start driver on a specified serial device.
The driver will choose /dev/ttyS4 by default if no port is specified
$ microstrain start -d /dev/ttyS1
Stop driver
$ microstrain stop

)DESCR_STR");



	PRINT_MODULE_USAGE_NAME("MicroStrain", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("ins");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS4", "<file:dev>", "Port", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Driver status");

	return PX4_OK;
}

extern "C" __EXPORT int microstrain_main(int argc, char *argv[])
{
	return MicroStrain::main(argc, argv);
}
