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
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	// Store port name
	memset(_port, '\0', sizeof(_port));
	const size_t max_len = math::min(sizeof(_port), strnlen(uart_port, sizeof(_port)));
	strncpy(_port, uart_port, max_len);

	// Enforce null termination
	_port[max_len] = '\0';

	// The scheduling rate (in microseconds) is set higher than the highest sensor data rate (in Hz)
	const int max_param_rate = math::max(_param_ms_imu_rate_hz.get(), _param_ms_mag_rate_hz.get(),
					     _param_ms_baro_rate_hz.get());

	// We always want the schedule rate faster than the highest sensor data rate
	_ms_schedule_rate_us = (1e6) / (2 * max_param_rate);

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
}

MicroStrain::~MicroStrain()
{
	if (device_uart.isOpen()) {
		device_uart.uartClose();
	}

	PX4_DEBUG("Destructor");
	_sensor_baro_pub.unadvertise();

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
		PX4_DEBUG("Number of bytes read %d(%d)", *out_length, max_length);
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

mip::CmdResult MicroStrain::forceIdle()
{
	// Setting to idle may fail the first couple times, so call it a few times in case the device is streaming too much data
	mip::CmdResult result;
	uint8_t set_to_idle_tries = 0;

	while (set_to_idle_tries++ < 3) {
		if (!!(result = mip_base_set_idle(&_device))) {
			break;

		} else {
			usleep(1_s);
		}
	}

	return result;
}

int MicroStrain::connectAtBaud(int32_t baud)
{
	if (device_uart.isOpen()) {
		if (device_uart.uartSetBaud(baud) == PX4_ERROR) {
			PX4_INFO(" - Failed to set UART %" PRIu32 " baud", baud);
		}

	} else if (device_uart.uartOpen(_port, baud) == PX4_ERROR) {
		PX4_INFO(" - Failed to open UART");
		PX4_ERR("ERROR: Could not open device port!");
		return PX4_ERROR;
	}

	PX4_INFO("Serial Port %s with baud of %" PRIu32 " baud", (device_uart.isOpen() ? "CONNECTED" : "NOT CONNECTED"), baud);

	// Re-init the interface with the correct timeouts
	mip_interface_init(&_device, _parse_buffer, sizeof(_parse_buffer), mip_timeout_from_baudrate(baud) * 1_ms, 250_ms,
			   &mipInterfaceUserSendToDevice, &mipInterfaceUserRecvFromDevice, &mip_interface_default_update, NULL);

	if (!(forceIdle())) {
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

	if (res != MIP_ACK_OK) {
		return res;
	}

	if (res_extended != MIP_ACK_OK) {
		PX4_DEBUG(node_, "Device does not appear to support the extended descriptors command.");
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

mip_cmd_result MicroStrain::getBaseRate(uint8_t descriptor_set, uint16_t *base_rate)
{
	// If the device supports the mip_3dm_get_base_rate command, use that one, otherwise use the specific function
	if (supportsDescriptor(MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_GET_BASE_RATE)) {
		return mip_3dm_get_base_rate(&_device, descriptor_set, base_rate);

	} else {
		switch (descriptor_set) {
		case MIP_SENSOR_DATA_DESC_SET:
			return mip_3dm_imu_get_base_rate(&_device, base_rate);

		case MIP_GNSS_DATA_DESC_SET:
			return mip_3dm_gps_get_base_rate(&_device, base_rate);

		case MIP_FILTER_DATA_DESC_SET:
			return mip_3dm_filter_get_base_rate(&_device, base_rate);

		default:
			return MIP_NACK_INVALID_PARAM;
		}
	}

}

mip_cmd_result MicroStrain::writeMessageFormat(uint8_t descriptor_set, uint8_t num_descriptors,
		const mip::DescriptorRate *descriptors)
{
	if (supportsDescriptor(MIP_3DM_CMD_DESC_SET, MIP_CMD_DESC_3DM_MESSAGE_FORMAT)) {
		return mip_3dm_write_message_format(&_device, MIP_SENSOR_DATA_DESC_SET, num_descriptors,
						    descriptors);

	} else {
		switch (descriptor_set) {
		case MIP_SENSOR_DATA_DESC_SET:
			return mip_3dm_write_imu_message_format(&_device, num_descriptors, descriptors);

		case MIP_GNSS_DATA_DESC_SET:
			return mip_3dm_write_gps_message_format(&_device, num_descriptors, descriptors);

		case MIP_FILTER_DATA_DESC_SET:
			return mip_3dm_write_filter_message_format(&_device, num_descriptors, descriptors);

		default:
			return MIP_NACK_INVALID_PARAM;
		}
	}
}

mip_cmd_result MicroStrain::configureImuMessageFormat()
{
	uint8_t num_imu_descriptors = 0;
	mip_descriptor_rate imu_descriptors[4];

	// Get the base rate
	uint16_t base_rate;
	mip_cmd_result res = getBaseRate(MIP_SENSOR_DATA_DESC_SET, &base_rate);

	PX4_INFO("The sensor base rate is %d", base_rate);

	if (res != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not get the base rate");
		return res;
	}

	// Configure the Message Format depending on if the device supports the descriptor
	uint16_t imu_decimation = base_rate / (uint16_t)_param_ms_imu_rate_hz.get();
	uint16_t mag_decimation = base_rate / (uint16_t)_param_ms_mag_rate_hz.get();
	uint16_t baro_decimation = base_rate / (uint16_t)_param_ms_baro_rate_hz.get();

	if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_ACCEL_SCALED)
	    && _param_ms_imu_rate_hz.get() > 0) {
		imu_descriptors[num_imu_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_SENSOR_ACCEL_SCALED, imu_decimation};
	}

	if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_GYRO_SCALED)
	    && _param_ms_imu_rate_hz.get() > 0) {
		imu_descriptors[num_imu_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_SENSOR_GYRO_SCALED, imu_decimation};
	}

	if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_MAG_SCALED)
	    && _param_ms_mag_rate_hz.get() > 0) {
		imu_descriptors[num_imu_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_SENSOR_MAG_SCALED, mag_decimation};
	}

	if (supportsDescriptor(MIP_SENSOR_DATA_DESC_SET, MIP_DATA_DESC_SENSOR_PRESSURE_SCALED)
	    && _param_ms_baro_rate_hz.get() > 0) {
		imu_descriptors[num_imu_descriptors++] = mip_descriptor_rate { MIP_DATA_DESC_SENSOR_PRESSURE_SCALED, baro_decimation};
	}


	// Write the settings
	res = writeMessageFormat(MIP_SENSOR_DATA_DESC_SET, num_imu_descriptors,
				 imu_descriptors);

	return res;

}

mip_cmd_result MicroStrain::writeBaudRate(uint32_t baudrate, uint8_t port)
{
	if (supportsDescriptor(MIP_BASE_CMD_DESC_SET, MIP_CMD_DESC_BASE_COMM_SPEED)) {
		return mip_base_write_comm_speed(&_device, port, baudrate);

	}

	return mip_3dm_write_uart_baudrate(&_device, baudrate);

}

bool MicroStrain::initializeIns()
{
	const uint32_t DESIRED_BAUDRATE = 921600;

	static constexpr uint32_t BAUDRATES[] {115200, 921600, 460800, 230400, 128000, 38400, 19200, 57600, 9600};
	bool is_connected = false;

	for (auto &baudrate : BAUDRATES) {
		if (connectAtBaud(baudrate) == PX4_OK) {
			PX4_INFO("found baudrate %" PRIu32, baudrate);
			is_connected = true;
			break;
		}
	}

	if (!is_connected) {
		PX4_ERR("Could not connect to the device, exiting");
		return false;
	}

	// Setting the device baudrate to the desired value
	PX4_INFO("Setting the baud to desired baud rate");

	if (writeBaudRate(DESIRED_BAUDRATE, 1) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set the baudrate!");
		return false;
	}

	tcflush(device_uart.uartGetFd(), TCIOFLUSH);

	// Connecting using the desired baudrate
	if (connectAtBaud(DESIRED_BAUDRATE) != PX4_OK) {
		PX4_INFO("ERROR: Could not Connect at %lu", DESIRED_BAUDRATE);
		return false;
	}

	// Get the supported descriptors for the device in use
	if (getSupportedDescriptors() != MIP_ACK_OK) {
		PX4_INFO("ERROR: Could not get descriptors");
		return false;
	}

	// Configure the IMU message formt based on what descriptors are supported
	if (configureImuMessageFormat() != MIP_ACK_OK) {
		PX4_INFO("ERROR: Could not write message format");
		return false;
	}

	// Register data callbacks
	mip_interface_register_packet_callback(&_device, &_sensor_data_handler, MIP_SENSOR_DATA_DESC_SET, false,
					       &sensorCallback,
					       this);


	if (mip_3dm_write_datastream_control(&_device, MIP_3DM_DATASTREAM_CONTROL_COMMAND_ALL_STREAMS, true) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not enable the data stream");
		return false;
	}

	// Resume the device
	if (mip_base_resume(&_device) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not resume the device!");
		return false;
	}

	return true;

}

void MicroStrain::sensorCallback(void *user, const mip_packet *packet, mip::Timestamp timestamp)
{
	MicroStrain *ref = static_cast<MicroStrain *>(user);

	assert(mip_packet_descriptor_set(packet) == MIP_SENSOR_DATA_DESC_SET);

	mip_sensor_scaled_accel_data accel;
	mip_sensor_scaled_gyro_data gyro;
	mip_sensor_scaled_mag_data mag;
	mip_sensor_scaled_pressure_data baro;

	bool accel_valid = false;
	bool gyro_valid = false;
	bool mag_valid = false;
	bool baro_valid = false;

	// Iterate through the packet and extract based on the descriptor present
	auto t = hrt_absolute_time();

	for (mip_field field = mip_field_first_from_packet(packet); mip_field_is_valid(&field); mip_field_next(&field)) {
		switch (mip_field_field_descriptor(&field)) {

		case MIP_DATA_DESC_SENSOR_ACCEL_SCALED:
			extract_mip_sensor_scaled_accel_data_from_field(&field, &accel);
			accel_valid = true;
			break;


		case MIP_DATA_DESC_SENSOR_GYRO_SCALED:
			extract_mip_sensor_scaled_gyro_data_from_field(&field, &gyro);
			gyro_valid = true;
			break;


		case MIP_DATA_DESC_SENSOR_MAG_SCALED:
			extract_mip_sensor_scaled_mag_data_from_field(&field, &mag);
			mag_valid = true;
			break;


		case MIP_DATA_DESC_SENSOR_PRESSURE_SCALED:
			extract_mip_sensor_scaled_pressure_data_from_field(&field, &baro);
			baro_valid = true;
			break;


		default:
			break;


		}
	}

	// Publish only if the corresponding data was extracted from the packet
	if (accel_valid) {
		ref->_px4_accel.update(t, accel.scaled_accel[0]*CONSTANTS_ONE_G,
				       accel.scaled_accel[1]*CONSTANTS_ONE_G,
				       accel.scaled_accel[2]*CONSTANTS_ONE_G);

	}

	if (gyro_valid) {
		ref->_px4_gyro.update(t, gyro.scaled_gyro[0],
				      gyro.scaled_gyro[1],
				      gyro.scaled_gyro[2]);
	}

	if (mag_valid) {
		ref->_px4_mag.update(t, mag.scaled_mag[0],
				     mag.scaled_mag[1],
				     mag.scaled_mag[2]);
	}

	if (baro_valid) {
		ref->_sensor_baro.timestamp = timestamp;
		ref->_sensor_baro.timestamp_sample = t;
		ref->_sensor_baro.pressure = baro.scaled_pressure * 100.f; // convert [Pa] to [mBar]
		ref->_sensor_baro_pub.publish(ref->_sensor_baro);

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
MicroStrain INS Driver.
Supports the CV7-AR and CV7-AHRS

Communicates over serial port an utilizes the manufacturer provided MIP SDK.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("MicroStrain", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS4", "<file:dev>", "INS Port", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Driver status");

	return 0;
}

extern "C" __EXPORT int microstrain_main(int argc, char *argv[])
{
	return MicroStrain::main(argc, argv);
}
