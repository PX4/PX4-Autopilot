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
#include "mip_sdk/src/mip/mip_all.h"
#include "mip_sdk/src/mip/mip_parser.h"

uint8_t external_heading_sensor_id = 1;
uint8_t gnss_antenna_sensor_id = 2;
uint8_t vehicle_frame_velocity_sensor_id = 3;

static MicrosStrain *ins{nullptr};


ModalIoSerial device_uart;

const uint8_t FILTER_ROLL_EVENT_ACTION_ID  = 1;
const uint8_t FILTER_PITCH_EVENT_ACTION_ID = 2;


MicrosStrain::MicrosStrain(const char *uart_port, int32_t rot) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	/* store port name */
	memset(_port, '\0', 20);
	int max_len = math::min((unsigned int)20, strlen(uart_port));
	strncpy(_port, uart_port, max_len);
	/* enforce null termination */
	_port[19] = '\0';

	rotation = static_cast<Rotation>(rot);

	device::Device::DeviceId device_id{};
	device_id.devid_s.devtype = DRV_INS_DEVTYPE_MS;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	device_id.devid_s.bus = 2;
	dev_id = device_id.devid;


	// Default to ROTATION_NONE
	_px4_accel.set_device_id(dev_id);
	_px4_gyro.set_device_id(dev_id);
	_px4_mag.set_device_id(dev_id);

	// Set the default values for the baro (which may not change)
	_sensor_baro.device_id = dev_id;
	_sensor_baro.pressure = 0;
	_sensor_baro.temperature = 0;
	_sensor_baro.error_count = 0;
}

MicrosStrain::~MicrosStrain()
{
	if (device_uart.is_open()) {
		device_uart.uart_close();
	}

	_sensor_baro_pub.unadvertise();
	//delete &_sensor_baro_pub;

	PX4_INFO("Destructor");
	_sensor_baro_pub.unadvertise();
	_sensor_selection_pub.unadvertise();
	_vehicle_local_position_pub.unadvertise();
	_vehicle_angular_velocity_pub.unadvertise();
	_vehicle_attitude_pub.unadvertise();
	_global_position_pub.unadvertise();
	_vehicle_odometry_pub.unadvertise();
	_debug_array_pub.unadvertise();
	_estimator_status_pub.unadvertise();


	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool mip_interface_user_recv_from_device(mip_interface *device, uint8_t *buffer, size_t max_length,
		timeout_type wait_time,
		size_t *out_length, timestamp_type *timestamp_out)
{
	(void)device;

	*timestamp_out = hrt_absolute_time();

	int res = device_uart.uart_read(buffer, max_length);

	if (res == -1 && errno != EAGAIN) {
		PX4_DEBUG("RX 1 %d(%d)", res, max_length);
		*out_length = 0;
		return false;
	}

	if (res >= 0) {
		*out_length = res;
		PX4_DEBUG("RX 2 %d(%d)", *out_length, max_length);
	}

	PX4_DEBUG("RX 3 %d(%d)", *out_length, max_length);

	ins->_debug_rx_bytes[0] = math::min<uint32_t>(ins->_debug_rx_bytes[0], *out_length);
	ins->_debug_rx_bytes[1] += *out_length;
	ins->_debug_rx_bytes[2] = math::max<uint32_t>(ins->_debug_rx_bytes[2], *out_length);
	ins->_debug_rx_bytes[3]++;
	return true;
}

bool mip_interface_user_send_to_device(mip_interface *device, const uint8_t *data, size_t length)
{

	PX4_DEBUG("TX %d", length);
	int res = device_uart.uart_write(const_cast<uint8_t *>(data), length);

	if (ins) {
		ins->_debug_tx_bytes += length;
	}

	if (res >= 0) {
		return true;
	}

	return false;

}

int MicrosStrain::connect_at_baud(int32_t baud)
{
	if (device_uart.is_open()) {
		if (device_uart.uart_set_baud(baud) == PX4_ERROR) {
			PX4_INFO(" - Failed to set UART %" PRIu32 " baud", baud);
		}

	} else if (device_uart.uart_open(_port, baud) == PX4_ERROR) {
		PX4_INFO(" - Failed to open UART");
		PX4_ERR("ERROR: Could not open device port!");
		return PX4_ERROR;
	}

	PX4_INFO("Serial Port %s with baud of %" PRIu32 " baud", (device_uart.is_open() ? "CONNECTED" : "NOT CONNECTED"), baud);

	// Re-init the interface with the correct timeouts
	// mip_interface_init(&device, parse_buffer, sizeof(parse_buffer), mip_timeout_from_baudrate(baud) * 1_ms, 250_ms);
	mip_interface_init(&device, parse_buffer, sizeof(parse_buffer), mip_timeout_from_baudrate(baud) * 1_ms, 250_ms,
			   &mip_interface_user_send_to_device, &mip_interface_user_recv_from_device, &mip_interface_default_update, NULL);

	PX4_INFO("mip_base_ping");

	if (mip_base_ping(&device) != MIP_ACK_OK) {
		PX4_INFO(" - Failed to Ping 1");
		usleep(200_ms);

		if (mip_base_ping(&device) != MIP_ACK_OK) {
			PX4_INFO(" - Failed to Ping 2");
			return PX4_ERROR;
		}
	}

	PX4_INFO("Successfully opened and pinged");
	return PX4_OK;
}

void MicrosStrain::set_sensor_rate(mip_descriptor_rate *descriptors, uint16_t len)
{
	// Get the base rate
	uint16_t base_rate;

	if (mip_3dm_get_base_rate(&device, MIP_SENSOR_DATA_DESC_SET, &base_rate) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not get sensor base rate format!");
		return;
	}

	PX4_INFO("The INS base rate is %d", base_rate);

	for (uint16_t i = 0; i < len; i++) {
		// Compute the desired decimation and update all of the sensors in this set
		float decimation = static_cast<float>(base_rate) / static_cast<float>(descriptors[i].decimation);

		descriptors[i].decimation = static_cast<uint16_t>(decimation);
	}

	// Write the settings
	mip_cmd_result res = mip_3dm_write_message_format(&device, MIP_SENSOR_DATA_DESC_SET, len, descriptors);

	if (res != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set sensor message format! Result of %d", res);
		return;
	}
}



void MicrosStrain::initialize_ins()
{
	if (_is_initialized) {
		return;
	}

	// first try default baudrate
	const uint32_t DEFAULT_BAUDRATE = 115200;
	const uint32_t DESIRED_BAUDRATE = 921600;

	if (connect_at_baud(DEFAULT_BAUDRATE) == PX4_ERROR) {

		static constexpr uint32_t BAUDRATES[] {9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600};
		bool is_connected = false;

		for (auto &baudrate : BAUDRATES) {
			if (connect_at_baud(baudrate) == PX4_OK) {
				PX4_INFO("found baudrate %" PRIu32, baudrate);
				is_connected = true;
				break;
			}
		}

		if (!is_connected) {
			_is_init_failed = true;
			PX4_WARN("Could not connect to the device, exiting");
			return;
		}
	}


	PX4_INFO("mip_base_set_idle");

	if (mip_base_set_idle(&device) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set the device to idle!");
		return;
	}

	PX4_INFO("Setting the baud to desired baud rate");

	usleep(500_ms);

	if (mip_3dm_write_uart_baudrate(&device, DESIRED_BAUDRATE) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set the baudrate!");
		_is_init_failed = true;
		return;
	}

	tcflush(device_uart.uart_get_fd(), TCIOFLUSH);

	usleep(500_ms);

	for (int i = 0; i < 10; i++) {
		PX4_INFO("Connection Attempt: %d", i);

		if (connect_at_baud(DESIRED_BAUDRATE) == PX4_OK) {
			break;
		}

		if (i >= 9) {
			PX4_ERR("ERROR: Could not reconnect at desired baud!");
			_is_init_failed = true;
			return;
		}
	}


	// Scaled Gyro and Accel at a high rate
	mip_descriptor_rate imu_sensors[4] = {
		{ MIP_DATA_DESC_SENSOR_ACCEL_SCALED, (uint16_t)_param_pr_update_rate.get()},
		{ MIP_DATA_DESC_SENSOR_GYRO_SCALED, (uint16_t)_param_pr_update_rate.get()},
		{ MIP_DATA_DESC_SENSOR_MAG_SCALED, (uint16_t)_param_sec_update_rate.get()},
		{ MIP_DATA_DESC_SENSOR_PRESSURE_SCALED, (uint16_t)_param_sec_update_rate.get()},

	};

	set_sensor_rate(imu_sensors, 4);


	//
	// Register data callbacks
	//
	mip_interface_register_packet_callback(&device, &sensor_data_handler, MIP_SENSOR_DATA_DESC_SET, false, &sensor_callback,
					       this);


	//
	// Setup the rotation based on PX4 standard rotation sets
	//

	if (mip_3dm_write_sensor_2_vehicle_transform_euler(&device, math::radians<float>(rot_lookup[rotation].roll),
			math::radians<float>(rot_lookup[rotation].pitch),
			math::radians<float>(rot_lookup[rotation].yaw)) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set sensor-to-vehicle transformation!");
		return;
	}


	if (mip_3dm_write_datastream_control(&device, MIP_3DM_DATASTREAM_CONTROL_COMMAND_ALL_STREAMS, true) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not enable the data stream");
		return;
	}

	//
	//Resume the device
	//

	if (mip_base_resume(&device) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not resume the device!");
		return;
	}

	_is_initialized = true;

}

void MicrosStrain::sensor_callback(void *user, const mip_packet *packet, mip::Timestamp timestamp)
{
	MicrosStrain *ref = static_cast<MicrosStrain *>(user);

	if (mip_packet_descriptor_set(packet) != MIP_SENSOR_DATA_DESC_SET) {
		return;
	}

	mip_sensor_scaled_accel_data accel;
	mip_sensor_scaled_gyro_data gyro;
	mip_sensor_scaled_mag_data mag;
	mip_sensor_scaled_pressure_data baro;

	bool accel_valid = false;
	bool gyro_valid = false;
	bool mag_valid = false;
	bool baro_valid = false;

	auto t = hrt_absolute_time() - (ref->_delay_offset * 1_us);

	for (mip_field field = mip_field_first_from_packet(packet); mip_field_is_valid(&field); mip_field_next(&field)) {
		switch (mip_field_field_descriptor(&field)) {
		case MIP_DATA_DESC_SENSOR_ACCEL_SCALED: extract_mip_sensor_scaled_accel_data_from_field(&field, &accel);
			accel_valid = true; break;

		case MIP_DATA_DESC_SENSOR_GYRO_SCALED: extract_mip_sensor_scaled_gyro_data_from_field(&field, &gyro); gyro_valid = true;
			break;

		case MIP_DATA_DESC_SENSOR_MAG_SCALED: extract_mip_sensor_scaled_mag_data_from_field(&field, &mag); mag_valid = true;
			break;

		case MIP_DATA_DESC_SENSOR_PRESSURE_SCALED: extract_mip_sensor_scaled_pressure_data_from_field(&field, &baro);
			baro_valid = true; break;

		default: break;

		}
	}

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

bool MicrosStrain::init()
{
	// Run on fixed interval
	ScheduleOnInterval(_param_ms_schedule.get());

	return true;
}


void MicrosStrain::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	initialize_ins();

	// Initialization failed, stop the module
	if (_is_init_failed) {
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

		_delay_offset = _param_sensor_delay.get();

	}

	mip_interface_update(&device, false);

	perf_end(_loop_perf);
}

int MicrosStrain::task_spawn(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	const char *dev = "/dev/ttyS4";
	int32_t rot = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "d:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			dev = myoptarg;
			break;

		case 'r':
			rot = atoi(myoptarg);

			if (rot >= ROTATION_MAX) {
				rot = ROTATION_NONE;
			}

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
	MicrosStrain *instance = new MicrosStrain(dev, rot);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		// Get a local reference
		ins = instance;

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

int MicrosStrain::print_status()
{
	PX4_INFO_RAW("Serial Port Open %d Handle %d Device %s\n", device_uart.is_open(), device_uart.uart_get_fd(),
		     _port);
	PX4_INFO_RAW("TX Bytes %lu\n", _debug_tx_bytes);
	PX4_INFO_RAW("Min %lu\n", _debug_rx_bytes[0]);
	PX4_INFO_RAW("Total %lu\n", _debug_rx_bytes[1]);
	PX4_INFO_RAW("Max %lu\n", _debug_rx_bytes[2]);
	PX4_INFO_RAW("Avg %f\n", static_cast<double>(_debug_rx_bytes[1] * 1.f / _debug_rx_bytes[3] * 1.f));
	_debug_rx_bytes[0] = UINT32_MAX;

	for (int i = 1; i < 4; i++) {
		_debug_rx_bytes[i] = 0;
	}

	_debug_tx_bytes = 0;

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int MicrosStrain::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MicrosStrain::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Microstrain INS Driver.

Communicates over serial port an utilizes the manufacturer provided MIP SDK.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("Microstrain", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS4", "<file:dev>", "INS Port", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, ROTATION_MAX, "See enum Rotation for values", true);

	return 0;
}

extern "C" __EXPORT int microstrain_main(int argc, char *argv[])
{
	return MicrosStrain::main(argc, argv);
}

