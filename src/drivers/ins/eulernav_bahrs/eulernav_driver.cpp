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

#include "eulernav_driver.h"
#include <px4_platform_common/getopt.h>
#include <drivers/drv_sensor.h>
#include <matrix/Quaternion.hpp>
#include <matrix/Euler.hpp>
#include <atmosphere/atmosphere.h>

EulerNavDriver::EulerNavDriver(const char *device_name)
	: ModuleParams{nullptr}
	, _serial_port{device_name, 115200, ByteSize::EightBits, Parity::None, StopBits::One, FlowControl::Disabled}
	, _data_buffer{}
	, _px4_accel{DRV_INS_DEVTYPE_BAHRS}
	, _px4_gyro{DRV_INS_DEVTYPE_BAHRS}
{
}

EulerNavDriver::~EulerNavDriver()
{
	deinitialize();
}

int EulerNavDriver::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd("bahrs", SCHED_DEFAULT, SCHED_PRIORITY_FAST_DRIVER,
					 Config::TASK_STACK_SIZE, (px4_main_t)&run_trampoline, argv);

	if (task_id < 0) {
		_task_id = -1;
		PX4_ERR("Failed to spawn task.");

	} else {
		_task_id = task_id;
	}

	return (_task_id < 0) ? 1 : 0;
}

EulerNavDriver *EulerNavDriver::instantiate(int argc, char *argv[])
{
	int option_index = 1;
	const char *option_arg{nullptr};
	const char *device_name{nullptr};

	while (true) {
		int option{px4_getopt(argc, argv, "d:", &option_index, &option_arg)};

		if (EOF == option) {
			break;
		}

		switch (option) {
		case 'd':
			device_name = option_arg;
			break;

		default:
			break;
		}
	}

	auto *instance = new EulerNavDriver(device_name);

	if (nullptr != instance) {
		instance->initialize();

	} else {
		PX4_ERR("Failed to initialize EULER-NAV driver.");
	}

	return instance;
}

int EulerNavDriver::custom_command(int argc, char *argv[])
{
	return print_usage("unrecognized command");
}

int EulerNavDriver::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the EULER-NAV Baro-Inertial AHRS.

### Examples

Attempt to start driver on a specified serial device.
$ eulernav_bahrs start -d /dev/ttyS1
Stop driver
$ eulernav_bahrs stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("eulernav_bahrs", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("ins");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");

	return PX4_OK;
}

int EulerNavDriver::print_status()
{
	if (_is_initialized)
	{
		PX4_INFO("Elapsed time: %llu [us].\n", hrt_elapsed_time(&_statistics.start_time));
		PX4_INFO("Total bytes received: %lu.\n", _statistics.total_bytes_received);
		PX4_INFO("Inertial messages received: %lu. Navigation messages received: %lu.\n",
			_statistics.inertial_message_counter, _statistics.navigation_message_counter);
		PX4_INFO("Failed CRC count: %lu.\n", _statistics.crc_failures);

	}
	else
	{
		PX4_INFO("Initialization failed. The driver is not running.\n");
	}

	return PX4_OK;
}

void EulerNavDriver::run()
{
	_statistics.start_time = hrt_absolute_time();

	while(!should_exit())
	{
		if (_is_initialized)
		{
			const auto bytes_read{_serial_port.readAtLeast(_serial_read_buffer, sizeof(_serial_read_buffer),
								       Config::MIN_BYTES_TO_READ, Config::SERIAL_READ_TIMEOUT_MS)};

			_statistics.total_bytes_received += bytes_read;

			if (bytes_read > 0)
			{
				if (!_data_buffer.push_back(_serial_read_buffer, bytes_read))
				{
					PX4_ERR("No space in data buffer");
				}
			}

			processDataBuffer();
		}
		else
		{
			// The driver failed to initialize in the instantiate() method, so we exit the loop.
			request_stop();
		}
	}
}

void EulerNavDriver::initialize()
{
	if (!_is_initialized)
	{
		if (_serial_port.open())
		{
			PX4_INFO("Serial port opened successfully.");
			_is_initialized = true;
		}
		else
		{
			PX4_ERR("Failed to open serial port");
		}

		if (_is_initialized)
		{
			if (!_data_buffer.allocate(Config::DATA_BUFFER_SIZE))
			{
				PX4_ERR("Failed to allocate data buffer");
				_is_initialized = false;
			}
		}

		if (_is_initialized)
		{
			_attitude_pub.advertise();
			_barometer_pub.advertise();
		}
	}
}

void EulerNavDriver::deinitialize()
{
	if (_serial_port.isOpen())
	{
		_serial_port.close();
	}

	_data_buffer.deallocate();
	_attitude_pub.unadvertise();
	_barometer_pub.unadvertise();
	_is_initialized = false;
}

void EulerNavDriver::processDataBuffer()
{
	static_assert(Config::MIN_MESSAGE_LENGTH >= (sizeof(CSerialProtocol::SMessageHeader) + sizeof(CSerialProtocol::CrcType_t)));
	using EMessageIds = CSerialProtocol::EMessageIds;

	while (_data_buffer.space_used() >= Config::MIN_MESSAGE_LENGTH)
	{
		if (!_next_message_info.is_detected)
		{
			_next_message_info.is_detected = findNextMessageHeader(_data_buffer);

			if (!_next_message_info.is_detected)
			{
				// No message header found, wait for more bytes to arrive.
				break;
			}

			if (!retrieveProtocolVersionAndMessageType(_data_buffer, _next_message_info.protocol_version, _next_message_info.message_code))
			{
				_next_message_info.is_detected = false;
			}
		}

		if (_next_message_info.is_detected)
		{
			static_assert(sizeof(CSerialProtocol::SMessageHeader) < Config::MIN_MESSAGE_LENGTH);

			const EMessageIds message_id{static_cast<EMessageIds>(_next_message_info.message_code)};
			const int32_t message_length{getMessageLength(message_id)};

			if ((message_length < 0) || (message_length < Config::MIN_MESSAGE_LENGTH) ||
			    (message_length > static_cast<int32_t>(sizeof(_message_storage))) || ((message_length % sizeof(uint32_t)) != 0U))
			{
				// The message is unknown, not supported, or does not fit into the temporary storage.
				_next_message_info.is_detected = false;
				continue;
			}

			const int32_t bytes_to_retrieve{message_length - static_cast<int32_t>(sizeof(CSerialProtocol::SMessageHeader))};

			if (static_cast<int32_t>(_data_buffer.space_used()) < bytes_to_retrieve)
			{
				// Do nothing and wait for more bytes to arrive.
				break;
			}

			// Get message from the data buffer
			uint8_t* bytes{reinterpret_cast<uint8_t*>(_message_storage)};

			bytes[0] = CSerialProtocol::uMarker1_;
			bytes[1] = CSerialProtocol::uMarker2_;
			bytes[2] = reinterpret_cast<uint8_t*>(&_next_message_info.protocol_version)[0];
			bytes[3] = reinterpret_cast<uint8_t*>(&_next_message_info.protocol_version)[1];
			bytes[4] = _next_message_info.message_code;

			if (static_cast<size_t>(bytes_to_retrieve) == _data_buffer.pop_front(bytes + sizeof(CSerialProtocol::SMessageHeader), bytes_to_retrieve))
			{
				const uint32_t message_length_in_words{message_length / sizeof(uint32_t)};
				const uint32_t actual_crc{crc32(_message_storage, message_length_in_words - 1)};
				const uint32_t expected_crc = _message_storage[message_length_in_words - 1];

				if (expected_crc != actual_crc)
				{
					++_statistics.crc_failures;
				}
				else
				{
					decodeMessageAndPublishData(bytes, message_id);
				}
			}

			_next_message_info.is_detected = false;
		}
	}
}

bool EulerNavDriver::findNextMessageHeader(Ringbuffer& buffer)
{
	bool result{false};

	while (buffer.space_used() >= sizeof(CSerialProtocol::SMessageHeader))
	{
		uint8_t sync_byte{0U};

		if ((1 == buffer.pop_front(&sync_byte, 1)) && (CSerialProtocol::uMarker1_ == sync_byte))
		{
			sync_byte = 0U;

			if ((1 == buffer.pop_front(&sync_byte, 1)) && (CSerialProtocol::uMarker2_ == sync_byte))
			{
				result = true;
				break;
			}
		}
	}

	return result;
}

bool EulerNavDriver::retrieveProtocolVersionAndMessageType(Ringbuffer& buffer, uint16_t& protocol_ver, uint8_t& message_code)
{
	bool status{true};
	auto bytes_to_pop{sizeof(protocol_ver)};

	// Note: BAHRS uses little endian
	if (bytes_to_pop != buffer.pop_front(reinterpret_cast<uint8_t*>(&protocol_ver), bytes_to_pop))
	{
		status = false;
	}

	if (status)
	{
		bytes_to_pop = 1;

		if (bytes_to_pop != buffer.pop_front(&message_code, bytes_to_pop))
		{
			status = false;
		}
	}

	return status;
}

void EulerNavDriver::decodeMessageAndPublishData(const uint8_t* data, CSerialProtocol::EMessageIds messsage_id)
{
	switch (messsage_id)
	{
	case CSerialProtocol::EMessageIds::eInertialData:
		handleInertialDataMessage(data);
		break;
	case CSerialProtocol::EMessageIds::eNavigationData:
		handleNavigationDataMessage(data);
		break;
	default:
		break;
	}
}

void EulerNavDriver::handleInertialDataMessage(const uint8_t* data)
{
	const CSerialProtocol::SInertialDataMessage* imu_msg{reinterpret_cast<const CSerialProtocol::SInertialDataMessage*>(data)};

	if (nullptr != imu_msg)
	{
		const auto& imu_data{imu_msg->oInertialData_};
		const bool accel_valid{((imu_data.uValidity_ & BIT_VALID_SPECIFIC_FORCE_X) > 0) &&
					((imu_data.uValidity_ & BIT_VALID_SPECIFIC_FORCE_Y) > 0) &&
					((imu_data.uValidity_ & BIT_VALID_SPECIFIC_FORCE_Z) > 0)};

		const bool gyro_valid{((imu_data.uValidity_ & BIT_VALID_ANGULAR_RATE_X) > 0) &&
					((imu_data.uValidity_ & BIT_VALID_ANGULAR_RATE_Y) > 0) &&
					((imu_data.uValidity_ & BIT_VALID_ANGULAR_RATE_Z) > 0)};

		const auto time = hrt_absolute_time();

		if (accel_valid)
		{
			const float accel_x{CSerialProtocol::skfSpecificForceScale_ * static_cast<float>(imu_data.iSpecificForceX_)};
			const float accel_y{CSerialProtocol::skfSpecificForceScale_ * static_cast<float>(imu_data.iSpecificForceY_)};
			const float accel_z{CSerialProtocol::skfSpecificForceScale_ * static_cast<float>(imu_data.iSpecificForceZ_)};

			_px4_accel.update(time, accel_x, accel_y, accel_z);
		}

		if (gyro_valid)
		{
			const float gyro_x{CSerialProtocol::skfAngularRateScale_ * static_cast<float>(imu_data.iAngularRateX_)};
			const float gyro_y{CSerialProtocol::skfAngularRateScale_ * static_cast<float>(imu_data.iAngularRateY_)};
			const float gyro_z{CSerialProtocol::skfAngularRateScale_ * static_cast<float>(imu_data.iAngularRateZ_)};

			_px4_gyro.update(time, gyro_x, gyro_y, gyro_z);
		}

		++_statistics.inertial_message_counter;
	}
}

void EulerNavDriver::handleNavigationDataMessage(const uint8_t* data)
{
	const CSerialProtocol::SNavigationDataMessage* nav_msg{reinterpret_cast<const CSerialProtocol::SNavigationDataMessage*>(data)};

	if (nullptr != nav_msg)
	{
		const auto& nav_data{nav_msg->oNavigationData_};
		const bool roll_valid{(nav_data.uValidity_ & BIT_VALID_ROLL) > 0};
		const bool pitch_valid{(nav_data.uValidity_ & BIT_VALID_PITCH) > 0};
		const bool yaw_valid{(nav_data.uValidity_ & BIT_VALID_MAGNETIC_HEADING) > 0};
		const auto time{hrt_absolute_time()};

		if (roll_valid && pitch_valid && yaw_valid)
		{
			const float roll{CSerialProtocol::skfAngleScale_ * static_cast<float>(nav_data.iRoll_)};
			const float pitch{CSerialProtocol::skfAngleScale_ * static_cast<float>(nav_data.iPitch_)};
			const float yaw{CSerialProtocol::skfAngleScale_ * static_cast<float>(nav_data.uMagneticHeading_)};

			const matrix::Quaternionf quat{matrix::Eulerf{roll, pitch, yaw}};
			px4::msg::VehicleAttitude attitude{};

			attitude.q[0] = quat(0);
			attitude.q[1] = quat(1);
			attitude.q[2] = quat(2);
			attitude.q[3] = quat(3);

			attitude.timestamp = time;
			attitude.timestamp_sample = time;

			_attitude_pub.publish(attitude);
		}

		const bool height_valid{(nav_data.uValidity_ & BIT_VALID_HEIGHT) > 0};

		if (height_valid)
		{
			const float height{(CSerialProtocol::skfHeightScale_ * static_cast<float>(nav_data.uPressureHeight_)) - CSerialProtocol::skfHeighOffset_};
			px4::msg::SensorBaro pressure{};

			pressure.pressure = atmosphere::getPressureFromAltitude(height);

			// EULER-NAV Baro-Inertial AHRS provides height estimate from a Kalman filter. It has got low noise and resolution
			// of about 17 cm. It causes PX4 autopilot to mistakenly report that pressure signal is stale. In order to prevent
			// the false alarms we add a small noise to the received height data.
			if (_statistics.navigation_message_counter % 2U == 0)
			{
				pressure.pressure += 0.01F;
			}

			pressure.timestamp = time;
			pressure.timestamp_sample = time;
			pressure.device_id = DRV_INS_DEVTYPE_BAHRS;
			pressure.temperature = NAN;

			_barometer_pub.publish(pressure);
		}

		++_statistics.navigation_message_counter;
	}
}

int32_t EulerNavDriver::getMessageLength(CSerialProtocol::EMessageIds messsage_id)
{
	int message_length{-1};

	switch (messsage_id)
	{
	case CSerialProtocol::EMessageIds::eInertialData:
		message_length = sizeof(CSerialProtocol::SInertialDataMessage);
		break;
	case CSerialProtocol::EMessageIds::eNavigationData:
		message_length = sizeof(CSerialProtocol::SNavigationDataMessage);
		break;
	default:
		break;
	}

	return message_length;
}

uint32_t EulerNavDriver::crc32(const uint32_t* buffer, size_t length)
{
	uint32_t crc = 0xFFFFFFFF;

	for (size_t i = 0; i < length; ++i)
	{
		crc = crc ^ buffer[i];

		for (uint8_t j = 0; j < 32; j++)
		{
			if (crc & 0x80000000)
			{
				crc = (crc << 1) ^ 0x04C11DB7;
			}
			else
			{
				crc = (crc << 1);
			}
		}
	}

	return crc;
}
