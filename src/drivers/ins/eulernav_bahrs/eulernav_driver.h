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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/Serial.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <uORB/topics/sensor_baro.h>
#include <Ringbuffer.hpp>
#include <containers/Array.hpp>
#include "CSerialProtocol.h"

class EulerNavDriver : public ModuleBase<EulerNavDriver>, public ModuleParams
{
public:
	/// @brief Class constructor
	/// @param device_name Serial port to open
	EulerNavDriver(const char *device_name);

	~EulerNavDriver();

	/// @brief Required by ModuleBase
	static int task_spawn(int argc, char *argv[]);

	/// @brief Required by ModuleBase
	static EulerNavDriver *instantiate(int argc, char *argv[]);

	/// @brief Required by ModuleBase
	static int custom_command(int argc, char *argv[]);

	/// @brief Required by ModuleBase
	static int print_usage(const char *reason = nullptr);

	/// @brief Overload of the method from the ModuleBase
	int print_status() final;

	/// @brief The main loop of the task.
	void run() final;

private:
	/// @brief Driver performance indicators
	class Statistics
	{
	public:
		uint32_t total_bytes_received{0U}; ///< Total number of received bytes
		uint32_t inertial_message_counter{0U}; ///< Total number of received inertial data messages
		uint32_t navigation_message_counter{0U}; ///< Total number of received navigation messages
		uint32_t crc_failures{0U}; ///< CRC failure counter
		hrt_abstime start_time{0U}; ///< Driver start time, [us]
	};

	/// @brief  Configuration constants
	class Config
	{
	public:
		static constexpr uint32_t TASK_STACK_SIZE{2048}; ///< Driver task stack size
		static constexpr uint32_t SERIAL_READ_BUFFER_SIZE{128}; ///< Buffer size for serial port read operations
		static constexpr uint32_t MIN_BYTES_TO_READ{16}; ///< Minimum number of bytes to wait for when reading from a serial port
		static constexpr uint32_t SERIAL_READ_TIMEOUT_MS{5}; ///< A timeout for serial port read operation
		static constexpr uint32_t DATA_BUFFER_SIZE{512}; ///< Size of a ring buffer for storing RX data stream
		static constexpr int32_t MIN_MESSAGE_LENGTH{12}; ///< Min length of a valid message. 5 bytes header + 4 bytes CRC + padding to 12 (multiple of 32 bit words)
	};

	/// @brief An object for grouping message-related attributes
	class NextMessageInfo
	{
	public:
		bool is_detected{false}; ///< A flag to indicate that the next message header was detected
		uint16_t protocol_version{0U}; ///< Protocol version retrieved from the next message header
		uint8_t message_code{0U}; ///< Message code retrieved from the next message header
	};

	/// @brief Perform initialization
	/// If the driver is not initialized, the function does the following:
	/// 1. Opens serial port
	/// 2. Allocates a ring buffer for RX data stream
	/// 3. Cleans up if any of the previous operations fails
	/// 4. Sets the "is initialized" flag to true on success
	void initialize();

	/// @brief De-initialize
	/// The function does the following:
	/// 1. Close the serial port, if it is opened
	/// 2. Deallocates the ring buffer
	/// 3. Resets the "is initialized" flag to false
	void deinitialize();

	/// @brief Process data in the ring buffer.
	/// Extracts and parses protocol messages.
	void processDataBuffer();

	/// @brief Searches the ring buffer for sync bytes.
	/// @param buffer Ring buffer to search
	/// @return True if sync bytes found, false if the number of bytes remaining in the buffer is less then message header length.
	static bool findNextMessageHeader(Ringbuffer &buffer);

	/// @brief Get protocol version and message code from the ring buffer.
	/// @param buffer The buffer to process
	/// @param protocol_ver Output protocol version
	/// @param message_code Output message code
	/// @return True on success, false on failure
	static bool retrieveProtocolVersionAndMessageType(Ringbuffer &buffer, uint16_t &protocol_ver, uint8_t &message_code);

	/// @brief Decode a message from BAHRS and publish its content.
	/// @param data An array of message bytes
	/// @param messsage_id Message ID
	void decodeMessageAndPublishData(const uint8_t *data, CSerialProtocol::EMessageIds messsage_id);

	/// @brief Decode and publish IMU data.
	/// @param data Inertial message data bytes
	void handleInertialDataMessage(const uint8_t *data);

	/// @brief Decode and publish vehicle attitude and pressure height.
	/// @param data Navigation message data bytes
	void handleNavigationDataMessage(const uint8_t *data);

	/// @brief Get message length by message ID
	/// @param messsage_id Query message ID
	/// @return Message length, or -1 if the message ID is unknown or not supported.
	static int32_t getMessageLength(CSerialProtocol::EMessageIds messsage_id);

	/// @brief Perform CRC
	/// @param buf Data buffer pointer
	/// @param len Number of words to include in the CRC.
	/// @return CRC value
	static uint32_t crc32(const uint32_t *buf, size_t len);

	device::Serial _serial_port; ///< Serial port object to read data from
	Ringbuffer _data_buffer; ///< A buffer for RX data stream
	uint8_t _serial_read_buffer[Config::SERIAL_READ_BUFFER_SIZE]; ///< A buffer for serial port read operation
	uint32_t _message_storage[8]; ///< A temporary storage for BAHRS messages being extracted
	NextMessageInfo _next_message_info{}; ///< Attributes of the next message detected in the data buffer
	Statistics _statistics{}; ///< Driver performance indicators
	bool _is_initialized{false}; ///< Initialization flag
	PX4Accelerometer _px4_accel; ///< Accelerometer sensor object for publishing acceleration data
	PX4Gyroscope _px4_gyro; ///< Gyroscope sensor object for publishing angular rate data
	uORB::PublicationMulti<px4::msg::VehicleAttitude> _attitude_pub{ORB_ID(vehicle_attitude)}; ///< Vehicle attitude publisher
	uORB::PublicationMulti<px4::msg::SensorBaro> _barometer_pub{ORB_ID(sensor_baro)}; ///< Pressure data publisher
};
