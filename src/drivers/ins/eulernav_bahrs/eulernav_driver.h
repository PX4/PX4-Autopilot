#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/Serial.hpp>
#include <Ringbuffer.hpp>
#include <containers/Array.hpp>
#include "CSerialProtocol.h"

class EulerNavDriver : public ModuleBase<EulerNavDriver>
{
public:
	EulerNavDriver(const char* device_name);

	~EulerNavDriver();

	/// @brief Required by ModuleBase
	static int task_spawn(int argc, char *argv[]);

	/// @brief Required by ModuleBase
	static EulerNavDriver* instantiate(int argc, char *argv[]);

	/// @brief Required by ModuleBase
	static int custom_command(int argc, char *argv[]);

	/// @brief Required by ModuleBase
	static int print_usage(const char *reason = nullptr);

	/// @brief Overload of the method from the ModuleBase
	int print_status() final;

	/// @brief The main loop of the task.
	void run() final;

private:
	class Statistics
	{
	public:
		uint32_t _total_bytes_received{0U};
		uint32_t _inertial_message_counter{0U};
		uint32_t _navigation_message_counter{0U};
		uint32_t _crc_failures{0U};
		hrt_abstime _start_time{0U};
	};

	class Config
	{
	public:
		/// Driver task stack size
		static constexpr uint32_t TASK_STACK_SIZE{2048};

		/// Buffer size for serial port read operations
		static constexpr uint32_t SERIAL_READ_BUFFER_SIZE{128};

		/// Minimum number of bytes to wait for when reading from a serial port
		static constexpr uint32_t MIN_BYTES_TO_READ{16};

		/// A timeout for serial port read operation
		static constexpr uint32_t SERIAL_READ_TIMEOUT_US{5000};

		/// Size of a ring buffer for storing RX data stream
		static constexpr uint32_t DATA_BUFFER_SIZE{512};

		/// Min length of a valid message. 5 bytes header + 4 bytes CRC + padding to 12 (multiple of 32 bit words)
		static constexpr int32_t MIN_MESSAGE_LENGTH{12};
	};

	class NextMessageInfo
	{
	public:
		bool _is_detected{false};
		uint16_t _protocol_version{0U};
		uint8_t _message_code{0U};
	};

	void initialize();

	void deinitialize();

	void processDataBuffer();

	static bool findNextMessageHeader(Ringbuffer& buffer);

	static bool retrieveProtocolVersionAndMessageType(Ringbuffer& buffer, uint16_t& protocol_ver, uint8_t& message_code);

	void decodeMessageAndPublishData(const uint8_t* data, CSerialProtocol::EMessageIds messsage_id);

	static int32_t getMessageLength(CSerialProtocol::EMessageIds messsage_id);

	static uint32_t crc32(const uint32_t* buf, size_t len);

	device::Serial _serial_port;
	Ringbuffer _data_buffer;
	uint8_t _serial_read_buffer[Config::SERIAL_READ_BUFFER_SIZE];
	uint32_t _message_storage[8];
	NextMessageInfo _next_message_info{};
	Statistics _statistics{};
	bool _is_initialized{false};
};

