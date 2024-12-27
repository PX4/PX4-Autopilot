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

	/// @brief The main loop of the task.
	void run() final;

private:
	static constexpr int TASK_STACK_SIZE{2048};
	static constexpr int SERIAL_READ_BUFFER_SIZE{128};
	static constexpr int MIN_BYTES_TO_READ{16};
	static constexpr int SERIAL_READ_TIMEOUT_MS{5};
	static constexpr int DATA_BUFFER_SIZE{512};

	// Min length of a valid message. 5 bytes header + 4 bytes CRC + padding to 12 (multiple of 32 bit words)
	static constexpr int MIN_MESSAGE_LENGTH{12};

	void processDataBuffer();

	static bool retrieveProtocolVersionAndMessageType(Ringbuffer& buffer, uint16_t& protocol_ver, uint8_t& message_code);

	static int getMessageLength(CSerialProtocol::EMessageIds messsage_id);

	static uint32_t crc32(const uint32_t* buf, size_t len);

	device::Serial _serial_port;
	Ringbuffer _data_buffer;
	px4::Array<uint8_t, SERIAL_READ_BUFFER_SIZE> _serial_read_buffer;
	px4::Array<uint32_t, 8> _message_storage;

	bool _is_initialized{false};
};

