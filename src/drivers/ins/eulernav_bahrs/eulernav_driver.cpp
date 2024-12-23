#include "eulernav_driver.h"
#include <px4_platform_common/getopt.h>
#include "CSerialProtocol.h"

EulerNavDriver::EulerNavDriver(const char* device_name)
	: _serial_port{device_name, 115200, ByteSize::EightBits, Parity::None, StopBits::One, FlowControl::Disabled}
	, _data_buffer{}
{
	_serial_port.open();

	if (_serial_port.isOpen())
	{
		PX4_INFO("Serial port opened successfully.");
		_is_initialized = true;
	}
	else
	{
		PX4_ERR("Failed to open serial port");
		_is_initialized = false;
	}

	if (_is_initialized)
	{
		if (false == _data_buffer.allocate(DATA_BUFFER_SIZE))
		{
			PX4_ERR("Failed to allocate data buffer");
			_is_initialized = false;
		}
	}
}

EulerNavDriver::~EulerNavDriver()
{
	if (_serial_port.isOpen())
	{
		_serial_port.close();
	}

	_data_buffer.deallocate();
}

int EulerNavDriver::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd("bahrs", SCHED_DEFAULT, SCHED_PRIORITY_FAST_DRIVER,
					 TASK_STACK_SIZE, (px4_main_t)&run_trampoline, argv);

	if (task_id < 0)
	{
		_task_id = -1;
		PX4_ERR("Failed to spawn task.");
	}
	else
	{
		_task_id = task_id;
	}

	return (_task_id < 0) ? 1 : 0;
}

EulerNavDriver* EulerNavDriver::instantiate(int argc, char *argv[])
{
	int option_index = 1;
	const char* option_arg{nullptr};
	const char* device_name{nullptr};

	while (true)
	{
		int option{px4_getopt(argc, argv, "d:", &option_index, &option_arg)};

		if (EOF == option)
		{
			break;
		}

		switch (option)
		{
		case 'd':
			device_name = option_arg;
			break;
		default:
			break;
		}
	}

	return new EulerNavDriver(device_name);
}

int EulerNavDriver::custom_command(int argc, char *argv[])
{
	return print_usage("unrecognized command");
}

int EulerNavDriver::print_usage(const char *reason)
{
	return 0;
}

void EulerNavDriver::run()
{
	while(false == should_exit())
	{
		px4_usleep(1000000);
		PX4_INFO("Running the EulerNavDriver::run...");

		// The declaration of readAtLeast() suggests that the timeout is in microseconds,
		// but it seems to be a bug, as readAtLeast() forwards the value to another method
		// that expects milliseconds.
		const auto bytes_read{_serial_port.readAtLeast(_serial_read_buffer.begin(), _serial_read_buffer.capacity(),
			                                       MIN_BYTES_TO_READ, SERIAL_READ_TIMEOUT_MS)};

		if (bytes_read > 0)
		{
			if (false == _data_buffer.push_back(_serial_read_buffer.begin(), _serial_read_buffer.size()))
			{
				PX4_ERR("No space in data buffer");
			}
		}

		processDataBuffer();
	}
}

void EulerNavDriver::processDataBuffer()
{
	static_assert(MIN_MESSAGE_LENGTH >= (sizeof(CSerialProtocol::SMessageHeader) + sizeof(CSerialProtocol::CrcType_t)));

	// We start from the first element and search for a message header. We remove
	// bytes from the buffer until a header is found. Then we parse the found message
	// and remove the corresponding data bytes. If a full message has not been received
	// yet, we leave the message bytes in the buffer and wait for more data to arrive.

	while (_data_buffer.space_used() >= MIN_MESSAGE_LENGTH)
	{
		uint8_t sync_byte{0U};
		uint16_t protocol_ver{0U};
		uint8_t message_code{0U};
		bool error{false};

		if (0 == _data_buffer.pop_front(&sync_byte, 1))
		{
			error = true;
		}

		if (false == error)
		{
			if (CSerialProtocol::uMarker1_ == sync_byte)
			{
				sync_byte = 0U;

				if (0 == _data_buffer.pop_front(&sync_byte, 1))
				{
					error = true;
				}

				if (false == error)
				{
					if (CSerialProtocol::uMarker2_ == sync_byte)
					{
						error = retrieveProtocolVersionAndMessageType(_data_buffer, protocol_ver, message_code);
					}
				}
			}
		}

		if (false == error)
		{
			switch (static_cast<CSerialProtocol::EMessageIds>(message_code))
			{
			case CSerialProtocol::EMessageIds::eInertialData:
				break;
			case CSerialProtocol::EMessageIds::eNavigationData:
				break;
			default:
				break;
			}
		}
	}

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
