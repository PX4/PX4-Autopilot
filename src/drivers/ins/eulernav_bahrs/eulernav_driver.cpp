#include "eulernav_driver.h"
#include <px4_platform_common/getopt.h>

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
		/*const auto bytes_read{_serial_port.readAtLeast(_serial_read_buffer.begin(), _serial_read_buffer.capacity(),
			                                       MIN_BYTES_TO_READ, SERIAL_READ_TIMEOUT_MS)};*/

		uint8_t test_data[] = {0x4e, 0x45, 0x02, 0x00, 0x01, 0x34, 0xce, 0xff, 0xd3, 0xff, 0x70, 0xe6, 0x05, 0x00, 0xfa, 0xff, 0x00, 0x00, 0x3f, 0x00, 0x32, 0x7d, 0x13, 0x65, \
			0x4e, 0x45, 0x02, 0x00, 0x02, 0x9a, 0xb0, 0x22, 0xfe, 0xff, 0x3b, 0x00, 0xa3, 0xff, 0x30, 0xc8, 0x1f, 0x00, 0x00, 0x00, 0x82, 0x98, 0xa3, 0x42, \
			0x4e, 0x45, 0x02, 0x00, 0x01, 0x35, 0xce, 0xff, 0xe2, 0xff, 0x65, 0xe6, 0x00, 0x00, 0xf9, 0xff, 0x00, 0x00, 0x3f, 0x00, 0x1d, 0x14, 0xf1, 0xf0, \
			0x4e, 0x45, 0x02, 0x00, 0x01, 0x36, 0xcf, 0xff, 0xda, 0xff, 0x6e, 0xe6, 0x00, 0x00, 0xf8, 0xff, 0x00, 0x00, 0x3f, 0x00, 0x9f, 0x1b, 0x2b, 0xd1
		};

		_data_buffer.push_back(test_data, sizeof(test_data));

		printf("Data buffer size: %d\n", _data_buffer.space_used());

		/*if (bytes_read > 0)
		{
			if (false == _data_buffer.push_back(_serial_read_buffer.begin(), _serial_read_buffer.size()))
			{
				PX4_ERR("No space in data buffer");
			}
		}*/

		processDataBuffer();
	}
}

void EulerNavDriver::processDataBuffer()
{
	static_assert(MIN_MESSAGE_LENGTH >= (sizeof(CSerialProtocol::SMessageHeader) + sizeof(CSerialProtocol::CrcType_t)));
	using EMessageIds = CSerialProtocol::EMessageIds;

	// We start from the first element and search for a message header. We remove
	// bytes from the buffer until a header is found. Then we parse the found message
	// and remove the corresponding data bytes. If a full message has not been received
	// yet, we leave the message bytes in the buffer and wait for more data to arrive.

	bool message_header_found{false};

	while (_data_buffer.space_used() >= MIN_MESSAGE_LENGTH)
	{
		uint8_t sync_byte{0U};
		uint16_t protocol_ver{0U};
		uint8_t message_code{0U};
		bool error{false};

		if (false == message_header_found)
		{
			if (0 == _data_buffer.pop_front(&sync_byte, 1))
			{
				error = true;
			}

			if (false == error)
			{
				if (CSerialProtocol::uMarker1_ == sync_byte)
				{
					sync_byte = 0U;
					printf("Found sync byte 1.\n");

					if (0 == _data_buffer.pop_front(&sync_byte, 1))
					{
						error = true;
					}

					if (false == error)
					{
						if (CSerialProtocol::uMarker2_ == sync_byte)
						{
							if (false == retrieveProtocolVersionAndMessageType(_data_buffer, protocol_ver, message_code))
							{
								error = true;
							}

							printf("Found sync byte 2.\n");

							if (false == error)
							{
								printf("Message found. Protocol version: %d. Message code: %d.\n", protocol_ver, message_code);
								message_header_found = true;
							}
							else
							{
								printf("Failed to retrieve message attributes.\n");
							}
						}
					}
				}
			}
		}

		if (message_header_found)
		{
			const EMessageIds message_id{static_cast<EMessageIds>(message_code)};
			const int32_t message_length{getMessageLength(message_id)};

			if (message_length > 0)
			{
				const uint32_t bytes_to_retrieve{static_cast<uint32_t>(message_length) - sizeof(CSerialProtocol::SMessageHeader)};
				printf("Starting to parse the message.\n");

				if (_data_buffer.space_used() >= bytes_to_retrieve)
				{
					if ((bytes_to_retrieve > 0) &&
						(bytes_to_retrieve + sizeof(CSerialProtocol::SMessageHeader)) <= sizeof(_message_storage))
					{
						printf("Preparing CRC.\n");
						uint8_t* bytes = reinterpret_cast<uint8_t*>(_message_storage);
						bytes[0] = CSerialProtocol::uMarker1_;
						bytes[1] = CSerialProtocol::uMarker2_;
						bytes[2] = reinterpret_cast<uint8_t*>(&protocol_ver)[0];
						bytes[3] = reinterpret_cast<uint8_t*>(&protocol_ver)[1];
						bytes[4] = message_code;

						printf("Bytes to retrieve: %lu.\n.", bytes_to_retrieve);

						if (bytes_to_retrieve != _data_buffer.pop_front(bytes + 5, bytes_to_retrieve))
						{
							error = true;
						}

						printf("Message:\n");

						for (int i = 0; i < message_length; ++i)
						{
							printf("%X, ", bytes[i]);
						}
						printf("\n");

						if (false == error)
						{
							uint32_t message_length_in_words{message_length / sizeof(uint32_t)};
							const uint32_t actual_crc{crc32(_message_storage, message_length_in_words - 1)};
							const uint32_t expected_crc = _message_storage[message_length_in_words - 1];

							if (expected_crc != actual_crc)
							{
								error = true;
								printf("CRC FAILED for: message code: %d.\n", message_code);
								printf("Message length in words: %lu.\n", message_length_in_words);
								printf("Expected CRC: %lu. Actual CRC: %lu.\n", expected_crc, actual_crc);
							}
							else
							{
								printf("CRC PASSED for: message code: %d.\n", message_code);
							}

						}

						if (false == error)
						{
							// decode message
						}
					}

					message_header_found = false;
				}
				else
				{
					// wait for more data to arrive
				}

			}
			else
			{
				message_header_found = false;
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

int EulerNavDriver::getMessageLength(CSerialProtocol::EMessageIds messsage_id)
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
