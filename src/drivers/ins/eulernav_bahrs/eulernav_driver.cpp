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
	const auto start_time{hrt_absolute_time()};
	auto time_of_previous_statistics_print{start_time};

	while(false == should_exit())
	{
#ifndef EULERNAV_BAHRS_PARSER_DEBUG
		const auto bytes_read{_serial_port.readAtLeast(_serial_read_buffer, sizeof(_serial_read_buffer),
			                                       MIN_BYTES_TO_READ, SERIAL_READ_TIMEOUT_US)};

		_statistics._total_bytes_read += bytes_read;

		if (bytes_read > 0)
		{
			if (false == _data_buffer.push_back(_serial_read_buffer, bytes_read))
			{
				PX4_ERR("No space in data buffer");
			}
		}
#else
		static int counter = 0;

		if (counter < 6)
		{
			uint8_t test_data_part_1[] = { 0x05, 0xce, 0xF0, 0x00, 0xab, 0xcd, 0x12,  // Rubbish
				0x4e, 0x45, 0x02, 0x00, 0x01, 0x34, 0xce, 0xff, 0xd3, 0xff, 0x70, 0xe6, 0x05, 0x00, 0xfa, 0xff, 0x00, 0x00, 0x3f, 0x00, 0x32, 0x7d, 0x13, 0x65, // Inertial message
				0x4e, 0x45, 0x02, 0x00, 0x02, 0x9a, 0xb0, 0x22, 0xfe, 0xff, 0x3b, 0x00, 0xa3, 0xff, 0x30, 0xc8, 0x1f, 0x00, 0x00, 0x00, 0x82, 0x98, 0xa3, 0x42, // Navigation message
				0x4e, 0x45, 0x02, 0x00, 0x01, 0x35, 0xce, 0xff, 0xe2, 0xff // Inertial message part 1
			};

			uint8_t test_data_part_2[] = { 0x65, 0xe6, 0x00, 0x00, 0xf9, 0xff, 0x00, 0x00, 0x3f, 0x00, 0x1d, 0x14, 0xf1, 0xf0, // Inertial message part 2
				0x4e, 0x45, 0x02, 0x00, 0x01, 0x36, 0xcf, 0xff, 0xda, 0xff, 0x6e, 0xe6, 0x00, 0x00, 0xf8, 0xff, 0x00, 0x00, 0x3f, 0x00, 0x9f, 0x1b, 0x2b, 0xd1, // Inertial message
				0x00, 0x4e, 0xcd, 0x34, 0x77, 0x86, 0xFF // Rubbish
			};

			uint8_t* data_to_push{nullptr};
			size_t byte_count_to_push{0U};

			if (counter % 2 == 0)
			{
				data_to_push = test_data_part_1;
				byte_count_to_push = sizeof(test_data_part_1);
			}
			else
			{
				data_to_push = test_data_part_2;
				byte_count_to_push = sizeof(test_data_part_2);
			}

			_data_buffer.push_back(data_to_push, byte_count_to_push);
			printf("Data buffer size: %d\n", _data_buffer.space_used());
			++counter;
		}
#endif // EULERNAV_BAHRS_PARSER_DEBUG

		processDataBuffer();

		if (hrt_elapsed_time(&time_of_previous_statistics_print) >= STATISTICS_PRINT_PERIOD)
		{
			PX4_INFO("Elapsed time: %llu [us]. Total bytes received: %lu.\n", hrt_elapsed_time(&start_time), _statistics._total_bytes_read);
			PX4_INFO("Inertial messages received: %lu. Navigation messages received: %lu.\n", _statistics._inertial_message_counter, _statistics._navigation_message_counter);
			time_of_previous_statistics_print = hrt_absolute_time();
		}
	}
}

void EulerNavDriver::processDataBuffer()
{
	static_assert(MIN_MESSAGE_LENGTH >= (sizeof(CSerialProtocol::SMessageHeader) + sizeof(CSerialProtocol::CrcType_t)));
	using EMessageIds = CSerialProtocol::EMessageIds;

	while (_data_buffer.space_used() >= MIN_MESSAGE_LENGTH)
	{
		if (false == _next_message_detected)
		{
			_next_message_detected = findNextMessageHeader(_data_buffer);

			if (_next_message_detected)
			{
				if (false == retrieveProtocolVersionAndMessageType(_data_buffer, _next_message_protocol_version, _next_message_code))
				{
					_next_message_detected = false;
				}
			}
		}

		if (_next_message_detected)
		{
			static_assert(sizeof(CSerialProtocol::SMessageHeader) < MIN_MESSAGE_LENGTH);

			const EMessageIds message_id{static_cast<EMessageIds>(_next_message_code)};
			const int32_t message_length{getMessageLength(message_id)};

			if ((message_length < 0) || (message_length < MIN_MESSAGE_LENGTH) ||
			    (message_length > static_cast<int32_t>(sizeof(_message_storage))) || ((message_length % sizeof(uint32_t)) != 0U))
			{
				// The message is unknown, not supported, or does not fit into the temporary storage.
				_next_message_detected = false;
			}

			if (_next_message_detected)
			{
				const int32_t bytes_to_retrieve{message_length - static_cast<int32_t>(sizeof(CSerialProtocol::SMessageHeader))};

				if (static_cast<int32_t>(_data_buffer.space_used()) < bytes_to_retrieve)
				{
					// Do nothing and wait for more bytes to arrive.
					break;
				}
				else
				{
					// Get message from the data buffer
					uint8_t* bytes{reinterpret_cast<uint8_t*>(_message_storage)};

					bytes[0] = CSerialProtocol::uMarker1_;
					bytes[1] = CSerialProtocol::uMarker2_;
					bytes[2] = reinterpret_cast<uint8_t*>(&_next_message_protocol_version)[0];
					bytes[3] = reinterpret_cast<uint8_t*>(&_next_message_protocol_version)[1];
					bytes[4] = _next_message_code;

					if (static_cast<size_t>(bytes_to_retrieve) == _data_buffer.pop_front(bytes + sizeof(CSerialProtocol::SMessageHeader), bytes_to_retrieve))
					{
						const uint32_t message_length_in_words{message_length / sizeof(uint32_t)};
						const uint32_t actual_crc{crc32(_message_storage, message_length_in_words - 1)};
						const uint32_t expected_crc = _message_storage[message_length_in_words - 1];

						if (expected_crc != actual_crc)
						{
							PX4_INFO("Protocol version %u, message code %u: CRC failed.\n", _next_message_protocol_version, _next_message_code);
						}
						else
						{
							decodeMessageAndPublishData(bytes, message_id);
						}
					}

					_next_message_detected = false;
				}
			}

		}
	}

}

bool EulerNavDriver::findNextMessageHeader(Ringbuffer& buffer)
{
	bool result{false};

	while (buffer.space_used() >= sizeof(CSerialProtocol::SMessageHeader))
	{
		uint8_t sync_byte{0U};

		if (1 == buffer.pop_front(&sync_byte, 1))
		{
			if (CSerialProtocol::uMarker1_ == sync_byte)
			{
				sync_byte = 0U;

				if (1 == buffer.pop_front(&sync_byte, 1))
				{
					if (CSerialProtocol::uMarker2_ == sync_byte)
					{
						result = true;
						break;
					}
				}

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
		++_statistics._inertial_message_counter;
		break;
	case CSerialProtocol::EMessageIds::eNavigationData:
		++_statistics._navigation_message_counter;
		break;
	default:
		break;
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
