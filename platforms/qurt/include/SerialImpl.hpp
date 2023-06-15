

#pragma once

#include <unistd.h>

#include <px4_platform_common/SerialCommon.hpp>

using device::SerialConfig::ByteSize;
using device::SerialConfig::Parity;
using device::SerialConfig::StopBits;
using device::SerialConfig::FlowControl;

namespace device
{

class SerialImpl
{
public:

	SerialImpl(const char *port, uint32_t baudrate, ByteSize bytesize, Parity parity, StopBits stopbits,
		   FlowControl flowcontrol);
	virtual ~SerialImpl();

	bool open();
	bool isOpen() const;

	bool close();

	ssize_t read(uint8_t *buffer, size_t buffer_size);
	ssize_t readAtLeast(uint8_t *buffer, size_t buffer_size, size_t character_count = 1, uint32_t timeout_us = 0);

	ssize_t write(const void *buffer, size_t buffer_size);

	const char *getPort() const;
	bool setPort(const char *port);

	uint32_t getBaudrate() const;
	bool setBaudrate(uint32_t baudrate);

	ByteSize getBytesize() const;
	bool setBytesize(ByteSize bytesize);

	Parity getParity() const;
	bool setParity(Parity parity);

	StopBits getStopbits() const;
	bool setStopbits(StopBits stopbits);

	FlowControl getFlowcontrol() const;
	bool setFlowcontrol(FlowControl flowcontrol);

private:

	int _serial_fd{-1};

	size_t _bytes_read{0};
	size_t _bytes_written{0};

	bool _open{false};

	char _port[32] {};

	uint32_t _baudrate{0};

	ByteSize _bytesize{ByteSize::EightBits};
	Parity _parity{Parity::None};
	StopBits _stopbits{StopBits::One};
	FlowControl _flowcontrol{FlowControl::Disabled};

	// Mutex used to lock the read functions
	//pthread_mutex_t read_mutex;

	// Mutex used to lock the write functions
	//pthread_mutex_t write_mutex;
};

} // namespace device
