
#include "SerialImpl.hpp"

#include <errno.h>
#include <fcntl.h> // open
#include <poll.h>
#include <string.h> // strncpy
#include <termios.h>

#include <px4_log.h>
#include <lib/mathlib/mathlib.h>

namespace device
{

SerialImpl::SerialImpl(const char *port, uint32_t baudrate, ByteSize bytesize, Parity parity, StopBits stopbits,
		       FlowControl flowcontrol) :
	_bytesize(bytesize),
	_parity(parity),
	_stopbits(stopbits),
	_flowcontrol(flowcontrol)
{
	if (port) {
		strncpy(_port, port, sizeof(_port) - 1);
		_port[sizeof(_port) - 1] = '\0';
	}

	setBaudrate(baudrate);
}

SerialImpl::~SerialImpl()
{
	if (isOpen()) {
		close();
	}
}

bool SerialImpl::open()
{
	// if already open first close
	if (isOpen()) {
		return true;
	}

	int serial_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (serial_fd < 0) {
		PX4_ERR("failed to open %s, errno: %d, %s", _port, errno, strerror(errno));
		return false;
	}

	_serial_fd = serial_fd;
	_open = true;

	if (configure()) {
		_configured = true;
	}

	return _open;
}

bool SerialImpl::isOpen() const
{
	return _open;
}

bool SerialImpl::close()
{
	int ret = 0;

	if (_serial_fd >= 0) {
		ret = ::close(_serial_fd);
	}

	_serial_fd = -1;
	_open = false;
	_configured = false;

	return (ret == 0);
}

ssize_t SerialImpl::read(uint8_t *buffer, size_t buffer_size)
{
	if (!_open) {
		open();
	}

	if (!_configured) {
		configure();
	}

	int ret_read = ::read(_serial_fd, buffer, buffer_size);

	if (ret_read == -1) {
		switch (errno) {
		case EBADF:
			// invalid file descriptor
			PX4_ERR("%s read error %d %s", _port, errno, strerror(errno));
			close();
			break;
		}

		PX4_DEBUG("%s read error %d %s", _port, errno, strerror(errno));

	} else {
		_bytes_read += ret_read;
	}

	return ret_read;
}

ssize_t SerialImpl::readAtLeast(uint8_t *buffer, size_t buffer_size, size_t character_count, uint32_t timeout_us)
{
	const size_t required_bytes = math::min(buffer_size, character_count);
	const hrt_abstime start_time_us = hrt_absolute_time();

	if (!_open) {
		open();
	}

	if (!_configured) {
		configure();
	}

	while (true) {
		int bytes_available = 0;
		int ioctl_ret = ::ioctl(_serial_fd, FIONREAD, (unsigned long)&bytes_available);

		if ((ioctl_ret == 0) && (bytes_available >= (int)required_bytes)) {
			return read(buffer, buffer_size);
		}

		if (bytes_available < (int)required_bytes) {

			if (timeout_us > 0) {
				const uint64_t elapsed_us = hrt_elapsed_time(&start_time_us);

				if (elapsed_us > timeout_us) {
					//PX4_WARN("readAtLeast timeout %d bytes available (%llu us elapsed)", bytes_available, elapsed_us);
					return -1;
				}
			}

			int desired_bytes = required_bytes - bytes_available;

			uint32_t sleeptime_us = desired_bytes * 1'000'000 / (_baudrate / 10);

			if (desired_bytes == 1 || sleeptime_us <= 1000) {

				int poll_timeout_ms = 0;

				if (timeout_us > 0) {
					poll_timeout_ms = timeout_us / 1000;
				}

				pollfd fds[1];
				fds[0].fd = _serial_fd;
				fds[0].events = POLLIN;

				int poll_ret = ::poll(fds, 1, poll_timeout_ms);

				if (poll_ret > 0) {
					if (fds[0].revents & POLLIN) {
						// There is data to read
					}
				}

			} else {
				if (timeout_us > 0) {
					sleeptime_us = math::min(sleeptime_us, timeout_us);
				}

				//PX4_INFO("%s %d/%d bytes available, sleep time %" PRIu32 "us", _port, bytes_available, required_bytes, sleeptime_us);

				px4_usleep(sleeptime_us);
			}
		}
	}

	return -1;
}

ssize_t SerialImpl::write(const void *buffer, size_t buffer_size)
{
	if (!_open) {
		open();
	}

	if (!_configured) {
		configure();
	}

	// POLLOUT: Writing is now possible

	// FIONWRITE: Get the number of bytes that have been written to the TX buffer.
	// FIONSPACE: Get the number of free bytes in the TX buffer

	int ret_write = ::write(_serial_fd, buffer, buffer_size);

	if (ret_write == -1) {

		switch (errno) {
		case EBADF:
			// invalid file descriptor
			close();
			break;
		}

		PX4_ERR("%s write error %d %s", _port, errno, strerror(errno));

	} else {
		_bytes_written += ret_write;
	}

	return ret_write;
}

const char *SerialImpl::getPort() const
{
	return _port;
}

bool SerialImpl::setPort(const char *port)
{
	if (port) {

		if (strcmp(port, _port) == 0) {
			return true;
		}

		if (::access(port, R_OK) != 0) {
			PX4_ERR("port %s not accessible", port);
			return false;
		}

		strncpy(_port, port, sizeof(_port) - 1);
		_port[sizeof(_port) - 1] = '\0';

		_configured = false;

		return true;
	}

	return false;
}

uint32_t SerialImpl::getBaudrate() const
{
	return _baudrate;
}

bool SerialImpl::setBaudrate(uint32_t baudrate)
{
	// process baud rate change
	int speed = 0;

	switch (baudrate) {
	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

#if defined(B460800)

	case 460800: speed = B460800; break;
#endif // B460800

#if defined(B500000)

	case 500000: speed = B500000; break;
#endif // B500000

#if defined(B576000)

	case 576000: speed = B576000; break;
#endif // B576000

#if defined(B921600)

	case 921600: speed = B921600; break;
#endif // B921600

#if defined(B1000000)

	case 1000000: speed = B1000000; break;
#endif // B1000000

#if defined(B1500000)

	case 1500000: speed = B1500000; break;
#endif // B1500000

#if defined(B2000000)

	case 2000000: speed = B2000000; break;
#endif // B2000000

#if defined(B2500000)

	case 2500000: speed = B2500000; break;
#endif // B2500000

#if defined(B3000000)

	case 3000000: speed = B3000000; break;
#endif // B3000000

#if defined(B3500000)

	case 3500000: speed = B3500000; break;
#endif // B3500000

#if defined(B4000000)

	case 4000000: speed = B4000000; break;
#endif // B4000000

	default:
		PX4_ERR("unknown baudrate: %" PRIu32, baudrate);
		return false;
	}

	// check if already configured
	if (speed == (int)_baudrate) {
		return true;
	}

	if (speed != 0) {
		_baudrate = speed;
		_configured = false;
		return true;
	}

	return false;
}

ByteSize SerialImpl::getBytesize() const
{
	return _bytesize;
}

void SerialImpl::setBytesize(ByteSize bytesize)
{
	if (bytesize != _bytesize) {
		_bytesize = bytesize;
		_configured = false;
	}
}

Parity SerialImpl::getParity() const
{
	return _parity;
}

void SerialImpl::setParity(Parity parity)
{
	if (parity != _parity) {
		_parity = parity;
		_configured = false;
	}
}

StopBits SerialImpl::getStopbits() const
{
	return _stopbits;
}

void SerialImpl::setStopbits(StopBits stopbits)
{
	if (stopbits != _stopbits) {
		_stopbits = stopbits;
		_configured = false;
	}
}

FlowControl SerialImpl::getFlowcontrol() const
{
	return _flowcontrol;
}

void SerialImpl::setFlowcontrol(FlowControl flowcontrol)
{
	if (flowcontrol != _flowcontrol) {
		_flowcontrol = flowcontrol;
		_configured = false;
	}
}

bool SerialImpl::configure()
{
	_configured = false;

	if (!isOpen()) {
		return false;
	}

	// fill the struct for the new configuration
	struct termios uart_config;

	if (tcgetattr(_serial_fd, &uart_config) != 0) {
		PX4_ERR("tcgetattr failed");
		return false;
	}

	// Input modes (c_iflag): Turn off input processing
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
	uart_config.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow ctrl

	// Output modes (c_oflag): Turn off output processing
	uart_config.c_oflag = 0;
	uart_config.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	uart_config.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

	// Local modes (c_lflag): No line processing
	uart_config.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);

	// Control Modes (c_cflag)
	uart_config.c_cflag = CREAD | CLOCAL;


	// ByteSize: termios CSIZE
	uart_config.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below

	switch (_bytesize) {
	case ByteSize::FiveBits:
		uart_config.c_cflag |= CS5;
		break;

	case ByteSize::SixBits:
		uart_config.c_cflag |= CS6;
		break;

	case ByteSize::SevenBits:
		uart_config.c_cflag |= CS7;
		break;

	case ByteSize::EightBits:
		uart_config.c_cflag |= CS8;
		break;
	}

	// StopBits: termios CSTOPB
	switch (_stopbits) {
	case StopBits::One:
		uart_config.c_cflag &= ~CSTOPB;
		break;

	case StopBits::Two:
		uart_config.c_cflag |= CSTOPB;
		break;
	}

	// Parity: termios PARENB, PARODD
	switch (_parity) {
	case Parity::None:
		uart_config.c_cflag &= ~(PARENB | PARODD);
		break;

	case Parity::Odd:
		uart_config.c_cflag |= (PARENB | PARODD);
		break;

	case Parity::Even:
		uart_config.c_cflag |= PARENB;
		uart_config.c_cflag &= ~PARODD;
		break;
	}

	// FlowControl: termios CRTSCTS
	switch (_flowcontrol) {
	case FlowControl::Disabled:
		uart_config.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
		break;

	case FlowControl::Enabled:
		uart_config.c_cflag |= CRTSCTS; // Enable RTS/CTS hardware flow control
		break;
	}

	// VMIN = 0, VTIME = 0: No blocking, return immediately with what is available
	uart_config.c_cc[VMIN] = 0;
	uart_config.c_cc[VTIME] = 0;


	// baudrate: input speed (cfsetispeed)
	int cfsetispeed_ret = cfsetispeed(&uart_config, _baudrate);

	if (cfsetispeed_ret != 0) {
		PX4_ERR("cfsetispeed failed: %d (errno=%d, %s)", cfsetispeed_ret, errno, strerror(errno));
		return false;
	}

	// baudrate: output speed (cfsetospeed)
	int cfsetospeed_ret = cfsetospeed(&uart_config, _baudrate);

	if (cfsetospeed_ret != 0) {
		PX4_ERR("cfsetospeed failed: %d (errno=%d, %s)", cfsetospeed_ret, errno, strerror(errno));
		return false;
	}

	// tcsetattr: set attributes
	int tcsetattr_ret = tcsetattr(_serial_fd, TCSANOW, &uart_config);

	if (tcsetattr_ret != 0) {
		PX4_ERR("tcsetattr failed: %d (errno=%d, %s)", tcsetattr_ret, errno, strerror(errno));
		return false;
	}

	PX4_INFO("%s configured successfully", _port);
	_configured = true;
	return true;
}

} // namespace device
