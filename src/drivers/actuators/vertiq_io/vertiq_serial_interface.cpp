
#include "vertiq_serial_interface.hpp"

#define UART_BAUDRATE 115200

void VertiqSerialInterface::deinit_serial()
{
	if (_uart_fd >= 0) {
		close(_uart_fd);
		_uart_fd = -1;
	}
}

int VertiqSerialInterface::init_serial(const char *uart_device)
{
	deinit_serial();
	_uart_fd = ::open(uart_device, O_RDWR | O_NOCTTY);

	if (_uart_fd < 0) {
		PX4_ERR("failed to open serial port: %s err: %d", uart_device, errno);
		return -errno;
	}

	PX4_INFO("Opened serial port successfully");

	return configure_serial_peripheral(UART_BAUDRATE);
}

int VertiqSerialInterface::configure_serial_peripheral(unsigned baud)
{
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	default:
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_uart_fd, &uart_config);

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		return -errno;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		return -errno;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		return -errno;
	}

	return 0;
}

int VertiqSerialInterface::process_serial_rx()
{
	if (_uart_fd < 0) {
		return -1;
	}

	// read from the uart. This must be non-blocking, so check first if there is data available
	int bytes_available = 0;
	int ret = ioctl(_uart_fd, FIONREAD, (unsigned long)&bytes_available);

	if (ret != 0) {
		PX4_ERR("Reading error");
		return -1;

	} else {
		PX4_INFO("able to check the port for data and found %d bytes of data", bytes_available);
	}

	if (bytes_available > 0) {
		const int buf_length = FRAME_SIZE;
		uint8_t buf[buf_length];

		int num_read = read(_uart_fd, buf, buf_length);

		for (uint8_t i = 0; i < num_read; i++) {
			uint8_t char_to_write = buf[i];
			write(_uart_fd, &char_to_write, 1);
		}
	}

	return 1;
}
