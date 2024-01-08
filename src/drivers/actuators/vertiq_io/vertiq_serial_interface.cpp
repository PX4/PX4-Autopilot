
#include "vertiq_serial_interface.hpp"

#define UART_BAUDRATE 115200

VertiqSerialInterface::VertiqSerialInterface(uint8_t num_clients) :
	_number_of_clients(num_clients)
{}

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

int VertiqSerialInterface::process_serial_rx(ClientAbstract *test[2])
{
	if (_uart_fd < 0) {
		return -1;
	}

	// read from the uart. This must be non-blocking, so check first if there is data available
	_bytes_available = 0;
	int ret = ioctl(_uart_fd, FIONREAD, (unsigned long)&_bytes_available);

	if (ret != 0) {
		PX4_ERR("Reading error");
		return -1;
	}

	if (_bytes_available > 0) {
		//Read the bytes available for us
		read(_uart_fd, _rx_buf, _bytes_available);

		//Put the data into our IQUART handler
		_iquart_interface.SetRxBytes(_rx_buf, _bytes_available);

		//Pointer to our RX data
		uint8_t *rx_buf_ptr = _rx_buf;

		//While we've got packets to look at
		while (_iquart_interface.PeekPacket(&rx_buf_ptr, &_bytes_available) == 1) {
			for (uint8_t i = 0; i < _number_of_clients; i++) {
				test[i]->ReadMsg(rx_buf_ptr, _bytes_available);
			}

			_iquart_interface.DropPacket();
		}

	}

	return 1;
}

int VertiqSerialInterface::process_serial_tx()
{
	//while there's stuff to write, write it
	//Clients are responsible for adding TX messages to the buffer through get/set/save commands
	while (_iquart_interface.GetTxBytes(_tx_buf, _bytes_available)) {
		write(_uart_fd, _tx_buf, _bytes_available);
	}

	return 1;
}

GenericInterface *VertiqSerialInterface::get_iquart_interface()
{
	return &_iquart_interface;
}
