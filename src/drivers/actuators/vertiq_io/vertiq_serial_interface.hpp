#pragma once

#include <px4_log.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include "iq-module-communication-cpp/inc/generic_interface.hpp"

class VertiqSerialInterface
{
public:

	/**
	* @brief Initialize our serial peripheral
	*/
	int init_serial(const char *uart_device);

	/**
	* Turn off and close the serial connection
	*/
	void deinit_serial();

	/**
	* set the Baudrate
	* @param baud
	* @return 0 on success, <0 on error
	*/
	int configure_serial_peripheral(unsigned baud);

	/**
	* @brief check to see if there is any data for us coming in over the serial port
	*/
	int process_serial_rx();

private:

	GenericInterface _iquart_interface;
	uint8_t _bytes_available;
	uint8_t _rx_buf[128];

	static constexpr int FRAME_SIZE = 10;
	int _uart_fd{-1};

#if ! defined(__PX4_QURT)
	struct termios		_orig_cfg;
	struct termios		_cfg;
#endif
	int   _speed = -1;

};


