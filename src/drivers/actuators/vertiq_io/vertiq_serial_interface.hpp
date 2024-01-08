#pragma once

#include <px4_log.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include "iq-module-communication-cpp/inc/generic_interface.hpp"
#include "iq-module-communication-cpp/inc/propeller_motor_control_client.hpp"

class VertiqSerialInterface
{
public:

	VertiqSerialInterface(uint8_t num_clients);

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
	int process_serial_rx(ClientAbstract *test[2]);

	/**
	* @brief check to see if there is any data that we need to transmit over serial
	*/
	int process_serial_tx();

	/**
	* @brief give access to our iquart interface so that others can use it
	* @return a pointer to our _iquart_itnerface object
	*/
	GenericInterface *get_iquart_interface();

private:
	uint8_t _number_of_clients;

	GenericInterface _iquart_interface;

	uint8_t _bytes_available;

	//Buffers for data to transmit or that we're receiving
	uint8_t _rx_buf[128];
	uint8_t _tx_buf[128];

	//The port that we're using for communication
	int _uart_fd{-1};

#if ! defined(__PX4_QURT)
	struct termios		_orig_cfg;
	struct termios		_cfg;
#endif
	int   _speed = -1;


};


