/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#pragma once

#include <px4_log.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include "iq-module-communication-cpp/inc/generic_interface.hpp"
#include "iq-module-communication-cpp/inc/propeller_motor_control_client.hpp"
#include "ifci.hpp"

class VertiqSerialInterface
{
public:

	VertiqSerialInterface(uint8_t num_clients);

	/**
	* @brief Initialize our serial peripheral
	*/
	int init_serial(const char *uart_device, unsigned baud);

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
	int process_serial_rx(IFCI *motor_interface, ClientAbstract **array_of_clients);

	/**
	* @brief check to see if there is any data that we need to transmit over serial
	*/
	int process_serial_tx();

	/**
	* @brief give access to our iquart interface so that others can use it
	* @return a pointer to our _iquart_itnerface object
	*/
	GenericInterface *get_iquart_interface();

	void SetNumberOfClients(uint8_t number_of_clients);

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


