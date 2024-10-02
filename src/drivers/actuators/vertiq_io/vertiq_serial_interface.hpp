/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

class VertiqSerialInterface
{
public:
	/**
	 * @brief Construct a new Vertiq Serial Interface object
	 *
	 */
	VertiqSerialInterface();

	/**
	* @brief Initialize our serial peripheral
	*/
	int InitSerial(const char *uart_device, unsigned baud);

	/**
	* Turn off and close the serial connection
	*/
	void DeinitSerial();

	/**
	* set the Baudrate
	* @param baud
	* @return 0 on success, <0 on error
	*/
	int ConfigureSerialPeripheral(unsigned baud);

	/**
	 * @brief Check to see if there are any valid IQUART packets for us to read
	 *
	 * @return true If there is a packet
	 * @return false If there is not a packet
	 */
	bool CheckForRx();

	/**
	 * @brief Read a packet from our packet finder and return a pointer to the beginning of the data
	 *
	 * @return uint8_t* A pointer to the start of the packet
	 */
	uint8_t *ReadAndSetRxBytes();

	/**
	 * @brief Send the data we got to all of our clients in order to keep everyone up to date with responses
	 *
	 * @param client_array A pointer to our array of clients
	 * @param number_of_clients The number of clients in the array
	 * @return int
	 */
	void ProcessSerialRx(ClientAbstract **client_array, uint8_t number_of_clients);

	/**
	* @brief check to see if there is any data that we need to transmit over serial
	*/
	void ProcessSerialTx();

	/**
	* @brief give access to our iquart interface so that others can use it
	* @return a pointer to our _iquart_itnerface object
	*/
	GenericInterface *GetIquartInterface();

private:
	/**
	 * @brief Ensures that the serial port is open. Opens it if not
	 *
	 */
	void ReOpenSerial();

	GenericInterface _iquart_interface;

	uint8_t _bytes_available;

	//Buffers for data to transmit or that we're receiving
	uint8_t _rx_buf[256];
	uint8_t _tx_buf[256];

	//The port that we're using for communication
	int _uart_fd{-1};
	char _port_in_use[20] {};

#if ! defined(__PX4_QURT)
	struct termios		_orig_cfg;
	struct termios		_cfg;
#endif
	int   _speed = -1;


};
