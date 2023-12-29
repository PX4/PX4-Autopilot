/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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

// Bring in the correct platform implementation
#ifdef __PX4_NUTTX
#include "nuttx/SerialImpl.hpp"
#elif defined(__PX4_QURT)
#include "qurt/SerialImpl.hpp"
#else
#include "posix/SerialImpl.hpp"
#endif

namespace device __EXPORT
{

class Serial
{
public:
	// Baud rate can be selected with constructor or by using setBaudrate
	Serial(const char *port, uint32_t baudrate = 0);
	virtual ~Serial();

	// Open sets up the port and gets it configured. Unless an alternate mode
	// is selected the port will be configured with parity disabled and 1 stop bit.
	bool open();
	bool isOpen() const;

	bool close();

	ssize_t read(uint8_t *buffer, size_t buffer_size);
	ssize_t readAtLeast(uint8_t *buffer, size_t buffer_size, size_t character_count = 1, uint32_t timeout_us = 0);

	ssize_t write(const void *buffer, size_t buffer_size);

	uint32_t getBaudrate() const;

	// If the port has already been opened it will be reconfigured with a change
	// of baudrate.
	bool setBaudrate(uint32_t baudrate);

	// SBUS has special configuration considerations and methods so it
	// is given a special mode. It has parity enabled and 2 stop bits
	bool getSBUSMode() const;
	bool setSBUSMode(bool enable);

	const char *getPort() const;

private:
	// Disable copy constructors
	Serial(const Serial &);
	Serial &operator=(const Serial &);

	// platform implementation
	SerialImpl _impl;
};

} // namespace device
