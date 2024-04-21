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

#include <SerialImpl.hpp>

#include <px4_platform_common/SerialCommon.hpp>

using device::SerialConfig::ByteSize;
using device::SerialConfig::Parity;
using device::SerialConfig::StopBits;
using device::SerialConfig::FlowControl;

namespace device __EXPORT
{

class Serial
{
public:
	Serial();
	Serial(const char *port, uint32_t baudrate = 57600,
	       ByteSize bytesize = ByteSize::EightBits, Parity parity = Parity::None,
	       StopBits stopbits = StopBits::One, FlowControl flowcontrol = FlowControl::Disabled);
	virtual ~Serial();

	// Open sets up the port and gets it configured based on desired configuration
	// The port is always opened in NON BLOCKING mode.
	bool open();
	bool isOpen() const;

	bool close();

	ssize_t read(uint8_t *buffer, size_t buffer_size);
	ssize_t readAtLeast(uint8_t *buffer, size_t buffer_size, size_t character_count = 1, uint32_t timeout_us = 0);

	ssize_t write(const void *buffer, size_t buffer_size);

	void flush();

	// If port is already open then the following configuration functions
	// will reconfigure the port. If the port is not yet open then they will
	// simply store the configuration in preparation for the port to be opened.

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

	bool getSingleWireMode() const;
	bool setSingleWireMode();

	bool getSwapRxTxMode() const;
	bool setSwapRxTxMode();

	bool getInvertedMode() const;
	bool setInvertedMode(bool enable);

	static bool validatePort(const char *port);
	bool setPort(const char *port);
	const char *getPort() const;

private:
	// Disable copy constructors
	Serial(const Serial &);
	Serial &operator=(const Serial &);

	// platform implementation
	SerialImpl _impl;
};

} // namespace device
