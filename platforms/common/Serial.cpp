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

#include <px4_platform_common/Serial.hpp>

namespace device
{

Serial::Serial() :
	_impl(nullptr, 57600, ByteSize::EightBits, Parity::None, StopBits::One, FlowControl::Disabled) {}


Serial::Serial(const char *port, uint32_t baudrate, ByteSize bytesize, Parity parity, StopBits stopbits,
	       FlowControl flowcontrol) :
	_impl(port, baudrate, bytesize, parity, stopbits, flowcontrol)
{
	// If no baudrate was specified then set it to a reasonable default value
	if (baudrate == 0) {
		(void) _impl.setBaudrate(9600);
	}
}

Serial::~Serial()
{
}

bool Serial::open()
{
	return _impl.open();
}

bool Serial::isOpen() const
{
	return _impl.isOpen();
}

bool Serial::close()
{
	return _impl.close();
}

ssize_t Serial::read(uint8_t *buffer, size_t buffer_size)
{
	return _impl.read(buffer, buffer_size);
}

ssize_t Serial::readAtLeast(uint8_t *buffer, size_t buffer_size, size_t character_count, uint32_t timeout_ms)
{
	return _impl.readAtLeast(buffer, buffer_size, character_count, timeout_ms);
}

ssize_t Serial::write(const void *buffer, size_t buffer_size)
{
	return _impl.write(buffer, buffer_size);
}

void Serial::flush()
{
	return _impl.flush();
}

uint32_t Serial::getBaudrate() const
{
	return _impl.getBaudrate();
}

bool Serial::setBaudrate(uint32_t baudrate)
{
	return _impl.setBaudrate(baudrate);
}

ByteSize Serial::getBytesize() const
{
	return _impl.getBytesize();
}

bool Serial::setBytesize(ByteSize bytesize)
{
	return _impl.setBytesize(bytesize);
}

Parity Serial::getParity() const
{
	return _impl.getParity();
}

bool Serial::setParity(Parity parity)
{
	return _impl.setParity(parity);
}

StopBits Serial::getStopbits() const
{
	return _impl.getStopbits();
}

bool Serial::setStopbits(StopBits stopbits)
{
	return _impl.setStopbits(stopbits);
}

FlowControl Serial::getFlowcontrol() const
{
	return _impl.getFlowcontrol();
}

bool Serial::setFlowcontrol(FlowControl flowcontrol)
{
	return _impl.setFlowcontrol(flowcontrol);
}

bool Serial::getSingleWireMode() const
{
	return _impl.getSingleWireMode();
}
bool Serial::setSingleWireMode()
{
	return _impl.setSingleWireMode();
}

bool Serial::getSwapRxTxMode() const
{
	return _impl.getSwapRxTxMode();
}
bool Serial::setSwapRxTxMode()
{
	return _impl.setSwapRxTxMode();
}

bool Serial::getInvertedMode() const
{
	return _impl.getInvertedMode();
}

bool Serial::setInvertedMode(bool enable)
{
	return _impl.setInvertedMode(enable);
}

const char *Serial::getPort() const
{
	return _impl.getPort();
}

bool Serial::validatePort(const char *port)
{
	return SerialImpl::validatePort(port);
}

bool Serial::setPort(const char *port)
{
	return _impl.setPort(port);
}

} // namespace device
