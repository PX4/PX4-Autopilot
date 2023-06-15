#include <px4_platform_common/Serial.hpp>

namespace device
{

Serial::Serial(const char *port, uint32_t baudrate, ByteSize bytesize, Parity parity, StopBits stopbits,
	       FlowControl flowcontrol) :
	_impl(port, baudrate, bytesize, parity, stopbits, flowcontrol)
{


	// TODO: Device

	// set_device_bus_type(device::Device::DeviceBusType::DeviceBusType_SERIAL);

	// char c = _port[strlen(_port) - 1]; // last digit of path (eg /dev/ttyS2)
	// set_device_bus(c - 48); // sub 48 to convert char to integer
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

ssize_t Serial::readAtLeast(uint8_t *buffer, size_t buffer_size, size_t character_count, uint32_t timeout_us)
{
	return _impl.readAtLeast(buffer, buffer_size, character_count, timeout_us);
}

ssize_t Serial::write(const void *buffer, size_t buffer_size)
{
	return _impl.write(buffer, buffer_size);
}

const char *Serial::getPort() const
{
	return _impl.getPort();
}

bool Serial::setPort(const char *port)
{
	return _impl.setPort(port);
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

} // namespace device
