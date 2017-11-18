/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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

/**
 * @file Device.hpp
 *
 * Definitions for the generic base classes in the device framework.
 */

#ifndef _DEVICE_DEVICE_HPP
#define _DEVICE_DEVICE_HPP

/*
 * Includes here should only cover the needs of the framework definitions.
 */
#include <px4_config.h>
#include <px4_posix.h>

#include <drivers/drv_device.h>

#define DEVICE_LOG(FMT, ...) PX4_LOG_NAMED(_name, FMT, ##__VA_ARGS__)
#define DEVICE_DEBUG(FMT, ...) PX4_LOG_NAMED_COND(_name, _debug_enabled, FMT, ##__VA_ARGS__)

/**
 * Namespace encapsulating all device framework classes, functions and data.
 */
namespace device
{

/**
 * Fundamental base class for all physical drivers (I2C, SPI).
 *
 * This class provides the basic driver template for I2C and SPI devices
 */
class __EXPORT Device
{
public:
	virtual ~Device() = default;

	// no copy, assignment, move, move assignment
	Device(const Device &) = delete;
	Device &operator=(const Device &) = delete;
	Device(Device &&) = delete;
	Device &operator=(Device &&) = delete;

	/**
	 * Initialise the driver and make it ready for use.
	 *
	 * @return	OK if the driver initialized OK, negative errno otherwise;
	 */
	virtual int	init() = 0;

	/**
	 * Read directly from the device.
	 *
	 * The actual size of each unit quantity is device-specific.
	 *
	 * @param offset	The device address at which to start reading
	 * @param data		The buffer into which the read values should be placed.
	 * @param count		The number of items to read.
	 * @return		The number of items read on success, negative errno otherwise.
	 */
	virtual int	read(unsigned address, void *data, unsigned count) { return -ENODEV; }

	/**
	 * Write directly to the device.
	 *
	 * The actual size of each unit quantity is device-specific.
	 *
	 * @param address	The device address at which to start writing.
	 * @param data		The buffer from which values should be read.
	 * @param count		The number of items to write.
	 * @return		The number of items written on success, negative errno otherwise.
	 */
	virtual int	write(unsigned address, void *data, unsigned count) { return -ENODEV; }

	virtual int reset() { return PX4_OK; }

	virtual int test(unsigned arg1 = 0, unsigned arg2 = 0) { return PX4_ERROR; }

	virtual int measure(unsigned address = 0) { return PX4_ERROR; }

	/** Device bus types for DEVID */
	enum DeviceBusType {
		DeviceBusType_UNKNOWN = 0,
		DeviceBusType_I2C     = 1,
		DeviceBusType_SPI     = 2,
		DeviceBusType_UAVCAN  = 3,
	};

	uint32_t get_device_id() const { return _device_id.devid; }

	DeviceBusType get_device_bus_type() const { return _device_id.devid_s.bus_type; }
	uint8_t get_device_bus() const { return _device_id.devid_s.bus; }
	uint8_t get_device_address() const { return _device_id.devid_s.address; }

	void set_device_type(uint8_t devtype) { _device_id.devid_s.devtype = devtype; }
	void set_device_address(int address) { _device_id.devid_s.address = address; }

	virtual bool external() { return false; }

protected:

	// TODO: make _device_id private
	union DeviceId {
		uint32_t devid;

		// broken out device elements
		struct DeviceStructure {
			enum DeviceBusType bus_type : 3;
			uint8_t bus: 5;    // which instance of the bus type
			uint8_t address;   // address on the bus (eg. I2C address)
			uint8_t devtype;   // device class specific device type
		} devid_s;

	} _device_id;             /**< device identifier information */

	const char	*_name{nullptr};			/**< driver name */
	bool		_debug_enabled{false};		/**< if true, debug messages are printed */

	Device(const char *name) : _name(name)
	{
		/* setup a default device ID. When bus_type is UNKNOWN the
		   other fields are invalid */
		_device_id.devid_s.bus_type = DeviceBusType_UNKNOWN;
		_device_id.devid_s.bus = 0;
		_device_id.devid_s.address = 0;
		_device_id.devid_s.devtype = 0;
	}

	Device(DeviceBusType bus_type, uint8_t bus, uint8_t address, uint8_t devtype)
	{
		_device_id.devid_s.bus_type = bus_type;
		_device_id.devid_s.bus = bus;
		_device_id.devid_s.address = address;
		_device_id.devid_s.devtype = devtype;
	}
};

} // namespace device

#endif /* _DEVICE_DEVICE_HPP */
