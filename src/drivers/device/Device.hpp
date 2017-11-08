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
	/**
	 * Destructor.
	 *
	 * Public so that anonymous devices can be destroyed.
	 */
	virtual ~Device();

	/*
	 * Direct access methods.
	 */

	/**
	 * Initialise the driver and make it ready for use.
	 *
	 * @return	OK if the driver initialized OK, negative errno otherwise;
	 */
	virtual int	init();

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
	virtual int	read(unsigned address, void *data, unsigned count);

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
	virtual int	write(unsigned address, void *data, unsigned count);

	/**
	 * Perform a device-specific operation.
	 *
	 * @param operation	The operation to perform.
	 * @param arg		An argument to the operation.
	 * @return		Negative errno on error, OK or positive value on success.
	 */
	virtual int	ioctl(unsigned operation, unsigned &arg);

	/** Device bus types for DEVID */
	enum DeviceBusType {
		DeviceBusType_UNKNOWN = 0,
		DeviceBusType_I2C     = 1,
		DeviceBusType_SPI     = 2,
		DeviceBusType_UAVCAN  = 3,
	};

	/**
	 * Return the bus ID the device is connected to.
	 *
	 * @return The bus ID
	 */
	virtual uint8_t get_device_bus() { return _device_id.devid_s.bus; }

	/**
	 * Return the bus type the device is connected to.
	 *
	 * @return The bus type
	 */
	virtual DeviceBusType get_device_bus_type() { return _device_id.devid_s.bus_type; }

	/**
	 * Return the bus address of the device.
	 *
	 * @return The bus address
	 */
	virtual uint8_t get_device_address() { return _device_id.devid_s.address; }

	/**
	 * Set the device type
	 *
	 * @return The device type
	 */
	virtual void set_device_type(uint8_t devtype) { _device_id.devid_s.devtype = devtype; }

	/*
	  broken out device elements. The bitfields are used to keep
	  the overall value small enough to fit in a float accurately,
	  which makes it possible to transport over the MAVLink
	  parameter protocol without loss of information.
	 */
	struct DeviceStructure {
		enum DeviceBusType bus_type : 3;
		uint8_t bus: 5;    // which instance of the bus type
		uint8_t address;   // address on the bus (eg. I2C address)
		uint8_t devtype;   // device class specific device type
	};

	union DeviceId {
		struct DeviceStructure devid_s;
		uint32_t devid;
	};

protected:
	const char	*_name;			/**< driver name */
	char		*_lock_name;		/**< name of the semaphore */
	bool		_debug_enabled;		/**< if true, debug messages are printed */
	union DeviceId	_device_id;             /**< device identifier information */

	/**
	 * Constructor
	 *
	 * @param name		Driver name
	 */
	Device(const char *name);

	/**
	 * Take the driver lock.
	 *
	 * Each driver instance has its own lock/semaphore.
	 *
	 * Note that we must loop as the wait may be interrupted by a signal.
	 *
	 * Careful: lock() calls cannot be nested!
	 */
	void		lock()
	{
		do {} while (px4_sem_wait(&_lock) != 0);
	}

	/**
	 * Release the driver lock.
	 */
	void		unlock()
	{
		px4_sem_post(&_lock);
	}

	DeviceId &get_device_id() { return _device_id; }

	px4_sem_t		_lock; /**< lock to protect access to all class members (also for derived classes) */

private:

	/** disable copy construction for this and all subclasses */
	Device(const Device &);

	/** disable assignment for this and all subclasses */
	Device &operator = (const Device &);

};

} // namespace device

#endif /* _DEVICE_DEVICE_HPP */
