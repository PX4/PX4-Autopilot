/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file vdevice.h
 *
 * Definitions for the generic base classes in the virtual device framework.
 */

#pragma once 

/*
 * Includes here should only cover the needs of the framework definitions.
 */
#include <px4_posix.h>

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <semaphore.h>

#define DEVICE_LOG(FMT, ...) PX4_LOG_NAMED(_name, FMT, ##__VA_ARGS__)
#define DEVICE_DEBUG(FMT, ...) PX4_LOG_NAMED_COND(_name, _debug_enabled, FMT, ##__VA_ARGS__)

/**
 * Namespace encapsulating all device framework classes, functions and data.
 */
namespace device __EXPORT
{

struct file_t {
	int fd;
	int flags;
	mode_t mode;
	void *priv;
	void *vdev;

	file_t() : fd(-1), flags(0), priv(NULL), vdev(NULL) {}
	file_t(int f, void *c, int d) : fd(d), flags(f), priv(NULL), vdev(c) {}
};

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
	virtual int	dev_read(unsigned address, void *data, unsigned count);

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
	virtual int	dev_write(unsigned address, void *data, unsigned count);

        /**
         * Perform a device-specific operation.
         *
         * @param operation     The operation to perform.
         * @param arg           An argument to the operation.
         * @return              Negative errno on error, OK or positive value on success.
         */
        virtual int     dev_ioctl(unsigned operation, unsigned &arg);

	/*
	  device bus types for DEVID
	 */
	enum DeviceBusType {
		DeviceBusType_UNKNOWN = 0,
		DeviceBusType_I2C     = 1,
		DeviceBusType_SPI     = 2,
		DeviceBusType_UAVCAN  = 3,
		DeviceBusType_SIM     = 4,
	};

	/*
	  broken out device elements. The bitfields are used to keep
	  the overall value small enough to fit in a float accurately,
	  which makes it possible to transport over the MAVLink
	  parameter protocol without loss of information.
	 */
	struct DeviceStructure {
		enum DeviceBusType bus_type:3;
		uint8_t bus:5;     // which instance of the bus type
		uint8_t address;   // address on the bus (eg. I2C address)
		uint8_t devtype;   // device class specific device type
	};

	union DeviceId {
		struct DeviceStructure devid_s;
		uint32_t devid;
	};

protected:
	const char	*_name;			/**< driver name */
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
	 */
	void		lock() {
		DEVICE_DEBUG("lock");
		do {} while (sem_wait(&_lock) != 0);
	}

	/**
	 * Release the driver lock.
	 */
	void		unlock() {
		DEVICE_DEBUG("unlock");
		sem_post(&_lock);
	}

private:
	sem_t		_lock;

	/** disable copy construction for this and all subclasses */
	Device(const Device &);

	/** disable assignment for this and all subclasses */
	Device &operator = (const Device &);
};

/**
 * Abstract class for any virtual character device
 */
class __EXPORT VDev : public Device
{
public:
	/**
	 * Constructor
	 *
	 * @param name		Driver name
	 * @param devname	Device node name
	 */
	VDev(const char *name, const char *devname);

	/**
	 * Destructor
	 */
	virtual ~VDev();

	virtual int	init();

	/**
	 * Perform a poll setup/teardown operation. Based on NuttX implementation.
	 *
	 * This is handled internally and should not normally be overridden.
	 *
	 * @param filep	Pointer to the internal file structure.
	 * @param fds		Poll descriptor being waited on.
	 * @param setup		True if this is establishing a request, false if
	 *			it is being torn down.
	 * @return		OK on success, or -errno otherwise.
	 */
	virtual int	poll(file_t *filep, px4_pollfd_struct_t *fds, bool setup);

	/**
	 * Test whether the device is currently open.
	 *
	 * This can be used to avoid tearing down a device that is still active.
	 * Note - not virtual, cannot be overridden by a subclass.
	 *
	 * @return              True if the device is currently open.
	 */
	bool            is_open() { return _open_count > 0; }

	/**
	 * Handle an open of the device.
	 *
	 * This function is called for every open of the device. The default
	 * implementation maintains _open_count and always returns OK.
	 *
	 * @param filep		Pointer to the NuttX file structure.
	 * @return		OK if the open is allowed, -errno otherwise.
	 */
	virtual int	open(file_t *filep);

	/**
	 * Handle a close of the device.
	 *
	 * This function is called for every close of the device. The default
	 * implementation maintains _open_count and returns OK as long as it is not zero.
	 *
	 * @param filep		Pointer to the NuttX file structure.
	 * @return		OK if the close was successful, -errno otherwise.
	 */
	virtual int	close(file_t *filep);

	/**
	 * Perform a read from the device.
	 *
	 * The default implementation returns -ENOSYS.
	 *
	 * @param filep		Pointer to the NuttX file structure.
	 * @param buffer	Pointer to the buffer into which data should be placed.
	 * @param buflen	The number of bytes to be read.
	 * @return		The number of bytes read or -errno otherwise.
	 */
	virtual ssize_t	read(file_t *filep, char *buffer, size_t buflen);

	/**
	 * Perform a write to the device.
	 *
	 * The default implementation returns -ENOSYS.
	 *
	 * @param filep		Pointer to the NuttX file structure.
	 * @param buffer	Pointer to the buffer from which data should be read.
	 * @param buflen	The number of bytes to be written.
	 * @return		The number of bytes written or -errno otherwise.
	 */
	virtual ssize_t	write(file_t *filep, const char *buffer, size_t buflen);

	/**
	 * Perform a logical seek operation on the device.
	 *
	 * The default implementation returns -ENOSYS.
	 *
	 * @param filep		Pointer to the NuttX file structure.
	 * @param offset	The new file position relative to whence.
	 * @param whence	SEEK_OFS, SEEK_CUR or SEEK_END.
	 * @return		The previous offset, or -errno otherwise.
	 */
	virtual off_t	seek(file_t *filep, off_t offset, int whence);

	/**
	 * Perform an ioctl operation on the device.
	 *
	 * The default implementation handles DIOC_GETPRIV, and otherwise
	 * returns -ENOTTY. Subclasses should call the default implementation
	 * for any command they do not handle themselves.
	 *
	 * @param filep		Pointer to the NuttX file structure.
	 * @param cmd		The ioctl command value.
	 * @param arg		The ioctl argument value.
	 * @return		OK on success, or -errno otherwise.
	 */
	virtual int	ioctl(file_t *filep, int cmd, unsigned long arg);

	static VDev *getDev(const char *path);
	static void showFiles(void);
	static void showDevices(void);
	static void showTopics(void);
	static const char *devList(unsigned int *next);
	static const char *topicList(unsigned int *next);

	/**
	 * Get the device name.
	 *
	 * @return the file system string of the device handle
	 */
	const char*	get_devname() { return _devname; }

protected:

	int register_driver(const char *name, void *data);
	int unregister_driver(const char *name);

	/**
	 * Check the current state of the device for poll events from the
	 * perspective of the file.
	 *
	 * This function is called by the default poll() implementation when
	 * a poll is set up to determine whether the poll should return immediately.
	 *
	 * The default implementation returns no events.
	 *
	 * @param filep		The file that's interested.
	 * @return		The current set of poll events.
	 */
	virtual pollevent_t poll_state(file_t *filep);

	/**
	 * Report new poll events.
	 *
	 * This function should be called anytime the state of the device changes
	 * in a fashion that might be interesting to a poll waiter.
	 *
	 * @param events	The new event(s) being announced.
	 */
	virtual void	poll_notify(pollevent_t events);

	/**
	 * Internal implementation of poll_notify.
	 *
	 * @param fds		A poll waiter to notify.
	 * @param events	The event(s) to send to the waiter.
	 */
	virtual void	poll_notify_one(px4_pollfd_struct_t *fds, pollevent_t events);

	/**
	 * Notification of the first open.
	 *
	 * This function is called when the device open count transitions from zero
	 * to one.  The driver lock is held for the duration of the call.
	 *
	 * The default implementation returns OK.
	 *
	 * @param filep		Pointer to the NuttX file structure.
	 * @return		OK if the open should proceed, -errno otherwise.
	 */
	virtual int	open_first(file_t *filep);

	/**
	 * Notification of the last close.
	 *
	 * This function is called when the device open count transitions from
	 * one to zero.  The driver lock is held for the duration of the call.
	 *
	 * The default implementation returns OK.
	 *
	 * @param filep		Pointer to the NuttX file structure.
	 * @return		OK if the open should return OK, -errno otherwise.
	 */
	virtual int	close_last(file_t *filep);

        /**
	 * Register a class device name, automatically adding device
	 * class instance suffix if need be.
	 *
	 * @param class_devname   Device class name
	 * @return class_instamce Class instance created, or -errno on failure
	 */
	virtual int register_class_devname(const char *class_devname);

        /**
	 * Register a class device name, automatically adding device
	 * class instance suffix if need be.
	 *
	 * @param class_devname   Device class name
	 * @param class_instance  Device class instance from register_class_devname()
	 * @return		  OK on success, -errno otherwise
	 */
	virtual int unregister_class_devname(const char *class_devname, unsigned class_instance);

	bool		_pub_blocked;		/**< true if publishing should be blocked */

private:
	static const unsigned _max_pollwaiters = 8;

	const char	*_devname;		/**< device node name */
	bool		_registered;		/**< true if device name was registered */
	unsigned	_open_count;		/**< number of successful opens */

	px4_pollfd_struct_t	*_pollset[_max_pollwaiters];

	/**
	 * Store a pollwaiter in a slot where we can find it later.
	 *
	 * Expands the pollset as required.  Must be called with the driver locked.
	 *
	 * @return		OK, or -errno on error.
	 */
	int		store_poll_waiter(px4_pollfd_struct_t *fds);

	/**
	 * Remove a poll waiter.
	 *
	 * @return		OK, or -errno on error.
	 */
	int		remove_poll_waiter(px4_pollfd_struct_t *fds);

	/* do not allow copying this class */
	VDev(const VDev&);
	//VDev operator=(const VDev&);
};

#if 0
/**
 * Abstract class for character device accessed via PIO
 */
class __EXPORT VPIO : public CDev
{
public:
	/**
	 * Constructor
	 *
	 * @param name		Driver name
	 * @param devname	Device node name
	 * @param base		Base address of the device PIO area
	 * @param irq		Interrupt assigned to the device (or zero if none)
	 */
	PIO(const char *name,
	    const char *devname,
	    unsigned long base
	    );
	virtual ~PIO();

	virtual int	init();

protected:

	/**
	 * Read a register
	 *
	 * @param offset	Register offset in bytes from the base address.
	 */
	uint32_t	reg(uint32_t offset) {
		return *(volatile uint32_t *)(_base + offset);
	}

	/**
	 * Write a register
	 *
	 * @param offset	Register offset in bytes from the base address.
	 * @param value	Value to write.
	 */
	void		reg(uint32_t offset, uint32_t value) {
		*(volatile uint32_t *)(_base + offset) = value;
	}

	/**
	 * Modify a register
	 *
	 * Note that there is a risk of a race during the read/modify/write cycle
	 * that must be taken care of by the caller.
	 *
	 * @param offset	Register offset in bytes from the base address.
	 * @param clearbits	Bits to clear in the register
	 * @param setbits	Bits to set in the register
	 */
	void		modify(uint32_t offset, uint32_t clearbits, uint32_t setbits) {
		uint32_t val = reg(offset);
		val &= ~clearbits;
		val |= setbits;
		reg(offset, val);
	}

private:
	unsigned long	_base;
};
#endif

} // namespace device

// class instance for primary driver of each class
enum CLASS_DEVICE {
	CLASS_DEVICE_PRIMARY=0,
	CLASS_DEVICE_SECONDARY=1,
	CLASS_DEVICE_TERTIARY=2
};

