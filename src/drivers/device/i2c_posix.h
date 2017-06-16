/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file i2c.h
 *
 * Base class for devices connected via I2C.
 */

#ifndef _DEVICE_I2C_H
#define _DEVICE_I2C_H

#include "vdev.h"

#include <px4_i2c.h>
#ifdef __PX4_LINUX
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#endif
#include <string>

namespace device __EXPORT
{

/**
 * Abstract class for character device on I2C
 */
class __EXPORT I2C : public VDev
{

public:

	/**
	 * Get the address
	 */
	int16_t		get_address() const { return _address; }

protected:
	/**
	 * The number of times a read or write operation will be retried on
	 * error.
	 */
	unsigned		_retries;

	/**
	 * The I2C bus number the device is attached to.
	 */
	int			_bus;

	/**
	 * @ Constructor
	 *
	 * @param name		Driver name
	 * @param devname	Device node name
	 * @param bus		I2C bus on which the device lives
	 * @param address	I2C bus address, or zero if set_address will be used
	 */
	I2C(const char *name,
	    const char *devname,
	    int bus,
	    uint16_t address);
	virtual ~I2C();

	virtual int	init();

	virtual ssize_t	read(file_t *handlep, char *buffer, size_t buflen);
	virtual ssize_t	write(file_t *handlep, const char *buffer, size_t buflen);
	virtual int	ioctl(file_t *handlep, int cmd, unsigned long arg);

	/**
	 * Perform an I2C transaction to the device.
	 *
	 * At least one of send_len and recv_len must be non-zero.
	 *
	 * @param send		Pointer to bytes to send.
	 * @param send_len	Number of bytes to send.
	 * @param recv		Pointer to buffer for bytes received.
	 * @param recv_len	Number of bytes to receive.
	 * @return		OK if the transfer was successful, -errno
	 *			otherwise.
	 */
	int		transfer(const uint8_t *send, unsigned send_len,
				 uint8_t *recv, unsigned recv_len);

	/**
	 * Perform a multi-part I2C transaction to the device.
	 *
	 * @param msgv		An I2C message vector.
	 * @param msgs		The number of entries in the message vector.
	 * @return		OK if the transfer was successful, -errno
	 *			otherwise.
	 */
	int		transfer(struct i2c_msg *msgv, unsigned msgs);

private:
	uint16_t		_address;
	int 			_fd;
	std::string		_dname;

	I2C(const device::I2C &);
	I2C operator=(const device::I2C &);
};

} // namespace device

#endif /* _DEVICE_I2C_H */
