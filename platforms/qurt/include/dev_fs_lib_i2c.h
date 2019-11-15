/****************************************************************************
 * Copyright (c) 2015 Mark Charlebois. All rights reserved.
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

#include <stdint.h>

/**
 * @file
 * The declarations in this file are released to DSPAL users and are used to
 * make file I/O call's for I2C device access.  Many of the data structures
 * are used in the parameter to a particular IOCTL function.
 *
 * @warning
 * The functions specified in this header file have received limited testing
 * and may not function properly.  A subsequent update to this release will include
 * fixes to bugs identified during testing, still ongoing.
 *
 * @par
 * Sample source files are referenced to demonstrate how a particular IOCTL
 * or data structure is used in the context of the POSIX standard file I/O functions
 * (open/close/read/write/ioctl).
 *
 * @par Writing Data to an I2C Slave Device
 * The POSIX write function can be used to write data to a specified register on
 * the slave device.  The first one or two bytes of data in the buffer parameter specifies
 * the address and to be written.  One byte is used for 8 bit register addressing, and two bytes
 * for 16 bit addressing.  The bytes which follow represent the data to written to the
 * specified register, if applicable.  No data would be required if the write function was
 * being called to specify the number of the register for a subsequent read operation.
 *
 * @par
 * See the next section for an alternative to calling the POSIX write and read functions
 * in sequence when reading from a slave device register.
 *
 * @par Reading Data from an I2C Slave Device
 * The most efficient method of reading data from a specified register on the
 * slave device is to use the I2C_IOCTL_RDWR IOCTL.  This provides an alternative
 * to calling the write function for the register number, followed by the read function.
 *
 * @par
 * Sample source code for read/write data to an I2C slave device is included below:
 * @include i2c_test_imp.c
 */

/**
 * @brief
 * The I2C device path uses the following format:
 * /dev/iic-{bus number}
 * Bus numbers start at 1 and may go to up to the max number of BLSPs supported
 * by the SoC.  Using /dev/i2c-{number} is deprecated.
 */
#define DEV_FS_I2C_DEVICE_TYPE_STRING  "/dev/iic-"
#define DEV_FS_I2C_SSC_DEVICE_TYPE_STRING  "/dev/iic_ssc-"

/**
 * @brief
 * ioctl codes used to extend the functionality of the standard read/write file
 * semantics for the i2c bus.
 */
enum DSPAL_I2C_IOCTLS {
	I2C_IOCTL_INVALID = -1, /**< invalid IOCTL code, used to return an error */
	I2C_IOCTL_SLAVE,        /**< used to select the i2c peripheral on the specified i2c bus */
	I2C_IOCTL_RDWR,         /**< used to initiate a write/read batch transfer */
	I2C_IOCTL_CONFIG,       /**< used to select the i2c peripheral on the
				* specified i2c bus. This has the same effect as
				* I2C_IOCTL_SLAVE and will be deprecated in future
				* release. It is recommended to use I2C_IOCTL_SLAVE
				* to select the i2c peripheral.
				*/
	I2C_IOCTL_WRITE_REG,    /**< used to write a register */
	I2C_IOCTL_READ,		/**< used to read data from i2c buffer. This has
				* the same effect as posix read() call.
				* NOTE: Both mechanisms assumes that a write call
				* has been made to write the starting register
				* address from which to read. The most efficient
				* method of reading data from a specified register
				* on the slave device is to use the I2C_IOCTL_RDWR
				* IOCTL.
				*/
	I2C_IOCTL_MAX_NUM,      /**< number of valid IOCTL codes defined for the I2C bus */
};

/**
 * @brief
 * Additional error codes added to the standard POSIX errno list.
 */
#define EDRIVER 65536 /**< Indicates an error from the underlying driver called by DSPAL */

/**
 * @brief
 * Structure used in the ioctl: I2C_IOCTL_SLAVE
 *
 * @par
 * This structure is used after calling the open function to select the slave device on
 * the I2C bus that will be the target of subsequent read/write functions.
 */
struct dspal_i2c_ioctl_slave_config {
	uint32_t flags;  			/**< reserved for future use */
	uint32_t slave_address;  		/**< the address of the slave device on the i2c bus */
	uint32_t bus_frequency_in_khz; 		/**< the bus frequency used to communication with the slave device in KHz */
	uint32_t byte_transer_timeout_in_usecs;	/**< the period of time to wait for a response from the slave device in usecs */
};

/**
 * @brief
 * Structure used in the ioctl: I2C_IOCTL_RDWR
 *
 * @par
 * This structure is used to initiate a write/read I2C bus transaction.  This consists
 * of the number of the register to be read as the write transaction, followed by one or more bytes of
 * data from the associated register in the read transaction.
 *
 * @par
 * Sample code demonstrating the use of this function is provided below:
 * @include i2c_test_imp.c
 */
struct dspal_i2c_ioctl_combined_write_read {
	uint32_t flags;  	/**< reserved for future use */
	uint8_t *write_buf;  	/**< the address of the buffer containing the value (register) to be read on the slave device */
	uint32_t write_buf_len;	/**< the length of the write_buf buffer */
	uint8_t *read_buf; 	/**< the address of the buffer containing the bytes read from the specified register on the slave device */
	uint32_t read_buf_len; 	/**< the length of the read_buf buffer */
};

struct dspal_i2c_ioctl_read {
	uint32_t flags;  	/**< reserved for future use */
	uint8_t *read_buf; 	/**< the address of the buffer containing the bytes read from the specified register on the slave device */
	uint32_t read_buf_len; 	/**< the length of the read_buf buffer */
};

