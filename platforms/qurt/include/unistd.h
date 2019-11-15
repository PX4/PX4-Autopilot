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

/**
 * @file
 * The declarations in this file are released to DSPAL users and are used to
 * make file I/O call's with a device path.
 */

#include "dspal_types.h"
#include <sys/cdefs.h>

#define F_OK 0

#define _SC_CLK_TCK 2

__BEGIN_DECLS

typedef unsigned long useconds_t;

/**
 * @brief
 * Please refer to the POSIX standard for details.
 */
int usleep(useconds_t usec);

/**
 * Close access to the particular port device or bus/peripheral device or file
 * This also disables the interrupt on that device if it has been configured so.
 * @param fd
 * File descriptor returned from the open function.
 * @return
 * - 0: The bus/port device was successfully closed.
 * - TODO: List error codes for all bus/port types.
 */
int close(int fd);

/**
 * Requests that data be read from the bus/port device or file associated with
 * the fd parameter.  In the case of an I2C or SPI bus peripheral the ID of register
 * to be read must have been previously written.  See the write function
 * below for additional information.
 * @par Reading from SPI/I2C
 * An IOCTL function performing a combined read/write operation for I2C and
 * SPI bus peripherals is defined in the def_fs_lib_{bus/port type}.h header file.
 *
 * @par Reading from GPIO
 * This reads the current value, i.e. HIGH or LOW, from the GPIO device associated
 * with fd.
 *
 * @param fd
 * File descriptor returned from the open function.
 * @param buffer
 * Pointer to a character array that will be used to store the accumulated data read
 * from the bus/port device.
 * @param count
 * The length in bytes of the buffer referenced in the buffer parameter.
 * In the case of GPIO, count is treated as DON'T CARE.
 * @return
 * - number of bytes read from fd
 * - negative error code on error
 * - TODO: List error codes for all bus/port types.
 */
size_t read(int fd, void *buf, size_t count);

/**
 * Requests that data be written to the bus/port device or file associated with the
 * fd parameter.
 * @par Buffer Format for SPI and I2C Bus Devices:
 * - SPI and I2C Devices:
 * The first byte (or two bytes when addressing 16-bit peripheral devices) of
 * this buffer is the register ID on the peripheral device to be written.
 *
 * @par write to GPIO
 * Write the value of the first byte of buffer to GPIO device associated with fd.
 *
 * @param fd
 * File descriptor returned from the open function.
 * @param buffer
 * Pointer to a character array containing the data to be written to the specified
 * bus/port device.  For I2C and SPI devices the device ID of the peripheral to
 * be written to is specified using the an IOCTL code.  See the dev_fs_lib_{bus-type}.h
 * files for more information.  Also, see the note above about how to specify the
 * ID of the register to be written.
 * @param count
 * number of bytes in buffer to be written to fd
 * In the case of GPIO, count is treated as DON'T CARE.
 * @return
 * - number of bytes written to fd
 * - negative error code on error
 */
ssize_t write(int fd, const void *buf, size_t count);

/**
 * synchronize a file's in-core state with storage device
 * NOTE: this is only applicable to file operation. Calling fsync() on bus
 * device has no effect
 *
 * @param fd
 * File descriptor returned from the open function.
 *
 * @return
 * -  0: The fsync operation was successfully processed.
 * - -1: on error
 * TODO: List error codes for all bus/port types.
 */
int fsync(int fd);

/**
 * @brief
 * Only supports getting _SC_CLK_TCK
 */
long sysconf(int name);

unsigned int sleep(unsigned int seconds);
__END_DECLS
