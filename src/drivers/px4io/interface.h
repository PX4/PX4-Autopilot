/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file interface.h
 *
 * PX4IO interface classes.
 */

#include <nuttx/config.h>

#include <stdint.h>

class PX4IO_interface
{
public:
	/**
	 * Check that the interface initialised OK.
	 *
	 * Does not check that communication has been established.
	 */
	virtual bool	ok() = 0;

	/**
	 * Set PX4IO registers.
	 *
	 * @param page		The register page to write
	 * @param offset	Offset of the first register to write
	 * @param values	Pointer to values to write
	 * @param num_values	The number of values to write
	 * @return		Zero on success.
	 */
	virtual int	set_reg(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values) = 0;

	/**
	 * Get PX4IO registers.
	 *
	 * @param page		The register page to read
	 * @param offset	Offset of the first register to read
	 * @param values	Pointer to store values read
	 * @param num_values	The number of values to read
	 * @return		Zero on success.
	 */	
	virtual int	get_reg(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values) = 0;
};

extern PX4IO_interface	*io_i2c_interface(int bus, uint8_t address);
extern PX4IO_interface	*io_serial_interface(int port);
