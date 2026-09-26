/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "Murata_SCH16T_FPGA_registers.hpp"
#include <drivers/device/spi.h>
#include <cstdint>

namespace Murata_SCH16T_FPGA
{
class FpgaSpiInterface
{
public:
	virtual ~FpgaSpiInterface() = default;
	virtual void transfer(uint8_t *send, uint8_t *recv, unsigned int len) = 0;
};

#define SEPARATOR_BYTE (0xff)
#define DUMMY_BYTE (0xff)

#define Register_Bit32 (1 << 3)
#define Register_Write (1 << 5)
class SCH16T_FPGA_driver
{
public:

	SCH16T_FPGA_driver(FpgaSpiInterface *spi_interface);
	~SCH16T_FPGA_driver() = default;

	void fpga_read_fifo8(uint16_t reg, uint8_t *value, uint8_t size);
	void fpga_write_reg8(uint16_t reg, uint8_t value);
	uint8_t fpga_read_reg8(uint16_t reg);
	void fpga_write_reg16(uint16_t reg, uint16_t value);
	uint16_t fpga_read_reg16(uint16_t reg);
	void fpga_write_ram8(uint16_t reg, uint16_t ram_addr, uint8_t *value, uint16_t size);
	void fpga_read_ram8(uint16_t reg, uint16_t ram_addr, uint8_t *value, uint16_t size);
	void fpga_init(uint8_t &fifo_enable, uint8_t &fifo_cmd_num, uint8_t direct_mode);
	void fpga_read_config(uint8_t &fifo_cmd_num);
	void fpga_write_cmd(uint32_t addr, uint8_t ptr, uint8_t read_valid, uint8_t is_sensor, uint8_t offset);
	void sensor_ctrl(uint8_t fifo_rst, uint8_t direct_mode, uint8_t fifo_enable, uint8_t fifo_cmd_num, uint8_t fifo_baudrate);
#if defined(DEBUG_BUILD)
	uint8_t fpga_test();
#endif

	static uint8_t gen_crc8(uint8_t *data);

private:

	typedef union {
		uint8_t raw[32];

		struct __attribute__((packed)) {
			uint8_t  cmd;
			uint8_t  reg_addr_l;
			uint8_t  reg_addr_h;
			uint8_t  data[29];
		} reg;

		struct __attribute__((packed)) {
			uint8_t  cmd;
			uint8_t  reg_addr_l;
			uint8_t  reg_addr_h;
			uint8_t  ram_addr_l;
			uint8_t  ram_addr_h;
			uint8_t  data[27];
		} ram;

		struct __attribute__((packed)) {
			uint8_t  cmd;
			uint8_t  reg_addr_l;
			uint8_t  reg_addr_h;
			uint8_t  size;
			uint8_t  data[28];
		} fifo;
	} FpgaFrame;

	typedef union {
		uint8_t raw[7];

		struct __attribute__((packed)) {
			uint8_t  cmd;
			uint8_t  reg_addr_l;
			uint8_t  reg_addr_h;
			uint8_t  reg_data[4];
		} reg;

	} SensorCtl;

	typedef union {
		uint8_t raw[6];
		struct __attribute__((packed)) {
			uint8_t  reg_addr_h;
			uint8_t  cmd_reg_addr_l;
			uint8_t  data[3];
			uint8_t  crc;
		} reg;

	} RegCtl;

	FpgaSpiInterface *_spi_interface{nullptr};

	void _register_write(uint16_t addr, uint32_t value, uint8_t *buff);
	void _register_read(uint16_t addr, uint8_t *buff);
	void sch16t_gen_cmd(uint32_t addr, uint8_t *buff, uint8_t read_valid, uint8_t is_sensor, uint8_t offset);
	void fill_fpga_frame(FpgaFrame &frame, uint8_t cmd, uint16_t reg_addr, uint16_t ram_addr);
};

} // namespace Murata_SCH16T_FPGA
