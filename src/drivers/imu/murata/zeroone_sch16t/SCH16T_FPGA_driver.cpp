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

#include "ZEROONE_SCH16T.hpp"
#include "SCH16T_FPGA_driver.hpp"
#if defined(DEBUG_BUILD)
#include <cstdlib>
#endif

namespace Murata_SCH16T_FPGA
{

SCH16T_FPGA_driver::SCH16T_FPGA_driver(FpgaSpiInterface *spi_interface) :
	_spi_interface(spi_interface)
{
}

void SCH16T_FPGA_driver::fpga_read_fifo8(uint16_t reg, uint8_t *value, uint8_t size)
{
	FpgaFrame frame{};

	if (value == nullptr) {
		return;
	}

	if (size > sizeof(frame.fifo.data) - 1) {
		return;
	}

	fill_fpga_frame(frame, FuncBit_Read | FuncBit_Fifo | FuncBit_Bit8, reg, 0);
	frame.fifo.size = size;
	frame.fifo.data[0] = SEPARATOR_BYTE;//separator byte
	memset(value, DUMMY_BYTE, size);
	memcpy(frame.fifo.data + 1, value, size);
	_spi_interface->transfer(frame.raw, frame.raw, size + 5);
	memcpy(value, frame.fifo.data + 1, size);
}

void SCH16T_FPGA_driver::fpga_write_reg8(uint16_t reg, uint8_t value)
{
	FpgaFrame frame{};

	fill_fpga_frame(frame, FuncBit_Write | FuncBit_Reg | FuncBit_Bit8, reg, 0);
	frame.reg.data[0] = value;
	_spi_interface->transfer(frame.raw, frame.raw, 4);
}

uint8_t SCH16T_FPGA_driver::fpga_read_reg8(uint16_t reg)
{
	FpgaFrame frame{};

	fill_fpga_frame(frame, FuncBit_Read | FuncBit_Reg | FuncBit_Bit8, reg, 0);
	frame.reg.data[0] = SEPARATOR_BYTE;
	frame.reg.data[1] = DUMMY_BYTE;
	_spi_interface->transfer(frame.raw, frame.raw, 5);
	return frame.reg.data[1];
}

void SCH16T_FPGA_driver::fpga_write_reg16(uint16_t reg, uint16_t value)
{
	FpgaFrame frame{};

	fill_fpga_frame(frame, FuncBit_Write | FuncBit_Reg | FuncBit_Bit16, reg, 0);
	frame.reg.data[0] = value & 0xff;
	frame.reg.data[1] = (value >> 8) & 0xff;

	_spi_interface->transfer(frame.raw, frame.raw, 5);
}

uint16_t SCH16T_FPGA_driver::fpga_read_reg16(uint16_t reg)
{
	FpgaFrame frame{};

	fill_fpga_frame(frame, FuncBit_Read | FuncBit_Reg | FuncBit_Bit16, reg, 0);
	frame.reg.data[0] = SEPARATOR_BYTE;
	frame.reg.data[1] = DUMMY_BYTE;
	frame.reg.data[2] = DUMMY_BYTE;
	_spi_interface->transfer(frame.raw, frame.raw, 6);
	return frame.reg.data[1] | (frame.reg.data[2] << 8);
}

void SCH16T_FPGA_driver::fpga_write_ram8(uint16_t reg, uint16_t ram_addr, uint8_t *value, uint16_t size)
{
	FpgaFrame frame{};

	if (size > sizeof(frame.fifo.data) - 1) {
		return;
	}

	if (value == nullptr) {
		return;
	}

	fill_fpga_frame(frame, FuncBit_Write | FuncBit_Ram | FuncBit_Bit8, reg, ram_addr);
	memcpy(frame.ram.data, value, size);
	_spi_interface->transfer(frame.raw, frame.raw, size + 5);
}

void SCH16T_FPGA_driver::fpga_read_ram8(uint16_t reg, uint16_t ram_addr, uint8_t *value, uint16_t size)
{
	FpgaFrame frame{};

	if (size > sizeof(frame.fifo.data) - 1) {
		return;
	}

	if (value == nullptr) {
		return;
	}

	fill_fpga_frame(frame, FuncBit_Read | FuncBit_Ram | FuncBit_Bit8, reg, ram_addr);
	frame.ram.data[0] = SEPARATOR_BYTE;//separator byte
	memset(value, DUMMY_BYTE, size);
	memcpy(frame.ram.data + 1, value, size);
	_spi_interface->transfer(frame.raw, frame.raw, size + 6);
	memcpy(value, frame.ram.data + 1, size);
}

uint8_t SCH16T_FPGA_driver::gen_crc8(uint8_t *data)
{
	uint16_t crc = 0xff;
	uint8_t byte_value;

	for (uint8_t c = 0; c < 6; c++) {
		byte_value = (c == 5) ? 0x00 : data[c];

		for (uint8_t i = 0; i < 8; i++) {
			uint8_t data_bit = (byte_value >> (7 - i)) & 1;
			crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0x2F) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
		}
	}

	return crc;
}

void SCH16T_FPGA_driver::_register_write(uint16_t addr, uint32_t value, uint8_t *buff)
{
	RegCtl reg_ctl{};
	reg_ctl.reg.cmd_reg_addr_l = (addr & 3) << 6;
	reg_ctl.reg.reg_addr_h = (addr >> 2) & 0xff;
	reg_ctl.reg.cmd_reg_addr_l = ((addr & 3) << 6);
	reg_ctl.reg.cmd_reg_addr_l |= Register_Write;
	reg_ctl.reg.cmd_reg_addr_l |= Register_Bit32;
	reg_ctl.reg.data[0] = (value >> 16) & 0xf;
	reg_ctl.reg.data[1] = (value >> 8) & 0xff;
	reg_ctl.reg.data[2] = value & 0xff;
	reg_ctl.reg.crc = gen_crc8(reg_ctl.raw);
	memcpy(buff, reg_ctl.raw, 6);
}

void SCH16T_FPGA_driver::_register_read(uint16_t addr, uint8_t *buff)
{
	RegCtl reg_ctl{};
	reg_ctl.reg.cmd_reg_addr_l = (addr & 3) << 6;
	reg_ctl.reg.reg_addr_h = (addr >> 2) & 0xff;
	reg_ctl.reg.cmd_reg_addr_l = ((addr & 3) << 6);
	reg_ctl.reg.cmd_reg_addr_l |= Register_Bit32;
	reg_ctl.reg.crc = gen_crc8(reg_ctl.raw);
	memcpy(buff, reg_ctl.raw, 6);
}

void SCH16T_FPGA_driver::sch16t_gen_cmd(uint32_t addr, uint8_t *buff, uint8_t read_valid, uint8_t is_sensor, uint8_t offset)
{
	buff[0] = read_valid ? 0x80 : 0x00;
	buff[0] |= is_sensor ? 0x40 : 0x00;
	buff[0] |= (offset & 0xf) << 2;
	buff[0] |= (offset >> 2) & 0x3;
	buff[1] = (offset & 3) << 6;
	_register_read(addr, buff + 2);
}

void SCH16T_FPGA_driver::fpga_write_cmd(uint32_t addr, uint8_t ptr, uint8_t read_valid, uint8_t is_sensor, uint8_t offset)
{
	uint8_t buff[16];
	sch16t_gen_cmd(addr, buff, read_valid, is_sensor, offset);
	fpga_write_ram8(Addr_W8_SensorReadCmdRam8, ptr << 3, buff, 8);
}

void SCH16T_FPGA_driver::fpga_read_config(uint8_t &fifo_cmd_num)
{
	/*the command is to notice the fpga to load next register to read,and the response is the previous register value.*/
	uint8_t ptr = 0;
	fpga_write_cmd(RATE_X2, ptr, 0, 0, 0);
	ptr++;
	fpga_write_cmd(RATE_Y2, ptr, 1, 1, 0);
	ptr++;
	fpga_write_cmd(RATE_Z2, ptr, 1, 1, 1);
	ptr++;
	fpga_write_cmd(ACC_X2, ptr, 1, 1, 2);
	ptr++;
	fpga_write_cmd(ACC_Y2, ptr, 1, 1, 3);
	ptr++;
	fpga_write_cmd(ACC_Z2, ptr, 1, 1, 4);
	ptr++;
	fpga_write_cmd(TEMP, ptr, 1, 1, 5);
	ptr++;
	fpga_write_cmd(STAT_SUM, ptr, 1, 1, 6);
	ptr++;
	fpga_write_cmd(STAT_SUM_SAT, ptr, 1, 0, 0);
	ptr++;
	fpga_write_cmd(STAT_COM, ptr, 1, 0, 1);
	ptr++;
	fpga_write_cmd(STAT_RATE_COM, ptr, 1, 0, 2);
	ptr++;
	fpga_write_cmd(STAT_RATE_X, ptr, 1, 0, 3);
	ptr++;
	fpga_write_cmd(STAT_RATE_Y, ptr, 1, 0, 4);
	ptr++;
	fpga_write_cmd(STAT_RATE_Z, ptr, 1, 0, 5);
	ptr++;
	fpga_write_cmd(STAT_ACC_X, ptr, 1, 0, 6);
	ptr++;
	fpga_write_cmd(STAT_ACC_Y, ptr, 1, 0, 7);
	ptr++;
	fpga_write_cmd(STAT_ACC_Z, ptr, 1, 0, 8);
	ptr++;
	fpga_write_cmd(STAT_ACC_Z, ptr, 1, 0, 9);
	ptr++;
	fifo_cmd_num = ptr;
}

void SCH16T_FPGA_driver::fpga_init(uint8_t &fifo_enable, uint8_t &fifo_cmd_num, uint8_t direct_mode)
{
	fifo_enable = 0;
	sensor_ctrl(1, direct_mode, fifo_enable, fifo_cmd_num, SPI_2Mhz);

	fpga_read_config(fifo_cmd_num);

	fifo_enable = 1;
	direct_mode = 1;
	sensor_ctrl(1, direct_mode, fifo_enable, fifo_cmd_num, SPI_2Mhz);
}

void SCH16T_FPGA_driver::sensor_ctrl(uint8_t fifo_rst, uint8_t direct_mode, uint8_t fifo_enable, uint8_t fifo_cmd_num,
				     uint8_t fifo_baudrate)
{
	uint32_t reg = (direct_mode & 1) << CTRL_Shift_Mode;
	reg |= (fifo_rst & 1) << CTRL_Shift_FifoRst;
	reg |= (fifo_enable & 1) << CTRL_Shift_FifoEnable;
	reg |= ((fifo_cmd_num - 1) & 0xff) << CTRL_Shift_CmdNumSub1;
	reg |= ((fifo_baudrate - 1) & 0xff) << CTRL_Shift_BaudSub1;

	SensorCtl sensor_ctl;

	sensor_ctl.reg.cmd = FuncBit_Write | FuncBit_Reg | FuncBit_Bit32;
	sensor_ctl.reg.reg_addr_l = Addr_W32_Sch16tCtrl & 0xff;
	sensor_ctl.reg.reg_addr_h = (Addr_W32_Sch16tCtrl >> 8) & 0xff;
	sensor_ctl.reg.reg_data[0] = reg & 0xff;
	sensor_ctl.reg.reg_data[1] = (reg >> 8) & 0xff;
	sensor_ctl.reg.reg_data[2] = (reg >> 16) & 0xff;
	sensor_ctl.reg.reg_data[3] = (reg >> 24) & 0xff;


	_spi_interface->transfer(sensor_ctl.raw, sensor_ctl.raw, 7);
}

void SCH16T_FPGA_driver::fill_fpga_frame(FpgaFrame &frame, uint8_t cmd, uint16_t reg_addr, uint16_t ram_addr)
{
	const uint8_t cmd_type = cmd & (FuncBit_Reg | FuncBit_Ram | FuncBit_Fifo);

	switch (cmd_type) {
	case FuncBit_Reg:
		frame.reg.cmd = cmd;
		frame.reg.reg_addr_l = reg_addr & 0xff;
		frame.reg.reg_addr_h = (reg_addr >> 8) & 0xff;
		break;

	case FuncBit_Ram:
		frame.ram.cmd = cmd;
		frame.ram.reg_addr_l = reg_addr & 0xff;
		frame.ram.reg_addr_h = (reg_addr >> 8) & 0xff;
		frame.ram.ram_addr_l = ram_addr & 0xff;
		frame.ram.ram_addr_h = (ram_addr >> 8) & 0xff;
		break;

	case FuncBit_Fifo:
		frame.fifo.cmd = cmd;
		frame.fifo.reg_addr_l = reg_addr & 0xff;
		frame.fifo.reg_addr_h = (reg_addr >> 8) & 0xff;
		break;

	default:
		frame.reg.cmd = cmd;
		frame.reg.reg_addr_l = reg_addr & 0xff;
		frame.reg.reg_addr_h = (reg_addr >> 8) & 0xff;
		break;
	}
}
#if defined(DEBUG_BUILD)
uint8_t SCH16T_FPGA_driver::fpga_test()
{
	uint16_t wreg, rreg = 0;
	wreg = rand() & 0xffff;
	fpga_write_reg16(Addr_RW16_TestReg, wreg);
	rreg = fpga_read_reg16(Addr_RW16_TestReg);

	if (wreg != rreg) {
		return 0;
	}

	return 1;
}
#endif

} // namespace Murata_SCH16T_FPGA
