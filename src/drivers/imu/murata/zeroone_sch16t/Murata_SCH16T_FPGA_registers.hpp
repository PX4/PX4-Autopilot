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

#include <cstdint>

namespace Murata_SCH16T_FPGA
{
static constexpr float FIFO_SAMPLE_DT{1e6f / 1475.f};

static constexpr uint16_t EOI = (1 << 1);               // End of Initialization
static constexpr uint16_t EN_SENSOR = (1 << 0);         // Enable RATE and ACC measurement
static constexpr uint16_t DRY_DRV_EN = (1 << 5);        // Enables Data ready function

static constexpr uint16_t FILTER_235HZ = (0b0000101101101);       // 235 Hz  filter
static constexpr uint16_t FILTER_68HZ = (0x0000);       // 68 Hz default filter
static constexpr uint16_t FILTER_30HZ = (0b0000001001001);       // 30 Hz  filter
static constexpr uint16_t FILTER_BYPASS = (0b0000000111111111);     // No filtering

static constexpr uint16_t RATE_300DPS_1518HZ = 0b0001001011011011; // Gyro XYZ range 300 deg/s @ 1518Hz
static constexpr uint16_t RATE_300DPS_3030HZ = 0b0001001010010010; // Gyro XYZ range 300 deg/s @ 3030Hz
static constexpr uint16_t RATE_300DPS_4419HZ = 0b0001001001001001; // Gyro XYZ range 300 deg/s @ 4419Hz

static constexpr uint16_t ACC12_8G_1518HZ = 0b0001001011011011;  // Acc XYZ range 8 G and 1518 update rate
static constexpr uint16_t ACC12_8G_3030HZ = 0b0001001010010010;     // Acc XYZ range 8 G and 3030 update rate
static constexpr uint16_t ACC12_8G_4419HZ = 0b0001001001001001;     // Acc XYZ range 8 G and 4419 update rate

static constexpr uint16_t ACC3_26G = (0b000 << 0);
static constexpr uint16_t SPI_SOFT_RESET = (0b1010);
static constexpr uint32_t POWER_ON_TIME = 250000;
// Data registers
#define RATE_X1         0x01 // 20 bit
#define RATE_Y1         0x02 // 20 bit
#define RATE_Z1         0x03 // 20 bit
#define ACC_X1          0x04 // 20 bit
#define ACC_Y1          0x05 // 20 bit
#define ACC_Z1          0x06 // 20 bit
#define ACC_X3          0x07 // 20 bit
#define ACC_Y3          0x08 // 20 bit
#define ACC_Z3          0x09 // 20 bit
#define RATE_X2         0x0A // 20 bit
#define RATE_Y2         0x0B // 20 bit
#define RATE_Z2         0x0C // 20 bit
#define ACC_X2          0x0D // 20 bit
#define ACC_Y2          0x0E // 20 bit
#define ACC_Z2          0x0F // 20 bit
#define TEMP            0x10 // 16 bit
// Status registers
#define STAT_SUM        0x14 // 16 bit
#define STAT_SUM_SAT    0x15 // 16 bit
#define STAT_COM        0x16 // 16 bit
#define STAT_RATE_COM   0x17 // 16 bit
#define STAT_RATE_X     0x18 // 16 bit
#define STAT_RATE_Y     0x19 // 16 bit
#define STAT_RATE_Z     0x1A // 16 bit
#define STAT_ACC_X      0x1B // 16 bit
#define STAT_ACC_Y      0x1C // 16 bit
#define STAT_ACC_Z      0x1D // 16 bit
// Control registers
#define CTRL_FILT_RATE  0x25 // 9 bit
#define CTRL_FILT_ACC12 0x26 // 9 bit
#define CTRL_FILT_ACC3  0x27 // 9 bit
#define CTRL_RATE       0x28 // 15 bit
#define CTRL_ACC12      0x29 // 15 bit
#define CTRL_ACC3       0x2A // 3 bit
#define CTRL_USER_IF    0x33 // 16 bit
#define CTRL_ST         0x34 // 13 bit
#define CTRL_MODE       0x35 // 4 bit
#define CTRL_RESET      0x36 // 4 bit
// Misc registers
#define ASIC_ID         0x3B // 12 bit
#define COMP_ID         0x3C // 16 bit
#define SN_ID1          0x3D // 16 bit
#define SN_ID2          0x3E // 16 bit
#define SN_ID3          0x3F // 16 bit

#define T_STALL_US   20U

#define SPI32BITCONVERT2_20BIT(x) (((int32_t)(((x)<<12)& 0xfffff000UL))>>12)


#define CTRL_Shift_Mode (0)
#define CTRL_Shift_FifoRst (1)
#define CTRL_Shift_FifoEnable (2)
#define CTRL_Shift_CmdNumSub1 (8)
#define CTRL_Shift_BaudSub1 (16)

/*
Addr_R32_Verison
check the version of FPGA
*/
#define Addr_R32_Verison (0x0000)
/*
Addr_RW16_TestReg
Test register, used for read-write testing to determine whether the FPGA is normal
*/
#define Addr_RW16_TestReg (0x0001)
/*
Addr_W32_Sch16tCtrl
bit[0]: '0' - Direct Mode, '1' - FPGA Read-Write Mode
bit[1]: '1' - Reset FIFO
bit[2]: '1' - Enable FPGA to Read Sensor data
bit[15:8]: Number of FPGA Sensor Reading Instructions - 1
bit[23:16]: SPI Clock Cycle + 1, Unit: 16.667ns. Example: 7 - Clock Cycle = (7 + 1) × 16.667 ≈ 133ns
*/
#define Addr_W32_Sch16tCtrl (0x0010)
/*
Addr_RW8_DirectRam8
Used for SPI data reading and writing in direct mode
*/
#define Addr_RW8_DirectRam8 (0x0011)
/*
Addr_RW16_DirectSpiCtrl
Write Function:
bit[3:0]: Function bits. 1 - Reset, 2 - Start Transmission
bit[15:8]: Number of transmitted bytes minus 1
Read Function:
bit[0]: SPI direct transmission busy signal. 1 - Busy, 0 - Idle
*/
#define Addr_RW16_DirectSpiCtrl (0x0012)
/*
Addr_W8_SensorReadCmdRam8
Sensor Reading Instruction RAM
Each instruction is 64 bits, occupying 8 bytes, including a 48-bit instruction and a 16-bit instruction definition.
*/
#define Addr_W8_SensorReadCmdRam8 (0x0013)
/*
Addr_W8_SensorValueFifo8
Sensor data stroge in fifo
*/
#define Addr_W8_SensorValueFifo8 (0x0014)
/*
Addr_RW8_SensorFifoCtrl
Read:
bit[7:0]: Number of packets contained in the FIFO
bit[8]: RDY signal abnormal flag
Write:
bit[0]: After reading one packet, write '1' to notify the FPGA to load the next data packet
*/
#define Addr_RW8_SensorFifoCtrl (0x0015)

#define FuncBit_Write (0 << 7)
#define FuncBit_Read (1 << 7)
#define FuncBit_Reg (0 << 5)
#define FuncBit_Ram (1 << 5)
#define FuncBit_Fifo (2 << 5)
#define FuncBit_Bit8 (0 << 3)
#define FuncBit_Bit16 (1 << 3)
#define FuncBit_Bit32 (2 << 3)
} // namespace Murata_SCH16T
