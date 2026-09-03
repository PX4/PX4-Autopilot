/**
 * @file ST_ASM330LHH_Registers.hpp
 *
 * ST ASM330LHH registers.
 *
 * Datasheet DS12232 (ASM330LHH), application note AN5296.
 * The ASM330LHHX / ASM330LHHXG1 share this WHO_AM_I and an identical
 * accelerometer / gyroscope / FIFO register map; their extra blocks (MLC,
 * FSM, sensor hub) are not used here.
 *
 * Copyright (c) 2026, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the STMicroelectronics nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

#pragma once

#include <cstddef>
#include <cstdint>

namespace ST_ASM330LHH
{

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

// SPI mode 3, max 10 MHz per datasheet; 8 MHz leaves margin on long harnesses
static constexpr uint32_t SPI_SPEED = 8 * 1000 * 1000;

static constexpr uint8_t DIR_READ = 0x80;

// WHO_AM_I is 0x6B on the ASM330LHH and ASM330LHHX. The LSM6DSR family reports the
// same value; probe() therefore identifies "an ASM330/LSM6DSR-class device", which is
// all this driver needs since the accel/gyro/FIFO register maps are identical.
static constexpr uint8_t WHO_AM_I_ID = 0x6B;

// Both sensors run at the 3.33 kHz ODR. 6.66 kHz is available but doubles SPI load for
// no benefit: PX4 filters below 1 kHz and the 32-period FIFO drain cap would be hit at
// low IMU_GYRO_RATEMAX values.
static constexpr uint32_t GYRO_ODR  = 3330; // Hz
static constexpr uint32_t ACCEL_ODR = 3330; // Hz

enum class Register : uint8_t {
	FIFO_CTRL1        = 0x07,
	FIFO_CTRL2        = 0x08,
	FIFO_CTRL3        = 0x09,
	FIFO_CTRL4        = 0x0A,
	COUNTER_BDR_REG1  = 0x0B,

	INT1_CTRL         = 0x0D,

	WHO_AM_I          = 0x0F,

	CTRL1_XL          = 0x10,
	CTRL2_G           = 0x11,
	CTRL3_C           = 0x12,
	CTRL4_C           = 0x13,
	CTRL6_C           = 0x15,
	CTRL7_G           = 0x16,
	CTRL8_XL          = 0x17,
	CTRL9_XL          = 0x18,

	OUT_TEMP_L        = 0x20,
	OUT_TEMP_H        = 0x21,

	FIFO_STATUS1      = 0x3A,
	FIFO_STATUS2      = 0x3B,

	FIFO_DATA_OUT_TAG = 0x78,
};

// ODR_XL[3:0] / ODR_G[3:0] occupy bits [7:4] of CTRL1_XL / CTRL2_G
enum ODR_BIT : uint8_t {
	ODR_3330HZ = Bit7 | Bit4, // 1001b -> 3.33 kHz (high-performance mode)
};

// BDR_XL[3:0] = FIFO_CTRL3[3:0], BDR_GY[3:0] = FIFO_CTRL3[7:4]
static constexpr uint8_t BDR_3330HZ = 0x09; // 1001b -> 3.33 kHz

enum CTRL1_XL_BIT : uint8_t {
	// FS_XL[1:0] = bits [3:2]; 01b selects +/-16 g on this family
	FS_XL_16G  = Bit2,
	LPF2_XL_EN = Bit1,
};

enum CTRL2_G_BIT : uint8_t {
	// FS_G[1:0] = bits [3:2]; 11b selects +/-2000 dps.
	// FS_4000 (Bit0) would give +/-4000 dps at 140 mdps/LSB and requires FS_G = 00b.
	FS_G_2000DPS = Bit3 | Bit2,
	FS_G_125DPS  = Bit1,
	FS_G_4000DPS = Bit0,
};

enum CTRL3_C_BIT : uint8_t {
	BOOT      = Bit7,
	BDU       = Bit6,
	H_LACTIVE = Bit5,
	PP_OD     = Bit4,
	SIM       = Bit3,
	IF_INC    = Bit2,
	SW_RESET  = Bit0,
};

enum CTRL4_C_BIT : uint8_t {
	SLEEP_G      = Bit6,
	INT2_ON_INT1 = Bit5,
	DRDY_MASK    = Bit3,
	I2C_DISABLE  = Bit2,
	LPF1_SEL_G   = Bit1,
};

enum CTRL6_C_BIT : uint8_t {
	// 0 = accelerometer high-performance mode (the reset default, asserted explicitly)
	XL_HM_MODE = Bit4,
};

enum CTRL7_G_BIT : uint8_t {
	// 0 = gyroscope high-performance mode (the reset default, asserted explicitly)
	G_HM_MODE = Bit7,
	HP_EN_G   = Bit6,
};

enum CTRL8_XL_BIT : uint8_t {
	// HPCF_XL[2:0] = bits [7:5]; 001b selects an accelerometer LPF2 cutoff of ODR/10
	LPF2_BW_ODR_DIV_10 = Bit5,
	HP_SLOPE_XL_EN     = Bit2, // must stay 0 so the filter is low-pass, not high-pass
};

enum CTRL9_XL_BIT : uint8_t {
	// ST recommends setting DEVICE_CONF during configuration on this family
	DEVICE_CONF = Bit1,
	DEN_XL_EN   = Bit3,
};

enum COUNTER_BDR_REG1_BIT : uint8_t {
	DATAREADY_PULSED = Bit7,
};

enum INT1_CTRL_BIT : uint8_t {
	INT1_FIFO_TH = Bit3,
};

enum FIFO_CTRL2_BIT : uint8_t {
	STOP_ON_WTM = Bit7,
	WTM8        = Bit0, // WTM[8]; the driver never exceeds 8-bit watermarks so this stays 0
};

enum FIFO_CTRL3_BIT : uint8_t {
	BDR_XL_3330HZ = BDR_3330HZ,
	BDR_GY_3330HZ = BDR_3330HZ << 4,
};

enum FIFO_CTRL4_BIT : uint8_t {
	FIFO_MODE_BYPASS     = 0x00,
	FIFO_MODE_CONTINUOUS = Bit2 | Bit1, // 110b
};

enum FIFO_STATUS2_BIT : uint8_t {
	DIFF_FIFO_8       = Bit0, // DIFF_FIFO[8]
	DIFF_FIFO_9       = Bit1, // DIFF_FIFO[9]
	FIFO_OVR_LATCHED  = Bit3,
	COUNTER_BDR_IA    = Bit4,
	FIFO_FULL_IA      = Bit5,
	FIFO_OVR_IA       = Bit6,
	FIFO_WTM_IA       = Bit7,
};

// TAG_SENSOR[4:0] occupies bits [7:3] of the FIFO tag byte
enum class FifoTag : uint8_t {
	GYRO_NC     = 0x01,
	ACCEL_NC    = 0x02,
	TEMPERATURE = 0x03,
	TIMESTAMP   = 0x04,
};

namespace FIFO
{
// FIFO word: 1-byte tag + 6-byte data = 7 bytes
static constexpr size_t WORD_SIZE = 7;
// Words batched per sample period: gyro + accel
static constexpr size_t MAX_WORDS_PER_PERIOD = 2;
// Max sample periods to drain per poll (bounded by sensor_{accel,gyro}_fifo capacity)
static constexpr size_t MAX_DRAIN_SAMPLES = 32;
// DIFF_FIFO is a 10-bit counter; the buffer itself is 3 KB, i.e. ~438 words
static constexpr size_t DEPTH = 438;
}

} // namespace ST_ASM330LHH
