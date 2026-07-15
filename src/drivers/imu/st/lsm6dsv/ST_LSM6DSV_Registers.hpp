/****************************************************************************
 *
 *   Copyright (c) 2024-2026 PX4 Development Team. All rights reserved.
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
 * @file ST_LSM6DSV_Registers.hpp
 *
 * ST LSM6DSV registers.
 *
 */

#pragma once

#include <cstddef>
#include <cstdint>

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace ST_LSM6DSV
{

static constexpr uint32_t SPI_SPEED = 8 * 1000 * 1000; // 8 MHz SPI data clock

static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHO_AM_I_ID       = 0x70; // LSM6DSV16X and LSM6DSV32X (same ID)
static constexpr uint8_t WHO_AM_I_DSK320X  = 0x75; // LSM6DSK320X (unique ID)
// The LSM6DSV80X and LSM6DSV320X share this ID and have an identical register map; they differ
// only in the high-g full-scale table and cannot be distinguished over SPI (no part-ID register;
// only the MIPI I3C Provisioned ID differs, which is not SPI-accessible). The variant is therefore
// selected explicitly at driver start (-T 80 | -T 320).
static constexpr uint8_t WHO_AM_I_HIGHG    = 0x73; // LSM6DSV80X / LSM6DSV320X (shared ID)

// Default-variant (16X / 32X / DSK320X) ODR: HAODR_SEL=01, code 0x0A = 2000 Hz
static constexpr uint32_t GYRO_ODR  = 2000;
static constexpr uint32_t ACCEL_ODR = 2000;

// HAODR mode-1 (HAODR_SEL=01) ODR code (written to CTRL1/CTRL2 [3:0] and FIFO BDR [3:0])
static constexpr uint8_t HAODR_MODE1_ODR_2000HZ = 0x0A;

// High-g variants (LSM6DSV80X / LSM6DSV320X) ODR: HAODR_SEL=00, code 0x0C = 7680 Hz
// (device max; matches the high-g channel ODR set so all channels co-batch at one rate)
static constexpr uint32_t GYRO_ODR_HIGHG  = 7680;
static constexpr uint32_t ACCEL_ODR_HIGHG = 7680;
static constexpr uint8_t HAODR_SEL0_ODR_7680HZ = 0x0C;

enum class Register : uint8_t {
	IF_CFG           = 0x03, // Interrupt polarity and output mode

	FIFO_CTRL1       = 0x07,
	FIFO_CTRL2       = 0x08,
	FIFO_CTRL3       = 0x09,
	FIFO_CTRL4       = 0x0A,

	COUNTER_BDR_REG1 = 0x0B, // XL_HG_BATCH_EN (high-g FIFO batching)

	INT1_CTRL        = 0x0D, // INT1 pin control

	WHO_AM_I         = 0x0F,

	CTRL1            = 0x10, // Accel ODR + FS
	CTRL2            = 0x11, // Gyro  ODR + FS
	CTRL3            = 0x12, // BDU, IF_INC, SW_RESET
	CTRL4            = 0x13,
	CTRL6            = 0x15, // Gyro FS
	CTRL8            = 0x17, // Accel FS + LPF2 BW
	CTRL9            = 0x18, // LPF2 enable

	FIFO_STATUS1     = 0x1B,
	FIFO_STATUS2     = 0x1C,

	STATUS_REG       = 0x1E,

	OUT_TEMP_L       = 0x20,
	OUT_TEMP_H       = 0x21,

	OUTX_L_G         = 0x22,

	OUTX_L_A         = 0x28,

	CTRL1_XL_HG      = 0x4E, // High-g accel: REGOUT_EN + ODR_XL_HG + FS_XL_HG

	FIFO_DATA_OUT_TAG = 0x78,
	FIFO_DATA_OUT_X_L = 0x79,

	HAODR_CFG        = 0x62,
};

// CTRL1 — Accelerometer control (ODR + operating mode + HAODR flag)
enum CTRL1_BIT : uint8_t {
	// ODR_XL [3:0] — set via HAODR mode-1 code 0x0A for 2000 Hz
	// OP_MODE_XL [2:0] in bits [6:4]: 0b000 = high-performance mode
	CTRL1_MODE_HAODR = Bit4, // bit 4 must be set for HAODR selection
};

// CTRL2 — Gyroscope control (ODR + operating mode + HAODR flag)
enum CTRL2_BIT : uint8_t {
	CTRL2_MODE_HAODR = Bit4,
};

// IF_CFG (0x03) — Interrupt polarity and output mode
enum IF_CFG_BIT : uint8_t {
	H_LACTIVE = Bit4, // Interrupt active-low
	PP_OD     = Bit3, // Push-pull (0) / Open-drain (1)
};

// INT1_CTRL (0x0D) — INT1 pin control
enum INT1_CTRL_BIT : uint8_t {
	INT1_FIFO_TH = Bit3, // FIFO threshold interrupt on INT1
};

// CTRL3
enum CTRL3_BIT : uint8_t {
	BDU       = Bit6, // Block Data Update
	IF_INC    = Bit2, // Register address auto-increment
	SW_RESET  = Bit0, // Software reset
};

// CTRL4
enum CTRL4_BIT : uint8_t {
	INT2_on_INT1 = Bit4, // Route INT2 signals to INT1
	DRDY_PULSED  = Bit1, // Pulsed interrupt mode (~65 µs)
};

// CTRL6 — Gyroscope full-scale
enum CTRL6_BIT : uint8_t {
	// FS_G [2:0]
	FS_G_2000DPS         = 0x04, // ±2000 dps (16X / 32X)
	FS_G_2000DPS_DSK320X = 0x0C, // ±2000 dps with bit3=1 for DSK320X (0x04 | Bit3)
	FS_G_4000DPS_HIGHG   = 0x0D, // ±4000 dps for 80X/320X (FS_G=101 | bit3); CTRL6 bit3 must be 1
};

// CTRL8 — Accelerometer full-scale + LPF2 bandwidth
enum CTRL8_BIT : uint8_t {
	// bit2 is hardware-reserved and differs by variant:
	//   16X / DSK320X / 80X = 0 (value OR'd with FS_XL)
	//   32X = 1 (must be preserved)
	// FS_XL [1:0] in bits [1:0]
	FS_XL_16G          = 0x03, // ±16 g for 16X / DSK320X / 80X (bit2=0)
	FS_XL_16G_DSV32X   = 0x06, // ±16 g for 32X (bit2=1, FS_XL=10)

	// HP_LPF2_XL_BW [2:0] in bits [7:5] — when LPF2 enabled via CTRL9
	LPF2_BW_ODR_DIV_10 = Bit5, // 0x20 → ODR/10
};

// CTRL9
enum CTRL9_BIT : uint8_t {
	LPF2_XL_EN = Bit3, // Enable accelerometer LPF2
};

// STATUS_REG
enum STATUS_REG_BIT : uint8_t {
	XLDA = Bit0, // Accelerometer new data available
	GDA  = Bit1, // Gyroscope new data available
	TDA  = Bit2, // Temperature new data available
};

// FIFO_CTRL3 — Batch Data Rate for accel and gyro
enum FIFO_CTRL3_BIT : uint8_t {
	// BDR_GY [3:0] in bits [7:4], BDR_XL [3:0] in bits [3:0]
	// Default variants: HAODR mode-1 code 0x0A (2000 Hz)
	BDR_XL_HAODR = HAODR_MODE1_ODR_2000HZ,
	BDR_GY_HAODR = HAODR_MODE1_ODR_2000HZ << 4,
	// LSM6DSV80X: code 0x0C (7680 Hz)
	BDR_XL_7680  = HAODR_SEL0_ODR_7680HZ,
	BDR_GY_7680  = HAODR_SEL0_ODR_7680HZ << 4,
};

// COUNTER_BDR_REG1 (0x0B)
enum COUNTER_BDR_REG1_BIT : uint8_t {
	XL_HG_BATCH_EN = Bit3, // Batch high-g accelerometer data in FIFO
};

// CTRL1_XL_HG (0x4E) — High-g accelerometer ODR + full-scale
enum CTRL1_XL_HG_BIT : uint8_t {
	XL_HG_REGOUT_EN = Bit7, // Enable high-g output registers (not needed for FIFO path)
	// ODR_XL_HG [5:3]: 111 = 7.68 kHz
	ODR_XL_HG_7680  = 0x38,
	// FS_XL_HG [2:0] — value/range mapping is variant-specific (see HighGFullScale tables)
};

// One high-g full-scale escalation step. The driver escalates through a variant-specific,
// ascending list of these on high-g clipping and de-escalates after a quiet period.
// scale_mg_per_lsb is taken verbatim from the datasheet (it is NOT range_g / 2^15: e.g. the
// LSM6DSV80X ±80 g range is 3.904 mg/LSB, not 2.44).
struct HighGFullScale {
	uint8_t  fs_code;         // FS_XL_HG [2:0] register code
	uint16_t range_g;         // ± full scale [g]
	float    scale_mg_per_lsb; // sensitivity [mg/LSB] (datasheet)
};

// FIFO_CTRL4 — FIFO mode
enum FIFO_CTRL4_BIT : uint8_t {
	FIFO_MODE_BYPASS     = 0x00,
	FIFO_MODE_CONTINUOUS = 0x06, // Continuous mode
};

// FIFO_STATUS2 (bit layout: [7]WTM_IA [6]OVR_IA [5]FULL_IA [4]CNT_BDR [3]OVR_LATCHED [2:1]0 [0]DIFF8)
enum FIFO_STATUS2_BIT : uint8_t {
	FIFO_OVR_LATCHED = Bit3,  // FIFO overrun latched (cleared on read)
	COUNTER_BDR_IA   = Bit4,  // Counter BDR reached
	FIFO_OVR_IA      = Bit6,  // FIFO overrun status (non-latched)
	DIFF_FIFO_8      = Bit0,  // bit 8 of FIFO diff count
};

// HAODR_CFG
enum HAODR_CFG_BIT : uint8_t {
	HAODR_SEL_MASK = 0x03, // HAODR_SEL [1:0]
	HAODR_MODE1    = 0x01, // HAODR_SEL=01 (2000 Hz ODR set)
	// HAODR_SEL=00 (1920/3840/7680 Hz ODR set) is the default (0x00)
};

// FIFO tag IDs (upper 5 bits of FIFO_DATA_OUT_TAG >> 3)
enum class FifoTag : uint8_t {
	GYRO_NC  = 0x01,
	ACCEL_NC = 0x02,    // low-g accelerometer
	TEMPERATURE = 0x03,
	TIMESTAMP = 0x04,
	ACCEL_HG = 0x1D,    // high-g accelerometer (LSM6DSV80X / LSM6DSV320X)
};

namespace FIFO
{
// FIFO word: 1-byte tag + 6-byte data = 7 bytes
static constexpr size_t WORD_SIZE = 7;
// Words batched per sample period: gyro + low-g, plus high-g on the 80X / 320X
static constexpr size_t MAX_WORDS_PER_PERIOD = 3;
// Max sample periods to drain per poll (avoid blocking scheduler)
static constexpr size_t MAX_DRAIN_SAMPLES = 32;
// Ceiling of the DIFF_FIFO word counter. The buffer itself is 1.5 KB, i.e. ~219 uncompressed words.
static constexpr size_t DEPTH = 512;
}

} // namespace ST_LSM6DSV
