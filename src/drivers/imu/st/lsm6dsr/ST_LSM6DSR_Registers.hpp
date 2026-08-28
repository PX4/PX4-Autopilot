/**
 * @file ST_LSM6DSR_Registers.hpp
 *
 * ST LSM6DSR registers.
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

namespace ST_LSM6DSR
{

static constexpr uint32_t SPI_SPEED = 8 * 1000 * 1000; // 8 MHz SPI data clock

static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHO_AM_I_ID = 0x6B;

static constexpr uint32_t GYRO_ODR  = 1666;
static constexpr uint32_t ACCEL_ODR = 1666;

static constexpr uint8_t ODR_1666HZ = 0x80;
static constexpr uint8_t FIFO_BDR_1666HZ = 0x08;

enum class Register : uint8_t {
	FIFO_CTRL1       = 0x07,
	FIFO_CTRL2       = 0x08,
	FIFO_CTRL3       = 0x09,
	FIFO_CTRL4       = 0x0A,

	INT1_CTRL        = 0x0D,

	WHO_AM_I         = 0x0F,

	CTRL1_XL         = 0x10,
	CTRL2_G          = 0x11,
	CTRL3_C          = 0x12,
	CTRL4_C          = 0x13,
	CTRL6_C          = 0x15,
	CTRL7_G          = 0x16,
	CTRL8_XL         = 0x17,
	CTRL9_XL         = 0x18,

	OUT_TEMP_L       = 0x20,
	OUT_TEMP_H       = 0x21,

	OUTX_L_G         = 0x22,

	OUTX_L_A         = 0x28,

	FIFO_STATUS1     = 0x3A,
	FIFO_STATUS2     = 0x3B,

	FIFO_DATA_OUT_TAG = 0x78,
	FIFO_DATA_OUT_X_L = 0x79,
};

enum CTRL1_XL_BIT : uint8_t {
	FS_XL_16G = Bit2,
	LPF2_XL_EN = Bit1,
};

enum CTRL2_G_BIT : uint8_t {
	FS_G_2000DPS = Bit3 | Bit2,
};

enum CTRL3_C_BIT : uint8_t {
	BDU       = Bit6,
	IF_INC    = Bit2,
	SW_RESET  = Bit0,
};

enum INT1_CTRL_BIT : uint8_t {
	INT1_FIFO_TH = Bit3,
};

enum CTRL8_XL_BIT : uint8_t {
	HP_SLOPE_XL_EN = Bit2,
};

enum FIFO_CTRL3_BIT : uint8_t {
	BDR_XL_1666HZ = FIFO_BDR_1666HZ,
	BDR_GY_1666HZ = FIFO_BDR_1666HZ << 4,
};

enum FIFO_CTRL4_BIT : uint8_t {
	FIFO_MODE_BYPASS     = 0x00,
	FIFO_MODE_CONTINUOUS = 0x06,
};

enum FIFO_STATUS2_BIT : uint8_t {
	DIFF_FIFO_8 = Bit0,
	DIFF_FIFO_9 = Bit1,
	FIFO_EMPTY_IA = Bit4,
	FIFO_FULL_IA = Bit5,
	FIFO_OVR_IA = Bit6,
	FIFO_WTM_IA = Bit7,
};

enum class FifoTag : uint8_t {
	GYRO_NC  = 0x01,
	ACCEL_NC = 0x02,
	TEMPERATURE = 0x03,
	TIMESTAMP = 0x04,
};

namespace FIFO
{
static constexpr size_t WORD_SIZE = 7;
static constexpr size_t MAX_DRAIN_SAMPLES = 32;
static constexpr size_t DEPTH = 512;
}

} // namespace ST_LSM6DSR
