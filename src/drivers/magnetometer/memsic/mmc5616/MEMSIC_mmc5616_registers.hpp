/**
 * @file MEMSIC_mmc5616_registers.hpp
 *
 * Memsic MMC5616 register definitions
 *
 */

#pragma once

#include <cstdint>

namespace MEMSIC_mmc5616
{

// From the datasheet
#define SENSOR_BIT_RESOLUTION 20
#define LSB_TO_MGAUSS_CONVERSION 0.0625f

// Half the range (+/- 30g). If 20 bits, then:
// Equivalent to 2^(20-1) = 524288
#define RAW_TO_SIGNED_OFFSET (1 << (SENSOR_BIT_RESOLUTION - 1))

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint32_t I2C_SPEED = 1000 * 1000; // 1MHz
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x30;

static constexpr uint8_t EXPECTED_PRODUCT_ID = 0b00010001;
static constexpr uint8_t EXPECTED_CHIP_ID = 0xD2; // revision I

enum class Register : uint8_t {
	XOUT0        = 0x00, // XOUT[19:12]
	XOUT1        = 0x01, // XOUT[11:4]
	YOUT0        = 0x02, // YOUT[19:12]
	YOUT1        = 0x03, // YOUT[11:4]
	ZOUT0        = 0x04, // ZOUT[19:12]
	ZOUT1        = 0x05, // ZOUT[11:4]
	XOUT2        = 0x06, // XOUT[3:0],FIFO_USAGE[3:0]
	YOUT2        = 0x07, // YOUT[3:0],2’B0,FIFO_FULL,FIFO_EMPTY
	ZOUT2        = 0x08, // ZOUT[3:0],4’B0
	TOUT         = 0x09, // TEMPERATURE OUTPUT
	TPH0         = 0x0A, // TPH_FORMAT, TPH[7:1]
	TPH1         = 0x0B, // 6’H00,TPH[9:8]
	TU           = 0x0C, // TU[1:0]
	FIFO_CONTROL = 0x0E, // FIFO CONTROL
	STATUS1      = 0x18, // DEVICE STATUS1
	STATUS0      = 0x19, // DEVICE STATUS0
	ODR          = 0x1A, // OUTPUT DATA RATE
	INTERNAL_CONTROL_0 = 0x1B, // CONTROL REGISTER 0
	INTERNAL_CONTROL_1 = 0x1C, // CONTROL REGISTER 1
	INTERNAL_CONTROL_2 = 0x1D, // CONTROL REGISTER 2
	ST_X_TH      = 0x1E, // X-AXIS SELFTEST THRESHOLD
	ST_Y_TH      = 0x1F, // Y-AXIS SELFTEST THRESHOLD
	ST_Z_TH      = 0x20, // Z-AXIS SELFTEST THRESHOLD
	R_CHIP_ID    = 0x21, // MASK PROGRAMMABLE. REV I -> 0XD2
	ANA_CONTROL  = 0x22, // ANALOG ADDR
	ST_X         = 0x27, // X-AXIS SELFTEST SET VALUE
	ST_Y         = 0x28, // Y-AXIS SELFTEST SET VALUE
	ST_Z         = 0x29, // Z-AXIS SELFTEST SET VALUE
	R_PRODUCT_ID = 0x39, // PRODUCT ID
};

enum STATUS1_BIT : uint8_t {
	MEAS_T_DONE = Bit7,
	MEAS_M_DONE = Bit6,
	SAT_SENSOR  = Bit5,
	OTP_READ_DONE = Bit4,
	ST_FAIL = Bit3,
	MDT_FLAG_INT = Bit2,
	MEAS_T_DONE_INT = Bit1,
	MEAS_M_DONE_INT = Bit0,
};

enum STATUS0_BIT : uint8_t {
	ACTIVITY_MODE1 = Bit7,
	ACTIVITY_MODE0 = Bit6,
	PROTOCOL_ERROR = Bit5,
	MDT_FLAG = Bit2,
	PENDING_INTERRUPT1 = Bit1,
	PENDING_INTERRUPT0 = Bit0,
};

enum CONTROL0_BIT : uint8_t {
	CMM_FREQ_EN = Bit7, // calculate measurement interval based on ODR
	AUTO_ST_EN = Bit6,  // Automatic SelfTest Enable
	AUTO_SR_EN = Bit5,  // Automatic Set/Reset Field Flip
	DO_RESET = Bit4,    // Manually Reset Field
	DO_SET = Bit3,      // Manually Set Field
	START_MDT = Bit2,   // Factory use only
	TAKE_MEAS_T = Bit1, // Measure Temperature
	TAKE_MEAS_M = Bit0, // Measure Magfield
};

enum CONTROL1_BIT : uint8_t {
	SW_RESET = Bit7,    // Soft power on reset, 20ms
	ST_ENM = Bit6,      // Self-test minus
	ST_ENP = Bit5,      // Self-test plus
	Z_INH = Bit4,       // Inhibit Z axis
	Y_INH = Bit3,       // Inhibit Y Axis
	X_INH = Bit2,       // Inhibit X Axis
	BW1 = Bit1,         // Length of decimation filter high bit
	BW0 = Bit0,         // Length of decimatoin filter low bit
};

enum CONTROL2_BIT : uint8_t {
	HPOWER = Bit7,      // Enable high-power mode
	INT_MEAS_DONE_EN = Bit6, // Factory Only
	INT_MDT_EN = Bit5,       // Factory Only
	CMM_EN = Bit4,      // Continuous Mode Enable
	EN_PRD_SET = Bit3,  // Enable periodic set
	PRD_SET2 = Bit2,    // How many samples between Mag field sets
	PRD_SET1 = Bit1,    // ditto
	PRD_Set0 = Bit0,    // ditto
};

} // namespace
