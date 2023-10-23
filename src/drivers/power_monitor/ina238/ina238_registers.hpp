//
// Created by roman on 10/18/23.
//

#ifndef PX4_SRC_DRIVERS_POWER_MONITOR_INA238_INA238_REGISTERS_HPP_
#define PX4_SRC_DRIVERS_POWER_MONITOR_INA238_INA238_REGISTERS_HPP_


namespace ina238
{

enum class Register : uint8_t {
	CONFIG = 0x00, // Configuration Register
	ADCCONFIG = 0x01, // ADC Configuration Register
	SHUNT_CAL = 0x02, // Shunt Calibration Register
	VS_BUS = 0x05,
	CURRENT = 0x07,
	MANUFACTURER_ID = 0x3e,
	DEVICE_ID = 0x3f
};

enum CONFIG_BIT : uint16_t {
	RANGE_BIT = (1 << 4),
	ADC_RESET_BIT = (1 << 15)
};

enum ADCCONFIG_BIT : uint16_t {
	AVERAGES_1 = (0 << 0),
	AVERAGES_4 = (1 << 0),
	AVERAGES_16 = (2 << 0),
	AVERAGES_64 = (3 << 0),
	AVERAGES_128 = (4 << 0),
	AVERAGES_256 = (5 << 0),
	AVERAGES_512 = (6 << 0),
	AVERAGES_1024 = (7 << 0),

	VTCT_50US   = (0 << 3),
	VTCT_84US   = (1 << 3),
	VTCT_150US  = (2 << 3),
	VTCT_280US  = (3 << 3),
	VTCT_540US  = (4 << 3),
	VTCT_1052US = (5 << 3),
	VTCT_2074US = (6 << 3),
	VTCT_4170US = (7 << 3),


	VSHCT_MASK   = (7 << 6),
	VSHCT_50US   = (0 << 6),
	VSHCT_84US   = (1 << 6),
	VSHCT_150US  = (2 << 6),
	VSHCT_280US  = (3 << 6),
	VSHCT_540US  = (4 << 6),
	VSHCT_1052US = (5 << 6),
	VSHCT_2074US = (6 << 6),
	VSHCT_4170US = (7 << 6),


	VBUSCT_MASK   = (7 << 9),
	VBUSCT_50US   = (0 << 9),
	VBUSCT_84US   = (1 << 9),
	VBUSCT_150US  = (2 << 9),
	VBUSCT_280US  = (3 << 9),
	VBUSCT_540US  = (4 << 9),
	VBUSCT_1052US = (5 << 9),
	VBUSCT_2074US = (6 << 9),
	VBUSCT_4170US = (7 << 9),


	MODE_SHUTDOWN_TRIG       = (0 << 12),
	MODE_BUS_TRIG            = (1 << 12),
	MODE_SHUNT_TRIG          = (2 << 12),
	MODE_SHUNT_BUS_TRIG      = (3 << 12),
	MODE_TEMP_TRIG           = (4 << 12),
	MODE_TEMP_BUS_TRIG       = (5 << 12),
	MODE_TEMP_SHUNT_TRIG     = (6 << 12),
	MODE_TEMP_SHUNT_BUS_TRIG = (7 << 12),

	MODE_SHUTDOWN_CONT       = (8 << 12),
	MODE_BUS_CONT            = (9 << 12),
	MODE_SHUNT_CONT          = (10 << 12),
	MODE_SHUNT_BUS_CONT      = (11 << 12),
	MODE_TEMP_CONT           = (12 << 12),
	MODE_TEMP_BUS_CONT       = (13 << 12),
	MODE_TEMP_SHUNT_CONT     = (14 << 12),
	MODE_TEMP_SHUNT_BUS_CONT = (15 << 12)

};

}
#endif //PX4_SRC_DRIVERS_POWER_MONITOR_INA238_INA238_REGISTERS_HPP_
