//
// Created by salimterryli on 2020/1/8.
//

#include "ADS1115.h"
#include <cassert>

ADS1115::ADS1115(I2C_Interface *interface) : _interface(interface)
{

}

int ADS1115::init()
{
	uint8_t config[2] = {0x00};
	config[0] = CONFIG_HIGH_OS_START_SINGLE | CONFIG_HIGH_MUX_P0NG | CONFIG_HIGH_PGA_6144 | CONFIG_HIGH_MODE_SS;
	config[1] = CONFIG_LOW_DR_128SPS | CONFIG_LOW_COMP_MODE_TRADITIONAL | CONFIG_LOW_COMP_POL_RESET |
		    CONFIG_LOW_COMP_LAT_NONE | CONFIG_LOW_COMP_QU_DISABLE;
	_interface->writeReg(ADDRESSPOINTER_REG_CONFIG, config, 2);
	return 0;
}

int ADS1115::setChannel(ADS1115::ChannelSelection ch)
{
	uint8_t buf[1] = {0x00};
	uint8_t next_mux_reg = CONFIG_HIGH_MUX_P0NG;

	switch (ch) {
	case A0:
		next_mux_reg = CONFIG_HIGH_MUX_P0NG;
		break;

	case A1:
		next_mux_reg = CONFIG_HIGH_MUX_P1NG;
		break;

	case A2:
		next_mux_reg = CONFIG_HIGH_MUX_P2NG;
		break;

	case A3:
		next_mux_reg = CONFIG_HIGH_MUX_P3NG;
		break;

	default:
		assert(false);
		break;
	}

	buf[0] = CONFIG_HIGH_OS_START_SINGLE | next_mux_reg | CONFIG_HIGH_PGA_6144 | CONFIG_HIGH_MODE_SS;
	_interface->writeReg(ADDRESSPOINTER_REG_CONFIG, buf, 1);
	return 0;
}

bool ADS1115::isSampleReady()
{
	uint8_t buf[1] = {0x00};
	_interface->readReg(ADDRESSPOINTER_REG_CONFIG, buf, 1); // Pull config register
	return !(buf[0] & (uint8_t) 0x80);
}

ADS1115::ChannelSelection ADS1115::getMeasurement(int16_t *value)
{
	uint8_t buf[2] = {0x00};
	_interface->readReg(ADDRESSPOINTER_REG_CONFIG, buf, 1); // Pull config register

	ChannelSelection channel;

	switch ((buf[0] & (uint8_t) 0x70) >> 4) {
	case 0x04:
		channel = A0;
		break;

	case 0x05:
		channel = A1;
		break;

	case 0x06:
		channel = A2;
		break;

	case 0x07:
		channel = A3;
		break;

	default:
		return Invalid;
	}

	_interface->readReg(ADDRESSPOINTER_REG_CONVERSATION, buf, 2);
	uint16_t raw_adc_val = buf[0] * 256 + buf[1];

	if (raw_adc_val & (uint16_t) 0x8000) {     // Negetive value
		raw_adc_val = ~raw_adc_val + 1;     // 2's complement
		*value = -raw_adc_val;

	} else {
		*value = raw_adc_val;
	}

	return channel;
}

ADS1115::ChannelSelection ADS1115::cycleMeasure(int16_t *value)
{
	uint8_t buf[2] = {0x00};
	_interface->readReg(ADDRESSPOINTER_REG_CONFIG, buf, 1); // Pull config register

	ChannelSelection channel;
	uint8_t next_mux_reg = CONFIG_HIGH_MUX_P0NG;

	switch ((buf[0] & (uint8_t) 0x70) >> 4) {
	case 0x04:
		channel = A0;
		next_mux_reg = CONFIG_HIGH_MUX_P1NG;
		break;

	case 0x05:
		channel = A1;
		next_mux_reg = CONFIG_HIGH_MUX_P2NG;
		break;

	case 0x06:
		channel = A2;
		next_mux_reg = CONFIG_HIGH_MUX_P3NG;
		break;

	case 0x07:
		channel = A3;
		next_mux_reg = CONFIG_HIGH_MUX_P0NG;
		break;

	default:
		return Invalid;
	}

	_interface->readReg(ADDRESSPOINTER_REG_CONVERSATION, buf, 2);
	uint16_t raw_adc_val = buf[0] * 256 + buf[1];

	if (raw_adc_val & (uint16_t) 0x8000) {     // Negetive value
		raw_adc_val = ~raw_adc_val + 1;     // 2's complement
		*value = -raw_adc_val;

	} else {
		*value = raw_adc_val;
	}

	buf[0] = CONFIG_HIGH_OS_START_SINGLE | next_mux_reg | CONFIG_HIGH_PGA_6144 | CONFIG_HIGH_MODE_SS;
	_interface->writeReg(ADDRESSPOINTER_REG_CONFIG, buf, 1);

	return channel;
}
