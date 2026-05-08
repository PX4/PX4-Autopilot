/**
 * @file dfrobot_h2.h
 *
 * I2C driver for DFRobot Gravity electrochemical gas sensors
 */

#pragma once

#include <math.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_gas_concentration.h>

/* DFRobot gas sensor protocol commands */
#define DFROBOT_CMD_CHANGE_MODE		0x78
#define DFROBOT_CMD_GET_GAS_CONC	0x86
#define DFROBOT_CMD_GET_TEMP		0x87
#define DFROBOT_CMD_GET_ALL		0x88

/* Protocol constants */
#define DFROBOT_PROTOCOL_HEAD		0xFF
#define DFROBOT_PROTOCOL_ADDR		0x01
#define DFROBOT_PASSIVE_MODE		0x04
#define DFROBOT_PACKET_LEN		9

/* DFRobot vendor gas codes */
#define DFROBOT_GAS_TYPE_VENDOR_NH3	0x02
#define DFROBOT_GAS_TYPE_VENDOR_H2S	0x03
#define DFROBOT_GAS_TYPE_VENDOR_CO	0x04
#define DFROBOT_GAS_TYPE_VENDOR_O2	0x05
#define DFROBOT_GAS_TYPE_VENDOR_H2	0x06
#define DFROBOT_GAS_TYPE_VENDOR_O3	0x2A
#define DFROBOT_GAS_TYPE_VENDOR_SO2	0x2B
#define DFROBOT_GAS_TYPE_VENDOR_NO2	0x2C
#define DFROBOT_GAS_TYPE_VENDOR_HCL	0x2E
#define DFROBOT_GAS_TYPE_VENDOR_CL2	0x31
#define DFROBOT_GAS_TYPE_VENDOR_HF	0x33
#define DFROBOT_GAS_TYPE_VENDOR_PH3	0x45

/* NTC thermistor constants */
#define NTC_BETA			3380.13f
#define NTC_R25				10000.0f
#define NTC_T25_KELVIN			298.15f
#define NTC_VCC				3.0f
#define NTC_ADC_MAX			1024.0f

using namespace time_literals;

extern "C" __EXPORT int dfrobot_gas_main(int argc, char *argv[]);

class DFROBOTGas : public device::I2C, public ModuleParams, public I2CSPIDriver<DFROBOTGas>
{
public:
	DFROBOTGas(const I2CSPIDriverConfig &config);
	~DFROBOTGas() = default;

	static void print_usage();
	void RunImpl();

	int    init() override;
	int    probe() override;
	void   print_status() override;

private:
	int send_command(uint8_t cmd, uint8_t data0 = 0);
	int read_response(uint8_t *buf);
	uint8_t calc_checksum(const uint8_t *buf);
	uint8_t convert_vendor_gas_type(uint8_t vendor_gas_type) const;
	float compute_temperature(uint8_t temp_h, uint8_t temp_l);

	float _last_concentration{0.0f};
	float _last_temperature{0.0f};
	uint8_t _last_gas_type{0};
	uint32_t _measurement_count{0};

	uORB::PublicationMulti<sensor_gas_concentration_s> _sensor_gas_pub{ORB_ID(sensor_gas_concentration)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_DFGAS>) _param_sens_en_dfgas
	)
};
