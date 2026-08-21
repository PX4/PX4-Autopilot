/**
 * @file dfrobot_h2.cpp
 *
 * I2C driver for DFRobot Gravity electrochemical gas sensors
 *
 * The sensor uses a custom 9-byte packet protocol over I2C:
 *   Send:  [0xFF, 0x01, CMD, D0, D1, D2, D3, D4, CHECKSUM]
 *   Recv:  [0xFF, CMD,  D0, D1, D2, D3, D4, D5, CHECKSUM]
 *
 * Checksum = (~(sum of bytes[1..7])) + 1
 *
 * The driver operates in passive mode and requests all data
 * (gas concentration and temperature) at 1 Hz via command 0x88.
 */

#include "dfrobot_h2.h"

DFROBOTGas::DFROBOTGas(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
{
}

uint8_t DFROBOTGas::calc_checksum(const uint8_t *buf)
{
	uint8_t sum = 0;

	for (int i = 1; i < 8; i++) {
		sum += buf[i];
	}

	return (~sum) + 1;
}

uint8_t DFROBOTGas::convert_vendor_gas_type(uint8_t vendor_gas_type) const
{
	switch (vendor_gas_type) {
	case DFROBOT_GAS_TYPE_VENDOR_O2:
		return sensor_gas_concentration_s::GAS_TYPE_O2;

	case DFROBOT_GAS_TYPE_VENDOR_CO:
		return sensor_gas_concentration_s::GAS_TYPE_CO;

	case DFROBOT_GAS_TYPE_VENDOR_H2S:
		return sensor_gas_concentration_s::GAS_TYPE_H2S;

	case DFROBOT_GAS_TYPE_VENDOR_NH3:
		return sensor_gas_concentration_s::GAS_TYPE_NH3;

	case DFROBOT_GAS_TYPE_VENDOR_H2:
		return sensor_gas_concentration_s::GAS_TYPE_H2;

	case DFROBOT_GAS_TYPE_VENDOR_O3:
		return sensor_gas_concentration_s::GAS_TYPE_O3;

	case DFROBOT_GAS_TYPE_VENDOR_SO2:
		return sensor_gas_concentration_s::GAS_TYPE_SO2;

	case DFROBOT_GAS_TYPE_VENDOR_NO2:
		return sensor_gas_concentration_s::GAS_TYPE_NO2;

	case DFROBOT_GAS_TYPE_VENDOR_HCL:
		return sensor_gas_concentration_s::GAS_TYPE_HCL;

	case DFROBOT_GAS_TYPE_VENDOR_CL2:
		return sensor_gas_concentration_s::GAS_TYPE_CL2;

	case DFROBOT_GAS_TYPE_VENDOR_HF:
		return sensor_gas_concentration_s::GAS_TYPE_HF;

	case DFROBOT_GAS_TYPE_VENDOR_PH3:
		return sensor_gas_concentration_s::GAS_TYPE_PH3;

	default:
		return 0;
	}
}

int DFROBOTGas::send_command(uint8_t cmd, uint8_t data0)
{
	uint8_t buf[DFROBOT_PACKET_LEN];
	buf[0] = DFROBOT_PROTOCOL_HEAD;
	buf[1] = DFROBOT_PROTOCOL_ADDR;
	buf[2] = cmd;
	buf[3] = data0;
	buf[4] = 0x00;
	buf[5] = 0x00;
	buf[6] = 0x00;
	buf[7] = 0x00;
	buf[8] = calc_checksum(buf);

	/* Write the 9-byte command packet to register 0x00 */
	uint8_t packet[DFROBOT_PACKET_LEN + 1];
	packet[0] = 0x00; /* register address */
	memcpy(&packet[1], buf, DFROBOT_PACKET_LEN);

	return transfer(packet, sizeof(packet), nullptr, 0);
}

int DFROBOTGas::read_response(uint8_t *buf)
{
	uint8_t reg = 0x00;
	int ret = transfer(&reg, 1, buf, DFROBOT_PACKET_LEN);

	if (ret != PX4_OK) {
		return ret;
	}

	/* Verify checksum */
	if (calc_checksum(buf) != buf[8]) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

float DFROBOTGas::compute_temperature(uint8_t temp_h, uint8_t temp_l)
{
	uint16_t temp_adc = ((uint16_t)temp_h << 8) | temp_l;

	if (temp_adc == 0) {
		return 0.0f;
	}

	float vpd3 = NTC_VCC * (float)temp_adc / NTC_ADC_MAX;
	float rth = vpd3 * NTC_R25 / (NTC_VCC - vpd3);

	if (rth <= 0.0f) {
		return 0.0f;
	}

	float temp = 1.0f / (1.0f / NTC_T25_KELVIN + logf(rth / NTC_R25) / NTC_BETA) - 273.15f;
	return temp;
}

int DFROBOTGas::probe()
{
	static constexpr hrt_abstime startup_settle_time_us = 900000;
	px4_usleep(startup_settle_time_us);

	int ret = send_command(DFROBOT_CMD_GET_GAS_CONC);

	if (ret != PX4_OK) {
		return ret;
	}

	px4_usleep(100000); /* sensor needs ~100ms to respond */

	uint8_t recv[DFROBOT_PACKET_LEN] = {};
	ret = read_response(recv);

	if (ret != PX4_OK) {
		return ret;
	}

	/* Verify response framing and ensure this is a supported DFRobot gas probe */
	if (recv[0] != DFROBOT_PROTOCOL_HEAD || recv[1] != DFROBOT_CMD_GET_GAS_CONC) {
		return PX4_ERROR;
	}

	if (convert_vendor_gas_type(recv[4]) == 0) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int DFROBOTGas::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	/* Switch sensor to passive (question-and-answer) mode */
	ret = send_command(DFROBOT_CMD_CHANGE_MODE, DFROBOT_PASSIVE_MODE);

	if (ret != PX4_OK) {
		PX4_ERR("failed to set passive mode");
		return ret;
	}

	px4_usleep(100000);

	/* Read and discard the mode-change response */
	uint8_t recv[DFROBOT_PACKET_LEN] = {};
	read_response(recv);

	_sensor_gas_pub.advertise();

	ScheduleOnInterval(1_s);

	return PX4_OK;
}

void DFROBOTGas::RunImpl()
{
	/* Read gas concentration first */
	int ret = send_command(DFROBOT_CMD_GET_GAS_CONC);

	if (ret != PX4_OK) {
		return;
	}

	px4_usleep(100000); /* sensor protocol examples use ~100ms response delay */

	uint8_t gas_recv[DFROBOT_PACKET_LEN] = {};
	ret = read_response(gas_recv);

	if (ret != PX4_OK || gas_recv[1] != DFROBOT_CMD_GET_GAS_CONC) {
		return;
	}

	/* Parse gas concentration:
	 *   gas_recv[2] = concentration high byte
	 *   gas_recv[3] = concentration low byte
	 *   gas_recv[4] = gas type
	 *   gas_recv[5] = decimal digits (0=1ppm, 1=0.1ppm, 2=0.01ppm)
	 */
	float concentration = (float)((uint16_t)(gas_recv[2] << 8) | gas_recv[3]);

	switch (gas_recv[5]) {
	case 1:
		concentration *= 0.1f;
		break;

	case 2:
		concentration *= 0.01f;
		break;

	default:
		break;
	}

	/* Read temperature with dedicated command (0x87) */
	float temperature = 0.0f;
	uint8_t temp_recv[DFROBOT_PACKET_LEN] = {};
	uint8_t temp_h = 0;
	uint8_t temp_l = 0;

	ret = send_command(DFROBOT_CMD_GET_TEMP);

	if (ret == PX4_OK) {
		px4_usleep(100000);
		ret = read_response(temp_recv);

		if (ret == PX4_OK && temp_recv[1] == DFROBOT_CMD_GET_TEMP) {
			temp_h = temp_recv[2];
			temp_l = temp_recv[3];
			temperature = compute_temperature(temp_h, temp_l);
		}
	}

	_last_concentration = concentration;
	_last_temperature = temperature;
	_last_gas_type = convert_vendor_gas_type(gas_recv[4]);
	_measurement_count++;

	sensor_gas_concentration_s msg{};
	msg.timestamp_sample = hrt_absolute_time();
	msg.device_id = get_device_id();
	msg.concentration_ppm = concentration;
	msg.temperature = temperature;
	msg.gas_type = _last_gas_type;
	msg.timestamp = hrt_absolute_time();

	_sensor_gas_pub.publish(msg);
}

void DFROBOTGas::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("Gas concentration: %.1f ppm", (double)_last_concentration);
	PX4_INFO("Sensor temperature: %.1f C", (double)_last_temperature);
	PX4_INFO("Gas type: %u", (unsigned)_last_gas_type);
	PX4_INFO("Measurements: %" PRIu32, _measurement_count);
}

void DFROBOTGas::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
I2C driver for DFRobot Gravity electrochemical gas sensors.

Reads gas concentration in ppm and sensor temperature at 1 Hz.

### Examples
Start the driver on external bus:
$ dfrobot_gas start -X

Start with explicit bus and address:
$ dfrobot_gas start -X -b 2 -a 0x74

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dfrobot_gas", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x74);
	PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG();
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int dfrobot_gas_main(int argc, char *argv[])
{
	using ThisDriver = DFROBOTGas;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.requested_bus = 2;
	cli.i2c_address = 0x74;
	cli.support_keep_running = true;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_GAS_DEVTYPE_DFROBOT_GAS);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
