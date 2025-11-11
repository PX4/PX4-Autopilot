
#include "tla2528.h"
//#include <px4_platform_common/getopt.h>
//#include <px4_platform_common/module.h>
//#include <drivers/drv_adc.h>
//#include <parameters/param.h>

#define READ 		0x10
#define WRITE 		0x08
#define SET_BIT 	0x18
#define CLEAR_BIT 	0x20

#define SYSTEM_STATUS	0x00
#define GENERAL_CFG	0x01
#define DATA_CFG	0x02
#define OSR_CFG		0x03
#define OPMODE_CFG	0x04
#define PIN_CFG		0x05
#define GPIO_CFG	0x07
#define GPO_DRIVE_CFG	0x09
#define GPO_VALUE	0x0B
#define GPI_VALUE	0x0D
#define SEQUENCE_CFG	0x10
#define CHANNEL_SEL	0x11
#define AUTO_SEQ_CH_SEL	0x12

TLA2528::init_config_t TLA2528::config_data{};


TLA2528::TLA2528(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	ModuleParams(nullptr),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
}

TLA2528::~TLA2528()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
}

int TLA2528::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	float ref_volt = 2.5f;
	param_get(param_find("ADC_TLA2528_REFV"), &ref_volt);

	_adc_report.device_id = this->get_device_id();
	_adc_report.v_ref = ref_volt;
	_adc_report.resolution = 4096;

	for (unsigned i = 0; i < 16; ++i) {
		_adc_report.channel_id[i] = -1;
	}

	ret = probe();

	if (ret != PX4_OK) {
		PX4_DEBUG("TLA2528::probing failed (%i)", ret);
		return ret;
	}

	ret = conf();

	if (ret != PX4_OK) {
		PX4_DEBUG("TLA2528::configuring failed (%i)", ret);
		return ret;
	}

	calibrate();
	ScheduleOnInterval(config_data.interval);

	return PX4_OK;
}

int TLA2528::calibrate()
{
	uint8_t send_data[3];
	uint8_t recv_data;

	send_data[0] = SET_BIT;
	send_data[1] = GENERAL_CFG;
	send_data[2] = 0x02;
	transfer(&send_data[0], 3, nullptr, 0);

	send_data[0] = READ;
	send_data[1] = GENERAL_CFG;
	transfer(&send_data[0], 2, &recv_data, 1);

	if (recv_data & (1u << 1)) {
		PX4_DEBUG("TLA2528::calibration failed");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int TLA2528::reset()
{
	uint8_t send_data[3];
	send_data[0] = SET_BIT;
	send_data[1] = GENERAL_CFG;
	send_data[2] = 0x01;
	transfer(&send_data[0], 3, nullptr, 0);

	return 0;
}

void TLA2528::adc_get()
{
	//Start sequential adc read
	uint8_t send_data[3] = {SET_BIT, SEQUENCE_CFG, 0x10};
	transfer(&send_data[0], 3, nullptr, 0);
	uint8_t recv_data[3];

	//Read adc data
	for (int i = 0; i < config_data.num_pins; i++) {
		transfer(nullptr, 0, &recv_data[0], 2);

		uint16_t tmp0 = ((uint16_t)recv_data[0]) << 8;
		uint16_t measurement = (tmp0 | recv_data[1]) >> 4;
		uint8_t ch_id = recv_data[1] & 0x0F;

		_adc_report.channel_id[i] = ch_id;
		_adc_report.raw_data[i] = measurement;
	}

	//Stop sequential adc read
	send_data[0] = CLEAR_BIT;
	send_data[1] = SEQUENCE_CFG;
	send_data[2] = 0x10;
	transfer(&send_data[0], 3, nullptr, 0);

	return;
}

int TLA2528::probe()
{
	// Set device in debug mode (should respond with 0xA5AX to all reads)
	uint8_t send_data[3] = {SET_BIT, DATA_CFG, 0x80};
	transfer(&send_data[0], 3, nullptr, 0);

	// Read
	uint8_t recv_data[2];
	send_data[0] = SET_BIT;
	send_data[0] = DATA_CFG;
	transfer(&send_data[0], 2, nullptr, 0);
	transfer(nullptr, 0, &recv_data[0], 2);

	// Turn debug mode off
	send_data[0] = CLEAR_BIT;
	send_data[1] = DATA_CFG;
	send_data[2] = 0x80;
	transfer(&send_data[0], 3, nullptr, 0);

	if (recv_data[0] == 165) {
		return PX4_OK;
	}

	return PX4_ERROR;
}


int TLA2528::conf()
{
	uint8_t send_data[3];

	// Configure pins as analog
	send_data[0] = SET_BIT;
	send_data[1] = PIN_CFG;
	send_data[2] = 0x00;
	int ret = transfer(&send_data[0], 3, nullptr, 0);

	//Append channel-id to measurements
	send_data[0] = SET_BIT;
	send_data[1] = DATA_CFG;
	send_data[2] = 0x10;
	ret |= transfer(&send_data[0], 3, nullptr, 0);

	//Activate pins
	send_data[0] = SET_BIT;
	send_data[1] = AUTO_SEQ_CH_SEL;
	send_data[2] = config_data.state;
	ret |= transfer(&send_data[0], 3, nullptr, 0);

	//Set seq-mode
	send_data[0] = SET_BIT;
	send_data[1] = SEQUENCE_CFG;
	send_data[2] = 0x01;
	ret |= transfer(&send_data[0], 3, nullptr, 0);

	return ret;
}

void TLA2528::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();	// nothing to do
}


void TLA2528::RunImpl()
{
	if (should_exit()) {
		PX4_INFO("stopping");
		return;	// stop and return immediately to avoid unexpected schedule from stopping procedure
	}

	perf_begin(_cycle_perf);
	adc_get();

	_adc_report.timestamp = hrt_absolute_time();
	_adc_report_pub.publish(_adc_report);

	for (unsigned i = 0; i < 16; ++i) {
		_adc_report.channel_id[i] = -1;
	}

	perf_end(_cycle_perf);
}
