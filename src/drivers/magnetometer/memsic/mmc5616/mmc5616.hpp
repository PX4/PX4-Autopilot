/**
 * @file mmc5616.hpp
 *
 * Driver for the MEMSIC MMC5616 connected via I2C.
 *
 */

#pragma once

#include "MEMSIC_mmc5616_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>

extern device::Device *MMC5616_I2C_interface(const I2CSPIDriverConfig &config);

using namespace MEMSIC_mmc5616;

class MMC5616 : public I2CSPIDriver<MMC5616>
{
public:
	MMC5616(device::Device *interface, const I2CSPIDriverConfig &config);
	~MMC5616() override;

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	void RunImpl();

	int init();
	void print_status() override;

private:
	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	typedef struct {
		float x;
		float y;
		float z;
		bool valid;
	} mmc5616_measurement_t;

	mmc5616_measurement_t _offset;

	void Reset();

	bool Configure();
	bool CheckIDs();

	mmc5616_measurement_t TakeSingleMeasurement();


	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	device::Device *_interface;
	PX4Magnetometer _px4_mag;

	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};
	perf_counter_t _timeout_perf{perf_alloc(PC_COUNT, MODULE_NAME": timeout")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": comms")};

	hrt_abstime _reset_timestamp{0};
	int _failure_count{0};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};

};
